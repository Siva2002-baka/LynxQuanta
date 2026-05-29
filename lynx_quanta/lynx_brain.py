#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

L1, L2 = 0.25, 0.25


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def smoothstep(u):
    u = clamp(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def ik_2link(x, z):
    """2-link planar IK. x=forward foot offset, z=vertical/depth leg length."""
    d = math.sqrt(x * x + z * z)
    d = clamp(d, 0.12, 0.48)
    cos_ki = (L1 * L1 + L2 * L2 - d * d) / (2.0 * L1 * L2)
    knee_inner = math.acos(clamp(cos_ki, -1.0, 1.0))
    knee = -(math.pi - knee_inner)
    alpha = math.atan2(x, z)
    cos_b = (L1 * L1 + d * d - L2 * L2) / (2.0 * L1 * d)
    beta = math.acos(clamp(cos_b, -1.0, 1.0))
    return alpha + beta, knee


class LynxBrain(Node):
    """
    V12 controller for M20 + arm with emergency stop and immediate wheel-direction updates.

    Goals:
      1) A/D spin uses BOTH legs and wheels.
      2) Spin stays close to in-place: cmd_vel linear.x/y = zero; yaw only.
      3) Leg lift and wheel yaw assist are synchronized.
      4) Static posture changes, especially SIT(4) -> STAND(1), are smoothed.
    """

    DT = 0.02
    WHEEL_RADIUS = 0.09

    # Static posture heights.
    H_STAND = 0.35
    H_STAIRS = 0.315
    H_CROUCH = 0.24

    # Normal walk/crab gait. Conservative because the arm shifts COM upward.
    GAIT_HZ = 1.05
    DUTY = 0.70
    STRIDE = 0.045
    LIFT = 0.040

    # V10 synchronized spin gait.
    # V9 was stable but too slow / too little lift. V10 adds lift and speed, but
    # keeps it far below the aggressive V8 values that caused jumping/falling.
    SPIN_H_STAND = 0.345          # lower bent-knee spin posture: more stable with arm load
    SPIN_HIPX_STANCE = 0.045      # narrower: avoids wide-leg jumping, still gives support polygon
    SPIN_STEP_X = 0.016           # slightly stronger tangential step so legs visibly contribute
    SPIN_STEP_Y = 0.006           # very small lateral shift; prevents side pop/jump
    SPIN_LIFT = 0.016             # visible lift, still below aggressive V8 lift
    SPIN_GAIT_HZ = 0.38           # faster than V10; still creep-like, one leg at a time
    SPIN_WHEEL_MIN = 0.04
    SPIN_WHEEL_MAX = 0.55
    SPIN_WHEEL_YAW_GAIN = 1.45
    SPIN_WHEEL_RAMP = 0.012       # smoother wheel torque ramp; prevents sudden body shove
    DEFAULT_SPIN_ANG_Z = 0.16

    # Posture interpolation. This fixes aggressive 4(SIT) -> 1(STAND) jumps.
    POSTURE_DUR_NORMAL = 1.80
    POSTURE_DUR_FROM_SIT = 3.20

    SIT_FL = [-0.436, 1.319, -2.809]
    SIT_FR = [+0.436, 1.319, -2.809]
    SIT_HL = [-0.436, 1.312, -2.792]
    SIT_HR = [+0.436, 1.312, -2.792]

    def __init__(self):
        super().__init__('lynx_brain')

        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        self.leg_pub = self.create_publisher(
            Float64MultiArray, '/leg_pose_controller/commands', 10)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Float64MultiArray, '/lynx/leg_cmd', self.leg_cmd_callback, 10)

        self.posture = 'STAND'
        self.walking = False
        self.walk_type = 'NONE'
        self.phase = 0.0
        self.spin_phase = 0.0
        self.spin_ang_z = self.DEFAULT_SPIN_ANG_Z

        self.current_wheels = [0.0, 0.0, 0.0, 0.0]
        self.target_wheels = [0.0, 0.0, 0.0, 0.0]

        stand_legs = self._target_pose_legs('STAND')
        self.current_legs = [list(x) for x in stand_legs]
        self.transition_active = False
        self.transition_t = 0.0
        self.transition_dur = self.POSTURE_DUR_NORMAL
        self.transition_start = [list(x) for x in stand_legs]
        self.transition_target = [list(x) for x in stand_legs]

        self.get_logger().info(
            'LYNX BRAIN V12 loaded: IMMEDIATE Q stop + immediate W/S direction switch + tuned stepping spin'
        )
        self.create_timer(self.DT, self.tick)

    # ------------------------------------------------------------------
    # Low-level publishers/state
    # ------------------------------------------------------------------
    def send_custom(self, fl, fr, hl, hr):
        legs = [list(fl), list(fr), list(hl), list(hr)]
        self.current_legs = [list(x) for x in legs]
        self.leg_pub.publish(Float64MultiArray(data=fl + fr + hl + hr))

    def publish_wheels(self, fl, fr, hl, hr, immediate=False):
        self.target_wheels = [float(fl), float(fr), float(hl), float(hr)]
        if immediate:
            self.current_wheels = list(self.target_wheels)
            self.wheel_pub.publish(Float64MultiArray(data=self.current_wheels))

    def _ramp_and_publish_wheels(self):
        out = []
        for cur, tgt in zip(self.current_wheels, self.target_wheels):
            delta = clamp(tgt - cur, -self.SPIN_WHEEL_RAMP, self.SPIN_WHEEL_RAMP)
            out.append(cur + delta)
        self.current_wheels = out
        self.wheel_pub.publish(Float64MultiArray(data=out))

    def stop_wheels(self, immediate=True):
        # Safety-critical: stop must not wait for ramping.
        self.publish_wheels(0.0, 0.0, 0.0, 0.0, immediate=immediate)

    # ------------------------------------------------------------------
    # Target postures and smooth transitions
    # ------------------------------------------------------------------
    def _standing_leg(self, hipx, height):
        hy, kn = ik_2link(0.0, height)
        return [hipx, hy, kn]

    def _target_pose_legs(self, posture):
        if posture == 'SIT':
            return [list(self.SIT_FL), list(self.SIT_FR), list(self.SIT_HL), list(self.SIT_HR)]

        h = {
            'STAND': self.H_STAND,
            'STAIRS': self.H_STAIRS,
            'CROUCH': self.H_CROUCH,
        }.get(posture, self.H_STAND)
        hy, kn = ik_2link(0.0, h)
        return [[0.0, hy, kn], [0.0, hy, kn], [0.0, hy, kn], [0.0, hy, kn]]

    def _start_posture_transition(self, posture):
        old_posture = self.posture
        self.posture = posture
        self.walking = False
        self.walk_type = 'NONE'
        self.stop_wheels(immediate=True)

        self.transition_active = True
        self.transition_t = 0.0
        self.transition_start = [list(x) for x in self.current_legs]
        self.transition_target = self._target_pose_legs(posture)
        # Slowest when rising out of sit; this prevents the violent jump from 4 -> 1.
        self.transition_dur = self.POSTURE_DUR_FROM_SIT if old_posture == 'SIT' or posture == 'STAND' else self.POSTURE_DUR_NORMAL
        self.get_logger().info(
            f'POSTURE TRANSITION: {old_posture} -> {posture}, dur={self.transition_dur:.2f}s'
        )

    def _tick_posture_transition(self):
        self.transition_t = min(self.transition_dur, self.transition_t + self.DT)
        u = smoothstep(self.transition_t / self.transition_dur)
        legs = []
        for p0, p1 in zip(self.transition_start, self.transition_target):
            legs.append([p0[i] + u * (p1[i] - p0[i]) for i in range(3)])
        self.send_custom(legs[0], legs[1], legs[2], legs[3])
        if self.transition_t >= self.transition_dur:
            self.transition_active = False
        return

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def leg_cmd_callback(self, msg):
        if not msg.data:
            return
        cmd = int(msg.data[0])

        if cmd == 0:
            # Emergency/normal stop: cancel motion immediately, zero wheels immediately.
            self.transition_active = False
            self.walking = False
            self.walk_type = 'NONE'
            self.stop_wheels(immediate=True)
            return

        if cmd in (1, 2, 3, 4):
            posture = {1: 'STAND', 2: 'STAIRS', 3: 'CROUCH', 4: 'SIT'}[cmd]
            self._start_posture_transition(posture)
            return

        self.transition_active = False
        self.stop_wheels(immediate=True)
        self.posture = 'STAND'
        self.walking = True
        self.walk_type = {5: 'FWD', 6: 'BWD', 7: 'C_L', 8: 'C_R', 9: 'S_CW', 10: 'S_CCW'}[cmd]
        if self.walk_type in ('S_CW', 'S_CCW'):
            self.spin_ang_z = self.DEFAULT_SPIN_ANG_Z
            self.spin_phase = 0.0

    def cmd_vel_callback(self, msg):
        lin = float(msg.linear.x)
        yaw = float(msg.angular.z)

        if abs(yaw) > 0.01 and abs(lin) < 0.02:
            new_type = 'S_CCW' if yaw > 0.0 else 'S_CW'
            if self.walk_type != new_type:
                self.spin_phase = 0.0
            self.transition_active = False
            self.posture = 'STAND'
            self.walking = True
            self.walk_type = new_type
            self.spin_ang_z = clamp(abs(yaw), 0.10, 0.80)
            self.get_logger().info(
                f'CMD_VEL SPIN: yaw={yaw:.3f} -> {self.walk_type}; synchronized leg lift + wheel yaw'
            )
            return

        if abs(lin) > 0.01 and abs(yaw) <= 0.01:
            self.transition_active = False
            self.walking = False
            self.walk_type = 'NONE'
            self.posture = 'STAND'
            wheel_rate = lin / self.WHEEL_RADIUS
            self.get_logger().info(f'CMD_VEL WHEEL DRIVE IMMEDIATE: linear.x={lin:.3f}, wheel_rate={wheel_rate:.3f}')
            # Safety-critical direction change: do not ramp W->S or S->W.
            self.publish_wheels(wheel_rate, wheel_rate, wheel_rate, wheel_rate, immediate=True)
            return

        if abs(lin) <= 0.01 and abs(yaw) <= 0.01:
            # Safety-critical stop: cancel gait and zero wheel controller immediately.
            self.transition_active = False
            self.walking = False
            self.walk_type = 'NONE'
            self.stop_wheels(immediate=True)

    # ------------------------------------------------------------------
    # Normal walk/crab foot trajectories
    # ------------------------------------------------------------------
    def _stance_swing_scalar(self, p, duty):
        st_end = duty * 2.0 * math.pi
        if p < st_end:
            prog = p / st_end
            s = 1.0 - 2.0 * prog
            lift = 0.0
        else:
            prog = (p - st_end) / (2.0 * math.pi - st_end)
            s = -1.0 + 2.0 * smoothstep(prog)
            lift = math.sin(math.pi * prog)
        return s, lift

    def _get_coords(self, p, d, mode='FWD'):
        s, lift = self._stance_swing_scalar(p, self.DUTY)
        val = d * self.STRIDE * s
        z = self.H_STAND - self.LIFT * lift
        if mode == 'FWD':
            return val, z, 0.0
        return 0.0, z, val

    # ------------------------------------------------------------------
    # V10 synchronized stepping spin
    # ------------------------------------------------------------------
    def _spin_leg_from_xyz(self, x, y, z):
        hipx = math.atan2(y, z)
        hy, kn = ik_2link(x, math.sqrt(z * z + y * y))
        return [hipx, hy, kn]

    def _spin_neutral_leg(self, base_y):
        return self._spin_leg_from_xyz(0.0, base_y, self.SPIN_H_STAND)

    def _spin_swing_leg(self, base_y, x_dir, y_dir, u):
        # Swing phase is smooth with visible but modest lift.
        s = smoothstep(u)
        disp = -1.0 + 2.0 * s
        lift = math.sin(math.pi * u)
        x = x_dir * self.SPIN_STEP_X * disp
        y = base_y + y_dir * self.SPIN_STEP_Y * disp
        z = self.SPIN_H_STAND - self.SPIN_LIFT * lift
        return self._spin_leg_from_xyz(x, y, z), lift

    def _spin_stepping_legs(self, spin_sign):
        spread = self.SPIN_HIPX_STANCE
        fl = self._spin_neutral_leg(-spread)
        fr = self._spin_neutral_leg(+spread)
        hl = self._spin_neutral_leg(-spread)
        hr = self._spin_neutral_leg(+spread)

        # Four-beat one-leg stepping sequence.
        # Only one leg moves; three legs remain planted.
        phase01 = self.spin_phase / (2.0 * math.pi)
        beat = int(phase01 * 4.0) % 4
        u = (phase01 * 4.0) - beat
        active_lift = 0.0

        if beat == 0:
            fl, active_lift = self._spin_swing_leg(-spread, -spin_sign, -spin_sign, u)
        elif beat == 1:
            hr, active_lift = self._spin_swing_leg(+spread, +spin_sign, +spin_sign, u)
        elif beat == 2:
            fr, active_lift = self._spin_swing_leg(+spread, +spin_sign, -spin_sign, u)
        else:
            hl, active_lift = self._spin_swing_leg(-spread, -spin_sign, +spin_sign, u)

        return fl, fr, hl, hr, active_lift

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def tick(self):
        if self.transition_active:
            self._tick_posture_transition()
            self._ramp_and_publish_wheels()
            return

        if not self.walking:
            # Hold/settle to current posture target smoothly if no explicit transition.
            legs = self._target_pose_legs(self.posture)
            self.send_custom(legs[0], legs[1], legs[2], legs[3])
            self._ramp_and_publish_wheels()
            return

        if self.walk_type in ('S_CW', 'S_CCW'):
            spin_sign = +1.0 if self.walk_type == 'S_CCW' else -1.0

            self.spin_phase = (self.spin_phase + 2.0 * math.pi * self.SPIN_GAIT_HZ * self.DT) % (2.0 * math.pi)
            fl, fr, hl, hr, active_lift = self._spin_stepping_legs(spin_sign)

            base_rate = clamp(abs(self.spin_ang_z) * self.SPIN_WHEEL_YAW_GAIN,
                              self.SPIN_WHEEL_MIN, self.SPIN_WHEEL_MAX)
            # Gate wheel yaw with the active leg lift.
            # When the foot is near the ground, wheels apply only a small assist.
            # When the foot is clearly unloaded, wheels rotate more. This matches
            # rotation timing to leg lift and reduces sideways shove/jump.
            lift_gate = smoothstep((active_lift - 0.18) / 0.72)
            sync_scale = 0.12 + 0.88 * lift_gate
            wheel_rate = base_rate * sync_scale

            if self.walk_type == 'S_CCW':
                wL, wR = -wheel_rate, +wheel_rate
            else:
                wL, wR = +wheel_rate, -wheel_rate

            self.publish_wheels(wL, wR, wL, wR)
            self.send_custom(fl, fr, hl, hr)
            self._ramp_and_publish_wheels()
            return

        self.phase = (self.phase + 2.0 * math.pi * self.GAIT_HZ * self.DT) % (2.0 * math.pi)
        pA = self.phase
        pB = (self.phase + math.pi) % (2.0 * math.pi)

        if self.walk_type in ('C_L', 'C_R'):
            d = -1.0 if self.walk_type == 'C_L' else +1.0
            cs = [
                self._get_coords(pA, d, 'CRAB'),
                self._get_coords(pB, d, 'CRAB'),
                self._get_coords(pB, d, 'CRAB'),
                self._get_coords(pA, d, 'CRAB'),
            ]
        else:
            d = +1.0 if self.walk_type == 'FWD' else -1.0
            cs = [
                self._get_coords(pA, d, 'FWD'),
                self._get_coords(pB, d, 'FWD'),
                self._get_coords(pB, d, 'FWD'),
                self._get_coords(pA, d, 'FWD'),
            ]

        final = []
        for x, z, y in cs:
            hipx = math.atan2(y, z)
            hy, kn = ik_2link(x, math.sqrt(z * z + y * y))
            final.append([hipx, hy, kn])

        self.send_custom(final[0], final[1], final[2], final[3])
        self._ramp_and_publish_wheels()


def main():
    rclpy.init()
    node = LynxBrain()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
