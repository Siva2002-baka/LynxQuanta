#!/usr/bin/env python3

import math
import time

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


def ik_2link_raw(x, z):
    """
    Raw 2-link planar IK.

    This returns a mathematical IK solution. The real M20 URDF needs
    front/rear sign calibration before publishing to the controller.
    """
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
    Lynx M20 brain controller.

    Important correction:
      The visual standing pose from Joint State Publisher shows that
      front and rear knees must bend with opposite hipy/knee signs.

      Front neutral:
        hipy ≈ -0.522, knee ≈ +1.157

      Rear neutral:
        hipy ≈ +0.522, knee ≈ -1.157

    V22 correction:
      Normal I/K walking now flips the rear-leg x command after calibration.
      This prevents front and rear pitch joints from fighting each other.
    """

    DT = 0.02
    WHEEL_RADIUS = 0.09

    # ------------------------------------------------------------------
    # Correct visual neutral pose from your screenshots
    # ------------------------------------------------------------------
    FRONT_HIPY_NEUTRAL = -0.522
    FRONT_KNEE_NEUTRAL = +1.157
    REAR_HIPY_NEUTRAL = +0.522
    REAR_KNEE_NEUTRAL = -1.157

    # ------------------------------------------------------------------
    # Static posture heights
    # ------------------------------------------------------------------
    H_STAND = 0.35
    H_STAIRS = 0.315
    H_CROUCH = 0.24

    # ------------------------------------------------------------------
    # Normal walking / old J-L crab gait
    # ------------------------------------------------------------------
    GAIT_HZ = 1.05
    DUTY = 0.70
    STRIDE = 0.045
    LIFT = 0.040

    # ------------------------------------------------------------------
    # PASTED A/D DIAGONAL-PAIR SPIN GAIT
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # Native M20-style A/D spin, corrected narrow HipX stance
    # ------------------------------------------------------------------
    # 20260624_114815 = CCW ground truth
    # 20260624_114847 = CW ground truth
    # Native robot spins by opening HipX wide, alternating diagonal leg
    # twist, and pulsing the wheels.  The previous value 0.038 rad was
    # too narrow; the display shows HipX roughly 25-42 deg during spin.
    SPIN_H_STAND = 0.335
    SPIN_HIPX_STANCE = 0.055

    SPIN_STEP_X = 0.032
    SPIN_STEP_Y = 0.012
    SPIN_LIFT = 0.026
    SPIN_GAIT_HZ = 0.70

    SPIN_WHEEL_MIN = 0.00
    SPIN_WHEEL_MAX = 0.90
    SPIN_WHEEL_YAW_GAIN = 2.10
    SPIN_WHEEL_RAMP = 0.060

    DEFAULT_SPIN_ANG_Z = 0.28

    # ------------------------------------------------------------------
    # High obstacle only. This does not affect I/J/K/L.
    # ------------------------------------------------------------------
    HIGH_READY_H = 0.330

    HIGH_H_STAND = 0.325
    HIGH_STEP_X = 0.075
    HIGH_LIFT = 0.165
    HIGH_GAIT_HZ = 0.20
    HIGH_WHEEL_RATE = 0.09

    # ------------------------------------------------------------------
    # Posture interpolation
    # ------------------------------------------------------------------
    POSTURE_DUR_NORMAL = 1.80
    POSTURE_DUR_FROM_SIT = 3.20

    # ------------------------------------------------------------------
    # SIT POSITION from your RViz screenshots
    # ------------------------------------------------------------------
    SIT_FL = [-0.436, -1.496,  2.792]
    SIT_FR = [ 0.436, -1.496,  2.792]
    SIT_HL = [-0.436,  1.496, -2.792]
    SIT_HR = [ 0.436,  1.496, -2.792]

    def __init__(self):
        super().__init__("lynx_brain")

        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            "/wheel_velocity_controller/commands",
            10,
        )
        self.leg_pub = self.create_publisher(
            Float64MultiArray,
            "/leg_pose_controller/commands",
            10,
        )

        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(Float64MultiArray, "/lynx/leg_cmd", self.leg_cmd_callback, 10)

        self.raw_stand_hy, self.raw_stand_kn = ik_2link_raw(0.0, self.H_STAND)

        self.posture = "STAND"
        self.walking = False
        self.walk_type = "NONE"

        self.phase = 0.0
        self.spin_phase = 0.0
        self.spin_ang_z = self.DEFAULT_SPIN_ANG_Z

        # Prevent old W/S /cmd_vel bursts from overriding I/J/K/L leg commands.
        self.ignore_cmd_vel_until = 0.0

        self.current_wheels = [0.0, 0.0, 0.0, 0.0]
        self.target_wheels = [0.0, 0.0, 0.0, 0.0]

        stand_legs = self._target_pose_legs("STAND")
        self.current_legs = [list(x) for x in stand_legs]

        self.transition_active = False
        self.transition_t = 0.0
        self.transition_dur = self.POSTURE_DUR_NORMAL
        self.transition_start = [list(x) for x in stand_legs]
        self.transition_target = [list(x) for x in stand_legs]

        self.get_logger().info(
            "LYNX BRAIN V22 loaded: fixed I/K rear x sign + calibrated knees + pasted A/D + old J/L"
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
        self.publish_wheels(0.0, 0.0, 0.0, 0.0, immediate=immediate)

    def hard_stop_wheels(self, bursts=8):
        self.target_wheels = [0.0, 0.0, 0.0, 0.0]
        self.current_wheels = [0.0, 0.0, 0.0, 0.0]

        msg = Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0])
        for _ in range(bursts):
            self.wheel_pub.publish(msg)

    def block_cmd_vel_briefly(self, seconds=0.40):
        self.ignore_cmd_vel_until = time.monotonic() + seconds

    # ------------------------------------------------------------------
    # Calibrated leg IK
    # ------------------------------------------------------------------
    def _clamp_leg_joints(self, leg, hipx, hipy, knee):
        """
        Clamp to URDF-ish limits. Keeps bad IK values from folding the robot.
        """
        if leg in ("fl", "hl"):
            hipx = clamp(hipx, -0.436, 0.611)
        else:
            hipx = clamp(hipx, -0.611, 0.436)

        if leg in ("fl", "fr"):
            hipy = clamp(hipy, -2.286, 2.583)
            knee = clamp(knee, -2.809, 2.792)
        else:
            hipy = clamp(hipy, -2.583, 2.286)
            knee = clamp(knee, -2.792, 2.809)

        return [hipx, hipy, knee]

    def _map_raw_ik_to_urdf(self, leg, hipy_raw, knee_raw):
        """
        Converts raw mathematical IK into the real URDF visual convention.
        """
        if leg in ("fl", "fr"):
            hipy_offset = self.FRONT_HIPY_NEUTRAL + self.raw_stand_hy
            knee_offset = self.FRONT_KNEE_NEUTRAL + self.raw_stand_kn
            hipy = -hipy_raw + hipy_offset
            knee = -knee_raw + knee_offset
        else:
            hipy_offset = self.REAR_HIPY_NEUTRAL - self.raw_stand_hy
            knee_offset = self.REAR_KNEE_NEUTRAL - self.raw_stand_kn
            hipy = hipy_raw + hipy_offset
            knee = knee_raw + knee_offset

        return hipy, knee

    def _leg_from_xyz(self, leg, x, y, z):
        """
        Build [hipx, hipy, knee] for a specific leg.

        x: forward/back foot offset
        y: lateral foot offset
        z: leg extension / body height
        """
        hipx = math.atan2(y, z)
        hipy_raw, knee_raw = ik_2link_raw(x, math.sqrt(z * z + y * y))
        hipy, knee = self._map_raw_ik_to_urdf(leg, hipy_raw, knee_raw)
        return self._clamp_leg_joints(leg, hipx, hipy, knee)

    # ------------------------------------------------------------------
    # Wheel convention helper
    # ------------------------------------------------------------------
    def _wheel_forward_pattern(self, rate):
        """
        Wheel command convention used by the pasted A/D spin and obstacle crawl.

        Your measured Gazebo convention:
          [-,+,-,+] = forward-ish crawl.
        """
        return [-rate, +rate, -rate, +rate]

    # ------------------------------------------------------------------
    # Target postures and smooth transitions
    # ------------------------------------------------------------------
    def _target_pose_legs(self, posture):
        if posture == "SIT":
            return [
                list(self.SIT_FL),
                list(self.SIT_FR),
                list(self.SIT_HL),
                list(self.SIT_HR),
            ]

        h = {
            "STAND": self.H_STAND,
            "STAIRS": self.H_STAIRS,
            "CROUCH": self.H_CROUCH,
            "HIGH_READY": self.HIGH_READY_H,
        }.get(posture, self.H_STAND)

        return [
            self._leg_from_xyz("fl", 0.0, 0.0, h),
            self._leg_from_xyz("fr", 0.0, 0.0, h),
            self._leg_from_xyz("hl", 0.0, 0.0, h),
            self._leg_from_xyz("hr", 0.0, 0.0, h),
        ]

    def _start_posture_transition(self, posture):
        old_posture = self.posture

        self.posture = posture
        self.walking = False
        self.walk_type = "NONE"
        self.hard_stop_wheels(bursts=8)

        self.transition_active = True
        self.transition_t = 0.0
        self.transition_start = [list(x) for x in self.current_legs]
        self.transition_target = self._target_pose_legs(posture)

        if old_posture == "SIT" or posture == "STAND":
            self.transition_dur = self.POSTURE_DUR_FROM_SIT
        else:
            self.transition_dur = self.POSTURE_DUR_NORMAL

        self.get_logger().info(
            f"POSTURE TRANSITION: {old_posture} -> {posture}, dur={self.transition_dur:.2f}s"
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

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def leg_cmd_callback(self, msg):
        if not msg.data:
            return

        cmd = int(msg.data[0])

        # Any leg command must immediately cancel W/S wheel-drive mode.
        self.block_cmd_vel_briefly(0.40)
        self.hard_stop_wheels(bursts=8)

        if cmd == 0:
            self.transition_active = False
            self.walking = False
            self.walk_type = "NONE"
            self.hard_stop_wheels(bursts=8)
            return

        if cmd in (1, 2, 3, 4):
            posture = {1: "STAND", 2: "STAIRS", 3: "CROUCH", 4: "SIT"}[cmd]
            self._start_posture_transition(posture)
            return

        if cmd == 11:
            # H = ready only, no movement.
            self._start_posture_transition("HIGH_READY")
            self.get_logger().info("HIGH OBSTACLE READY")
            return

        self.transition_active = False
        self.walking = True

        # Old working behavior:
        #   I/K = forward/back leg walk
        #   J/L = old crab/side-walk
        # 2 is only posture. It does not change I/J/K/L behavior.
        if cmd == 5:
            self.posture = "STAND"
            self.walk_type = "FWD"
            self.phase = 0.0

        elif cmd == 6:
            self.posture = "STAND"
            self.walk_type = "BWD"
            self.phase = 0.0

        elif cmd == 7:
            self.posture = "STAND"
            self.walk_type = "C_L"
            self.phase = 0.0

        elif cmd == 8:
            self.posture = "STAND"
            self.walk_type = "C_R"
            self.phase = 0.0

        elif cmd == 9:
            self.posture = "STAND"
            self.walk_type = "S_CW"
            self.spin_phase = 0.0
            self.spin_ang_z = self.DEFAULT_SPIN_ANG_Z

        elif cmd == 10:
            self.posture = "STAND"
            self.walk_type = "S_CCW"
            self.spin_phase = 0.0
            self.spin_ang_z = self.DEFAULT_SPIN_ANG_Z

        elif cmd == 12:
            self.posture = "HIGH_READY"
            self.walk_type = "HIGH_UP"
            self.phase = 0.0

        elif cmd == 13:
            self.posture = "HIGH_READY"
            self.walk_type = "HIGH_DOWN"
            self.phase = 0.0

        else:
            self.get_logger().warn(f"Unknown leg command: {cmd}")
            self.walking = False
            self.walk_type = "NONE"
            self.hard_stop_wheels(bursts=8)

    def cmd_vel_callback(self, msg):
        if time.monotonic() < self.ignore_cmd_vel_until:
            return

        lin = float(msg.linear.x)
        yaw = float(msg.angular.z)

        if abs(yaw) > 0.01 and abs(lin) < 0.02:
            new_type = "S_CCW" if yaw > 0.0 else "S_CW"

            if self.walk_type != new_type:
                self.spin_phase = 0.0

            self.transition_active = False
            self.posture = "STAND"
            self.walking = True
            self.walk_type = new_type
            self.spin_ang_z = clamp(abs(yaw), 0.10, 0.80)

            self.get_logger().info(
                f"CMD_VEL SPIN: yaw={yaw:.3f} -> {self.walk_type}; calibrated pasted diagonal-pair spin"
            )
            return

        if abs(lin) > 0.01 and abs(yaw) <= 0.01:
            self.transition_active = False
            self.walking = False
            self.walk_type = "NONE"
            self.posture = "STAND"

            wheel_rate = lin / self.WHEEL_RADIUS

            self.get_logger().info(
                f"CMD_VEL WHEEL DRIVE IMMEDIATE: linear.x={lin:.3f}, wheel_rate={wheel_rate:.3f}"
            )

            # W/S path unchanged because it already works.
            self.publish_wheels(
                wheel_rate,
                wheel_rate,
                wheel_rate,
                wheel_rate,
                immediate=True,
            )
            return

        if abs(lin) <= 0.01 and abs(yaw) <= 0.01:
            self.transition_active = False
            self.walking = False
            self.walk_type = "NONE"
            self.hard_stop_wheels(bursts=4)

    # ------------------------------------------------------------------
    # Normal walk / old crab trajectories
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

    def _get_coords(self, p, d, mode="FWD"):
        s, lift = self._stance_swing_scalar(p, self.DUTY)

        val = d * self.STRIDE * s
        z = self.H_STAND - self.LIFT * lift

        if mode == "FWD":
            return val, z, 0.0

        # Old working crab: same stride value, lateral axis.
        return 0.0, z, val

    # ------------------------------------------------------------------
    # PASTED A/D DIAGONAL-PAIR SPIN with calibrated leg IK
    # ------------------------------------------------------------------
    def _spin_neutral_leg(self, leg, base_y):
        return self._leg_from_xyz(leg, 0.0, base_y, self.SPIN_H_STAND)

    def _spin_swing_leg(self, leg, base_y, x_dir, y_dir, u):
        """
        Native-style spin foot path.

        The M20 display showed wide HipX angles during real spin:
          CCW sample: FL +37..42, FR -23, HL -23..-24, HR +31..38 deg
          CW sample:  FL +25..26, FR -33..-34, HL -33..-36, HR +24..26 deg

        In our URDF convention this is generated as a wide lateral base_y
        plus a small diagonal twist.  Lift is intentionally small because
        the real robot mostly drags/loads/unloads the wheels rather than
        doing a high stepping gait.
        """
        s = smoothstep(u)
        disp = -1.0 + 2.0 * s
        lift = math.sin(math.pi * u)

        x = x_dir * self.SPIN_STEP_X * disp
        y = base_y + y_dir * self.SPIN_STEP_Y * disp
        z = self.SPIN_H_STAND - self.SPIN_LIFT * lift

        return self._leg_from_xyz(leg, x, y, z), lift

    def _spin_stepping_legs(self, spin_sign):
        """
        Native two-beat diagonal spin.

        spin_sign > 0 = CCW, matching 20260624_114815.
        spin_sign < 0 = CW,  matching 20260624_114847.

        Beat A loads FR+HL and twists FL+HR.
        Beat B loads FL+HR and twists FR+HL.
        """
        spread = self.SPIN_HIPX_STANCE

        fl = self._spin_neutral_leg("fl", -spread)
        fr = self._spin_neutral_leg("fr", +spread)
        hl = self._spin_neutral_leg("hl", -spread)
        hr = self._spin_neutral_leg("hr", +spread)

        phase01 = self.spin_phase / (2.0 * math.pi)

        if phase01 < 0.5:
            u = phase01 / 0.5

            fl, lift_fl = self._spin_swing_leg(
                "fl", -spread, -spin_sign, -spin_sign, u
            )
            hr, lift_hr = self._spin_swing_leg(
                "hr", +spread, +spin_sign, +spin_sign, u
            )
            active_lift = max(lift_fl, lift_hr)
            beat = 0

        else:
            u = (phase01 - 0.5) / 0.5

            fr, lift_fr = self._spin_swing_leg(
                "fr", +spread, +spin_sign, -spin_sign, u
            )
            hl, lift_hl = self._spin_swing_leg(
                "hl", -spread, -spin_sign, +spin_sign, u
            )
            active_lift = max(lift_fr, lift_hl)
            beat = 1

        return fl, fr, hl, hr, active_lift, beat

    def _native_spin_wheels(self, spin_sign, beat, base_rate):
        """
        Wheel pulses copied from the real-robot spin tendency.

        CCW video 114815: FR/HL were the stronger positive wheels.
        CW  video 114847: FL/HR were the stronger negative wheels.

        Values below are normalized ratios multiplied by base_rate so the
        selected teleop spin level still controls aggressiveness.
        """
        if spin_sign > 0.0:
            # CCW: dominant FR + HL, small FL/HR assist.
            if beat == 0:
                return [0.12 * base_rate, 0.95 * base_rate, 1.00 * base_rate, 0.18 * base_rate]
            return [0.18 * base_rate, 0.25 * base_rate, 1.00 * base_rate, 0.35 * base_rate]

        # CW: mirror tendency, dominant FL + HR negative.
        if beat == 0:
            return [-0.95 * base_rate, 0.20 * base_rate, -0.25 * base_rate, -0.35 * base_rate]
        return [-0.18 * base_rate, 0.05 * base_rate, 0.18 * base_rate, -1.00 * base_rate]

    # ------------------------------------------------------------------
    # High obstacle helpers
    # ------------------------------------------------------------------
    def _high_step_swing_leg(self, leg, direction, u):
        s = smoothstep(u)
        x = direction * self.HIGH_STEP_X * (-1.0 + 2.0 * s)
        z = self.HIGH_H_STAND - self.HIGH_LIFT * math.sin(math.pi * u)
        return self._leg_from_xyz(leg, x, 0.0, z)

    def _four_beat_high_step_legs(self, direction):
        fl = self._leg_from_xyz("fl", 0.0, 0.0, self.HIGH_H_STAND)
        fr = self._leg_from_xyz("fr", 0.0, 0.0, self.HIGH_H_STAND)
        hl = self._leg_from_xyz("hl", 0.0, 0.0, self.HIGH_H_STAND)
        hr = self._leg_from_xyz("hr", 0.0, 0.0, self.HIGH_H_STAND)

        phase01 = self.phase / (2.0 * math.pi)
        beat = int(phase01 * 4.0) % 4
        u = (phase01 * 4.0) - beat

        if beat == 0:
            fl = self._high_step_swing_leg("fl", direction, u)
        elif beat == 1:
            fr = self._high_step_swing_leg("fr", direction, u)
        elif beat == 2:
            hl = self._high_step_swing_leg("hl", direction, u)
        else:
            hr = self._high_step_swing_leg("hr", direction, u)

        return fl, fr, hl, hr

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def tick(self):
        if self.transition_active:
            self._tick_posture_transition()
            self._ramp_and_publish_wheels()
            return

        if not self.walking:
            legs = self._target_pose_legs(self.posture)
            self.send_custom(legs[0], legs[1], legs[2], legs[3])
            self._ramp_and_publish_wheels()
            return

        # --------------------------------------------------------------
        # A/D spin: pasted diagonal-pair leg-dominant
        # --------------------------------------------------------------
        if self.walk_type in ("S_CW", "S_CCW"):
            spin_sign = +1.0 if self.walk_type == "S_CCW" else -1.0

            self.spin_phase = (
                self.spin_phase
                + 2.0 * math.pi * self.SPIN_GAIT_HZ * self.DT
            ) % (2.0 * math.pi)

            fl, fr, hl, hr, active_lift, beat = self._spin_stepping_legs(spin_sign)

            base_rate = clamp(
                abs(self.spin_ang_z) * self.SPIN_WHEEL_YAW_GAIN,
                self.SPIN_WHEEL_MIN,
                self.SPIN_WHEEL_MAX,
            )

            # Real robot keeps wheel assist even when the leg lift is small.
            # Gate only removes the harsh start/stop impulses.
            lift_gate = 0.35 + 0.65 * smoothstep(active_lift)
            w = self._native_spin_wheels(spin_sign, beat, base_rate * lift_gate)
            self.publish_wheels(w[0], w[1], w[2], w[3])

            self.send_custom(fl, fr, hl, hr)
            self._ramp_and_publish_wheels()
            return

        # --------------------------------------------------------------
        # Optional high obstacle
        # --------------------------------------------------------------
        if self.walk_type in ("HIGH_UP", "HIGH_DOWN"):
            direction = +1.0 if self.walk_type == "HIGH_UP" else -1.0

            self.phase = (
                self.phase
                + 2.0 * math.pi * self.HIGH_GAIT_HZ * self.DT
            ) % (2.0 * math.pi)

            fl, fr, hl, hr = self._four_beat_high_step_legs(direction)

            wheel_rate = self.HIGH_WHEEL_RATE * direction
            self.publish_wheels(
                -wheel_rate,
                +wheel_rate,
                -wheel_rate,
                +wheel_rate,
            )

            self.send_custom(fl, fr, hl, hr)
            self._ramp_and_publish_wheels()
            return

        # --------------------------------------------------------------
        # Normal walk / old J-L crab
        # --------------------------------------------------------------
        self.phase = (
            self.phase + 2.0 * math.pi * self.GAIT_HZ * self.DT
        ) % (2.0 * math.pi)

        pA = self.phase
        pB = (self.phase + math.pi) % (2.0 * math.pi)

        if self.walk_type in ("C_L", "C_R"):
            d = -1.0 if self.walk_type == "C_L" else +1.0

            # Old working J/L crab pattern.
            cs = [
                self._get_coords(pA, d, "CRAB"),
                self._get_coords(pB, d, "CRAB"),
                self._get_coords(pB, d, "CRAB"),
                self._get_coords(pA, d, "CRAB"),
            ]
        else:
            d = -1.0 if self.walk_type == "FWD" else +1.0

            cs = [
                self._get_coords(pA, d, "FWD"),
                self._get_coords(pB, d, "FWD"),
                self._get_coords(pB, d, "FWD"),
                self._get_coords(pA, d, "FWD"),
            ]

        # CRITICAL V22 FIX:
        # After front/rear knee calibration, rear legs need opposite x sign
        # for normal I/K walking. Without this, front and rear pitch axes fight.
        #
        # J/L crab is unaffected because x is 0 in CRAB mode.
        fl = self._leg_from_xyz("fl",  cs[0][0], cs[0][2], cs[0][1])
        fr = self._leg_from_xyz("fr",  cs[1][0], cs[1][2], cs[1][1])
        hl = self._leg_from_xyz("hl", -cs[2][0], cs[2][2], cs[2][1])
        hr = self._leg_from_xyz("hr", -cs[3][0], cs[3][2], cs[3][1])

        self.send_custom(fl, fr, hl, hr)
        self._ramp_and_publish_wheels()


def main():
    rclpy.init()
    node = LynxBrain()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()