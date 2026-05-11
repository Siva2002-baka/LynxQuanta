#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


L1, L2 = 0.25, 0.25


def ik_2link(x, z):
    """2-link planar IK. x=forward foot offset, z=depth → (hipy, knee)."""
    d = math.sqrt(x**2 + z**2)
    d = max(0.12, min(d, 0.48))
    cos_ki = (L1**2 + L2**2 - d**2) / (2.0 * L1 * L2)
    knee_inner = math.acos(max(-1.0, min(1.0, cos_ki)))
    knee = -(math.pi - knee_inner)
    alpha = math.atan2(x, z)
    cos_b = (L1**2 + d**2 - L2**2) / (2.0 * L1 * d)
    beta = math.acos(max(-1.0, min(1.0, cos_b)))
    return alpha + beta, knee


class LynxBrain(Node):
    DT = 0.02
    WHEEL_RADIUS = 0.09
    H_STAND = 0.38

    # ── Walking gait parameters ─────────────────────────────────────────────
    GAIT_HZ = 1.3
    DUTY    = 0.65
    STRIDE  = 0.07
    LIFT    = 0.06

    # ── Spin (legs + wheels, wheels now in CORRECT direction) ───────────────
    # 2.0 rad/s wheel speed gives ~0.8 rad/s body rotation = ~46°/s,
    # full 360° in ~8 seconds.  Matched-direction with legs (same rotation
    # sense) so wheels and legs cooperate — no faceplant.
    SPIN_WHEEL_RATE = 2.0

    SIT_FL  = [-0.436, 1.319, -2.809]
    SIT_FR  = [+0.436, 1.319, -2.809]
    SIT_HL  = [-0.436, 1.312, -2.792]
    SIT_HR  = [+0.436, 1.312, -2.792]
    SIT_DUR = 1.5

    def __init__(self):
        super().__init__('lynx_brain')
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        self.leg_pub = self.create_publisher(
            Float64MultiArray, '/leg_pose_controller/commands', 10)
        self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/lynx/leg_cmd', self.leg_cmd_callback, 10)

        self.posture, self.walking, self.walk_type = "STAND", False, "NONE"
        self.phase, self.sit_t = 0.0, 0.0
        self.create_timer(self.DT, self.tick)

    # ── Command callbacks ───────────────────────────────────────────────────

    def leg_cmd_callback(self, msg):
        cmd = int(msg.data[0])
        if cmd == 0:
            self.walking = False
            self.wheel_pub.publish(Float64MultiArray(data=[0.0] * 4))
        elif cmd in (1, 2, 3, 4):
            self.walking = False
            self.posture = {1: "STAND", 2: "STAIRS",
                            3: "CROUCH", 4: "SIT"}[cmd]
            if cmd == 4:
                self.sit_t = 0.0
            self.wheel_pub.publish(Float64MultiArray(data=[0.0] * 4))
        else:
            self.posture, self.walking = "STAND", True
            self.walk_type = {5: "FWD", 6: "BWD",
                              7: "C_L", 8: "C_R",
                              9: "S_CW", 10: "S_CCW"}[cmd]

    def cmd_vel_callback(self, msg):
        if abs(msg.linear.x) > 0.01:
            self.walking = False
            self.wheel_pub.publish(Float64MultiArray(
                data=[float(msg.linear.x / self.WHEEL_RADIUS)] * 4))

    # ── Foot-coordinate generator ──────────────────────────────────────────

    def _get_coords(self, p, d, mode="FWD"):
        st_end = self.DUTY * 2.0 * math.pi
        if p < st_end:
            prog = p / st_end
            val = d * self.STRIDE * (1.0 - 2.0 * prog)
            return (val, self.H_STAND, 0.0) if mode == "FWD" \
                else (0.0, self.H_STAND, val)
        else:
            prog = (p - st_end) / (2.0 * math.pi - st_end)
            val = d * self.STRIDE * (-1.0 + 2.0 * prog)
            z = self.H_STAND - self.LIFT * math.sin(math.pi * prog)
            return (val, z, 0.0) if mode == "FWD" else (0.0, z, val)

    # ── Sit pose — smoothstep interpolation to verified joint targets ──────

    def _sit_pose_legs(self):
        """Returns [fl, fr, hl, hr], each [hipx, hipy, knee]."""
        a = min(1.0, self.sit_t / self.SIT_DUR)
        s = a * a * (3.0 - 2.0 * a)        # smoothstep

        # Starting pose: STAND (foot directly below hip)
        stand_hipy, stand_knee = ik_2link(0.0, self.H_STAND)
        stand = [0.0, stand_hipy, stand_knee]

        def lerp(p0, p1):
            return [p0[i] + s * (p1[i] - p0[i]) for i in range(3)]

        return [
            lerp(stand, self.SIT_FL),
            lerp(stand, self.SIT_FR),
            lerp(stand, self.SIT_HL),
            lerp(stand, self.SIT_HR),
        ]

    # ── Main 50 Hz tick ────────────────────────────────────────────────────

    def tick(self):

        # ── SIT (interpolated to verified joint targets) ───────────────────
        if self.posture == "SIT":
            self.sit_t = min(self.SIT_DUR, self.sit_t + self.DT)
            fl, fr, hl, hr = self._sit_pose_legs()
            self.send_custom(fl, fr, hl, hr)
            return

        # ── Static postures ────────────────────────────────────────────────
        if not self.walking:
            h = {"STAIRS": 0.32, "CROUCH": 0.22}.get(self.posture, self.H_STAND)
            hy, kn = ik_2link(0.0, h)
            self.leg_pub.publish(Float64MultiArray(data=[0.0, hy, kn] * 4))
            return

        # ── Advance gait phase ─────────────────────────────────────────────
        self.phase = (self.phase + 2.0 * math.pi * self.GAIT_HZ * self.DT) \
                     % (2.0 * math.pi)
        pA, pB = self.phase, (self.phase + math.pi) % (2.0 * math.pi)

        # ── SPIN (legs + wheels in MATCHED direction) ──────────────────────
        if "S_" in self.walk_type:
            if "CCW" in self.walk_type:
                ld, rd = -1.0, +1.0
            else:
                ld, rd =  1.0, -1.0

            cs = [
                self._get_coords(pA, ld),    # FL — left
                self._get_coords(pB, rd),    # FR — right
                self._get_coords(pB, ld),    # HL — left
                self._get_coords(pA, rd),    # HR — right
            ]

            wL = self.SPIN_WHEEL_RATE * ld
            wR = self.SPIN_WHEEL_RATE * rd
            self.wheel_pub.publish(Float64MultiArray(
                data=[wL, wR, wL, wR]))

        # ── CRAB walk ──────────────────────────────────────────────────────
        elif "C_" in self.walk_type:
            d = -1.0 if "L" in self.walk_type else 1.0
            cs = [
                self._get_coords(pA, d, "CRAB"),
                self._get_coords(pB, d, "CRAB"),
                self._get_coords(pB, d, "CRAB"),
                self._get_coords(pA, d, "CRAB"),
            ]

        # ── FORWARD / BACKWARD ─────────────────────────────────────────────
        else:
            d = 1.0 if "FWD" in self.walk_type else -1.0
            cs = [
                self._get_coords(pA, d),
                self._get_coords(pB, d),
                self._get_coords(pB, d),
                self._get_coords(pA, d),
            ]

        # ── Convert (x, z, y) → joint angles via IK ────────────────────────
        final = []
        for c in cs:
            x, z, y = c
            hipx = math.atan2(y, z)
            hy, kn = ik_2link(x, math.sqrt(z**2 + y**2))
            final.append([hipx, hy, kn])
        self.send_custom(final[0], final[1], final[2], final[3])

    def send_custom(self, fl, fr, hl, hr):
        self.leg_pub.publish(Float64MultiArray(data=fl + fr + hl + hr))


def main():
    rclpy.init()
    rclpy.spin(LynxBrain())
    rclpy.shutdown()


if __name__ == '__main__':
    main()