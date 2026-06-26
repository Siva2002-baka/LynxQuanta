#!/usr/bin/env python3

import sys
import termios
import tty
import datetime
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray, String, Float32


BANNER = """
╔════════════════════════════════════════════════════════════╗
║        LYNX M20 TELEOP V21 — NEW SIT POSE                 ║
╠════════════════════════════════════════════════════════════╣
║  DOG CONTROL                                              ║
║  W / S      → Wheel forward / backward at selected speed  ║
║  E / Z      → Increase / decrease speed                   ║
║  Q          → Emergency stop dog motion                   ║
║  A / D      → Pasted diagonal-pair spin left / right      ║
║  I / K      → Leg walk forward / backward                 ║
║  J / L      → Old crab / side-walk left / right           ║
║  H          → High obstacle READY stance                  ║
║  ↑ Arrow    → High obstacle climb / get up                ║
║  ↓ Arrow    → High obstacle descend / get down            ║
║  X          → Switch facing                               ║
║  Space      → Emergency stop                              ║
║  1/2/3/4    → Stand / Stairs / Crouch / Sit               ║
║                                                            ║
║  NOTE: Front/rear knee signs are calibrated in brain.      ║
║  NOTE: 4 now uses your new sit pose.                       ║
║                                                            ║
║  ARM CONTROL                                              ║
║  0          → Arm HOME                                    ║
║  9          → Arm READY                                   ║
║  8          → Arm STOW                                    ║
║  M          → Manual arm target                           ║
║  O          → Open gripper                                ║
║  C          → Close gripper                               ║
╚════════════════════════════════════════════════════════════╝
"""

WORKSPACE_TEXT = """
Manual arm target format after pressing M:
  x y z roll_deg pitch_deg yaw_deg  (or just  x y z  for zero orientation)

Normal safe range (arm_base_link frame):
  x:  0.05 to 0.45 m
  y: -0.25 to 0.25 m
  z:  0.10 to 0.40 m

Near-limit guidance (x, y, z share a 626 mm 3D budget — pushing one shrinks the others):
  x near max (~0.55 m)  ->  y: +-0.15 m,   z: 0.10 to 0.30 m
  y near max (~0.40 m)  ->  x: 0.05-0.40 m, z: 0.10 to 0.35 m
  z near max (~0.60 m)  ->  x: 0.05-0.35 m, y: +-0.20 m

If rejected, the controller prints the exact max for each axis given your other two values.

Joint limits: J1 +-154 | J2 0-195 | J3 -175-0 | J4 +-106 | J5 +-75 | J6 +-100 deg
"""

KEY_LABELS = {
    "w": "W  → WHEELS FORWARD",
    "s": "S  → WHEELS BACKWARD",
    "e": "E  → SPEED UP",
    "z": "Z  → SPEED DOWN",
    "q": "Q  → EMERGENCY STOP",
    "a": "A  → PASTED DIAGONAL SPIN LEFT",
    "d": "D  → PASTED DIAGONAL SPIN RIGHT",
    "i": "I  → WALK FORWARD",
    "k": "K  → WALK BACKWARD",
    "j": "J  → OLD CRAB LEFT",
    "l": "L  → OLD CRAB RIGHT",
    "h": "H  → HIGH OBSTACLE READY",
    "\x1b[A": "↑  → HIGH OBSTACLE CLIMB / GET UP",
    "\x1b[B": "↓  → HIGH OBSTACLE DESCEND / GET DOWN",
    "x": "X  → SWITCH FACING",
    " ": "SPACE → EMERGENCY STOP",
    "1": "1  → POSE: STAND",
    "2": "2  → POSE: STAIRS",
    "3": "3  → POSE: CROUCH",
    "4": "4  → POSE: SIT",
    "0": "0  → ARM: HOME",
    "9": "9  → ARM: READY",
    "8": "8  → ARM: STOW",
    "m": "M  → ARM: MANUAL TARGET",
    "o": "O  → GRIPPER OPEN",
    "c": "C  → GRIPPER CLOSE",
}

FLIP_MAP = {
    "w": "s",
    "s": "w",
    "a": "d",
    "d": "a",
    "i": "k",
    "k": "i",
    "j": "l",
    "l": "j",
}


def log(msg, facing):
    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    tag = "FORWARD " if facing else "REVERSED"
    print(f"[{ts}]  [facing: {tag}]  {msg}", flush=True)


def rpy_to_quat(roll_rad, pitch_rad, yaw_rad):
    cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
    cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
    cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class LynxTeleop(Node):
    SPEED_LEVELS = [0.18, 0.30, 0.42, 0.55]
    SPIN_LEVELS = [0.12, 0.20, 0.28, 0.36]

    def __init__(self):
        super().__init__("lynx_teleop")

        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.leg_cmd_pub = self.create_publisher(Float64MultiArray, "/lynx/leg_cmd", 10)
        self.arm_named_pub = self.create_publisher(String, "/arm/named_pose", 10)
        self.arm_target_pub = self.create_publisher(PoseStamped, "/arm/target_pose", 10)
        self.gripper_pub = self.create_publisher(Float32, "/arm/gripper_cmd", 10)

        self.wheel_direct_pub = self.create_publisher(
            Float64MultiArray,
            "/wheel_velocity_controller/commands",
            10,
        )

        self.settings = termios.tcgetattr(sys.stdin)

        self.facing_forward = True
        self.speed_idx = 0
        self.active_motion = "STOP"

        print(
            "[TELEOP] LYNX TELEOP V21 loaded: new sit pose + calibrated brain expected",
            flush=True,
        )
        self.print_speed()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)

        # Arrow keys arrive as escape sequences.
        if key == "\x1b":
            key += sys.stdin.read(2)
        else:
            key = key.lower()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def print_speed(self):
        print(
            f"[SPEED] level {self.speed_idx + 1}/{len(self.SPEED_LEVELS)} | "
            f"linear={self.SPEED_LEVELS[self.speed_idx]:.2f} m/s | "
            f"spin={self.SPIN_LEVELS[self.speed_idx]:.2f} rad/s",
            flush=True,
        )

    def republish_active_motion(self):
        if self.active_motion == "FWD":
            self.publish_cmd_vel(
                linear_x=-self.SPEED_LEVELS[self.speed_idx],
                angular_z=0.0,
                active="FWD",
            )

        elif self.active_motion == "BWD":
            self.publish_cmd_vel(
                linear_x=+self.SPEED_LEVELS[self.speed_idx],
                angular_z=0.0,
                active="BWD",
            )

        elif self.active_motion == "SPIN_L":
            self.publish_cmd_vel(
                linear_x=0.0,
                angular_z=+self.SPIN_LEVELS[self.speed_idx],
                active="SPIN_L",
            )

        elif self.active_motion == "SPIN_R":
            self.publish_cmd_vel(
                linear_x=0.0,
                angular_z=-self.SPIN_LEVELS[self.speed_idx],
                active="SPIN_R",
            )

        else:
            self.publish_cmd_vel(
                linear_x=0.0,
                angular_z=0.0,
                active="STOP",
            )

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0, active=None):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)

        for _ in range(3):
            self.vel_pub.publish(msg)
            time.sleep(0.005)

        if active is not None:
            self.active_motion = active

        print(
            f"[CMD_VEL] active={self.active_motion} | "
            f"linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}",
            flush=True,
        )

    def send_leg(self, code):
        msg = Float64MultiArray()
        msg.data = [float(code)]
        self.leg_cmd_pub.publish(msg)
        print(f"[LEG_CMD] code={code}", flush=True)

    def stop_before_leg_mode(self):
        """
        Stop W/S wheel drive before I/J/K/L/H/pose modes.
        """
        self.active_motion = "STOP"

        zero = Twist()
        wheel_stop = Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0])

        for _ in range(8):
            self.vel_pub.publish(zero)
            self.wheel_direct_pub.publish(wheel_stop)
            time.sleep(0.006)

    def emergency_stop(self):
        self.active_motion = "STOP"

        zero = Twist()
        leg_stop = Float64MultiArray(data=[0.0])
        wheel_stop = Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0])

        for _ in range(12):
            self.vel_pub.publish(zero)
            self.leg_cmd_pub.publish(leg_stop)
            self.wheel_direct_pub.publish(wheel_stop)
            time.sleep(0.01)

        print("[EMERGENCY STOP] /cmd_vel=0, /lynx/leg_cmd=0, wheels=[0,0,0,0]", flush=True)

    def send_arm_named(self, name):
        msg = String()
        msg.data = name
        self.arm_named_pub.publish(msg)
        print(f"\n[ARM] sent named pose: {name}\n", flush=True)

    def send_gripper(self, value):
        msg = Float32()
        msg.data = float(max(0.0, min(1.0, value)))
        self.gripper_pub.publish(msg)

        state = "OPEN" if msg.data > 0.5 else "CLOSED"
        print(f"\n[ARM] gripper: {state} ({msg.data:.2f})\n", flush=True)

    def send_arm_target(self, x, y, z, roll_deg, pitch_deg, yaw_deg):
        qx, qy, qz, qw = rpy_to_quat(
            math.radians(roll_deg),
            math.radians(pitch_deg),
            math.radians(yaw_deg),
        )

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "arm_base_link"

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.arm_target_pub.publish(msg)

        print(
            f"\n[ARM] target xyz=({x:.3f},{y:.3f},{z:.3f}) "
            f"rpy=({roll_deg:.1f},{pitch_deg:.1f},{yaw_deg:.1f})\n",
            flush=True,
        )

    def manual_arm_input(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        print("\nManual arm target input")
        print(WORKSPACE_TEXT)

        raw = input("Enter target: ").strip()

        try:
            values = [float(v) for v in raw.split()]
        except ValueError:
            print("[ARM] Invalid input. All values must be numbers.")
            return

        if len(values) == 6:
            x, y, z, roll_deg, pitch_deg, yaw_deg = values
        elif len(values) == 4:
            x, y, z, yaw_deg = values
            roll_deg = 0.0
            pitch_deg = 0.0
        else:
            print("[ARM] Invalid input. Use x y z roll pitch yaw, or x y z yaw.")
            return

        if not (0.05 <= x <= 0.62 and -0.40 <= y <= 0.40 and 0.05 <= z <= 0.65):
            print(
                f"[ARM] Target rejected: outside workspace. "
                f"Given x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )
            return

        self.send_arm_target(x, y, z, roll_deg, pitch_deg, yaw_deg)

    def run(self):
        print(BANNER)
        print(WORKSPACE_TEXT)

        try:
            while rclpy.ok():
                key = self.get_key()

                if key == "\x03":
                    break

                if key == "x":
                    self.facing_forward = not self.facing_forward
                    log(KEY_LABELS["x"], self.facing_forward)
                    continue

                effective_key = key

                if not self.facing_forward and key in FLIP_MAP:
                    effective_key = FLIP_MAP[key]

                log(
                    KEY_LABELS.get(effective_key, f"[{repr(key)}] → unbound key"),
                    self.facing_forward,
                )

                if effective_key == "e":
                    self.speed_idx = min(len(self.SPEED_LEVELS) - 1, self.speed_idx + 1)
                    self.print_speed()
                    self.republish_active_motion()

                elif effective_key == "z":
                    self.speed_idx = max(0, self.speed_idx - 1)
                    self.print_speed()
                    self.republish_active_motion()

                elif effective_key == "w":
                    self.publish_cmd_vel(
                        linear_x=-self.SPEED_LEVELS[self.speed_idx],
                        angular_z=0.0,
                        active="FWD",
                    )

                elif effective_key == "s":
                    self.publish_cmd_vel(
                        linear_x=+self.SPEED_LEVELS[self.speed_idx],
                        angular_z=0.0,
                        active="BWD",
                    )

                elif effective_key == "q":
                    self.emergency_stop()

                elif effective_key == "a":
                    self.publish_cmd_vel(
                        linear_x=0.0,
                        angular_z=+self.SPIN_LEVELS[self.speed_idx],
                        active="SPIN_L",
                    )

                elif effective_key == "d":
                    self.publish_cmd_vel(
                        linear_x=0.0,
                        angular_z=-self.SPIN_LEVELS[self.speed_idx],
                        active="SPIN_R",
                    )

                elif effective_key == "i":
                    self.stop_before_leg_mode()
                    self.send_leg(5)

                elif effective_key == "k":
                    self.stop_before_leg_mode()
                    self.send_leg(6)

                elif effective_key == "j":
                    self.stop_before_leg_mode()
                    self.send_leg(7)

                elif effective_key == "l":
                    self.stop_before_leg_mode()
                    self.send_leg(8)

                elif effective_key == "h":
                    self.stop_before_leg_mode()
                    self.send_leg(11)

                elif effective_key == "\x1b[A":
                    self.stop_before_leg_mode()
                    self.send_leg(12)

                elif effective_key == "\x1b[B":
                    self.stop_before_leg_mode()
                    self.send_leg(13)

                elif effective_key == " ":
                    self.emergency_stop()

                elif effective_key in ["1", "2", "3", "4"]:
                    self.stop_before_leg_mode()
                    self.send_leg(int(effective_key))

                elif effective_key == "0":
                    self.send_arm_named("home")

                elif effective_key == "9":
                    self.send_arm_named("ready")

                elif effective_key == "8":
                    self.send_arm_named("stow")

                elif effective_key == "m":
                    self.manual_arm_input()

                elif effective_key == "o":
                    self.send_gripper(1.0)

                elif effective_key == "c":
                    self.send_gripper(0.0)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main():
    rclpy.init()
    node = LynxTeleop()

    try:
        node.run()
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