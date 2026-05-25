#!/usr/bin/env python3

import sys
import termios
import tty
import datetime
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray, String, Float32


BANNER = """
╔════════════════════════════════════════════════════════════╗
║              LYNX M20 Pro Teleop + Piper Arm              ║
╠════════════════════════════════════════════════════════════╣
║  DOG CONTROL                                               ║
║  W / S      → Wheels Forward / Back                        ║
║  Q          → STOP dog motion                              ║
║  A / D      → Spin Left (CCW) / Right (CW)                 ║
║  I / K      → Walk Forward / Backward                      ║
║  J / L      → Crab-walk Left / Right                       ║
║  X          → SWITCH FACING                                ║
║  Space      → Stop walking / spinning                      ║
║  1/2/3/4    → Stand / Stairs / Crouch / Sit                ║
║                                                            ║
║  ARM CONTROL                                               ║
║  0          → Arm HOME                                     ║
║  9          → Arm READY                                    ║
║  8          → Arm STOW                                     ║
║  M          → Manual arm target: x y z roll pitch yaw      ║
║  O          → Open gripper                                 ║
║  C          → Close gripper                                ║
╚════════════════════════════════════════════════════════════╝
"""

WORKSPACE_TEXT = """
╔════════════════════════════════════════════════════════════╗
║              PRACTICAL PIPER TELEOP WORKSPACE             ║
╠════════════════════════════════════════════════════════════╣
║  Recommended working zone:                                ║
║    x:  0.20 to 0.38 m    forward from arm_base_link        ║
║    y: -0.12 to 0.12 m    left/right from arm_base_link     ║
║    z:  0.18 to 0.32 m    height from arm_base_link         ║
║                                                            ║
║  Hard input guard used by teleop:                          ║
║    x:  0.15 to 0.45 m                                     ║
║    y: -0.20 to 0.20 m                                     ║
║    z:  0.10 to 0.40 m                                     ║
║                                                            ║
║  Manual format after pressing M:                           ║
║    x y z roll_deg pitch_deg yaw_deg                        ║
║                                                            ║
║  Good first test values:                                   ║
║    Center:      0.30  0.00  0.25  0 0 0                    ║
║    Forward:     0.35  0.00  0.23  0 0 0                    ║
║    Back:        0.25  0.00  0.25  0 0 0                    ║
║    Left:        0.30  0.08  0.25  0 0 0                    ║
║    Right:       0.30 -0.08  0.25  0 0 180                  ║
║    Higher:      0.30  0.00  0.30  0 0 0                    ║
║    Lower:       0.30  0.00  0.20  0 0 0                    ║
║                                                            ║
║  Avoid:                                                    ║
║    x < 0.15 or x > 0.45                                    ║
║    |y| > 0.15 for normal testing                           ║
║    z < 0.10 or z > 0.35                                    ║
║    mistakes like y=-10.0; use -0.10 for 10 cm              ║
╚════════════════════════════════════════════════════════════╝
"""

KEY_LABELS = {
    'w': 'W  → WHEELS FORWARD',
    's': 'S  → WHEELS BACKWARD',
    'q': 'Q  → STOP DOG MOTION',
    'a': 'A  → SPIN LEFT (CCW)',
    'd': 'D  → SPIN RIGHT (CW)',
    'i': 'I  → WALK FORWARD',
    'k': 'K  → WALK BACKWARD',
    'j': 'J  → CRAB LEFT',
    'l': 'L  → CRAB RIGHT',
    'x': 'X  → SWITCH FACING',
    ' ': 'SPACE → STOP GAIT',
    '1': '1  → POSE: STAND',
    '2': '2  → POSE: STAIRS',
    '3': '3  → POSE: CROUCH',
    '4': '4  → POSE: SIT',
    '0': '0  → ARM: HOME',
    '9': '9  → ARM: READY',
    '8': '8  → ARM: STOW',
    'm': 'M  → ARM: MANUAL TARGET',
    'o': 'O  → GRIPPER OPEN',
    'c': 'C  → GRIPPER CLOSE',
}

FLIP_MAP = {
    'w': 's',
    's': 'w',
    'a': 'd',
    'd': 'a',
    'i': 'k',
    'k': 'i',
    'j': 'l',
    'l': 'j',
}


def log(msg, facing):
    ts = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
    tag = 'FORWARD ' if facing else 'REVERSED'
    print(f'[{ts}]  [facing: {tag}]  {msg}', flush=True)


def rpy_to_quat(roll_rad, pitch_rad, yaw_rad):
    """
    Convert roll, pitch, yaw to quaternion.
    Convention:
      roll  = rotation around X
      pitch = rotation around Y
      yaw   = rotation around Z
    """
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class LynxTeleop(Node):

    def __init__(self):
        super().__init__('lynx_teleop')

        # Dog publishers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.leg_cmd_pub = self.create_publisher(
            Float64MultiArray, '/lynx/leg_cmd', 10
        )

        # Arm publishers
        self.arm_named_pub = self.create_publisher(
            String, '/arm/named_pose', 10
        )
        self.arm_target_pub = self.create_publisher(
            PoseStamped, '/arm/target_pose', 10
        )
        self.gripper_pub = self.create_publisher(
            Float32, '/arm/gripper_cmd', 10
        )

        self.settings = termios.tcgetattr(sys.stdin)

        # True  = natural front facing
        # False = reversed front
        self.facing_forward = True

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1).lower()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    # ─────────────────────────────────────────────
    # Dog commands
    # ─────────────────────────────────────────────

    def send_vel(self, linear_x):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)

    def send_leg(self, code):
        msg = Float64MultiArray()
        msg.data = [float(code)]
        self.leg_cmd_pub.publish(msg)

    # ─────────────────────────────────────────────
    # Arm commands
    # ─────────────────────────────────────────────

    def send_arm_named(self, name):
        msg = String()
        msg.data = name
        self.arm_named_pub.publish(msg)
        print(f'\n[ARM] sent named pose: {name}\n', flush=True)

    def send_gripper(self, value):
        """
        value:
          0.0 = closed
          1.0 = open
        """
        msg = Float32()
        msg.data = float(max(0.0, min(1.0, value)))
        self.gripper_pub.publish(msg)

        state = 'OPEN' if msg.data > 0.5 else 'CLOSED'
        print(f'\n[ARM] gripper: {state} ({msg.data:.2f})\n', flush=True)

    def send_arm_target(self, x, y, z, roll_deg, pitch_deg, yaw_deg):
        """
        Sends full 6D target to /arm/target_pose.

        Input:
          x, y, z in metres, relative to arm_base_link
          roll_deg, pitch_deg, yaw_deg in degrees
        """
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        qx, qy, qz, qw = rpy_to_quat(roll, pitch, yaw)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'arm_base_link'

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.arm_target_pub.publish(msg)

        print(
            f'\n[ARM] sent manual target:\n'
            f'      xyz = ({x:.3f}, {y:.3f}, {z:.3f}) m\n'
            f'      rpy = ({roll_deg:.1f}, {pitch_deg:.1f}, {yaw_deg:.1f}) deg\n',
            flush=True
        )

    def manual_arm_input(self):
        """
        Manual arm input.

        New format:
          x y z roll_deg pitch_deg yaw_deg

        Backward-compatible old format:
          x y z yaw_deg
        """
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        print('\nManual arm target input')
        print('Format 6D: x y z roll_deg pitch_deg yaw_deg')
        print('Example:   0.30 0.00 0.25 0 0 0')
        print('Old format also allowed: x y z yaw_deg')
        print('Example:   0.30 0.00 0.25 0')
        print('Units: x/y/z in metres, RPY in degrees')
        print('Frame: arm_base_link')

        print(WORKSPACE_TEXT)

        raw = input('\nEnter target: ').strip()
        parts = raw.split()

        try:
            values = [float(v) for v in parts]
        except ValueError:
            print('\n[ARM] Invalid input. All values must be numbers.\n')
            return

        if len(values) == 6:
            x, y, z, roll_deg, pitch_deg, yaw_deg = values

        elif len(values) == 4:
            x, y, z, yaw_deg = values
            roll_deg = 0.0
            pitch_deg = 0.0

        else:
            print('\n[ARM] Invalid input.')
            print('Use either:')
            print('  x y z roll_deg pitch_deg yaw_deg')
            print('or:')
            print('  x y z yaw_deg\n')
            return

        # Teleop-side workspace guard.
        # This prevents accidental impossible inputs like y = -10.
        if not (0.15 <= x <= 0.45 and -0.20 <= y <= 0.20 and 0.10 <= z <= 0.40):
            print('\n[ARM] Target rejected: outside allowed teleop workspace.')
            print(f'      Given: x={x:.3f}, y={y:.3f}, z={z:.3f}')
            print('      Allowed:')
            print('        x: 0.15 to 0.45 m')
            print('        y: -0.20 to 0.20 m')
            print('        z: 0.10 to 0.40 m\n')
            return

        self.send_arm_target(x, y, z, roll_deg, pitch_deg, yaw_deg)

    # ─────────────────────────────────────────────
    # UI
    # ─────────────────────────────────────────────

    def show_facing_banner(self):
        bar = '═' * 50
        state = 'FORWARD' if self.facing_forward else 'REVERSED'

        if not self.facing_forward:
            extra = 'All directional keys now act in reverse.'
        else:
            extra = 'Natural front facing restored.'

        print(f'\n  {bar}', flush=True)
        print(f'  ★  FACING → {state}', flush=True)
        print(f'     {extra}', flush=True)
        print(f'  {bar}\n', flush=True)

    def run(self):
        print(BANNER)
        print(WORKSPACE_TEXT)
        print('  Starting facing: FORWARD\n', flush=True)

        try:
            while rclpy.ok():
                key = self.get_key()

                if key == '\x03':
                    break

                # Toggle facing
                if key == 'x':
                    self.facing_forward = not self.facing_forward
                    log(KEY_LABELS['x'], self.facing_forward)
                    self.show_facing_banner()
                    continue

                # Apply facing flip only to dog directional keys
                effective_key = key
                if not self.facing_forward and key in FLIP_MAP:
                    effective_key = FLIP_MAP[key]

                label = KEY_LABELS.get(
                    effective_key,
                    f'[{repr(key)}] → unbound key'
                )

                if effective_key != key:
                    label = (
                        f'{key.upper()} (flipped→{effective_key.upper()})  → '
                        + label.split('→', 1)[1].strip()
                    )

                log(label, self.facing_forward)

                # ─────────────────────────────────────────
                # Dog dispatch
                # ─────────────────────────────────────────

                if effective_key == 'w':
                    self.send_vel(-0.6)

                elif effective_key == 's':
                    self.send_vel(0.6)

                elif effective_key == 'q':
                    self.send_vel(0.0)
                    self.send_leg(0)

                elif effective_key == 'a':
                    self.send_leg(10)  # Spin Left / CCW

                elif effective_key == 'd':
                    self.send_leg(9)   # Spin Right / CW

                elif effective_key == 'i':
                    self.send_leg(5)   # Walk Forward

                elif effective_key == 'k':
                    self.send_leg(6)   # Walk Backward

                elif effective_key == 'j':
                    self.send_leg(7)   # Crab Left

                elif effective_key == 'l':
                    self.send_leg(8)   # Crab Right

                elif effective_key == ' ':
                    self.send_leg(0)

                elif effective_key in ['1', '2', '3', '4']:
                    self.send_leg(int(effective_key))

                # ─────────────────────────────────────────
                # Arm dispatch
                # ─────────────────────────────────────────

                elif effective_key == '0':
                    self.send_arm_named('home')

                elif effective_key == '9':
                    self.send_arm_named('ready')

                elif effective_key == '8':
                    self.send_arm_named('stow')

                elif effective_key == 'm':
                    self.manual_arm_input()

                elif effective_key == 'o':
                    self.send_gripper(1.0)

                elif effective_key == 'c':
                    self.send_gripper(0.0)

        except Exception as e:
            print(f'\nError: {e}')

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main():
    rclpy.init()
    node = LynxTeleop()

    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()