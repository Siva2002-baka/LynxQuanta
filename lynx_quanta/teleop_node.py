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
║        LYNX M20 TELEOP V12 — E/Z LIVE SPEED + SYNC STEP SPIN              ║
╠════════════════════════════════════════════════════════════╣
║  DOG CONTROL                                               ║
║  W / S      → Wheel forward / backward at selected speed  ║
║  E / Z      → Increase / decrease speed and republish active motion               ║
║  Q          → STOP dog motion                             ║
║  A / D      → STEPPING SPIN left/right using legs + wheels  ║
║  I / K      → Leg walk forward / backward                 ║
║  J / L      → Crab-walk left / right                      ║
║  X          → SWITCH FACING                               ║
║  Space      → Stop walking / spinning                     ║
║  1/2/3/4    → Stand / Stairs / Crouch / Sit               ║
║                                                            ║
║  ARM CONTROL                                               ║
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
  x y z roll_deg pitch_deg yaw_deg
Example:
  0.30 0.00 0.25 0 0 0
Allowed xyz guard:
  x: 0.15 to 0.45 m
  y: -0.20 to 0.20 m
  z: 0.10 to 0.40 m
"""

KEY_LABELS = {
    'w': 'W  → WHEELS FORWARD',
    's': 'S  → WHEELS BACKWARD',
    'e': 'E  → SPEED UP',
    'z': 'Z  → SPEED DOWN',
    'q': 'Q  → STOP DOG MOTION',
    'a': 'A  → SYNC STEPPING SPIN LEFT (LEGS + WHEELS)',
    'd': 'D  → SYNC STEPPING SPIN RIGHT (LEGS + WHEELS)',
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
    'w': 's', 's': 'w',
    'a': 'd', 'd': 'a',
    'i': 'k', 'k': 'i',
    'j': 'l', 'l': 'j',
}


def log(msg, facing):
    ts = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
    tag = 'FORWARD ' if facing else 'REVERSED'
    print(f'[{ts}]  [facing: {tag}]  {msg}', flush=True)


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
        super().__init__('lynx_teleop')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.leg_cmd_pub = self.create_publisher(Float64MultiArray, '/lynx/leg_cmd', 10)
        self.arm_named_pub = self.create_publisher(String, '/arm/named_pose', 10)
        self.arm_target_pub = self.create_publisher(PoseStamped, '/arm/target_pose', 10)
        self.gripper_pub = self.create_publisher(Float32, '/arm/gripper_cmd', 10)
        # Direct emergency wheel publisher. This bypasses lynx_brain delay on Q.
        self.wheel_direct_pub = self.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.facing_forward = True
        self.speed_idx = 0
        self.active_motion = 'STOP'
        print('[TELEOP] LYNX TELEOP V12 loaded: IMMEDIATE Q stop, E/Z live speed, C close, A/D tuned stepping spin', flush=True)
        self.print_speed()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1).lower()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def print_speed(self):
        print(
            f'[SPEED] level {self.speed_idx + 1}/{len(self.SPEED_LEVELS)} | '
            f'linear={self.SPEED_LEVELS[self.speed_idx]:.2f} m/s | '
            f'spin={self.SPIN_LEVELS[self.speed_idx]:.2f} rad/s',
            flush=True,
        )


    def republish_active_motion(self):
        """When E/Z changes speed, immediately update /cmd_vel for the current active motion."""
        if self.active_motion == 'FWD':
            self.publish_cmd_vel(linear_x=-self.SPEED_LEVELS[self.speed_idx], angular_z=0.0, active='FWD')
        elif self.active_motion == 'BWD':
            self.publish_cmd_vel(linear_x=+self.SPEED_LEVELS[self.speed_idx], angular_z=0.0, active='BWD')
        elif self.active_motion == 'SPIN_L':
            self.publish_cmd_vel(linear_x=0.0, angular_z=+self.SPIN_LEVELS[self.speed_idx], active='SPIN_L')
        elif self.active_motion == 'SPIN_R':
            self.publish_cmd_vel(linear_x=0.0, angular_z=-self.SPIN_LEVELS[self.speed_idx], active='SPIN_R')
        else:
            # Stopped: publish zero so ros2 topic echo /cmd_vel still shows an update.
            self.publish_cmd_vel(linear_x=0.0, angular_z=0.0, active='STOP')

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0, active=None):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        # Publish a small burst so direction changes W<->S are not missed by subscribers.
        for _ in range(3):
            self.vel_pub.publish(msg)
            time.sleep(0.005)
        if active is not None:
            self.active_motion = active
        print(
            f'[CMD_VEL] active={self.active_motion} | linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}',
            flush=True
        )

    def send_leg(self, code):
        msg = Float64MultiArray()
        msg.data = [float(code)]
        self.leg_cmd_pub.publish(msg)
        print(f'[LEG_CMD] code={code}', flush=True)

    def publish_wheel_zero_direct(self):
        self.wheel_direct_pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0]))

    def emergency_stop(self):
        """Immediate stop: send repeated zeros to cmd_vel, leg command, and wheel controller."""
        self.active_motion = 'STOP'
        zero = Twist()
        leg_stop = Float64MultiArray(data=[0.0])
        wheel_stop = Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0])

        # Burst zeros. This handles missed samples and bypasses any brain ramp delay.
        for _ in range(8):
            self.vel_pub.publish(zero)
            self.leg_cmd_pub.publish(leg_stop)
            self.wheel_direct_pub.publish(wheel_stop)
            time.sleep(0.01)

        print('[EMERGENCY STOP] /cmd_vel=0, /lynx/leg_cmd=0, wheels=[0,0,0,0]', flush=True)

    def send_arm_named(self, name):
        msg = String()
        msg.data = name
        self.arm_named_pub.publish(msg)
        print(f'\n[ARM] sent named pose: {name}\n', flush=True)

    def send_gripper(self, value):
        msg = Float32()
        msg.data = float(max(0.0, min(1.0, value)))
        self.gripper_pub.publish(msg)
        state = 'OPEN' if msg.data > 0.5 else 'CLOSED'
        print(f'\n[ARM] gripper: {state} ({msg.data:.2f})\n', flush=True)

    def send_arm_target(self, x, y, z, roll_deg, pitch_deg, yaw_deg):
        qx, qy, qz, qw = rpy_to_quat(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
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
        print(f'\n[ARM] target xyz=({x:.3f},{y:.3f},{z:.3f}) rpy=({roll_deg:.1f},{pitch_deg:.1f},{yaw_deg:.1f})\n', flush=True)

    def manual_arm_input(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        print('\nManual arm target input')
        print(WORKSPACE_TEXT)
        raw = input('Enter target: ').strip()
        try:
            values = [float(v) for v in raw.split()]
        except ValueError:
            print('[ARM] Invalid input. All values must be numbers.')
            return
        if len(values) == 6:
            x, y, z, roll_deg, pitch_deg, yaw_deg = values
        elif len(values) == 4:
            x, y, z, yaw_deg = values
            roll_deg = 0.0
            pitch_deg = 0.0
        else:
            print('[ARM] Invalid input. Use x y z roll pitch yaw, or x y z yaw.')
            return
        if not (0.15 <= x <= 0.45 and -0.20 <= y <= 0.20 and 0.10 <= z <= 0.40):
            print(f'[ARM] Target rejected: outside workspace. Given x={x:.3f}, y={y:.3f}, z={z:.3f}')
            return
        self.send_arm_target(x, y, z, roll_deg, pitch_deg, yaw_deg)

    def run(self):
        print(BANNER)
        print(WORKSPACE_TEXT)
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == '\x03':
                    break

                if key == 'x':
                    self.facing_forward = not self.facing_forward
                    log(KEY_LABELS['x'], self.facing_forward)
                    continue

                effective_key = key
                if not self.facing_forward and key in FLIP_MAP:
                    effective_key = FLIP_MAP[key]

                log(KEY_LABELS.get(effective_key, f'[{repr(key)}] → unbound key'), self.facing_forward)

                if effective_key == 'e':
                    self.speed_idx = min(len(self.SPEED_LEVELS) - 1, self.speed_idx + 1)
                    self.print_speed()
                    self.republish_active_motion()

                elif effective_key == 'z':
                    self.speed_idx = max(0, self.speed_idx - 1)
                    self.print_speed()
                    self.republish_active_motion()

                elif effective_key == 'w':
                    # Existing robot convention from your old teleop: W is negative linear.x.
                    self.publish_cmd_vel(linear_x=-self.SPEED_LEVELS[self.speed_idx], angular_z=0.0, active='FWD')

                elif effective_key == 's':
                    self.publish_cmd_vel(linear_x=+self.SPEED_LEVELS[self.speed_idx], angular_z=0.0, active='BWD')

                elif effective_key == 'q':
                    self.emergency_stop()

                elif effective_key == 'a':
                    # Visible /cmd_vel spin command. Brain V10 converts this to synchronized stepping legs + wheels.
                    self.publish_cmd_vel(0.0, +self.SPIN_LEVELS[self.speed_idx], active='SPIN_L')

                elif effective_key == 'd':
                    self.publish_cmd_vel(0.0, -self.SPIN_LEVELS[self.speed_idx], active='SPIN_R')

                elif effective_key == 'i':
                    self.publish_cmd_vel(0.0, 0.0, active='STOP')
                    self.send_leg(5)

                elif effective_key == 'k':
                    self.publish_cmd_vel(0.0, 0.0, active='STOP')
                    self.send_leg(6)

                elif effective_key == 'j':
                    self.publish_cmd_vel(0.0, 0.0, active='STOP')
                    self.send_leg(7)

                elif effective_key == 'l':
                    self.publish_cmd_vel(0.0, 0.0, active='STOP')
                    self.send_leg(8)

                elif effective_key == ' ':
                    self.emergency_stop()

                elif effective_key in ['1', '2', '3', '4']:
                    self.publish_cmd_vel(0.0, 0.0, active='STOP')
                    self.send_leg(int(effective_key))

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
