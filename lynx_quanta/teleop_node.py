#!/usr/bin/env python3
import sys, termios, tty, datetime
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

BANNER = """
╔══════════════════════════════════════════════╗
║         LYNX M20 Pro Teleop (FIXED)          ║
╠══════════════════════════════════════════════╣
║  W / S      → Wheels Forward / Back          ║
║  Q          → STOP EVERYTHING                ║
║  A / D      → Spin Left (CCW) / Right (CW)   ║
║  I / K      → Walk Forward / Backward        ║
║  J / L      → Crab-walk Left / Right         ║
║  X          → SWITCH FACING (reverse front)  ║
║  Space      → Stop walking / spinning        ║
║  1/2/3/4    → Stand/Stairs/Crouch/Sit        ║
╚══════════════════════════════════════════════╝
"""

KEY_LABELS = {
    'w': 'W  → WHEELS FORWARD',
    's': 'S  → WHEELS BACKWARD',
    'q': 'Q  → EMERGENCY STOP (ALL)',
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
    '4': '4  → POSE: SIT (BELLY)',
}

# Movement keys (the ones whose meaning flips when facing is reversed).
# Postures, Q, Space, X are NOT flipped.
DIRECTIONAL_KEYS = set('wsadikjl')

# When facing is reversed, every directional key acts as its mirror partner.
# (W↔S drive, I↔K walk, A↔D spin, J↔L crab)
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


class LynxTeleop(Node):
    def __init__(self):
        super().__init__('lynx_teleop')
        self.vel_pub     = self.create_publisher(Twist, '/cmd_vel', 10)
        self.leg_cmd_pub = self.create_publisher(Float64MultiArray, '/lynx/leg_cmd', 10)
        self.settings    = termios.tcgetattr(sys.stdin)

        # True  = natural front facing (default)
        # False = reversed (user treats the rear as the new "front")
        self.facing_forward = True

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1).lower()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def send_vel(self, linear_x):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)

    def send_leg(self, code):
        msg = Float64MultiArray()
        msg.data = [float(code)]
        self.leg_cmd_pub.publish(msg)

    def show_facing_banner(self):
        bar = '═' * 50
        state = 'FORWARD' if self.facing_forward else 'REVERSED'
        extra = ('All directional keys now act in REVERSE — '
                 'rear of robot is the new front.'
                 if not self.facing_forward
                 else 'Natural front facing restored.')
        print(f'\n  {bar}', flush=True)
        print(f'  ★  FACING → {state}', flush=True)
        print(f'     {extra}', flush=True)
        print(f'  {bar}\n', flush=True)

    def run(self):
        print(BANNER)
        print(f'  Starting facing: FORWARD\n', flush=True)
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == '\x03': break  # Ctrl-C

                # ── X: toggle facing (NOT a directional key, never flipped) ──
                if key == 'x':
                    self.facing_forward = not self.facing_forward
                    log(KEY_LABELS['x'], self.facing_forward)
                    self.show_facing_banner()
                    continue

                # ── Apply facing flip if directional key in reversed mode ───
                effective_key = key
                if not self.facing_forward and key in FLIP_MAP:
                    effective_key = FLIP_MAP[key]

                # ── Log the actual world-frame action ───────────────────────
                # When in REVERSED mode and user pressed W, the label shown
                # will be the effective action ("WHEELS BACKWARD"), so the
                # user always sees what's physically happening.
                label = KEY_LABELS.get(effective_key,
                                       f'[{repr(key)}] → unbound key')
                # Indicate when the pressed key was flipped, for clarity
                if effective_key != key:
                    label = f'{key.upper()} (flipped→{effective_key.upper()})  → ' \
                            + label.split('→', 1)[1].strip()
                log(label, self.facing_forward)

                # ── Dispatch based on effective_key ─────────────────────────
                if   effective_key == 'w': self.send_vel(-0.6)
                elif effective_key == 's': self.send_vel(0.6)
                elif effective_key == 'q':
                    self.send_vel(0.0)
                    self.send_leg(0)
                elif effective_key == 'a': self.send_leg(10)  # Spin Left
                elif effective_key == 'd': self.send_leg(9)   # Spin Right
                elif effective_key == 'i': self.send_leg(5)   # Walk Forward
                elif effective_key == 'k': self.send_leg(6)   # Walk Backward
                elif effective_key == 'j': self.send_leg(7)   # Crab Left
                elif effective_key == 'l': self.send_leg(8)   # Crab Right
                elif effective_key == ' ': self.send_leg(0)
                elif effective_key in '1234': self.send_leg(int(effective_key))

        except Exception as e:
            print(f"\nError: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main():
    rclpy.init()
    node = LynxTeleop()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()