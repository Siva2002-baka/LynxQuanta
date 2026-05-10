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
    ' ': 'SPACE → STOP GAIT',
    '1': '1  → POSE: STAND',
    '2': '2  → POSE: STAIRS',
    '3': '3  → POSE: CROUCH',
    '4': '4  → POSE: SIT (BELLY)',
}

def log(msg):
    ts = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
    print(f'[{ts}]  {msg}', flush=True)

class LynxTeleop(Node):
    def __init__(self):
        super().__init__('lynx_teleop')
        self.vel_pub     = self.create_publisher(Twist, '/cmd_vel', 10)
        self.leg_cmd_pub = self.create_publisher(Float64MultiArray, '/lynx/leg_cmd', 10)
        self.settings    = termios.tcgetattr(sys.stdin)

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

    def run(self):
        print(BANNER)
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == '\x03': break # Ctrl-C

                label = KEY_LABELS.get(key, f'[{repr(key)}] → unbound key')
                log(label)

                if   key == 'w': self.send_vel(-0.6)
                elif key == 's': self.send_vel(0.6)
                elif key == 'q':
                    self.send_vel(0.0)
                    self.send_leg(0)
                
                # Logic Swaps applied here
                elif key == 'a': self.send_leg(10) # Spin Left
                elif key == 'd': self.send_leg(9)  # Spin Right
                elif key == 'i': self.send_leg(5)  # Walk Forward
                elif key == 'k': self.send_leg(6)  # Walk Backward
                elif key == 'j': self.send_leg(7)  # Crab Left
                elif key == 'l': self.send_leg(8)  # Crab Right
                
                elif key == ' ': self.send_leg(0)
                elif key in '1234': self.send_leg(int(key))
        
        except Exception as e:
            print(f"\nError: {e}")
        finally:
            # Always restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    node = LynxTeleop()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()