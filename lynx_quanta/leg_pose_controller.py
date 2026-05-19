#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool

class LynxAdvancedLegController(Node):
    def __init__(self):
        super().__init__('lynx_leg_manager')

        # Publishers
        self.joint_pub = self.create_publisher(Float64MultiArray, '/leg_pose_controller/commands', 10)
        self.wheel_lock_pub = self.create_publisher(Bool, '/lynx/wheel_lock', 10)

        # Subscriber from Teleop
        self.sub = self.create_subscription(Float64MultiArray, '/leg_controller/commands', self.handle_teleop, 10)

        # State Tracking
        self.current_state = "STAND"
        self.get_logger().info("LYNX M20 Leg Manager Active: Stand, Stair, Crouch, Sit modes ready.")

    def handle_teleop(self, msg):
        # We check the incoming knee joint value (index 2) to determine the state
        knee_val = msg.data[2]
        
        lock_wheels = False
        state_name = "MOVING"

        if knee_val >= -1.5:
            state_name = "STAND"
        elif -2.0 <= knee_val < -1.5:
            state_name = "STAIR"
        elif -2.5 <= knee_val < -2.0:
            state_name = "CROUCH"
        else:
            state_name = "SIT (LOCKED)"
            lock_wheels = True

        # Broadcast wheel lock status
        lock_msg = Bool()
        lock_msg.data = lock_wheels
        self.wheel_lock_pub.publish(lock_msg)

        # Forward the joint positions to the hardware
        self.joint_pub.publish(msg)
        
        if state_name != self.current_state:
            self.get_logger().info(f"Transitioned to {state_name}")
            self.current_state = state_name

def main():
    rclpy.init()
    rclpy.spin(LynxAdvancedLegController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()