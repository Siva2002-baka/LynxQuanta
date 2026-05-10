#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class M20SkidSteerController(Node):
    def __init__(self):
        super().__init__("m20_skid_steer_controller")

        self.wheel_separation = 0.453152
        self.wheel_radius = 0.09

        self.max_wheel_speed = 30.0  # rad/s safety limit

        self.sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )

        self.pub = self.create_publisher(
            Float64MultiArray,
            "/wheel_velocity_controller/commands",
            10
        )
        self.locked = False
        self.lock_sub = self.create_subscription(Bool, '/lynx/wheel_lock', self.lock_callback, 10)

        def lock_callback(self, msg):
            self.locked = msg.data

        # Update your cmd_vel_callback
        def cmd_vel_callback(self, msg):
            if self.locked:
                return # Do nothing if sitting

        self.get_logger().info("M20 skid-steer controller started")

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        wz = msg.angular.z

        v_left = v - (wz * self.wheel_separation / 2.0)
        v_right = v + (wz * self.wheel_separation / 2.0)

        wl = v_left / self.wheel_radius
        wr = v_right / self.wheel_radius

        wl = self.clamp(wl, -self.max_wheel_speed, self.max_wheel_speed)
        wr = self.clamp(wr, -self.max_wheel_speed, self.max_wheel_speed)

        cmd = Float64MultiArray()

        # Joint order must match YAML:
        # fl_wheel_joint, fr_wheel_joint, hl_wheel_joint, hr_wheel_joint
        cmd.data = [
            wl,   # fl
            wr,   # fr
            wl,   # hl
            wr,   # hr
        ]

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = M20SkidSteerController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()