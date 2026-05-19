#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmJointPublisher(Node):
    def __init__(self):
        super().__init__('arm_joint_publisher')

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_joint_values)
        self.published = False

    def publish_joint_values(self):
        if self.published:
            return

        msg = JointTrajectory()

        msg.joint_names = [
            'arm_joint1',
            'arm_joint2',
            'arm_joint3',
            'arm_joint4',
            'arm_joint5',
            'arm_joint6',
            'arm_joint7',
            'arm_joint8'
        ]

        point = JointTrajectoryPoint()

        # Joint positions in radians
        point.positions = [
            0.2,
            0.2,
            0.2,
            0.2,
            0.2,
            0.2,
            0.2,
            0.2
        ]

        # Reach this position in 2 seconds
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0

        msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info('Published arm joint trajectory')

        self.published = True


def main(args=None):
    rclpy.init(args=args)

    node = ArmJointPublisher()
    rclpy.spin_once(node, timeout_sec=2.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()