#!/usr/bin/env python3



import sys
import math
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):

    def __init__(self, robot_name, x, y, z, yaw):
        super().__init__(
            'initial_pose_publisher_' + robot_name
        )

        self.robot_name = robot_name
        topic = f'/{robot_name}/initialpose'

        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            topic,
            10
        )

        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.yaw = float(yaw)

    def publish_initial_pose(self):

        msg = PoseWithCovarianceStamped()

        # Use the map frame
        msg.header.frame_id = 'map'

        # IMPORTANT: Use the ROS clock (simulation time)
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.z

        # Convert yaw to quaternion
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        # Set initial pose covariance: x, y and yaw uncertainty
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Initial pose sent for {self.robot_name}: '
            f'x={self.x}, y={self.y}, yaw={self.yaw}, '
            f'sim_time={msg.header.stamp.sec}.{msg.header.stamp.nanosec}'
        )


def main():

    rclpy.init()

    if len(sys.argv) < 6:
        print(
            'Usage: initial_pose_publisher.py '
            '<robot_name> <x> <y> <z> <yaw>'
        )
        rclpy.shutdown()
        return

    robot_name = sys.argv[1]
    x = sys.argv[2]
    y = sys.argv[3]
    z = sys.argv[4]
    yaw = sys.argv[5]

    node = InitialPosePublisher(robot_name, x, y, z, yaw)

    try:
        # Wait for simulation time and AMCL subscription.
        timeout = 10.0
        elapsed = 0.0

        while rclpy.ok() and elapsed < timeout:

            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed += 0.1

            clock_ready = node.get_clock().now().nanoseconds > 0
            subscriber_ready = node.publisher_.get_subscription_count() > 0

            if clock_ready and subscriber_ready:
                break

        if not clock_ready:
            node.get_logger().error(
                'Simulation clock is not active. Initial pose not published.'
            )

        elif not subscriber_ready:
            node.get_logger().error(
                'No subscriber found for initial pose. Initial pose not published.'
            )

        else:
            # Publish three times to reduce the chance of a missed message.
            for _ in range(3):
                node.publish_initial_pose()
                rclpy.spin_once(node, timeout_sec=0.2)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
