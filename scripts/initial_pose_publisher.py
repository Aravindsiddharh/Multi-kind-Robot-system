#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self, robot_name, x, y, z, yaw):
        super().__init__('initial_pose_publisher_' + robot_name)

        # Build topic name
        topic = f'/{robot_name}/initialpose'
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, topic, 10)

        # Fill message
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)

        # Convert yaw to quaternion (z-rotation)
        qz = math.sin(float(yaw) / 2.0)
        qw = math.cos(float(yaw) / 2.0)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Publish once
        self.publisher_.publish(msg)
        self.get_logger().info(f"Initial pose published for {robot_name}: x={x}, y={y}, z={z}, yaw={yaw}")


def main():
    rclpy.init()

    if len(sys.argv) < 6:
        print("Usage: initial_pose_publisher.py <robot_name> <x> <y> <z> <yaw>")
        return

    robot_name = sys.argv[1]
    x = sys.argv[2]
    y = sys.argv[3]
    z = sys.argv[4]
    yaw = sys.argv[5]

    node = InitialPosePublisher(robot_name, x, y, z, yaw)

    # spin_once is enough since we only publish once
    rclpy.spin_once(node, timeout_sec=1.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
