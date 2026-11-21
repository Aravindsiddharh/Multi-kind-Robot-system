#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy
)


class MultiRobotTFRelay(Node):

    def __init__(self):
        super().__init__('multi_robot_tf_relay')

        dynamic_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        static_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.tf_pub = self.create_publisher(
            TFMessage, '/tf', dynamic_qos
        )

        self.tf_static_pub = self.create_publisher(
            TFMessage, '/tf_static', static_qos
        )

        self.static_transforms = {}

        for robot in ['robot1', 'robot2']:

            self.create_subscription(
                TFMessage,
                f'/{robot}/tf',
                self.publish_dynamic,
                dynamic_qos
            )

            self.create_subscription(
                TFMessage,
                f'/{robot}/tf_static',
                self.publish_static,
                static_qos
            )

        self.get_logger().info(
            'Combining robot1 and robot2 TF for RViz'
        )

    def publish_dynamic(self, msg):
        self.tf_pub.publish(msg)

    def publish_static(self, msg):
        for transform in msg.transforms:
            self.static_transforms[
                transform.child_frame_id
            ] = transform

        combined = TFMessage()
        combined.transforms = list(
            self.static_transforms.values()
        )

        self.tf_static_pub.publish(combined)


def main():
    rclpy.init()
    node = MultiRobotTFRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()