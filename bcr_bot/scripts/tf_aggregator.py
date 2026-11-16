#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy
)

from tf2_msgs.msg import TFMessage


class TFAggregator(Node):

    def __init__(self):
        super().__init__('tf_aggregator')

        # Robot names are configurable through a ROS parameter.
        self.declare_parameter('robot_names', 'robot1,robot2')

        robot_names = self.get_parameter(
            'robot_names'
        ).value

        self.robot_names = [
            name.strip()
            for name in robot_names.split(',')
            if name.strip()
        ]

        if not self.robot_names:
            raise ValueError('At least one robot name is required')

        # Dynamic TF: /tf
        self.dynamic_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Static TF: /tf_static
        self.static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Global publishers
        self.dynamic_publisher = self.create_publisher(
            TFMessage,
            '/tf',
            self.dynamic_qos
        )

        self.static_publisher = self.create_publisher(
            TFMessage,
            '/tf_static',
            self.static_qos
        )

        self.subscriptions = []

        # Subscribe to every robot's TF topics.
        for robot in self.robot_names:

            dynamic_topic = f'/{robot}/tf'
            static_topic = f'/{robot}/tf_static'

            dynamic_sub = self.create_subscription(
                TFMessage,
                dynamic_topic,
                self.dynamic_callback,
                self.dynamic_qos
            )

            static_sub = self.create_subscription(
                TFMessage,
                static_topic,
                self.static_callback,
                self.static_qos
            )

            self.subscriptions.extend([
                dynamic_sub,
                static_sub
            ])

            self.get_logger().info(
                f'Subscribed to {dynamic_topic} and {static_topic}'
            )

        self.get_logger().info(
            f'TF aggregation started for: {self.robot_names}'
        )

    def dynamic_callback(self, msg):
        # Forward dynamic transforms without modifying frame IDs.
        self.dynamic_publisher.publish(msg)

    def static_callback(self, msg):
        # Forward static transforms without modifying frame IDs.
        self.static_publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = TFAggregator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

