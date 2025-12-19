#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class ControllerSwitcher(Node):
    def __init__(self):
        super().__init__('controller_switcher')

        # QoS for the planner_selector publisher
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Publishes to /planner_selector (Nav2 PlannerSelector listens to this)
        self.controller_pub = self.create_publisher(String, 'controller_selector', qos_profile)

        # Subscribes to your custom trigger topic (e.g., /change_controller)
        self.subscriber = self.create_subscription(
            String,
            'change_controller',
            self.change_planner_callback,
            10
        )

        self.get_logger().info("ControllerSwitcher is ready. Publish to /change_controller to switch.")

    def change_planner_callback(self, msg: String):
        controller_msg = String()
        controller_msg.data = msg.data
        self.controller_pub.publish(controller_msg)
        self.get_logger().info(f"Switched controller to: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerSwitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
