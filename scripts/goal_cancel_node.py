#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.srv import CancelGoal

class CancelViaPublisher(Node):
    def __init__(self):
        super().__init__('cancel_via_publisher')

        # Subscriber (fake publisher interface)
        self.sub = self.create_subscription(String, 'nav_cancel', self.cancel_callback, 10)
        self.cli = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self.get_logger().info("Listening on /nav_cancel topic. Publish 'cancel' to stop goals.")

    def cancel_callback(self, msg):
        if msg.data.strip().lower() == "cancel":
            if not self.cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("Cancel service not available!")
                return
            req = CancelGoal.Request()
            future = self.cli.call_async(req)
            future.add_done_callback(self.done_callback)

    def done_callback(self, future):
        try:
            res = future.result()
            self.get_logger().warn(f"Cancel request sent. {len(res.goals_canceling)} goals being canceled.")
        except Exception as e:
            self.get_logger().error(f"Cancel call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CancelViaPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
