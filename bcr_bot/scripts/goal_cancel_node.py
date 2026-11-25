#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.srv import CancelGoal


class CancelViaPublisher(Node):

    def __init__(self):
        super().__init__('cancel_via_publisher')

        self.sub = self.create_subscription(
            String,
            'goal_cancel',
            self.cancel_callback,
            10
        )

        # Relative path supports robot namespaces
        self.cli = self.create_client(
            CancelGoal,
            'navigate_to_pose/_action/cancel_goal'
        )

        self.get_logger().info(
            'CancelViaPublisher ready. '
            'Publish "cancel" to nav_cancel.'
        )

    def cancel_callback(self, msg):

        command = msg.data.strip().lower()

        if command != 'cancel':
            self.get_logger().warn(
                f'Unknown command: {command}'
            )
            return

        if not self.cli.service_is_ready():
            self.get_logger().error(
                'NavigateToPose cancel service '
                'is not available.'
            )
            return

        # Empty goal_info requests cancellation
        # of all eligible goals on this action.
        request = CancelGoal.Request()

        future = self.cli.call_async(request)
        future.add_done_callback(
            self.done_callback
        )

        self.get_logger().warn(
            'Navigation cancellation requested.'
        )

    def done_callback(self, future):

        try:
            response = future.result()

            if response.return_code == (
                CancelGoal.Response.ERROR_NONE
            ):
                count = len(response.goals_canceling)

                self.get_logger().info(
                    f'Cancellation accepted for '
                    f'{count} goal(s).'
                )

            else:
                self.get_logger().error(
                    'Cancellation failed. '
                    f'Return code: {response.return_code}'
                )

        except Exception as exc:
            self.get_logger().error(
                f'Cancel call failed: {exc}'
            )


def main(args=None):

    rclpy.init(args=args)
    node = CancelViaPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
