
#!/usr/bin/env python3

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class PauseResumeNav(Node):

    def __init__(self):
        super().__init__('pause_resume_nav')

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )

        self.command_sub = self.create_subscription(
            String,
            'nav_control',
            self.command_callback,
            10
        )

        self.paused = False
        self.pending_pose = None
        self.active_handle = None
        self.sending = False
        self.cancel_requested = False

        self.get_logger().info(
            'PauseResumeNav ready (ROS 2 Jazzy)'
        )

    def goal_callback(self, msg):

        self.pending_pose = deepcopy(msg)

        self.get_logger().info(
            'Navigation goal received.'
        )

        if self.paused:
            self.get_logger().info(
                'Paused: goal stored for resume.'
            )
            return

        self.process_pending_goal()

    def command_callback(self, msg):

        command = msg.data.strip().lower()

        if command == 'pause':

            if self.paused:
                return

            self.paused = True
            self.get_logger().warn(
                'Pause requested.'
            )

            self.request_cancel()

        elif command == 'resume':

            if not self.paused:
                self.get_logger().info(
                    'Navigation is not paused.'
                )
                return

            self.paused = False
            self.get_logger().info(
                'Resume requested.'
            )

            self.process_pending_goal()

        else:
            self.get_logger().warn(
                f'Unknown command: {command}'
            )

    def process_pending_goal(self):

        if self.paused or self.pending_pose is None:
            return

        if self.sending:
            return

        if self.active_handle is not None:
            self.request_cancel()
            return

        if not self.nav_client.server_is_ready():
            self.get_logger().warn(
                'NavigateToPose server not ready.'
            )
            return

        pose = self.pending_pose
        self.pending_pose = None

        goal = NavigateToPose.Goal()
        goal.pose = deepcopy(pose)

        # Refresh timestamp using ROS/simulation time.
        goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        self.sending = True

        future = self.nav_client.send_goal_async(goal)

        future.add_done_callback(
            lambda f, p=pose:
            self.goal_response_callback(f, p)
        )

    def goal_response_callback(self, future, pose):

        self.sending = False

        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'Goal submission failed: {exc}'
            )
            if self.pending_pose is None:
                self.pending_pose = pose
            return

        if not handle.accepted:
            self.get_logger().warn(
                'Navigation goal rejected.'
            )
            return

        self.active_handle = handle
        self.cancel_requested = False

        self.get_logger().info(
            'Navigation goal accepted.'
        )

        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f, h=handle, p=pose:
            self.result_callback(f, h, p)
        )

        if self.paused or self.pending_pose is not None:
            # Keep original destination for resume,
            # unless another goal has replaced it.
            if self.pending_pose is None:
                self.pending_pose = pose

            self.request_cancel()

    def request_cancel(self):

        if self.active_handle is None:
            return

        if self.cancel_requested:
            return

        self.cancel_requested = True

        self.get_logger().warn(
            'Requesting navigation cancellation.'
        )

        future = self.active_handle.cancel_goal_async()

        future.add_done_callback(
            self.cancel_callback
        )

    def cancel_callback(self, future):

        try:
            response = future.result()

            if response.return_code == (
                CancelGoal.Response.ERROR_NONE
            ):
                self.get_logger().info(
                    'Cancellation request accepted.'
                )
            else:
                self.get_logger().error(
                    'Cancellation request failed: '
                    f'{response.return_code}'
                )

        except Exception as exc:
            self.get_logger().error(
                f'Cancellation error: {exc}'
            )

    def result_callback(self, future, handle, pose):

        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            self.get_logger().error(
                f'Navigation result error: {exc}'
            )
            return

        if self.active_handle is not handle:
            return

        self.active_handle = None
        self.cancel_requested = False

        if status == GoalStatus.STATUS_SUCCEEDED:

            self.get_logger().info(
                'Navigation succeeded.'
            )

        elif status == GoalStatus.STATUS_CANCELED:

            self.get_logger().warn(
                'Navigation canceled.'
            )

        elif status == GoalStatus.STATUS_ABORTED:

            self.get_logger().error(
                'Navigation aborted.'
            )

        else:
            self.get_logger().warn(
                f'Navigation status: {status}'
            )

        if not self.paused:
            self.process_pending_goal()


def main(args=None):

    rclpy.init(args=args)

    node = PauseResumeNav()

    executor = MultiThreadedExecutor(
        num_threads=2
    )

    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
