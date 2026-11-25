#!/usr/bin/env python3

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class PauseResumeNav(Node):

    def __init__(self):
        super().__init__('pause_resume_nav')

        # ------------------------------------------
        # Nav2 action client
        # ------------------------------------------
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # ------------------------------------------
        # Subscribers
        # ------------------------------------------
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

        # ------------------------------------------
        # Navigation state
        # ------------------------------------------
        self.paused = False

        # Destination awaiting execution
        self.pending_pose = None

        # Currently executing destination
        self.active_pose = None
        self.active_handle = None

        # Goal being submitted
        self.sending = False
        self.sending_pose = None

        # Cancellation state
        self.cancel_requested = False

        # Periodically retry pending destinations
        self.timer = self.create_timer(
            1.0,
            self.process_pending_goal
        )

        self.get_logger().info(
            'PauseResumeNav ready - ROS 2 Jazzy'
        )

    # ==============================================
    # RECEIVE GOAL FROM RVIZ
    # ==============================================
    def goal_callback(self, msg):

        self.pending_pose = deepcopy(msg)

        self.get_logger().info(
            'Navigation destination received: '
            f'x={msg.pose.position.x:.3f}, '
            f'y={msg.pose.position.y:.3f}'
        )

        if self.paused:
            self.get_logger().warn(
                'Robot paused. New destination saved.'
            )
            return

        self.process_pending_goal()

    # ==============================================
    # PAUSE / RESUME COMMAND
    # ==============================================
    def command_callback(self, msg):

        command = msg.data.strip().lower()

        if command == 'pause':

            if self.paused:
                self.get_logger().info(
                    'Robot is already paused.'
                )
                return

            self.paused = True

            # CRITICAL FIX:
            # Preserve the active navigation goal.
            # A newer pending goal takes priority.
            if self.pending_pose is None:

                if self.active_pose is not None:
                    self.pending_pose = deepcopy(
                        self.active_pose
                    )

                elif self.sending_pose is not None:
                    self.pending_pose = deepcopy(
                        self.sending_pose
                    )

            self.get_logger().warn(
                'PAUSE REQUESTED'
            )

            if self.pending_pose is not None:
                self.get_logger().info(
                    'Destination saved for resume.'
                )
            else:
                self.get_logger().warn(
                    'No managed destination to save.'
                )

            self.request_cancel()

        elif command == 'resume':

            if not self.paused:
                self.get_logger().info(
                    'Robot is already in running mode.'
                )
                return

            self.paused = False

            self.get_logger().info(
                'RESUME REQUESTED'
            )

            if self.pending_pose is None:
                self.get_logger().warn(
                    'Cannot resume: no destination stored.'
                )
                return

            self.get_logger().info(
                'Resuming navigation to: '
                f'x={self.pending_pose.pose.position.x:.3f}, '
                f'y={self.pending_pose.pose.position.y:.3f}'
            )

            self.process_pending_goal()

        else:
            self.get_logger().warn(
                f'Unknown command: {command}'
            )

    # ==============================================
    # PROCESS PENDING GOAL
    # ==============================================
    def process_pending_goal(self):

        if self.paused:
            return

        if self.pending_pose is None:
            return

        # Wait for an in-flight goal submission
        if self.sending:
            return

        # Do not send a new goal while the old
        # action is still running or cancelling.
        if self.active_handle is not None:

            if not self.cancel_requested:
                self.request_cancel()

            return

        if not self.nav_client.server_is_ready():
            return

        self.send_navigation_goal()

    # ==============================================
    # SEND NAVIGATION GOAL
    # ==============================================
    def send_navigation_goal(self):

        if self.paused:
            return

        if self.pending_pose is None:
            return

        pose = deepcopy(self.pending_pose)

        self.pending_pose = None
        self.sending_pose = deepcopy(pose)
        self.sending = True

        goal = NavigateToPose.Goal()

        goal.pose = deepcopy(pose)
        goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        self.get_logger().info(
            'Sending NavigateToPose goal...'
        )

        try:
            future = self.nav_client.send_goal_async(
                goal
            )

            future.add_done_callback(
                lambda f, p=pose:
                self.goal_response_callback(f, p)
            )

        except Exception as exc:

            self.sending = False
            self.sending_pose = None

            if self.pending_pose is None:
                self.pending_pose = pose

            self.get_logger().error(
                f'Failed to send goal: {exc}'
            )

    # ==============================================
    # GOAL RESPONSE
    # ==============================================
    def goal_response_callback(self, future, pose):

        self.sending = False
        self.sending_pose = None

        try:
            handle = future.result()

        except Exception as exc:

            self.get_logger().error(
                f'Goal response error: {exc}'
            )

            if self.pending_pose is None:
                self.pending_pose = deepcopy(pose)

            return

        if not handle.accepted:

            self.get_logger().error(
                'NavigateToPose goal rejected.'
            )

            # Do not endlessly retry rejected goals.
            if self.paused and self.pending_pose is None:
                self.pending_pose = deepcopy(pose)

            return

        self.active_handle = handle
        self.active_pose = deepcopy(pose)
        self.cancel_requested = False

        self.get_logger().info(
            'Navigation goal ACCEPTED.'
        )

        result_future = handle.get_result_async()

        result_future.add_done_callback(
            lambda f, h=handle:
            self.result_callback(f, h)
        )

        # Pause may have arrived during submission
        if self.paused:

            if self.pending_pose is None:
                self.pending_pose = deepcopy(pose)

            self.request_cancel()

        # A newer destination may have arrived
        elif self.pending_pose is not None:

            self.request_cancel()

    # ==============================================
    # CANCEL NAVIGATION
    # ==============================================
    def request_cancel(self):

        if self.active_handle is None:
            return

        if self.cancel_requested:
            return

        self.cancel_requested = True

        self.get_logger().warn(
            'Cancelling current navigation goal...'
        )

        try:
            future = (
                self.active_handle.cancel_goal_async()
            )

            future.add_done_callback(
                self.cancel_callback
            )

        except Exception as exc:

            self.cancel_requested = False

            self.get_logger().error(
                f'Cancellation request failed: {exc}'
            )

    # ==============================================
    # CANCEL RESPONSE
    # ==============================================
    def cancel_callback(self, future):

        try:
            response = future.result()

            if response.return_code == (
                CancelGoal.Response.ERROR_NONE
            ):

                self.get_logger().info(
                    'Cancellation accepted. '
                    'Waiting for final action result.'
                )

            else:

                self.cancel_requested = False

                self.get_logger().error(
                    'Cancellation rejected: '
                    f'{response.return_code}'
                )

        except Exception as exc:

            self.cancel_requested = False

            self.get_logger().error(
                f'Cancel response error: {exc}'
            )

    # ==============================================
    # NAVIGATION RESULT
    # ==============================================
    def result_callback(self, future, handle):

        if self.active_handle is not handle:
            return

        try:
            result = future.result()
            status = result.status

        except Exception as exc:

            self.get_logger().error(
                f'Navigation result error: {exc}'
            )
            return

        self.active_handle = None
        self.active_pose = None
        self.cancel_requested = False

        if status == GoalStatus.STATUS_SUCCEEDED:

            self.get_logger().info(
                'Navigation SUCCEEDED.'
            )

        elif status == GoalStatus.STATUS_CANCELED:

            self.get_logger().warn(
                'Navigation CANCELED.'
            )

        elif status == GoalStatus.STATUS_ABORTED:

            self.get_logger().error(
                'Navigation ABORTED.'
            )

        else:

            self.get_logger().warn(
                f'Navigation ended with status {status}'
            )

        # If resume was requested while canceling,
        # the saved destination is sent now.
        if not self.paused:
            self.process_pending_goal()


def main(args=None):

    rclpy.init(args=args)

    node = PauseResumeNav()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
