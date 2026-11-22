#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import math
import threading
import time
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String


class PauseResumeNav(Node):
    def __init__(self):
        super().__init__('pause_resume_nav')

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.command_sub = self.create_subscription(String, 'nav_control', self.command_callback, 10)

        self.pause_requested = False
        self.pause_event = threading.Event()
        self.pause_event.set()

        self.goal_handle = None
        self.stored_goal = None

        self.get_logger().info('PauseResumeNav node is running.')

    def command_callback(self, msg):
        command = msg.data.strip().lower()
        if command == 'pause':
            self.get_logger().warn("[User] Pause requested.")
            self.pause_requested = True
            self.pause_event.clear()

            if self.goal_handle:
                cancel_future = self.goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.cancel_done_callback)

            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            for _ in range(10):
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.1)

        elif command == 'resume':
            self.get_logger().info("[User] Resume requested.")
            self.pause_requested = False
            self.pause_event.set()
            if self.stored_goal:
                x, y, theta = self.stored_goal
                self.send_navigation_goal(x, y, theta)

    def goal_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w
        theta = math.atan2(z, w) * 2
        self.send_navigation_goal(x, y, theta)

    def cancel_done_callback(self, future):
        result = future.result()
        if result and result.return_code == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Goal canceled successfully.")
        else:
            self.get_logger().error(f"Failed to cancel goal. Return code: {result.return_code if result else 'Unknown'}")

    def send_navigation_goal(self, x, y, theta):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose action server not available.')
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = 'map'
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = x
        nav_goal.pose.pose.position.y = y
        nav_goal.pose.pose.orientation.z = math.sin(theta / 2.0)
        nav_goal.pose.pose.orientation.w = math.cos(theta / 2.0)

        self.stored_goal = (x, y, theta)

        future = self.nav_to_pose_client.send_goal_async(nav_goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle or not self.goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by NavigateToPose.')
            return

        self.get_logger().info('Goal accepted. Waiting for result...')

        get_result_future = self.goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result is None:
            self.get_logger().warn('No result received.')
            return

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Goal was aborted.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Goal was canceled.')
        else:
            self.get_logger().warn(f'Goal ended with status: {status}')


def main(args=None):
    rclpy.init(args=args)
    node = PauseResumeNav()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from rclpy.executors import MultiThreadedExecutor
# from nav2_msgs.action import NavigateToPose
# from action_msgs.msg import GoalStatus
# import math
# import threading
# import time
# from geometry_msgs.msg import PoseStamped, Twist
# from tf_transformations import euler_from_quaternion


# class PauseResumeNav(Node):
#     def __init__(self):
#         super().__init__('pause_resume_nav')

#         self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
#         self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

#         self.pause_requested = False
#         self.pause_event = threading.Event()
#         self.pause_event.set()

#         self.goal_handle = None
#         self.stored_goal = None

#         self.goal_sub = self.create_subscription(
#             PoseStamped,
#             '/goal_pose',
#             self.goal_callback,
#             10
#         )

#         self.monitor_thread = threading.Thread(target=self._keyboard_monitor_thread, daemon=True)
#         self.monitor_thread.start()

#         self.get_logger().info('PauseResumeNav node is running.')

#     def goal_callback(self, msg: PoseStamped):
#         x = msg.pose.position.x
#         y = msg.pose.position.y
#         quat = msg.pose.orientation
#         _, _, theta = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

#         self.get_logger().info(f"Received 2D goal: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")
#         self.send_navigation_goal(x, y, theta)

#     def _keyboard_monitor_thread(self):
#         while True:
#             user_input = input().strip().lower()
#             if user_input == 'pause':
#                 self.get_logger().warn("[User] Pause requested.")
#                 self.pause_requested = True
#                 self.pause_event.clear()

#                 if self.goal_handle:
#                     cancel_future = self.goal_handle.cancel_goal_async()
#                     cancel_future.add_done_callback(self.cancel_done_callback)

#                 twist = Twist()
#                 twist.linear.x = 0.0
#                 twist.angular.z = 0.0
#                 for _ in range(10):
#                     self.cmd_vel_pub.publish(twist)
#                     time.sleep(0.1)

#             elif user_input == 'resume':
#                 self.get_logger().info("[User] Resume requested.")
#                 self.pause_requested = False
#                 self.pause_event.set()
#                 if self.stored_goal:
#                     x, y, theta = self.stored_goal
#                     self.send_navigation_goal(x, y, theta)

#     def cancel_done_callback(self, future):
#         result = future.result()
#         if result and result.return_code == GoalStatus.STATUS_CANCELED:
#             self.get_logger().warn("Goal canceled successfully.")
#         else:
#             self.get_logger().warn(f"Failed to cancel goal. Return code: {result.return_code if result else 'Unknown'}")

#     def send_navigation_goal(self, x, y, theta):
#         if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
#             self.get_logger().error('NavigateToPose action server not available.')
#             return

#         nav_goal = NavigateToPose.Goal()
#         nav_goal.pose.header.frame_id = 'map'
#         nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
#         nav_goal.pose.pose.position.x = x
#         nav_goal.pose.pose.position.y = y
#         nav_goal.pose.pose.orientation.z = math.sin(theta / 2.0)
#         nav_goal.pose.pose.orientation.w = math.cos(theta / 2.0)

#         self.stored_goal = (x, y, theta)

#         future = self.nav_to_pose_client.send_goal_async(nav_goal)
#         future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         self.goal_handle = future.result()
#         if not self.goal_handle or not self.goal_handle.accepted:
#             self.get_logger().warn('Goal was rejected by NavigateToPose.')
#             return

#         self.get_logger().info('Goal accepted. Waiting for result...')

#         get_result_future = self.goal_handle.get_result_async()
#         get_result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         result = future.result()
#         if result is None:
#             self.get_logger().warn('No result received.')
#             return

#         status = result.status
#         if status == GoalStatus.STATUS_SUCCEEDED:
#             self.get_logger().info('Goal succeeded!')
#         elif status == GoalStatus.STATUS_ABORTED:
#             self.get_logger().warn('Goal was aborted.')
#         elif status == GoalStatus.STATUS_CANCELED:
#             self.get_logger().warn('Goal was canceled.')
#         else:
#             self.get_logger().warn(f'Goal ended with status: {status}')


# def main(args=None):
#     rclpy.init(args=args)
#     node = PauseResumeNav()
#     executor = MultiThreadedExecutor()
#     executor.add_node(node)
#     executor.spin()
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
