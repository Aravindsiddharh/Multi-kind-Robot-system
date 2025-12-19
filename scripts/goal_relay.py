#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class MultiGoalRelay(Node):
    def __init__(self):
        super().__init__('goal_relay')

        # Declare parameters
        self.declare_parameter('robot_names', ['robot1', 'robot2'])
        self.declare_parameter('active_robot', 'all')  # 'all' or one robot name

        # Load params
        self.robot_names = self.get_parameter('robot_names').get_parameter_value().string_array_value
        self.active_robot = self.get_parameter('active_robot').get_parameter_value().string_value

        # Subscribe to RViz's /goal_pose
        self.sub = self.create_subscription(PoseStamped, '/goal_pose', self.cb, 10)

        # Create publishers dynamically for each robot
        self.goal_publishers = {
            name: self.create_publisher(PoseStamped, f'/{name}/goal_pose', 10)
            for name in self.robot_names
        }

        self.get_logger().info(
            f"Relaying /goal_pose → {[f'/{n}/goal_pose' for n in self.robot_names]}"
            f" | Active mode: {self.active_robot}"
        )

        # Timer to update params dynamically
        self.create_timer(1.0, self.check_param_update)

    def check_param_update(self):
        active_robot = self.get_parameter('active_robot').get_parameter_value().string_value
        if active_robot != self.active_robot:
            self.active_robot = active_robot
            self.get_logger().info(f"Switched relay active mode → {self.active_robot}")

    def cb(self, msg: PoseStamped):
        if self.active_robot == 'all':
            # Broadcast to all robots
            for pub in self.goal_publishers.values():
                pub.publish(msg)
        elif self.active_robot in self.goal_publishers:
            # Send only to the selected robot
            self.goal_publishers[self.active_robot].publish(msg)
        else:
            self.get_logger().warn(f"Active robot '{self.active_robot}' not in {self.robot_names}")

def main(args=None):
    rclpy.init(args=args)
    node = MultiGoalRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
