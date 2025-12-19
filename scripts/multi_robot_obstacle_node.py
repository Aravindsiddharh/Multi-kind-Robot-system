#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
import tf2_ros
import numpy as np


class InterRobotObstacleBroadcaster(Node):
    def __init__(self):
        super().__init__('inter_robot_obstacle_broadcaster')

        # <<<<<<<<<<<<< EDIT ONLY THESE TWO LINES >>>>>>>>>>>>>>
        self.robot_namespaces = ['robot1', 'robot2',] #'robot3']   # add/remove here
        self.safety_radius = 0.65                               # meters
        # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< >>>>>>>>>>

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # One publisher per robot
        self.publisher = {}
        for ns in self.robot_namespaces:
            self.publisher[ns] = self.create_publisher(
                PointCloud, f'/{ns}/other_robots_pc', 10)

        self.timer = self.create_timer(0.1, self.publish_callback)  # 10 Hz
        self.get_logger().info(f"Inter-robot 2D broadcaster STARTED for {self.robot_namespaces}")

    def publish_callback(self):
        for my_robot in self.robot_namespaces:
            points = []

            for other_robot in self.robot_namespaces:
                if other_robot == my_robot:
                    continue

                try:
                    trans = self.tf_buffer.lookup_transform(
                        'map',
                        f'{other_robot}/base_link',
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.05)
                    )

                    cx = trans.transform.translation.x
                    cy = trans.transform.translation.y

                    # 14-point circle = perfect for costmap (smooth + cheap)
                    for angle in np.linspace(0, 2*np.pi, 14, endpoint=False):
                        px = cx + self.safety_radius * np.cos(angle)
                        py = cy + self.safety_radius * np.sin(angle)
                        points.append(Point32(x=px, y=py, z=0.0))

                except Exception:
                    continue  # robot not visible yet

            if points:
                msg = PointCloud()
                msg.header.frame_id = 'map'
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.points = points
                self.publisher[my_robot].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = InterRobotObstacleBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()