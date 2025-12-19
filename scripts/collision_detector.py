#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
from shapely.geometry import Point, Polygon


class SlowdownCollisionDetector(Node):
    def __init__(self):
        super().__init__('collision_detector')
        self.get_logger().info("Collision detector node started.")

        self.latest_scan = None
        self.last_publish_time = self.get_clock().now()
        self.publish_interval_sec = 1.0  # Publish at most once per second

        self.threshold_distance = 1.0  # Optional filter if needed

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            5
        )

        self.slowdown_sub = self.create_subscription(
            PolygonStamped,
            'polygon_slowdown',
            self.slowdown_callback,
            1
        )

        self.alert_pub = self.create_publisher(String, '/collision_status', 10)

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def slowdown_callback(self, msg: PolygonStamped):
        if self.latest_scan is None:
            return

        # Build slowdown polygon as shapely Polygon
        polygon = Polygon([(p.x, p.y) for p in msg.polygon.points])
        if not polygon.is_valid or polygon.is_empty:
            return

        # Iterate over scan points to check if any point lies within polygon
        angle = self.latest_scan.angle_min
        closest_distance = None
        for r in self.latest_scan.ranges:
            if 0.0 < r < self.latest_scan.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                if polygon.contains(Point(x, y)):
                    if closest_distance is None or r < closest_distance:
                        closest_distance = r
            angle += self.latest_scan.angle_increment

        if closest_distance is not None:
            # Check cooldown
            now = self.get_clock().now()
            if (now - self.last_publish_time).nanoseconds / 1e9 >= self.publish_interval_sec:
                alert_msg = String()
                alert_msg.data = f"Obstacle detected at {closest_distance:.2f} meters"
                self.alert_pub.publish(alert_msg)
                self.get_logger().info(alert_msg.data)
                self.last_publish_time = now


def main(args=None):
    rclpy.init(args=args)
    node = SlowdownCollisionDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
