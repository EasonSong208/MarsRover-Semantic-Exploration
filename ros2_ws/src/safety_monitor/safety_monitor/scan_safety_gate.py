#!/usr/bin/env python3
"""Split laser scans into a 40 cm local channel and a gated SLAM channel."""

import copy
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def make_local_scan(message: LaserScan, maximum_range: float) -> LaserScan:
    """Return a scan that cannot mark obstacles beyond ``maximum_range``."""
    result = copy.deepcopy(message)
    result.range_max = min(float(message.range_max), maximum_range)
    result.ranges = [
        value if math.isfinite(value) and value <= maximum_range else math.inf
        for value in message.ranges
    ]
    return result


class ScanSafetyGate(Node):
    def __init__(self):
        super().__init__('scan_safety_gate')
        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('local_topic', 'scan_local')
        self.declare_parameter('slam_topic', 'scan_slam')
        self.declare_parameter('slam_allow_topic', '/safety/slam_scan_allowed')
        self.declare_parameter('local_max_range', 0.40)
        self.declare_parameter('allow_state_timeout', 0.50)

        self.local_max_range = float(self.get_parameter('local_max_range').value)
        self.allow_state_timeout = float(self.get_parameter('allow_state_timeout').value)
        self.slam_allowed = False
        self.last_allow_time = None

        self.local_pub = self.create_publisher(
            LaserScan, self.get_parameter('local_topic').value, 10
        )
        self.slam_pub = self.create_publisher(
            LaserScan, self.get_parameter('slam_topic').value, 10
        )
        self.create_subscription(
            Bool,
            self.get_parameter('slam_allow_topic').value,
            self.on_slam_allow,
            10,
        )
        self.create_subscription(
            LaserScan, self.get_parameter('input_topic').value, self.on_scan, 10
        )
        self.get_logger().info(
            f'Scan safety gate started: local range <= {self.local_max_range:.2f} m; '
            'SLAM is fail-closed'
        )

    def on_slam_allow(self, message: Bool):
        self.slam_allowed = bool(message.data)
        self.last_allow_time = time.monotonic()

    def on_scan(self, message: LaserScan):
        self.local_pub.publish(make_local_scan(message, self.local_max_range))
        allow_is_fresh = (
            self.last_allow_time is not None
            and time.monotonic() - self.last_allow_time <= self.allow_state_timeout
        )
        if self.slam_allowed and allow_is_fresh:
            self.slam_pub.publish(message)


def main():
    rclpy.init()
    node = ScanSafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
