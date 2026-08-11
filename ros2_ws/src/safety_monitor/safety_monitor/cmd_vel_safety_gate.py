#!/usr/bin/env python3
"""Fail-closed final Twist gate between all command producers and the chassis."""

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class CmdVelSafetyGate(Node):
    def __init__(self):
        super().__init__('cmd_vel_safety_gate')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/controller/cmd_vel')
        self.declare_parameter('stop_topic', '/safety/stop_requested')
        self.declare_parameter('blocked_topic', '/safety/cmd_gate_blocked')
        self.declare_parameter('input_timeout', 0.30)
        self.declare_parameter('safety_state_timeout', 0.50)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('max_linear_x', 0.26)
        self.declare_parameter('max_linear_y', 0.20)
        self.declare_parameter('max_angular_z', 1.00)

        self.input_timeout = float(self.get_parameter('input_timeout').value)
        self.safety_state_timeout = float(self.get_parameter('safety_state_timeout').value)
        self.max_linear_x = float(self.get_parameter('max_linear_x').value)
        self.max_linear_y = float(self.get_parameter('max_linear_y').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)

        self.command = Twist()
        self.last_command_time = None
        self.stop_requested = True
        self.last_safety_time = None
        self.last_blocked = None

        self.output_pub = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10
        )
        self.blocked_pub = self.create_publisher(
            Bool, self.get_parameter('blocked_topic').value, 10
        )
        self.create_subscription(
            Twist, self.get_parameter('input_topic').value, self.on_command, 20
        )
        self.create_subscription(
            Bool, self.get_parameter('stop_topic').value, self.on_stop, 10
        )
        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self.publish)
        self.get_logger().info('Final cmd_vel safety gate started in fail-closed mode')

    def on_command(self, message: Twist):
        command = Twist()
        command.linear.x = clamp(message.linear.x, self.max_linear_x)
        command.linear.y = clamp(message.linear.y, self.max_linear_y)
        command.angular.z = clamp(message.angular.z, self.max_angular_z)
        self.command = command
        self.last_command_time = time.monotonic()

    def on_stop(self, message: Bool):
        self.stop_requested = bool(message.data)
        self.last_safety_time = time.monotonic()
        if self.stop_requested:
            self.output_pub.publish(Twist())

    def publish(self):
        now = time.monotonic()
        command_fresh = (
            self.last_command_time is not None
            and now - self.last_command_time <= self.input_timeout
        )
        safety_fresh = (
            self.last_safety_time is not None
            and now - self.last_safety_time <= self.safety_state_timeout
        )
        blocked = self.stop_requested or not safety_fresh or not command_fresh
        self.output_pub.publish(Twist() if blocked else self.command)
        self.blocked_pub.publish(Bool(data=blocked))
        if blocked != self.last_blocked:
            if blocked:
                self.get_logger().warning('cmd_vel output blocked')
            else:
                self.get_logger().info('cmd_vel output enabled')
            self.last_blocked = blocked


def main():
    rclpy.init()
    node = CmdVelSafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
