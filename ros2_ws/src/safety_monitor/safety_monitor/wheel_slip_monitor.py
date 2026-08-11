#!/usr/bin/env python3
"""ROS adapter for timestamp-aligned robot motion mismatch detection."""

from datetime import datetime, timezone
import json
import math
import os

from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from safety_monitor.slip_detection_core import (
    DetectorConfig,
    DetectorState,
    SlipDetectorCore,
    relative_tilt_angle,
)


def _stamp_seconds(message, fallback: float) -> float:
    stamp = message.header.stamp
    value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    return value if value > 0.0 else fallback


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _roll_pitch_from_quaternion(q):
    roll = math.atan2(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y),
    )
    pitch_term = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, pitch_term)))
    return roll, pitch


class WheelSlipMonitor(Node):
    def __init__(self):
        super().__init__('wheel_slip_monitor')

        defaults = {
            'odom_raw_topic': 'odom_raw',
            'imu_gyro_topic': 'imu_corrected',
            'imu_orientation_topic': 'imu',
            'rf2o_topic': 'odom_rf2o',
            'command_topic': '/cmd_vel',
            'state_topic': '/safety/slip_state',
            'stop_topic': '/safety/stop_requested',
            'slam_allow_topic': '/safety/slam_scan_allowed',
            'diagnostics_topic': '/diagnostics',
            'check_rate': 20.0,
            'rotation_window': 0.35,
            'rotation_min_expected_deg': 5.0,
            'rotation_abs_error_deg': 3.0,
            'rotation_rel_error': 0.35,
            'rotation_confirmations': 3,
            'linear_window': 0.60,
            'linear_min_expected': 0.04,
            'linear_abs_error': 0.04,
            'linear_rel_error': 0.40,
            'lateral_abs_error': 0.035,
            'straight_max_expected_yaw_deg': 6.0,
            'linear_confirmations': 3,
            'data_timeout': 0.50,
            'maximum_sample_gap': 0.15,
            'tilt_enter_deg': 8.0,
            'tilt_exit_deg': 5.0,
            'tilt_stop_deg': 15.0,
            'tilt_reference_roll_deg': -177.0,
            'tilt_reference_pitch_deg': -0.2,
            'stop_on_excessive_tilt': True,
            'slam_max_expected_yaw_rate': 0.35,
            'slam_max_observed_yaw_rate': 0.45,
            'slam_resume_stable_time': 0.80,
            'cancel_nav2_on_trip': True,
            'event_log_path': '~/.ros/safety_monitor/slip_events.jsonl',
            'nav2_cancel_services': [
                '/navigate_to_pose/_action/cancel_goal',
                '/navigate_through_poses/_action/cancel_goal',
            ],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        radians = math.radians
        config = DetectorConfig(
            rotation_window=self.get_parameter('rotation_window').value,
            rotation_min_expected=radians(self.get_parameter('rotation_min_expected_deg').value),
            rotation_abs_error=radians(self.get_parameter('rotation_abs_error_deg').value),
            rotation_rel_error=self.get_parameter('rotation_rel_error').value,
            rotation_confirmations=self.get_parameter('rotation_confirmations').value,
            linear_window=self.get_parameter('linear_window').value,
            linear_min_expected=self.get_parameter('linear_min_expected').value,
            linear_abs_error=self.get_parameter('linear_abs_error').value,
            linear_rel_error=self.get_parameter('linear_rel_error').value,
            lateral_abs_error=self.get_parameter('lateral_abs_error').value,
            straight_max_expected_yaw=radians(
                self.get_parameter('straight_max_expected_yaw_deg').value
            ),
            linear_confirmations=self.get_parameter('linear_confirmations').value,
            data_timeout=self.get_parameter('data_timeout').value,
            maximum_sample_gap=self.get_parameter('maximum_sample_gap').value,
            tilt_enter=radians(self.get_parameter('tilt_enter_deg').value),
            tilt_exit=radians(self.get_parameter('tilt_exit_deg').value),
            tilt_stop=radians(self.get_parameter('tilt_stop_deg').value),
            stop_on_excessive_tilt=self.get_parameter('stop_on_excessive_tilt').value,
            slam_max_expected_yaw_rate=self.get_parameter('slam_max_expected_yaw_rate').value,
            slam_max_observed_yaw_rate=self.get_parameter('slam_max_observed_yaw_rate').value,
            slam_resume_stable_time=self.get_parameter('slam_resume_stable_time').value,
        )
        self.detector = SlipDetectorCore(config)
        self.tilt_reference_roll = radians(
            self.get_parameter('tilt_reference_roll_deg').value
        )
        self.tilt_reference_pitch = radians(
            self.get_parameter('tilt_reference_pitch_deg').value
        )
        self.raw_roll = math.nan
        self.raw_pitch = math.nan
        self.relative_tilt = math.nan
        self.last_state = None
        self.cancel_sent = False
        self.last_cancel_attempt_time = float('-inf')
        self.cancel_unavailable_warned = False

        self.create_subscription(
            Odometry, self.get_parameter('odom_raw_topic').value, self.on_odom_raw, 50
        )
        self.create_subscription(
            Imu, self.get_parameter('imu_gyro_topic').value, self.on_gyro, 100
        )
        self.create_subscription(
            Imu, self.get_parameter('imu_orientation_topic').value, self.on_orientation, 50
        )
        self.create_subscription(
            Odometry, self.get_parameter('rf2o_topic').value, self.on_rf2o, 20
        )
        self.create_subscription(
            Twist, self.get_parameter('command_topic').value, self.on_command, 20
        )

        self.state_pub = self.create_publisher(String, self.get_parameter('state_topic').value, 10)
        self.stop_pub = self.create_publisher(Bool, self.get_parameter('stop_topic').value, 10)
        self.slam_allow_pub = self.create_publisher(
            Bool, self.get_parameter('slam_allow_topic').value, 10
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, self.get_parameter('diagnostics_topic').value, 10
        )
        self.create_service(Trigger, '/safety/reset', self.on_reset)

        self.cancel_nav2_on_trip = self.get_parameter('cancel_nav2_on_trip').value
        self.event_log_path = os.path.expanduser(
            self.get_parameter('event_log_path').value
        )
        cancel_services = self.get_parameter('nav2_cancel_services').value
        self.nav_cancel_clients = [self.create_client(CancelGoal, name) for name in cancel_services]

        rate = float(self.get_parameter('check_rate').value)
        self.create_timer(1.0 / rate, self.check)
        self.get_logger().info(
            'Motion mismatch monitor started: odom_raw vs calibrated gyro; '
            'RF2O translation enabled; level reference roll=%.2f deg pitch=%.2f deg'
            % (
                math.degrees(self.tilt_reference_roll),
                math.degrees(self.tilt_reference_pitch),
            )
        )

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def on_odom_raw(self, msg: Odometry):
        stamp = _stamp_seconds(msg, self.now_seconds())
        pose = msg.pose.pose
        self.detector.add_odom(
            stamp, pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation)
        )

    def on_rf2o(self, msg: Odometry):
        stamp = _stamp_seconds(msg, self.now_seconds())
        pose = msg.pose.pose
        self.detector.add_rf2o(
            stamp, pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation)
        )

    def on_gyro(self, msg: Imu):
        self.detector.add_gyro(
            _stamp_seconds(msg, self.now_seconds()), msg.angular_velocity.z
        )

    def on_orientation(self, msg: Imu):
        stamp = _stamp_seconds(msg, self.now_seconds())
        q = msg.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if msg.orientation_covariance[0] == -1.0 or norm < 0.5:
            return
        roll, pitch = _roll_pitch_from_quaternion(q)
        tilt = relative_tilt_angle(
            roll,
            pitch,
            self.tilt_reference_roll,
            self.tilt_reference_pitch,
        )
        self.raw_roll = roll
        self.raw_pitch = pitch
        self.relative_tilt = tilt
        self.detector.set_orientation(stamp, tilt, 0.0)

    def on_command(self, msg: Twist):
        self.detector.set_command(
            self.now_seconds(), msg.linear.x, msg.linear.y, msg.angular.z
        )

    def on_reset(self, _request, response):
        success, reason = self.detector.reset(self.now_seconds())
        response.success = success
        response.message = reason
        if success:
            self.cancel_sent = False
            self.last_cancel_attempt_time = float('-inf')
            self.cancel_unavailable_warned = False
            self.get_logger().warning('Safety latch reset; a fresh Nav2 goal is required')
        return response

    def cancel_nav2_goals(self):
        if not self.cancel_nav2_on_trip or self.cancel_sent:
            return 'disabled' if not self.cancel_nav2_on_trip else 'already_sent'
        now = self.now_seconds()
        if now - self.last_cancel_attempt_time < 0.5:
            return 'rate_limited'
        self.last_cancel_attempt_time = now
        request = CancelGoal.Request()
        any_sent = False
        for client in self.nav_cancel_clients:
            if client.service_is_ready():
                client.call_async(request)
                any_sent = True
        self.cancel_sent = any_sent
        if any_sent:
            self.cancel_unavailable_warned = False
        elif not self.cancel_unavailable_warned:
            self.get_logger().warning('Safety tripped, but Nav2 cancel services are not ready')
            self.cancel_unavailable_warned = True
        return 'sent' if any_sent else 'services_unavailable'

    def record_trip_event(self, result, nav2_cancel_effect: str):
        event = {
            'wall_time_utc': datetime.now(timezone.utc).isoformat(),
            'ros_stamp': result.stamp,
            'state': result.state.value,
            'reason': result.reason,
            'measurements': {
                'expected_yaw_rad': result.expected_yaw,
                'observed_yaw_rad': result.observed_yaw,
                'yaw_error_rad': result.yaw_error,
                'expected_forward_m': result.expected_forward,
                'observed_forward_m': result.observed_forward,
                'longitudinal_error_m': result.longitudinal_error,
                'lateral_error_m': result.lateral_error,
                'tilted': result.tilted,
                'excessive_tilt': result.excessive_tilt,
                'raw_roll_deg': math.degrees(self.raw_roll),
                'raw_pitch_deg': math.degrees(self.raw_pitch),
                'reference_roll_deg': math.degrees(self.tilt_reference_roll),
                'reference_pitch_deg': math.degrees(self.tilt_reference_pitch),
                'relative_tilt_deg': math.degrees(self.relative_tilt),
            },
            'effects': {
                'stop_requested': result.stop_requested,
                'slam_scan_allowed': result.slam_scan_allowed,
                'cmd_vel_gate_expected': 'blocked' if result.stop_requested else 'open',
                'nav2_cancel': nav2_cancel_effect,
            },
        }
        try:
            directory = os.path.dirname(self.event_log_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(self.event_log_path, 'a', encoding='utf-8') as stream:
                stream.write(json.dumps(event, ensure_ascii=False, sort_keys=True) + '\n')
            self.get_logger().error(
                'Slip event recorded: reason=%s stop=%s slam_scan_allowed=%s '
                'nav2_cancel=%s path=%s'
                % (
                    result.reason,
                    result.stop_requested,
                    result.slam_scan_allowed,
                    nav2_cancel_effect,
                    self.event_log_path,
                )
            )
        except OSError as error:
            self.get_logger().error(
                'Failed to persist slip event at %s: %s' % (self.event_log_path, error)
            )

    def _diagnostic_values(self, result):
        pairs = {
            'state': result.state.value,
            'reason': result.reason,
            'slam_scan_allowed': str(result.slam_scan_allowed),
            'tilted': str(result.tilted),
            'rotation_healthy': str(result.rotation_healthy),
            'linear_healthy': str(result.linear_healthy),
            'expected_yaw_rad': f'{result.expected_yaw:.6f}',
            'observed_yaw_rad': f'{result.observed_yaw:.6f}',
            'yaw_error_rad': f'{result.yaw_error:.6f}',
            'expected_forward_m': f'{result.expected_forward:.6f}',
            'observed_forward_m': f'{result.observed_forward:.6f}',
            'longitudinal_error_m': f'{result.longitudinal_error:.6f}',
            'lateral_error_m': f'{result.lateral_error:.6f}',
            'raw_roll_deg': f'{math.degrees(self.raw_roll):.3f}',
            'raw_pitch_deg': f'{math.degrees(self.raw_pitch):.3f}',
            'reference_roll_deg': f'{math.degrees(self.tilt_reference_roll):.3f}',
            'reference_pitch_deg': f'{math.degrees(self.tilt_reference_pitch):.3f}',
            'relative_tilt_deg': f'{math.degrees(self.relative_tilt):.3f}',
        }
        return [KeyValue(key=key, value=value) for key, value in pairs.items()]

    def publish_diagnostics(self, result):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'safety_monitor/motion_mismatch'
        status.hardware_id = 'jetrover'
        status.message = result.reason
        if result.state == DetectorState.TRIPPED:
            status.level = DiagnosticStatus.ERROR
        elif result.state in (DetectorState.SUSPECT, DetectorState.DEGRADED):
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.OK
        status.values = self._diagnostic_values(result)
        array.status = [status]
        self.diagnostics_pub.publish(array)

    def check(self):
        result = self.detector.evaluate(self.now_seconds())
        self.state_pub.publish(String(data=f'{result.state.value}:{result.reason}'))
        self.stop_pub.publish(Bool(data=result.stop_requested))
        self.slam_allow_pub.publish(Bool(data=result.slam_scan_allowed))
        self.publish_diagnostics(result)

        state_changed = result.state != self.last_state
        if state_changed:
            message = f'safety state {self.last_state} -> {result.state.value}: {result.reason}'
            if result.state == DetectorState.TRIPPED:
                self.get_logger().error(message)
            elif result.state in (DetectorState.SUSPECT, DetectorState.DEGRADED):
                self.get_logger().warning(message)
            else:
                self.get_logger().info(message)
            self.last_state = result.state
        if result.state == DetectorState.TRIPPED:
            nav2_cancel_effect = self.cancel_nav2_goals()
            if state_changed:
                self.record_trip_event(result, nav2_cancel_effect)


def main():
    rclpy.init()
    node = WheelSlipMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
