"""ROS2 node for a bounded, explicitly authorized RGB-D homing experiment."""

import csv
import json
import math
from pathlib import Path
import signal
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from .homing_state_machine import (
    Command, HomingConfig, HomingRunner, Observation, State,
    apply_motion_safety_gate,
)
from .motion_smoke_policy import graph_errors
from .red_marker_detector import (
    DetectorConfig, MarkerDetection, build_reference, depth_preview,
    detect_red_marker,
)
from .rgbd_synchronizer import (
    DIRECT_SLOP, FIXED_OFFSET, ConsecutiveMatchGate, OffsetEstimator,
    RgbDepthSynchronizer, SensorSyncState, SyncMatch, classify_sensor_state,
)


class RedMarkerHomingNode(Node):
    """Perception/state-machine wrapper; it never starts any dependency node."""

    def __init__(self):
        super().__init__('red_marker_homing_test')
        self._declare_parameters()
        self.dry_run = bool(self._value('dry_run'))
        self.confirmed = bool(self._value('confirmed'))
        self.enable_base_motion = bool(self._value('enable_base_motion'))
        self.output_dir = Path(str(self._value('output_directory'))).expanduser()
        self.bridge = CvBridge()
        self.config = self._homing_config()
        self.detector_config = self._detector_config()
        self.runner = HomingRunner(self.config)
        self.require_camera_pose_ready = bool(
            self._value('require_camera_pose_ready'))
        self.camera_pose_ready_topic = str(
            self._value('camera_pose_ready_topic'))
        self.camera_pose_ready_timeout_sec = float(
            self._value('camera_pose_ready_timeout_sec'))
        if (not math.isfinite(self.camera_pose_ready_timeout_sec)
                or self.camera_pose_ready_timeout_sec < 0.0):
            raise ValueError('camera_pose_ready_timeout_sec cannot be negative')
        self.camera_pose_ready = False
        self.camera_pose_ready_time = None
        self.last_camera_pose_gate = None

        self.sync_mode = str(self._value('sync_mode'))
        if self.sync_mode not in (DIRECT_SLOP, FIXED_OFFSET):
            raise ValueError('sync_mode must be direct_slop or fixed_offset')
        self.compare_sync_modes = bool(self._value('compare_sync_modes'))
        self.depth_freshness_limit_sec = (
            float(self._value('depth_freshness_limit_ms')) / 1000.0)
        self.sensor_warn_timeout_sec = (
            float(self._value('sensor_warn_timeout_ms')) / 1000.0)
        self.sensor_stop_timeout_sec = (
            float(self._value('sensor_stop_timeout_ms')) / 1000.0)
        self.sensor_log_throttle_sec = (
            float(self._value('sensor_log_throttle_ms')) / 1000.0)
        self._validate_sync_parameters()

        queue_size = int(self._value('sync_queue_size'))
        max_buffer_age_ms = float(self._value('sync_buffer_max_age_ms'))
        modes = {self.sync_mode}
        if self.compare_sync_modes:
            modes.update((DIRECT_SLOP, FIXED_OFFSET))
        self.synchronizers = {}
        if DIRECT_SLOP in modes:
            self.synchronizers[DIRECT_SLOP] = RgbDepthSynchronizer(
                DIRECT_SLOP, float(self._value('direct_slop_ms')),
                queue_size=queue_size, max_buffer_age_ms=max_buffer_age_ms)
        if FIXED_OFFSET in modes:
            self.synchronizers[FIXED_OFFSET] = RgbDepthSynchronizer(
                FIXED_OFFSET, float(self._value('fixed_offset_slop_ms')),
                depth_stamp_offset_ms=float(
                    self._value('depth_stamp_offset_ms')),
                queue_size=queue_size, max_buffer_age_ms=max_buffer_age_ms)
        self.offset_estimator = (
            OffsetEstimator(
                int(self._value('offset_estimation_samples')),
                queue_size=max(queue_size, 30))
            if bool(self._value('estimate_offset')) else None)
        self.sync_recovery_gate = ConsecutiveMatchGate(required_count=2)

        self.latest_rgb = None
        self.latest_depth = None
        self.latest_detection = None
        self.latest_detection_time = None
        self.detection_sequence = 0
        self.reference_last_sequence = -1
        self.latest_debug = None
        self.fx = 0.0
        self.rgb_time = None
        self.depth_time = None
        self.matched_depth_time = None
        self.rgb_stamp = None
        self.depth_stamp = None
        self.rgb_depth_stamp_delta = math.inf
        self.corrected_rgb_depth_stamp_delta = math.inf
        self.camera_info_time = None
        self.reference_samples = []
        self.reference_started = None
        self.last_state = self.runner.state
        self.sensor_state = SensorSyncState.SENSOR_SILENCE
        self.last_sensor_state = self.sensor_state
        self.last_sensor_log_time = -math.inf
        self.last_stats_time = self._now()
        self.sync_started = self._now()
        self.offset_estimate_reported = False
        self.graph_ready = False
        self.graph_stable_count = 0
        self.graph_gate_started = self._now()
        self.command_publisher = None
        self.shutdown_requested = False
        self.last_odom = None
        self.had_exception = False
        self.output_error = False
        self.csv_file = None
        self.csv_writer = None
        self.sync_csv_file = None
        self.sync_csv_writer = None

        rgb_topic = str(self._value('rgb_topic'))
        depth_topic = str(self._value('depth_topic'))
        info_topic = str(self._value('camera_info_topic'))
        self.cmd_vel_topic = str(self._value('cmd_vel_topic'))
        if self.cmd_vel_topic != '/cmd_vel':
            raise ValueError('cmd_vel_topic is fixed to /cmd_vel for graph safety')
        self.create_subscription(Image, rgb_topic, self._rgb_callback, 10)
        self.create_subscription(Image, depth_topic, self._depth_callback, 10)
        self.create_subscription(
            CameraInfo, info_topic, self._camera_info_callback, 10)
        ready_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.create_subscription(
            Bool, self.camera_pose_ready_topic,
            self._camera_pose_ready_callback, ready_qos)
        if bool(self._value('log_odom')):
            self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.debug_publisher = self.create_publisher(
            Image, '/red_marker_homing/debug_image', 10)
        self.state_publisher = self.create_publisher(
            String, '/red_marker_homing/state', 10)
        self.metrics_publisher = self.create_publisher(
            String, '/red_marker_homing/metrics', 10)
        self.timer = self.create_timer(1.0 / self.config.control_rate_hz, self._tick)
        self._open_outputs()
        self.get_logger().warning(
            'red_marker_homing_test is standalone; it does not start hardware. '
            f'dry_run={self.dry_run} confirmed={self.confirmed} '
            f'enable_base_motion={self.enable_base_motion} '
            f'sync_mode={self.sync_mode} compare={self.compare_sync_modes} '
            f'require_camera_pose_ready={self.require_camera_pose_ready}')

    def _declare_parameters(self):
        defaults = {
            'confirmed': False, 'dry_run': True,
            'enable_base_motion': False,
            'rgb_topic': '/depth_cam/rgb/image_raw',
            'depth_topic': '/depth_cam/depth/image_raw',
            'camera_info_topic': '/depth_cam/rgb/camera_info',
            'cmd_vel_topic': '/cmd_vel',
            'output_directory': '/tmp/red_marker_homing_test',
            'log_odom': False,
            'require_camera_pose_ready': True,
            'camera_pose_ready_topic': '/camera_pose_ready',
            'camera_pose_ready_timeout_sec': 0.0,
            'forward_distance_nominal': 0.25, 'forward_speed': 0.05,
            'turn_angle_deg': 30.0, 'turn_speed': 0.15,
            'yaw_kp': 0.8, 'yaw_max_speed': 0.08,
            'yaw_min_speed': 0.025, 'yaw_pixel_tolerance': 8.0,
            'depth_relative_tolerance': 0.03,
            'backup_speed': -0.03, 'backup_pulse_duration': 0.35,
            'backup_settle_duration': 0.50, 'segment_stop_duration': 0.50,
            'max_backup_pulses': 40, 'sensor_timeout': 0.5,
            'marker_lost_timeout': 0.5, 'total_timeout': 90.0,
            'zero_hold_duration': 2.0,
            'reference_collection_duration': 2.0,
            'min_reference_samples': 20,
            'reference_max_u_mad': 4.0,
            'reference_max_depth_mad_fraction': 0.02,
            'final_stable_duration': 1.0, 'state_timeout': 15.0,
            # Legacy names remain accepted for configuration compatibility.
            'sync_queue_size': 10,
            'max_rgb_depth_stamp_delta': 0.07,
            'sync_mode': DIRECT_SLOP,
            'direct_slop_ms': 70.0,
            'fixed_offset_slop_ms': 25.0,
            'depth_stamp_offset_ms': 0.0,
            'compare_sync_modes': True,
            'estimate_offset': True,
            'offset_estimation_samples': 200,
            'sync_buffer_max_age_ms': 1000.0,
            'depth_freshness_limit_ms': 150.0,
            'sensor_warn_timeout_ms': 1000.0,
            'sensor_stop_timeout_ms': 500.0,
            'sensor_log_throttle_ms': 2000.0,
            'sync_stats_period_sec': 5.0,
            'hsv_low_1': [0, 100, 60], 'hsv_high_1': [12, 255, 255],
            'hsv_low_2': [168, 100, 60], 'hsv_high_2': [179, 255, 255],
            'morphology_kernel_size': 5, 'morphology_iterations': 1,
            'depth_erode_kernel_size': 7, 'min_marker_area': 500.0,
            'max_marker_area': 120000.0, 'min_valid_depth_pixels': 50,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _value(self, name):
        return self.get_parameter(name).value

    def _homing_config(self):
        names = HomingConfig.__dataclass_fields__
        values = {
            name: self._value(name) for name in names if name != 'control_rate_hz'}
        return HomingConfig(**values)

    def _detector_config(self):
        values = {}
        for name in DetectorConfig.__dataclass_fields__:
            value = self._value(name)
            if name.startswith('hsv_'):
                value = tuple(int(item) for item in value)
            values[name] = value
        return DetectorConfig(**values)

    def _validate_sync_parameters(self):
        positive = (
            self.depth_freshness_limit_sec, self.sensor_warn_timeout_sec,
            self.sensor_stop_timeout_sec, self.sensor_log_throttle_sec,
            float(self._value('direct_slop_ms')),
            float(self._value('fixed_offset_slop_ms')),
            float(self._value('sync_buffer_max_age_ms')),
            float(self._value('sync_stats_period_sec')),
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError('sync slop, freshness, timeout and period must be positive')
        if self.sensor_stop_timeout_sec > self.config.sensor_timeout:
            raise ValueError('sensor_stop_timeout_ms cannot exceed sensor_timeout')
        if self.sensor_warn_timeout_sec < self.sensor_stop_timeout_sec:
            raise ValueError('sensor_warn_timeout_ms cannot be shorter than stop timeout')
        if int(self._value('offset_estimation_samples')) < 1:
            raise ValueError('offset_estimation_samples must be positive')

    def _open_outputs(self):
        try:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            self.csv_file = (self.output_dir / 'state_log.csv').open(
                'w', newline='', encoding='utf-8')
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=[
                'elapsed_time', 'state', 'sensor_state', 'depth_fresh',
                'camera_pose_ready', 'camera_pose_gate_ready',
                'camera_pose_ready_age',
                'marker_detected', 'u_current', 'u_ref', 'pixel_error',
                'angle_error', 'depth_current', 'depth_ref', 'depth_error',
                'area', 'bbox', 'valid_depth_count', 'backup_pulse_count',
                'decision_linear_x', 'decision_angular_z',
                'cmd_linear_x', 'cmd_angular_z', 'rgb_age', 'depth_age',
                'camera_info_age', 'odom_x', 'odom_y',
                'rgb_depth_stamp_delta', 'corrected_stamp_delta',
                'detection_age', 'detection_reason', 'detection_sequence',
            ])
            self.csv_writer.writeheader()
            self.sync_csv_file = (self.output_dir / 'sync_samples.csv').open(
                'w', newline='', encoding='utf-8')
            self.sync_csv_writer = csv.DictWriter(
                self.sync_csv_file, fieldnames=[
                    'wall_time', 'sync_mode', 'rgb_stamp_sec',
                    'depth_stamp_sec', 'raw_delta_ms',
                    'depth_stamp_offset_ms', 'corrected_delta_ms', 'matched',
                    'rgb_buffer_size', 'depth_buffer_size', 'depth_fresh',
                    'controller_state', 'reason',
                ])
            self.sync_csv_writer.writeheader()
        except OSError as error:
            self.output_error = True
            self.get_logger().error(
                f'OUTPUT_DISABLED: cannot open diagnostic files: {error}')
            self._close_file(self.csv_file)
            self._close_file(self.sync_csv_file)
            self.csv_file = self.csv_writer = None
            self.sync_csv_file = self.sync_csv_writer = None

    def _now(self):
        return time.monotonic()

    @staticmethod
    def _stamp_sec(message):
        return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def _rgb_callback(self, message):
        now = self._now()
        self.rgb_time = now
        stamp = self._stamp_sec(message)
        if self.offset_estimator is not None:
            self.offset_estimator.add_rgb(stamp)
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(message, 'rgb8')
            self.latest_rgb = rgb_image
        except (ValueError, cv2.error) as error:
            self.get_logger().error(f'RGB_CONVERSION_FAILED: {error}')
            return
        active_matched = self._feed_synchronizers('rgb', message, stamp, now)
        if not active_matched:
            self._detect_rgb_only(rgb_image)

    def _depth_callback(self, message):
        now = self._now()
        self.depth_time = now
        stamp = self._stamp_sec(message)
        if self.offset_estimator is not None:
            self.offset_estimator.add_depth(stamp)
        self._feed_synchronizers('depth', message, stamp, now)

    def _feed_synchronizers(self, kind, message, stamp, now):
        active_matched = False
        for mode, synchronizer in self.synchronizers.items():
            if kind == 'rgb':
                matches, samples = synchronizer.add_rgb(message, stamp, now)
            else:
                matches, samples = synchronizer.add_depth(message, stamp, now)
            self._write_sync_samples(samples, now)
            if mode == self.sync_mode:
                for match in matches:
                    active_matched = True
                    self._process_active_match(match, now)
        return active_matched

    def _process_active_match(self, match: SyncMatch, now: float):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(match.rgb.message, 'rgb8')
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                match.depth.message, 'passthrough')
        except (ValueError, cv2.error) as error:
            self.get_logger().error(f'RGBD_CONVERSION_FAILED: {error}')
            return
        self.rgb_stamp = match.rgb.stamp_sec
        self.depth_stamp = match.depth.stamp_sec
        self.rgb_depth_stamp_delta = match.raw_delta_ms / 1000.0
        self.corrected_rgb_depth_stamp_delta = match.corrected_delta_ms / 1000.0
        self.matched_depth_time = match.depth.arrival_time
        self.sync_recovery_gate.mark_match()
        self._detect(self.latest_rgb, self.latest_depth)

    def _detect_rgb_only(self, rgb_image):
        empty_depth = np.zeros(rgb_image.shape[:2], dtype=np.uint16)
        self._detect(rgb_image, empty_depth)

    def _detect(self, rgb_image, depth_image):
        try:
            self.latest_detection = detect_red_marker(
                rgb_image, depth_image, self.detector_config)
            self.latest_detection_time = self._now()
            self.detection_sequence += 1
            self.latest_debug = self._render_debug(self.latest_detection)
        except (ValueError, cv2.error) as error:
            self.get_logger().error(f'DETECTION_FAILED: {error}')
            self.latest_detection = MarkerDetection(reason='detector_error')
            self.latest_detection_time = self._now()
            self.detection_sequence += 1

    def _camera_info_callback(self, message):
        self.fx = float(message.k[0])
        self.camera_info_time = self._now()

    def _camera_pose_ready_callback(self, message):
        was_ready = self.camera_pose_ready
        self.camera_pose_ready = bool(message.data)
        self.camera_pose_ready_time = self._now()
        if was_ready and not self.camera_pose_ready:
            self._publish_command(Command())
            self.get_logger().warning(
                'CAMERA_POSE_READY_DROPPED: immediate zero command applied')

    def _camera_pose_gate_ready(self, now):
        if not self.require_camera_pose_ready:
            ready = True
        elif not self.camera_pose_ready:
            ready = False
        elif self.camera_pose_ready_timeout_sec == 0.0:
            ready = True
        else:
            ready = (
                self.camera_pose_ready_time is not None
                and now - self.camera_pose_ready_time
                <= self.camera_pose_ready_timeout_sec)
        if ready != self.last_camera_pose_gate:
            self.get_logger().warning(
                f'CAMERA_POSE_GATE_READY={ready}; '
                f'required={self.require_camera_pose_ready}')
            self.last_camera_pose_gate = ready
        return ready

    def _odom_callback(self, message):
        self.last_odom = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y))

    def _render_debug(self, detection):
        image = cv2.cvtColor(self.latest_rgb, cv2.COLOR_RGB2BGR)
        if detection.detected:
            x, y, width, height = detection.bbox
            color = (0, 255, 0) if detection.valid else (0, 165, 255)
            cv2.rectangle(image, (x, y), (x + width, y + height), color, 2)
            cv2.circle(image, (round(detection.u), round(detection.v)), 4, color, -1)
        cv2.putText(
            image, f'{self.runner.state.name} {self.sensor_state.value}', (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        return image

    def _age(self, stamp, now):
        return math.inf if stamp is None else max(0.0, now - stamp)

    def _update_sensor_state(self, now):
        self.sensor_state = classify_sensor_state(
            now, self.rgb_time, self.depth_time, self.matched_depth_time,
            depth_freshness_limit_sec=self.depth_freshness_limit_sec,
            sensor_stop_timeout_sec=self.sensor_stop_timeout_sec,
            sensor_warn_timeout_sec=self.sensor_warn_timeout_sec)
        if self.sensor_state is not SensorSyncState.SYNC_OK:
            self.sync_recovery_gate.mark_unhealthy()
        elif not self.sync_recovery_gate.ready:
            self.sensor_state = SensorSyncState.RGB_ONLY
        if self.sensor_state != self.last_sensor_state:
            self.get_logger().info(
                f'SENSOR_STATE {self.last_sensor_state.value} '
                f'-> {self.sensor_state.value}')
            self.last_sensor_state = self.sensor_state
        if (self.sensor_state is not SensorSyncState.SYNC_OK
                and now - self.last_sensor_log_time >= self.sensor_log_throttle_sec):
            self.get_logger().warning(
                f'{self.sensor_state.value}: holding unsafe motion; '
                'node remains active and will retry synchronization')
            self.last_sensor_log_time = now

    def _observation(self, now):
        detection = self.latest_detection
        if (self.latest_detection_time is None
                or now - self.latest_detection_time > self.config.marker_lost_timeout):
            detection = None
        depth_age = self._age(self.matched_depth_time, now)
        depth_fresh = (
            self.sensor_state is SensorSyncState.SYNC_OK
            and depth_age <= self.depth_freshness_limit_sec)
        return Observation(
            detection=detection, rgb_age=self._age(self.rgb_time, now),
            depth_age=depth_age,
            camera_info_age=self._age(self.camera_info_time, now), fx=self.fx,
            sensor_state=self.sensor_state.value, depth_fresh=depth_fresh)

    def _publisher_nodes(self, topic):
        return [
            f'/{info.node_namespace.strip("/")}/{info.node_name}'.replace('//', '/')
            for info in self.get_publishers_info_by_topic(topic)
            if info.node_name != self.get_name()
        ]

    def _graph_errors(self):
        publishers = {
            topic: self._publisher_nodes(topic)
            for topic in (self.cmd_vel_topic, '/controller/cmd_vel', '/cmd_vel_nav')
        }
        names = [name for name, _namespace in self.get_node_names_and_namespaces()]
        return graph_errors(publishers, names)

    def _update_graph_gate(self):
        if self.dry_run or not self.confirmed or not self.enable_base_motion:
            return True
        errors = list(self._graph_errors())
        if self.count_subscribers(self.cmd_vel_topic) < 1:
            errors.append('/cmd_vel must have at least one controller subscriber')
        if errors:
            self.graph_stable_count = 0
            if self.command_publisher is not None:
                self.get_logger().error('COMMAND_GRAPH_CONFLICT: ' + '; '.join(errors))
                return False
            if self._now() - self.graph_gate_started < 5.0:
                return True
            self.get_logger().error('COMMAND_GRAPH_REFUSED: ' + '; '.join(errors))
            return False
        self.graph_stable_count += 1
        if self.graph_stable_count >= 2 and self.command_publisher is None:
            self.command_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
            self.graph_ready = True
            self.get_logger().warning('COMMAND_GRAPH_READY: publisher created')
        return self.graph_ready

    def _collect_reference(self, now):
        if self.runner.state is not State.COLLECT_REFERENCE:
            self.reference_samples.clear()
            self.reference_started = None
            self.reference_last_sequence = -1
            return
        detection = self.latest_detection
        if (self.sensor_state is not SensorSyncState.SYNC_OK
                or detection is None or not detection.valid):
            self.reference_samples.clear()
            self.reference_started = None
            self.reference_last_sequence = -1
            return
        if self.detection_sequence == self.reference_last_sequence:
            return
        self.reference_last_sequence = self.detection_sequence
        if self.reference_started is None:
            self.reference_started = now
        self.reference_samples.append(detection)
        if (now - self.reference_started >= self.config.reference_collection_duration
                and len(self.reference_samples) >= int(
                    self._value('min_reference_samples'))):
            reference = build_reference(
                self.reference_samples, float(self._value('yaw_pixel_tolerance')),
                float(self._value('depth_relative_tolerance')))
            stable = (
                reference.u_mad <= float(self._value('reference_max_u_mad'))
                and reference.depth_mad <= reference.depth * float(
                    self._value('reference_max_depth_mad_fraction')))
            if not stable:
                self.get_logger().warning(
                    'REFERENCE_UNSTABLE: restarting continuous collection')
                self.reference_samples.clear()
                self.reference_started = now
                return
            self.runner.set_reference(reference)
            self._save_stage('reference')

    def _safe_command(
        self, command, observation, graph_safe, camera_pose_gate_ready,
    ):
        return apply_motion_safety_gate(
            command, dry_run=self.dry_run, confirmed=self.confirmed,
            enable_base_motion=self.enable_base_motion,
            graph_safe=graph_safe,
            require_camera_pose_ready=self.require_camera_pose_ready,
            camera_pose_ready=camera_pose_gate_ready,
            sensor_state=self.sensor_state.value,
            depth_fresh=observation.depth_fresh)

    def _publish_command(self, command):
        if command.linear_x and command.angular_z:
            raise RuntimeError('combined-axis command rejected')
        if self.command_publisher is not None:
            message = Twist()
            message.linear.x = command.linear_x
            message.angular.z = command.angular_z
            self.command_publisher.publish(message)
        return command

    def _tick(self):
        now = self._now()
        self._update_sensor_state(now)
        graph_safe = self._update_graph_gate()
        camera_pose_gate_ready = self._camera_pose_gate_ready(now)
        observation = self._observation(now)
        self._collect_reference(now)
        runner_confirmed = self.confirmed and (self.dry_run or graph_safe)
        runner_pose_gate = (
            camera_pose_gate_ready or self.dry_run
            or not self.require_camera_pose_ready)
        decision = self.runner.tick(
            now, observation, confirmed=runner_confirmed,
            graph_safe=(graph_safe or self.dry_run or not self.confirmed),
            motion_gate_ready=runner_pose_gate)
        safe_decision = self._safe_command(
            decision, observation, graph_safe, camera_pose_gate_ready)
        command = self._publish_command(safe_decision)
        self._publish_debug_and_metrics(
            now, observation, decision, command, camera_pose_gate_ready)
        self._handle_state_change()
        self._maybe_log_sync_stats(now)
        if self.runner.terminal:
            self.shutdown_requested = True

    def _publish_debug_and_metrics(
        self, now, observation, decision, command, camera_pose_gate_ready,
    ):
        if self.latest_debug is not None:
            self.debug_publisher.publish(
                self.bridge.cv2_to_imgmsg(self.latest_debug, encoding='bgr8'))
        detection = observation.detection or MarkerDetection()
        reference = self.runner.reference
        pixel_error = detection.u - reference.u if reference else math.nan
        angle_error = (
            math.atan(pixel_error / observation.fx)
            if reference and observation.fx > 0.0 else math.nan)
        depth_error = detection.depth - reference.depth if reference else math.nan
        elapsed = now - (self.runner.start_time or now)
        detection_age = (
            math.inf if self.latest_detection_time is None
            else now - self.latest_detection_time)
        row = {
            'elapsed_time': elapsed, 'state': self.runner.state.name,
            'sensor_state': self.sensor_state.value,
            'depth_fresh': observation.depth_fresh,
            'camera_pose_ready': self.camera_pose_ready,
            'camera_pose_gate_ready': camera_pose_gate_ready,
            'camera_pose_ready_age': self._age(
                self.camera_pose_ready_time, now),
            'marker_detected': detection.detected, 'u_current': detection.u,
            'u_ref': reference.u if reference else math.nan,
            'pixel_error': pixel_error, 'angle_error': angle_error,
            'depth_current': detection.depth,
            'depth_ref': reference.depth if reference else math.nan,
            'depth_error': depth_error, 'area': detection.area,
            'bbox': detection.bbox,
            'valid_depth_count': detection.valid_depth_count,
            'backup_pulse_count': self.runner.backup_pulse_count,
            'decision_linear_x': decision.linear_x,
            'decision_angular_z': decision.angular_z,
            'cmd_linear_x': command.linear_x, 'cmd_angular_z': command.angular_z,
            'rgb_age': observation.rgb_age, 'depth_age': observation.depth_age,
            'camera_info_age': observation.camera_info_age,
            'odom_x': self.last_odom[0] if self.last_odom else math.nan,
            'odom_y': self.last_odom[1] if self.last_odom else math.nan,
            'rgb_depth_stamp_delta': self.rgb_depth_stamp_delta,
            'corrected_stamp_delta': self.corrected_rgb_depth_stamp_delta,
            'detection_age': detection_age,
            'detection_reason': detection.reason,
            'detection_sequence': self.detection_sequence,
        }
        self._safe_csv_write(self.csv_writer, self.csv_file, row, 'state_log.csv')
        self.state_publisher.publish(String(data=self.runner.state.name))
        self.metrics_publisher.publish(String(data=json.dumps(row, default=str)))

    def _write_sync_samples(self, samples, now):
        depth_fresh = (
            self.matched_depth_time is not None
            and now - self.matched_depth_time <= self.depth_freshness_limit_sec)
        for sample in samples:
            row = {
                'wall_time': time.time(), 'sync_mode': sample.sync_mode,
                'rgb_stamp_sec': sample.rgb_stamp_sec,
                'depth_stamp_sec': sample.depth_stamp_sec,
                'raw_delta_ms': sample.raw_delta_ms,
                'depth_stamp_offset_ms': sample.depth_stamp_offset_ms,
                'corrected_delta_ms': sample.corrected_delta_ms,
                'matched': sample.matched,
                'rgb_buffer_size': sample.rgb_buffer_size,
                'depth_buffer_size': sample.depth_buffer_size,
                'depth_fresh': depth_fresh,
                'controller_state': self.runner.state.name,
                'reason': sample.reason,
            }
            self._safe_csv_write(
                self.sync_csv_writer, self.sync_csv_file, row,
                'sync_samples.csv')

    def _safe_csv_write(self, writer, file_object, row, label):
        if writer is None or file_object is None:
            return
        try:
            writer.writerow(row)
            file_object.flush()
        except OSError as error:
            self.output_error = True
            self.get_logger().error(f'OUTPUT_WRITE_FAILED {label}: {error}')

    def _maybe_log_sync_stats(self, now):
        period = float(self._value('sync_stats_period_sec'))
        if now - self.last_stats_time < period:
            return
        summaries = {
            mode: synchronizer.summary()
            for mode, synchronizer in self.synchronizers.items()}
        compact = '; '.join(
            f'{mode}: matched={item["matched_count"]} '
            f'rate={item["match_rate"]:.3f} '
            f'median_abs_ms={item["median_abs_delta_ms"]}'
            for mode, item in summaries.items())
        self.get_logger().info('SYNC_STATS ' + compact)
        self.last_stats_time = now
        if (self.offset_estimator is not None and not self.offset_estimate_reported
                and len(self.offset_estimator.raw_deltas_ms)
                >= int(self._value('offset_estimation_samples'))):
            estimate = self.offset_estimator.estimated_depth_stamp_offset_ms
            self.get_logger().info(
                f'estimated_depth_stamp_offset_ms={estimate:.3f}; '
                'diagnostic only, active offset unchanged')
            self.offset_estimate_reported = True

    def _handle_state_change(self):
        state = self.runner.state
        if state is self.last_state:
            return
        self.get_logger().info(f'STATE {self.last_state.name} -> {state.name}')
        names = {
            State.STOP_AFTER_FORWARD: 'after_forward',
            State.STOP_AFTER_LEFT: 'after_left',
            State.STOP_AFTER_RIGHT: 'after_right',
            State.SUCCESS: 'final', State.FINAL_STOP: 'final',
        }
        if state in names:
            self._save_stage(names[state])
        self.last_state = state

    def _save_stage(self, name):
        if self.latest_rgb is None or self.latest_depth is None:
            return
        try:
            cv2.imwrite(
                str(self.output_dir / f'{name}_rgb.png'),
                cv2.cvtColor(self.latest_rgb, cv2.COLOR_RGB2BGR))
            cv2.imwrite(
                str(self.output_dir / f'{name}_depth_preview.png'),
                depth_preview(self.latest_depth))
            if self.latest_depth.dtype == 'uint16':
                cv2.imwrite(
                    str(self.output_dir / f'{name}_depth_raw_16uc1.png'),
                    self.latest_depth)
            if self.latest_debug is not None:
                cv2.imwrite(
                    str(self.output_dir / f'{name}_debug.png'), self.latest_debug)
        except (OSError, cv2.error) as error:
            self.output_error = True
            self.get_logger().error(f'IMAGE_WRITE_FAILED: {error}')

    def request_stop(self, signum):
        self.get_logger().warning(f'SIGNAL {signum}: requesting zero cleanup')
        self.shutdown_requested = True

    def zero_cleanup(self):
        """Publish zero at 20 Hz for the reviewed minimum tail duration."""
        deadline = time.monotonic() + self.config.zero_hold_duration
        while time.monotonic() <= deadline:
            self._publish_command(Command())
            time.sleep(1.0 / self.config.control_rate_hz)

    def _sync_summary(self):
        estimate = (
            self.offset_estimator.estimated_depth_stamp_offset_ms
            if self.offset_estimator is not None else None)
        return {
            'parameters': {
                name: self._value(name) for name in (
                    'sync_mode', 'direct_slop_ms', 'fixed_offset_slop_ms',
                    'depth_stamp_offset_ms', 'compare_sync_modes',
                    'estimate_offset', 'offset_estimation_samples',
                    'sync_queue_size', 'sync_buffer_max_age_ms',
                    'depth_freshness_limit_ms', 'sensor_warn_timeout_ms',
                    'sensor_stop_timeout_ms', 'sensor_log_throttle_ms',
                    'sync_stats_period_sec')},
            'modes': {
                mode: synchronizer.summary()
                for mode, synchronizer in self.synchronizers.items()},
            'estimated_depth_stamp_offset_ms': estimate,
            'offset_estimation_sample_count': (
                len(self.offset_estimator.raw_deltas_ms)
                if self.offset_estimator is not None else 0),
            'duration_sec': self._now() - self.sync_started,
            'had_exception': self.had_exception,
            'output_error': self.output_error,
            'final_state': self.runner.state.name,
            'final_sensor_state': self.sensor_state.value,
        }

    def close_outputs(self):
        reference = self.runner.reference
        summary = {
            'state': self.runner.state.name, 'reason': self.runner.reason,
            'dry_run': self.dry_run, 'confirmed': self.confirmed,
            'enable_base_motion': self.enable_base_motion,
            'require_camera_pose_ready': self.require_camera_pose_ready,
            'camera_pose_ready': self.camera_pose_ready,
            'camera_pose_gate_ready': self.last_camera_pose_gate,
            'backup_pulse_count': self.runner.backup_pulse_count,
            'goal_claim': ('camera-relative marker distance and horizontal bearing; '
                           'not a unique global x/y/yaw pose'),
            'reference': reference.__dict__ if reference else None,
        }
        self._safe_json_write(self.output_dir / 'summary.json', summary)
        self._safe_json_write(
            self.output_dir / 'sync_summary.json', self._sync_summary())
        self._close_file(self.csv_file)
        self._close_file(self.sync_csv_file)

    def _safe_json_write(self, path, data):
        try:
            path.write_text(json.dumps(data, indent=2), encoding='utf-8')
        except OSError as error:
            self.output_error = True
            self.get_logger().error(f'OUTPUT_WRITE_FAILED {path.name}: {error}')

    @staticmethod
    def _close_file(file_object):
        if file_object is None:
            return
        try:
            file_object.flush()
            file_object.close()
        except OSError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RedMarkerHomingNode()

    def handle_signal(signum, _frame):
        node.request_stop(signum)

    previous_handlers = {
        signum: signal.signal(signum, handle_signal)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except BaseException:
        node.had_exception = True
        raise
    finally:
        node.zero_cleanup()
        node.close_outputs()
        node.destroy_node()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
