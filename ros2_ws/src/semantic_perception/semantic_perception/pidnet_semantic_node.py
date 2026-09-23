"""ROS2 node that publishes PIDNet-S hazard5 semantic segmentation."""

from collections import deque
import json
import time

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String

from semantic_perception.pidnet_contract import (
    calculate_ratios,
    colorize_prediction,
    make_overlay,
)
from semantic_perception.pidnet_inference import PIDNetInference


SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
MASK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
    # 语义 mask 是 ~10-30Hz 图像流, 用 BEST_EFFORT 发布: 慢消费端(如 fusion/
    # rviz)跟不上时丢弃新帧而不是阻塞 pidnet 主循环。下游 fusion 已按 BEST_EFFORT 订阅。
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class PIDNetSemanticNode(Node):
    """Subscribe to RGB and publish real hazard5 masks and diagnostics."""

    def __init__(self) -> None:
        """Load the model and create the isolated semantic interfaces."""
        super().__init__('pidnet_semantic_node')
        self.declare_parameter('model_path', '')
        self.declare_parameter('input_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('mask_topic', '/semantic/mask')
        self.declare_parameter('info_topic', '/semantic/info')
        self.declare_parameter('color_topic', '/semantic/color')
        self.declare_parameter('overlay_topic', '/semantic/overlay')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('precision', 'fp16')
        self.declare_parameter('allow_fp32_fallback', True)
        # 跳帧: 相机 30fps, 只处理每 N 帧接收的 RGB, 其余直接丢弃, 降 CPU。
        # mask 是"最新值缓存"型消费 (fusion 只用最新 mask, 不与点云时间同步),
        # N=2 → ~15fps, N=3 → ~10fps, 对障碍物图层无影响。
        self.declare_parameter('process_every_n', 2)
        self.declare_parameter('publish_debug_images', True)
        # overlay 每帧生成 45% 权重混叠图, CPU 开销接近一次推理, 且无下游订阅。
        # 默认砍掉; 需要人眼可视化时再显式打开 (publish_debug_images 只管 color)。
        self.declare_parameter('publish_overlay', False)
        self.declare_parameter('overlay_raw_weight', 0.55)
        self.declare_parameter('log_every_n', 30)

        model_path = self._string_parameter('model_path')
        if not model_path:
            raise ValueError('model_path is required')
        input_topic = self._string_parameter('input_topic')
        mask_topic = self._string_parameter('mask_topic')
        info_topic = self._string_parameter('info_topic')
        color_topic = self._string_parameter('color_topic')
        overlay_topic = self._string_parameter('overlay_topic')
        device = self._string_parameter('device')
        precision = self._string_parameter('precision')
        allow_fallback = bool(
            self.get_parameter('allow_fp32_fallback').value)
        self._publish_debug = bool(
            self.get_parameter('publish_debug_images').value)
        self._publish_overlay = bool(
            self.get_parameter('publish_overlay').value)
        self._overlay_raw_weight = float(
            self.get_parameter('overlay_raw_weight').value)
        self._log_every_n = int(self.get_parameter('log_every_n').value)
        if self._log_every_n <= 0:
            raise ValueError('log_every_n must be positive')
        self._process_every_n = int(self.get_parameter('process_every_n').value)
        if self._process_every_n < 1:
            raise ValueError('process_every_n must be positive')
        self._received_frames = 0

        self._bridge = CvBridge()
        self._engine = PIDNetInference(
            model_path=model_path,
            device=device,
            precision=precision,
            allow_fp32_fallback=allow_fallback,
        )
        self._mask_publisher = self.create_publisher(
            Image, mask_topic, MASK_QOS)
        self._info_publisher = self.create_publisher(String, info_topic, 10)
        self._color_publisher = None
        self._overlay_publisher = None
        if self._publish_debug:
            self._color_publisher = self.create_publisher(
                Image, color_topic, SENSOR_QOS)
            if self._publish_overlay:
                self._overlay_publisher = self.create_publisher(
                    Image, overlay_topic, SENSOR_QOS)
        self._image_subscription = self.create_subscription(
            Image, input_topic, self._image_callback, SENSOR_QOS)

        self._sequence = 0
        self._output_times = deque(maxlen=31)
        self._reported_fallback = False
        metadata = self._engine.metadata
        self.get_logger().info(
            'PIDNet-S hazard5 ready; input=%s mask=%s info=%s '
            'checkpoint_epoch=%s val_miou=%s loaded=%s precision=%s '
            'debug=%s overlay=%s skip_every=%s'
            % (
                input_topic,
                mask_topic,
                info_topic,
                metadata['epoch'],
                metadata['val_miou'],
                metadata['loaded_tensors'],
                metadata['precision'],
                self._publish_debug,
                self._publish_overlay,
                self._process_every_n,
            )
        )

    def _string_parameter(self, name: str) -> str:
        value = self.get_parameter(name).value
        if not isinstance(value, str):
            raise ValueError(f'{name} must be a string')
        return value

    def _image_callback(self, message: Image) -> None:
        """Infer and publish one semantic result for one RGB input."""
        # 跳帧: 只处理每 N 帧接收的 RGB, 其余直接丢弃。
        # 频率突变对下游无影响: 语义 mask 是"最新值缓存"型消费 (fusion 的
        # mask_cb 只存最新 mask, 点云投影的 TF 查的是点云自己的 stamp, 不查
        # mask 的 stamp), 慢速更不易丢帧 (BEST_EFFORT producer 慢于 consumer)。
        self._received_frames += 1
        if self._received_frames % self._process_every_n != 0:
            return
        callback_started = time.perf_counter()
        try:
            rgb = self._bridge.imgmsg_to_cv2(
                message, desired_encoding='rgb8')
            mask, inference_ms = self._engine.predict(rgb)
        except (CvBridgeError, RuntimeError, ValueError) as error:
            self.get_logger().error(
                f'PIDNet frame failed; no mask published: {type(error).__name__}: {error}')
            return

        if self._engine.last_fallback_reason and not self._reported_fallback:
            self._reported_fallback = True
            self.get_logger().warn(
                'FP16 inference failed; switched permanently to FP32: '
                f'{self._engine.last_fallback_reason}')

        mask_message = self._bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_message.header = message.header
        self._mask_publisher.publish(mask_message)

        if self._publish_debug:
            color = colorize_prediction(mask)
            color_message = self._bridge.cv2_to_imgmsg(color, encoding='rgb8')
            color_message.header = message.header
            self._color_publisher.publish(color_message)
            # overlay 默认砍掉 (publish_overlay=false): 混叠是每帧最贵的 CPU 步骤
            # (5 个 float32 大数组), 且无下游订阅。需要可视化时显式打开。
            if self._publish_overlay:
                overlay = make_overlay(rgb, mask, self._overlay_raw_weight)
                overlay_message = self._bridge.cv2_to_imgmsg(
                    overlay, encoding='rgb8')
                overlay_message.header = message.header
                self._overlay_publisher.publish(overlay_message)

        now = time.perf_counter()
        self._output_times.append(now)
        output_fps = 0.0
        if len(self._output_times) >= 2:
            output_fps = (
                (len(self._output_times) - 1)
                / (self._output_times[-1] - self._output_times[0])
            )
        callback_ms = (now - callback_started) * 1000.0
        self._sequence += 1
        info = calculate_ratios(mask)
        info.update({
            'inference_ms': round(inference_ms, 3),
            'callback_ms': round(callback_ms, 3),
            'output_fps': round(output_fps, 3),
            'precision': self._engine.precision,
            'sequence': self._sequence,
            'width': int(mask.shape[1]),
            'height': int(mask.shape[0]),
        })
        info_message = String()
        info_message.data = json.dumps(info, separators=(',', ':'))
        self._info_publisher.publish(info_message)

        if self._sequence == 1 or self._sequence % self._log_every_n == 0:
            self.get_logger().info(
                'frame=%d inference_ms=%.3f callback_ms=%.3f fps=%.3f '
                'precision=%s'
                % (
                    self._sequence,
                    inference_ms,
                    callback_ms,
                    output_fps,
                    self._engine.precision,
                )
            )


def main(args=None) -> None:
    """Run the PIDNet-S semantic node."""
    rclpy.init(args=args)
    node = None
    try:
        node = PIDNetSemanticNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
