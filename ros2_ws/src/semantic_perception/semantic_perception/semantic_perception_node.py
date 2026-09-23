"""ROS2 node that publishes fake semantic perception results."""

import json

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from semantic_perception.mask_generator import (
    calculate_class_ratios,
    generate_fake_mask,
)


DEFAULT_INPUT_TOPIC = '/camera/color/image_raw'
MASK_TOPIC = '/semantic/mask'
INFO_TOPIC = '/semantic/info'


class FakeSemanticNode(Node):
    """Convert each RGB input and publish a random semantic mask and ratios."""

    def __init__(self) -> None:
        """Initialize the Phase 0 semantic perception interfaces."""
        super().__init__('fake_semantic_node')

        self.declare_parameter('input_topic', DEFAULT_INPUT_TOPIC)
        input_topic = self.get_parameter('input_topic').value
        if not isinstance(input_topic, str) or not input_topic:
            raise ValueError('input_topic must be a non-empty string')

        self._bridge = CvBridge()
        self._rng = np.random.default_rng()
        self._mask_publisher = self.create_publisher(
            Image,
            MASK_TOPIC,
            qos_profile_sensor_data,
        )
        self._info_publisher = self.create_publisher(String, INFO_TOPIC, 10)
        self._image_subscription = self.create_subscription(
            Image,
            input_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Fake semantic perception ready; input={input_topic}, '
            f'mask={MASK_TOPIC}, info={INFO_TOPIC}'
        )

    def _image_callback(self, message: Image) -> None:
        """Publish one fake semantic result for one input image."""
        try:
            cv_image = self._bridge.imgmsg_to_cv2(
                message,
                desired_encoding='passthrough',
            )
        except CvBridgeError as error:
            self.get_logger().error(f'Failed to convert input image: {error}')
            return

        if cv_image.ndim < 2:
            self.get_logger().error('Converted input image has fewer than 2 dimensions')
            return

        height, width = cv_image.shape[:2]
        mask = generate_fake_mask(height, width, self._rng)

        mask_message = self._bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_message.header = message.header
        self._mask_publisher.publish(mask_message)

        ratios = calculate_class_ratios(mask)
        info_message = String()
        info_message.data = json.dumps(
            {name: round(value, 6) for name, value in ratios.items()},
            separators=(',', ':'),
        )
        self._info_publisher.publish(info_message)


def main(args=None) -> None:
    """Run the fake semantic perception node."""
    rclpy.init(args=args)
    node = FakeSemanticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
