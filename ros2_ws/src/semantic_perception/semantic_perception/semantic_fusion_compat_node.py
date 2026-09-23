"""Project-owned compatibility entry for the existing semantic fusion node."""

import rclpy
from sensor_msgs_py import point_cloud2

from navigation.semantic_obstacle_fusion import SemanticObstacleFusion
from semantic_perception.pidnet_pointcloud_compat import coerce_xyz


class SemanticFusionCompatNode(SemanticObstacleFusion):
    """Reuse the existing backend with Humble PointCloud2 array compatibility."""

    def _extract_xyz(self, message):
        """Accept both structured XYZ records and plain Nx3 NumPy arrays."""
        try:
            points = point_cloud2.read_points_numpy(
                message, field_names=('x', 'y', 'z'))
            return coerce_xyz(points)
        except Exception as error:
            if not self._warned_cloud_parse:
                self._warned_cloud_parse = True
                self.get_logger().warn(f'point cloud compatibility failed: {error}')
            return None


def main(args=None) -> None:
    """Run the existing backend with only its XYZ conversion adapted."""
    rclpy.init(args=args)
    node = None
    try:
        node = SemanticFusionCompatNode()
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
