"""ROS-independent helpers for stable DDS graph discovery."""

from dataclasses import dataclass


REQUIRED_GRAPH_NODES = (
    '/ros_robot_controller',
    '/odom_publisher',
)
GRAPH_DISCOVERY_TIMEOUT_SEC = 5.0
GRAPH_DISCOVERY_STABLE_SNAPSHOTS = 2


def normalize_node_name(name: str, namespace: str) -> str:
    """Return a canonical fully-qualified ROS node name."""
    clean_name = name.strip('/')
    clean_namespace = namespace.strip('/')
    if clean_namespace:
        return f'/{clean_namespace}/{clean_name}'
    return f'/{clean_name}'


def normalize_snapshot(
    nodes: list[tuple[str, str]] | tuple[tuple[str, str], ...],
) -> tuple[str, ...]:
    """Normalize and sort a graph snapshot while preserving duplicates."""
    return tuple(sorted(normalize_node_name(name, namespace) for name, namespace in nodes))


def required_nodes_exactly_once(nodes: tuple[str, ...]) -> bool:
    """Require exactly one instance of each controller-chain node."""
    return all(nodes.count(required) == 1 for required in REQUIRED_GRAPH_NODES)


@dataclass
class StableDiscovery:
    """Track consecutive satisfactory graph snapshots."""

    consecutive_ready: int = 0

    def observe(self, nodes: tuple[str, ...]) -> bool:
        if required_nodes_exactly_once(nodes):
            self.consecutive_ready += 1
        else:
            self.consecutive_ready = 0
        return self.consecutive_ready >= GRAPH_DISCOVERY_STABLE_SNAPSHOTS
