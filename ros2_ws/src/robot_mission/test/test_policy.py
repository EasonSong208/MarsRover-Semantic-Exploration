"""Unit tests for ROS-independent readiness policy."""

from robot_mission.policy import (
    CheckResult,
    Severity,
    check_topic,
    check_unique_node,
    summarize,
    warn_publishers,
)


ODOMETRY = 'nav_msgs/msg/Odometry'


def test_topic_passes_with_expected_type_and_single_publisher():
    result = check_topic('/odom', [ODOMETRY], ODOMETRY, ['/ekf_filter_node'], 1)
    assert result.severity == Severity.PASS


def test_topic_fails_when_publisher_is_duplicated():
    result = check_topic(
        '/odom', [ODOMETRY], ODOMETRY,
        ['/ekf_filter_node', '/robot/ekf_filter_node'], 1,
    )
    assert result.severity == Severity.FAIL
    assert 'found 2' in result.detail


def test_unique_node_rejects_duplicates():
    result = check_unique_node(
        'odom_publisher', ['/odom_publisher', '/robot/odom_publisher'])
    assert result.severity == Severity.FAIL


def test_direct_control_publishers_are_an_explicit_warning():
    result = warn_publishers('/controller/cmd_vel', ['/joystick_control'])
    assert result.severity == Severity.WARN


def test_summary_never_authorizes_motion():
    severity, message = summarize([
        CheckResult(Severity.PASS, 'graph', 'ok'),
    ])
    assert severity == Severity.PASS
    assert 'USER CONFIRMATION' in message


def test_failure_blocks_motion_readiness():
    severity, message = summarize([
        CheckResult(Severity.FAIL, 'tf', 'missing'),
    ])
    assert severity == Severity.FAIL
    assert message == 'NOT READY FOR MOTION'
