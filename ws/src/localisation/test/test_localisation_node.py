
# ws/src/localisation/test/test_localisation_node.py

import math
from unittest.mock import Mock

import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from localisation import localisation_node as node_module
from localisation.localisation_node import LocalisationNode, pose_covariance
from localisation.scan_matcher import ScanMatchResult


ESTIMATED_POSE = (0.12, -1.10, math.radians(87.0))


# Create an isolated ROS node with mocked publishers and subscriptions
@pytest.fixture
def node_harness(monkeypatch):
    publishers = {}
    subscriptions = {}

    def create_publisher(self, message_type, topic, qos):
        publisher = Mock()
        publishers[topic] = (message_type, publisher, qos)
        return publisher

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = Mock()
        subscriptions[topic] = (
            message_type,
            callback,
            qos,
        )
        return subscription

    monkeypatch.setattr(
        LocalisationNode,
        'create_publisher',
        create_publisher,
    )
    monkeypatch.setattr(
        LocalisationNode,
        'create_subscription',
        create_subscription,
    )

    was_running = rclpy.ok()

    if not was_running:
        rclpy.init()

    node = LocalisationNode()

    try:
        yield node, publishers, subscriptions
    finally:
        node.destroy_node()

        if not was_running:
            rclpy.shutdown()


# Create a synthetic LaserScan message with realistic metadata
def _make_scan(frame_id='laser_frame'):
    scan = LaserScan()

    scan.header.frame_id = frame_id
    scan.header.stamp.sec = 123
    scan.header.stamp.nanosec = 456789

    scan.angle_min = -math.pi
    scan.angle_increment = 2.0 * math.pi / 360
    scan.angle_max = (
        scan.angle_min + 359 * scan.angle_increment
    )

    scan.range_min = 0.05
    scan.range_max = 12.0
    scan.ranges = [1.0] * 360

    return scan


# Build a result matching the scan matcher's return type
def _make_result(
    pose=ESTIMATED_POSE,
    valid=True,
    reason='ok',
):
    return ScanMatchResult(
        pose=pose,
        rms_error_m=0.01 if valid else math.inf,
        inlier_count=100 if valid else 0,
        wall_inliers={
            'home': 0,
            'left': 30 if valid else 0,
            'opposite': 40 if valid else 0,
            'right': 30 if valid else 0,
        },
        iterations=5,
        valid=valid,
        reason=reason,
    )


# Check the expected ROS interfaces and message types
def test_ros_interfaces(node_harness):
    node, publishers, subscriptions = node_harness

    assert '/localisation/pose' in publishers
    assert '/localisation/valid' in publishers
    assert '/scan' in subscriptions

    assert publishers['/localisation/pose'][0] is PoseWithCovarianceStamped
    assert publishers['/localisation/valid'][0] is Bool
    assert subscriptions['/scan'][0] is LaserScan

    assert subscriptions['/scan'][1] == node.on_scan

    # The node must not subscribe to odometry or IMU yet
    assert '/odom' not in subscriptions
    assert '/imu' not in subscriptions


# Check the node starts from the configured home-wall prior
def test_initial_pose(node_harness):
    node, _, _ = node_harness

    expected = node.arena.nominal_start_pose(
        node.footprint[0]
    )

    assert node.initial_pose == pytest.approx(expected)
    assert node.home_target == pytest.approx(expected)
    assert node.previous_pose is None
    assert node.arena.frame_id == 'map'


# Confirm the initial seeds cover the permitted lateral starting range
def test_initial_lateral_seeds(node_harness):
    node, _, _ = node_harness

    lower, upper = node.arena.start_lateral_bounds(
        node.footprint[1]
    )

    lateral_positions = [
        node.arena.pose_to_arena(*pose)[1]
        for pose in node.initial_seeds
    ]

    assert len(lateral_positions) == 4
    assert min(lateral_positions) == pytest.approx(lower)
    assert max(lateral_positions) == pytest.approx(upper)

    for pose in node.initial_seeds:
        assert pose[2] == pytest.approx(node.initial_pose[2])


# Check the first scan uses the home-wall prior and alternative guesses
def test_first_scan_initialisation(node_harness, monkeypatch):
    node, _, _ = node_harness
    calls = []

    def fake_match_scan(**kwargs):
        calls.append(kwargs)
        return _make_result()

    monkeypatch.setattr(
        node_module,
        'match_scan',
        fake_match_scan,
    )

    node.on_scan(_make_scan())

    assert len(calls) == 1
    assert calls[0]['initial_pose'] == node.initial_pose
    assert calls[0]['alternate_initial_poses'] == node.initial_seeds
    assert calls[0]['walls'] == node.walls
    assert calls[0]['config'] == node.config

    assert node.previous_pose == ESTIMATED_POSE


# Verify LiDAR preprocessing applies the configured offset and yaw
def test_lidar_preprocessing(node_harness, monkeypatch):
    node, _, _ = node_harness
    captured = {}

    def fake_match_scan(**kwargs):
        captured.update(kwargs)
        return _make_result()

    monkeypatch.setattr(
        node_module,
        'match_scan',
        fake_match_scan,
    )

    node.on_scan(_make_scan())

    points = captured['points']

    assert len(points) > 0
    assert len(points) < 360

    # LiDAR beam +90 degrees points along robot-forward
    # Its position is 20 mm ahead of base_footprint
    assert any(
        abs(x - 1.020) < 1e-6 and abs(y) < 1e-6
        for x, y in points
    )


# Check a valid match publishes the global pose and correct timestamp
def test_valid_pose_published(node_harness, monkeypatch):
    node, publishers, _ = node_harness

    monkeypatch.setattr(
        node_module,
        'match_scan',
        lambda **kwargs: _make_result(),
    )

    scan = _make_scan()
    node.on_scan(scan)

    pose_publisher = publishers['/localisation/pose'][1]
    valid_publisher = publishers['/localisation/valid'][1]

    pose_publisher.publish.assert_called_once()
    valid_publisher.publish.assert_called_once()

    message = pose_publisher.publish.call_args.args[0]
    validity = valid_publisher.publish.call_args.args[0]

    assert isinstance(message, PoseWithCovarianceStamped)
    assert message.header.frame_id == 'map'

    assert message.header.stamp.sec == scan.header.stamp.sec
    assert message.header.stamp.nanosec == scan.header.stamp.nanosec

    assert message.pose.pose.position.x == pytest.approx(
        ESTIMATED_POSE[0]
    )
    assert message.pose.pose.position.y == pytest.approx(
        ESTIMATED_POSE[1]
    )
    assert message.pose.pose.position.z == 0.0

    assert message.pose.pose.orientation.z == pytest.approx(
        math.sin(ESTIMATED_POSE[2] / 2.0)
    )
    assert message.pose.pose.orientation.w == pytest.approx(
        math.cos(ESTIMATED_POSE[2] / 2.0)
    )

    assert validity.data is True


# Check the covariance follows the map's configured forward direction
def test_pose_covariance(node_harness):
    node, _, _ = node_harness

    covariance = pose_covariance(
        node.arena,
        node.config,
    )

    assert len(covariance) == 36

    # For forward_axis=y, lateral uncertainty lies along map x
    assert covariance[0] == pytest.approx(0.05 ** 2)
    assert covariance[7] == pytest.approx(0.02 ** 2)

    assert covariance[1] == pytest.approx(0.0)
    assert covariance[6] == pytest.approx(0.0)

    assert covariance[35] == pytest.approx(
        math.radians(5.0) ** 2
    )

    # Unobserved height, roll and pitch have large variances
    assert covariance[14] == 1e6
    assert covariance[21] == 1e6
    assert covariance[28] == 1e6


# Use the previously estimated pose for subsequent scans
def test_previous_pose_reused(node_harness, monkeypatch):
    node, _, _ = node_harness
    calls = []

    def fake_match_scan(**kwargs):
        calls.append(kwargs)
        return _make_result()

    monkeypatch.setattr(
        node_module,
        'match_scan',
        fake_match_scan,
    )

    node.on_scan(_make_scan())
    node.on_scan(_make_scan())

    assert len(calls) == 2

    assert calls[0]['initial_pose'] == node.initial_pose
    assert calls[1]['initial_pose'] == ESTIMATED_POSE

    assert calls[1]['alternate_initial_poses'] == ()


# Reject a failed match without publishing a new pose
def test_failed_match(node_harness, monkeypatch):
    node, publishers, _ = node_harness

    monkeypatch.setattr(
        node_module,
        'match_scan',
        lambda **kwargs: _make_result(
            valid=False,
            reason='insufficient wall support',
        ),
    )

    node.on_scan(_make_scan())

    pose_publisher = publishers['/localisation/pose'][1]
    valid_publisher = publishers['/localisation/valid'][1]

    pose_publisher.publish.assert_not_called()
    valid_publisher.publish.assert_called_once()

    message = valid_publisher.publish.call_args.args[0]

    assert message.data is False
    assert node.previous_pose is None


# Retain the last valid pose when a later scan cannot be matched
def test_last_valid_pose_retained(node_harness, monkeypatch):
    node, publishers, _ = node_harness
    calls = []

    results = iter([
        _make_result(),
        _make_result(valid=False, reason='not enough walls'),
        _make_result(),
    ])

    def fake_match_scan(**kwargs):
        calls.append(kwargs)
        return next(results)

    monkeypatch.setattr(
        node_module,
        'match_scan',
        fake_match_scan,
    )

    node.on_scan(_make_scan())

    assert node.previous_pose == ESTIMATED_POSE

    node.on_scan(_make_scan())

    assert node.previous_pose == ESTIMATED_POSE

    # A failed scan must not overwrite the last successful estimate
    pose_publisher = publishers['/localisation/pose'][1]
    assert pose_publisher.publish.call_count == 1

    node.on_scan(_make_scan())

    assert calls[2]['initial_pose'] == ESTIMATED_POSE
    assert pose_publisher.publish.call_count == 2


# Reject scans with an unexpected coordinate frame
def test_incorrect_scan_frame(node_harness, monkeypatch):
    node, publishers, _ = node_harness

    matcher = Mock()
    monkeypatch.setattr(
        node_module,
        'match_scan',
        matcher,
    )

    node.on_scan(_make_scan(frame_id='wrong_lidar_frame'))

    matcher.assert_not_called()

    publishers['/localisation/pose'][1].publish.assert_not_called()

    validity = (
        publishers['/localisation/valid'][1]
        .publish.call_args.args[0]
    )

    assert validity.data is False


# Accept the same scan frame with an optional leading slash
def test_scan_frame_with_slash(node_harness, monkeypatch):
    node, publishers, _ = node_harness

    matcher = Mock(return_value=_make_result())

    monkeypatch.setattr(
        node_module,
        'match_scan',
        matcher,
    )

    node.on_scan(_make_scan(frame_id='/laser_frame'))

    matcher.assert_called_once()
    publishers['/localisation/pose'][1].publish.assert_called_once()


# Reject invalid scan metadata without calling the optimiser
def test_invalid_scan_metadata(node_harness, monkeypatch):
    node, publishers, _ = node_harness

    matcher = Mock()
    monkeypatch.setattr(
        node_module,
        'match_scan',
        matcher,
    )

    scan = _make_scan()
    scan.angle_increment = 0.0

    node.on_scan(scan)

    matcher.assert_not_called()

    publishers['/localisation/pose'][1].publish.assert_not_called()

    validity = (
        publishers['/localisation/valid'][1]
        .publish.call_args.args[0]
    )

    assert validity.data is False


# Ensure invalid or missing ranges are passed to the matcher as no points
def test_invalid_ranges(node_harness, monkeypatch):
    node, publishers, _ = node_harness

    calls = []

    def fake_match_scan(**kwargs):
        calls.append(kwargs)
        return _make_result(
            valid=False,
            reason='no scan points',
        )

    monkeypatch.setattr(
        node_module,
        'match_scan',
        fake_match_scan,
    )

    scan = _make_scan()
    scan.ranges = [math.inf, math.nan, 0.0] * 120

    node.on_scan(scan)

    assert len(calls) == 1
    assert calls[0]['points'] == []

    publishers['/localisation/pose'][1].publish.assert_not_called()

    validity = (
        publishers['/localisation/valid'][1]
        .publish.call_args.args[0]
    )

    assert validity.data is False


# Handle recoverable preprocessing and matching exceptions
@pytest.mark.parametrize(
    'error',
    [ValueError('invalid scan'), ArithmeticError('numerical failure')],
)
def test_matching_exception(node_harness, monkeypatch, error):
    node, publishers, _ = node_harness

    def failing_matcher(**kwargs):
        raise error

    monkeypatch.setattr(
        node_module,
        'match_scan',
        failing_matcher,
    )

    node.on_scan(_make_scan())

    publishers['/localisation/pose'][1].publish.assert_not_called()

    validity = (
        publishers['/localisation/valid'][1]
        .publish.call_args.args[0]
    )

    assert validity.data is False
    assert node.previous_pose is None


# Check each processed scan publishes its current validity
def test_validity_sequence(node_harness, monkeypatch):
    node, publishers, _ = node_harness

    results = iter([
        _make_result(valid=True),
        _make_result(valid=False),
        _make_result(valid=True),
    ])

    monkeypatch.setattr(
        node_module,
        'match_scan',
        lambda **kwargs: next(results),
    )

    for _ in range(3):
        node.on_scan(_make_scan())

    validity_publisher = publishers['/localisation/valid'][1]

    values = [
        call.args[0].data
        for call in validity_publisher.publish.call_args_list
    ]

    assert values == [True, False, True]

    pose_publisher = publishers['/localisation/pose'][1]
    assert pose_publisher.publish.call_count == 2


# The prior comes from match.yaml; the simulator's start override is not a parameter here
def test_prior_ignores_simulator_start_override(node_harness):
    node, _, _ = node_harness

    for name in ('start_x_m', 'start_y_m', 'start_yaw_deg'):
        assert not node.has_parameter(name)

    assert node.initial_pose == pytest.approx(
        node_module.match_pose(
            node.arena, node.footprint[0], node_module.load_match_config(), 'start'
        )
    )
