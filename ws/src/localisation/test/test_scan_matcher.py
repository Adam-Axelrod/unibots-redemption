
# ws/src/localisation/test/test_scan_matcher.py

import math
import random
from pathlib import Path

import pytest
import yaml

from localisation.frames import load_arena_frame
from localisation.scan_matcher import (
    match_scan,
    preprocess_scan,
    transform_points,
    wall_residuals,
)
from localisation.sensors import lidar_mount, load_sensor_config
from robot_sim.lidar import lidar_world_pose, simulate_ranges


ARENA = load_arena_frame()
SENSORS = load_sensor_config()
TRANSLATION, ROTATION = lidar_mount(SENSORS)

FOOTPRINT = tuple(SENSORS['robot']['footprint_m'])
WALLS = ARENA.wall_segments()
SEGMENTS = list(WALLS.values())

CONFIG_PATH = Path(__file__).resolve().parent.parent / 'config' / 'localisation.yaml'
CONFIG = yaml.safe_load(CONFIG_PATH.read_text(encoding='utf-8'))

START = ARENA.nominal_start_pose(FOOTPRINT[0])
BEAMS = 360
RANGE_MIN = 0.05
RANGE_MAX = 12.0


# Generate a LiDAR scan using the actual configured sensor mounting
def _simulate_scan(pose, segments=None, noise_std=0.0, seed=42):
    if segments is None:
        segments = SEGMENTS

    lidar_pose = lidar_world_pose(
        pose,
        TRANSLATION,
        ROTATION[2],
    )

    ranges = simulate_ranges(
        lidar_pose,
        BEAMS,
        segments,
        RANGE_MAX,
    )

    if noise_std > 0:
        rng = random.Random(seed)
        ranges = [
            max(RANGE_MIN + 1e-6, r + rng.gauss(0.0, noise_std))
            for r in ranges
        ]

    return ranges


# Convert a simulated scan into robot-frame LiDAR points
def _scan_points(pose, segments=None, noise_std=0.0, seed=42):
    ranges = _simulate_scan(
        pose,
        segments=segments,
        noise_std=noise_std,
        seed=seed,
    )

    return preprocess_scan(
        ranges=ranges,
        angle_min=-math.pi,
        angle_increment=2.0 * math.pi / BEAMS,
        lidar_translation_m=TRANSLATION,
        lidar_yaw_rad=ROTATION[2],
        usable_sector_deg=SENSORS['lidar']['usable_sector_deg'],
        range_min=RANGE_MIN,
        range_max=RANGE_MAX,
    )


# Calculate translation and heading errors between two poses
def _pose_error(estimated, actual):
    position_error = math.hypot(
        estimated[0] - actual[0],
        estimated[1] - actual[1],
    )

    yaw_error = math.atan2(
        math.sin(estimated[2] - actual[2]),
        math.cos(estimated[2] - actual[2]),
    )

    return position_error, abs(yaw_error)


# Check that valid scan points respect the sensor mounting and usable sector
def test_preprocess_scan():
    points = preprocess_scan(
        ranges=[1.0, 1.0, 0.04, 2.0, math.inf, 12.0],
        angle_min=-math.pi,
        angle_increment=math.pi / 2.0,
        lidar_translation_m=(0.020, 0.0, 0.144),
        lidar_yaw_rad=-math.pi / 2.0,
        usable_sector_deg=(-100.0, 100.0),
        range_min=0.05,
        range_max=12.0,
    )

    assert len(points) == 2
    assert points[0] == pytest.approx((0.020, 1.0))
    assert points[1] == pytest.approx((2.020, 0.0))


# Reject invalid scan preprocessing parameters
def test_preprocess_invalid_parameters():
    arguments = dict(
        ranges=[1.0],
        angle_min=0.0,
        angle_increment=0.1,
        lidar_translation_m=(0.020, 0.0),
        lidar_yaw_rad=-math.pi / 2.0,
        usable_sector_deg=(-100.0, 100.0),
        range_min=0.05,
        range_max=12.0,
    )

    with pytest.raises(ValueError):
        preprocess_scan(**arguments, stride=0)

    with pytest.raises(ValueError):
        preprocess_scan(
            **{**arguments, 'angle_increment': 0.0}
        )

    with pytest.raises(ValueError):
        preprocess_scan(
            **{**arguments, 'usable_sector_deg': (-200.0, 100.0)}
        )


# Transform points from the robot frame into the arena map frame
def test_transform_points():
    points = [(1.0, 0.0), (0.0, 1.0)]

    transformed = transform_points(
        points,
        (0.5, -0.5, math.pi / 2.0),
    )

    assert transformed[0] == pytest.approx((0.5, 0.5))
    assert transformed[1] == pytest.approx((-0.5, -0.5))


# Points on walls have zero residual and interior points have nonzero residual
def test_wall_residuals():
    points = [
        (ARENA.side_length_m / 2.0, 0.0),
        (0.0, ARENA.side_length_m / 2.0),
        (ARENA.side_length_m / 2.0 - 0.10, 0.0),
    ]

    residuals = wall_residuals(
        points,
        (0.0, 0.0, 0.0),
        WALLS,
    )

    assert residuals[0] is not None
    assert residuals[1] is not None
    assert residuals[2] is not None

    assert abs(residuals[0][1]) < 1e-9
    assert abs(residuals[1][1]) < 1e-9
    assert abs(residuals[2][1]) == pytest.approx(0.10)


# Recover the nominal starting position from a clean scan
def test_nominal_start_pose():
    points = _scan_points(START)

    result = match_scan(
        points,
        START,
        WALLS,
        CONFIG,
    )

    position_error, yaw_error = _pose_error(result.pose, START)

    assert result.valid, result.reason
    assert position_error < 0.005
    assert yaw_error < math.radians(0.5)
    assert result.rms_error_m < 0.005
    assert result.inlier_count > 20


# Recover different lateral starting positions without ground-truth initialisation
@pytest.mark.parametrize(
    'lateral_offset',
    [-0.20, -0.10, 0.0, 0.10, 0.20],
)
def test_lateral_start_positions(lateral_offset):
    true_pose = (
        lateral_offset,
        START[1],
        START[2],
    )

    points = _scan_points(true_pose)

    result = match_scan(
        points,
        START,
        WALLS,
        CONFIG,
    )

    position_error, yaw_error = _pose_error(result.pose, true_pose)

    assert result.valid, result.reason
    assert position_error < 0.02
    assert yaw_error < math.radians(1.0)


# Recover positions near the lateral starting limits using multiple guesses
@pytest.mark.parametrize(
    'lateral_offset',
    [-0.28, 0.28],
)
def test_lateral_bounds_with_alternate_seeds(lateral_offset):
    true_pose = (
        lateral_offset,
        START[1],
        START[2],
    )

    points = _scan_points(true_pose)

    result = match_scan(
        points,
        START,
        WALLS,
        CONFIG,
        alternate_initial_poses=[
            (-0.20, START[1], START[2]),
            (0.20, START[1], START[2]),
        ],
    )

    position_error, yaw_error = _pose_error(result.pose, true_pose)

    assert result.valid, result.reason
    assert position_error < 0.02
    assert yaw_error < math.radians(1.0)


# Recover heading errors and simultaneous translation errors
@pytest.mark.parametrize(
    'true_pose',
    [
        (0.0, -1.10, math.radians(85.0)),
        (0.0, -1.10, math.radians(95.0)),
        (0.12, -1.10, math.radians(85.0)),
        (-0.12, -1.10, math.radians(95.0)),
    ],
)
def test_position_and_heading_recovery(true_pose):
    points = _scan_points(true_pose)

    result = match_scan(
        points,
        START,
        WALLS,
        CONFIG,
    )

    position_error, yaw_error = _pose_error(result.pose, true_pose)

    assert result.valid, result.reason
    assert position_error < 0.025
    assert yaw_error < math.radians(1.5)


# Recover the pose when scan measurements contain Gaussian noise
@pytest.mark.parametrize(
    'noise_std',
    [0.005, 0.020, 0.050],
)
def test_noisy_scan(noise_std):
    true_pose = (
        0.10,
        -1.10,
        math.radians(93.0),
    )

    points = _scan_points(
        true_pose,
        noise_std=noise_std,
        seed=2026,
    )

    result = match_scan(
        points,
        START,
        WALLS,
        CONFIG,
    )

    position_error, yaw_error = _pose_error(result.pose, true_pose)

    assert result.valid, result.reason
    assert position_error < 0.06
    assert yaw_error < math.radians(3.0)


# Reject obstacle returns while retaining enough visible wall measurements
def test_partial_occlusion():
    true_pose = (
        0.08,
        -1.10,
        math.pi / 2.0,
    )

    obstacle = ((-0.35, 0.40), (0.35, 0.40))

    points = _scan_points(
        true_pose,
        segments=[*SEGMENTS, obstacle],
    )

    result = match_scan(
        points,
        START,
        WALLS,
        CONFIG,
    )

    position_error, yaw_error = _pose_error(result.pose, true_pose)

    assert result.valid, result.reason
    assert position_error < 0.04
    assert yaw_error < math.radians(2.0)


# Report failure when an empty scan contains no localisation information
def test_empty_scan():
    result = match_scan(
        [],
        START,
        WALLS,
        CONFIG,
    )

    assert not result.valid
    assert result.inlier_count == 0
    assert math.isinf(result.rms_error_m)


# Report failure when only one wall provides geometric support
def test_single_wall_is_insufficient():
    half = ARENA.side_length_m / 2.0

    points = [
        (x / 10.0, half)
        for x in range(-10, 11)
    ]

    result = match_scan(
        points,
        (0.0, 0.0, 0.0),
        WALLS,
        CONFIG,
    )

    assert not result.valid


# Reject invalid scan matching configuration
def test_invalid_matching_config():
    bad_config = {
        'scan_matching': {
            **CONFIG['scan_matching'],
            'huber_delta_m': 0.0,
        }
    }

    with pytest.raises(ValueError):
        match_scan(
            _scan_points(START),
            START,
            WALLS,
            bad_config,
        )
