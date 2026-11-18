# ws/src/localisation/test/test_match_config.py

import copy
import math

import pytest

from localisation.frames import ArenaFrame, load_arena_frame, load_match_config, match_pose
from localisation.scan_matcher import preprocess_scan
from localisation.sensors import lidar_mount, load_sensor_config


ARENA = load_arena_frame()
MATCH = load_match_config()
SENSORS = load_sensor_config()
LENGTH = SENSORS['robot']['footprint_m'][0]


def _with(section, **offsets):
    match = copy.deepcopy(MATCH)
    match[section].update(offsets)
    return match


# The configured start and return target both resolve to the nominal home-wall pose
def test_configured_start_and_end_are_nominal():
    start = match_pose(ARENA, LENGTH, MATCH, 'start')
    end = match_pose(ARENA, LENGTH, MATCH, 'end')

    assert start == pytest.approx((0.0, -1.15, math.pi / 2.0))
    assert end == pytest.approx(start)
    assert start == pytest.approx(ARENA.nominal_start_pose(LENGTH))


# Moving the return target leaves the starting prior alone
def test_end_is_independent_of_start():
    match = _with('end', lateral_offset_m=0.2, inward_offset_m=0.5, yaw_deg=180.0)

    assert match_pose(ARENA, LENGTH, match, 'start') == pytest.approx((0.0, -1.15, math.pi / 2.0))
    assert match_pose(ARENA, LENGTH, match, 'end') == pytest.approx((-0.2, -0.65, -math.pi / 2.0))


# Facing +y, left is -x and anticlockwise yaw increases heading
def test_offsets_follow_arena_axes():
    match = _with('start', lateral_offset_m=0.1, inward_offset_m=0.05, yaw_deg=10.0)

    assert match_pose(ARENA, LENGTH, match, 'start') == pytest.approx(
        (-0.1, -1.10, math.radians(100.0)))


# The same offsets follow the arena when its forward axis is changed
@pytest.mark.parametrize('axis, sign, expected', [
    ('x', 1, (-1.10, 0.1, math.radians(10.0))),
    ('x', -1, (1.10, -0.1, math.radians(190.0))),
    ('y', -1, (0.1, 1.10, math.radians(-80.0))),
])
def test_other_forward_axes(axis, sign, expected):
    arena = ArenaFrame('map', axis, sign, ARENA.side_length_m, ARENA.guide_separation_m)
    match = _with('start', lateral_offset_m=0.1, inward_offset_m=0.05, yaw_deg=10.0)
    x, y, yaw = match_pose(arena, LENGTH, match, 'start')

    assert (x, y) == pytest.approx(expected[:2])
    assert math.cos(yaw - expected[2]) == pytest.approx(1.0)


@pytest.mark.parametrize('section, key, value', [
    ('start', 'lateral_offset_m', None),
    ('start', 'inward_offset_m', 'abc'),
    ('start', 'yaw_deg', math.nan),
    ('end', 'lateral_offset_m', math.inf),
    ('end', 'yaw_deg', True),
])
def test_invalid_offsets_rejected(section, key, value):
    match = _with(section, **{key: value})

    with pytest.raises(ValueError):
        match_pose(ARENA, LENGTH, match, section)


def test_missing_section_rejected():
    match = copy.deepcopy(MATCH)
    del match['end']

    with pytest.raises(ValueError):
        match_pose(ARENA, LENGTH, match, 'end')


# With the configured mounting, a LiDAR 0 degree return lies ahead of the robot
def test_forward_beam_is_robot_forward():
    translation, (_, _, yaw) = lidar_mount(SENSORS)
    points = preprocess_scan(
        ranges=[1.0, 1.0, 1.0, 1.0],
        angle_min=-math.pi,
        angle_increment=math.pi / 2.0,
        lidar_translation_m=translation,
        lidar_yaw_rad=yaw,
        usable_sector_deg=(-100.0, 100.0),
        range_min=0.05,
        range_max=12.0,
    )

    assert yaw == pytest.approx(0.0)
    # Beams at -90, 0 and +90 degrees survive the sector: right, forward and left
    expected = [(0.020, -1.0), (1.020, 0.0), (0.020, 1.0)]

    assert len(points) == len(expected)
    for point, target in zip(points, expected):
        assert point == pytest.approx(target)
