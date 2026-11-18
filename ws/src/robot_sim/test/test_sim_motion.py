# ws/src/robot_sim/test/test_sim_motion.py

import math

import pytest

from localisation.sensors import load_sensor_config
from robot_sim import world as world_lib
from robot_sim.collision import motion_collides


ARENA = world_lib.load('competition_arena')
FOOTPRINT = tuple(load_sensor_config()['robot']['footprint_m'])
SEGMENTS = ARENA['segments']
START = ARENA['start']


# Move the start pose in the robot frame
def _moved(pose, forward, left, turn=0.0):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * forward - s * left, y + s * forward + c * left, yaw + turn


# The arena starts the robot touching the home wall, facing inwards
def test_arena_start_pose():
    x, y, yaw = START

    assert ARENA['size'] == (2.5, 2.5)
    assert abs(x) < 1e-9
    assert math.isclose(y, -1.25 + FOOTPRINT[0] / 2.0)
    assert math.isclose(yaw, math.pi / 2.0)


def test_forward_from_home_wall_allowed():
    assert not motion_collides(START, _moved(START, 0.02, 0.0), FOOTPRINT, SEGMENTS)


def test_backward_into_home_wall_blocked():
    assert motion_collides(START, _moved(START, -0.02, 0.0), FOOTPRINT, SEGMENTS)


def test_rotation_into_home_wall_blocked():
    assert motion_collides(START, _moved(START, 0.0, 0.0, 0.1), FOOTPRINT, SEGMENTS)


def test_free_space_rotation_and_strafe_allowed():
    centre = (0.0, 0.0, 0.0)

    assert not motion_collides(centre, _moved(centre, 0.0, 0.0, 0.5), FOOTPRINT, SEGMENTS)
    assert not motion_collides(centre, _moved(centre, 0.0, 0.05), FOOTPRINT, SEGMENTS)


# --- simulator start pose override ---

from robot_sim.sim_node import parse_start_override


def _override(x, y, yaw_deg):
    return parse_start_override(x, y, yaw_deg, FOOTPRINT, SEGMENTS)


def test_no_override_keeps_configured_start():
    assert _override('', '', '') is None
    assert _override('  ', '', ' ') is None
    assert world_lib.load('competition_arena')['start'] == START


@pytest.mark.parametrize('x', [-0.2, 0.2, -0.28, 0.28])
def test_lateral_start_overrides(x):
    pose = _override(str(x), str(START[1]), '90')

    assert pose == pytest.approx((x, START[1], math.pi / 2.0))


@pytest.mark.parametrize('x, y, yaw_deg', [(0.10, -1.10, 85.0), (-0.15, -1.10, 95.0)])
def test_heading_offset_clear_of_home_wall(x, y, yaw_deg):
    pose = _override(str(x), str(y), str(yaw_deg))

    assert pose == pytest.approx((x, y, math.radians(yaw_deg)))


@pytest.mark.parametrize('values', [
    ('0.2', '', ''),
    ('0.2', '-1.15', ''),
    ('', '-1.15', '90'),
    ('abc', '-1.15', '90'),
    ('nan', '-1.15', '90'),
    ('0.2', 'inf', '90'),
])
def test_invalid_or_incomplete_override_rejected(values):
    with pytest.raises(ValueError):
        _override(*values)


@pytest.mark.parametrize('x, y, yaw_deg', [
    (0.0, -1.20, 90.0),
    (0.0, -1.15, 85.0),
    (1.20, 0.0, 0.0),
])
def test_override_into_wall_rejected(x, y, yaw_deg):
    with pytest.raises(ValueError):
        _override(str(x), str(y), str(yaw_deg))
