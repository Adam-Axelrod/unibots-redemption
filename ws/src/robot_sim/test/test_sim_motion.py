# ws/src/robot_sim/test/test_sim_motion.py

import math

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
