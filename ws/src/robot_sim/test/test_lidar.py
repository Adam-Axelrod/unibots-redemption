# ws/src/robot_sim/test/test_lidar.py

import math

import pytest

from localisation.sensors import lidar_mount, load_sensor_config
from robot_sim.lidar import beam_angles, lidar_world_pose, simulate_ranges


HALF = 1.25
ARENA = [
    ((-HALF, -HALF), (HALF, -HALF)),
    ((HALF, -HALF), (HALF, HALF)),
    ((HALF, HALF), (-HALF, HALF)),
    ((-HALF, HALF), (-HALF, -HALF)),
]
OFFSET = (0.020, 0.0)
YAW = -math.pi / 2.0


# The configured mounting is 20 mm forward, 144 mm up and turned -90 degrees
def test_configured_mount():
    (x, y, z), (roll, pitch, yaw) = lidar_mount(load_sensor_config())

    assert (x, y, z) == pytest.approx((0.020, 0.0, 0.144))
    assert (roll, pitch, yaw) == pytest.approx((0.0, 0.0, YAW))


# The forward offset rotates with the robot
def test_offset_follows_heading():
    x, y, yaw = lidar_world_pose((1.0, 2.0, math.pi / 2.0), OFFSET, YAW)

    assert (x, y) == pytest.approx((1.0, 2.02))
    assert yaw == pytest.approx(0.0)


# The last beam stops one increment short of a full turn
def test_beam_angles():
    angles = beam_angles(360)

    assert angles[0] == pytest.approx(-math.pi)
    assert angles[-1] == pytest.approx(math.pi - math.radians(1.0))


# At yaw -90 degrees, LiDAR +90 degrees is robot forward
def test_ranges_from_lidar_origin():
    beams = 360
    lidar = lidar_world_pose((0.0, 0.0, 0.0), OFFSET, YAW)
    ranges = simulate_ranges(lidar, beams, ARENA, 12.0)

    # Index of LiDAR angle a is (a + 180) degrees
    forward, left, back, right = ranges[270], ranges[0], ranges[90], ranges[180]

    assert forward == pytest.approx(HALF - 0.020)
    assert back == pytest.approx(HALF + 0.020)
    assert left == pytest.approx(HALF)
    assert right == pytest.approx(HALF)
