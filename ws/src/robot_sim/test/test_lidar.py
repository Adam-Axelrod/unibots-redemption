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
YAW = 0.0


# The configured mounting is 20 mm forward, 144 mm up and facing forward
def test_configured_mount():
    (x, y, z), (roll, pitch, yaw) = lidar_mount(load_sensor_config())

    assert (x, y, z) == pytest.approx((0.020, 0.0, 0.144))
    assert (roll, pitch, yaw) == pytest.approx((0.0, 0.0, YAW))


# The forward offset rotates with the robot
def test_offset_follows_heading():
    x, y, yaw = lidar_world_pose((1.0, 2.0, math.pi / 2.0), OFFSET, YAW)

    assert (x, y) == pytest.approx((1.0, 2.02))
    assert yaw == pytest.approx(math.pi / 2.0)


# The last beam stops one increment short of a full turn
def test_beam_angles():
    angles = beam_angles(360)

    assert angles[0] == pytest.approx(-math.pi)
    assert angles[-1] == pytest.approx(math.pi - math.radians(1.0))


# At yaw 0, LiDAR 0 degrees is robot forward and +90 degrees is robot left
def test_ranges_from_lidar_origin():
    beams = 360
    lidar = lidar_world_pose((0.0, 0.0, 0.0), OFFSET, YAW)
    ranges = simulate_ranges(lidar, beams, ARENA, 12.0)

    # Index of LiDAR angle a is (a + 180) degrees
    back, right, forward, left = ranges[0], ranges[90], ranges[180], ranges[270]

    assert forward == pytest.approx(HALF - 0.020)
    assert back == pytest.approx(HALF + 0.020)
    assert left == pytest.approx(HALF)
    assert right == pytest.approx(HALF)


# A wall just ahead of the robot must appear at LiDAR 0 degrees, whichever way the robot faces
@pytest.mark.parametrize('heading', [0.0, math.pi / 2.0, -2.0])
def test_forward_beam_follows_robot_heading(heading):
    c, s = math.cos(heading), math.sin(heading)
    # A short wall across the robot's forward axis, 0.5 m ahead of its centre
    centre = (0.5 * c, 0.5 * s)
    wall = [((centre[0] + 0.3 * s, centre[1] - 0.3 * c),
             (centre[0] - 0.3 * s, centre[1] + 0.3 * c))]

    (tx, ty, _), (_, _, yaw) = lidar_mount(load_sensor_config())
    lidar = lidar_world_pose((0.0, 0.0, heading), (tx, ty), yaw)
    ranges = simulate_ranges(lidar, 360, wall, 12.0)

    assert ranges[180] == pytest.approx(0.5 - tx)
    assert min(ranges) == pytest.approx(ranges[180])


# The launch-file TF publishes the same mounting the simulator and localisation read
def test_static_tf_matches_sensor_config():
    from robot_bringup.lidar_tf import lidar_transform

    config = load_sensor_config()
    (x, y, z), (roll, pitch, yaw) = lidar_mount(config)
    node = lidar_transform()
    # launch_ros keeps the raw arguments privately; they are plain strings here
    args = node._Node__arguments

    assert [float(a) for a in args[:6]] == pytest.approx([x, y, z, yaw, pitch, roll])
    assert args[6:] == [config['robot']['base_frame'], config['lidar']['frame_id']]
    assert yaw == pytest.approx(0.0)
