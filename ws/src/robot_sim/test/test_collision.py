
# ws/src/robot_sim/test/test_collision.py

import math

import pytest

from robot_sim.collision import (
    collides_with_segments,
    footprint_corners,
    motion_collides,
)


FOOTPRINT = (0.200, 0.200)
HOME_WALL = [((-1.25, -1.25), (1.25, -1.25))]
START = (0.0, -1.15, math.pi / 2.0)


# The corners follow the robot orientation
def test_rotated_corners():
    corners = footprint_corners((0.0, 0.0, math.pi / 4.0), FOOTPRINT)

    assert max(x for x, _ in corners) == pytest.approx(math.sqrt(2) * 0.1)
    assert min(y for _, y in corners) == pytest.approx(-math.sqrt(2) * 0.1)


# Exact edge contact is permitted
def test_touching_home_wall():
    assert not collides_with_segments(START, FOOTPRINT, HOME_WALL)


# Driving into the home wall is blocked
def test_reverse_into_wall():
    pose = (0.0, -1.151, math.pi / 2.0)

    assert collides_with_segments(pose, FOOTPRINT, HOME_WALL)


# Driving away from the home wall is permitted
def test_forward_from_wall():
    end = (0.0, -1.14, math.pi / 2.0)

    assert not motion_collides(START, end, FOOTPRINT, HOME_WALL)


# Sliding along the wall is permitted
def test_slide_along_wall():
    end = (0.2, -1.15, math.pi / 2.0)

    assert not motion_collides(START, end, FOOTPRINT, HOME_WALL)


# Turning at the home wall cannot push a corner through it
def test_turn_into_home_wall():
    end = (0.0, -1.15, math.pi / 2.0 + 0.15)

    assert motion_collides(START, end, FOOTPRINT, HOME_WALL)


# Rotation in free space is permitted
def test_rotation_in_open_space():
    assert not motion_collides(
        (0.0, 0.0, 0.0),
        (0.0, 0.0, math.pi / 2),
        FOOTPRINT,
        HOME_WALL,
    )


# An obstacle can intersect a corner without touching the centre
def test_corner_collision():
    obstacle = [((0.13, -0.02), (0.13, 0.02))]

    assert collides_with_segments(
        (0.0, 0.0, math.pi / 4),
        FOOTPRINT,
        obstacle,
    )
    assert not collides_with_segments(
        (0.0, 0.0, 0.0),
        FOOTPRINT,
        obstacle,
    )


# A segment along the rectangle boundary is only touching
def test_boundary_overlap():
    segment = [((0.10, -1.0), (0.10, 1.0))]

    assert not collides_with_segments(
        (0.0, 0.0, 0.0),
        FOOTPRINT,
        segment,
    )


# A short obstacle inside the robot footprint is detected
def test_segment_inside_footprint():
    obstacle = [((-0.02, 0.01), (0.02, 0.01))]

    assert collides_with_segments(
        (0.0, 0.0, 0.0),
        FOOTPRINT,
        obstacle,
    )


# A long motion cannot jump across a thin wall
def test_thin_wall_tunnelling():
    wall = [((0.0, -1.0), (0.0, 1.0))]

    assert motion_collides(
        (-0.5, 0.0, 0.0),
        (0.5, 0.0, 0.0),
        FOOTPRINT,
        wall,
    )


# Intermediate rotation can collide even if both endpoints are clear
def test_rotation_sweeps_corner_into_wall():
    wall = [((-1.0, -0.11), (1.0, -0.11))]
    start = (0.0, 0.0, 0.0)
    end = (0.0, 0.0, math.pi / 2.0)

    assert not collides_with_segments(start, FOOTPRINT, wall)
    assert not collides_with_segments(end, FOOTPRINT, wall)
    assert motion_collides(start, end, FOOTPRINT, wall)


# Invalid dimensions and step size are rejected
def test_invalid_geometry():
    with pytest.raises(ValueError):
        collides_with_segments((0, 0, 0), (0, 0.2), HOME_WALL)

    with pytest.raises(ValueError):
        motion_collides(
            START,
            START,
            FOOTPRINT,
            HOME_WALL,
            max_corner_step_m=0,
        )
