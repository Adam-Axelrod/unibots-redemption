
# ws/src/robot_sim/robot_sim/collision.py

import math


# Return the footprint corners in world coordinates
def footprint_corners(pose, footprint_m):
    x, y, yaw = pose
    length, width = footprint_m
    hx, hy = length / 2.0, width / 2.0
    c, s = math.cos(yaw), math.sin(yaw)

    return [
        (x + c * dx - s * dy, y + s * dx + c * dy)
        for dx, dy in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))
    ]


# Detect whether a segment crosses the interior of an axis-aligned rectangle
def _segment_in_rectangle(a, b, hx, hy):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    t_min, t_max = 0.0, 1.0

    for pos, delta, half in ((ax, dx, hx), (ay, dy, hy)):
        if abs(delta) < 1e-15:
            if abs(pos) >= half:
                return False
            continue

        t1, t2 = (-half - pos) / delta, (half - pos) / delta
        t_min = max(t_min, min(t1, t2))
        t_max = min(t_max, max(t1, t2))

        if t_min >= t_max:
            return False

    return t_min < t_max


# Test an oriented footprint against the environment line segments
def collides_with_segments(pose, footprint_m, segments, tolerance_m=1e-9):
    x, y, yaw = pose
    length, width = footprint_m

    if length <= 0 or width <= 0:
        raise ValueError('footprint dimensions must be positive')
    if tolerance_m < 0:
        raise ValueError('tolerance_m must be non-negative')

    hx = max(0.0, length / 2.0 - tolerance_m)
    hy = max(0.0, width / 2.0 - tolerance_m)
    c, s = math.cos(yaw), math.sin(yaw)

    # Transform segments into the robot frame
    for a, b in segments:
        local_a = (
            c * (a[0] - x) + s * (a[1] - y),
            -s * (a[0] - x) + c * (a[1] - y),
        )
        local_b = (
            c * (b[0] - x) + s * (b[1] - y),
            -s * (b[0] - x) + c * (b[1] - y),
        )

        if _segment_in_rectangle(local_a, local_b, hx, hy):
            return True

    return False


# Check intermediate poses to avoid stepping through a thin wall
def motion_collides(
    start_pose,
    end_pose,
    footprint_m,
    segments,
    max_corner_step_m=0.01,
):
    if max_corner_step_m <= 0:
        raise ValueError('max_corner_step_m must be positive')

    x0, y0, yaw0 = start_pose
    x1, y1, yaw1 = end_pose

    delta_yaw = math.atan2(
        math.sin(yaw1 - yaw0),
        math.cos(yaw1 - yaw0),
    )

    length, width = footprint_m
    corner_radius = math.hypot(length, width) / 2.0

    path_length = (
        math.hypot(x1 - x0, y1 - y0)
        + corner_radius * abs(delta_yaw)
    )
    steps = max(1, math.ceil(path_length / max_corner_step_m))

    for i in range(steps + 1):
        t = i / steps

        pose = (
            x0 + t * (x1 - x0),
            y0 + t * (y1 - y0),
            yaw0 + t * delta_yaw,
        )

        if collides_with_segments(pose, footprint_m, segments):
            return True

    return False
