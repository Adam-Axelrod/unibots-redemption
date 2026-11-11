
# ws/src/robot_sim/robot_sim/world.py

import math


# Create the four line segments of an axis-aligned rectangle
def _rect(cx, cy, w, h):
    x0, x1 = cx - w / 2.0, cx + w / 2.0
    y0, y1 = cy - h / 2.0, cy + h / 2.0
    return [
        ((x0, y0), (x1, y0)),
        ((x1, y0), (x1, y1)),
        ((x1, y1), (x0, y1)),
        ((x0, y1), (x0, y0)),
    ]


# Create an empty 6 m by 4 m test environment
def empty_room():
    return {
        'name': 'empty_room',
        'size': (6.0, 4.0),
        'start': (0.0, 0.0, 0.0),
        'segments': _rect(0.0, 0.0, 6.0, 4.0),
    }


# Create the test environment with three obstacles
def furnished_room():
    segs = _rect(0.0, 0.0, 6.0, 4.0)
    segs += _rect(1.5, 1.0, 1.2, 0.8)
    segs += _rect(-1.8, -1.2, 0.6, 0.6)
    segs += _rect(2.0, -1.4, 0.4, 1.2)
    return {
        'name': 'furnished_room',
        'size': (6.0, 4.0),
        'start': (-2.4, -1.6, 0.0),
        'segments': segs,
    }


# Create an irregular room for testing non-rectangular boundaries
def l_shaped_room():
    pts = [(-3, -2), (3, -2), (3, 1), (0, 1), (0, 2), (-3, 2)]
    segs = [(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))]
    return {
        'name': 'l_shaped_room',
        'size': (6.0, 4.0),
        'start': (-2.4, -1.4, 0.0),
        'segments': [
            ((float(a[0]), float(a[1])), (float(b[0]), float(b[1])))
            for a, b in segs
        ],
    }


# Create the competition arena using the fixed localisation geometry
def competition_arena():
    from localisation.frames import load_arena_frame
    from localisation.sensors import load_sensor_config

    arena = load_arena_frame()
    robot_length_m = load_sensor_config()['robot']['footprint_m'][0]

    return {
        'name': 'competition_arena',
        'size': (arena.side_length_m, arena.side_length_m),
        'start': arena.nominal_start_pose(robot_length_m),
        'segments': list(arena.wall_segments().values()),
    }


WORLDS = {
    'empty_room': empty_room,
    'furnished_room': furnished_room,
    'l_shaped_room': l_shaped_room,
    'competition_arena': competition_arena,
}


# Load a registered simulation environment by name
def load(name):
    if name not in WORLDS:
        raise ValueError(
            f"unknown world '{name}'. Available: {', '.join(sorted(WORLDS))}"
        )
    return WORLDS[name]()


# Find the nearest intersection of a LiDAR ray with the environment
def raycast(origin, angle, segments, max_range):
    px, py = origin
    dx, dy = math.cos(angle), math.sin(angle)
    best = max_range

    for (x1, y1), (x2, y2) in segments:
        sx, sy = x2 - x1, y2 - y1
        denom = sx * dy - sy * dx
        if abs(denom) < 1e-12:
            continue

        t = (-(x1 - px) * sy + sx * (y1 - py)) / denom
        u = (dx * (y1 - py) - dy * (x1 - px)) / denom

        if 0.0 <= t < best and 0.0 <= u <= 1.0:
            best = t

    return best


# Calculate the shortest distance from a point to the environment
def min_distance(point, segments):
    px, py = point
    best = float('inf')

    for (x1, y1), (x2, y2) in segments:
        sx, sy = x2 - x1, y2 - y1
        length_sq = sx * sx + sy * sy

        if length_sq < 1e-12:
            d = math.hypot(px - x1, py - y1)
        else:
            # Project the point onto the finite segment
            t = max(
                0.0,
                min(
                    1.0,
                    ((px - x1) * sx + (py - y1) * sy) / length_sq,
                ),
            )
            d = math.hypot(
                px - (x1 + t * sx),
                py - (y1 + t * sy),
            )

        best = min(best, d)

    return best
