"""The simulated room: walls and obstacles the lidar can see.

A world is just a list of line segments. Walls are the room outline; obstacles
are boxes inside it (tables, boxes, whatever). The lidar raycasts against these
segments -- there is no 3D geometry and no physics engine.

To add your own room, write a new function here and add it to WORLDS at the
bottom. Then run with:  ros2 launch robot_bringup sim.launch.py world:=my_room
"""
import math


def _rect(cx, cy, w, h):
    """Four segments making an axis-aligned rectangle centred on (cx, cy)."""
    x0, x1 = cx - w / 2.0, cx + w / 2.0
    y0, y1 = cy - h / 2.0, cy + h / 2.0
    return [
        ((x0, y0), (x1, y0)),
        ((x1, y0), (x1, y1)),
        ((x1, y1), (x0, y1)),
        ((x0, y1), (x0, y0)),
    ]


def empty_room():
    """A bare 6 m x 4 m room. Start here."""
    return {
        'name': 'empty_room',
        'size': (6.0, 4.0),
        'start': (0.0, 0.0, 0.0),          # x, y, yaw
        'segments': _rect(0.0, 0.0, 6.0, 4.0),
    }


def furnished_room():
    """Same room with three obstacles, to test obstacle handling."""
    segs = _rect(0.0, 0.0, 6.0, 4.0)
    segs += _rect(1.5, 1.0, 1.2, 0.8)      # table
    segs += _rect(-1.8, -1.2, 0.6, 0.6)    # box
    segs += _rect(2.0, -1.4, 0.4, 1.2)     # shelf
    return {
        'name': 'furnished_room',
        'size': (6.0, 4.0),
        'start': (-2.4, -1.6, 0.0),
        'segments': segs,
    }


def l_shaped_room():
    """A non-rectangular room -- breaks naive bounding-box assumptions."""
    pts = [(-3, -2), (3, -2), (3, 1), (0, 1), (0, 2), (-3, 2)]
    segs = [(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))]
    return {
        'name': 'l_shaped_room',
        'size': (6.0, 4.0),
        'start': (-2.4, -1.4, 0.0),
        'segments': [((float(a[0]), float(a[1])), (float(b[0]), float(b[1])))
                     for a, b in segs],
    }


WORLDS = {
    'empty_room': empty_room,
    'furnished_room': furnished_room,
    'l_shaped_room': l_shaped_room,
}


def load(name):
    if name not in WORLDS:
        raise ValueError(
            f"unknown world '{name}'. Available: {', '.join(sorted(WORLDS))}")
    return WORLDS[name]()


def raycast(origin, angle, segments, max_range):
    """Distance from `origin` along `angle` to the nearest segment.

    Returns max_range if nothing is hit. This is the whole lidar model.
    """
    px, py = origin
    dx, dy = math.cos(angle), math.sin(angle)
    best = max_range

    for (x1, y1), (x2, y2) in segments:
        sx, sy = x2 - x1, y2 - y1
        denom = sx * dy - sy * dx
        if abs(denom) < 1e-12:
            continue                      # ray parallel to segment
        t = (-(x1 - px) * sy + sx * (y1 - py)) / denom
        u = (dx * (y1 - py) - dy * (x1 - px)) / denom
        if 0.0 <= t < best and 0.0 <= u <= 1.0:
            best = t

    return best


def min_distance(point, segments):
    """Shortest distance from a point to any segment. Used for collisions."""
    px, py = point
    best = float('inf')

    for (x1, y1), (x2, y2) in segments:
        sx, sy = x2 - x1, y2 - y1
        length_sq = sx * sx + sy * sy
        if length_sq < 1e-12:
            d = math.hypot(px - x1, py - y1)
        else:
            # project the point onto the segment, clamped to its ends
            t = max(0.0, min(1.0, ((px - x1) * sx + (py - y1) * sy) / length_sq))
            d = math.hypot(px - (x1 + t * sx), py - (y1 + t * sy))
        best = min(best, d)

    return best
