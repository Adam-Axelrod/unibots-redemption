"""coverage_map -- a lidar map of the room, for planning where to drive.

Plain Python, no ROS, so it can be tested on its own. Two pieces:

GridMap     an occupancy grid in the odom frame. Every scan marks the cells
            each beam passed through as free and the cell it hit as occupied.
            Cells near an occupied cell are "blocked": the chassis centre must
            stay out of them. bfs() and line_of_sight() plan paths on it.

wall_angle  which way the room's walls run, from one scan, so the sweep's lanes
            can be lined up with them whichever way the robot starts facing.

The map is in the odom frame, so it inherits odometry drift. Over one 180 s
match that is small; over a long run the walls will smear.
"""
import math
from collections import deque

# 8-connected neighbours
NEIGHBOURS = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]


class GridMap:

    HIT = 3          # evidence added when a beam ends in a cell
    MISS = -1        # ... and taken away when a beam passes through it
    # HIGH is high so that beams skimming past an obstacle's corner do not wear
    # it away; the price is that something that moves off (another robot)
    # lingers on the map for a couple of seconds.
    LOW, HIGH = -3, 12
    OCCUPIED = 2     # at or above this, the cell is a wall/obstacle

    def __init__(self, resolution, inflate_radius):
        self.res = resolution
        self.score = {}      # (i, j) -> evidence; a cell never seen is absent
        self.blocked = {}    # (i, j) -> how many occupied cells are within inflate_radius
        # (the 1e-9: 0.15 / 0.05 is 3.0000000000000004, and ceil would make it 4)
        r = int(math.ceil(inflate_radius / resolution - 1e-9))
        self._disk = [(di, dj) for di in range(-r, r + 1) for dj in range(-r, r + 1)
                      if di * di + dj * dj <= r * r]

    # ---------------------------------------------------------------- cells

    def cell(self, x, y):
        return (math.floor(x / self.res), math.floor(y / self.res))

    def centre(self, c):
        return ((c[0] + 0.5) * self.res, (c[1] + 0.5) * self.res)

    def is_free(self, c):
        """Seen, and not an obstacle."""
        s = self.score.get(c)
        return s is not None and s < self.OCCUPIED

    def traversable(self, c):
        """Free and far enough from everything for the chassis centre."""
        return self.is_free(c) and c not in self.blocked

    # ------------------------------------------------------------- updating

    def update(self, x, y, yaw, beams, max_range):
        """Add one scan taken from pose (x, y, yaw).

        beams: (angle, range, hit) per beam, angle in the robot frame. hit is
        False for a beam that returned nothing (it only clears space).
        """
        free, hits = set(), set()
        for angle, r, hit in beams:
            a = yaw + angle
            ca, sa = math.cos(a), math.sin(a)
            reach = min(r, max_range)
            # Stop a cell short of the hit, so a beam grazing a wall does not
            # wipe out the wall cell it is about to hit.
            for k in range(int(reach / self.res) - 1):
                d = k * self.res
                free.add(self.cell(x + ca * d, y + sa * d))
            if hit and r <= max_range:
                hits.add(self.cell(x + ca * r, y + sa * r))
        for c in free - hits:
            self._add(c, self.MISS)
        for c in hits:
            self._add(c, self.HIT)

    def _add(self, c, delta):
        old = self.score.get(c, 0)
        new = max(self.LOW, min(self.HIGH, old + delta))
        self.score[c] = new
        was, now = old >= self.OCCUPIED, new >= self.OCCUPIED
        if was == now:
            return
        # An obstacle appeared or went away: update the blocked zone round it.
        step = 1 if now else -1
        for di, dj in self._disk:
            n = (c[0] + di, c[1] + dj)
            count = self.blocked.get(n, 0) + step
            if count > 0:
                self.blocked[n] = count
            else:
                self.blocked.pop(n, None)

    # ------------------------------------------------------------- planning

    def bfs(self, start):
        """Distance in steps from `start` to every cell we can drive to, and
        each cell's parent on the way there (to rebuild a path).

        If `start` is itself blocked -- the robot is parked against a wall --
        we may pass through blocked-but-free cells to get clear: only for a
        few steps, and only ones no nearer to obstacles than the last (fewer or
        as many occupied cells in range), so the way out leads away from them.
        """
        escape_steps = len(self._disk) ** 0.5     # about the blocked zone's width
        dist, parent = {start: 0}, {start: None}
        queue = deque([start])
        while queue:
            c = queue.popleft()
            escaping = not self.traversable(c) and dist[c] < escape_steps
            for di, dj in NEIGHBOURS:
                n = (c[0] + di, c[1] + dj)
                if n in dist:
                    continue
                if self.traversable(n) or (escaping and self.is_free(n) and
                                           self.blocked.get(n, 0) <= self.blocked[c]):
                    dist[n] = dist[c] + 1
                    parent[n] = c
                    queue.append(n)
        return dist, parent

    @staticmethod
    def path_to(parent, goal):
        path = []
        while goal is not None:
            path.append(goal)
            goal = parent[goal]
        return path[::-1]

    def line_of_sight(self, x0, y0, x1, y1, skip=0.0):
        """True if the chassis centre can go straight from (x0, y0) to (x1, y1).
        The first `skip` metres are not checked, to let us leave a wall."""
        length = math.hypot(x1 - x0, y1 - y0)
        n = int(length / (self.res * 0.5)) + 1
        for k in range(n + 1):
            t = k / n
            if t * length < skip:
                continue
            if not self.traversable(self.cell(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)):
                return False
        return True


def wall_angle(points):
    """Which way the walls run, from one scan's (x, y) points in scan order.

    Returns (angle, confidence): angle in [0, pi/2) in the robot frame, walls
    being at that angle or at right angles to it; confidence 0..1 is how much
    of the outline agrees. Works by histogramming the direction of each short
    piece of outline between neighbouring points.
    """
    quarter = math.pi / 2
    n = 90                                  # 1 degree bins over 0..90
    bins = [0.0] * n
    pieces = []
    for (x1, y1), (x2, y2) in zip(points, points[1:]):
        d = math.hypot(x2 - x1, y2 - y1)
        if 0.01 < d < 0.15:                 # neighbours on the same surface
            a = math.atan2(y2 - y1, x2 - x1) % quarter
            bins[int(a / quarter * n) % n] += d
            pieces.append((a, d))
    total = sum(bins)
    if total == 0.0:
        return 0.0, 0.0
    peak = (max(range(n), key=lambda i: bins[i]) + 0.5) * quarter / n
    # Refine: the length-weighted mean of every piece within 3 deg of the
    # peak, measured as an offset from it so 89 deg and 1 deg average to 0.
    num = den = 0.0
    for a, d in pieces:
        off = (a - peak + quarter / 2) % quarter - quarter / 2
        if abs(off) < math.radians(3):
            num += off * d
            den += d
    return (peak + num / den) % quarter, den / total
