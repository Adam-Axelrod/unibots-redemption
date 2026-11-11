"""bcd -- Boustrophedon Cellular Decomposition, with minimum-turn sweeping.

Plain Python, no ROS, so it can be tested on its own. Works on square cells
(the driver's coverage grid, one lane wide), given as (i, j) pairs.

After Luo et al., "Complete Coverage Path Planning for Intelligent Sweeping
Robot" (IEEE IPEC 2021):

    1. Decompose: sweep a line across the free cells; a new region starts
       wherever the line's free stretch splits or joins (Choset's BCD). Each
       region can be swept by straight back-and-forth lanes.
    2. Merge, as the paper proposes, neighbouring regions whose union can
       still be swept by one set of parallel lanes. Fewer regions, fewer
       turns and fewer journeys between them.
    3. Cost turns as distance: path length + turns x their cost, so a plan
       with fewer turns can win over a slightly shorter one.
    4. For each region choose the lane direction, and the corner to start in,
       and choose the order of regions, for the lowest total cost: every order
       is tried when there are few regions, as the paper does; nearest-next
       otherwise.

Unlike the paper's arbitrary sweep angles (rotating calipers), lanes here run
along the grid's two axes: the arena is a rectangle, and the grid is lined up
with its walls.
"""
import itertools
import math

AXES = (0, 1)        # lanes along i (j constant), or along j (i constant)


def _intervals(values):
    """Sorted integers as runs of consecutive values: [(lo, hi), ...]."""
    runs = []
    for v in sorted(values):
        if runs and v == runs[-1][1] + 1:
            runs[-1][1] = v
        else:
            runs.append([v, v])
    return [tuple(r) for r in runs]


def _lines(region, axis):
    """The region's lanes along `axis`: {other coordinate: [axis coordinates]}."""
    lines = {}
    for c in region:
        lines.setdefault(c[1 - axis], []).append(c[axis])
    return lines


def sweepable(region, axis):
    """True if every lane along `axis` crosses the region in one stretch:
    then one set of parallel lanes covers it without gaps or repeats."""
    lines = _lines(region, axis)
    return all(len(_intervals(v)) == 1 for v in lines.values()) and \
        len(_intervals(lines)) == 1


def decompose(cells, sweep):
    """Boustrophedon cellular decomposition of `cells`, sweeping a line along
    axis `sweep`. Returns a list of regions (sets of cells)."""
    slices = {}
    for c in cells:
        slices.setdefault(c[sweep], []).append(c[1 - sweep])
    regions, open_, last = [], [], None
    for k in sorted(slices):
        if last is not None and k != last + 1:
            open_ = []                       # a gap in the sweep: nothing carries on
        here = _intervals(slices[k])
        now = []
        for lo, hi in here:
            touching = [(a, b, r) for a, b, r in open_ if a <= hi and lo <= b]
            region = None
            if len(touching) == 1:
                a, b, r = touching[0]
                # Carry on only if that stretch did not split: a split, or
                # stretches joining, is where BCD starts new regions.
                if sum(1 for lo2, hi2 in here if a <= hi2 and lo2 <= b) == 1:
                    region = r
            if region is None:
                regions.append(set())
                region = len(regions) - 1
            for o in range(lo, hi + 1):
                regions[region].add((k, o) if sweep == 0 else (o, k))
            now.append((lo, hi, region))
        open_, last = now, k
    return regions


def merge(regions):
    """Join neighbouring regions while the union can still be swept by one
    set of parallel lanes, in either direction."""
    regions = [set(r) for r in regions]
    merged = True
    while merged:
        merged = False
        for a, b in itertools.combinations(range(len(regions)), 2):
            ra, rb = regions[a], regions[b]
            if not any((i + di, j + dj) in rb for i, j in ra
                       for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1))):
                continue
            union = ra | rb
            if sweepable(union, 0) or sweepable(union, 1):
                regions[a] = union
                del regions[b]
                merged = True
                break
    return regions


def lanes(region, axis, from_low_line, from_low_end):
    """The region's back-and-forth lanes along `axis`, as [(start, end), ...]:
    lanes in order from the low or high side, the first run from its low or
    high end, then alternating."""
    lines = _lines(region, axis)
    out = []
    forward = from_low_end
    for o in sorted(lines, reverse=not from_low_line):
        lo, hi = min(lines[o]), max(lines[o])
        a, b = (lo, hi) if forward else (hi, lo)
        out.append(((a, o), (b, o)) if axis == 0 else ((o, a), (o, b)))
        forward = not forward
    return out


def _options(region, lane_width, uturn_cost, prefer_axis=None, switch_cost=0.0):
    """Every way to sweep a region: (cost inside it, axis, lanes). Lanes
    across `prefer_axis` cost `switch_cost` more, so that a replan keeps the
    direction we were already sweeping unless the other is clearly better."""
    options = []
    for axis in AXES:
        if not sweepable(region, axis):
            continue
        for low_line in (True, False):
            for low_end in (True, False):
                ls = lanes(region, axis, low_line, low_end)
                length = sum(abs(a[0] - b[0]) + abs(a[1] - b[1]) for a, b in ls) * lane_width
                shifts = (len(ls) - 1) * lane_width
                cost = length + shifts + (len(ls) - 1) * uturn_cost
                if prefer_axis is not None and axis != prefer_axis:
                    cost += switch_cost
                options.append((cost, axis, ls))
    return options


def _distance(a, b, lane_width):
    return math.hypot(a[0] - b[0], a[1] - b[1]) * lane_width


def plan(cells, start, lane_width, turn_cost, uturn_cost, exhaustive=6,
         prefer_axis=None, switch_cost=0.0):
    """A complete coverage plan for `cells`, starting from cell `start`.

    Returns (cost, [(region, axis, lanes), ...]) in the order to sweep them.
    `turn_cost` and `uturn_cost` are a 90 deg turn and an about turn as metres
    of driving (the paper's c_t and c_r). Every order of regions is tried when
    there are at most `exhaustive` of them; otherwise the nearest next.
    `prefer_axis` and `switch_cost`: see _options.
    """
    best = (math.inf, [])
    for sweep in AXES:
        regions = merge(decompose(cells, sweep))
        options = [_options(r, lane_width, uturn_cost, prefer_axis, switch_cost)
                   for r in regions]
        keep = [k for k, o in enumerate(options) if o]
        if not keep:
            continue
        if len(keep) <= exhaustive:
            orders = itertools.permutations(keep)
        else:
            orders = [_nearest_order(keep, options, start, lane_width)]
        for order in orders:
            cost, chosen = _best_corners(order, options, start, lane_width, turn_cost)
            if cost < best[0]:
                best = (cost, [(regions[k], o[1], o[2]) for k, o in zip(order, chosen)])
    return best


def _best_corners(order, options, start, lane_width, turn_cost):
    """For a fixed order of regions, the cheapest way into and through each:
    dynamic programming over which option (axis and corner) each one uses."""
    # state: option index -> (cost so far, choices)
    states = {None: (0.0, [], start, None)}
    for k in order:
        nxt = {}
        for cost, chosen, pos, axis in states.values():
            for n, (inside, ax, ls) in enumerate(options[k]):
                c = cost + _distance(pos, ls[0][0], lane_width) + inside
                if axis is not None and ax != axis:
                    c += turn_cost                 # lanes the other way: turn to them
                if n not in nxt or c < nxt[n][0]:
                    nxt[n] = (c, chosen + [(inside, ax, ls)], ls[-1][1], ax)
        states = nxt
    cost, chosen, _, _ = min(states.values(), key=lambda s: s[0])
    return cost, chosen


def _nearest_order(keep, options, start, lane_width):
    order, pos, left = [], start, set(keep)
    while left:
        k = min(left, key=lambda k: min(_distance(pos, o[2][0][0], lane_width)
                                        for o in options[k]))
        order.append(k)
        left.remove(k)
        pos = options[k][0][2][-1][1]
    return order
