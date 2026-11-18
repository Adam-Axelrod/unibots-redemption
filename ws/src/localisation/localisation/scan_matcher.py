
# ws/src/localisation/localisation/scan_matcher.py

import math
from dataclasses import dataclass
from typing import Mapping, Sequence


Point = tuple[float, float]
Pose = tuple[float, float, float]
Segment = tuple[Point, Point]


@dataclass(frozen=True)
class ScanMatchResult:
    pose: Pose
    rms_error_m: float
    inlier_count: int
    wall_inliers: dict[str, int]
    iterations: int
    valid: bool
    reason: str


# Convert valid scan ranges into points relative to base_footprint
def preprocess_scan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    lidar_translation_m: Sequence[float],
    lidar_yaw_rad: float,
    usable_sector_deg: Sequence[float],
    range_min: float,
    range_max: float,
    stride: int = 1,
) -> list[Point]:
    if stride < 1:
        raise ValueError('stride must be positive')
    if angle_increment == 0 or not math.isfinite(angle_increment):
        raise ValueError('angle_increment must be finite and nonzero')
    if not 0 <= range_min < range_max:
        raise ValueError('range limits must be increasing and non-negative')
    if len(lidar_translation_m) < 2 or len(usable_sector_deg) != 2:
        raise ValueError('invalid lidar mounting or sector')

    sector_min, sector_max = (
        math.radians(float(v)) for v in usable_sector_deg
    )

    if not -math.pi <= sector_min < sector_max <= math.pi:
        raise ValueError('usable sector must be within [-180, 180] degrees')

    dx = float(lidar_translation_m[0])
    dy = float(lidar_translation_m[1])

    points = []

    for i in range(0, len(ranges), stride):
        distance = float(ranges[i])

        if not math.isfinite(distance):
            continue
        if not range_min < distance < range_max:
            continue

        angle = lidar_yaw_rad + angle_min + i * angle_increment
        angle = math.atan2(math.sin(angle), math.cos(angle))

        if not sector_min <= angle <= sector_max:
            continue

        points.append((
            dx + distance * math.cos(angle),
            dy + distance * math.sin(angle),
        ))

    return points


# Transform base_footprint points into map coordinates
def transform_points(
    points: Sequence[Point],
    pose: Pose,
) -> list[Point]:
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)

    return [
        (x + c * px - s * py, y + s * px + c * py)
        for px, py in points
    ]


# Return the nearest finite wall and its signed point-to-line residual
def _closest_wall(
    point: Point,
    walls: Mapping[str, Segment],
):
    px, py = point
    closest = None

    for name, (a, b) in walls.items():
        vx = b[0] - a[0]
        vy = b[1] - a[1]
        length_sq = vx * vx + vy * vy

        if length_sq <= 0:
            continue

        wx = px - a[0]
        wy = py - a[1]

        # Reject projections beyond the finite wall segment
        t = (wx * vx + wy * vy) / length_sq

        if not 0 <= t <= 1:
            continue

        length = math.sqrt(length_sq)
        nx, ny = -vy / length, vx / length
        residual = nx * wx + ny * wy

        if closest is None or abs(residual) < abs(closest[1]):
            closest = (name, residual, (nx, ny))

    return closest


# Calculate signed point-to-wall residuals for a candidate pose
def wall_residuals(
    points: Sequence[Point],
    pose: Pose,
    walls: Mapping[str, Segment],
) -> list[tuple[str, float] | None]:
    residuals = []

    for point in transform_points(points, pose):
        match = _closest_wall(point, walls)

        if match is None:
            residuals.append(None)
        else:
            name, residual, _ = match
            residuals.append((name, residual))

    return residuals


# Calculate inlier residuals and their derivatives with respect to x, y, yaw
def _linearise(
    points,
    pose,
    walls,
    max_distance,
):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    rows = []

    for px, py in points:
        rx = c * px - s * py
        ry = s * px + c * py

        wall = _closest_wall((x + rx, y + ry), walls)

        if wall is None:
            continue

        name, residual, (nx, ny) = wall

        if abs(residual) > max_distance:
            continue

        jacobian = (
            nx,
            ny,
            -nx * ry + ny * rx,
        )

        rows.append((name, residual, jacobian))

    return rows


# Apply the Huber loss to reduce the effect of outlier measurements
def _huber(residual, delta):
    absolute = abs(residual)

    if absolute <= delta:
        return 0.5 * residual * residual

    return delta * (absolute - 0.5 * delta)


# Calculate a robust objective with a fixed penalty for unmatched points
def _score(
    points,
    pose,
    walls,
    max_distance,
    huber_delta,
):
    rows = _linearise(points, pose, walls, max_distance)

    outlier_penalty = _huber(max_distance, huber_delta)

    score = outlier_penalty * (len(points) - len(rows))
    score += sum(
        _huber(residual, huber_delta)
        for _, residual, _ in rows
    )

    return score


# Solve a three-variable linear system using Gaussian elimination
def _solve_three(matrix, vector):
    rows = [
        list(matrix[i]) + [vector[i]]
        for i in range(3)
    ]

    for col in range(3):
        pivot = max(
            range(col, 3),
            key=lambda row: abs(rows[row][col]),
        )

        if abs(rows[pivot][col]) < 1e-12:
            return None

        rows[col], rows[pivot] = rows[pivot], rows[col]

        scale = rows[col][col]

        for j in range(col, 4):
            rows[col][j] /= scale

        for row in range(3):
            if row == col:
                continue

            factor = rows[row][col]

            for j in range(col, 4):
                rows[row][j] -= factor * rows[col][j]

    return tuple(rows[i][3] for i in range(3))


# Refine one initial pose using robust Gauss-Newton optimisation
def _optimise(
    points,
    initial_pose,
    walls,
    max_distance,
    delta,
    max_iterations,
    position_tol,
    yaw_tol,
):
    pose = initial_pose
    iterations = 0

    for iteration in range(max_iterations):
        rows = _linearise(points, pose, walls, max_distance)

        if len(rows) < 3:
            break

        h = [[0.0] * 3 for _ in range(3)]
        g = [0.0] * 3

        # Assemble the weighted normal equations
        for _, residual, jac in rows:
            weight = min(
                1.0,
                delta / max(abs(residual), 1e-12),
            )

            for j in range(3):
                g[j] += weight * jac[j] * residual

                for k in range(3):
                    h[j][k] += weight * jac[j] * jac[k]

        # Small diagonal damping improves numerical stability
        for j in range(3):
            h[j][j] += 1e-4

        step = _solve_three(h, [-value for value in g])

        if step is None:
            break

        # Limit each update to avoid large changes in wall association
        scale = min(
            1.0,
            0.15 / max(abs(step[0]), abs(step[1]), 1e-12),
            math.radians(10) / max(abs(step[2]), 1e-12),
        )

        old_score = _score(
            points, pose, walls, max_distance, delta
        )

        accepted = False

        # Backtrack until the proposed update reduces the objective
        for factor in (1.0, 0.5, 0.25, 0.125, 0.0625):
            fraction = scale * factor

            next_yaw = pose[2] + fraction * step[2]
            next_yaw = math.atan2(
                math.sin(next_yaw),
                math.cos(next_yaw),
            )

            candidate = (
                pose[0] + fraction * step[0],
                pose[1] + fraction * step[1],
                next_yaw,
            )

            new_score = _score(
                points, candidate, walls, max_distance, delta
            )

            if new_score < old_score - 1e-12:
                pose = candidate
                accepted = True
                break

        if not accepted:
            break

        iterations = iteration + 1

        # Stop once translation and rotation updates are sufficiently small
        position_step = max(
            abs(fraction * step[0]),
            abs(fraction * step[1]),
        )
        rotation_step = abs(fraction * step[2])

        if position_step < position_tol and rotation_step < yaw_tol:
            break

    return pose, iterations


# Estimate the robot pose and assess the available wall evidence
def match_scan(
    points: Sequence[Point],
    initial_pose: Pose,
    walls: Mapping[str, Segment],
    config: Mapping,
    alternate_initial_poses: Sequence[Pose] = (),
) -> ScanMatchResult:
    params = config.get('scan_matching', config)

    max_distance = float(params['max_point_to_wall_m'])
    delta = float(params['huber_delta_m'])
    max_iterations = int(params['max_iterations'])
    position_tol = float(params['convergence_step_m'])
    yaw_tol = math.radians(float(params['convergence_step_deg']))
    min_wall = int(params['min_inliers_per_wall'])
    max_rms = float(params['max_inlier_rms_m'])

    if (
        not 0 < delta <= max_distance
        or max_iterations < 1
        or min_wall < 1
        or max_rms <= 0
    ):
        raise ValueError('invalid scan matching configuration')

    if not points or not walls:
        return ScanMatchResult(
            pose=initial_pose,
            rms_error_m=math.inf,
            inlier_count=0,
            wall_inliers={},
            iterations=0,
            valid=False,
            reason='no scan points or walls',
        )

    # Optimise from the main prior and any additional starting guesses
    candidates = []

    for seed in (initial_pose, *alternate_initial_poses):
        pose, iterations = _optimise(
            points,
            seed,
            walls,
            max_distance,
            delta,
            max_iterations,
            position_tol,
            yaw_tol,
        )

        score = _score(
            points, pose, walls, max_distance, delta
        )

        candidates.append((score, pose, iterations))

    _, pose, iterations = min(
        candidates,
        key=lambda candidate: candidate[0],
    )

    # Calculate the final inlier RMS and support from each wall
    rows = _linearise(
        points, pose, walls, max_distance
    )

    count = len(rows)

    rms = (
        math.sqrt(
            sum(residual ** 2 for _, residual, _ in rows) / count
        )
        if count
        else math.inf
    )

    counts = {name: 0 for name in walls}

    for name, _, _ in rows:
        counts[name] += 1

    supported_walls = [
        name
        for name, n in counts.items()
        if n >= min_wall
    ]

    # At least two non-parallel walls must support the estimate
    has_independent_walls = False

    for i, name_a in enumerate(supported_walls):
        a1, a2 = walls[name_a]
        da = (
            a2[0] - a1[0],
            a2[1] - a1[1],
        )

        for name_b in supported_walls[i + 1:]:
            b1, b2 = walls[name_b]
            db = (
                b2[0] - b1[0],
                b2[1] - b1[1],
            )

            norm = math.hypot(*da) * math.hypot(*db)

            if norm == 0:
                continue

            alignment = abs(
                (da[0] * db[0] + da[1] * db[1]) / norm
            )

            if alignment < 0.9:
                has_independent_walls = True
                break

        if has_independent_walls:
            break

    valid = has_independent_walls and rms <= max_rms

    reason = (
        'ok'
        if valid
        else 'insufficient independent wall support or high residual'
    )

    return ScanMatchResult(
        pose=pose,
        rms_error_m=rms,
        inlier_count=count,
        wall_inliers=counts,
        iterations=iterations,
        valid=valid,
        reason=reason,
    )
