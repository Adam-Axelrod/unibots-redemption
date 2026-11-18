
# ws/src/localisation/localisation/frames.py

from dataclasses import dataclass
from math import atan2, cos, isfinite, radians, sin
from pathlib import Path

import yaml


Point = tuple[float, float]
Pose = tuple[float, float, float]
Segment = tuple[Point, Point]

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
CONFIG_PATH = CONFIG_DIR / "arena.yaml"
MATCH_OFFSET_KEYS = ("inward_offset_m", "lateral_offset_m", "yaw_deg")


def wrap_angle(angle_rad: float) -> float:
    return atan2(sin(angle_rad), cos(angle_rad))


@dataclass(frozen=True)
class ArenaFrame:
    frame_id: str
    forward_axis: str
    forward_sign: int
    side_length_m: float
    guide_separation_m: float

    def __post_init__(self) -> None:
        if not self.frame_id:
            raise ValueError("frame_id must not be empty")
        if self.forward_axis not in ("x", "y"):
            raise ValueError("forward_axis must be x or y")
        if type(self.forward_sign) is not int or self.forward_sign not in (-1, 1):
            raise ValueError("forward_sign must be +1 or -1")
        if self.side_length_m <= 0:
            raise ValueError("inner_side_length_m must be positive")
        if not 0 < self.guide_separation_m <= self.side_length_m:
            raise ValueError("inner_edge_separation_m must fit inside the arena")

    @property
    def forward_vector(self) -> Point:
        if self.forward_axis == "x":
            return float(self.forward_sign), 0.0
        return 0.0, float(self.forward_sign)

    @property
    def forward_yaw_rad(self) -> float:
        fx, fy = self.forward_vector
        return atan2(fy, fx)

    # Arena coordinates are (forward, left), centred on the arena.
    def arena_to_map(self, forward_m: float, left_m: float) -> Point:
        fx, fy = self.forward_vector
        return fx * forward_m - fy * left_m, fy * forward_m + fx * left_m

    def map_to_arena(self, x_m: float, y_m: float) -> Point:
        fx, fy = self.forward_vector
        return fx * x_m + fy * y_m, -fy * x_m + fx * y_m

    # Headings are anticlockwise from the arena's forward direction.
    def pose_to_map(self, forward_m: float, left_m: float, heading_rad: float) -> Pose:
        x, y = self.arena_to_map(forward_m, left_m)
        return x, y, wrap_angle(self.forward_yaw_rad + heading_rad)

    def pose_to_arena(self, x_m: float, y_m: float, yaw_rad: float) -> Pose:
        forward, left = self.map_to_arena(x_m, y_m)
        return forward, left, wrap_angle(yaw_rad - self.forward_yaw_rad)

    # Walls are fixed at +/- half the configured inner side length.
    def wall_segments(self) -> dict[str, Segment]:
        h = self.side_length_m / 2.0

        arena_walls = {
            "home": ((-h, -h), (-h, h)),
            "left": ((-h, h), (h, h)),
            "opposite": ((h, h), (h, -h)),
            "right": ((h, -h), (-h, -h)),
        }

        return {
            name: (self.arena_to_map(*a), self.arena_to_map(*b))
            for name, (a, b) in arena_walls.items()
        }

    # The start pose is a prior; LiDAR estimates the actual lateral position.
    def nominal_start_pose(self, robot_length_m: float) -> Pose:
        if not 0 < robot_length_m < self.side_length_m:
            raise ValueError("robot_length_m must fit inside the arena")

        forward = -self.side_length_m / 2.0 + robot_length_m / 2.0
        return self.pose_to_map(forward, 0.0, 0.0)

    # Offsets are inward, to the left facing inward, and anticlockwise from the nominal start.
    def match_relative_pose(
        self,
        robot_length_m: float,
        inward_m: float,
        lateral_m: float,
        yaw_rad: float,
    ) -> Pose:
        forward, left, heading = self.pose_to_arena(*self.nominal_start_pose(robot_length_m))
        return self.pose_to_map(forward + inward_m, left + lateral_m, heading + yaw_rad)

    def start_lateral_bounds(self, robot_width_m: float) -> tuple[float, float]:
        if not 0 < robot_width_m <= self.guide_separation_m:
            raise ValueError("robot_width_m must fit between the scoring guides")

        limit = (self.guide_separation_m - robot_width_m) / 2.0
        return -limit, limit


# Use the installed ROS configuration when available.
def get_config_path(name: str) -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        package_share = get_package_share_directory("localisation")
        return Path(package_share) / "config" / name
    except (ImportError, LookupError):
        return CONFIG_DIR / name


def get_arena_config_path() -> Path:
    return get_config_path("arena.yaml")


def load_match_config(path: str | Path | None = None) -> dict:
    config_path = Path(path) if path is not None else get_config_path("match.yaml")

    with config_path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


# Map-frame poses for a match.yaml section such as start or end
def match_pose(arena: ArenaFrame, robot_length_m: float, match: dict, section: str) -> Pose:
    offsets = match.get(section) if isinstance(match, dict) else None
    if not isinstance(offsets, dict):
        raise ValueError(f"match config needs a {section} section")

    values = []
    for key in MATCH_OFFSET_KEYS:
        value = offsets.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{section}.{key} must be a number, got {value!r}")
        if not isfinite(value):
            raise ValueError(f"{section}.{key} must be finite")
        values.append(float(value))

    inward, lateral, yaw_deg = values
    return arena.match_relative_pose(robot_length_m, inward, lateral, radians(yaw_deg))


def load_arena_frame(path: str | Path | None = None) -> ArenaFrame:
    config_path = Path(path) if path is not None else get_arena_config_path()

    with config_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    frame = config["frame"]
    walls = config["walls"]
    guides = config["scoring_guides"]

    return ArenaFrame(
        frame_id=frame["frame_id"],
        forward_axis=frame["forward_axis"],
        forward_sign=frame["forward_sign"],
        side_length_m=float(walls["inner_side_length_m"]),
        guide_separation_m=float(guides["inner_edge_separation_m"]),
    )


if __name__ == "__main__":
    arena = load_arena_frame()

    print("Forward yaw (rad):", arena.forward_yaw_rad)
    print("Nominal start pose:", arena.nominal_start_pose(0.200))
    print("Lateral start bounds:", arena.start_lateral_bounds(0.200))
    print("Wall segments:", arena.wall_segments())
