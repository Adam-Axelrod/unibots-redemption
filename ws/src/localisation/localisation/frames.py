
# ws/src/localisation/localisation/frames.py

from dataclasses import dataclass
from math import atan2, cos, sin
from pathlib import Path

import yaml


Point = tuple[float, float]
Pose = tuple[float, float, float]
Segment = tuple[Point, Point]

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "arena.yaml"


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

    def start_lateral_bounds(self, robot_width_m: float) -> tuple[float, float]:
        if not 0 < robot_width_m <= self.guide_separation_m:
            raise ValueError("robot_width_m must fit between the scoring guides")
        limit = (self.guide_separation_m - robot_width_m) / 2.0
        return -limit, limit


def load_arena_frame(path: str | Path = CONFIG_PATH) -> ArenaFrame:
    with Path(path).open(encoding="utf-8") as stream:
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
