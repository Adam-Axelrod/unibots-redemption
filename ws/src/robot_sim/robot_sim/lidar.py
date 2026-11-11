# ws/src/robot_sim/robot_sim/lidar.py

import math

from robot_sim.world import raycast


# Transform the LiDAR mounting offset and yaw into the world frame
def lidar_world_pose(robot_pose, translation_m, yaw_rad):
    x, y, theta = robot_pose
    dx, dy = translation_m[0], translation_m[1]
    c, s = math.cos(theta), math.sin(theta)
    return x + c * dx - s * dy, y + s * dx + c * dy, theta + yaw_rad


# Return evenly spaced beam angles over a full turn, in the LiDAR frame
def beam_angles(beams):
    increment = 2.0 * math.pi / beams
    return [-math.pi + i * increment for i in range(beams)]


# Raycast every beam from the LiDAR's world pose
def simulate_ranges(lidar_pose, beams, segments, range_max):
    lx, ly, lyaw = lidar_pose
    return [
        float(raycast((lx, ly), lyaw + a, segments, range_max))
        for a in beam_angles(beams)
    ]
