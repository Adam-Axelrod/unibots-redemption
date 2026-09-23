"""Small helpers shared by the control nodes. Nothing ROS-specific to learn here."""
import math


def yaw_from_quaternion(q):
    """Pull the 2D heading out of an Odometry quaternion, in radians."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def angle_diff(target, current):
    """Shortest signed angle from `current` to `target`, in [-pi, pi].

    Use this instead of (target - current) or the robot will sometimes spin
    the long way round when the angle wraps past +/-180 degrees.
    """
    return math.atan2(math.sin(target - current), math.cos(target - current))
