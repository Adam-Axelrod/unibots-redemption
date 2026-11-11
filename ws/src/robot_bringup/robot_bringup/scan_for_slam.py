#!/usr/bin/env python3
"""scan_for_slam -- /scan, tidied up the way slam_toolbox needs it, as /scan_slam.

Two things about the raw scan would trip slam_toolbox up:

    A full circle counts its start and end as one beam: 360 readings from -180
    to +180 deg. slam_toolbox works out how many readings to expect from the
    angles (361) and throws away every scan that does not match.

    The chassis blocks the lidar's rear, and returns off it look like a wall
    moving with the robot. Beams outside `lidar_view` of the robot's straight
    ahead are blanked, as in square_driver. Which beams those are depends on how
    the lidar is mounted (robot_control.lidar_mount).

The angles themselves stay as the lidar gave them: slam_toolbox reads the same
mounting transform and turns them itself.

Run by slam.launch.py.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from robot_control.lidar_mount import LidarMount


def tidy(msg, view, mount):
    """A copy of the scan with consistent angles, and only the beams within
    `view` radians of the robot's straight ahead. `mount` is the lidar's yaw
    on the robot."""
    out = LaserScan()
    out.header = msg.header
    out.angle_min = msg.angle_min
    out.angle_increment = msg.angle_increment
    out.angle_max = msg.angle_min + (len(msg.ranges) - 1) * msg.angle_increment
    out.time_increment = msg.time_increment
    out.scan_time = msg.scan_time
    out.range_min = msg.range_min
    out.range_max = msg.range_max
    ranges = []
    for i, r in enumerate(msg.ranges):
        a = msg.angle_min + i * msg.angle_increment + mount
        ahead = abs(math.atan2(math.sin(a), math.cos(a))) <= view
        ranges.append(float(r) if ahead else float('inf'))   # inf: no return here
    out.ranges = ranges
    return out


class ScanForSlam(Node):

    def __init__(self):
        super().__init__('scan_for_slam')
        self.declare_parameter('lidar_view', 100.0)    # deg either side of ahead it can see
        self.view = math.radians(self.get_parameter('lidar_view').value)
        self.mount = LidarMount(self)
        self.pub = self.create_publisher(LaserScan, 'scan_slam', qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg: LaserScan):
        yaw = self.mount.yaw(msg.header.frame_id)
        if yaw is None:
            self.get_logger().warn('no base_footprint -> laser_frame transform yet -- '
                                   'is robot.launch.py running?', throttle_duration_sec=2.0)
            return
        self.pub.publish(tidy(msg, self.view, yaw))


def main(args=None):
    rclpy.init(args=args)
    node = ScanForSlam()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C. rclpy's SIGINT handler shuts the context down before spin()
        # returns, so this arrives as ExternalShutdownException rather than
        # KeyboardInterrupt -- catching only the latter exits with a traceback.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
