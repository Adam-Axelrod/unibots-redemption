#!/usr/bin/env python3
"""room_mapper -- rough room size from a single lidar sweep.

Takes one scan, projects every return into the robot's frame, and reports the
bounding box. This is NOT SLAM: it assumes the robot can see most of the room
from where it is standing, and it will under-report in an L-shaped room or
anywhere the walls are occluded.

Publishes [width, length] in metres on /room_size.

Run it:  ros2 run robot_control room_mapper
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class RoomMapper(Node):

    def __init__(self):
        super().__init__('room_mapper')

        self.declare_parameter('report_every', 2.0)   # seconds between reports

        self.pub = self.create_publisher(Float32MultiArray, 'room_size', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)

        self.latest = None
        self.create_timer(self.get_parameter('report_every').value, self.report)
        self.get_logger().info('measuring the room from /scan')

    def on_scan(self, msg: LaserScan):
        xs, ys = [], []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r >= msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            xs.append(r * math.cos(angle))
            ys.append(r * math.sin(angle))

        if len(xs) < 8:
            return                        # too few returns to mean anything
        self.latest = (max(xs) - min(xs), max(ys) - min(ys))

    def report(self):
        if self.latest is None:
            return
        width, length = self.latest
        msg = Float32MultiArray()
        msg.data = [float(width), float(length)]
        self.pub.publish(msg)
        self.get_logger().info(f'room is about {width:.2f} m x {length:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = RoomMapper()
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
