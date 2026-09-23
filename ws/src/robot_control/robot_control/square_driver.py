#!/usr/bin/env python3
"""square_driver -- drive a 1 m square. The simplest closed-loop example.

This is the file to copy when you want to try your own driving pattern. It
shows the whole shape of a control node in about 80 lines:

    1. subscribe to /odom so you know where you are
    2. run a state machine on a timer
    3. publish /cmd_vel to move

Run it:   ros2 run robot_control square_driver
Watch it: http://localhost:8080
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from robot_control.util import yaw_from_quaternion, angle_diff


class SquareDriver(Node):

    def __init__(self):
        super().__init__('square_driver')

        self.declare_parameter('side', 1.0)          # metres
        self.declare_parameter('speed', 0.25)        # m/s
        self.declare_parameter('turn_speed', 0.8)    # rad/s

        self.side = self.get_parameter('side').value
        self.speed = self.get_parameter('speed').value
        self.turn_speed = self.get_parameter('turn_speed').value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)

        self.pose = None                 # (x, y, yaw), filled in by on_odom
        self.state = 'driving'           # 'driving' or 'turning'
        self.anchor = None               # where the current leg started
        self.legs_done = 0

        self.create_timer(0.05, self.tick)   # 20 Hz control loop
        self.get_logger().info(f'driving a {self.side} m square')

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def tick(self):
        if self.pose is None:
            return                        # no odometry yet, stay put

        x, y, yaw = self.pose
        if self.anchor is None:
            self.anchor = self.pose

        cmd = Twist()

        if self.state == 'driving':
            travelled = math.hypot(x - self.anchor[0], y - self.anchor[1])
            if travelled < self.side:
                cmd.linear.x = self.speed
            else:
                self.state = 'turning'
                self.anchor = self.pose
                self.target_yaw = yaw + math.pi / 2.0

        elif self.state == 'turning':
            error = angle_diff(self.target_yaw, yaw)
            if abs(error) > 0.03:
                cmd.angular.z = self.turn_speed * (1.0 if error > 0 else -1.0)
            else:
                self.state = 'driving'
                self.anchor = self.pose
                self.legs_done += 1
                self.get_logger().info(f'corner {self.legs_done}')

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())        # always stop the robot on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
