#!/usr/bin/env python3
"""sweep_planner -- cover a room with a back-and-forth "lawnmower" pattern.

The pattern, one lane at a time:

    drive forward until the lidar says a wall is close
    turn 90 deg, shift sideways by one lane width, turn 90 deg the same way
    drive back the other way, turning the opposite way at the next wall

It uses /scan to find the walls (so it does not need to know the room size in
advance) and /odom to measure turns and lane shifts.

Run it:   ros2 run robot_control sweep_planner
Watch it: http://localhost:8080
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from robot_control.util import yaw_from_quaternion, angle_diff


class SweepPlanner(Node):

    def __init__(self):
        super().__init__('sweep_planner')

        self.declare_parameter('speed', 0.25)          # m/s along a lane
        self.declare_parameter('turn_speed', 0.8)      # rad/s
        self.declare_parameter('lane_width', 0.4)      # m between lanes
        self.declare_parameter('wall_margin', 0.35)    # stop this far off a wall
        self.declare_parameter('front_arc', 0.35)      # rad, width of "ahead"

        self.speed = self.get_parameter('speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.lane_width = self.get_parameter('lane_width').value
        self.wall_margin = self.get_parameter('wall_margin').value
        self.front_arc = self.get_parameter('front_arc').value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)

        self.pose = None
        self.front = math.inf            # metres to the nearest thing ahead
        self.state = 'lane'
        self.turn_sign = 1.0             # +1 = turn left at the wall, -1 = right
        self.anchor = None
        self.target_yaw = 0.0
        self.lanes_done = 0

        self.create_timer(0.05, self.tick)
        self.get_logger().info('sweeping -- lanes %.2f m apart' % self.lane_width)

    # ------------------------------------------------------------------ input

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def on_scan(self, msg: LaserScan):
        """Nearest valid return within a narrow cone straight ahead."""
        nearest = math.inf
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r >= msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= self.front_arc:      # angle 0 == straight ahead
                nearest = min(nearest, r)
        self.front = nearest

    # ----------------------------------------------------------- state machine

    def tick(self):
        if self.pose is None:
            return

        x, y, yaw = self.pose
        if self.anchor is None:
            self.anchor = self.pose

        cmd = Twist()

        if self.state == 'lane':
            # Drive down the lane until a wall shows up ahead.
            if self.front > self.wall_margin:
                cmd.linear.x = self.speed
            else:
                self.lanes_done += 1
                self.get_logger().info(f'wall reached, lane {self.lanes_done} done')
                self._begin_turn(yaw)
                self.state = 'turn_a'

        elif self.state == 'turn_a':
            if self._turning(yaw, cmd):
                self.anchor = self.pose
                self.state = 'shift'

        elif self.state == 'shift':
            # Sidestep by one lane width, then turn to face back down the room.
            moved = math.hypot(x - self.anchor[0], y - self.anchor[1])
            if moved < self.lane_width and self.front > self.wall_margin:
                cmd.linear.x = self.speed
            else:
                self._begin_turn(yaw)
                self.state = 'turn_b'

        elif self.state == 'turn_b':
            if self._turning(yaw, cmd):
                self.turn_sign *= -1.0       # next wall, turn the other way
                self.anchor = self.pose
                self.state = 'lane'

        self.pub.publish(cmd)

    def _begin_turn(self, yaw):
        self.target_yaw = yaw + self.turn_sign * math.pi / 2.0

    def _turning(self, yaw, cmd):
        """Drive the turn; return True once we are pointing the right way."""
        error = angle_diff(self.target_yaw, yaw)
        if abs(error) > 0.03:
            cmd.angular.z = self.turn_speed * (1.0 if error > 0 else -1.0)
            return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SweepPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
