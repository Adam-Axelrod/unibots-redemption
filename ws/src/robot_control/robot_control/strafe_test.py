#!/usr/bin/env python3
"""strafe_test -- strafe right, strafe left, then stop.

Run it:   ros2 run robot_control strafe_test
Slower:   ros2 run robot_control strafe_test --ros-args -p speed:=0.1

Stays stopped once it has finished. Ctrl-C to exit.
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist


class StrafeTest(Node):

    def __init__(self):
        super().__init__('strafe_test')

        self.declare_parameter('speed', 0.15)      # m/s
        self.declare_parameter('duration', 2.0)    # seconds each way

        self.speed = self.get_parameter('speed').value
        self.duration = self.get_parameter('duration').value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.start = None
        self.phase = None

        self.create_timer(0.05, self.tick)   # 20 Hz control loop

    def tick(self):
        # Wait for the robot to be listening, or the first leg would be cut short.
        if self.start is None:
            if self.pub.get_subscription_count() == 0:
                return
            self.start = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.start).nanoseconds / 1e9

        cmd = Twist()
        if elapsed < self.duration:
            phase = 'strafing right'
            cmd.linear.y = -self.speed       # +y is left
        elif elapsed < 2.0 * self.duration:
            phase = 'strafing left'
            cmd.linear.y = self.speed
        else:
            phase = 'stopped, Ctrl-C to exit'

        if phase != self.phase:
            self.phase = phase
            self.get_logger().info(phase)

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = StrafeTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C. rclpy's SIGINT handler shuts the context down before spin()
        # returns, so this arrives as ExternalShutdownException rather than
        # KeyboardInterrupt -- catching only the latter exits with a traceback.
        pass
    finally:
        # Only possible if we are exiting for our own reasons -- after Ctrl-C the
        # context is already gone and the 0.5 s command watchdog in the sim and
        # in the ESP32 firmware is what stops the robot. See CLAUDE.md.
        if rclpy.ok():
            node.pub.publish(Twist())        # always stop the robot on exit
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
