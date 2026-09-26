#!/usr/bin/env python3
"""sim_node -- a 2D stand-in for the real robot.

It speaks exactly the same ROS topics as the hardware, so any node you write
against the simulator runs unchanged on the real car:

    subscribes  /cmd_vel   geometry_msgs/Twist     (your commands)
    publishes   /scan      sensor_msgs/LaserScan   (simulated lidar)
    publishes   /odom      nav_msgs/Odometry       (where the robot thinks it is)
    broadcasts  odom -> base_footprint TF

There is no physics engine. The robot is a point that integrates the velocity
you command, and the lidar is a raycast against line segments. That is enough
to test driving patterns, and it runs anywhere Python runs.

Run it:   ros2 launch robot_bringup sim.launch.py
Watch it: http://localhost:8080
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from robot_sim import world as world_lib
from robot_sim.viz_server import VizServer


def yaw_to_quaternion(yaw):
    """2D heading -> quaternion (only z and w are non-zero when rolling flat)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class SimNode(Node):

    def __init__(self):
        super().__init__('sim_node')

        # --- parameters (override: --ros-args -p world:=furnished_room) ---
        self.declare_parameter('world', 'empty_room')
        self.declare_parameter('rate', 20.0)           # Hz, physics + scan
        self.declare_parameter('scan_beams', 180)
        self.declare_parameter('scan_range_max', 12.0)
        self.declare_parameter('robot_radius', 0.12)   # metres, for collisions
        self.declare_parameter('cmd_timeout', 0.5)     # stop if commands stop
        self.declare_parameter('viz_port', 8080)

        world_name = self.get_parameter('world').value
        self.rate = self.get_parameter('rate').value
        self.beams = self.get_parameter('scan_beams').value
        self.range_max = self.get_parameter('scan_range_max').value
        self.radius = self.get_parameter('robot_radius').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        self.world = world_lib.load(world_name)
        self.segments = self.world['segments']

        # --- robot state ---
        self.x, self.y, self.theta = self.world['start']
        self.vx = self.vy = self.wz = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.trail = [(self.x, self.y)]
        self.collided = False

        # --- ROS interface: identical to the real robot ---
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- browser visualiser ---
        self.viz = VizServer(self.get_parameter('viz_port').value, self.world)
        self.viz.start()

        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self.step)

        self.get_logger().info(
            f"simulating '{world_name}' at {self.rate:.0f} Hz -- "
            f'watch it at http://localhost:{self.get_parameter("viz_port").value}')

    # ------------------------------------------------------------------ input

    def on_cmd_vel(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y      # mecanum wheels can strafe sideways
        self.wz = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def _browser_override(self):
        """Arrow-key driving from the web page, when there is no /cmd_vel."""
        cmd = self.viz.take_command()
        if cmd is not None:
            self.vx, self.vy, self.wz = cmd
            self.last_cmd_time = self.get_clock().now()

    # ------------------------------------------------------------- simulation

    def step(self):
        self._browser_override()

        # Safety: if nobody is commanding us, stop. The real robot does this too.
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if age > self.cmd_timeout:
            self.vx = self.vy = self.wz = 0.0

        # Integrate body velocities into the world frame (mecanum: vy is real).
        cos_t, sin_t = math.cos(self.theta), math.sin(self.theta)
        nx = self.x + (self.vx * cos_t - self.vy * sin_t) * self.dt
        ny = self.y + (self.vx * sin_t + self.vy * cos_t) * self.dt

        # Collision: refuse the move if it would put us inside a wall.
        if world_lib.min_distance((nx, ny), self.segments) > self.radius:
            self.x, self.y = nx, ny
            self.collided = False
        else:
            self.collided = True

        self.theta = math.atan2(math.sin(self.theta + self.wz * self.dt),
                                math.cos(self.theta + self.wz * self.dt))

        if math.hypot(self.x - self.trail[-1][0], self.y - self.trail[-1][1]) > 0.05:
            self.trail.append((self.x, self.y))
            if len(self.trail) > 4000:
                self.trail.pop(0)

        ranges = self.publish_scan()
        self.publish_odom()
        self.viz.update({
            'x': self.x, 'y': self.y, 'theta': self.theta,
            'vx': self.vx, 'vy': self.vy, 'wz': self.wz,
            'collided': self.collided,
            'trail': self.trail,
            'ranges': ranges,
            'angle_min': -math.pi,
            'angle_increment': 2.0 * math.pi / self.beams,
        })

    # ----------------------------------------------------------------- output

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / self.beams
        scan.range_min = 0.05
        scan.range_max = float(self.range_max)

        ranges = []
        for i in range(self.beams):
            # Beam angles are relative to the robot, so add its heading.
            angle = self.theta + scan.angle_min + i * scan.angle_increment
            d = world_lib.raycast((self.x, self.y), angle, self.segments, self.range_max)
            ranges.append(float(d))

        scan.ranges = ranges
        self.scan_pub.publish(scan)
        return ranges

    def publish_odom(self):
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(self.theta)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C. rclpy's SIGINT handler shuts the context down before spin()
        # returns, so this arrives as ExternalShutdownException rather than
        # KeyboardInterrupt -- catching only the latter exits with a traceback.
        pass
    finally:
        node.viz.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
