#!/usr/bin/env python3
"""odom_tf -- real robot only. Turns the MCU's odometry into ROS's odometry.

The ESP32 publishes wheel odometry on /odom_raw, but it does not broadcast the
odom -> base_footprint transform that the rest of ROS expects. This node does
that, and republishes the message as /odom so control code sees the same topic
name in simulation and on the hardware.

The simulator publishes /odom and this TF itself, so it does NOT run this node.

Accuracy note: this is raw wheel odometry. Mecanum wheels slip sideways, so
heading drifts over a long run. If that becomes a problem, fuse the IMU with
robot_localization (see CLAUDE.md) -- but start here, it is far simpler.
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomTf(Node):

    def __init__(self):
        super().__init__('odom_tf')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(Odometry, 'odom_raw', self.on_odom_raw, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('republishing /odom_raw as /odom + TF')

    def on_odom_raw(self, msg: Odometry):
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose = msg.pose
        out.twist = msg.twist
        self.pub.publish(out)

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTf()
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
