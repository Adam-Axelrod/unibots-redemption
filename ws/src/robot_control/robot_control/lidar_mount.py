"""Which way the lidar faces on the robot.

/scan's angles are the lidar's own: its 0 deg is wherever the lidar's 0 deg
points, which on our robot is out of the right side. The launch files say how
it is mounted, as the base_link -> laser_frame transform, and this reads it, so
that every node agrees with slam_toolbox and nobody hard-codes the angle.

    mount = LidarMount(node)
    yaw = mount.yaw(msg.header.frame_id)   # None until the transform arrives
    angle_on_robot = msg.angle_min + i * msg.angle_increment + yaw

Until the transform arrives a scan could point anywhere: callers must not drive.
"""
import math

from rclpy.time import Time


class LidarMount:

    def __init__(self, node):
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self._yaw = None

    def yaw(self, frame='laser_frame'):
        """The lidar's yaw on the robot in radians, or None if not known yet.
        It is a fixed mounting, so once known it is kept."""
        if self._yaw is None:
            from tf2_ros import TransformException
            try:
                q = self.buffer.lookup_transform('base_footprint', frame or 'laser_frame',
                                                 Time()).transform.rotation
            except TransformException:
                return None
            self._yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return self._yaw
