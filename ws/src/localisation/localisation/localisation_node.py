
# ws/src/localisation/localisation/localisation_node.py

import math

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from localisation.frames import get_config_path, load_arena_frame, load_match_config, match_pose
from localisation.scan_matcher import match_scan, preprocess_scan
from localisation.sensors import lidar_mount, load_sensor_config


# Load the scan matching configuration from the installed ROS package
def load_localisation_config():
    with get_config_path('localisation.yaml').open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


# Convert configured planar uncertainty into a ROS covariance matrix
def pose_covariance(arena, config):
    sigma = config['initial_pose_sigma']

    forward_var = float(sigma['inward_m']) ** 2
    left_var = float(sigma['lateral_m']) ** 2
    yaw_var = math.radians(float(sigma['yaw_deg'])) ** 2

    fx, fy = arena.forward_vector

    covariance = [0.0] * 36

    covariance[0] = fx * fx * forward_var + fy * fy * left_var
    covariance[1] = fx * fy * (forward_var - left_var)
    covariance[6] = covariance[1]
    covariance[7] = fy * fy * forward_var + fx * fx * left_var

    # Height, roll and pitch cannot be estimated from this 2D scan matcher
    covariance[14] = 1e6
    covariance[21] = 1e6
    covariance[28] = 1e6

    covariance[35] = yaw_var

    return covariance


class LocalisationNode(Node):

    # Initialise the arena, sensor mounting, prior and ROS interfaces
    def __init__(self):
        super().__init__('localisation_node')

        self.arena = load_arena_frame()
        self.sensors = load_sensor_config()
        self.config = load_localisation_config()
        self.match = load_match_config()

        self.walls = self.arena.wall_segments()

        lidar = self.sensors['lidar']
        translation, rotation = lidar_mount(self.sensors)

        self.lidar_translation = translation
        self.lidar_yaw = rotation[2]
        self.lidar_frame = lidar['frame_id']
        self.usable_sector = lidar['usable_sector_deg']

        self.footprint = tuple(
            float(v)
            for v in self.sensors['robot']['footprint_m']
        )

        # The prior comes from match.yaml only, never from simulator truth
        self.initial_pose = match_pose(
            self.arena, self.footprint[0], self.match, 'start'
        )
        self.home_target = match_pose(
            self.arena, self.footprint[0], self.match, 'end'
        )

        # Initial guesses cover the permitted lateral starting positions
        forward, _, heading = self.arena.pose_to_arena(
            *self.initial_pose
        )

        lateral_min, lateral_max = self.arena.start_lateral_bounds(
            self.footprint[1]
        )

        self.initial_seeds = [
            self.arena.pose_to_map(forward, lateral, heading)
            for lateral in (
                lateral_min,
                lateral_min / 2.0,
                lateral_max / 2.0,
                lateral_max,
            )
        ]

        self.previous_pose = None
        self.last_validity = None

        # These are provisional covariance values, not a fitted posterior
        self.covariance = pose_covariance(
            self.arena,
            self.config,
        )

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/localisation/pose',
            10,
        )

        self.valid_pub = self.create_publisher(
            Bool,
            '/localisation/valid',
            10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            lidar['topic'],
            self.on_scan,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'LiDAR-only localisation ready: '
            f'map={self.arena.frame_id}, '
            f'scan={lidar["topic"]}, '
            f'initial_pose={self.initial_pose}'
        )

    # Report localisation validity and log only when its state changes
    def publish_validity(self, valid, reason=''):
        message = Bool()
        message.data = bool(valid)
        self.valid_pub.publish(message)

        if valid == self.last_validity:
            return

        self.last_validity = valid

        if valid:
            self.get_logger().info('Localisation valid')
        else:
            self.get_logger().warning(
                f'Localisation invalid: {reason}'
            )

    # Process each LiDAR scan and estimate the robot's map pose
    def on_scan(self, scan):
        if scan.header.frame_id.lstrip('/') != self.lidar_frame:
            self.publish_validity(
                False,
                f'unexpected scan frame {scan.header.frame_id}',
            )
            return

        try:
            points = preprocess_scan(
                ranges=scan.ranges,
                angle_min=scan.angle_min,
                angle_increment=scan.angle_increment,
                lidar_translation_m=self.lidar_translation,
                lidar_yaw_rad=self.lidar_yaw,
                usable_sector_deg=self.usable_sector,
                range_min=scan.range_min,
                range_max=scan.range_max,
            )

            # Use the home-wall prior only until the first valid match
            if self.previous_pose is None:
                initial_pose = self.initial_pose
                alternate_poses = self.initial_seeds
            else:
                initial_pose = self.previous_pose
                alternate_poses = ()

            result = match_scan(
                points=points,
                initial_pose=initial_pose,
                walls=self.walls,
                config=self.config,
                alternate_initial_poses=alternate_poses,
            )

        except (ValueError, TypeError, ArithmeticError) as error:
            self.publish_validity(False, str(error))
            return

        if not result.valid:
            self.publish_validity(False, result.reason)
            return

        self.previous_pose = result.pose

        self.publish_pose(
            result.pose,
            scan.header.stamp,
        )

        self.publish_validity(True)

    # Publish the estimated x, y and yaw in the fixed map frame
    def publish_pose(self, pose, stamp):
        x, y, yaw = pose

        message = PoseWithCovarianceStamped()

        message.header.stamp = stamp
        message.header.frame_id = self.arena.frame_id

        message.pose.pose.position.x = float(x)
        message.pose.pose.position.y = float(y)
        message.pose.pose.position.z = 0.0

        message.pose.pose.orientation.x = 0.0
        message.pose.pose.orientation.y = 0.0
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)

        message.pose.covariance = self.covariance

        self.pose_pub.publish(message)


# Start the localisation ROS node
def main(args=None):
    rclpy.init(args=args)
    node = LocalisationNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
