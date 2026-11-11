from launch_ros.actions import Node

from localisation.sensors import lidar_mount, load_sensor_config


# Static base_footprint -> laser_frame transform from localisation's sensors.yaml
def lidar_transform():
    config = load_sensor_config()
    (x, y, z), (roll, pitch, yaw) = lidar_mount(config)
    return Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_footprint_to_laser',
        # Arguments: x y z yaw pitch roll parent child.
        arguments=[str(v) for v in (x, y, z, yaw, pitch, roll)]
                  + [config['robot']['base_frame'], config['lidar']['frame_id']],
    )
