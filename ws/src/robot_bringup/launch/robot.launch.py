"""Launch the REAL robot. Run this on the Raspberry Pi, not on your laptop.

Before this will do anything, the micro-ROS agent must be running in another
terminal (see scripts/start_agent.sh) -- that is what puts the ESP32's topics
on the ROS graph.

    ros2 launch robot_bringup robot.launch.py
    ros2 launch robot_bringup robot.launch.py lidar:=false       # no lidar
    ros2 launch robot_bringup robot.launch.py lidar_port:=/dev/ttyACM0

Check it worked:
    ros2 topic echo /scan --once     # lidar is alive
    ros2 topic echo /odom --once     # ESP32 + agent are alive
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar = LaunchConfiguration('lidar')
    lidar_port = LaunchConfiguration('lidar_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar', default_value='true',
            description='start the MS200 lidar driver'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/oradar',
            description='lidar serial device. /dev/oradar is the symlink made '
                        'by oradar.rules; fall back to /dev/ttyACM0 if the '
                        'udev rule is not installed'),

        # Oradar/Orbbec MS200.
        #
        # We start the driver's executable directly instead of including its own
        # ms200_scan.launch.py, for two reasons:
        #   1. That file is written in Foxy-era syntax (node_executable=,
        #      node_name=), which was REMOVED in Humble -- it raises TypeError.
        #   2. It also publishes its own base_link->lidar transform at 0.18 m,
        #      which would fight with the static transform below.
        #
        # frame_id is forced to laser_frame (the driver defaults to 'lidar') so
        # that the frame names are identical in simulation and on the hardware.
        Node(
            package='oradar_lidar',
            executable='oradar_scan',
            name='ms200',
            output='screen',
            condition=IfCondition(lidar),
            parameters=[{
                'device_model': 'MS200',
                'frame_id': 'laser_frame',
                'scan_topic': '/scan',
                'port_name': lidar_port,
                'baudrate': 230400,
                'angle_min': 0.0,
                'angle_max': 360.0,
                'range_min': 0.05,
                'range_max': 20.0,
                'clockwise': False,
                'motor_speed': 10,
            }],
        ),

        # The ESP32 publishes /odom_raw; this makes it look like the simulator.
        Node(
            package='robot_bringup',
            executable='odom_tf',
            name='odom_tf',
            output='screen',
        ),

        # Where the sensors sit on the chassis. Measured from the Yahboom X3.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0.05', '0', '0', '0',
                       'base_footprint', 'base_link'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['-0.0046412', '0', '0.094079', '0', '0', '0',
                       'base_link', 'laser_frame'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['-0.002999', '-0.0030001', '0.031701', '0', '0', '0',
                       'base_link', 'imu_frame'],
        ),
    ])
