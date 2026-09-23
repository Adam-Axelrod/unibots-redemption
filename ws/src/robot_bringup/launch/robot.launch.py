"""Launch the REAL robot. Run this on the Raspberry Pi, not on your laptop.

Before this will do anything, the micro-ROS agent must be running in another
terminal (see scripts/start_agent.sh) -- that is what puts the ESP32's topics
on the ROS graph.

    ros2 launch robot_bringup robot.launch.py
    ros2 launch robot_bringup robot.launch.py lidar:=false      # skip the lidar

Check it worked:
    ros2 topic echo /scan --once     # lidar is alive
    ros2 topic echo /odom --once     # ESP32 + agent are alive
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar = LaunchConfiguration('lidar')
    lidar_launch = LaunchConfiguration('lidar_launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar', default_value='true',
            description='start the MS200 lidar driver'),
        DeclareLaunchArgument(
            'lidar_launch', default_value='ms200_scan.launch.py',
            description='launch file inside the oradar_lidar package. If you '
                        'get "file not found", list the real names with: '
                        'ls $(ros2 pkg prefix oradar_lidar)/share/oradar_lidar/launch'),

        # Oradar/Orbbec MS200. Publishes /scan with frame_id laser_frame, which
        # is what the static transform below expects. Its defaults are
        # /dev/ttyACM0 at 230400 baud -- note that is a different device class
        # from the ESP32 on /dev/ttyUSB0, so the two do not collide.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('oradar_lidar'), 'launch', lidar_launch])),
            condition=IfCondition(lidar),
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
