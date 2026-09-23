"""Launch the REAL robot. Run this on the Raspberry Pi, not on your laptop.

Before this will do anything, the micro-ROS agent must be running in another
terminal (see scripts/start_agent.sh) -- that is what puts the ESP32's topics
on the ROS graph.

    ros2 launch robot_bringup robot.launch.py

The lidar driver is NOT started here, because which package it is depends on
which lidar is fitted. Start it yourself, then check:  ros2 topic echo /scan
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
