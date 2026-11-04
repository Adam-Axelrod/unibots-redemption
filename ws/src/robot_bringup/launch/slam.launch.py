"""Start SLAM: slam_toolbox, building a map and correcting odometry drift.

Start the robot (or the simulator) first, then in another terminal:

    ros2 launch robot_bringup slam.launch.py
    ros2 run robot_control square_driver --ros-args -p use_slam:=true

It publishes /map and the map -> odom transform. This includes slam_toolbox's own
launch file rather than starting its node directly, because that also handles
whichever way the installed version needs to be brought up.

Needs the image rebuilt once to have slam_toolbox: add --build to docker compose up.

Check it worked:
    ros2 topic hz /scan_slam         # our tidied copy of /scan
    ros2 topic echo /map --once      # the map, after the robot has moved a little
    ros2 run tf2_ros tf2_echo map odom
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('robot_bringup'),
                          'config', 'slam_toolbox.yaml')
    slam_launch = os.path.join(get_package_share_directory('slam_toolbox'),
                               'launch', 'online_async_launch.py')

    return LaunchDescription([
        Node(
            package='robot_bringup',
            executable='scan_for_slam',
            name='scan_for_slam',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={'slam_params_file': params, 'use_sim_time': 'false'}.items(),
        ),
    ])
