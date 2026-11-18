"""Launch the simulated robot. Nothing here touches hardware.

    ros2 launch robot_bringup sim.launch.py
    ros2 launch robot_bringup sim.launch.py world:=furnished_room
    ros2 launch robot_bringup sim.launch.py drive:=diff    # wheels that cannot strafe
    ros2 launch robot_bringup sim.launch.py world:=competition_arena \
        start_x_m:=0.20 start_y_m:=-1.15 start_yaw_deg:=90  # move ground truth only

Then open http://localhost:8080 and, in another terminal, run whatever you are
working on:  ros2 run robot_control square_driver
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from robot_bringup.lidar_tf import lidar_transform


def generate_launch_description():
    world = LaunchConfiguration('world')
    port = LaunchConfiguration('viz_port')
    drive = LaunchConfiguration('drive')
    # Kept as strings so an omitted value stays '' rather than a number.
    start = {name: ParameterValue(LaunchConfiguration(name), value_type=str)
             for name in ('start_x_m', 'start_y_m', 'start_yaw_deg')}

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='empty_room',
            description='empty_room | furnished_room | l_shaped_room '
                        '(add your own in robot_sim/world.py)'),
        DeclareLaunchArgument(
            'viz_port', default_value='8080',
            description='port for the browser visualiser'),
        DeclareLaunchArgument(
            'drive', default_value='mecanum',
            description="mecanum | diff -- diff ignores linear.y, like wheels "
                        'that cannot strafe'),
        DeclareLaunchArgument(
            'start_x_m', default_value='',
            description='simulator start x, map frame, metres; give all three '
                        'start_* or none (none = the world\'s own start)'),
        DeclareLaunchArgument(
            'start_y_m', default_value='',
            description='simulator start y, map frame, metres'),
        DeclareLaunchArgument(
            'start_yaw_deg', default_value='',
            description='simulator start heading, map frame, degrees'),

        Node(
            package='robot_sim',
            executable='sim_node',
            name='sim_node',
            output='screen',
            parameters=[{'world': world, 'viz_port': port, 'drive': drive, **start}],
        ),

        # The sim publishes odom -> base_footprint itself. These two fill in the
        # rest of the tree so /scan can be transformed like it is on the robot.
        # The lidar mounting is shared with the real robot via sensors.yaml.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0.05', '0', '0', '0',
                       'base_footprint', 'base_link'],
        ),
        lidar_transform(),
    ])
