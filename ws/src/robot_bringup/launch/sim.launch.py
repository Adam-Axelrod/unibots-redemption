"""Launch the simulated robot. Nothing here touches hardware.

    ros2 launch robot_bringup sim.launch.py
    ros2 launch robot_bringup sim.launch.py world:=furnished_room

Then open http://localhost:8080 and, in another terminal, run whatever you are
working on:  ros2 run robot_control square_driver
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world = LaunchConfiguration('world')
    port = LaunchConfiguration('viz_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='empty_room',
            description='empty_room | furnished_room | l_shaped_room '
                        '(add your own in robot_sim/world.py)'),
        DeclareLaunchArgument(
            'viz_port', default_value='8080',
            description='port for the browser visualiser'),

        Node(
            package='robot_sim',
            executable='sim_node',
            name='sim_node',
            output='screen',
            parameters=[{'world': world, 'viz_port': port}],
        ),

        # The sim publishes odom -> base_footprint itself. These two fill in the
        # rest of the tree so /scan can be transformed like it is on the robot.
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
    ])
