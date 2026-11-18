
# ws/src/localisation/launch/localisation.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node


# Launch LiDAR-only localisation against the fixed arena walls
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='localisation',
            executable='localisation_node',
            name='localisation_node',
            output='screen',
        ),
    ])
