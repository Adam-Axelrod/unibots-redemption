from setuptools import setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adam Axelrod',
    maintainer_email='adambaxelrod@gmail.com',
    description='Driving patterns and lidar logic. This is where you write your code.',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Add your own node here, then rebuild, or ros2 run will not find it.
            'square_driver = robot_control.square_driver:main',
            'sweep_planner = robot_control.sweep_planner:main',
            'room_mapper = robot_control.room_mapper:main',
        ],
    },
)
