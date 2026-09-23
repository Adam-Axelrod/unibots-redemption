import os
from glob import glob
from setuptools import setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adam Axelrod',
    maintainer_email='adambaxelrod@gmail.com',
    description='Launch files for the simulator and the real robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_tf = robot_bringup.odom_tf:main',
        ],
    },
)
