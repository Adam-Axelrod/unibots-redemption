from setuptools import setup

package_name = 'robot_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    # index.html lives inside the package so viz_server.py can find it next to itself
    package_data={package_name: ['web/*.html']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adam Axelrod',
    maintainer_email='adambaxelrod@gmail.com',
    description='2D simulator standing in for the Yahboom X3, with a browser visualiser.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sim_node = robot_sim.sim_node:main',
        ],
    },
)
