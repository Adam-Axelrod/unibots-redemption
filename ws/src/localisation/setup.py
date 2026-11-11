
# ws/src/localisation/setup.py

import os
from glob import glob

from setuptools import setup


package_name = 'localisation'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adam Axelrod',
    maintainer_email='adambaxelrod@gmail.com',
    description='Fixed-map localisation for the Unibots competition arena.',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
