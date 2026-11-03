import os
from glob import glob

from setuptools import setup

package_name = 'tb3_obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='TurtleBot3 激光雷达避障 (Gazebo 仿真, ROS2 Humble)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'avoid_node = tb3_obstacle_avoidance.avoid_node:main',
        ],
    },
)
