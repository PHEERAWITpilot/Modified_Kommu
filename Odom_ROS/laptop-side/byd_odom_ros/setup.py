from setuptools import setup
import os
from glob import glob

package_name = 'byd_odom_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kommu.AI',
    maintainer_email='dev@example.com',
    description='BYD Dolphin live odometry bridge to ROS2 / RViz',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_node = byd_odom_ros.odom_node:main',
        ],
    },
)
