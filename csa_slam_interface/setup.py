from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'csa_slam_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', 
         [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='jack',
    maintainer_email='jack@jacklab.org',
    description='CSA SLAM Interface Node: publishes pose from poses.txt to /camera/pose',
    license='MIT',
    entry_points={
        'console_scripts': [
            'slam_pose_node = csa_slam_interface.slam_pose_node:main',
        ],
    },
)
