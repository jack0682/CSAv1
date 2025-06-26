from setuptools import setup
import os
from glob import glob

package_name = 'csa_image_input'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/csa_image_input']),
        ('share/' + package_name + '/config', ['config/image_input.yaml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'opencv-python',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='jack',
    maintainer_email='jack@example.com',
    description='Image publisher node for CSA input layer',
    license='MIT',
    entry_points={
        'console_scripts': [
            'image_publisher_node = csa_image_input.image_publisher_node:main'
        ],
    },
)
