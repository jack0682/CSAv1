from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'csa_yolo_inference'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'torch',
        'PyYAML',
        'rclpy',
        'cv_bridge',
    ],
    zip_safe=True,
    maintainer='jack',
    maintainer_email='jack@jacklab.org',
    description='CSA YOLOv5 + StrongSORT 기반 객체 인식 및 추적 노드.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_tracker_node = csa_yolo_inference.yolo_tracker_node:main',
        ],
    },
)
