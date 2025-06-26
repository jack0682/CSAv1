from setuptools import setup
import os
from glob import glob

package_name = 'csa_semantic_mapper'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'numpy',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='jack',
    maintainer_email='jack@jacklab.org',
    description='CSA Stage 1 Semantic Mapper Node',
    license='MIT',
    # tests_require 제거
    entry_points={
        'console_scripts': [
            'semantic_mapper_node = csa_semantic_mapper.semantic_mapper_node:main',
        ],
    },
)
