from setuptools import setup
import os
from glob import glob

package_name = 'csa_utils'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),  # ✅ ROS2 index marker
        ('share/' + package_name, ['package.xml']),  # ✅ ROS2 metadata
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'matplotlib',
    ],
    zip_safe=True,
    maintainer='jack',
    maintainer_email='jack@jacklab.org',
    description='CSA Utility Library: logging, coordinate transform, ID tracking, scene graph visualization, and time sync.',
    license='MIT',
    entry_points={
        'console_scripts': [
            # utils only; no scripts
        ],
    },
)
