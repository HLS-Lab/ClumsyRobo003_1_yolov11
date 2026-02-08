"""Setup configuration for yolo_rpi_core package."""

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo_rpi_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package resource marker
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        # NOTE: ultralytics, torch, opencv are pre-installed in the Docker container.
        # Do NOT add them here to avoid pulling GPU dependencies.
    ],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='user@example.com',
    description='YOLOv11 object detection node for Raspberry Pi 4',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_rpi_core.yolo_node:main',
        ],
    },
)
