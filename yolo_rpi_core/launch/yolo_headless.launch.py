#!/usr/bin/env python3
"""
Headless launch file for RPi4 deployment.

Starts camera and YOLO nodes only (no GUI viewer).
Use remote rqt_image_view from your PC to visualize.

Usage:
    ros2 launch yolo_rpi_core yolo_headless.launch.py
    ros2 launch yolo_rpi_core yolo_headless.launch.py video_device:=/dev/video1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for headless YOLO pipeline."""

    pkg_share = get_package_share_directory('yolo_rpi_core')
    default_params_file = os.path.join(pkg_share, 'config', 'yolo_params.yaml')

    # -------------------------------------------------------------------------
    # Launch Arguments
    # -------------------------------------------------------------------------
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='V4L2 camera device path'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolo11n.pt',
        description='Path to YOLO model file'
    )

    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.5',
        description='Confidence threshold for detections'
    )

    # -------------------------------------------------------------------------
    # Node 1: V4L2 Camera
    # -------------------------------------------------------------------------
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': [640, 480],
        }],
    )

    # -------------------------------------------------------------------------
    # Node 2: YOLO Detection
    # -------------------------------------------------------------------------
    yolo_node = Node(
        package='yolo_rpi_core',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[
            default_params_file,
            {
                'model_path': LaunchConfiguration('model_path'),
                'device': 'cpu',
                'conf_threshold': LaunchConfiguration('conf_threshold'),
            }
        ],
    )

    return LaunchDescription([
        video_device_arg,
        model_path_arg,
        conf_threshold_arg,
        camera_node,
        yolo_node,
    ])
