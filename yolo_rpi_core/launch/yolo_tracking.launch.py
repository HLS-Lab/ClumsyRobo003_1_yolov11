#!/usr/bin/env python3
"""
Launch file for the full YOLO + Object Tracking pipeline.

Starts camera, YOLO detection, centroid tracker, and dummy actuator nodes.

Usage:
    ros2 launch yolo_rpi_core yolo_tracking.launch.py
    ros2 launch yolo_rpi_core yolo_tracking.launch.py tracking_target_class:=cup
    ros2 launch yolo_rpi_core yolo_tracking.launch.py video_device:=/dev/video1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the full tracking pipeline."""

    pkg_share = get_package_share_directory('yolo_rpi_core')
    yolo_params = os.path.join(pkg_share, 'config', 'yolo_params.yaml')
    tracking_params = os.path.join(
        pkg_share, 'config', 'tracking_params.yaml'
    )

    # -------------------------------------------------------------------------
    # Launch Arguments
    # -------------------------------------------------------------------------
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='V4L2 camera device path',
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolo11n.pt',
        description='Path to YOLO model file',
    )

    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.5',
        description='Confidence threshold for YOLO detections',
    )

    tracking_target_arg = DeclareLaunchArgument(
        'tracking_target_class',
        default_value='person',
        description='YOLO class name to track (use "all" for any)',
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
            yolo_params,
            {
                'model_path': LaunchConfiguration('model_path'),
                'device': 'cpu',
                'conf_threshold': LaunchConfiguration('conf_threshold'),
            },
        ],
    )

    # -------------------------------------------------------------------------
    # Node 3: Object Tracker
    # -------------------------------------------------------------------------
    tracker_node = Node(
        package='yolo_rpi_core',
        executable='tracker_node',
        name='tracker_node',
        output='screen',
        parameters=[
            tracking_params,
            {
                'tracking_target_class': LaunchConfiguration(
                    'tracking_target_class'
                ),
            },
        ],
    )

    # -------------------------------------------------------------------------
    # Node 4: Dummy Actuator
    # -------------------------------------------------------------------------
    actuator_node = Node(
        package='yolo_rpi_core',
        executable='dummy_actuator_node',
        name='dummy_actuator_node',
        output='screen',
        parameters=[tracking_params],
    )

    return LaunchDescription([
        video_device_arg,
        model_path_arg,
        conf_threshold_arg,
        tracking_target_arg,
        camera_node,
        yolo_node,
        tracker_node,
        actuator_node,
    ])
