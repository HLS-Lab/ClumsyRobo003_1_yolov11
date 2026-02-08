#!/usr/bin/env python3
"""
Launch file for YOLO detection node.

Usage:
    ros2 launch yolo_rpi_core yolo.launch.py
    ros2 launch yolo_rpi_core yolo.launch.py model_path:=yolo11s.pt conf_threshold:=0.7
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for YOLO node."""

    # Get package share directory
    pkg_share = get_package_share_directory('yolo_rpi_core')
    default_params_file = os.path.join(pkg_share, 'config', 'yolo_params.yaml')

    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolo11n.pt',
        description='Path to YOLO model file'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description='Inference device (cpu or cuda)'
    )

    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.5',
        description='Confidence threshold for detections'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to parameters YAML file'
    )

    # YOLO Node
    yolo_node = Node(
        package='yolo_rpi_core',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'model_path': LaunchConfiguration('model_path'),
                'device': LaunchConfiguration('device'),
                'conf_threshold': LaunchConfiguration('conf_threshold'),
            }
        ],
        remappings=[
            # Remap input topic if needed (uncomment and modify)
            # ('/image_raw', '/camera/image_raw'),
        ]
    )

    return LaunchDescription([
        model_path_arg,
        device_arg,
        conf_threshold_arg,
        params_file_arg,
        yolo_node,
    ])
