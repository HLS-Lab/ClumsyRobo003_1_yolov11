#!/usr/bin/env python3
"""
Launch file for YOLO object detection node.

This launch file starts the yolo_node with configurable parameters.
Parameters can be overridden via command line arguments.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for YOLO node."""
    
    # Get package share directory
    pkg_share = get_package_share_directory('yolo_rpi_core')
    
    # Default config file path
    default_config = os.path.join(pkg_share, 'config', 'yolo_params.yaml')
    
    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolo11n.pt',
        description='Path to YOLO model file'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description='Device for inference (cpu or cuda)'
    )
    
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.5',
        description='Confidence threshold for detections'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to parameter config file'
    )
    
    # YOLO Node
    yolo_node = Node(
        package='yolo_rpi_core',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'model_path': LaunchConfiguration('model_path'),
                'device': LaunchConfiguration('device'),
                'conf_threshold': LaunchConfiguration('conf_threshold'),
            }
        ],
        # Remap input topic if needed
        # remappings=[
        #     ('/image_raw', '/camera/image_raw'),
        # ]
    )
    
    return LaunchDescription([
        model_path_arg,
        device_arg,
        conf_threshold_arg,
        config_file_arg,
        yolo_node,
    ])
