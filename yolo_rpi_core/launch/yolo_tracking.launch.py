#!/usr/bin/env python3
"""
Launch file for the full YOLO + Object Tracking pipeline.

Starts camera, YOLO detection, centroid tracker, and actuator nodes.
By default, the dummy actuator is used. Pass use_real_motor:=true to
activate the real serial motor actuator (hardware must be connected).

Usage::

    # Default (dummy actuator — no hardware needed)
    ros2 launch yolo_rpi_core yolo_tracking.launch.py

    # Real motor hardware
    ros2 launch yolo_rpi_core yolo_tracking.launch.py use_real_motor:=true

    # Real motor with custom serial port
    ros2 launch yolo_rpi_core yolo_tracking.launch.py \\
        use_real_motor:=true serial_port:=/dev/ttyACM0

    # Class-specific tracking
    ros2 launch yolo_rpi_core yolo_tracking.launch.py tracking_target_class:=cup
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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

    use_real_motor_arg = DeclareLaunchArgument(
        'use_real_motor',
        default_value='false',
        description=(
            'Set to "true" to launch SerialMotorActuatorNode (real hardware). '
            'Default "false" launches DummyActuatorNode (log-only, no hardware).'
        ),
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial device path for the motor controller (use_real_motor only)',
    )

    simulation_mode_arg = DeclareLaunchArgument(
        'simulation_mode',
        default_value='false',
        description=(
            'Set to "true" to run SerialMotorActuatorNode in simulation mode '
            '(no serial I/O — useful for dev / CI without hardware).'
        ),
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
    # Node 4a: Dummy Actuator (default — no hardware)
    # -------------------------------------------------------------------------
    dummy_actuator_node = Node(
        package='yolo_rpi_core',
        executable='dummy_actuator_node',
        name='dummy_actuator_node',
        output='screen',
        parameters=[tracking_params],
        condition=UnlessCondition(LaunchConfiguration('use_real_motor')),
    )

    # -------------------------------------------------------------------------
    # Node 4b: Serial Motor Actuator (real hardware — use_real_motor:=true)
    # -------------------------------------------------------------------------
    serial_motor_actuator_node = Node(
        package='yolo_rpi_core',
        executable='serial_motor_actuator_node',
        name='serial_motor_actuator_node',
        output='screen',
        parameters=[
            tracking_params,
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'simulation_mode': LaunchConfiguration('simulation_mode'),
            },
        ],
        condition=IfCondition(LaunchConfiguration('use_real_motor')),
    )

    return LaunchDescription([
        video_device_arg,
        model_path_arg,
        conf_threshold_arg,
        tracking_target_arg,
        use_real_motor_arg,
        serial_port_arg,
        simulation_mode_arg,
        camera_node,
        yolo_node,
        tracker_node,
        dummy_actuator_node,
        serial_motor_actuator_node,
    ])

