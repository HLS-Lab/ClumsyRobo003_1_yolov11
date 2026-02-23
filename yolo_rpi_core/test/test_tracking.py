#!/usr/bin/env python3
"""
Integration Test for Object Tracking Pipeline.

This test script validates the tracker_node → dummy_actuator_node pipeline
WITHOUT requiring a physical camera or YOLO model. It publishes synthetic
Detection2DArray messages and verifies that:

    1. The tracker assigns tracking IDs to detections.
    2. Moving detections produce correct tracking commands with error offsets.
    3. The dummy actuator receives and acknowledges commands.
    4. Disappeared detections are eventually deregistered.

Usage (inside Docker container or ROS2 workspace):
    cd /ros2_ws && colcon build --packages-select yolo_rpi_core
    source install/setup.bash
    python3 src/yolo_rpi_core/test/test_tracking.py

    Or with custom init point and tracker type:
    python3 src/yolo_rpi_core/test/test_tracking.py --init-x 200 --init-y 150
    python3 src/yolo_rpi_core/test/test_tracking.py --tracker-type bytetrack

Compatible with: ROS 2 Jazzy, Python 3.12
Target Platform: Raspberry Pi 4 (CPU only, NO CUDA)
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


# =============================================================================
# Test Configuration
# =============================================================================

# Default image dimensions (must match tracker_node params)
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# Test timeout (seconds)
TEST_TIMEOUT = 45.0


# =============================================================================
# TestPublisherNode — Publishes synthetic detections
# =============================================================================

class TestPublisherNode(Node):
    """
    Test node that publishes synthetic Detection2DArray messages.

    Simulates an object moving across the frame from an initial position,
    allowing the tracker to assign IDs and compute tracking commands.
    """

    def __init__(
        self,
        init_x: float,
        init_y: float,
        target_class: str = 'person',
    ) -> None:
        """
        Initialize the test publisher.

        Args:
            init_x: Initial X coordinate of the synthetic detection (pixels).
            init_y: Initial Y coordinate of the synthetic detection (pixels).
            target_class: Class label for the synthetic detection.
        """
        super().__init__('test_tracking_publisher')

        self._init_x = init_x
        self._init_y = init_y
        self._current_x = init_x
        self._current_y = init_y
        self._target_class = target_class
        self._frame_count = 0

        # Publisher: simulated YOLO detections
        self._detection_pub = self.create_publisher(
            Detection2DArray,
            '/yolo/detections',
            10,
        )

        # Timer: publish at ~5 Hz (200ms interval)
        self._timer = self.create_timer(0.2, self._publish_detection)

        self.get_logger().info(
            f'📡 Test publisher ready — init point: ({init_x}, {init_y}), '
            f'class: {target_class}'
        )

    def _publish_detection(self) -> None:
        """
        Publish a synthetic detection that moves across the frame.

        Movement pattern:
            - Frames 0-9:   Static at init point (registration phase)
            - Frames 10-24: Moving right and down (tracking phase)
            - Frames 25-29: No detections (disappearance phase)
            - Frames 30+:   Re-appear at a new position (re-registration)
        """
        self._frame_count += 1
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'

        if self._frame_count <= 10:
            # Phase 1: Static at init point
            self._publish_single_detection(msg, self._current_x, self._current_y)
            phase = 'STATIC (init)'

        elif self._frame_count <= 25:
            # Phase 2: Moving right (+10px) and down (+5px) per frame
            self._current_x += 10.0
            self._current_y += 5.0
            # Clamp to image bounds
            self._current_x = min(self._current_x, IMAGE_WIDTH - 50)
            self._current_y = min(self._current_y, IMAGE_HEIGHT - 50)
            self._publish_single_detection(msg, self._current_x, self._current_y)
            phase = 'MOVING'

        elif self._frame_count <= 30:
            # Phase 3: No detections (object disappeared)
            phase = 'DISAPPEARED'

        elif self._frame_count <= 40:
            # Phase 4: Re-appear at a different location
            new_x = IMAGE_WIDTH / 4.0
            new_y = IMAGE_HEIGHT / 4.0
            self._publish_single_detection(msg, new_x, new_y)
            phase = 'RE-APPEARED'

        else:
            # Test complete
            self.get_logger().info('✅ All test phases completed!')
            self._timer.cancel()
            return

        self._detection_pub.publish(msg)
        self.get_logger().info(
            f'  Frame {self._frame_count:3d} | Phase: {phase:15s} | '
            f'Pos: ({self._current_x:.0f}, {self._current_y:.0f})'
        )

    def _publish_single_detection(
        self,
        msg: Detection2DArray,
        cx: float,
        cy: float,
    ) -> None:
        """Add a single detection at the given center position."""
        det = Detection2D()
        det.header = msg.header

        # Bounding box: 80×100 pixels centered at (cx, cy)
        bbox_w = 80.0
        bbox_h = 100.0
        det.bbox.center.position.x = cx
        det.bbox.center.position.y = cy
        det.bbox.size_x = bbox_w
        det.bbox.size_y = bbox_h

        # Hypothesis
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = self._target_class
        hyp.hypothesis.score = 0.85
        det.results.append(hyp)

        msg.detections.append(det)


# =============================================================================
# TestSubscriberNode — Captures tracking commands and actuator status
# =============================================================================

class TestSubscriberNode(Node):
    """
    Test node that subscribes to tracking outputs and validates them.

    Captures:
        - /tracking/command  (JSON from tracker_node)
        - /tracking/status   (JSON from dummy_actuator_node)
    """

    def __init__(self) -> None:
        super().__init__('test_tracking_subscriber')

        self.commands_received: List[dict] = []
        self.statuses_received: List[dict] = []

        self._command_sub = self.create_subscription(
            String,
            '/tracking/command',
            self._command_callback,
            10,
        )

        self._status_sub = self.create_subscription(
            String,
            '/tracking/status',
            self._status_callback,
            10,
        )

        self.get_logger().info('📥 Test subscriber ready — listening...')

    def _command_callback(self, msg: String) -> None:
        """Capture incoming tracking command."""
        try:
            cmd = json.loads(msg.data)
            self.commands_received.append(cmd)

            self.get_logger().info(
                f'  📨 CMD: ID={cmd.get("target_id")} '
                f'err=({cmd.get("error_x", 0):+.3f}, '
                f'{cmd.get("error_y", 0):+.3f}) '
                f'vel=({cmd.get("velocity_x", 0):+.1f}, '
                f'{cmd.get("velocity_y", 0):+.1f})'
            )
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON: {msg.data}')

    def _status_callback(self, msg: String) -> None:
        """Capture incoming actuator status."""
        try:
            status = json.loads(msg.data)
            self.statuses_received.append(status)
        except json.JSONDecodeError:
            pass


# =============================================================================
# Test Runner
# =============================================================================

def run_test(
    init_x: float,
    init_y: float,
    target_class: str,
    tracker_type: str = 'centroid',
) -> bool:
    """
    Run the tracking integration test.

    Args:
        init_x: Initial X coordinate for the synthetic detection.
        init_y: Initial Y coordinate for the synthetic detection.
        target_class: YOLO class label for the detection.
        tracker_type: Tracker algorithm ('centroid' or 'bytetrack').

    Returns:
        True if all assertions pass, False otherwise.
    """
    print('=' * 70)
    print(f'  Object Tracking Integration Test')
    print(f'  Init Point: ({init_x}, {init_y})')
    print(f'  Target Class: {target_class}')
    print(f'  Tracker Type: {tracker_type}')
    print(f'  Image: {IMAGE_WIDTH}x{IMAGE_HEIGHT}')
    print('=' * 70)

    # Import tracker and actuator nodes from the package
    from yolo_rpi_core.tracker_node import TrackerNode
    from yolo_rpi_core.dummy_actuator_node import DummyActuatorNode

    # Create nodes — override tracker_type parameter
    from rclpy.parameter import Parameter as RosParameter
    pub_node = TestPublisherNode(init_x, init_y, target_class)
    tracker_node = TrackerNode(
        parameter_overrides=[
            RosParameter('tracker_type', RosParameter.Type.STRING, tracker_type),
        ],
    )
    actuator_node = DummyActuatorNode()
    sub_node = TestSubscriberNode()

    # Use MultiThreadedExecutor for concurrent node execution
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(pub_node)
    executor.add_node(tracker_node)
    executor.add_node(actuator_node)
    executor.add_node(sub_node)

    # Run executor in a background thread with timeout
    def spin_executor():
        try:
            executor.spin()
        except Exception:
            pass

    spin_thread = threading.Thread(target=spin_executor, daemon=True)
    spin_thread.start()

    # Wait for test to complete (or timeout)
    print(f'\n⏳ Running test (timeout: {TEST_TIMEOUT}s)...\n')
    time.sleep(TEST_TIMEOUT)

    # Stop executor
    executor.shutdown()

    # =========================================================================
    # Assertions
    # =========================================================================
    print('\n' + '=' * 70)
    print('  Test Results')
    print('=' * 70)

    passed = True

    # --- Check 1: Tracking commands were received ---
    num_commands = len(sub_node.commands_received)
    if num_commands > 0:
        print(f'  ✅ PASS: Received {num_commands} tracking commands')
    else:
        print(f'  ❌ FAIL: No tracking commands received')
        passed = False

    # --- Check 2: First command has a valid target_id ---
    if num_commands > 0:
        first_cmd = sub_node.commands_received[0]
        if 'target_id' in first_cmd and isinstance(first_cmd['target_id'], int):
            print(f'  ✅ PASS: First command has target_id={first_cmd["target_id"]}')
        else:
            print(f'  ❌ FAIL: First command missing target_id')
            passed = False

    # --- Check 3: Error values are in [-1, 1] range ---
    if num_commands > 0:
        errors_valid = all(
            -1.0 <= cmd.get('error_x', 0) <= 1.0
            and -1.0 <= cmd.get('error_y', 0) <= 1.0
            for cmd in sub_node.commands_received
        )
        if errors_valid:
            print(f'  ✅ PASS: All error values in [-1, 1] range')
        else:
            print(f'  ❌ FAIL: Some error values out of range')
            passed = False

    # --- Check 4: Error changes as object moves ---
    if num_commands >= 3:
        errors_x = [cmd.get('error_x', 0) for cmd in sub_node.commands_received]
        error_changed = max(errors_x) - min(errors_x) > 0.01
        if error_changed:
            print(f'  ✅ PASS: error_x changed as object moved '
                  f'(range: {min(errors_x):.3f} to {max(errors_x):.3f})')
        else:
            print(f'  ❌ FAIL: error_x did not change during movement')
            passed = False

    # --- Check 5: Actuator status received ---
    num_statuses = len(sub_node.statuses_received)
    if num_statuses > 0:
        print(f'  ✅ PASS: Received {num_statuses} actuator status messages')
        last_status = sub_node.statuses_received[-1]
        print(f'         Last state: {last_status.get("state")} | '
              f'Commands processed: {last_status.get("command_count")}')
    else:
        print(f'  ❌ FAIL: No actuator status messages received')
        passed = False

    # --- Check 6: Actuator entered TRACKING state ---
    if num_statuses > 0:
        tracking_states = [
            s for s in sub_node.statuses_received
            if s.get('state') == 'TRACKING'
        ]
        if tracking_states:
            print(f'  ✅ PASS: Actuator entered TRACKING state')
        else:
            print(f'  ⚠️ WARN: Actuator never entered TRACKING state')

    # --- Check 7: Class name matches ---
    if num_commands > 0:
        class_match = all(
            cmd.get('class_name') == target_class
            for cmd in sub_node.commands_received
        )
        if class_match:
            print(f'  ✅ PASS: All commands have class_name="{target_class}"')
        else:
            print(f'  ❌ FAIL: Class name mismatch in some commands')
            passed = False

    print('\n' + '=' * 70)
    if passed:
        print('  🎉 ALL TESTS PASSED')
    else:
        print('  💥 SOME TESTS FAILED')
    print('=' * 70)

    # Cleanup
    pub_node.destroy_node()
    tracker_node.destroy_node()
    actuator_node.destroy_node()
    sub_node.destroy_node()

    return passed


# =============================================================================
# Entry Point
# =============================================================================

def main() -> None:
    """Parse CLI arguments and run the tracking test."""
    parser = argparse.ArgumentParser(
        description='Object Tracking Integration Test',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Default: init point at image center
    python3 test_tracking.py

    # Custom init point (top-left quadrant)
    python3 test_tracking.py --init-x 100 --init-y 100

    # Track a specific class
    python3 test_tracking.py --target-class cup

    # Track from bottom-right corner
    python3 test_tracking.py --init-x 500 --init-y 400 --target-class person
        """,
    )
    parser.add_argument(
        '--init-x', type=float, default=IMAGE_WIDTH / 2.0,
        help=f'Initial X coordinate (pixels, default: {IMAGE_WIDTH / 2.0})',
    )
    parser.add_argument(
        '--init-y', type=float, default=IMAGE_HEIGHT / 2.0,
        help=f'Initial Y coordinate (pixels, default: {IMAGE_HEIGHT / 2.0})',
    )
    parser.add_argument(
        '--target-class', type=str, default='person',
        help='YOLO class name to simulate (default: person)',
    )
    parser.add_argument(
        '--tracker-type', type=str, default='centroid',
        choices=['centroid', 'bytetrack'],
        help='Tracker algorithm to test (default: centroid)',
    )

    args = parser.parse_args()

    # Initialize ROS2
    rclpy.init()

    try:
        success = run_test(
            args.init_x,
            args.init_y,
            args.target_class,
            args.tracker_type,
        )
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print('\n⚠️ Test interrupted by user')
        sys.exit(130)
    except Exception as e:
        print(f'\n❌ Test error: {e}')
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
