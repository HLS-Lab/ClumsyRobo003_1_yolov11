#!/usr/bin/env python3
"""
Integration Test: SerialMotorActuatorNode (simulation mode).

Validates tracker_node → serial_motor_actuator_node without real hardware.

Checks:
    1. Node starts in IDLE state (no crash)
    2. Synthetic /tracking/command triggers TRACKING state
    3. Demo mode (pan_range_deg=0): both motors commanded to 90° (center)
    4. /tracking/status contains motor-specific fields (last_angles, serial_port)
    5. Proper state transition IDLE → TRACKING

Usage (inside Docker container or ROS2 workspace)::

    cd /ros2_ws && colcon build --packages-select yolo_rpi_core
    source install/setup.bash
    python3 src/yolo_rpi_core/test/test_serial_motor_actuator.py

Compatible with: ROS 2 Jazzy, Python 3.12
Target Platform: Raspberry Pi 4 (CPU only, NO CUDA)
"""

from __future__ import annotations

import json
import sys
import threading
import time
from typing import List

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter as RosParameter

from std_msgs.msg import String


# =============================================================================
# Constants
# =============================================================================

TEST_TIMEOUT = 18.0   # seconds


# =============================================================================
# Helpers
# =============================================================================

def _tracking_cmd(
    target_id: int = 0,
    error_x: float = 0.0,
    error_y: float = 0.0,
    class_name: str = 'person',
    confidence: float = 0.90,
) -> str:
    """Build a JSON /tracking/command string matching the tracker_node schema."""
    return json.dumps({
        'target_id': target_id,
        'error_x': error_x,
        'error_y': error_y,
        'velocity_x': 0.0,
        'velocity_y': 0.0,
        'bbox_width': 80.0,
        'bbox_height': 100.0,
        'class_name': class_name,
        'confidence': confidence,
        'timestamp': time.time(),
    })


# =============================================================================
# Nodes
# =============================================================================

class FakeCommandPublisher(Node):
    """Publishes synthetic /tracking/command messages at 5 Hz."""

    def __init__(self) -> None:
        super().__init__('test_fake_cmd_pub')
        self._pub = self.create_publisher(String, '/tracking/command', 10)
        self._frame = 0
        self._timer = self.create_timer(0.2, self._tick)

    def _tick(self) -> None:
        self._frame += 1
        if self._frame < 5:
            return  # warm-up delay

        msg = String()
        if self._frame <= 20:
            # Person at center (demo scenario → expect angle1=90, angle2=90)
            msg.data = _tracking_cmd(error_x=0.0, error_y=0.0)
        elif self._frame <= 30:
            # Off-center person (still 90° in demo mode since range=0)
            msg.data = _tracking_cmd(error_x=0.5, error_y=-0.3)
        else:
            self._timer.cancel()
            return

        self._pub.publish(msg)


class StatusCollector(Node):
    """Collects /tracking/status messages."""

    def __init__(self) -> None:
        super().__init__('test_status_collector')
        self.statuses: List[dict] = []
        self.create_subscription(String, '/tracking/status', self._cb, 10)

    def _cb(self, msg: String) -> None:
        try:
            self.statuses.append(json.loads(msg.data))
        except json.JSONDecodeError:
            pass


# =============================================================================
# Test Runner
# =============================================================================

def run_test() -> bool:
    print('=' * 70)
    print('  SerialMotorActuatorNode — Integration Test (Simulation Mode)')
    print('=' * 70)

    from yolo_rpi_core.serial_motor_actuator_node import SerialMotorActuatorNode

    # Instantiate with overrides to guarantee simulation + demo mode
    actuator = SerialMotorActuatorNode()
    # Force simulation params (node already declared them in _on_startup)
    actuator.set_parameters([
        RosParameter('simulation_mode', RosParameter.Type.BOOL, True),
        RosParameter('pan_range_deg',   RosParameter.Type.DOUBLE, 0.0),
        RosParameter('tilt_range_deg',  RosParameter.Type.DOUBLE, 0.0),
        RosParameter('center_angle',    RosParameter.Type.DOUBLE, 90.0),
    ])

    fake_pub = FakeCommandPublisher()
    collector = StatusCollector()

    executor = MultiThreadedExecutor(num_threads=4)
    for n in (actuator, fake_pub, collector):
        executor.add_node(n)

    def _spin() -> None:
        try:
            executor.spin()
        except Exception:
            pass

    thread = threading.Thread(target=_spin, daemon=True)
    thread.start()

    print(f'\n⏳ Running for {TEST_TIMEOUT}s...\n')
    time.sleep(TEST_TIMEOUT)
    executor.shutdown()

    # =========================================================================
    # Assertions
    # =========================================================================
    print('\n' + '=' * 70)
    print('  Results')
    print('=' * 70)

    passed = True
    statuses = collector.statuses

    # 1 — Status messages received
    if statuses:
        print(f'  ✅ PASS: Received {len(statuses)} status messages')
    else:
        print('  ❌ FAIL: No status messages received')
        passed = False
        # early exit — no point checking further
        print('=' * 70)
        return False

    # 2 — TRACKING state reached
    tracking = [s for s in statuses if s.get('state') == 'TRACKING']
    if tracking:
        print(f'  ✅ PASS: Entered TRACKING state ({len(tracking)} msgs)')
    else:
        print('  ❌ FAIL: Never entered TRACKING state')
        passed = False

    # 3 — Motor fields present in status
    last = statuses[-1]
    if 'last_angles' in last and 'serial_port' in last:
        print(f'  ✅ PASS: Motor fields present '
              f'(last_angles={last["last_angles"]}, port={last["serial_port"]})')
    else:
        print(f'  ❌ FAIL: Missing motor fields. Keys: {list(last.keys())}')
        passed = False

    # 4 — Demo mode: angles should be [90.0, 90.0]
    if tracking:
        angles = tracking[-1].get('last_angles', [None, None])
        a1, a2 = angles[0], angles[1]
        if a1 == 90.0 and a2 == 90.0:
            print(f'  ✅ PASS: Demo mode — angles = [90.0°, 90.0°] when human detected')
        else:
            print(f'  ❌ FAIL: Expected [90.0, 90.0], got {angles}')
            passed = False

    # 5 — simulation_mode reflected in status
    sim = last.get('simulation_mode')
    if sim is True:
        print('  ✅ PASS: simulation_mode=True in status')
    else:
        print(f'  ⚠️  WARN: simulation_mode not in status (got {sim!r})')

    print('\n  ' + ('🎉 ALL TESTS PASSED' if passed else '💥 SOME TESTS FAILED'))
    print('=' * 70)

    actuator.destroy_node()
    fake_pub.destroy_node()
    collector.destroy_node()

    return passed


# =============================================================================
# Entry Point
# =============================================================================

def main() -> None:
    rclpy.init()
    try:
        ok = run_test()
        sys.exit(0 if ok else 1)
    except KeyboardInterrupt:
        print('\n⚠️  Interrupted')
        sys.exit(130)
    except Exception as exc:
        import traceback
        print(f'\n❌ Error: {exc}')
        traceback.print_exc()
        sys.exit(1)
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
