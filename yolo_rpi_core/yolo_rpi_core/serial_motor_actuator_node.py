#!/usr/bin/env python3
"""
Serial Motor Actuator Node for ROS 2.

Real actuator node that replaces DummyActuatorNode when a physical motor
controller board is attached via USB-serial.

Motor Controller Interface (control_v1 protocol):
    Transport : UART / USB-Serial (/dev/ttyUSB0)
    Baud rate : 115200
    Command   : JSON + newline  →  '{"angle1": 90, "angle2": 90}\\n'
    Response  : JSON line (logged, non-blocking)

Angle mapping from tracking command::

    angle1  (pan)  = clamp(center + error_x * pan_range_deg,  min_angle, max_angle)
    angle2  (tilt) = clamp(center + error_y * tilt_range_deg, min_angle, max_angle)

    With default params (center=90, pan_range_deg=0, tilt_range_deg=0):
      → Both motors always commanded to 90° when human is detected (demo mode).
    Set pan_range_deg / tilt_range_deg > 0 to enable proportional tracking.

Usage::

    # Simulation (no hardware) — default
    ros2 run yolo_rpi_core serial_motor_actuator_node

    # Real hardware
    ros2 run yolo_rpi_core serial_motor_actuator_node \\
        --ros-args -p simulation_mode:=false -p serial_port:=/dev/ttyUSB0

Compatible with: ROS 2 Jazzy, Python 3.12
Target Platform: Raspberry Pi 4 (CPU only, NO CUDA)
"""

from __future__ import annotations

import json
import time
from typing import List, Optional

import rclpy
from std_msgs.msg import String

from yolo_rpi_core.base_actuator import BaseActuatorNode, ActuatorState
from yolo_rpi_core.serial_motor_controller import SerialMotorController


# =============================================================================
# Helper
# =============================================================================

def _clamp(value: float, lo: float, hi: float) -> float:
    """Clamp *value* to the inclusive [lo, hi] range."""
    return max(lo, min(hi, value))


# =============================================================================
# SerialMotorActuatorNode
# =============================================================================

class SerialMotorActuatorNode(BaseActuatorNode):
    """
    Real actuator node: translates tracking errors into serial motor commands.

    Inherits all ROS plumbing, state machine (IDLE/TRACKING/ERROR), JSON
    parsing, status publishing, and lifecycle management from BaseActuatorNode.

    Implements:
        _on_startup()          — opens serial port via SerialMotorController
        _execute_command()     — maps error_x/y → angle1/angle2, sends JSON
        _stop()                — sends center (90,90) and marks controller safe
        _on_state_change()     — logs state transitions

    Optional overrides:
        _publish_status()      — extends parent JSON with motor-specific fields

    ROS Parameters:
        actuator_name       (str,   default 'serial_pan_tilt')
        status_publish_rate (float, default 2.0)
        serial_port         (str,   default '/dev/ttyUSB0')
        serial_baud         (int,   default 115200)
        serial_timeout      (float, default 1.0)
        simulation_mode     (bool,  default True)
        center_angle        (float, default 90.0)
        pan_range_deg       (float, default 0.0)   — 0 = always center (demo)
        tilt_range_deg      (float, default 0.0)   — 0 = always center (demo)
        min_angle           (float, default 0.0)
        max_angle           (float, default 180.0)
    """

    def __init__(self) -> None:
        """Initialize node — hardware init happens in _on_startup()."""
        super().__init__(
            node_name='serial_motor_actuator_node',
            actuator_name='serial_pan_tilt',
        )

        # Hardware state (set in _on_startup)
        self._controller: Optional[SerialMotorController] = None
        self._current_angle1: float = 90.0
        self._current_angle2: float = 90.0

        self.get_logger().info(
            '🔌 SerialMotorActuatorNode created — hardware initialized in _on_startup.'
        )

    # =========================================================================
    # Abstract method implementations
    # =========================================================================

    def _on_startup(self) -> None:
        """
        Hardware initialization hook — called once at the end of __init__.

        Reads ROS parameters, instantiates SerialMotorController, opens port.
        """
        # --- Read motor-specific parameters ---
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('serial_timeout', 1.0)
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('center_angle', 90.0)
        self.declare_parameter('pan_range_deg', 0.0)
        self.declare_parameter('tilt_range_deg', 0.0)
        self.declare_parameter('min_angle', 0.0)
        self.declare_parameter('max_angle', 180.0)

        self._serial_port: str = (
            self.get_parameter('serial_port').get_parameter_value().string_value
        )
        self._serial_baud: int = (
            self.get_parameter('serial_baud').get_parameter_value().integer_value
        )
        self._serial_timeout: float = (
            self.get_parameter('serial_timeout').get_parameter_value().double_value
        )
        self._simulation_mode: bool = (
            self.get_parameter('simulation_mode').get_parameter_value().bool_value
        )
        self._center_angle: float = (
            self.get_parameter('center_angle').get_parameter_value().double_value
        )
        self._pan_range_deg: float = (
            self.get_parameter('pan_range_deg').get_parameter_value().double_value
        )
        self._tilt_range_deg: float = (
            self.get_parameter('tilt_range_deg').get_parameter_value().double_value
        )
        self._min_angle: float = (
            self.get_parameter('min_angle').get_parameter_value().double_value
        )
        self._max_angle: float = (
            self.get_parameter('max_angle').get_parameter_value().double_value
        )

        mode_str = 'SIMULATION' if self._simulation_mode else f'REAL ({self._serial_port})'
        self.get_logger().info(
            f'⚙️  Startup: mode={mode_str} | center={self._center_angle}° | '
            f'pan_range=±{self._pan_range_deg}° | tilt_range=±{self._tilt_range_deg}°'
        )

        # --- Open serial port ---
        self._controller = SerialMotorController(
            port=self._serial_port,
            baud=self._serial_baud,
            timeout=self._serial_timeout,
            simulation_mode=self._simulation_mode,
        )

        connected = self._controller.connect()
        if not connected:
            self.get_logger().error(
                f'❌ Failed to connect to motor controller at {self._serial_port}. '
                'Node will start in ERROR state — check serial connection.'
            )
            self._set_state(ActuatorState.ERROR)
        else:
            self.get_logger().info('✅ Motor controller connected.')

    def _execute_command(self, command: dict) -> bool:
        """
        Translate a tracking command into motor angle commands and send them.

        Angle formula::

            angle1 = clamp(center + error_x * pan_range_deg,  min, max)
            angle2 = clamp(center + error_y * tilt_range_deg, min, max)

        With pan_range_deg = tilt_range_deg = 0 (default / demo mode):
            → Both angles = center (90°) whenever a human is detected.

        Args:
            command: Validated tracking command dict with error_x, error_y, etc.

        Returns:
            True if motor command was sent successfully, False on failure.
        """
        if self._controller is None:
            self.get_logger().error('Motor controller not initialized.')
            return False

        error_x: float = command.get('error_x', 0.0)
        error_y: float = command.get('error_y', 0.0)
        target_id: int = command.get('target_id', -1)
        class_name: str = command.get('class_name', 'unknown')

        # Map tracking error to motor angles
        angle1 = _clamp(
            self._center_angle + error_x * self._pan_range_deg,
            self._min_angle,
            self._max_angle,
        )
        angle2 = _clamp(
            self._center_angle + error_y * self._tilt_range_deg,
            self._min_angle,
            self._max_angle,
        )

        self._current_angle1 = angle1
        self._current_angle2 = angle2

        # Send to hardware
        ok = self._controller.send(angle1, angle2)
        if not ok:
            self.get_logger().error(
                f'Serial send failed (angle1={angle1:.1f}°, angle2={angle2:.1f}°)'
            )
            return False

        self.get_logger().info(
            f'🎯 Tracking ID:{target_id} [{class_name}] | '
            f'error=({error_x:+.3f}, {error_y:+.3f}) | '
            f'→ angle1={angle1:.1f}°, angle2={angle2:.1f}°'
        )
        return True

    def _stop(self) -> None:
        """
        Emergency stop — send both motors to center (90°, 90°).

        Called when:
            - Target is lost
            - An error is detected
            - Node is shutting down
        """
        self._current_angle1 = self._center_angle
        self._current_angle2 = self._center_angle

        if self._controller is not None:
            self._controller.send(self._center_angle, self._center_angle)
            self.get_logger().info(
                f'🛑 STOP — Motors centered at ({self._center_angle}°, {self._center_angle}°)'
            )

    # =========================================================================
    # Optional overrides
    # =========================================================================

    def _on_state_change(
        self,
        old_state: ActuatorState,
        new_state: ActuatorState,
    ) -> None:
        """Log state transitions with icons."""
        icons = {
            ActuatorState.IDLE: '💤',
            ActuatorState.TRACKING: '🏃',
            ActuatorState.ERROR: '❌',
        }
        icon = icons.get(new_state, '❓')
        self.get_logger().info(f'{icon} State: {old_state.value} → {new_state.value}')

    def _publish_status(self) -> None:
        """
        Extend parent status with motor-specific fields.

        Published JSON schema (extends BaseActuatorNode status)::

            {
                "actuator_name": str,
                "state": str,
                "command_count": int,
                "timestamp": float,
                "last_command": dict | null,
                "serial_port": str,
                "simulation_mode": bool,
                "last_angles": [float, float]
            }
        """
        status = {
            'actuator_name': self._actuator_name,
            'state': self._state.value,
            'command_count': self._command_count,
            'timestamp': time.time(),
            'last_command': self._last_command,
            'serial_port': self._serial_port if hasattr(self, '_serial_port') else 'unknown',
            'simulation_mode': self._simulation_mode if hasattr(self, '_simulation_mode') else True,
            'last_angles': [self._current_angle1, self._current_angle2],
        }

        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    def destroy_node(self) -> None:
        """Clean up: stop motors and close serial port before shutdown."""
        self.get_logger().info('Shutting down SerialMotorActuatorNode...')
        try:
            self._stop()
        except Exception as exc:
            self.get_logger().error(f'Stop error on shutdown: {exc}')

        if self._controller is not None:
            self._controller.close()

        self._set_state(ActuatorState.IDLE)
        # Call grandparent (Node.destroy_node) directly to avoid double _stop()
        from rclpy.node import Node as _Node
        _Node.destroy_node(self)


# =============================================================================
# Entry Point
# =============================================================================

def main(args: Optional[List[str]] = None) -> None:
    """Entry point for the serial motor actuator node."""
    rclpy.init(args=args)
    node: Optional[SerialMotorActuatorNode] = None

    try:
        node = SerialMotorActuatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'SerialMotorActuatorNode error: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
