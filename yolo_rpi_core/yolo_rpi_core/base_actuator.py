#!/usr/bin/env python3
"""
Abstract Base Actuator Node for ROS 2.

This module defines the `BaseActuatorNode` — an abstract base class that
provides a standardized interface for any actuator that responds to object
tracking commands.  It follows the **Template Method** design pattern:
the base class handles ROS plumbing (subscription, status publishing, state
management), while subclasses implement the actual hardware control logic.

Design Principles (OOP):
    - **Open/Closed Principle**: extend via subclassing, not modification.
    - **Template Method Pattern**: `_on_command_received()` orchestrates the
      workflow; subclasses override `_execute_command()` and `_stop()`.
    - **Liskov Substitution**: any subclass can replace DummyActuatorNode
      without affecting the rest of the pipeline.

How to create a real actuator:
    1. Create a new file, e.g. `servo_actuator_node.py`.
    2. Subclass `BaseActuatorNode`.
    3. Override `_execute_command(command: dict) -> bool` with motor control.
    4. Override `_stop() -> None` with emergency stop logic.
    5. Optionally override `_on_startup()` for hardware initialization.
    6. Register the new node in `setup.py` entry_points.

Example:
    class ServoActuatorNode(BaseActuatorNode):
        def __init__(self):
            super().__init__(node_name='servo_actuator_node')
            self._pan_servo = ServoDriver(pin=18)
            self._tilt_servo = ServoDriver(pin=19)

        def _execute_command(self, command: dict) -> bool:
            pan_angle = command['error_x'] * 90.0   # map to degrees
            tilt_angle = command['error_y'] * 45.0
            self._pan_servo.set_angle(pan_angle)
            self._tilt_servo.set_angle(tilt_angle)
            return True

        def _stop(self) -> None:
            self._pan_servo.set_angle(0.0)
            self._tilt_servo.set_angle(0.0)

Compatible with: ROS 2 Jazzy, Python 3.12
Target Platform: Raspberry Pi 4 (CPU only, NO CUDA)
"""

from __future__ import annotations

import json
import time
from abc import ABC, abstractmethod
from enum import Enum
from typing import List, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


# =============================================================================
# Actuator State Enum
# =============================================================================

class ActuatorState(Enum):
    """
    Possible operational states of an actuator node.

    State transitions:
        IDLE ──────► TRACKING  (first valid command received)
        TRACKING ──► IDLE      (target lost / explicit stop)
        TRACKING ──► ERROR     (hardware failure / invalid command)
        ERROR ─────► IDLE      (reset / recovery)
        ANY ───────► IDLE      (stop command)
    """
    IDLE = "IDLE"
    TRACKING = "TRACKING"
    ERROR = "ERROR"


# =============================================================================
# BaseActuatorNode — Abstract Base Class
# =============================================================================

class BaseActuatorNode(Node, ABC):
    """
    Abstract base class for actuator nodes that consume tracking commands.

    This class provides:
        - Subscription to `/tracking/command` (JSON-encoded String)
        - Publication of `/tracking/status` (JSON-encoded String)
        - State machine (IDLE → TRACKING → ERROR)
        - Heartbeat timer for periodic status publishing
        - JSON parsing and validation of incoming commands

    Subclasses MUST implement:
        - `_execute_command(command: dict) -> bool`
        - `_stop() -> None`

    Subclasses MAY override:
        - `_on_startup() -> None`  (called once after __init__)
        - `_validate_command(command: dict) -> bool`
        - `_on_state_change(old: ActuatorState, new: ActuatorState) -> None`

    Args:
        node_name:      ROS2 node name (default: 'base_actuator_node').
        actuator_name:  Human-readable name for status messages.
    """

    # Required keys in a valid tracking command JSON payload
    REQUIRED_COMMAND_KEYS = {
        'target_id', 'error_x', 'error_y',
        'class_name', 'confidence', 'timestamp',
    }

    def __init__(
        self,
        node_name: str = 'base_actuator_node',
        actuator_name: str = 'base_actuator',
    ) -> None:
        """
        Initialize the base actuator node.

        Args:
            node_name:     Unique ROS2 node name.
            actuator_name: Descriptive name for this actuator (used in status).
        """
        super().__init__(node_name)

        # -----------------------------------------------------------------
        # Parameters
        # -----------------------------------------------------------------
        self.declare_parameter('actuator_name', actuator_name)
        self._actuator_name: str = (
            self.get_parameter('actuator_name')
            .get_parameter_value().string_value
        )

        self.declare_parameter('status_publish_rate', 2.0)
        status_rate: float = (
            self.get_parameter('status_publish_rate')
            .get_parameter_value().double_value
        )

        # -----------------------------------------------------------------
        # State Management
        # -----------------------------------------------------------------
        self._state: ActuatorState = ActuatorState.IDLE
        self._last_command: Optional[dict] = None
        self._command_count: int = 0

        # -----------------------------------------------------------------
        # Subscriber: Tracking commands from tracker_node
        # -----------------------------------------------------------------
        self._command_sub = self.create_subscription(
            String,
            '/tracking/command',
            self._on_command_received,
            10,
        )

        # -----------------------------------------------------------------
        # Publisher: Actuator status feedback
        # -----------------------------------------------------------------
        self._status_pub = self.create_publisher(
            String,
            '/tracking/status',
            10,
        )

        # -----------------------------------------------------------------
        # Heartbeat Timer: Publish status at regular intervals
        # -----------------------------------------------------------------
        timer_period = 1.0 / status_rate if status_rate > 0 else 1.0
        self._status_timer = self.create_timer(
            timer_period, self._publish_status
        )

        # -----------------------------------------------------------------
        # Startup hook for subclasses
        # -----------------------------------------------------------------
        self._on_startup()

        self.get_logger().info(
            f'{self.__class__.__name__} [{self._actuator_name}] '
            f'initialized — state: {self._state.value}'
        )

    # =====================================================================
    # Abstract Methods — MUST be implemented by subclasses
    # =====================================================================

    @abstractmethod
    def _execute_command(self, command: dict) -> bool:
        """
        Execute a tracking command on the physical hardware.

        This is the core method that subclasses implement to translate
        tracking error offsets into actuator movements.

        The `command` dictionary contains at minimum:
            - target_id (int): Tracked object ID
            - error_x (float): Horizontal offset [-1.0, 1.0]
            - error_y (float): Vertical offset [-1.0, 1.0]
            - velocity_x (float): Horizontal velocity (px/frame)
            - velocity_y (float): Vertical velocity (px/frame)
            - class_name (str): Object class
            - confidence (float): Detection confidence

        Args:
            command: Parsed tracking command dictionary.

        Returns:
            True if command was executed successfully, False on failure.
        """
        ...

    @abstractmethod
    def _stop(self) -> None:
        """
        Emergency stop — immediately halt all actuator movement.

        Called when:
            - The tracked target is lost.
            - An error is detected.
            - The node is shutting down.

        Subclasses should set all motors/servos to a safe resting position.
        """
        ...

    # =====================================================================
    # Virtual Methods — MAY be overridden by subclasses
    # =====================================================================

    def _on_startup(self) -> None:
        """
        Hook called once after __init__ completes.

        Override this to perform hardware initialization, GPIO setup, etc.
        The default implementation does nothing.
        """
        pass

    def _validate_command(self, command: dict) -> bool:
        """
        Validate a parsed tracking command.

        Override to add custom validation (e.g., range checks on error).
        Default implementation checks that all required keys are present.

        Args:
            command: Parsed JSON dictionary.

        Returns:
            True if valid, False otherwise.
        """
        return self.REQUIRED_COMMAND_KEYS.issubset(command.keys())

    def _on_state_change(
        self,
        old_state: ActuatorState,
        new_state: ActuatorState,
    ) -> None:
        """
        Notification hook for actuator state transitions.

        Override to react to state changes (e.g., LED indicators, buzzers).

        Args:
            old_state: Previous actuator state.
            new_state: New actuator state.
        """
        pass

    # =====================================================================
    # Template Method — Command Processing Pipeline
    # =====================================================================

    def _on_command_received(self, msg: String) -> None:
        """
        Template method: parse, validate, execute, and report.

        This method implements the full command processing pipeline:
            1. Parse JSON payload
            2. Validate command structure
            3. Execute on hardware (subclass)
            4. Update state
            5. Publish status

        Do NOT override this method. Override `_execute_command()` instead.
        """
        # Step 1: Parse JSON
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON command: {e}')
            self._set_state(ActuatorState.ERROR)
            return

        # Step 2: Validate
        if not self._validate_command(command):
            self.get_logger().warning(
                f'Invalid command (missing keys): {command.keys()}'
            )
            return

        # Step 3: Execute
        try:
            success = self._execute_command(command)
        except Exception as e:
            self.get_logger().error(f'Command execution failed: {e}')
            self._set_state(ActuatorState.ERROR)
            self._stop()
            return

        # Step 4: Update state
        if success:
            self._last_command = command
            self._command_count += 1
            if self._state != ActuatorState.TRACKING:
                self._set_state(ActuatorState.TRACKING)
        else:
            self.get_logger().warning('Command execution returned False')
            self._set_state(ActuatorState.ERROR)
            self._stop()

    # =====================================================================
    # State Machine
    # =====================================================================

    def _set_state(self, new_state: ActuatorState) -> None:
        """
        Transition to a new actuator state.

        Logs the transition and invokes the `_on_state_change` hook.
        """
        if new_state == self._state:
            return

        old_state = self._state
        self._state = new_state

        self.get_logger().info(
            f'State transition: {old_state.value} → {new_state.value}'
        )
        self._on_state_change(old_state, new_state)

    @property
    def state(self) -> ActuatorState:
        """Current actuator state (read-only)."""
        return self._state

    @property
    def actuator_name(self) -> str:
        """Human-readable actuator name (read-only)."""
        return self._actuator_name

    # =====================================================================
    # Status Publishing
    # =====================================================================

    def _publish_status(self) -> None:
        """
        Publish the current actuator status as a JSON String.

        Published JSON schema:
            {
                "actuator_name": str,
                "state": str,          # IDLE / TRACKING / ERROR
                "command_count": int,
                "timestamp": float,
                "last_command": dict | null
            }
        """
        status = {
            'actuator_name': self._actuator_name,
            'state': self._state.value,
            'command_count': self._command_count,
            'timestamp': time.time(),
            'last_command': self._last_command,
        }

        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    # =====================================================================
    # Lifecycle
    # =====================================================================

    def destroy_node(self) -> None:
        """Clean up: stop actuator before shutting down."""
        self.get_logger().info('Shutting down — sending stop command')
        try:
            self._stop()
        except Exception as e:
            self.get_logger().error(f'Error during stop: {e}')
        self._set_state(ActuatorState.IDLE)
        super().destroy_node()
