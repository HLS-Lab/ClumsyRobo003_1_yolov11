#!/usr/bin/env python3
"""
Dummy Actuator Node for ROS 2.

This node is a concrete implementation of `BaseActuatorNode` that simulates
actuator behavior by logging commands instead of driving real motors.
It serves two purposes:

    1. **Development & Testing**: Validate the tracking pipeline end-to-end
       without physical hardware.
    2. **Reference Implementation**: Demonstrate how to subclass
       `BaseActuatorNode` for real actuator drivers.

Usage:
    ros2 run yolo_rpi_core dummy_actuator_node

How to create a REAL actuator from this template:
    ┌──────────────────────────────────────────────────────────────────┐
    │  1. Copy this file as a template                                │
    │  2. Rename class to e.g. ServoActuatorNode                      │
    │  3. Override _execute_command() with motor control logic         │
    │  4. Override _stop() with emergency stop logic                   │
    │  5. Optionally override _on_startup() for GPIO init              │
    │  6. Register in setup.py entry_points                            │
    └──────────────────────────────────────────────────────────────────┘

    Example (servo-based pan/tilt):

        class ServoActuatorNode(BaseActuatorNode):
            def __init__(self):
                super().__init__(
                    node_name='servo_actuator_node',
                    actuator_name='pan_tilt_servo',
                )
                # Initialize servo hardware here

            def _on_startup(self):
                # GPIO setup, PWM init, etc.
                self._pan_pwm = PWM(pin=18, freq=50)
                self._tilt_pwm = PWM(pin=19, freq=50)

            def _execute_command(self, command: dict) -> bool:
                pan = command['error_x'] * 90.0    # ±90 degrees
                tilt = command['error_y'] * 45.0   # ±45 degrees
                self._pan_pwm.set_angle(pan)
                self._tilt_pwm.set_angle(tilt)
                return True

            def _stop(self):
                self._pan_pwm.set_angle(0.0)
                self._tilt_pwm.set_angle(0.0)

Compatible with: ROS 2 Jazzy, Python 3.12
Target Platform: Raspberry Pi 4 (CPU only, NO CUDA)
"""

from __future__ import annotations

from typing import List, Optional

import rclpy

from yolo_rpi_core.base_actuator import BaseActuatorNode, ActuatorState


class DummyActuatorNode(BaseActuatorNode):
    """
    Dummy actuator that logs tracking commands without driving real hardware.

    Inherits all ROS plumbing, state management, and command parsing from
    `BaseActuatorNode`.  This class only implements the two abstract methods:

        - `_execute_command()`: Logs the computed pan/tilt angles.
        - `_stop()`: Logs the stop event.

    The "simulated" pan/tilt values are computed as:
        pan  = error_x × 90.0  (degrees, ±90° range)
        tilt = error_y × 45.0  (degrees, ±45° range)

    These values are purely illustrative; a real actuator subclass would
    send actual PWM signals or serial commands.
    """

    # -----------------------------------------------------------------
    # Configuration: simulated angle ranges
    # -----------------------------------------------------------------
    PAN_RANGE_DEG: float = 90.0  # Max pan angle (degrees)
    TILT_RANGE_DEG: float = 45.0  # Max tilt angle (degrees)

    def __init__(self) -> None:
        """Initialize the dummy actuator node."""
        super().__init__(
            node_name="dummy_actuator_node",
            actuator_name="dummy_pan_tilt",
        )

        # Simulated internal state
        self._current_pan: float = 0.0
        self._current_tilt: float = 0.0

        self.get_logger().info(
            "🤖 DummyActuatorNode ready — no real motors attached, "
            "commands will be logged only."
        )

    # =====================================================================
    # Abstract Method Implementations
    # =====================================================================

    def _execute_command(self, command: dict) -> bool:
        """
        Simulate actuator movement by computing and logging pan/tilt angles.

        In a real actuator, this method would:
            1. Convert error_x/error_y to motor commands.
            2. Send commands to servo/stepper drivers via GPIO/serial.
            3. Return True if the motor acknowledged the command.

        Args:
            command: Parsed tracking command dict with keys:
                - error_x (float): Horizontal offset [-1.0, 1.0]
                - error_y (float): Vertical offset [-1.0, 1.0]
                - target_id (int): Tracked object ID
                - class_name (str): Object class label
                - velocity_x (float): Horizontal velocity
                - velocity_y (float): Vertical velocity

        Returns:
            True always (simulation never fails).
        """
        error_x = command.get("error_x", 0.0)
        error_y = command.get("error_y", 0.0)
        target_id = command.get("target_id", -1)
        class_name = command.get("class_name", "unknown")
        velocity_x = command.get("velocity_x", 0.0)
        velocity_y = command.get("velocity_y", 0.0)

        # Compute simulated pan/tilt angles
        self._current_pan = error_x * self.PAN_RANGE_DEG
        self._current_tilt = error_y * self.TILT_RANGE_DEG

        # Log the simulated movement
        self.get_logger().info(
            f"🎯 Tracking ID:{target_id} [{class_name}] | "
            f"Pan: {self._current_pan:+6.1f}° | "
            f"Tilt: {self._current_tilt:+5.1f}° | "
            f"Velocity: ({velocity_x:+.1f}, {velocity_y:+.1f}) px/f"
        )

        return True

    def _stop(self) -> None:
        """
        Simulate emergency stop by resetting pan/tilt to center (0°, 0°).

        In a real actuator, this would:
            1. Immediately stop all motor movement.
            2. Return servos to a safe resting position.
            3. Optionally engage brakes for stepper motors.
        """
        self._current_pan = 0.0
        self._current_tilt = 0.0

        self.get_logger().info("🛑 STOP — Pan/Tilt reset to (0°, 0°)")

    # =====================================================================
    # Optional Overrides (demonstration)
    # =====================================================================

    def _on_startup(self) -> None:
        """
        Startup hook — called once after __init__.

        In a real actuator, you would initialize hardware here:
            - GPIO pin configuration
            - PWM channel setup
            - Serial port connection
            - Homing/calibration routine
        """
        self.get_logger().info("⚙️ Startup: Dummy actuator — no hardware to initialize")

    def _on_state_change(
        self,
        old_state: ActuatorState,
        new_state: ActuatorState,
    ) -> None:
        """
        React to state transitions.

        In a real actuator, you might:
            - Turn on an LED when TRACKING
            - Sound a buzzer on ERROR
            - Disable motors when IDLE
        """
        state_icons = {
            ActuatorState.IDLE: "💤",
            ActuatorState.TRACKING: "🏃",
            ActuatorState.ERROR: "❌",
        }

        icon = state_icons.get(new_state, "❓")
        self.get_logger().info(f"{icon} State: {old_state.value} → {new_state.value}")


# =============================================================================
# Entry Point
# =============================================================================


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for the dummy actuator node."""
    rclpy.init(args=args)
    node: Optional[DummyActuatorNode] = None

    try:
        node = DummyActuatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"DummyActuatorNode error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
