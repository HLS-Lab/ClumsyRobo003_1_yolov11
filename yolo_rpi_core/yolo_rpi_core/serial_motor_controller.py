#!/usr/bin/env python3
"""
Serial Motor Controller — low-level hardware I/O adapter.

Wraps UART communication with the motor controller board (control_v1 protocol).

Protocol (inferred from control_v1/serial_test.py):
    Transport  : UART / USB-Serial (/dev/ttyUSB0)
    Baud rate  : 115200
    Framing    : JSON + newline delimiter  →  '{"angle1": 90, "angle2": 90}\\n'
    Response   : JSON line from controller (ACK / telemetry, logged but not blocking)

Usage::

    # Real hardware
    ctrl = SerialMotorController('/dev/ttyUSB0', 115200)
    ctrl.connect()
    ctrl.send(angle1=90, angle2=90)
    ctrl.close()

    # Simulation (no hardware)
    ctrl = SerialMotorController(simulation_mode=True)
    ctrl.connect()   # always succeeds
    ctrl.send(90, 90)  # prints to stdout instead of writing serial

Compatible with: ROS 2 Jazzy, Python 3.12
Target Platform: Raspberry Pi 4 (CPU-only, NO CUDA)
"""

from __future__ import annotations

import json
import logging
import threading
import time
from typing import Optional


logger = logging.getLogger(__name__)


class SerialMotorController:
    """
    Low-level serial I/O adapter for the motor controller board.

    Implements the control_v1 JSON-over-UART protocol::

        TX  →  '{"angle1": <float>, "angle2": <float>}\\n'
        RX  ←  '{"ack": 1}\\n'  (or any JSON — logged, not blocking)

    Args:
        port:            Serial device path (e.g. '/dev/ttyUSB0').
        baud:            Baud rate (default 115200).
        timeout:         Serial read/write timeout in seconds (default 1.0).
        simulation_mode: If True, no real serial I/O is performed; safe for CI
                         and development without hardware.
    """

    def __init__(
        self,
        port: str = '/dev/ttyUSB0',
        baud: int = 115200,
        timeout: float = 1.0,
        simulation_mode: bool = False,
    ) -> None:
        self._port = port
        self._baud = baud
        self._timeout = timeout
        self._simulation_mode = simulation_mode

        self._serial: Optional[object] = None   # serial.Serial instance
        self._reader_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._connected = False

        # Last known commanded angles (for status reporting)
        self._last_angle1: float = 90.0
        self._last_angle2: float = 90.0

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """
        Open the serial port and start the background reader thread.

        Returns:
            True if connection succeeded (or simulation mode is active).
        """
        if self._simulation_mode:
            logger.info(
                '[SerialMotorController] SIMULATION MODE — no serial port opened.'
            )
            self._connected = True
            return True

        try:
            import serial as _serial  # imported lazily to allow CI without pyserial
            self._serial = _serial.Serial(
                self._port,
                self._baud,
                timeout=self._timeout,
            )
            self._connected = True
            logger.info(
                f'[SerialMotorController] Connected to {self._port} at {self._baud} baud.'
            )
            # Start background reader (daemon — exits when main process exits)
            self._stop_event.clear()
            self._reader_thread = threading.Thread(
                target=self._reader_loop,
                daemon=True,
                name='serial_reader',
            )
            self._reader_thread.start()
            return True

        except Exception as exc:
            logger.error(f'[SerialMotorController] Failed to connect: {exc}')
            self._connected = False
            return False

    def close(self) -> None:
        """Close the serial port and stop the reader thread."""
        self._stop_event.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=2.0)

        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

        self._connected = False
        logger.info('[SerialMotorController] Closed.')

    @property
    def is_connected(self) -> bool:
        """True if the controller is ready to accept commands."""
        return self._connected

    @property
    def last_angles(self) -> tuple[float, float]:
        """Last commanded (angle1, angle2) pair."""
        return (self._last_angle1, self._last_angle2)

    # ------------------------------------------------------------------
    # Command sending
    # ------------------------------------------------------------------

    def send(self, angle1: float, angle2: float) -> bool:
        """
        Send a motor angle command to the controller.

        Encodes the command as::

            {"angle1": <angle1>, "angle2": <angle2>}\\n

        Args:
            angle1: Pan motor target angle in degrees (0–180, center = 90).
            angle2: Tilt motor target angle in degrees (0–180, center = 90).

        Returns:
            True if the command was sent successfully, False on error.
        """
        self._last_angle1 = angle1
        self._last_angle2 = angle2

        payload = json.dumps({'angle1': round(angle1, 2), 'angle2': round(angle2, 2)}) + '\n'

        if self._simulation_mode:
            logger.info(f'[SIM] SEND → {payload.strip()}')
            return True

        if not self._connected or self._serial is None:
            logger.error('[SerialMotorController] Not connected — cannot send.')
            return False

        try:
            self._serial.write(payload.encode('utf-8'))
            logger.debug(f'[SerialMotorController] SEND → {payload.strip()}')
            return True
        except Exception as exc:
            logger.error(f'[SerialMotorController] Write error: {exc}')
            return False

    # ------------------------------------------------------------------
    # Background reader (non-blocking — log only)
    # ------------------------------------------------------------------

    def _reader_loop(self) -> None:
        """
        Background thread: read lines from the controller and log them.

        This mirrors the pattern from control_v1/serial_test.py (read_from_port).
        Does not block command sending; purely informational.
        """
        while not self._stop_event.is_set():
            try:
                if self._serial and self._serial.in_waiting > 0:
                    line = self._serial.readline().decode('utf-8', errors='replace').strip()
                    if line:
                        try:
                            data = json.loads(line)
                            logger.debug(f'[SerialMotorController] RECV ← {data}')
                        except json.JSONDecodeError:
                            logger.debug(f'[SerialMotorController] RECV (raw) ← {line}')
            except OSError as exc:
                logger.error(f'[SerialMotorController] Serial read error: {exc}')
                break
            except Exception as exc:
                logger.warning(f'[SerialMotorController] Reader error: {exc}')
            time.sleep(0.01)  # 10 ms poll — same as control_v1/serial_test.py
