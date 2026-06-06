"""
sensors/servo.py — Servo sweep controller.

Uses gpiozero's AngularServo on hardware PWM (GPIO12 = PWM channel 0).
The servo sweeps from SERVO_MIN_ANGLE to SERVO_MAX_ANGLE in steps of
SERVO_STEP_DEG, then reverses, continuously.

A caller can:
  - Ask for the current angle at any time (current_angle property).
  - Manually set a specific angle (set_angle).
  - Run an automatic sweep via step() in the main logger loop, or let
    the background sweep thread handle it (start_sweep / stop_sweep).
"""

import threading
import time
import logging
from typing import Optional

from gpiozero import AngularServo
from gpiozero.pins.rpigpio import RPiGPIOFactory

import config

logger = logging.getLogger(__name__)


class ServoSweep:
    """
    Controls a hobby servo for lidar scanning.

    Two operating modes:
      Manual: call step() yourself (used by logger.py for synchronised scanning)
      Auto:   call start_sweep() to run a background thread
    """

    def __init__(self):
        # Use RPiGPIOFactory to ensure hardware PWM is available on GPIO12
        factory = RPiGPIOFactory()
        self._servo = AngularServo(
            pin=config.SERVO_GPIO_PIN,
            min_angle=config.SERVO_MIN_ANGLE,
            max_angle=config.SERVO_MAX_ANGLE,
            min_pulse_width=config.SERVO_MIN_PULSE_WIDTH,
            max_pulse_width=config.SERVO_MAX_PULSE_WIDTH,
            pin_factory=factory,
        )
        self._angle: float = 0.0
        self._direction: float = 1.0  # +1 = sweeping toward max, -1 = toward min
        self._lock = threading.Lock()
        self._sweep_thread: Optional[threading.Thread] = None
        self._running = False

        # Move to start position
        self.set_angle(config.SERVO_MIN_ANGLE)
        logger.info(
            "ServoSweep ready on GPIO%d. Range: %.1f° to %.1f°, step: %.1f°",
            config.SERVO_GPIO_PIN,
            config.SERVO_MIN_ANGLE,
            config.SERVO_MAX_ANGLE,
            config.SERVO_STEP_DEG,
        )

    # ── Public API ────────────────────────────────────────────────────────────

    @property
    def current_angle(self) -> float:
        """Current servo angle in degrees. Thread-safe."""
        with self._lock:
            return self._angle

    def set_angle(self, angle: float) -> None:
        """
        Move servo to an absolute angle and update internal state.
        Clamps to [SERVO_MIN_ANGLE, SERVO_MAX_ANGLE].
        """
        clamped = max(config.SERVO_MIN_ANGLE, min(config.SERVO_MAX_ANGLE, angle))
        self._servo.angle = clamped
        with self._lock:
            self._angle = clamped

    def step(self) -> float:
        """
        Advance the servo by one step in the current sweep direction.
        Reverses direction at the limits.
        Returns the new angle.

        Call this from the main logger loop to keep lidar and servo in lockstep.
        """
        with self._lock:
            new_angle = self._angle + self._direction * config.SERVO_STEP_DEG

            # Reverse at limits
            if new_angle >= config.SERVO_MAX_ANGLE:
                new_angle = config.SERVO_MAX_ANGLE
                self._direction = -1.0
            elif new_angle <= config.SERVO_MIN_ANGLE:
                new_angle = config.SERVO_MIN_ANGLE
                self._direction = 1.0

            self._angle = new_angle

        self._servo.angle = new_angle
        return new_angle

    def center(self) -> None:
        """Move to 0° (nadir). Useful for shutdown / calibration."""
        self.set_angle(0.0)

    def start_sweep(self) -> None:
        """
        Start an automatic background sweep thread.
        Use this for standalone testing; in normal logging use step() instead.
        """
        if self._sweep_thread and self._sweep_thread.is_alive():
            logger.warning("Sweep thread already running.")
            return
        self._running = True
        self._sweep_thread = threading.Thread(
            target=self._sweep_loop, daemon=True, name="servo-sweep"
        )
        self._sweep_thread.start()
        logger.info("Background sweep started.")

    def stop_sweep(self) -> None:
        """Stop the automatic sweep thread and center the servo."""
        self._running = False
        if self._sweep_thread:
            self._sweep_thread.join(timeout=3.0)
        self.center()
        logger.info("Sweep stopped, servo centered.")

    def close(self) -> None:
        """Release GPIO resources."""
        self.stop_sweep()
        self._servo.close()

    # ── Internal ──────────────────────────────────────────────────────────────

    def _sweep_loop(self) -> None:
        while self._running:
            self.step()
            time.sleep(config.SERVO_DWELL_SEC)
