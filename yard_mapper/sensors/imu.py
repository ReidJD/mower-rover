"""
sensors/imu.py — BNO055 9-DOF IMU driver.

Uses the Adafruit CircuitPython BNO055 library (adafruit-circuitpython-bno055).
The BNO055 runs onboard sensor fusion and outputs calibrated Euler angles
(heading, roll, pitch) directly over I2C.

NOTE: The Teyleten Robot BNO055 module (ASIN B0D47G672B) uses the BNO055 chip,
NOT the newer BNO085. The BNO055 and BNO085 have different libraries and
slightly different APIs, but equivalent capability for this project.

A background thread polls the IMU at IMU_POLL_HZ. Call get_reading() from
any thread to obtain the latest angles.

Wiring (Pi 4):
    BNO055 VCC  → 3.3 V (pin 1)
    BNO055 GND  → GND   (pin 6)
    BNO055 SDA  → GPIO2 (pin 3)  — I2C1 SDA
    BNO055 SCL  → GPIO3 (pin 5)  — I2C1 SCL

The module uses I2C address 0x28 by default (0x29 if the ADR pin is pulled high).
Verify with: i2cdetect -y 1

Calibration note: The BNO055 requires a brief calibration routine on first use.
The sensor will report calibration status 0–3 for each subsystem (sys, gyro,
accel, mag). The _poll_loop logs a warning if calibration is below 2/3.
Walk the sensor in a figure-8 pattern to calibrate the magnetometer.
"""

import threading
import time
import logging
from dataclasses import dataclass, field
from typing import Optional

import config

logger = logging.getLogger(__name__)


@dataclass
class ImuReading:
    roll_deg: float      # Rotation around forward axis (+= right side down)
    pitch_deg: float     # Rotation around lateral axis (+= nose down)
    heading_deg: float   # Magnetic heading, 0–360
    timestamp: float     # time.monotonic()


# A zero reading used before the first valid sample arrives
_ZERO_READING = ImuReading(roll_deg=0.0, pitch_deg=0.0, heading_deg=0.0, timestamp=0.0)


class ImuSensor:
    """
    Background reader for the BNO085.

    Usage:
        imu = ImuSensor()
        imu.start()
        reading = imu.get_reading()
        imu.stop()
    """

    def __init__(self, poll_hz: int = config.IMU_POLL_HZ):
        self._poll_interval = 1.0 / poll_hz
        self._lock = threading.Lock()
        self._latest: ImuReading = _ZERO_READING
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._sample_count = 0

    # ── Public API ────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Initialise the BNO055 and start the background polling thread."""
        # Import here so that non-Pi environments (e.g., laptop) can import
        # sensors/imu.py without crashing on missing adafruit libraries.
        try:
            import board
            import adafruit_bno055

            i2c = board.I2C()
            self._bno = adafruit_bno055.BNO055_I2C(i2c)
            # Use NDOF mode (Nine Degrees Of Freedom) — the default fusion mode.
            # This gives absolute orientation relative to magnetic north.
            self._bno.mode = adafruit_bno055.NDOF_MODE
        except Exception as exc:
            raise RuntimeError(f"Failed to initialise BNO055: {exc}") from exc

        self._running = True
        self._thread = threading.Thread(
            target=self._poll_loop, daemon=True, name="imu-poller"
        )
        self._thread.start()
        logger.info("ImuSensor started at %d Hz", config.IMU_POLL_HZ)

    def stop(self) -> None:
        """Stop the polling thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        logger.info("ImuSensor stopped. Samples: %d", self._sample_count)

    def get_reading(self) -> ImuReading:
        """
        Return the most recent IMU reading. Returns a zero reading if the
        sensor hasn't produced data yet. Thread-safe.
        """
        with self._lock:
            return self._latest

    @property
    def sample_count(self) -> int:
        return self._sample_count

    # ── Internal ──────────────────────────────────────────────────────────────

    def _poll_loop(self) -> None:
        _calib_warned = False
        while self._running:
            try:
                # BNO055 .euler returns (heading, roll, pitch) in degrees.
                # Returns None for each axis if calibration is insufficient.
                euler = self._bno.euler
                if euler is not None and all(v is not None for v in euler):
                    heading, roll, pitch = euler

                    # Log calibration status periodically (0=uncalibrated, 3=fully calibrated)
                    if not _calib_warned:
                        cal = self._bno.calibration_status  # (sys, gyro, accel, mag)
                        if any(c < 2 for c in cal):
                            logger.warning(
                                "BNO055 calibration low: sys=%d gyro=%d accel=%d mag=%d — "
                                "move sensor in a figure-8 to calibrate magnetometer",
                                *cal,
                            )
                        else:
                            _calib_warned = True  # stop warning once calibrated

                    reading = ImuReading(
                        roll_deg=float(roll),
                        pitch_deg=float(pitch),
                        heading_deg=float(heading),
                        timestamp=time.monotonic(),
                    )
                    with self._lock:
                        self._latest = reading
                    self._sample_count += 1
            except Exception as exc:  # noqa: BLE001
                logger.warning("IMU read error: %s", exc)

            time.sleep(self._poll_interval)
