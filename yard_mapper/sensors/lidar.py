"""
sensors/lidar.py — TFmini-S single-point lidar driver.

The TFmini-S sends 9-byte frames continuously at the configured rate (default 100 Hz):
    Byte 0: 0x59  (header)
    Byte 1: 0x59  (header)
    Byte 2: DIST_L  (distance low byte, cm)
    Byte 3: DIST_H  (distance high byte, cm)
    Byte 4: STRENGTH_L
    Byte 5: STRENGTH_H
    Byte 6: TEMP_L   (raw temperature, not in °C)
    Byte 7: TEMP_H
    Byte 8: CHECKSUM (sum of bytes 0-7, mod 256)

This module runs a background thread that reads frames continuously.
Call get_reading() from any thread to obtain the latest valid measurement.
"""

import threading
import time
import logging
from dataclasses import dataclass
from typing import Optional

import serial

import config

logger = logging.getLogger(__name__)

FRAME_HEADER = 0x59
FRAME_LEN = 9

# Signal strength below this value indicates a low-confidence reading.
MIN_STRENGTH = 100

# Sentinel value returned when no valid reading is available yet.
INVALID_MM = -1


@dataclass
class LidarReading:
    distance_mm: int       # Distance in millimetres
    strength: int          # Signal strength (higher = better)
    temperature_raw: int   # Raw temperature word (not calibrated °C)
    timestamp: float       # time.monotonic() when the frame was parsed


class LidarSensor:
    """
    Continuous background reader for the TFmini-S.

    Usage:
        lidar = LidarSensor()
        lidar.start()
        reading = lidar.get_reading()   # latest valid frame
        lidar.stop()
    """

    def __init__(self, port: str = config.LIDAR_PORT, baud: int = config.LIDAR_BAUD):
        self._port = port
        self._baud = baud
        self._serial: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._latest: Optional[LidarReading] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._frame_count = 0
        self._error_count = 0

    # ── Public API ────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Open the serial port and start the background reader thread."""
        self._serial = serial.Serial(
            port=self._port,
            baudrate=self._baud,
            timeout=0.1,
        )
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True, name="lidar-reader")
        self._thread.start()
        logger.info("LidarSensor started on %s @ %d baud", self._port, self._baud)

    def stop(self) -> None:
        """Stop the reader thread and close the serial port."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        logger.info(
            "LidarSensor stopped. Frames: %d valid, %d errors.",
            self._frame_count,
            self._error_count,
        )

    def get_reading(self) -> Optional[LidarReading]:
        """
        Return the most recent valid LidarReading, or None if no valid
        frame has been received yet.  Thread-safe.
        """
        with self._lock:
            return self._latest

    def get_distance_mm(self) -> int:
        """
        Convenience method — returns distance in mm, or INVALID_MM (-1)
        if no reading is available or the last reading had low signal strength.
        """
        reading = self.get_reading()
        if reading is None:
            return INVALID_MM
        if reading.strength < MIN_STRENGTH:
            return INVALID_MM
        return reading.distance_mm

    @property
    def frame_count(self) -> int:
        return self._frame_count

    @property
    def error_count(self) -> int:
        return self._error_count

    # ── Internal ──────────────────────────────────────────────────────────────

    def _read_loop(self) -> None:
        """Background thread: synchronise to the frame stream and parse frames."""
        buf = bytearray()
        while self._running:
            try:
                chunk = self._serial.read(FRAME_LEN)
                if not chunk:
                    continue
                buf.extend(chunk)

                # Scan buffer for a valid frame header pair
                while len(buf) >= FRAME_LEN:
                    # Find start of frame
                    if buf[0] != FRAME_HEADER or buf[1] != FRAME_HEADER:
                        # Discard one byte and re-scan (handles stream misalignment)
                        buf = buf[1:]
                        self._error_count += 1
                        continue

                    frame = buf[:FRAME_LEN]
                    expected_checksum = sum(frame[:8]) & 0xFF
                    if frame[8] != expected_checksum:
                        # Bad checksum — discard and re-sync
                        buf = buf[1:]
                        self._error_count += 1
                        logger.debug("Checksum mismatch: got 0x%02X expected 0x%02X", frame[8], expected_checksum)
                        continue

                    # Valid frame — parse
                    distance_cm = (frame[3] << 8) | frame[2]
                    strength = (frame[5] << 8) | frame[4]
                    temp_raw = (frame[7] << 8) | frame[6]

                    reading = LidarReading(
                        distance_mm=distance_cm * 10,
                        strength=strength,
                        temperature_raw=temp_raw,
                        timestamp=time.monotonic(),
                    )
                    with self._lock:
                        self._latest = reading
                    self._frame_count += 1
                    buf = buf[FRAME_LEN:]  # consume the frame

            except serial.SerialException as exc:
                logger.error("Serial error in lidar read loop: %s", exc)
                time.sleep(0.1)
            except Exception as exc:  # noqa: BLE001
                logger.exception("Unexpected error in lidar read loop: %s", exc)
                time.sleep(0.1)
