"""
logger.py — Main data-collection orchestrator for yard_mapper.

Starts all sensors, then runs a synchronised step-and-measure loop:
  1. Step servo one position
  2. Wait SERVO_DWELL_SEC for servo to settle
  3. Read latest lidar distance
  4. Stamp with current IMU reading
  5. Write row to database

Handles SIGINT (Ctrl-C) gracefully: flushes the database, centers the
servo, and exits cleanly.

Usage:
    python logger.py [--db PATH] [--duration SECONDS]

Options:
    --db PATH          Override database path from config.py
    --duration SECS    Stop after this many seconds (default: run until Ctrl-C)
    --verbose          Enable DEBUG logging
"""

import argparse
import logging
import signal
import sys
import time

import config
from sensors.lidar import LidarSensor, INVALID_MM
from sensors.servo import ServoSweep
from sensors.imu import ImuSensor
from storage.database import ScanDatabase, ScanRow


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Yard mapper data logger")
    p.add_argument("--db", default=config.DB_PATH, help="Path to SQLite database")
    p.add_argument("--duration", type=float, default=None, help="Run for N seconds then stop")
    p.add_argument("--verbose", action="store_true", help="Enable DEBUG logging")
    return p.parse_args()


def setup_logging(verbose: bool) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
        level=level,
    )


def main() -> None:
    args = parse_args()
    setup_logging(args.verbose)
    log = logging.getLogger("logger")

    # ── Initialise hardware ───────────────────────────────────────────────────
    log.info("Initialising sensors...")
    lidar = LidarSensor()
    servo = ServoSweep()
    imu = ImuSensor()

    lidar.start()
    imu.start()

    # Brief warm-up: let lidar and IMU produce their first readings
    log.info("Warming up sensors (2 s)...")
    time.sleep(2.0)

    # ── Database ──────────────────────────────────────────────────────────────
    db = ScanDatabase(path=args.db)
    db.open()

    # ── Graceful shutdown on SIGINT ───────────────────────────────────────────
    _shutdown = False

    def _handle_sigint(sig, frame):
        nonlocal _shutdown
        log.info("Shutdown signal received.")
        _shutdown = True

    signal.signal(signal.SIGINT, _handle_sigint)
    signal.signal(signal.SIGTERM, _handle_sigint)

    # ── Main loop ─────────────────────────────────────────────────────────────
    log.info("Logging started. Press Ctrl-C to stop.")
    start_time = time.monotonic()
    loop_count = 0
    invalid_count = 0

    try:
        while not _shutdown:
            # Check duration limit
            if args.duration and (time.monotonic() - start_time) >= args.duration:
                log.info("Duration limit reached (%.1f s).", args.duration)
                break

            # Step servo and wait for it to settle
            angle = servo.step()
            time.sleep(config.SERVO_DWELL_SEC)

            # Read sensors
            lidar_reading = lidar.get_reading()
            imu_reading = imu.get_reading()

            if lidar_reading is None:
                invalid_count += 1
                log.debug("No lidar reading yet (step %d)", loop_count)
                continue

            distance_mm = lidar_reading.distance_mm
            strength = lidar_reading.strength

            # Log a warning if signal is weak but still record the row
            if strength < 100:
                log.debug("Low lidar signal strength: %d at angle %.1f°", strength, angle)

            row = ScanRow(
                servo_deg=angle,
                lidar_mm=distance_mm,
                strength=strength,
                imu_roll_deg=imu_reading.roll_deg,
                imu_pitch_deg=imu_reading.pitch_deg,
                imu_heading_deg=imu_reading.heading_deg,
                # GPS fields will be added in Phase 3
            )
            db.append_row(row)
            loop_count += 1

            # Progress report every 500 rows
            if loop_count % 500 == 0:
                elapsed = time.monotonic() - start_time
                log.info(
                    "%d rows | %.1f rows/s | lidar: %d mm @ %.1f° | roll: %.1f° pitch: %.1f°",
                    loop_count,
                    loop_count / elapsed,
                    distance_mm,
                    angle,
                    imu_reading.roll_deg,
                    imu_reading.pitch_deg,
                )

    finally:
        log.info("Shutting down...")
        db.flush()
        db.close()
        servo.close()
        lidar.stop()
        imu.stop()

        elapsed = time.monotonic() - start_time
        log.info(
            "Done. %d rows written in %.1f s (%.1f rows/s). "
            "%d invalid lidar reads. "
            "Lidar errors: %d.",
            loop_count,
            elapsed,
            loop_count / elapsed if elapsed > 0 else 0,
            invalid_count,
            lidar.error_count,
        )


if __name__ == "__main__":
    main()
