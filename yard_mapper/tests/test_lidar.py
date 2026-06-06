"""
tests/test_lidar.py — Interactive bench test for the TFmini-S.

Run this FIRST before combining sensors.

What it does:
  1. Opens the TFmini-S on LIDAR_PORT.
  2. Prints live distance readings to the console at ~5 Hz.
  3. After 10 seconds, prints a summary (min, max, mean, std-dev).

Pass the --accuracy flag to run an accuracy validation: you'll be prompted
to hold the sensor at 5 known distances; the script records 50 samples at
each and reports the error.

Usage (from the yard_mapper directory):
    python -m tests.test_lidar
    python -m tests.test_lidar --accuracy
    python -m tests.test_lidar --port /dev/ttyUSB0   # if using USB-UART adapter
"""

import argparse
import statistics
import sys
import time

# Allow running from the project root
sys.path.insert(0, ".")

from sensors.lidar import LidarSensor, INVALID_MM
import config


def live_display(duration: float, port: str) -> None:
    lidar = LidarSensor(port=port)
    lidar.start()

    print(f"\nLive lidar output ({duration:.0f} s) — hold Ctrl-C to stop early\n")
    print(f"{'Time':>6}  {'Dist (mm)':>10}  {'Strength':>10}  {'Frames':>8}  {'Errors':>8}")
    print("-" * 55)

    samples = []
    start = time.monotonic()
    try:
        while True:
            elapsed = time.monotonic() - start
            if elapsed >= duration:
                break

            r = lidar.get_reading()
            if r and r.distance_mm != INVALID_MM:
                samples.append(r.distance_mm)
                print(
                    f"{elapsed:6.1f}  {r.distance_mm:>10}  {r.strength:>10}  "
                    f"{lidar.frame_count:>8}  {lidar.error_count:>8}",
                    end="\r",
                )
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        lidar.stop()

    print()  # newline after \r
    if samples:
        print(f"\n── Summary ({'{}' .format(len(samples))} samples) ──────────────────")
        print(f"  Min : {min(samples)} mm")
        print(f"  Max : {max(samples)} mm")
        print(f"  Mean: {statistics.mean(samples):.1f} mm")
        print(f"  StdD: {statistics.stdev(samples):.1f} mm" if len(samples) > 1 else "")
    else:
        print("No valid samples received. Check wiring and port.")


def accuracy_test(port: str) -> None:
    """Prompt user to hold the sensor at 5 known distances and report error."""
    known_distances_mm = [300, 500, 750, 1000, 1500]
    lidar = LidarSensor(port=port)
    lidar.start()
    time.sleep(1.0)  # let the stream start

    print("\n── Accuracy validation ──────────────────────────────────")
    print("Hold a flat target perpendicular to the sensor beam.")
    print("The script will take 50 samples at each distance.\n")

    results = []
    for target_mm in known_distances_mm:
        input(f"Position target at {target_mm} mm, then press Enter...")
        samples = []
        deadline = time.monotonic() + 2.0  # collect for 2 seconds
        while time.monotonic() < deadline:
            r = lidar.get_reading()
            if r and r.distance_mm != INVALID_MM and r.strength >= 100:
                samples.append(r.distance_mm)
            time.sleep(0.02)

        if not samples:
            print(f"  {target_mm} mm: NO DATA")
            continue

        mean = statistics.mean(samples)
        error = mean - target_mm
        results.append((target_mm, mean, error))
        print(f"  {target_mm:5} mm target | {mean:7.1f} mm measured | error: {error:+.1f} mm | n={len(samples)}")

    lidar.stop()

    if results:
        errors = [abs(r[2]) for r in results]
        print(f"\nMean absolute error: {statistics.mean(errors):.1f} mm")
        print("Accuracy test complete.")


def main() -> None:
    p = argparse.ArgumentParser(description="TFmini-S bench test")
    p.add_argument("--port", default=config.LIDAR_PORT)
    p.add_argument("--duration", type=float, default=15.0, help="Live display duration (s)")
    p.add_argument("--accuracy", action="store_true", help="Run accuracy validation instead")
    args = p.parse_args()

    if args.accuracy:
        accuracy_test(args.port)
    else:
        live_display(args.duration, args.port)


if __name__ == "__main__":
    main()
