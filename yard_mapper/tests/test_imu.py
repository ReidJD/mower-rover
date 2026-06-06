"""
tests/test_imu.py — Interactive bench test for the BNO055 IMU.

Run this after confirming the BNO055 is wired to I2C (SDA=GPIO2, SCL=GPIO3).
Before running, verify the sensor is detected:
    i2cdetect -y 1
You should see address 0x28 (default) or 0x29 (if ADR pin is pulled high).

Tests:
  live   : Stream roll/pitch/heading at 5 Hz. Tilt the board and watch the values.
  tilt   : Guided tilt test — you'll be prompted to hold known angles; reports error.
  noise  : Hold the sensor still for 30 s and measure angle noise (std-dev).

Usage:
    python -m tests.test_imu --test live
    python -m tests.test_imu --test tilt
    python -m tests.test_imu --test noise
"""

import argparse
import math
import statistics
import sys
import time

sys.path.insert(0, ".")

from sensors.imu import ImuSensor


def test_live(duration: float) -> None:
    imu = ImuSensor()
    imu.start()
    time.sleep(0.5)

    print(f"\nLive IMU output ({duration:.0f} s) — tilt the sensor and watch values\n")
    print(f"{'Time':>6}  {'Roll':>8}  {'Pitch':>8}  {'Heading':>10}  {'Samples':>8}")
    print("-" * 55)

    start = time.monotonic()
    try:
        while True:
            elapsed = time.monotonic() - start
            if elapsed >= duration:
                break
            r = imu.get_reading()
            print(
                f"{elapsed:6.1f}  {r.roll_deg:>+7.2f}°  {r.pitch_deg:>+7.2f}°  "
                f"{r.heading_deg:>9.2f}°  {imu.sample_count:>8}",
                end="\r",
            )
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        imu.stop()
    print()


def test_tilt(imu: ImuSensor) -> None:
    """Prompt user to tilt the sensor to known angles; report accuracy."""
    print("\n── Tilt accuracy test ───────────────────────────────────")
    print("For each prompt, tilt the sensor to the specified roll angle")
    print("and hold still. The script records 30 samples.\n")

    known_rolls = [0, 10, 20, 30, -10, -20, -30]
    results = []

    for target in known_rolls:
        input(f"Tilt to {target:+d}° roll (flat side down = 0°), then press Enter...")
        samples = []
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            r = imu.get_reading()
            samples.append(r.roll_deg)
            time.sleep(0.05)

        mean_roll = statistics.mean(samples)
        error = mean_roll - target
        results.append((target, mean_roll, error))
        print(f"  Target: {target:+4d}°  Measured: {mean_roll:+7.2f}°  Error: {error:+.2f}°  n={len(samples)}")

    errors = [abs(r[2]) for r in results]
    print(f"\nMean absolute roll error: {statistics.mean(errors):.2f}°")
    print("Tilt test complete.")


def test_noise(duration: float = 30.0) -> None:
    """Hold the sensor perfectly still and measure noise floor."""
    imu = ImuSensor()
    imu.start()
    time.sleep(1.0)

    print(f"\n── Noise test ({duration:.0f} s) ──────────────────────────────")
    print("Hold the sensor completely still on a flat, vibration-free surface.")
    print("Recording...")

    rolls, pitches, headings = [], [], []
    start = time.monotonic()
    try:
        while time.monotonic() - start < duration:
            r = imu.get_reading()
            rolls.append(r.roll_deg)
            pitches.append(r.pitch_deg)
            headings.append(r.heading_deg)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        imu.stop()

    def stats(values, label):
        if len(values) < 2:
            return
        print(f"  {label:<10}  mean: {statistics.mean(values):+8.3f}°  "
              f"std: {statistics.stdev(values):.4f}°  "
              f"range: {max(values)-min(values):.4f}°")

    print(f"\n── Results ({len(rolls)} samples) ──────────────────────────────")
    stats(rolls, "Roll")
    stats(pitches, "Pitch")
    stats(headings, "Heading")
    print("\nFor mower use, roll/pitch std-dev should ideally be < 0.5° at rest.")


def main() -> None:
    p = argparse.ArgumentParser(description="BNO085 IMU bench test")
    p.add_argument("--test", choices=["live", "tilt", "noise"], default="live")
    p.add_argument("--duration", type=float, default=30.0)
    args = p.parse_args()

    if args.test == "live":
        test_live(args.duration)
    elif args.test == "tilt":
        imu = ImuSensor()
        imu.start()
        time.sleep(1.0)
        try:
            test_tilt(imu)
        finally:
            imu.stop()
    elif args.test == "noise":
        test_noise(args.duration)


if __name__ == "__main__":
    main()
