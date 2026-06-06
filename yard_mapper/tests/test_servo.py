"""
tests/test_servo.py — Interactive bench test for the servo.

Run this after confirming the servo is wired to GPIO12.

Tests (pick via --test flag):
  sweep   : Run one full sweep from min to max angle and back. Verify visually.
  steps   : Step through all angles one by one, pausing so you can inspect each.
  goto    : Move to a specific angle (--angle). Useful for pulse-width calibration.
  center  : Move to 0° (nadir) and hold.

Usage:
    python -m tests.test_servo --test sweep
    python -m tests.test_servo --test steps
    python -m tests.test_servo --test goto --angle 45
    python -m tests.test_servo --test center
"""

import argparse
import sys
import time

sys.path.insert(0, ".")

from sensors.servo import ServoSweep
import config


def test_sweep(servo: ServoSweep, sweeps: int = 2) -> None:
    print(f"\nRunning {sweeps} full sweep(s): {config.SERVO_MIN_ANGLE}° ↔ {config.SERVO_MAX_ANGLE}°")
    print("Watch the servo physically — it should reach both ends smoothly.\n")
    total_steps = int((config.SERVO_MAX_ANGLE - config.SERVO_MIN_ANGLE) / config.SERVO_STEP_DEG)
    total_steps_for_run = total_steps * 2 * sweeps  # forward + back per sweep

    for i in range(total_steps_for_run):
        angle = servo.step()
        print(f"  Step {i+1:4d}/{total_steps_for_run}  angle: {angle:+6.1f}°", end="\r")
        time.sleep(config.SERVO_DWELL_SEC)

    print("\nSweep test complete.")


def test_steps(servo: ServoSweep) -> None:
    print(f"\nStepping through angles from {config.SERVO_MIN_ANGLE}° to {config.SERVO_MAX_ANGLE}°")
    print("Press Enter at each step to advance, or Ctrl-C to stop.\n")
    angles = []
    a = config.SERVO_MIN_ANGLE
    while a <= config.SERVO_MAX_ANGLE:
        angles.append(a)
        a += config.SERVO_STEP_DEG

    for angle in angles:
        servo.set_angle(angle)
        try:
            input(f"  → {angle:+6.1f}°  (press Enter for next)")
        except KeyboardInterrupt:
            print("\nStopped.")
            break

    print("Steps test complete.")


def test_goto(servo: ServoSweep, target: float) -> None:
    print(f"\nMoving to {target}° and holding. Press Ctrl-C to exit.")
    servo.set_angle(target)
    try:
        while True:
            time.sleep(0.5)
            print(f"  Current angle: {servo.current_angle:+6.1f}°", end="\r")
    except KeyboardInterrupt:
        pass
    print()


def test_center(servo: ServoSweep) -> None:
    print("\nMoving to 0° (nadir/center). Press Ctrl-C to exit.")
    servo.center()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


def main() -> None:
    p = argparse.ArgumentParser(description="Servo bench test")
    p.add_argument("--test", choices=["sweep", "steps", "goto", "center"], default="sweep")
    p.add_argument("--angle", type=float, default=0.0, help="Target angle for --test goto")
    p.add_argument("--sweeps", type=int, default=2, help="Number of sweeps for --test sweep")
    args = p.parse_args()

    print("Initialising servo...")
    servo = ServoSweep()

    try:
        if args.test == "sweep":
            test_sweep(servo, args.sweeps)
        elif args.test == "steps":
            test_steps(servo)
        elif args.test == "goto":
            test_goto(servo, args.angle)
        elif args.test == "center":
            test_center(servo)
    finally:
        servo.close()
        print("Servo closed.")


if __name__ == "__main__":
    main()
