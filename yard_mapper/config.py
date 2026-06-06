"""
config.py — Central configuration for yard_mapper.

Edit this file to match your wiring and physical setup before running.
All GPIO numbers use BCM (Broadcom) numbering.
"""

# ── Serial port ───────────────────────────────────────────────────────────────
# TFmini-S connects to the Pi's primary UART.
# On Pi 4 this is /dev/serial0 (mapped to /dev/ttyAMA0 after disabling
# the serial console in setup.sh).
LIDAR_PORT: str = "/dev/serial0"
LIDAR_BAUD: int = 115200  # TFmini-S default; do not change unless you've reconfigured the sensor

# ── Servo ─────────────────────────────────────────────────────────────────────
# GPIO pin for servo signal wire (BCM numbering).
# GPIO12 is hardware PWM channel 0 — preferred over software PWM.
SERVO_GPIO_PIN: int = 12

# Sweep range in degrees. 0 = nadir (straight down).
# Positive = mower's right side, negative = mower's left side.
SERVO_MIN_ANGLE: float = -60.0   # degrees
SERVO_MAX_ANGLE: float = 60.0    # degrees
SERVO_STEP_DEG: float = 2.0      # degrees per step

# Dwell time at each step before taking a lidar reading (seconds).
# 50 ms gives the servo time to settle and the lidar time to return a fresh frame.
SERVO_DWELL_SEC: float = 0.05

# Pulse width calibration — adjust if your servo doesn't reach full travel.
# Values below are typical for most hobby servos (500 µs to 2500 µs).
SERVO_MIN_PULSE_WIDTH: float = 0.0005   # seconds (500 µs)
SERVO_MAX_PULSE_WIDTH: float = 0.0025   # seconds (2500 µs)

# ── IMU ───────────────────────────────────────────────────────────────────────
# BNO085 connects via I2C (SDA=GPIO2, SCL=GPIO3 — Pi hardware I2C bus 1).
# No pin config needed here; adafruit-blinka auto-detects board.SCL / board.SDA.
IMU_POLL_HZ: int = 50  # How often the IMU background thread reads (Hz)

# ── Database ──────────────────────────────────────────────────────────────────
DB_PATH: str = "/home/pi/yard_mapper_data/scan.db"
# The directory will be created automatically by database.py if it doesn't exist.

# ── Physical constants ────────────────────────────────────────────────────────
# Height of the lidar sensor above ground level when the mower is on flat
# ground, measured in millimetres. Measure with a tape after mounting.
# Used by the post-processing script; not needed at log time.
SENSOR_HEIGHT_MM: float = 600.0   # TODO: update after physical mounting

# Direction the servo sweeps relative to the mower.
# "right_positive" means positive angles point to the mower's right.
# This affects the sign of the lateral offset in post-processing.
SWEEP_DIRECTION: str = "right_positive"
