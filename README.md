# Mower-Mounted Yard Terrain Mapper

A learning project to build a Raspberry Pi-based terrain mapping device that mounts on a riding mower. The device sweeps a single-point LiDAR across the mower's track while logging GPS position and IMU tilt data, then post-processes the results into a 3D elevation map — showing where to top-dress or backfill for a smooth lawn.

---

## Project Status

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | LiDAR + servo sweep + IMU tilt correction | 🔧 In Progress |
| 2 | RTK GPS integration (u-blox ZED-F9P) | 📋 Planned |
| 3 | Post-processing & 3D map generation | 📋 Planned |
| 4 | Enclosure, weatherproofing & field use | 📋 Planned |

---

## How It Works

A TFmini-S LiDAR is mounted on a servo that continuously sweeps ±60° across the mower's path. As the mower drives, each sweep captures a cross-section of the terrain below. A BNO055 IMU measures the mower's roll and pitch so the raw distances can be corrected for tilt. RTK GPS (Phase 2) provides centimeter-accurate position for each scan.

After a mowing session, the logged data is post-processed into a point cloud and rendered as a heatmap showing high and low spots across the yard.

### Sensor Geometry

```
          servo angle θ (from nadir)
          ↓
  ┌───────────────────────┐  ← enclosure (mounted on mower)
  │    LiDAR + servo      │
  └────────┬──────────────┘
           │  d = raw LiDAR distance
           │╲ θ
           │ ╲
           │  ╲
    h_below = d·cos(θ + pitch)    ← height above ground
    y_offset = d·sin(θ)·cos(roll) ← lateral offset
```

---

## Hardware

### Bill of Materials (~$595)

| Component | Part | ~Price |
|-----------|------|--------|
| Compute | Raspberry Pi 4 (2GB+) | $45 |
| LiDAR | TFmini-S (12m range, UART) | $30 |
| Servo | MG996R (metal gear, 180°) | $10 |
| IMU | BNO055 9-DOF (I2C) | $15 |
| GPS (Phase 2) | Ardusimple SimpleRTK2B (ZED-F9P) | $220 |
| GPS antenna | L1/L2 survey antenna | $75 |
| Power | PlusRoc IP68 12V→5V buck converter | $20 |
| USB-C connector | DWEII panel-mount USB-C (9mm hole) | $10 |
| Cable glands | PG7 metric (M12 thread, 3–6.5mm cable) | $8 |
| MicroSD | 32GB+ Class 10 | $10 |
| Hardware | M2/M3/M5 screws, wire, connectors | ~$15 |
| Enclosure | 3D printed (PLA or PETG) | ~$5 |

### Wiring

**Power**
- Mower battery (12V) → PlusRoc buck converter → 5V USB-C out
- DWEII panel-mount USB-C → 2-wire pigtail → GPIO pin 4 (+5V) & pin 6 (GND)
- ⚠️ The Pi's built-in USB-C port is too close to the enclosure wall for a cable — power via GPIO instead

**LiDAR (TFmini-S)**
- TX → GPIO15 / RXD (`/dev/serial0`)
- VCC → 5V, GND → GND

**Servo (MG996R)**
- Signal → GPIO12 (hardware PWM)
- VCC → 5V (dedicated rail), GND → GND

**IMU (BNO055)**
- SDA → GPIO2, SCL → GPIO3 (I2C1)
- VCC → 3.3V, GND → GND
- Default I2C address: `0x28` (verify with `i2cdetect -y 1`)

---

## Software

### Repository Structure

```
mower-rover/
├── README.md
├── yard_mapper/
│   ├── config.py               # All tunable parameters
│   ├── logger.py               # Main data-capture loop
│   ├── requirements.txt        # Python dependencies
│   ├── setup.sh                # Pi setup script (venv + interfaces)
│   ├── sensors/
│   │   ├── lidar.py            # TFmini-S background reader thread
│   │   ├── servo.py            # MG996R sweep controller
│   │   └── imu.py              # BNO055 background reader thread
│   ├── storage/
│   │   └── database.py         # SQLite WAL data logger
│   └── tests/
│       ├── test_lidar.py       # LiDAR bench test & accuracy check
│       ├── test_servo.py       # Servo sweep & positioning test
│       └── test_imu.py         # IMU live stream, tilt & noise tests
└── enclosure/
    └── yard_mapper_enclosure.py  # CadQuery parametric enclosure design
```

### Key Parameters (`config.py`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `LIDAR_PORT` | `/dev/serial0` | UART port for TFmini-S |
| `SERVO_GPIO_PIN` | `12` | Hardware PWM GPIO pin |
| `SERVO_MIN_ANGLE` | `-60.0` | Sweep left limit (degrees) |
| `SERVO_MAX_ANGLE` | `60.0` | Sweep right limit (degrees) |
| `SERVO_STEP_DEG` | `2.0` | Step size per measurement |
| `SERVO_DWELL_SEC` | `0.05` | Settle time before reading LiDAR |
| `IMU_POLL_HZ` | `50` | IMU background poll rate |
| `SENSOR_HEIGHT_MM` | `600.0` | Nominal sensor height above ground |

---

## Raspberry Pi Setup

### 1. Enable Interfaces

```bash
sudo raspi-config nonint do_i2c 0           # Enable I2C
sudo raspi-config nonint do_serial_hw 0     # Enable UART hardware
sudo raspi-config nonint do_serial_cons 1   # Disable serial console (free UART for LiDAR)
sudo reboot
```

Verify after reboot:
```bash
ls /dev/i2c-1 /dev/serial0    # Both should exist
i2cdetect -y 1                # Should show 0x28 (BNO055)
```

### 2. Clone and Install

```bash
git clone https://github.com/ReidJD/mower-rover.git
cd mower-rover/yard_mapper
chmod +x setup.sh && ./setup.sh
```

`setup.sh` creates a Python venv at `~/venv/yard_mapper` and installs all dependencies.

### 3. Activate the Venv

```bash
source ~/venv/yard_mapper/bin/activate
cd ~/mower-rover/yard_mapper
```

---

## Usage

### Bench Test Each Sensor

```bash
# LiDAR — live distance stream
python -m tests.test_lidar

# LiDAR — accuracy validation (5 known distances, 50 samples each)
python -m tests.test_lidar --accuracy

# Servo — full sweep test
python -m tests.test_servo --test sweep

# Servo — step through angles manually (press Enter to advance)
python -m tests.test_servo --test steps

# IMU — live roll/pitch/heading stream
python -m tests.test_imu --test live

# IMU — guided tilt accuracy test
python -m tests.test_imu --test tilt

# IMU — noise floor measurement (hold still 30s)
python -m tests.test_imu --test noise
```

### Start Logging

```bash
# Log to default DB path (set in config.py)
python logger.py

# Custom path and duration (seconds)
python logger.py --db /home/pi/scans/session_01.db --duration 3600

# Verbose output (print each row)
python logger.py --verbose
```

Press **Ctrl-C** to stop gracefully. Data is committed to SQLite every 100 rows.

---

## Enclosure

The enclosure is designed in CadQuery (Python-based parametric CAD). Open `enclosure/yard_mapper_enclosure.py` in [CQ-editor](https://github.com/CadQuery/CQ-editor) to preview and export STL for 3D printing.

**Features:**
- Two PG7 cable glands (servo/LiDAR bundle, GPS spare)
- 9mm panel-mount hole for DWEII USB-C power connector (thin-wall, connector flange handles retention)
- Internal BNO055 shelf with M2 mounting posts
- Top flanges with M3 tap holes for lid attachment
- M5 bottom holes for mower-rail pipe saddle clamps
- Fully parametric — adjust wall thickness, Pi footprint, tube OD

**Print recommendations:** PETG for outdoor use, 3 perimeters, 20% infill, no supports needed.

---

## Data Schema

Each row in the SQLite database represents one LiDAR measurement:

| Column | Type | Description |
|--------|------|-------------|
| `id` | INTEGER | Auto-increment primary key |
| `ts` | REAL | `time.monotonic()` timestamp |
| `servo_deg` | REAL | Servo angle at time of reading |
| `lidar_mm` | INTEGER | Raw LiDAR distance (mm) |
| `strength` | INTEGER | LiDAR signal strength (>100 = valid) |
| `imu_roll` | REAL | Mower roll (degrees) |
| `imu_pitch` | REAL | Mower pitch (degrees) |
| `imu_heading` | REAL | Magnetic heading (degrees) |
| `gps_lat` | REAL | Latitude (Phase 2, nullable) |
| `gps_lon` | REAL | Longitude (Phase 2, nullable) |
| `gps_alt` | REAL | Altitude m (Phase 2, nullable) |
| `gps_fix` | INTEGER | RTK fix quality 0–5 (Phase 2, nullable) |

---

## Phase 2: RTK GPS

The GPS subsystem uses a **u-blox ZED-F9P** (Ardusimple SimpleRTK2B) for centimeter-accurate positioning. A fixed base station sends RTCM correction data to the rover over WiFi via NTRIP.

```
Base station (fixed) → WiFi → NTRIP caster on Pi → rover ZED-F9P (serial)
```

---

## License

MIT — learn from it, adapt it, improve it.
