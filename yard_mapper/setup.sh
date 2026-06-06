#!/usr/bin/env bash
# ============================================================
# yard_mapper — Raspberry Pi setup script
# Run once after imaging Pi OS Lite (64-bit).
# Usage: chmod +x setup.sh && ./setup.sh
# ============================================================
set -e

echo "==> Updating package lists..."
sudo apt-get update -qq

echo "==> Installing system dependencies..."
sudo apt-get install -y -qq \
    python3-pip \
    python3-venv \
    python3-dev \
    i2c-tools \
    libgpiod2 \
    git

# ── Enable UART and I2C via raspi-config non-interactively ──────────────────
echo "==> Enabling UART (serial0)..."
# Disable serial console so we can use UART for lidar
sudo raspi-config nonint do_serial_hw 0    # enable hardware serial
sudo raspi-config nonint do_serial_cons 1  # disable serial console on ttyAMA0

echo "==> Enabling I2C..."
sudo raspi-config nonint do_i2c 0

# ── Python virtual environment ──────────────────────────────────────────────
echo "==> Creating Python virtual environment at ~/yard_mapper_venv..."
python3 -m venv ~/yard_mapper_venv
source ~/yard_mapper_venv/bin/activate

echo "==> Installing Python packages..."
pip install --upgrade pip -q
pip install -r requirements.txt -q

# ── Verify I2C and serial devices are visible ────────────────────────────────
echo ""
echo "==> Checking I2C bus..."
if i2cdetect -y 1 2>/dev/null | grep -q "[0-9a-f][0-9a-f]"; then
    echo "    I2C devices found."
else
    echo "    No I2C devices detected yet (normal if BNO085 not wired up)."
fi

echo ""
echo "==> Checking serial port..."
if [ -e /dev/serial0 ]; then
    echo "    /dev/serial0 exists. Good."
else
    echo "    WARNING: /dev/serial0 not found. A reboot may be needed."
fi

echo ""
echo "============================================================"
echo "  Setup complete. Activate the venv before running scripts:"
echo "  source ~/yard_mapper_venv/bin/activate"
echo ""
echo "  If UART or I2C were just enabled, reboot now:"
echo "  sudo reboot"
echo "============================================================"
