#!/usr/bin/env bash
set -euo pipefail

# Usage: ./deploy/setup_depthai.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HAMBOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="$HAMBOT_DIR/hambot_venv"
DEPTHAI_DIR="$(dirname "$HAMBOT_DIR")/depthai-python"

# Verify HamBot venv exists before proceeding
if [ ! -d "$VENV_DIR" ]; then
  echo "ERROR: HamBot venv not found at $VENV_DIR"
  echo "       Run setup_hambot.sh first."
  exit 1
fi

# 1) udev rule (outside venv, requires sudo)
echo "==> Installing Movidius/OAK udev rule..."
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules > /dev/null
sudo udevadm control --reload-rules && sudo udevadm trigger

# 2) pip installs inside hambot_venv
echo "==> Activating HamBot venv and installing depthai dependencies..."
# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"
pip install --upgrade pip
pip install --extra-index-url https://www.piwheels.org/simple/ opencv-python numpy depthai

# 3) Clone depthai-python and run install_requirements.py
echo "==> Cloning depthai-python..."
if [ -d "$DEPTHAI_DIR/.git" ]; then
  echo "==> depthai-python already cloned. Pulling latest..."
  git -C "$DEPTHAI_DIR" pull --ff-only
else
  git clone https://github.com/luxonis/depthai-python.git "$DEPTHAI_DIR"
fi

echo "==> Running depthai install_requirements.py..."
cd "$DEPTHAI_DIR/examples"
python install_requirements.py
cd - >/dev/null

# 4) Install simplejpeg
echo "==> Installing simplejpeg..."
pip install simplejpeg --force-reinstall --no-cache-dir

echo "==> DepthAI setup complete."
echo
echo "What this means:"
echo " - Movidius/OAK udev rule written to /etc/udev/rules.d/80-movidius.rules"
echo " - depthai, opencv-python, numpy, simplejpeg installed into $VENV_DIR"
echo " - depthai-python examples cloned to $DEPTHAI_DIR"
echo " - Reconnect your OAK camera (unplug/replug) if it was already plugged in."
