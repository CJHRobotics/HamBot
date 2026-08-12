#!/usr/bin/env bash
set -euo pipefail

# Usage: ./deploy/setup_oled.sh
# Installs the OLED network display into the shared HamBot venv and wires up
# a systemd oneshot + NetworkManager dispatcher hook.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HAMBOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="$HAMBOT_DIR/hambot_venv"
VENV_PY="$VENV_DIR/bin/python"
PY_SCRIPT="$HAMBOT_DIR/packages/hambot_oled/src/hambot_oled/network_display.py"
RUN_USER="${SUDO_USER:-$USER}"

if [ ! -x "$VENV_PY" ]; then
  echo "ERROR: HamBot venv not found at $VENV_DIR"
  echo "       Run setup_hambot.sh first."
  exit 1
fi

echo "[1/3] Installing hambot_oled into $VENV_DIR..."
# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"
pip install "$HAMBOT_DIR/packages/hambot_oled"

echo "[2/3] Writing NetworkManager dispatcher hook..."
DISPATCHER_SCRIPT="/etc/NetworkManager/dispatcher.d/99-hambot-oled"
sudo tee "$DISPATCHER_SCRIPT" > /dev/null <<EOF
#!/bin/sh
# Run once on each network state change
exec $VENV_PY $PY_SCRIPT
EOF
sudo chmod +x "$DISPATCHER_SCRIPT"
sudo systemctl restart NetworkManager

echo "[3/3] Writing systemd oneshot boot service..."
SYSTEMD_SERVICE="/etc/systemd/system/hambot_oled.service"
sudo tee "$SYSTEMD_SERVICE" > /dev/null <<EOF
[Unit]
Description=Run OLED network display once at boot
Wants=network-online.target
After=network-online.target
ConditionPathExists=/dev/i2c-1

[Service]
Type=oneshot
User=${RUN_USER}
WorkingDirectory=${HAMBOT_DIR}
Environment=PYTHONUNBUFFERED=1
ExecStartPre=/bin/sh -c 'for i in 1 2 3; do [ -e /dev/i2c-1 ] && exit 0; sleep 1; done; exit 0'
ExecStart=${VENV_PY} ${PY_SCRIPT}
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable hambot_oled.service
sudo systemctl start hambot_oled.service

echo "==> OLED setup complete."
