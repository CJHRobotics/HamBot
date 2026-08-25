#!/usr/bin/env bash
set -euo pipefail

# Usage: ./deploy/setup_link.sh
# Installs the Vivid HamBot Console control listener into the shared HamBot
# venv and puts it on PATH for non-interactive SSH sessions.
#
# There is no systemd unit here on purpose: the console launches one listener
# per driving session over SSH, so the robot is only held while someone is
# actually driving it.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HAMBOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="$HAMBOT_DIR/hambot_venv"
VENV_PY="$VENV_DIR/bin/python"
VENV_BIN="$VENV_DIR/bin/hambot-link"
VENV_CAM="$VENV_DIR/bin/hambot-camera"
LINK_PATH="/usr/local/bin/hambot-link"
CAM_PATH="/usr/local/bin/hambot-camera"

if [ ! -x "$VENV_PY" ]; then
  echo "ERROR: HamBot venv not found at $VENV_DIR"
  echo "       Run setup_hambot.sh first."
  exit 1
fi

if ! "$VENV_PY" -c "import robot_systems" >/dev/null 2>&1; then
  echo "ERROR: robot_systems is not installed in $VENV_DIR"
  echo "       Run setup_hambot.sh first."
  exit 1
fi

echo "[1/3] Installing hambot_link into $VENV_DIR..."
# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"
pip install "$HAMBOT_DIR/packages/hambot_link"

echo "[2/3] Exposing hambot-link and hambot-camera on PATH..."
# `ssh hambot@robot hambot-link` runs a non-login, non-interactive shell, so the
# .bashrc venv auto-activation never fires. Symlinks in /usr/local/bin keep the
# console's SSH commands short and independent of the venv location.
sudo ln -sfn "$VENV_BIN" "$LINK_PATH"
sudo ln -sfn "$VENV_CAM" "$CAM_PATH"

echo "[3/3] Verifying..."
"$LINK_PATH" --help > /dev/null
"$CAM_PATH" --help > /dev/null

echo "==> Control link setup complete."
echo " - control: $LINK_PATH -> $VENV_BIN"
echo " - camera:  $CAM_PATH -> $VENV_CAM"
echo " - the console invokes them as: ssh hambot@<robot> hambot-link"
echo "                                ssh hambot@<robot> hambot-camera"
echo " - install the console's public key in ~/.ssh/authorized_keys to allow it."
