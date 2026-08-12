#!/usr/bin/env bash
set -euo pipefail

# Usage: ./deploy/install.sh [hambot|oled|depthai|both|all]
# Defaults: target=all
#
#   hambot  — robot_systems venv + .bashrc auto-activation
#   oled    — OLED display (installs into the shared venv + systemd + NM hook)
#   depthai — DepthAI/OAK camera udev rules + pip deps (requires hambot first)
#   both    — hambot + oled
#   all     — hambot + oled + depthai
TARGET="${1:-all}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HAMBOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

copy_examples() {
    echo "==> Copying examples to ~/Desktop/Examples..."
    mkdir -p "$HOME/Desktop/Examples"
    cp -r "$HAMBOT_DIR/examples/." "$HOME/Desktop/Examples/"
}

case "$TARGET" in
  hambot)  "$SCRIPT_DIR/setup_hambot.sh";  copy_examples ;;
  oled)    "$SCRIPT_DIR/setup_oled.sh" ;;
  depthai) "$SCRIPT_DIR/setup_depthai.sh" ;;
  both)
    "$SCRIPT_DIR/setup_hambot.sh"
    "$SCRIPT_DIR/setup_oled.sh"
    copy_examples
    ;;
  all)
    "$SCRIPT_DIR/setup_hambot.sh"
    "$SCRIPT_DIR/setup_oled.sh"
    "$SCRIPT_DIR/setup_depthai.sh"
    copy_examples
    ;;
  *)
    echo "Usage: $0 [hambot|oled|depthai|both|all]"
    exit 1
    ;;
esac

echo "==> Install complete for target: $TARGET"
