#!/usr/bin/env bash
set -euo pipefail

# Usage: ./deploy/install.sh [--force] [--link] [hambot|oled|depthai|link|both|all]
# Defaults: target=all
#
#   hambot  — robot_systems venv + .bashrc auto-activation + demos symlink
#   oled    — OLED display (installs into the shared venv + systemd + NM hook)
#   depthai — DepthAI/OAK camera udev rules + pip deps (requires hambot first)
#   link    — Vivid console control listener (requires hambot first)
#   both    — hambot + oled
#   all     — hambot + oled + depthai
#
#   --force   Delete and recreate the venv before installing. Use if the venv
#             is corrupted or you want a clean start.
#   --link    Also install the console control listener alongside the target.
#             'all' does not include it by default. To add only the listener
#             to a robot that is already set up, use the 'link' target - or
#             run deploy/setup_link.sh directly.

FORCE=0
LINK=0
TARGET=""
for arg in "$@"; do
  case "$arg" in
    --force|-f) FORCE=1 ;;
    --link|-l)  LINK=1 ;;
    hambot|oled|depthai|link|both|all) TARGET="$arg" ;;
    -h|--help)
      sed -n '3,20p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown argument: $arg"
      echo "Usage: $0 [--force] [--link] [hambot|oled|depthai|link|both|all]"
      exit 1
      ;;
  esac
done
TARGET="${TARGET:-all}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HAMBOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="$HAMBOT_DIR/hambot_venv"

APT_PACKAGES=(
    python3-opencv          # cv2 (color_shuttle, cameraGUI)
    python3-pil.imagetk     # PIL.ImageTk for cameraGUI
    python3-tk              # tkinter for cameraGUI
    network-manager         # nmcli for OLED network display
    wireless-tools          # iwgetid for OLED network display
)

apt_install() {
    echo "==> Installing system packages (apt)..."
    sudo apt-get update
    sudo apt-get install -y "${APT_PACKAGES[@]}"
}

force_rebuild_venv() {
    if [ -d "$VENV_DIR" ]; then
        echo "==> --force: removing existing venv at $VENV_DIR"
        rm -rf "$VENV_DIR"
    fi
}

link_demos() {
    echo "==> Linking demos to ~/Desktop/Demos..."
    mkdir -p "$HOME/Desktop"
    ln -sfn "$HAMBOT_DIR/demos" "$HOME/Desktop/Demos"
}

if [ "$FORCE" = "1" ] && [[ "$TARGET" != "hambot" && "$TARGET" != "both" && "$TARGET" != "all" ]]; then
    echo "ERROR: --force needs a target that rebuilds the venv (hambot, both, all)."
    echo "       '$TARGET' alone would delete the venv without recreating it."
    exit 1
fi

# The link target installs no apt packages, so skip the package pass entirely.
# Adding the listener to a working robot should not touch its packages.
if [ "$TARGET" != "link" ]; then
    apt_install
fi

if [ "$FORCE" = "1" ]; then
    force_rebuild_venv
fi

case "$TARGET" in
  hambot)  "$SCRIPT_DIR/setup_hambot.sh";  link_demos ;;
  oled)    "$SCRIPT_DIR/setup_oled.sh" ;;
  depthai) "$SCRIPT_DIR/setup_depthai.sh" ;;
  link)    "$SCRIPT_DIR/setup_link.sh" ;;
  both)
    "$SCRIPT_DIR/setup_hambot.sh"
    "$SCRIPT_DIR/setup_oled.sh"
    link_demos
    ;;
  all)
    "$SCRIPT_DIR/setup_hambot.sh"
    "$SCRIPT_DIR/setup_oled.sh"
    "$SCRIPT_DIR/setup_depthai.sh"
    link_demos
    ;;
  *)
    echo "Usage: $0 [--force] [--link] [hambot|oled|depthai|link|both|all]"
    exit 1
    ;;
esac

if [ "$LINK" = "1" ] && [ "$TARGET" != "link" ]; then
    "$SCRIPT_DIR/setup_link.sh"
fi

echo "==> Install complete for target: $TARGET"
