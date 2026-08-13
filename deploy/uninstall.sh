#!/usr/bin/env bash
set -euo pipefail

# Usage: ./deploy/uninstall.sh [hambot|oled|depthai|both|all]
# Removes systemd/NM hooks, .bashrc snippet, udev rules, and the shared venv.
# Does NOT delete the monorepo directory itself.
TARGET="${1:-both}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HAMBOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="$HAMBOT_DIR/hambot_venv"
DEPTHAI_DIR="$(dirname "$HAMBOT_DIR")/depthai-python"

BASHRC="$HOME/.bashrc"
SNIPPET_TAG="# >>> HamBot global auto-activate >>>"
SNIPPET_END="# <<< HamBot global auto-activate <<<"

UDEV_RULE="/etc/udev/rules.d/80-movidius.rules"
OLED_SERVICE="/etc/systemd/system/hambot_oled.service"
NM_DISPATCHER="/etc/NetworkManager/dispatcher.d/99-hambot-oled"

remove_hambot() {
  if [ -f "$BASHRC" ] && grep -qF "$SNIPPET_TAG" "$BASHRC"; then
    echo "==> Removing HamBot bash snippet from $BASHRC"
    awk -v start="$SNIPPET_TAG" -v end="$SNIPPET_END" '
      $0==start {inblk=1; next}
      $0==end   {inblk=0; next}
      !inblk {print}
    ' "$BASHRC" > "$BASHRC.tmp" && mv "$BASHRC.tmp" "$BASHRC"
  fi
  if [ -d "$VENV_DIR" ]; then
    echo "==> Removing venv: $VENV_DIR"
    rm -rf "$VENV_DIR"
  fi
  if [ -L "$HOME/Desktop/Demos" ]; then
    echo "==> Removing ~/Desktop/Demos symlink"
    rm -f "$HOME/Desktop/Demos"
  fi
  if [ -d "$HOME/Desktop/Examples" ]; then
    echo "==> Removing legacy ~/Desktop/Examples directory"
    rm -rf "$HOME/Desktop/Examples"
  fi
}

remove_oled() {
  if systemctl list-unit-files 2>/dev/null | grep -q "^hambot_oled.service"; then
    echo "==> Disabling hambot_oled.service"
    sudo systemctl disable hambot_oled.service || true
    sudo systemctl stop hambot_oled.service || true
  fi
  if [ -f "$OLED_SERVICE" ]; then
    sudo rm -f "$OLED_SERVICE"
    sudo systemctl daemon-reload
  fi
  if [ -f "$NM_DISPATCHER" ]; then
    sudo rm -f "$NM_DISPATCHER"
    sudo systemctl restart NetworkManager || true
  fi
}

remove_depthai() {
  if [ -f "$UDEV_RULE" ]; then
    sudo rm -f "$UDEV_RULE"
    sudo udevadm control --reload-rules && sudo udevadm trigger
  fi
  if [ -d "$DEPTHAI_DIR" ]; then
    echo "==> Removing $DEPTHAI_DIR"
    rm -rf "$DEPTHAI_DIR"
  fi
  if [ -f "$VENV_DIR/bin/activate" ]; then
    # shellcheck disable=SC1090
    source "$VENV_DIR/bin/activate"
    pip uninstall -y depthai simplejpeg || true
  fi
}

case "$TARGET" in
  hambot)  remove_hambot ;;
  oled)    remove_oled ;;
  depthai) remove_depthai ;;
  both)    remove_hambot; remove_oled ;;
  all)     remove_depthai; remove_oled; remove_hambot ;;
  *)
    echo "Usage: $0 [hambot|oled|depthai|both|all]"
    exit 1
    ;;
esac

echo "==> Uninstall complete for target: $TARGET"
echo "Note: open a NEW terminal to pick up ~/.bashrc changes."
