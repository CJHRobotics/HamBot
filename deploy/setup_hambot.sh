#!/usr/bin/env bash
set -euo pipefail

# Usage: ./deploy/setup_hambot.sh
# Installs the HamBot robot_systems package into a venv inside the monorepo.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HAMBOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="$HAMBOT_DIR/hambot_venv"

echo "==> HamBot monorepo: $HAMBOT_DIR"

# 1) Create venv WITH system packages so picamera2/numpy from apt are visible
echo "==> Creating virtual environment with system packages..."
python3 -m venv --system-site-packages "$VENV_DIR"

echo "==> Installing robot_systems into the venv..."
# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"
python -m pip install --upgrade pip setuptools wheel
pip install "$HAMBOT_DIR/packages/robot_systems"

# 2) Global auto-activation for interactive bash sessions
BASHRC="$HOME/.bashrc"
SNIPPET_TAG="# >>> HamBot global auto-activate >>>"
SNIPPET_END="# <<< HamBot global auto-activate <<<"

AUTO_SNIPPET=$(cat <<EOF
# >>> HamBot global auto-activate >>>
# On interactive bash shells, automatically activate the HamBot venv.
# Preserve system 'python3' and 'pip3' so system tooling still works.
if [ -n "\$PS1" ] && [ -t 0 ]; then
  VENV_PATH="$VENV_DIR"
  if [ -d "\$VENV_PATH" ]; then
    if [ -z "\${VIRTUAL_ENV:-}" ] || [ "\$VIRTUAL_ENV" != "\$VENV_PATH" ]; then
      . "\$VENV_PATH/bin/activate"
    fi
    command -v /usr/bin/python3 >/dev/null 2>&1 && alias python3='/usr/bin/python3'
    command -v /usr/bin/pip3    >/dev/null 2>&1 && alias pip3='/usr/bin/pip3'
  fi
fi
# <<< HamBot global auto-activate <<<
EOF
)

if grep -qF "$SNIPPET_TAG" "$BASHRC" 2>/dev/null; then
  echo "==> Updating existing HamBot auto-activation snippet in $BASHRC"
  awk -v start="$SNIPPET_TAG" -v end="$SNIPPET_END" -v repl="$AUTO_SNIPPET" '
    $0==start {print repl; inblk=1; next}
    $0==end   {inblk=0; next}
    !inblk {print}
  ' "$BASHRC" > "$BASHRC.tmp"
  mv "$BASHRC.tmp" "$BASHRC"
else
  echo "==> Adding HamBot auto-activation snippet to $BASHRC"
  printf '\n%s\n%s\n' "$AUTO_SNIPPET" "$SNIPPET_END" >> "$BASHRC"
fi

echo "==> Done."
echo " - venv: $VENV_DIR"
echo " - New interactive terminals auto-activate the venv."
echo " - 'python3'/'pip3' remain the system defaults."
