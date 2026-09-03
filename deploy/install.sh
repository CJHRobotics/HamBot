#!/usr/bin/env bash
set -euo pipefail

# Usage: ./deploy/install.sh [--force] [--link] [--ref REF] [hambot|oled|depthai|link|both|all]
# Defaults: target=all, ref=current
#
#   hambot  — robot_systems venv + .bashrc auto-activation + demos symlink
#   oled    — OLED display (installs into the shared venv + systemd + NM hook)
#   depthai — DepthAI/OAK camera udev rules + pip deps (requires hambot first)
#   link    — Vivid console control listener (requires hambot first)
#   both    — hambot + oled
#   all     — hambot + oled + depthai + link
#
#   --force   Delete and recreate the venv before installing. Use if the venv
#             is corrupted or you want a clean start.
#   --link    Also install the console control listener alongside a target that
#             does not already include it ('hambot', 'oled', 'depthai', 'both').
#             Redundant with 'all'. To add only the listener to a robot that is
#             already set up, use the 'link' target.
#   --ref     Check out a version of the monorepo before installing, so one
#             command both moves the robot and installs it.
#               current   install whatever is checked out (default) — leaves
#                         git alone, so callers that already chose a version
#                         (the lab Ansible playbook) stay the authority
#               latest    newest vX.Y.Z release tag; this is how you restore a
#                         robot by hand to the last known-good release
#               main      latest development state
#               vX.Y.Z    a specific release, e.g. --ref v0.2.0
#               <branch>  a feature branch; <sha> a specific commit
#             Anything but 'current' refuses to run if the working tree has
#             uncommitted changes, rather than discarding them.

FORCE=0
LINK=0
TARGET=""
REF=""
REF_EXPLICIT=0
# argv minus --ref, for the hand-off after a checkout: the ref has already been
# applied by then, and a release older than this flag would reject it outright.
PASSTHRU=()
while [ $# -gt 0 ]; do
  case "$1" in
    --force|-f) FORCE=1; PASSTHRU+=("$1") ;;
    --link|-l)  LINK=1;  PASSTHRU+=("$1") ;;
    --ref)
      if [ $# -lt 2 ]; then
        echo "ERROR: --ref needs a value (latest, main, vX.Y.Z, <branch>, <sha>, current)."
        exit 1
      fi
      REF="$2"
      REF_EXPLICIT=1
      shift
      ;;
    --ref=*) REF="${1#--ref=}"; REF_EXPLICIT=1 ;;
    hambot|oled|depthai|link|both|all) TARGET="$1"; PASSTHRU+=("$1") ;;
    -h|--help)
      sed -n '/^# Usage:/,/^$/p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown argument: $1"
      echo "Usage: $0 [--force] [--link] [--ref REF] [hambot|oled|depthai|link|both|all]"
      exit 1
      ;;
  esac
  shift
done
TARGET="${TARGET:-all}"
REF="${REF:-current}"

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

# Resolve --ref to a concrete commit and check it out. Runs before apt and the
# venv so the rest of the install always matches the code on disk.
checkout_ref() {
    if [ "$REF" = "current" ]; then
        # Stay silent when this is just the default, so callers that already
        # chose a version (the lab Ansible playbook) see no new output.
        if [ "$REF_EXPLICIT" = "1" ]; then
            echo "==> --ref current: leaving the checkout alone."
        fi
        return
    fi

    if ! git -C "$HAMBOT_DIR" rev-parse --git-dir >/dev/null 2>&1; then
        echo "ERROR: $HAMBOT_DIR is not a git clone, so --ref cannot select a version."
        echo "       Re-run with '--ref current' to install what is on disk."
        exit 1
    fi

    # A dirty tree means someone is mid-edit. Checking out over that either
    # fails halfway or silently discards their work, so stop and let them choose.
    if [ -n "$(git -C "$HAMBOT_DIR" status --porcelain)" ]; then
        echo "ERROR: $HAMBOT_DIR has uncommitted changes; refusing to check out '$REF'."
        echo "       Commit or stash them, or re-run with '--ref current' to keep them."
        exit 1
    fi

    echo "==> Fetching refs from origin..."
    if ! git -C "$HAMBOT_DIR" fetch --tags --prune origin; then
        echo "    WARNING: fetch failed (offline?); resolving '$REF' from local refs only."
    fi

    local resolved="$REF"
    if [ "$REF" = "latest" ]; then
        resolved="$(git -C "$HAMBOT_DIR" tag -l 'v*' --sort=-v:refname | head -n 1)"
        if [ -z "$resolved" ]; then
            echo "ERROR: no vX.Y.Z release tags found, so '--ref latest' has nothing to install."
            echo "       Use '--ref main' for the development state, or tag a release first."
            exit 1
        fi
        echo "==> --ref latest resolves to $resolved"
    fi

    # For a branch, "install $REF" means its current tip, not whatever this
    # clone last fetched into the local branch.
    if git -C "$HAMBOT_DIR" show-ref --verify --quiet "refs/remotes/origin/$resolved"; then
        resolved="origin/$resolved"
    fi

    if ! git -C "$HAMBOT_DIR" rev-parse --verify --quiet "$resolved^{commit}" >/dev/null; then
        echo "ERROR: '$REF' is not a known tag, branch, or commit."
        exit 1
    fi

    echo "==> Checking out $resolved ($(git -C "$HAMBOT_DIR" rev-parse --short "$resolved^{commit}"))..."
    git -C "$HAMBOT_DIR" checkout --detach --quiet "$resolved"
    git -C "$HAMBOT_DIR" --no-pager log -1 --format='    %h %s'
}

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

# 'all' installs the listener and --link asks for it too, so guard against
# running the setup twice when both are given.
LINK_INSTALLED=0
install_link() {
    if [ "$LINK_INSTALLED" = "0" ]; then
        "$SCRIPT_DIR/setup_link.sh"
        LINK_INSTALLED=1
    fi
}

if [ "$FORCE" = "1" ] && [[ "$TARGET" != "hambot" && "$TARGET" != "both" && "$TARGET" != "all" ]]; then
    echo "ERROR: --force needs a target that rebuilds the venv (hambot, both, all)."
    echo "       '$TARGET' alone would delete the venv without recreating it."
    exit 1
fi

# The checkout can replace this script mid-run, and bash reads scripts
# incrementally — so hand off to the new version instead of running a mix.
if [ "${HAMBOT_REF_APPLIED:-0}" != "1" ]; then
    SELF_BEFORE="$(sha256sum "$0" | cut -d' ' -f1)"
    checkout_ref
    export HAMBOT_REF_APPLIED=1
    if [ "$(sha256sum "$0" | cut -d' ' -f1)" != "$SELF_BEFORE" ]; then
        echo "==> install.sh changed with the checkout; re-running that version."
        exec "$0" ${PASSTHRU[@]+"${PASSTHRU[@]}"}
    fi
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
  link)    install_link ;;
  both)
    "$SCRIPT_DIR/setup_hambot.sh"
    "$SCRIPT_DIR/setup_oled.sh"
    link_demos
    ;;
  all)
    "$SCRIPT_DIR/setup_hambot.sh"
    "$SCRIPT_DIR/setup_oled.sh"
    "$SCRIPT_DIR/setup_depthai.sh"
    install_link
    link_demos
    ;;
  *)
    echo "Usage: $0 [--force] [--link] [--ref REF] [hambot|oled|depthai|link|both|all]"
    exit 1
    ;;
esac

echo "==> Install complete for target: $TARGET"
echo "    version: $(git -C "$HAMBOT_DIR" describe --tags --always 2>/dev/null || echo 'unknown')"
