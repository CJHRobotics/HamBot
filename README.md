# HamBot

Monorepo for the HamBot robot platform (Raspberry Pi 4, LEGO Build HAT, BNO055 IMU, RPLidar, Pi Camera, optional OAK camera and SSD1306 OLED).

Active design work — the backend abstraction (`step()`, swappable drive
backends, the ROS2/OpenCR question) — is specified in [HANDOFF.md](HANDOFF.md).
Read that before starting on `packages/robot_systems`.

## Layout

```
HamBot/
├── packages/
│   ├── robot_systems/   # core library: motors, IMU, lidar, camera
│   ├── hambot_oled/     # SSD1306 network-status display
│   └── hambot_link/     # control listener + camera stream for the Vivid HamBot Console
├── demos/               # runnable scripts (symlinked to ~/Desktop/Demos on install)
├── deploy/              # Pi setup scripts (venv, systemd, udev, bashrc)
└── docs/                # figures + hardware notes (moved into packages/robot_systems)
```

## Quick start (on the Pi)

```bash
git clone https://github.com/CJHRobotics/HamBot.git ~/HamBot
cd ~/HamBot
./deploy/install.sh all     # or: hambot | oled | depthai | link | both
```

`all` includes the Vivid console control listener and camera stream. The narrower targets leave it out; add `--link` to include it with one of those, or use the `link` target to add only the listener to a robot that is already set up.

`install.sh` installs whatever is checked out. Pass `--ref` to have it select a version first — `--ref v0.2.0` to pin, `--ref main` to follow development. Restoring a robot by hand to the last known-good release is:

```bash
./deploy/install.sh --ref latest all
```

`install.sh` handles apt packages, the venv, `.bashrc`, systemd units, and the Desktop symlink — you'll be prompted for `sudo` when it needs it.

`install.sh` creates `~/HamBot/hambot_venv`, installs both packages into it, wires up `.bashrc` auto-activation, and (for `oled`/`all`) writes the systemd + NetworkManager hooks.

## Updating a robot

Use `--ref`, not `git pull`: once the installer has checked out a version the
repo sits on a detached HEAD, where `git pull` refuses to run.

```bash
./deploy/install.sh --ref latest all
```

## Development (any host)

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -e packages/robot_systems -e packages/hambot_oled -e packages/hambot_link
```

## Uninstall

```bash
./deploy/uninstall.sh all
```

Removes the venv, systemd unit, NetworkManager hook, udev rule, and `.bashrc` snippet. Does not delete the repo.
