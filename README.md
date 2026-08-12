# HamBot

Monorepo for the HamBot robot platform (Raspberry Pi 4, LEGO Build HAT, BNO055 IMU, RPLidar, Pi Camera, optional OAK camera and SSD1306 OLED).

## Layout

```
HamBot/
├── packages/
│   ├── robot_systems/   # core library: motors, IMU, lidar, camera
│   └── hambot_oled/     # SSD1306 network-status display
├── demos/               # runnable example scripts
├── examples/            # copied to ~/Desktop/Examples on install
├── deploy/              # Pi setup scripts (venv, systemd, udev, bashrc)
└── docs/                # figures + hardware notes (moved into packages/robot_systems)
```

## Quick start (on the Pi)

```bash
git clone git@github.com:biorobaw/HamBot.git ~/HamBot
cd ~/HamBot
sudo apt update && sudo apt install -y python3-opencv network-manager wireless-tools
./deploy/install.sh all     # or: hambot | oled | depthai | both
```

`install.sh` creates `~/HamBot/hambot_venv`, installs both packages into it, wires up `.bashrc` auto-activation, and (for `oled`/`all`) writes the systemd + NetworkManager hooks.

## Development (any host)

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -e packages/robot_systems -e packages/hambot_oled
```

## Uninstall

```bash
./deploy/uninstall.sh all
```

Removes the venv, systemd unit, NetworkManager hook, udev rule, and `.bashrc` snippet. Does not delete the repo.
