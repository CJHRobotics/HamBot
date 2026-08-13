# HamBot Deployment

Scripts for setting up the HamBot stack on a Raspberry Pi 4 running **64-bit Raspberry Pi OS (Bookworm)**.

Everything installs into a single shared venv at `<monorepo>/hambot_venv`. No external repos are cloned — the packages come from `packages/` in this monorepo.

## System requirements

Bookworm's default image already includes `git`, `python3-venv`, `python3-pip`, `python3-picamera2`, and `python3-numpy`. Everything else — `python3-opencv`, `python3-pil.imagetk`, `python3-tk`, `network-manager`, `wireless-tools` — is installed automatically by `install.sh` (via `sudo apt`).

If you plan to use the OLED, also enable I²C once:

```bash
sudo raspi-config nonint do_i2c 0
sudo usermod -aG i2c $USER    # log out/in afterwards
```

## Install

```bash
./deploy/install.sh [target]
```

Targets:

| Target | What it does |
| --- | --- |
| `hambot` | Creates `hambot_venv`, installs `packages/robot_systems`, adds a `.bashrc` auto-activation snippet, and symlinks `demos/` to `~/Desktop/Demos` (so `git pull` refreshes the shortcut). |
| `oled` | Installs `packages/hambot_oled` into the shared venv, writes `/etc/systemd/system/hambot_oled.service` and a NetworkManager dispatcher hook. Requires `hambot` first. |
| `depthai` | Adds the OAK-camera udev rule, `pip install`s `depthai`/`opencv-python`/`simplejpeg` into the shared venv, and clones `depthai-python` next to the monorepo. Requires `hambot` first. |
| `both` | `hambot` + `oled`. |
| `all` (default) | `hambot` + `oled` + `depthai`. |

## Individual scripts

Each step can also be run standalone: `setup_hambot.sh`, `setup_oled.sh`, `setup_depthai.sh`. They discover the monorepo location from their own path, so they work wherever you cloned it.

## Uninstall

```bash
./deploy/uninstall.sh [target]
```

Removes the venv, systemd unit, NetworkManager hook, udev rule, and the `.bashrc` snippet. It does **not** delete the monorepo directory.

Open a new terminal afterwards to pick up the `.bashrc` change.
