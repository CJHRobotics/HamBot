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
./deploy/install.sh [--force] [--link] [target]
```

`--force` deletes and recreates `hambot_venv` before installing. Use it if the venv is corrupted or you want a clean rebuild. Only valid with `hambot`, `both`, or `all` (the targets that create the venv).

`all` installs the Vivid console control listener and camera stream, so a full install leaves the robot ready to drive. The narrower targets (`hambot`, `oled`, `depthai`, `both`) leave it out; add `--link` to include it alongside one of those.

Installing the listener does not by itself expose the robot: `hambot-link` only runs when someone connects with a key already present in `authorized_keys`. That file, not the presence of the binary, is the access boundary.

To add the listener to a robot that is **already** set up, do not use `--link` on its own — with no target that still means `all`, a full reinstall. Use the `link` target, which touches nothing else and installs no apt packages:

```bash
git pull && ./deploy/install.sh link
```

Targets:

| Target | What it does |
| --- | --- |
| `hambot` | Creates `hambot_venv`, installs `packages/robot_systems`, adds a `.bashrc` auto-activation snippet, and symlinks `demos/` to `~/Desktop/Demos` (so `git pull` refreshes the shortcut). |
| `oled` | Installs `packages/hambot_oled` into the shared venv, writes `/etc/systemd/system/hambot_oled.service` and a NetworkManager dispatcher hook. Requires `hambot` first. |
| `depthai` | Adds the OAK-camera udev rule, `pip install`s `depthai`/`opencv-python`/`simplejpeg` into the shared venv, and clones `depthai-python` next to the monorepo. Requires `hambot` first. |
| `link` | Installs `packages/hambot_link` into the shared venv and symlinks `hambot-link` and `hambot-camera` into `/usr/local/bin` so the Vivid console can launch them over SSH. No systemd unit — the console starts one process per driving or viewing session, so the motors and camera stay free for demos otherwise. Requires `hambot` first. |
| `both` | `hambot` + `oled`. |
| `all` (default) | `hambot` + `oled` + `depthai` + `link`. A full install leaves the robot ready for the Vivid console. |

## Individual scripts

Each step can also be run standalone: `setup_hambot.sh`, `setup_oled.sh`, `setup_depthai.sh`, `setup_link.sh`. They discover the monorepo location from their own path, so they work wherever you cloned it.

## Uninstall

```bash
./deploy/uninstall.sh [target]
```

Removes the venv, systemd unit, NetworkManager hook, udev rule, the `hambot-link`/`hambot-camera` symlinks, and the `.bashrc` snippet. It does **not** delete the monorepo directory.

Open a new terminal afterwards to pick up the `.bashrc` change.
