# HamBot_Deploy

Deployment scripts for setting up the **HamBot** robotics environment and the optional **OLED network display** on a Raspberry Pi 4 running **64-bit Raspberry Pi OS (Bookworm)**.

These scripts automate:
- Cloning the relevant repositories
- Creating Python virtual environments with system packages enabled
- Installing required Python dependencies (`pip install .`)
- Configuring global venv auto-activation (HamBot)
- Installing/removing systemd + NetworkManager hooks (OLED)

---

## Requirements

This project assumes you are running a **Raspberry Pi 4 with 64-bit Bookworm OS**.  
The following packages are already included in the default image and **do not need to be installed** separately:
- `git`
- `python3-venv`
- `python3-pip`
- `python3-picamera2`
- `python3-numpy`

You only need to install:

```bash
sudo apt update
sudo apt install -y \
    python3-opencv \
    network-manager \
    wireless-tools
````

---

## Installation

All scripts live in the `scripts/` folder.

### Install HamBot only

```bash
bash scripts/setup_hambot.sh
```

* Clones `https://github.com/biorobaw/HamBot` into `~/HamBot`
* Creates `hambot_venv` inside that repo
* Installs the `robot_systems` package
* Modifies `~/.bashrc` so **all new terminals auto-activate the venv**

> Notes:
>
> * `python` and `pip` use the venv by default
> * `python3` and `pip3` remain the system defaults

### Install OLED only

```bash
bash scripts/setup_oled.sh
```

* Clones `https://github.com/biorobaw/HamBot_OLED` into the `hambot` user’s home directory
* Creates a `.venv` in that repo and installs the OLED package
* Installs a NetworkManager dispatcher hook (`/etc/NetworkManager/dispatcher.d/99-hambot-oled`)
* Installs and enables a systemd oneshot service (`hambot_oled.service`) that runs at boot

### Combined installer

Use the dispatcher script to install both components at once:

```bash
bash scripts/install.sh both
```

Options:

* `bash scripts/install.sh hambot`
* `bash scripts/install.sh oled`
* `bash scripts/install.sh both` (default)

---

## Uninstall

To remove installed components and undo the setup:

```bash
bash scripts/uninstall.sh hambot   # removes HamBot repo, venv, and .bashrc snippet
bash scripts/uninstall.sh oled     # removes OLED repo, venv, service + dispatcher hook
bash scripts/uninstall.sh both     # removes everything
```

Notes:

* `uninstall.sh` is idempotent (safe to run multiple times).
* After uninstalling HamBot, open a new terminal to reload your `.bashrc` without the venv auto-activation snippet.
* After uninstalling OLED, the `hambot_oled.service` and dispatcher hook are gone, and NetworkManager is restarted.

---

## Development Notes

- `setup_hambot.sh` uses `--system-site-packages` for the venv so it can see system-wide libraries.  
- It is **critical** that `picamera2`, `numpy`, and `opencv` are installed via **apt** (system packages) and **not via pip**.  
  - Installing these with pip will cause the HamBot environment to fail because they depend on Raspberry Pi–specific builds and drivers provided only by the system packages.  
  - Correct install:
    ```bash
    sudo apt install -y python3-picamera2 python3-numpy python3-opencv
    ```
- Both setup scripts explicitly `cd` into the cloned repo before running `pip install .` so the correct `pyproject.toml` is picked up.  
- The installer is safe to re-run: existing repos will `git pull`, and existing venvs will be reused.  

---

## Quick sanity check

After installing HamBot:

```bash
# Open a new terminal
echo $VIRTUAL_ENV    # should show ~/HamBot/hambot_venv
which python         # should be ~/HamBot/hambot_venv/bin/python
which python3        # remains /usr/bin/python3

python -c "import robot_systems; print('HamBot OK')"
```

After installing OLED:

```bash
systemctl status hambot_oled.service --no-pager
ls /etc/NetworkManager/dispatcher.d/99-hambot-oled
```

---

## License

MIT
