# HamBot Demos

Runnable scripts that exercise the `robot_systems` package. This directory is symlinked to `~/Desktop/Demos` by `deploy/install.sh`, so `git pull` on the monorepo refreshes the shortcut automatically.

## Prerequisites

Install the monorepo first (see the [top-level README](../README.md)):

```bash
./deploy/install.sh hambot
```

That creates `hambot_venv` with `robot_systems` installed and adds a `.bashrc` snippet so new terminals auto-activate it.

## Demos

| Script | What it does | Hardware used |
| --- | --- | --- |
| `systems_check.py` | Prints IMU / lidar / motor status so you can verify hardware is wired correctly. | IMU, RPLidar, motors |
| `cameraGUI.py` | GUI viewer for the Pi camera stream. | Pi Camera |
| `remote_control.py` | Drive the robot with the keyboard over SSH (arrow keys + `+`/`-` for speed). | 2WD drive |
| `color_shuttle.py` | Drives between a blue and a pink cylinder using camera blob detection and lidar to stop. | 2WD drive, Pi Camera, RPLidar |

Run one:

```bash
python demos/remote_control.py
```

Ctrl-C stops the robot cleanly (`robot_systems` installs a SIGINT handler).

## `remote_control.py`

Works over SSH — no graphical session or extra client required. Just `ssh pi@hambot` and run it.

- Arrow keys drive; hold two at once to arc.
- `space` — stop.
- `+` / `-` — bump cruise speed (10–75 rpm).
- `q` — quit.

Tune `CRUISE_RPM_START`, `TURN_RATIO`, and the min/max speed constants at the top of the file to match your robot.

## `color_shuttle.py` tuning

Edit the constants near the top of the file:

- `BLUE_RGB` / `PINK_RGB` — measure your actual cylinders under the real lighting; blob detection is a raw RGB threshold.
- `COLOR_TOLERANCE` — widen if the robot never spots a landmark, narrow if it locks onto the wrong thing.
- `STOP_MM` — how close (front lidar) before the robot stops.
- `CRUISE_RPM`, `SEARCH_RPM`, `STEER_KP` — motion feel.
