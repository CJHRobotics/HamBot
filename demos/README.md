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

- `BLUE_HSV` / `PINK_HSV` — target colors in **OpenCV HSV** (H: 0-179, S: 0-255, V: 0-255). Sample them with `cameraGUI.py` under the real lighting; only H matters for matching.
- `HUE_TOLERANCE` — ± hue units around each target. Widen if landmarks aren't detected, narrow if the robot locks onto the wrong color.
- `MIN_SATURATION` / `MIN_VALUE` — global floors that reject washed-out and dark pixels. Lower them if the color reads dim; raise them if the mask picks up walls and shadows.
- `STOP_MM` — how close (front lidar) before the robot stops.
- `CRUISE_RPM`, `SEARCH_RPM`, `STEER_KP` — motion feel.

## `cameraGUI.py`

Live HSV tuner. Click a pixel to sample its HSV into the sliders; adjust hue tolerance + S/V floors to see what gets accepted. The values plug directly into `Camera.set_target_colors(...)`.

The tuner shows one of two modes based on the sampled S value:

- **`[chromatic]`** — normal color (blue, pink, red, green…). Match band is hue ± `hue_tolerance` with S/V floors filtering out washed-out and dark pixels.
- **`[achromatic]`** — black, white, or gray. Hue is meaningless for these because near-gray pixels have unstable/undefined hue, so the matcher switches to `value ± value_tolerance` with S capped at the achromatic threshold. Click a black pixel and lower the value slider will follow — tune `value_tolerance` to widen the accepted brightness band.

Pass `value_tolerance=...` to `Camera.set_target_colors` alongside `hue_tolerance=...` when targeting black/white/gray.
