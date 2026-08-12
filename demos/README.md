# HamBot_Demos

Demo scripts for the [HamBot](https://github.com/CJHRobotics/HamBot) robot platform. Each demo is a standalone script under `demos/`.

## Setup (on the Pi)

```bash
python3 -m venv --system-site-packages hambot_env
source hambot_env/bin/activate
pip install -r requirements.txt
```

`--system-site-packages` is needed so the venv can see `picamera2` from Raspberry Pi OS.

## Demos

| Script | What it does | Hardware used |
| --- | --- | --- |
| `demos/color_shuttle.py` | Drives between a blue and a pink cylinder, using the camera to find them and lidar to stop. | 2WD drive, Pi Camera, RPLidar |

Run one:

```bash
python demos/color_shuttle.py
```

Ctrl-C stops the robot cleanly (HamBot installs a SIGINT handler).

### `color_shuttle.py` tuning

Open the file and adjust the constants near the top:

- `BLUE_RGB` / `PINK_RGB` — measure your actual cylinders under the real lighting; blob detection is a raw RGB threshold.
- `COLOR_TOLERANCE` — widen if the robot never spots a landmark, narrow if it locks onto the wrong thing.
- `STOP_MM` — how close (front lidar) before it stops.
- `CRUISE_RPM`, `SEARCH_RPM`, `STEER_KP` — motion feel.
