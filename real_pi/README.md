# real_pi — native MiniBunker station (no Docker, no ROS)

The Raspberry Pi 5 real-robot path as a **single Python process**:

```
camera → detector (HSV) → perception_state + distance → behaviour FSM → CAN → Bunker Mini
```

This is the lightweight alternative to the ROS-in-Docker stack
(`../start_real.sh`). Full rationale, hardware bring-up, the CAN protocol and the
safety/test procedure are in **[../docs/REAL_PI_NATIVE.md](../docs/REAL_PI_NATIVE.md)** —
read that first.

> On Raspberry Pi OS, numpy/OpenCV/PyYAML/picamera2 come from **apt** and the venv
> is `--system-site-packages`; `requirements.txt` only pip-installs `python-can`
> + `flask`. Do NOT `pip install numpy` on the Pi (libopenblas shadow — see doc §3).

## Quick start (on the Pi)

```bash
python3 -m venv ~/mb-venv --system-site-packages && source ~/mb-venv/bin/activate
pip install -r requirements.txt

# tests (no robot needed):
python tests/test_fsm.py && python tests/test_detector.py && python tests/test_bunker_can.py

# perception only, real camera, no motion (annotated frames -> /tmp/mbframes):
python run.py --no-can --save-frames /tmp/mbframes
# ...or record a video instead:
python run.py --no-can --save-video /tmp/mb_run.mp4

# full station (boots DISARMED; type 'r'<Enter> to ARM). Needs can0 + the robot:
python run.py

# with the web control panel (live video + calibration + ARM + teleop in the browser):
python run.py --web                 # -> http://raspberrypi2.local:8080
```
Keys (terminal or panel): **R** = ARM · **F** = DISARM · **q** = quit · teleop
(mission `none`): **W/A/S/D** drive, **X** stop.

**One-command start (recommended):** `start_real_pi.sh` bounces CAN up (gs_usb),
verifies the Bunker is on the bus, then launches `run.py --web`. Install the alias
once, then just run `mb` (Bunker ON, **RC transmitter OFF**, e-stop in hand):
```bash
echo "alias mb='bash ~/minibunker-workshop/real_pi/start_real_pi.sh'" >> ~/.bashrc && source ~/.bashrc
mb                # safe ordered start + --web panel   (mb -f skips the CAN check)
```
Stop it (Ctrl-C) **before** power-cycling the Bunker, then `mb` again. (Standalone
CAN bring-up: `bash can_up.sh`.)

## Web control panel (`--web`)

A tiny Flask panel (the native replacement for the sim's Streamlit UI, which is
ROS-only), sharing the **same Controls** as the keyboard (one motion owner). Two tabs:

- **Drive** — live annotated camera, the **HSV mask** as a second feed, telemetry
  (state, distance, battery, `ctrl_mode`, ESTOP), **ARM/DISARM**, **mission** (none /
  ball / cone), hold-to-drive **WASD** teleop, and the mission/proximity banners.
- **Calibration** — pick `green_ball`/`cone`, drag the **H/S/V + min_area** sliders and
  watch the mask clean up live, then **distance**: hold the object at a known distance →
  *Calibrate from view* → metres. Copy the shown `config.yaml` snippet to keep it.

**Mission behaviour** (distance-driven; thresholds in `config.yaml`): **ball** →
*retrieved* + stop + mission→none at `ball_retrieve_m` (stays armed); **cone** →
*danger* + back-off + stop + mission→none at `cone_danger_m` (stays armed); **teleop**
→ operator warning within `proximity_warn_m`.

Two ways to run it (details in **[../docs/REAL_PI_NATIVE.md](../docs/REAL_PI_NATIVE.md) §8b**):

```bash
# A) on the Pi (recommended): loop + panel in one process
python run.py --web                          # browse http://raspberrypi2.local:8080

# B) page hosted on the laptop, API on the Pi:
python run.py --web                          # on the Pi
python panel/serve_laptop.py                 # on the laptop -> http://localhost:8090
#   then set the page's "API base" to http://raspberrypi2.local:8080
```
The physical e-stop always overrides; the panel shows an ESTOP banner.

A safe full-pipeline check **with the e-stop engaged** (the rover computes a
ball-follow but can't move) is in **[../docs/REAL_PI_NATIVE.md](../docs/REAL_PI_NATIVE.md) §6**
— it includes the real bring-up telemetry as a worked example.

## Viewing the debug frames / video (over SSH)

```bash
# (a) serve, view in your laptop browser:
cd /tmp/mbframes && python3 -m http.server 8000      # -> http://raspberrypi2.local:8000
# (b) copy to the laptop (run on the LAPTOP):
scp "pi@raspberrypi2.local:/tmp/mbframes/*.jpg" .
# (c) as a video: run with --save-video, or build one from frames:
ffmpeg -framerate 4 -pattern_type glob -i '/tmp/mbframes/f*.jpg' \
       -c:v libx264 -pix_fmt yuv420p /tmp/mb_run.mp4
```
See the doc **§8** for details.

## Safety

Boots **DISARMED** · hard caps in `config.yaml → behavior/limits`, re-clamped to
the Bunker HW ceiling (1.5 m/s / 0.785 rad/s) · watchdog zeroes on stall ·
**Ctrl-C → zero motion + STANDBY**. Keep the e-stop in hand; fenced arena only.
**Real autonomous motion validated** on the Bunker (drove under CAN, `ctrl_mode=1`,
`actual_v` tracking the follow command). **The RC transmitter must be OFF** — RC
overrides CAN and the base will fault to EXCEPTION (doc **§5.4**).

## Files

| File | What |
| --- | --- |
| `run.py` | the loop: Controls hub (stdin + web), ARM gate, watchdog, e-stop, viz/record, mission outcomes |
| `config.yaml` | all knobs — HSV, `detector/distance`, mission thresholds (`ball_retrieve_m` …), `behavior/`, `can:` |
| `start_real_pi.sh` · `can_up.sh` | one-command start (`mb`) · gs_usb CAN bring-up |
| `minibunker_real/bunker_can.py` | AgileX protocol-v2 CAN driver (replaces `bunker_base`) |
| `minibunker_real/webpanel.py` · `panel/` | Flask `--web` panel + page (Pi/laptop served) |
| `minibunker_real/{detector,perception_state,fsm}.py` | HSV detector / packing+annotation / FSM with mission completion — lifted from the sim nodes |
| `minibunker_real/distance.py` | one-point pixel distance estimator |
| `minibunker_real/camera.py` | picamera2 / V4L2 / video / synthetic source |
| `tests/` | FSM, detector, distance, CAN (incl. vcan0 loopback) |
