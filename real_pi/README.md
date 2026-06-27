# real_pi — native MiniBunker station (no Docker, no ROS)

The Raspberry Pi 5 real-robot path as a **single Python process**:

```
camera → detector (HSV/CNN) → perception_state → behaviour FSM → CAN → Bunker Mini
```

This is the lightweight alternative to the ROS-in-Docker stack
(`../start_real.sh`). Full rationale, hardware bring-up, the CAN protocol and the
safety/test procedure are in **[../docs/REAL_PI_NATIVE.md](../docs/REAL_PI_NATIVE.md)** —
read that first.

> On Raspberry Pi OS, numpy/OpenCV/PyYAML/picamera2 come from **apt** and the venv
> is `--system-site-packages`; `requirements.txt` only pip-installs `python-can`.
> Do NOT `pip install numpy` on the Pi (libopenblas shadow — see the doc §3).

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

# full station (boots DISARMED; type 'a'<Enter> to ARM). Needs can0 + the robot:
python run.py
```

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
Validated on the real Bunker with the e-stop **engaged** (full follow pipeline,
zero motion); first real drive procedure is in the doc **§7**.

## Files

| File | What |
| --- | --- |
| `run.py` | the loop: ARM gate, watchdog, e-stop, in-process teleop |
| `config.yaml` | all knobs (ROS-free subset of `minibunker.yaml` + `can:`) |
| `minibunker_real/bunker_can.py` | AgileX protocol-v2 CAN driver (NEW) |
| `minibunker_real/{detector,perception_state,fsm}.py` | lifted from the sim ROS nodes |
| `minibunker_real/camera.py` | picamera2 / V4L2 / video / synthetic source |
| `tests/` | FSM, detector, CAN (incl. vcan0 loopback) |
