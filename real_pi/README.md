# real_pi — native MiniBunker station (no Docker, no ROS)

The Raspberry Pi 5 real-robot path as a **single Python process**:

```
camera → detector (HSV/CNN) → perception_state → behaviour FSM → CAN → Bunker Mini
```

This is the lightweight alternative to the ROS-in-Docker stack
(`../start_real.sh`). Full rationale, hardware bring-up, the CAN protocol and the
safety/test procedure are in **[../docs/REAL_PI_NATIVE.md](../docs/REAL_PI_NATIVE.md)** —
read that first.

## Quick start (on the Pi)

```bash
python3 -m venv ~/mb-venv --system-site-packages && source ~/mb-venv/bin/activate
pip install -r requirements.txt

# tests (no robot needed):
python tests/test_fsm.py && python tests/test_detector.py && python tests/test_bunker_can.py

# perception only, real camera, no motion:
python run.py --no-can --save-frames /tmp/mbframes

# full station (boots DISARMED; type 'a'<Enter> to ARM). Needs can0 + the robot:
python run.py
```

## Safety

Boots **DISARMED** · hard caps in `config.yaml → behavior/limits`, re-clamped to
the Bunker HW ceiling (1.5 m/s / 0.785 rad/s) · watchdog zeroes on stall ·
**Ctrl-C → zero motion + STANDBY**. Keep the e-stop in hand; fenced arena only.

## Files

| File | What |
| --- | --- |
| `run.py` | the loop: ARM gate, watchdog, e-stop, in-process teleop |
| `config.yaml` | all knobs (ROS-free subset of `minibunker.yaml` + `can:`) |
| `minibunker_real/bunker_can.py` | AgileX protocol-v2 CAN driver (NEW) |
| `minibunker_real/{detector,perception_state,fsm}.py` | lifted from the sim ROS nodes |
| `minibunker_real/camera.py` | picamera2 / V4L2 / video / synthetic source |
| `tests/` | FSM, detector, CAN (incl. vcan0 loopback) |
