# AgileX MiniBunker 2.0 Workshop

An AgileX Bunker Mini 2.0 tracked rover that uses a
Raspberry Pi 5 with a Pi Camera and colour (HSV) vision to recognise a green ball ("sample" to "collect") and construction cones ("hazards" to avoid), estimates distance from pixels, and drives a reactive behaviour. The real robot runs a native, no-ROS Python stack with a browser control panel; the same detection and behaviour logic also runs in Gazebo simulation. Everything is YAML-configurable.

> Status: the real robot is driving on the native Pi path. The Bunker Mini finds and follows the green ball under HSV vision, reads distance from bounding-box size, completes the mission (ball → retrieved, cone → danger and back-off), and disarms, all from a Flask control panel on the Pi, no ROS and no Docker (camera → HSV → distance → FSM → CAN). See [real_pi/](real_pi/) and [docs/REAL_PI_NATIVE.md](docs/REAL_PI_NATIVE.md). The Gazebo sim and Streamlit UI remain for the shared logic. (A CNN approach backend is scaffolded but not implemented yet.)

## Gallery

Real MiniBunker, the native control panel, running on the real Bunker Mini with no
ROS:

[![Real MiniBunker native panel](assets/mb_real_ui.png)](assets/mb_real_ui.mp4)

Watch the video: [assets/mb_real_ui.mp4](assets/mb_real_ui.mp4). It shows the live
camera and HSV mask, distance calibration, and mission plus WASD control on the
real rover (see [docs/REAL_PI_NATIVE.md](docs/REAL_PI_NATIVE.md)).

| Gazebo simulation | Streamlit control panel |
| --- | --- |
| ![Gazebo arena](assets/mb_sim_gz.png) | ![Control panel](assets/mb_sim_ui.png) |
| [Watch video](assets/minibunker_sim_gz.mp4): the rover, cones, and the ore ball in the Gazebo arena | [Watch video](assets/minibunker_sim_ui.mp4): live camera, telemetry, ARM/DISARM, mission and WASD knobs |

## What it is

- Same detection and behaviour, sim and real. The HSV detector and reactive FSM
  run in the Gazebo sim and, lifted ROS-free, on the real Pi
  ([real_pi/](real_pi/)). Only the camera source and the drive sink swap.
- Colour (HSV) vision. Hand-tuned hue/saturation/value thresholds turn pixels into
  the green ball and orange cone, calibrated live in the browser through a mask
  view and sliders. A YOLOv8-nano CNN backend is scaffolded for the future but not
  used in the workshop.
- Distance from pixels. A one-point calibration turns bounding-box size into
  metres, so the rover acts on real distances without a depth sensor.
- Reactive state machine: SEARCH, then APPROACH, then AVOID, followed by
  per-mission completion. A ball ends in retrieved and stop; a cone ends in danger,
  back-off, and stop. The mission then resets to none and stays armed. It boots
  disarmed.
- Browser control panel (real). A native Flask panel gives live camera and HSV
  mask views, a calibration tab, mission control, WASD teleop, and ARM/DISARM. The
  sim uses a Streamlit panel over rosbridge instead
  ([app.py](catkin_ws/src/minibunker_ui/app.py)).
- One config file drives each side: [real_pi/config.yaml](real_pi/config.yaml) for
  the real robot,
  [minibunker.yaml](catkin_ws/src/minibunker_bringup/config/minibunker.yaml) for
  the sim.

## Runtime environment: native on the Pi, no Docker, no ROS

The real station runs as a single native Python process on the Raspberry Pi 5
([real_pi/](real_pi/)), not in Docker and not under ROS.

Why: ROS Noetic isn't native on Raspberry Pi OS (Bookworm), so the ROS path means a
long arm64 Docker image build (ROS plus the C++ CAN driver, and it drags in the
Gazebo packages the Pi doesn't need). The real station only needs camera, detector,
behaviour, and CAN, and none of that needs ROS at runtime. The hard logic (HSV
detection, the behaviour FSM) is already ROS-free in the sim nodes, so it ports
almost verbatim.

The trade-off we accepted: the native stack loses sim-to-real parity, since it
can't run in Gazebo, and it drops the ROS/Streamlit UI in favour of a native Flask
panel. The Gazebo sim (ROS, on an x86 laptop) stays for the shared
detection/behaviour logic and as a hardware-free playground. A Docker/ROS real path
also still exists (`start_real.sh`,
[docs/HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md)) as an alternative. Full rationale
is in [docs/REAL_PI_NATIVE.md](docs/REAL_PI_NATIVE.md) §0.

## Quick start

**Real robot (the workshop path)** — on the Raspberry Pi, with the Bunker on and
the RC transmitter off (it overrides CAN), e-stop in hand:

```bash
cd ~/minibunker-workshop/real_pi
mb                       # bring CAN up (gs_usb) + launch the control panel
# then open http://raspberrypi2.local:8080 in a laptop browser
```

Panel keys: R arms, F disarms, q quits. Teleop (mission `none`) uses W/A/S/D to
drive and X to stop. Full setup, calibration, and safety notes are in
[docs/REAL_PI_NATIVE.md](docs/REAL_PI_NATIVE.md).

**Gazebo sim** — on Windows, run from PowerShell (the `.sh` scripts are invoked
through Git's bundled bash by full path; from a Git Bash prompt drop the prefix and
use `bash ./start_sim.sh`):

```powershell
git clone --recurse-submodules https://github.com/billisandr/minibunker-workshop
cd minibunker-workshop
& "C:\Program Files\Git\bin\bash.exe" ./start_sim.sh                              # Gazebo sim (HSV baseline)
& "C:\Program Files\Git\bin\bash.exe" ./catkin_ws/src/minibunker_ui/run_ui.sh     # http://localhost:8501
```

The full sim walkthrough, including a hardware-free synthetic loop, is in
[docs/QUICKSTART.md](docs/QUICKSTART.md).

## Repo layout

```
real_pi/                     native (no-Docker/ROS) Pi station — THE WORKSHOP PATH
├── run.py                   ONE loop: camera→detector→distance→FSM→CAN; ARM gate, watchdog, e-stop, --web panel
├── config.yaml              the one file you tune (HSV, distances, mission thresholds, CAN)
├── minibunker_real/         detector.py · perception_state.py · distance.py · fsm.py · camera.py · bunker_can.py · webpanel.py
├── panel/                   the Flask panel page (Drive + Calibration tabs)
├── can_up.sh start_real_pi.sh   gs_usb CAN bring-up · one-command `mb` start
└── tests/                   fsm, detector, distance, CAN (incl. a vcan0 loopback)
catkin_ws/src/               ROS / Gazebo sim (x86 laptop)
├── minibunker_perception/   detector_node (HSV/CNN), pi_camera_node, image_pub_node
├── minibunker_behavior/     behavior_node (state machine)
├── minibunker_bringup/      launch + minibunker world + camera xacro + master YAML
├── minibunker_ui/           Streamlit app + rosbridge launch
├── ugv_gazebo_sim/          (submodule) Bunker / Bunker-Mini Gazebo description
├── bunker_ros/              (submodule) real driver: bunker_base/bringup/msgs
└── ugv_sdk/                 (submodule) C++ CAN layer (protocol decoded into real_pi/bunker_can.py)
docker/   start_sim.sh   start_real.sh   training/   docs/
```

## Built on

- [`agilexrobotics/ugv_gazebo_sim`](https://github.com/agilexrobotics/ugv_gazebo_sim) — Gazebo models (Bunker and Bunker-Mini)
- [`agilexrobotics/bunker_ros`](https://github.com/agilexrobotics/bunker_ros) — ROS1 driver for the real Bunker Mini
- [`agilexrobotics/ugv_sdk`](https://github.com/agilexrobotics/ugv_sdk) — C++ CAN layer

## Docs

- [docs/REAL_PI_NATIVE.md](docs/REAL_PI_NATIVE.md) — the real workshop path: native
  (no-Docker, no-ROS) Pi stack, covering setup, the CAN protocol, HSV and distance
  calibration, the control panel, mission behaviour, and a full test walkthrough
  with real-Bunker results
- [real_pi/README.md](real_pi/README.md) — `real_pi/` quick start and file map
- [docs/QUICKSTART.md](docs/QUICKSTART.md) — running the sim, the hardware-free loop, and the Streamlit UI
- [docs/HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md) — Pi 5, camera, and CAN setup (Docker/ROS path)
- [docs/ARENA.md](docs/ARENA.md) — the physical arena and object spec
- [docs/TRAINING.md](docs/TRAINING.md) — CNN training (Roboflow to YOLOv8n), deferred and out of current scope

## Safety

The rover boots disarmed, so no motion frame is sent until you arm it. Hard speed
caps in `behavior/limits` are clamped again to the Bunker's hardware ceiling (1.5
m/s, 0.785 rad/s), a watchdog zeroes motion on stale input, and Ctrl-C or exit zeros
motion and returns to standby. Calibrating the mask and distance is risk-free,
since it's perception only and stays disarmed.

On the real robot the drive stage is instructor-led: the RC transmitter must be
off, since it outranks the computer over CAN, keep the hardware e-stop in hand, run
only inside the fenced arena, and start with low caps. The software caps are a
backstop, not a substitute for the e-stop. The full ritual is in
[docs/REAL_PI_NATIVE.md](docs/REAL_PI_NATIVE.md) (real path) and
[docs/ARENA.md](docs/ARENA.md) (arena).

## Troubleshooting

| Problem | Fix |
| ------- | --- |
| Panel won't load | Is `run.py --web` (or `mb`) running? Open `http://raspberrypi2.local:8080`. Or serve the page from the laptop (`real_pi/panel/serve_laptop.py`) and set its API base to the Pi |
| Rover won't move | Not armed (press R or the ARM button); or mission is `none` (that's teleop, hold W/A/S/D); or the RC is on (`ctrl_mode=3`); or the e-stop is engaged |
| Panel shows "base not in CAN mode" or an exception | The RC transmitter is on, turn it off (RC overrides CAN). Power-cycle the Bunker to clear an exception latch — [docs/REAL_PI_NATIVE.md](docs/REAL_PI_NATIVE.md) §5.4 |
| "CAN TX error: Network is down" | The gs_usb bus went down — `bash real_pi/can_up.sh` bounces `can0` (gs_usb does not support `restart-ms`) |
| No or flaky detection boxes | The HSV mask is wrong for today's light. In the Calibration tab, raise `S low` (shadows are low-saturation), then adjust `min_area` for speckle. The mask is your ground truth |
| Colours look wrong (red/blue swapped) | Set `camera/picam_swap_rb` (picamera2's "RGB888" is BGR-ordered). Check the sensor with `rpicam-hello --list-cameras` |
| A/D don't turn | Fixed: A/D now turn, and ARM/DISARM moved to R/F for classic WASD |
| numpy `libopenblas.so.0` ImportError | You pip-installed numpy on the Pi. Run `pip uninstall -y numpy opencv-python` and use apt's build instead (the venv is `--system-site-packages`) |
| Real-robot drive step | Needs the physical Bunker, a fenced arena, RC off, and the e-stop in hand — see [docs/REAL_PI_NATIVE.md](docs/REAL_PI_NATIVE.md) §6–7 |

## Status

**Done**
- Sim validated: detector, behaviour, Gazebo, and the Streamlit UI all work together.
- Real robot on the Pi ([docs/REAL_PI_NATIVE.md](docs/REAL_PI_NATIVE.md)): the
  native no-ROS stack drives under CAN, with HSV calibration and a live mask,
  pixel distance, mission completion (ball retrieved or cone danger, then stop and
  mission reset to none), a teleop proximity warning, and the Flask control panel
  (Drive and Calibration tabs, WASD).
- A participant handout and instructor cards exist for this station (v2), kept
  in a separate internal document set alongside our other workshop stations.

**Remaining / optional**
- Tune HSV and distance under the actual arena lighting (already doable live in
  the panel).
- Operating procedure: keep the RC transmitter off, since it overrides CAN and
  triggers an exception, and bring up the gs_usb CAN interface each session
  (`real_pi/can_up.sh`). Run with the e-stop in hand.
- Optional: a systemd autostart unit for `run.py`, and Git LFS for the demo videos.
- Future, out of current scope: train the CNN backend
  ([docs/TRAINING.md](docs/TRAINING.md)) for lighting-robust detection.

---
