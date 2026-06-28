# AgileX MiniBunker 2.0 Workshop

A Space Summer School station: an AgileX **Bunker Mini 2.0** tracked rover that uses a
**Raspberry Pi 5 + Pi Camera** and **colour (HSV) vision** to recognise a **green ball**
(the "ore sample" to collect) and **construction cones** (hazards), estimates **distance
from pixels**, and drives a reactive *space-mining* behaviour. The real robot runs a
**native, no-ROS Python stack** with a **browser control panel**; the same detection +
behaviour logic also runs in **Gazebo simulation**. Fully **YAML-configurable**.

It's a sibling of the SenseLAB live-knobs stations (the `ros_z1_*` ZED sims, the
[`g1-sim2sim-workshop`](../g1-sim2sim-workshop/)): the same "turn a knob, watch the robot
react" teaching shape, here applied to camera-driven autonomy on a real rover.

> **Status: real robot driving on the native Pi path.**
> The Bunker Mini finds and follows the green ball under **HSV** vision, reads
> **distance** from bounding-box size, completes the mission (ball → *retrieved*,
> cone → *danger* + back-off) and disarms — all from a **Flask control panel** on the
> Pi, no ROS / no Docker (camera → HSV → distance → FSM → CAN). The Gazebo sim +
> Streamlit UI remain for the shared logic. A learned-vision (CNN) backend is scaffolded
> but **out of scope** for the current workshop. Design rationale: **[plan.md](plan.md)**.

## Gallery

**Real MiniBunker — the native control panel** (running on the real Bunker Mini, no ROS):

[![Real MiniBunker native panel](assets/mb_real_ui.png)](assets/mb_real_ui.mp4)

▶ [Watch video](assets/mb_real_ui.mp4) · live camera + HSV mask, distance calibration, mission + WASD on the real rover (see [docs/REAL_PI_NATIVE.md](docs/REAL_PI_NATIVE.md))

| Gazebo simulation | Streamlit control panel |
| --- | --- |
| ![Gazebo arena](assets/mb_sim_gz.png) | ![Control panel](assets/mb_sim_ui.png) |
| ▶ [Watch video](assets/minibunker_sim_gz.mp4) · rover, cones and the ore ball in the Gazebo arena | ▶ [Watch video](assets/minibunker_sim_ui.mp4) · live camera, telemetry, ARM/DISARM, mission + WASD knobs |

---

## Runtime environment: native on the Pi (no Docker, no ROS)

The real station runs as a **single native Python process** on the Raspberry Pi 5
([real_pi/](real_pi/)) — not in Docker and not under ROS.

Why: ROS Noetic isn't native on Raspberry Pi OS (Bookworm), so the ROS path means a long
arm64 Docker image build (ROS + the C++ CAN driver, and it drags in the Gazebo packages
the Pi doesn't need). The real station only needs **camera → detector → behaviour → CAN**;
none of that needs ROS at runtime, and the hard logic (HSV detection, the behaviour FSM)
is already ROS-free in the sim nodes, so it ports almost verbatim.

**Trade-off accepted:** the native stack loses **sim↔real parity** (it can't run in
Gazebo) and the ROS/Streamlit UI — replaced by a native **Flask** panel. The **Gazebo
sim** (ROS, on an x86 laptop) stays for the shared detection/behaviour logic and as a
hardware-free playground. A Docker/ROS *real* path also still exists (`start_real.sh`,
[docs/HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md)) as an alternative. Full rationale:
[docs/REAL_PI_NATIVE.md](docs/REAL_PI_NATIVE.md) §0.

---

## What participants do

1. **Calibrate the eyes** — on the panel's **Calibration** tab, tune the HSV colour mask
   (live mask + sliders) until the green ball / orange cone is a clean blob and shadows
   drop out; then one-point-calibrate **distance** (hold the object at a known distance).
2. **Give it a mission** — pick `ball` or `cone`, **ARM**, and watch the reactive state
   machine hunt: `SEARCH → APPROACH → AVOID`. It steers on the object's position in the
   image (a P-controller) and judges range from the calibrated pixel distance.
3. **Drive it** — set mission `none` and take the wheel with **WASD** teleop.

**Per-mission completion** (every distance is in [real_pi/config.yaml](real_pi/config.yaml)):
- **ball** → at `ball_retrieve_m` it logs **"ball retrieved"**, shows a success banner,
  switches to mission `none` and **DISARMs**.
- **cone** → at `cone_danger_m` it shows a **danger** banner, backs off one step, then
  mission `none` + **DISARM**.
- **none (teleop)** → any object within `proximity_warn_m` raises an operator warning.

Two ideas the station leans on:
- **HSV is a rule you write, not a model you train** — hue/saturation/value thresholds
  turn pixels into a blob. A shadow lowers saturation+value but keeps hue, so it reads as
  "dark green" until you raise `S low` — participants meet classic-CV's brittleness by
  hand. (A YOLOv8-nano **CNN** backend is scaffolded for the future but is **out of the
  current workshop's scope**.)
- **Distance is a pixel proxy** — apparent size = range, true only because the object's
  real size is fixed; no depth sensor.

---

## Setup

**Real robot** (Raspberry Pi OS Bookworm 64-bit). numpy / OpenCV / PyYAML / picamera2 come
from **apt**; the venv is `--system-site-packages`; pip adds only `python-can` + `flask`:

```bash
sudo apt install -y python3-venv python3-numpy python3-opencv python3-yaml python3-picamera2 can-utils
git clone --recurse-submodules https://github.com/billisandr/minibunker-workshop.git
cd minibunker-workshop && git checkout real-pi-native
python3 -m venv ~/mb-venv --system-site-packages && source ~/mb-venv/bin/activate
pip install -r real_pi/requirements.txt
```
The repo is private — git over HTTPS needs a **Personal Access Token** (not your account
password). Do **not** `pip install numpy` on the Pi (use apt's). Full setup + safety →
[docs/REAL_PI_NATIVE.md](docs/REAL_PI_NATIVE.md).

**Gazebo sim** (x86 laptop, Windows). Run the `.sh` scripts through Git's bundled bash by
full path (from a Git Bash prompt drop the prefix and use `bash ./start_sim.sh`):

```powershell
git clone --recurse-submodules https://github.com/billisandr/minibunker-workshop
cd minibunker-workshop
```

## Running it

**Real robot** — Bunker on, **RC transmitter OFF** (it overrides CAN), e-stop in hand:

```bash
cd ~/minibunker-workshop/real_pi
mb                    # bring CAN up (gs_usb) + launch the control panel
# then open http://raspberrypi2.local:8080 in a laptop browser
```
Panel keys: **R** = ARM · **F** = DISARM · **q** = quit · teleop (mission `none`):
**W/A/S/D** drive, **X** stop. (`mb` is `real_pi/start_real_pi.sh`; bus-only bring-up is
`bash real_pi/can_up.sh`.)

**Gazebo sim** — from PowerShell:

```powershell
& "C:\Program Files\Git\bin\bash.exe" ./start_sim.sh                              # Gazebo sim (HSV baseline)
& "C:\Program Files\Git\bin\bash.exe" ./catkin_ws/src/minibunker_ui/run_ui.sh     # http://localhost:8501
```
Full sim walkthrough (incl. a hardware-free synthetic loop) → [docs/QUICKSTART.md](docs/QUICKSTART.md).

---

## Architecture

The same **HSV detector** + **reactive FSM** run in the sim (ROS nodes) and on the real
Pi (lifted ROS-free into `real_pi/`); only the **camera source** and the **drive sink**
swap. One config file is the only thing a host edits.

```
real_pi/                          native, no-ROS Pi station — THE WORKSHOP PATH
├── run.py                        ONE loop: camera→detector→distance→FSM→CAN; ARM gate, watchdog, e-stop, --web panel
├── config.yaml                   the one file you tune (HSV, distances, mission thresholds, CAN)
├── minibunker_real/
│   ├── detector.py               HSV (+ optional CNN) backends — lifted from the sim node
│   ├── perception_state.py       role-based target/obstacle/hazard packing + annotation
│   ├── distance.py               one-point pixel distance estimator
│   ├── fsm.py                    SEARCH→APPROACH→AVOID + per-mission completion (retrieve / danger)
│   ├── camera.py                 picamera2 / V4L2 / video / synthetic source
│   ├── bunker_can.py             AgileX protocol-v2 CAN driver (replaces the C++ bunker_base)
│   └── webpanel.py               Flask control panel (Drive + Calibration tabs)
├── panel/index.html              the panel page (Pi- or laptop-served)
├── can_up.sh · start_real_pi.sh  gs_usb CAN bring-up · one-command `mb` start
└── tests/                        fsm, detector, distance, CAN (incl. a vcan0 loopback)

catkin_ws/src/                    ROS / Gazebo sim (x86 laptop)
├── minibunker_perception/        detector_node (HSV/CNN), pi_camera_node, image_pub_node
├── minibunker_behavior/          behavior_node (state machine)
├── minibunker_bringup/           launch + minibunker world + camera xacro + master YAML
├── minibunker_ui/                Streamlit app + rosbridge launch
├── ugv_gazebo_sim/ · bunker_ros/ · ugv_sdk/   (submodules) AgileX description · ROS driver · C++ CAN
docker/   start_sim.sh   start_real.sh   training/   docs/
```

On the real robot, **`bunker_can.py`** speaks the AgileX **protocol v2** straight over
socketcan (motion `0x111`, control-mode `0x421`, state `0x211`/`0x221`) — the same frames
the C++ SDK's `SetMotionCommand`/`EnableCommandedMode` emit, so no ROS or `bunker_base` is
needed. Detection is **HSV-only** in the workshop; the **distance estimator** turns
bounding-box height into metres; the **FSM is the single owner of motion** — teleop and
autonomy both flow through one **ARM gate + speed clamp + watchdog**, so DISARM (and the
hardware e-stop) is always authoritative. The CAN protocol table + full walkthrough are in
[docs/REAL_PI_NATIVE.md](docs/REAL_PI_NATIVE.md).

## Built on

- [`agilexrobotics/ugv_gazebo_sim`](https://github.com/agilexrobotics/ugv_gazebo_sim) — Gazebo models (Bunker + Bunker-Mini)
- [`agilexrobotics/bunker_ros`](https://github.com/agilexrobotics/bunker_ros) — ROS1 driver for the real Bunker Mini
- [`agilexrobotics/ugv_sdk`](https://github.com/agilexrobotics/ugv_sdk) — C++ CAN layer (protocol decoded into `real_pi/bunker_can.py`)

---

## Documentation

- [docs/REAL_PI_NATIVE.md](docs/REAL_PI_NATIVE.md) — **the real workshop path**: native
  (no-Docker, no-ROS) Pi stack — setup, CAN protocol, HSV + distance calibration, the
  control panel, mission behaviour, full test walkthrough + real-Bunker results
- [real_pi/README.md](real_pi/README.md) — `real_pi/` quick start + file map
- [ExerciseHandouts/MiniB2_rover/](../ExerciseHandouts/MiniB2_rover/) — participant **handout** + **instructor cards** (v2; HTML + PDF)
- [docs/QUICKSTART.md](docs/QUICKSTART.md) — run the **sim** / hardware-free / Streamlit UI
- [docs/HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md) — Pi 5 + camera + CAN (Docker/ROS path)
- [docs/ARENA.md](docs/ARENA.md) — physical arena + object spec
- [docs/TRAINING.md](docs/TRAINING.md) — CNN training (Roboflow → YOLOv8n) — **deferred / out of current scope**
- [plan.md](plan.md) — full design + phased roadmap + open decisions (§15)

---

## Safety

Boots **DISARMED** — no motion frame is sent until you ARM. Hard speed caps in
`behavior/limits` are clamped again to the Bunker HW ceiling (1.5 m/s / 0.785 rad/s); a
**watchdog** zeroes motion on stale input; **Ctrl-C / exit → zero + STANDBY**. Calibrating
the mask + distance is **risk-free** (perception only, DISARMED).

On the **real robot** the drive stage is instructor-led: the **RC transmitter must be OFF**
(it out-ranks the computer over CAN), keep the **hardware e-stop in hand**, run only inside
the **fenced arena**, and start with low caps — the software caps are a backstop, **not** a
substitute for the e-stop. Full ritual: [docs/REAL_PI_NATIVE.md](docs/REAL_PI_NATIVE.md)
(real path) + [docs/ARENA.md](docs/ARENA.md) (arena).

---

## Troubleshooting

| Problem | Fix |
| ------- | --- |
| Panel won't load | Is `run.py --web` (or `mb`) running? Open `http://raspberrypi2.local:8080`. Or serve the page from the laptop (`real_pi/panel/serve_laptop.py`) and set its **API base** to the Pi |
| Rover won't move | Not **ARMed** (press **R** / the ARM button); or mission `none` (that's teleop — hold **W/A/S/D**); or the **RC is on** (`ctrl_mode=3`); or the e-stop is engaged |
| Panel shows "base not in CAN mode" / EXCEPTION | The **RC transmitter is on** — turn it off (RC overrides CAN). Power-cycle the Bunker to clear an EXCEPTION latch — [docs/REAL_PI_NATIVE.md](docs/REAL_PI_NATIVE.md) §5.4 |
| "CAN TX error: Network is down" | The gs_usb bus went down — `bash real_pi/can_up.sh` to bounce `can0` (gs_usb does **not** support `restart-ms`) |
| No / flaky detection boxes | The HSV mask is wrong for today's light — Calibration tab, raise **`S low`** (shadows are low-saturation), then `min_area` for speckle. The mask is your ground truth |
| Colours look wrong (red/blue swapped) | Set `camera/picam_swap_rb` (picamera2 "RGB888" is BGR-ordered). Check the sensor with `rpicam-hello --list-cameras` |
| `A`/`D` don't turn | Fixed — **A/D now turn**; ARM/DISARM moved to **R/F** (classic WASD) |
| numpy `libopenblas.so.0` ImportError | You pip-installed numpy on the Pi — `pip uninstall -y numpy opencv-python` and use apt's (the venv is `--system-site-packages`) |
| Real-robot drive step | Needs the physical Bunker, fenced arena, RC off, e-stop in hand — see [docs/REAL_PI_NATIVE.md](docs/REAL_PI_NATIVE.md) §6–7 |

---

## Project status

**Done**
- ✅ **Sim** validated — detector → behaviour → Gazebo + the Streamlit UI.
- ✅ **Real robot on the Pi** ([docs/REAL_PI_NATIVE.md](docs/REAL_PI_NATIVE.md)) — native
  no-ROS stack **driving under CAN**: HSV calibration + live mask, **pixel distance**,
  **mission completion** (ball *retrieved* / cone *danger* → DISARM), teleop proximity
  warning, and the **Flask control panel** (Drive + Calibration tabs, WASD).
- ✅ **Participant handout + instructor cards** ([../ExerciseHandouts/MiniB2_rover/](../ExerciseHandouts/MiniB2_rover/), v2).

**Remaining / optional**
- Tune **HSV + distance** under the actual arena lighting (done live in the panel).
- Operating procedure: **RC transmitter OFF**, and the **gs_usb CAN bring-up** each
  session (`real_pi/can_up.sh`). Run with the e-stop in hand.
- Optional: a `systemd` **autostart** for `run.py`; **Git LFS** for the demo videos.
- Future (out of current scope): train the **CNN** backend ([docs/TRAINING.md](docs/TRAINING.md))
  for lighting-robust detection.

---

*Space Summer School · Technical University of Crete · SenseLAB*
