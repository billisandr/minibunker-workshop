# REAL_PI_NATIVE — native (no-Docker, no-ROS) bring-up on the Raspberry Pi 5

The Phase-C real-robot path, run as a **single native Python process** on the Pi
instead of the ROS-in-Docker stack. This is plan.md **§9.4** ("documented no-ROS
fallback") promoted to the primary real-deployment path. Written **2026-06-26**.

> The Docker/ROS path (`start_real.sh`, `docs/HARDWARE_SETUP.md`) still exists and
> is unchanged. This document is the **alternative** chosen for the actual Pi:
> lighter, faster to iterate, no arm64 image build. Trade-off: **no sim parity**
> and the drive/CAN layer is re-implemented (see §2).

---

## 0. Why native instead of Docker (the decision)

On `raspberrypi2` (Raspberry Pi OS **Bookworm 64-bit**, Pi 5, 8 GB) we chose to
**skip Docker**:

- ROS Noetic isn't native on Bookworm, so the Docker path means a long arm64
  image build (ROS + the C++ `ugv_sdk`/`bunker_base`, and it also drags in the
  gazebo sim packages we don't need on the Pi).
- The real station only needs: **camera → detector → behaviour → CAN to the
  Bunker**. None of that requires ROS at runtime.
- The hard logic (HSV/CNN detection, the behaviour FSM) is already ROS-free at
  its core in the sim nodes, so it ports almost verbatim.

What we give up (know this): **sim↔real parity** (the native stack can't be run
in Gazebo), the Streamlit/rosbridge UI, and live mission-switching. The drive
layer is re-implemented as a direct CAN driver (§2) instead of `bunker_base`.

---

## 1. The native stack (`real_pi/`)

```
real_pi/
  run.py              # the station: ONE loop, ARM gate, watchdog, e-stop
  config.yaml         # ROS-free subset of minibunker.yaml + a can: block
  requirements.txt    # numpy, opencv-python, python-can, PyYAML
  minibunker_real/
    config.py         # dotted-key YAML loader (replaces rospy.get_param)
    camera.py         # picamera2 -> V4L2 -> video/synthetic frame source
    detector.py       # HSV/CNN backends — LIFTED VERBATIM from detector_node
    perception_state.py  # role-based 7-slot packing — lifted from detector_node
    fsm.py            # behaviour FSM -> (linear, angular) — lifted from behavior_node
    bunker_can.py     # NEW: AgileX protocol-v2 CAN driver (replaces bunker_base)
  tests/
    test_bunker_can.py  # frame encoding + (Pi) vcan0 loopback
    test_fsm.py         # FSM safety + transitions
    test_detector.py    # HSV detection + perception_state packing
```

Dataflow (one process, `behavior/rate_hz` ≈ 20 Hz):

```
camera.read() → detector.detect() → perception_state.pack() → fsm.step()
              → clamp(behavior/limits) → clamp(can/hw_max) → bunker.send_motion(0x111)
```

**Lifted = identical maths.** `detector.py`, `perception_state.py` and `fsm.py`
are byte-for-byte the sim logic with `rospy` removed (rospy.Time → `time.monotonic()`,
`rospy.get_param` → `config.get`). Tune HSV/behaviour the same way you do in sim —
the knob names in `real_pi/config.yaml` match `minibunker.yaml`.

---

## 2. The CAN driver (the one genuinely new piece)

`bunker_can.py` talks the **AgileX protocol v2** directly over socketcan
(`python-can`), reproducing what the C++ SDK's `SetMotionCommand` /
`EnableCommandedMode` send. Frame layout decoded from
`catkin_ws/src/ugv_sdk/src/protocol_v2/agilex_msg_parser_v2.c`:

| Dir | Name | CAN ID | Bytes |
| --- | --- | --- | --- |
| TX | **Motion command** | `0x111` | `int16 BE ×1000`: `[0:2]` linear m/s, `[2:4]` angular rad/s, `[4:6]` lateral=0, `[6:8]` steering=0 |
| TX | **Ctrl-mode config** | `0x421` | `byte0` = control mode: `0x01` = CAN, `0x00` = STANDBY; rest reserved |
| RX | **System state** | `0x211` | `byte0` mode, `[2:4]` battery×0.1 (BE), `[4:6]` error code |
| RX | **Motion state** | `0x221` | `[0:2]` actual linear×1000, `[2:4]` actual angular×1000 |

Key facts:
- The Bunker is **differential/tracked**, so lateral & steering stay 0 and the
  **base firmware mixes the tracks from (linear, angular)** — no v0 track-width
  mixing is needed (simpler than plan.md §9.4 feared).
- On ARM we send `0x421` = `CONTROL_MODE_CAN` so the base accepts CAN commands;
  on shutdown/e-stop we send zero motion **and** `CONTROL_MODE_STANDBY`.
- Hard safety ceiling: **1.5 m/s / 0.7853 rad/s** (`ugv_sdk/.../bunker_params.hpp`),
  clamped inside `send_motion()` regardless of config.
- Protocol: Bunker Mini 2.0 is **AGX_V2**. If your unit reports V1, the IDs differ
  — extend `bunker_can.py` with the v1 frames (see `protocol_v1/`).

---

## 3. One-time Pi setup

```bash
# 3.1 system deps. numpy/OpenCV/PyYAML/picamera2 come from APT (NOT pip — see the
# libopenblas note below), plus the camera + CAN tooling.
sudo apt update
sudo apt install -y python3-venv python3-numpy python3-opencv python3-yaml \
                    python3-picamera2 can-utils

# 3.2 get the code (this branch). The repo is PRIVATE -> git over HTTPS needs a
# Personal Access Token (classic, 'repo' scope) as the password, or `gh auth
# login` first. Your GitHub *account password* will be rejected.
cd ~ && git clone --recurse-submodules https://github.com/billisandr/minibunker-workshop.git
cd minibunker-workshop && git checkout real-pi-native

# 3.3 python venv. --system-site-packages so apt's numpy/cv2/picamera2 are
# importable; pip then only adds python-can.
python3 -m venv ~/mb-venv --system-site-packages
source ~/mb-venv/bin/activate
pip install -r real_pi/requirements.txt
```

> **Do NOT `pip install numpy` / `opencv-python` on the Pi.** The piwheels numpy
> wheel needs `libopenblas.so.0` and, installed into the venv, shadows the
> working apt build -> `ImportError: libopenblas.so.0: cannot open shared object
> file`. If you already did, recover with `pip uninstall -y numpy opencv-python`
> (the `--system-site-packages` venv then falls back to the apt builds:
> numpy 1.24.x, cv2 4.6.x).

---

## 4. Tests (what we can validate, and what needs hardware)

From `real_pi/` with the venv active:

```bash
# pure-logic (no hardware): FSM + detector + CAN frame encoding
python tests/test_fsm.py
python tests/test_detector.py
python tests/test_bunker_can.py        # layer-1 encode/decode always runs

# CAN loopback WITHOUT the robot — virtual CAN:
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
python tests/test_bunker_can.py        # now the vcan0 loopback layer runs too

# perception on the real camera, no motion, no CAN bus:
python run.py --no-can --save-frames /tmp/mbframes   # inspect annotated frames
```

Status on `raspberrypi2` (2026-06-26): **camera present** (`/dev/video0` + ISP
nodes), **no CAN adapter wired yet** (`eth0 lo wlan0` only — no `can0`). So:

| Test | Now | Needs |
| --- | --- | --- |
| FSM / detector / CAN-encode unit tests | ✅ | nothing |
| vcan0 loopback (frame on a real bus) | ✅ | `vcan` module (in-kernel) |
| Camera capture + HSV on real frames | ✅ | the Pi camera (present) |
| **Armed `run.py` actually moving wheels** | ⛔ deferred | **a CAN adapter + the Bunker** |

---

## 5. CAN bring-up (when the adapter arrives)

The Bunker speaks CAN at **500 kbps** on `can0`. Pick the adapter:

```bash
# USB-CAN (gs_usb/slcan) — appears as a native CAN netdev:
sudo ip link set can0 up type can bitrate 500000

# MCP2515 CAN HAT — enable the overlay in /boot/firmware/config.txt first:
#   dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25
# reboot, then the same `ip link set can0 up ...`.

candump can0      # sanity: you should see Bunker frames (0x211, 0x221, ...)
```

Then set `can/channel: can0` in `real_pi/config.yaml` (default).

---

## 6. Running the station (with the robot)

**SAFETY — non-negotiable:** boots **DISARMED** (no motion frames sent), hard
speed caps in `behavior/limits` clamped again to the HW ceiling, watchdog zeroes
on any stall, **Ctrl-C → zero motion + STANDBY**. Keep the hardware e-stop in
hand; run inside the fenced arena only; start with low caps.

```bash
source ~/mb-venv/bin/activate && cd ~/minibunker-workshop/real_pi
sudo ip link set can0 up type can bitrate 500000     # if not already up

# autonomous (set mission/follow_item: ball|cone in config.yaml):
python run.py
# teleop only (set mission/follow_item: none):
python run.py        # then drive with the keys below

# at the prompt (each key + Enter):
#   a = ARM    d = DISARM    q = quit
#   w/s = forward/back   j/l = turn left/right   x = stop   (mission=none)
```

`run.py --headless` (or `--save-frames DIR`) drops the OpenCV window — use it when
running over SSH without X. `--can vcan0` / `--no-can` are dry-run modes.

Teleop is **in-process** (one motion owner on CAN, never a second sender). Keys
are line-buffered (work over SSH); a WASD press auto-expires via the watchdog, so
the rover drives for `behavior/teleop/timeout_ms` then stops unless re-pressed.

---

## 7. Workflow log (how this got built, 2026-06-26)

1. SSH'd to `pi@raspberrypi2` (Bookworm 64-bit, Pi 5). Survey: git ✓, **docker
   absent**, no `can0`, camera present (`/dev/video0`), 46 GB free, 8 GB RAM.
2. **Decision:** skip Docker/ROS → native Python (§0), on branch `real-pi-native`
   in the same repo (keeps detection/FSM logic diffable against sim; `real_pi/`
   is purely additive so it can't break the validated sim).
3. Extracted the Bunker **protocol-v2 CAN frames** from the vendored `ugv_sdk`
   (motion `0x111`, ctrl-mode `0x421`, state `0x211`/`0x221`) → `bunker_can.py`.
4. Lifted `detector.py`, `perception_state.py`, `fsm.py` from the ROS nodes
   (rospy removed); wrote `camera.py`, `config.py`, `run.py` (loop + safety).
5. Tests: 16/16 pure-logic pass on the dev laptop + a 60-frame end-to-end
   pipeline smoke (synthetic camera → HSV → pack → FSM). vcan loopback added for
   the Pi.
6. On the Pi: repo is **private** (git needs a PAT / `gh auth login`, not the
   account password). Use **apt** numpy/OpenCV, not pip (piwheels numpy ⇒
   `libopenblas.so.0` missing + shadows the apt build); `test_fsm` 7/7 first try.
7. **Deferred:** real-motor drive until a CAN adapter is wired (§4 table).

## 8. Open items / next steps

- **Wire a CAN adapter** (USB-CAN or MCP2515 HAT), `candump` the Bunker, then the
  first **armed, low-cap** drive test in the fenced arena.
- **Camera tuning:** confirm `/dev/video0` is the Pi Camera (not an ISP node);
  set `camera/source` + re-check HSV ranges under arena lighting.
- **Autostart (optional):** a `systemd` unit for `run.py` once drive is trusted.
- **CNN:** drop a trained `.onnx` in and set `detector/backend: cnn` (HSV is the
  working default).
- If a unit reports **protocol v1**, add the v1 frames to `bunker_can.py`.
```
