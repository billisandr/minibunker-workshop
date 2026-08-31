# REAL_PI_NATIVE — native (no-Docker, no-ROS) bring-up on the Raspberry Pi 5

The Phase-C real-robot path, run as a single native Python process on the Pi
instead of the ROS-in-Docker stack. This is the documented no-ROS fallback
promoted to the primary real-deployment path. Written 2026-06-26, including
the real-hardware bring-up results captured the same day.

> The Docker/ROS path (`start_real.sh`, `docs/HARDWARE_SETUP.md`) still exists and
> is unchanged. This document covers the alternative we chose for the actual Pi:
> lighter, faster to iterate, and with no arm64 image build. The trade-off is no
> sim parity, and the drive/CAN layer is re-implemented (see §2).

Contents: §0 decision, §1 stack, §2 CAN driver, §3 one-time setup, §4
hardware-free tests (with outputs), §5 live-CAN bring-up (worked example), §6
autonomous-follow dry-run with the e-stop engaged (worked example), §7 running
plus the first real drive, §8 viewing frames and video, §9 status, §10 log, §11
open items.

---

## 0. Why native instead of Docker (the decision)

On `raspberrypi2` (Raspberry Pi OS Bookworm 64-bit, Pi 5, 8 GB) we chose to skip
Docker:

- ROS Noetic isn't native on Bookworm, so the Docker path means a long arm64
  image build (ROS plus the C++ `ugv_sdk`/`bunker_base`, and it also drags in the
  Gazebo sim packages we don't need on the Pi).
- The real station only needs camera, detector, behaviour, and CAN to the Bunker.
  None of that requires ROS at runtime.
- The hard logic (HSV/CNN detection, the behaviour FSM) is already ROS-free at
  its core in the sim nodes, so it ports almost verbatim.

What we give up, and it's worth knowing this going in: sim-to-real parity, since
the native stack can't run in Gazebo, and the Streamlit/rosbridge UI, replaced by
a native Flask panel with its own live mission-switching (§8b). The drive layer is
re-implemented as a direct CAN driver (§2) instead of `bunker_base`.

---

## 1. The native stack (`real_pi/`)

```
real_pi/
  run.py              # the station: ONE loop, Controls hub, ARM gate, watchdog, viz/record/web
  config.yaml         # ROS-free subset of minibunker.yaml + a can: block
  requirements.txt    # python-can + flask (numpy/OpenCV/PyYAML come from apt — see §3)
  minibunker_real/
    config.py         # dotted-key YAML loader (replaces rospy.get_param)
    camera.py         # picamera2 -> V4L2 -> video/synthetic frame source
    detector.py       # HSV/CNN backends — LIFTED VERBATIM from detector_node
    perception_state.py  # role-based 7-slot packing — lifted from detector_node
    fsm.py            # behaviour FSM -> (linear, angular) — lifted from behavior_node
    bunker_can.py     # NEW: AgileX protocol-v2 CAN driver (replaces bunker_base)
    webpanel.py       # NEW: optional Flask control panel (--web), shares Controls
  panel/
    index.html        # the panel page (Pi-served or laptop-served; configurable API base)
    serve_laptop.py   # static host for the laptop-server option
  tests/
    test_bunker_can.py  # frame encoding + (Pi) vcan0 loopback
    test_fsm.py         # FSM safety + transitions
    test_detector.py    # HSV detection + perception_state packing
```

Dataflow (one process, `behavior/rate_hz` runs at roughly 20 Hz):

```
camera.read() → detector.detect() → perception_state.pack() → fsm.step()
              → clamp(behavior/limits) → clamp(can/hw_max) → bunker.send_motion(0x111)
```

Lifted means identical maths. `detector.py`, `perception_state.py`, and `fsm.py`
are byte-for-byte the sim logic with `rospy` removed (`rospy.Time` becomes
`time.monotonic()`, `rospy.get_param` becomes `config.get`). Tune HSV and
behaviour the same way you do in sim; the knob names in `real_pi/config.yaml`
match `minibunker.yaml`.

---

## 2. The CAN driver (the one genuinely new piece)

`bunker_can.py` talks the AgileX protocol v2 directly over socketcan
(`python-can`), reproducing what the C++ SDK's `SetMotionCommand` and
`EnableCommandedMode` send. The frame layout was decoded from
`catkin_ws/src/ugv_sdk/src/protocol_v2/agilex_msg_parser_v2.c`:

| Dir | Name | CAN ID | Bytes |
| --- | --- | --- | --- |
| TX | Motion command | `0x111` | `int16 BE ×1000`: `[0:2]` linear m/s, `[2:4]` angular rad/s, `[4:6]` lateral=0, `[6:8]` steering=0 |
| TX | Ctrl-mode config | `0x421` | `byte0` = control mode: `0x01` = CAN, `0x00` = STANDBY; rest reserved |
| RX | System state | `0x211` | `byte0` vehicle_state (0 NORMAL / 1 ESTOP / 2 EXCEPTION), `byte1` control_mode (0 STANDBY / 1 CAN), `[2:4]` battery×0.1 (BE), `[4:6]` error_code (BE) |
| RX | Motion state | `0x221` | `[0:2]` actual linear×1000, `[2:4]` actual angular×1000 |

Decode gotcha, caught on the real robot: byte0 of `0x211` is `vehicle_state`, not
`control_mode` (control_mode is byte1). The first cut read byte0 as the mode and
so reported "mode=1" when the rover was actually e-stopped
(`vehicle_state=ESTOP=1`). Fixed in `bunker_can.py`; `BunkerState` now exposes
`vehicle_state`, `control_mode`, and an `estop_engaged` helper.

Key facts:
- The Bunker is differential/tracked, so lateral and steering stay 0 and the base
  firmware mixes the tracks from (linear, angular). No v0 track-width mixing is
  needed, simpler than we'd expected going in.
- On ARM we send `0x421` = `CONTROL_MODE_CAN` so the base accepts CAN commands;
  on shutdown or e-stop we send zero motion and `CONTROL_MODE_STANDBY`.
- Hard safety ceiling: 1.5 m/s / 0.7853 rad/s (`ugv_sdk/.../bunker_params.hpp`),
  clamped inside `send_motion()` regardless of config.
- Protocol: Bunker Mini 2.0 is AGX_V2. If your unit reports V1, the IDs differ,
  so extend `bunker_can.py` with the v1 frames (see `protocol_v1/`).

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

Do not `pip install numpy` or `opencv-python` on the Pi. The piwheels numpy wheel
needs `libopenblas.so.0`, and once it's installed into the venv it shadows the
working apt build, producing `ImportError: libopenblas.so.0: cannot open shared
object file`. If you already did this, recover with `pip uninstall -y numpy
opencv-python` (the `--system-site-packages` venv then falls back to the apt
builds: numpy 1.24.x, cv2 4.6.x).

---

## 4. Hardware-free tests (run these first)

From `real_pi/` with the venv active. None of these need the robot.

### 4.1 Pure-logic unit tests
```bash
python tests/test_fsm.py
python tests/test_detector.py
python tests/test_bunker_can.py     # layer-1 frame encode/decode always runs
```
Expected, validated on `raspberrypi2`, 2026-06-26:
```
# test_fsm.py
PASS test_approach_steers_toward_target
... 7/7 passed
# test_detector.py
PASS test_hsv_finds_green_ball
... 4/4 passed
# test_bunker_can.py   (vcan0 absent -> the loopback layer skips)
PASS test_motion_is_big_endian_x1000
[skip] vcan0 / python-can not available
... 5/5 passed
```

### 4.2 CAN loopback without the robot (virtual CAN)
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
python tests/test_bunker_can.py     # now the vcan0 loopback layer runs too
```
Expected: the loopback line replaces the skip, proving the frames are correct on a
real socketcan bus:
```
[ok] vcan0 loopback: motion, ctrl-mode, vehicle_state+battery decode
5/5 passed
```

### 4.3 Real camera, no motion, no CAN bus
```bash
python run.py --no-can --save-frames /tmp/mbframes   # Ctrl-C / 'q'+Enter to stop
```
This runs camera, detector, and FSM headless, and writes annotated frames (every
5th) to `/tmp/mbframes`. Confirmed output on the IMX219 (Pi Camera v2):
```
[camera] picamera2 (libcamera)
... 640x480 frames, brightness ~126, clean DISARMED -> STANDBY on exit
```
(To view the frames, see §8.)

### 4.4 Camera colour and health
Wrong or "strange" colours, especially reds and blues swapped, on either camera
are a software channel-order issue, not bad sensors: picamera2's `"RGB888"`
already returns a BGR-ordered array (libcamera fourcc names are byte-reversed),
so no swap is needed. The default is correct
(`camera/picam_swap_rb: false`); flip it only if your build differs.

Diagnose it: this writes both interpretations and prints channel means, so you
can compare against the stock libcamera tool as ground truth:
```bash
python tests/camera_check.py            # -> /tmp/cam_asis.jpg + /tmp/cam_swapped.jpg
rpicam-still -o /tmp/cam_ref.jpg        # (or libcamera-still) ground-truth colour
cd /tmp && python3 -m http.server 8000  # view all three from the laptop browser
# whichever of asis/swapped matches cam_ref sets camera/picam_swap_rb (false/true)
```

Camera health, meaning is the sensor even detected and happy:
```bash
rpicam-hello --list-cameras             # lists sensor(s) + modes (imx219 = Pi Cam v2)
rpicam-still -o /tmp/t.jpg              # stock capture; correct colour here = HW fine
dmesg | grep -i imx219                  # probe / I2C errors if it isn't detected
```
If `rpicam-still` colours are right but the app's are wrong, it's our pixel order
(toggle `picam_swap_rb`). If `rpicam-still` is also wrong, it's white-balance or
lighting: set `camera/awb_enable: false` and `camera/colour_gains: [r, b]` for a
fixed cast, or re-tune the HSV ranges in `config.yaml` under the arena light.

---

## 5. Live-CAN bring-up — worked example (e-stop engaged)

The Bunker speaks CAN at 500 kbps on `can0`. Keep the e-stop engaged for
everything in this section; these are read-only checks, so nothing should move.

### 5.1 Interface up + raw sanity
```bash
ip -br link show | grep -i can          # confirm a can0 appeared
bash real_pi/can_up.sh                   # bounce can0 up @ 500k (gs_usb-safe)
candump can0                            # Ctrl-C after you see frames
```
(`can_up.sh` is just `ip link set can0 down; up type can bitrate 500000`; the
gs_usb adapter needs this each session and after any bus-off, see §5.3.) On the
real Bunker you should see a steady stream. These are the actual IDs we saw
(system, motion, RC, and odometry/actuator state):
```
can0  211   [8]  01 00 01 00 00 80 00 00     # system state (see decode below)
can0  221   [8]  00 00 00 00 00 00 00 00     # motion state (actual vel = 0)
can0  241   [8]  AA 00 00 00 00 00 00 61     # RC state (counter increments)
can0  311   [8]  00 00 00 00 00 00 00 00     # odometry
```

### 5.2 Confirm the driver decodes the robot (no commands sent)
```bash
source ~/mb-venv/bin/activate && cd ~/minibunker-workshop/real_pi
python - <<'PY'
import time
from minibunker_real.bunker_can import BunkerCAN
b = BunkerCAN(channel="can0")
print("listening 6s (no commands sent)...")
t0 = time.time()
while time.time() - t0 < 6:
    s = b.poll(timeout=0.2)
    print(f"vstate={s.vehicle_state}{' ESTOP' if s.estop_engaged else ''} "
          f"ctrl_mode={s.control_mode} batt={s.battery_voltage:.1f}V "
          f"err={s.error_code} actual v={s.actual_linear:+.2f} w={s.actual_angular:+.2f}")
    time.sleep(0.5)
b.bus.shutdown()
PY
```
Worked output, decoding `0x211 = 01 00 01 00 00 80 ..`:
```
vstate=1 ESTOP ctrl_mode=0 batt=25.6V err=128 actual v=+0.00 w=+0.00
```
That is vehicle_state=ESTOP (e-stop engaged, as expected), control_mode=STANDBY
(nothing has enabled CAN yet), battery 25.6 V (healthy), and error_code `0x80`.
Seeing a sane battery reading plus the ESTOP flag means the full RX path works.

### 5.3 Troubleshooting — `OSError 100 Network is down` / bus-off
`can0` opens even when the link is down, so `run.py` can start and then every send
fails. If a node transmits with nothing ACKing (Bunker off, CAN cable loose, wrong
bitrate), the controller hits bus-off. On the gs_usb USB-CAN adapter used here
that latches the interface to `state STOPPED / DOWN` and it cannot auto-recover;
recovery is always a manual down/up bounce. The driver is hardened (sends are
best-effort, the loop and shutdown never crash, and the panel shows a CAN TX error
banner), so you can bounce the bus without restarting `run.py`:

```bash
ip -details link show can0     # gs_usb shows "can state STOPPED" + "state DOWN" when faulted
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000      # NO restart-ms on gs_usb!
candump can0                   # you MUST see 0x211/0x221 from the base (Bunker on + wired)
```

gs_usb gotcha: `... up type can bitrate 500000 restart-ms 100` fails with `Device
doesn't support restart from Bus Off` and leaves the link down. Omit
`restart-ms`. (A MCP2515 CAN HAT does support it; gs_usb does not.)

If `candump` stays silent after a clean up, the Bunker isn't transmitting: check
it's powered on, the CAN cable is seated, and the bitrate is 500000. To bring
`can0` up automatically at boot, add a `systemd-networkd` `.network`/`.link` unit
or a tiny `ip link` unit (optional; otherwise re-run the up command each session).

### 5.4 Troubleshooting — RC-vs-CAN contention / `ctrl_mode=3` / EXCEPTION lock
The `0x211` control_mode field tells you who is driving the base: `0`=STANDBY,
`1`=CAN (us), `2`=UART, `3`=RC (handheld remote). AgileX gives the RC priority
over CAN. If the RC transmitter is on, you'll see `ctrl_mode` flip between 1 and 3
while armed, since our `CONTROL_MODE_CAN` is repeatedly overridden by the RC. That
thrash, especially with an e-stop toggle mid-drive, can fault the base to
`vehicle_state=2` EXCEPTION, which latches: after that, `ctrl_mode` sticks at 3,
`actual_v` freezes, and CAN commands are ignored.

Fix:
1. Turn the RC transmitter off (or set its mode switch to hand control to CAN or
   external). With the RC on, CAN can never hold `ctrl_mode=1`.
2. Clear the exception: DISARM and quit `run.py`, then power-cycle the Bunker
   (engaging and releasing the e-stop may also reset it). Idle should return to
   `ctrl_mode=0`, `vstate=0`.
3. Re-arm with the RC off: `ctrl_mode` should go to 1 and stay there.

The panel and the headless console now flag this ("base not in CAN mode ... RC
on?" and "base EXCEPTION ... power-cycle").

---

## 6. Autonomous-follow dry-run — worked example (e-stop engaged)

This is the safe way to validate the whole autonomous pipeline before any real
motion. With the e-stop engaged the base ignores motion commands, so the stack
arms, detects the ball, and computes follow velocities while the rover physically
cannot move, so you can verify behaviour from the telemetry alone.

```bash
# 1. follow the ball
sed -i 's/^  follow_item:.*/  follow_item: ball/' real_pi/config.yaml
# 2. run with live state telemetry + recorded frames
source ~/mb-venv/bin/activate && cd ~/minibunker-workshop/real_pi
python run.py --save-frames /tmp/mbframes
```
Then read the telemetry: it must show `vstate=1 ESTOP!` before you arm, which is
your safety gate. Type `a`+Enter to arm, hold a green ball in front of the camera,
then `d` to disarm and `q` to quit.

Worked output (the real 2026-06-26 dry-run, abridged):
```
[STOP    ] armed=False cmd v=+0.00 w=+0.00 | batt=25.6V ctrl_mode=0 vstate=1 ESTOP! actual_v=+0.00
a
[run] >>> ARMED
[run] sent CONTROL_MODE_CAN
[SEARCH  ] armed=True  cmd v=+0.00 w=+0.50 | batt=25.6V ctrl_mode=1 vstate=1 ESTOP! actual_v=+0.00
[APPROACH] armed=True  cmd v=+0.20 w=+0.46 | batt=25.6V ctrl_mode=1 vstate=1 ESTOP! actual_v=+0.00
[APPROACH] armed=True  cmd v=+0.17 w=-0.61 | batt=25.6V ctrl_mode=1 vstate=1 ESTOP! actual_v=+0.00
[SEARCH  ] armed=True  cmd v=+0.00 w=+0.50 | batt=25.6V ctrl_mode=1 vstate=1 ESTOP! actual_v=+0.00
d
[run] >>> DISARMED
[STOP    ] armed=False cmd v=+0.00 w=+0.00 | batt=25.6V ctrl_mode=1 vstate=1 ESTOP! actual_v=+0.00
```
What this proves:
- `ctrl_mode` flips 0 to 1 on arm, meaning our `0x421` reached the base and it
  entered CAN mode.
- SEARCH moves to APPROACH as the ball appears; `v` is roughly 0.2 forward and `w`
  steers toward it.
- `w` flips sign (+0.46 to -0.61) as the ball crosses sides, showing the
  P-control tracking works.
- `actual_v=0` and `vstate=ESTOP` hold throughout, so we're commanding follow
  behaviour, not actually moving. Safe.
- The brief APPROACH-to-SEARCH blips are HSV momentarily losing the ball (from
  lighting or motion blur); tune the HSV ranges or `behavior/limits/lost_frames`
  if it's twitchy.

---

## 7. Running the station + the first real drive

Safety, non-negotiable: the stack boots disarmed (no motion frames), hard caps in
`behavior/limits` are re-clamped to the hardware ceiling, the watchdog zeroes
motion on stall, and Ctrl-C zeroes motion and returns to standby. Keep the
hardware e-stop in hand, stay in the fenced arena only, and start with low caps.

One-command start, recommended: `mb`. `start_real_pi.sh` does the safe ordered
start: it bounces CAN up (gs_usb), confirms the Bunker is on the bus, then
launches `run.py --web`. Install the alias once:
```bash
echo "alias mb='bash ~/minibunker-workshop/real_pi/start_real_pi.sh'" >> ~/.bashrc
source ~/.bashrc
```
then, with the Bunker on, the RC off, and the e-stop in hand:
```bash
mb                    # bounce CAN + sanity-check + launch the --web panel
mb -f                 # skip the CAN frame check (force)
mb --save-video /tmp/run.mp4    # extra args pass through to run.py
```
It refuses to start if a `run.py` is already running (two motion owners on one
bus would be unsafe) or if no CAN frames are seen (Bunker off or an RC issue).
Stop it with Ctrl-C before power-cycling the Bunker, then run `mb` again.

Manual equivalent:
```bash
source ~/mb-venv/bin/activate && cd ~/minibunker-workshop/real_pi
bash can_up.sh                # bounce can0 up (gs_usb)

python run.py                 # autonomous: needs mission/follow_item: ball|cone
python run.py --headless      # same, no OpenCV window (SSH without X)

# keys (each + Enter):  r=ARM  f=DISARM  q=quit
#   teleop (mission/follow_item: none):  w/a/s/d=drive  x=stop
```
Teleop runs in-process, so there's one motion owner on CAN and never a second
sender. Keys are line-buffered (they work over SSH); a WASD press auto-expires via
the watchdog, so the rover drives for `behavior/teleop/timeout_ms` then stops
unless you press it again.

### First real motion (release the e-stop) — recommended order
Everything upstream of the wheels is validated (§5–§6); the only untested thing is
real motion. Do it in two steps:

1. Teleop sanity first. Set `follow_item: none`, drop the caps, release the
   e-stop, arm, give a single `w`+Enter nudge, confirm the wheels move the right
   way and `actual_v` goes non-zero, then hit `x` and disarm:
   ```bash
   sed -i 's/^  follow_item:.*/  follow_item: none/' real_pi/config.yaml
   # optional: lower first-drive caps
   sed -i 's/^    max_linear:.*/    max_linear: 0.15/'  real_pi/config.yaml
   sed -i 's/^    max_angular:.*/    max_angular: 0.4/' real_pi/config.yaml
   python run.py --headless          # 'a' ARM, one 'w', watch actual_v, 'x', 'd', 'q'
   ```
2. Then autonomous follow for real. Set `follow_item: ball` back, release the
   e-stop, arm, and place the ball; it drives to it through COLLECT, then
   RETREAT, then SEARCH.

If anything looks wrong, hit the e-stop, which is instant, or press `d`/`q`/Ctrl-C
(all zero the motion). The base also stops on its own if motion frames stop
arriving.

---

## 8. Viewing the debug frames & video

`run.py --save-frames DIR` writes annotated JPEGs; `run.py --save-video FILE`
records an annotated video. Over SSH, with no display, you can view them one of
these ways:

### (a) Serve over HTTP, view in your laptop browser (easiest)
```bash
# on the Pi:
cd /tmp/mbframes && python3 -m http.server 8000
```
On your laptop open `http://raspberrypi2.local:8000` (or the Pi's IP address) and
click any `f000xx.jpg`. `Ctrl-C` on the Pi stops the server.

### (b) Copy to the laptop with scp
```bash
# in a LAPTOP terminal (enter the Pi password):
mkdir -p ~/mbframes_local
scp "pi@raspberrypi2.local:/tmp/mbframes/*.jpg" ~/mbframes_local/
```

### (c) Record / watch as a video
Record directly while running, in one file, so no glob is needed:
```bash
python run.py --save-video /tmp/mb_run.mp4      # .mp4 -> mp4v; .avi -> MJPG
```
Or build a video from already-saved frames with ffmpeg (frames are every 5th of a
20 Hz loop, so roughly 4 fps real-time):
```bash
sudo apt install -y ffmpeg
ffmpeg -framerate 4 -pattern_type glob -i '/tmp/mbframes/f*.jpg' \
       -c:v libx264 -pix_fmt yuv420p /tmp/mb_run.mp4
```
Then view the single file either through (a) the HTTP server (`cd /tmp && python3
-m http.server 8000`, then open `http://raspberrypi2.local:8000/mb_run.mp4`) or
(b) by scp-ing it to the laptop. If an `.mp4` comes out empty on the Pi's OpenCV
build, use `--save-video /tmp/mb_run.avi` (MJPG) or the ffmpeg route instead.

---

## 8b. Web control panel (`--web`)

A tiny Flask panel gives live annotated video (MJPEG), telemetry, ARM/DISARM, a
mission switch, and press-and-hold teleop for the native stack. It shares the same
`Controls` object as the keyboard, so it's just another input to the one loop
(still a single motion owner on CAN). The sim's Streamlit/rosbridge UI does not
work here, since there's no ROS; this panel is its native replacement. It needs
`pip install flask` (already in `requirements.txt`).

Files: `minibunker_real/webpanel.py` (server), `panel/index.html` (page), and
`panel/serve_laptop.py` (a laptop static-host helper).

### Option A — run the panel on the Pi (recommended)
```bash
source ~/mb-venv/bin/activate && cd ~/minibunker-workshop/real_pi
python run.py --web                 # control loop + panel in one process
#   --web implies headless; --web-port 8080 (default), --web-host 0.0.0.0
```
Then on your laptop open `http://raspberrypi2.local:8080` (leave the page's "API
base" field blank to use the same origin). You can arm, switch mission, and drive
all from the browser, with the live camera.

### Option B — run the panel server on the laptop
The control loop and API always run on the Pi (`python run.py --web`), since
that's where the CAN and camera are. Option B only moves the static page to the
laptop and points it at the Pi's API (CORS is open):
```bash
# on the PI (as in Option A):
python run.py --web
# on the LAPTOP, from real_pi/panel/:
python serve_laptop.py              # -> http://localhost:8090
```
Open `http://localhost:8090`, and set the page's API base to
`http://raspberrypi2.local:8080`. Use this if you want to host or iterate on the
UI from the laptop while the Pi serves only JSON and the MJPEG stream.

Endpoints: `GET /api/state`, `GET /stream.mjpg`, `GET /mask.mjpg`, `GET|POST
/api/hsv`, `POST /api/arm?armed=1`, `POST /api/teleop?key=w`, `POST
/api/mission?follow=ball`, `POST /api/maskclass?cls=green_ball`. POSTs use query
params (CORS-"simple", so no preflight) so the laptop-served page can call the
Pi. Safety is unchanged: the panel's ARM flows through the same FSM ARM gate,
clamp, and watchdog, and the physical e-stop always overrides (shown as an ESTOP
banner). Web teleop auto-expires via the watchdog on release or link loss.

### Panel layout — Drive + Calibration
- Camera column, on both tabs: the annotated stream, with the live HSV mask as a
  second feed beneath it.
- Drive tab (default): ARM/DISARM, mission, hold-to-drive teleop, and telemetry.
- Calibration tab: reveals a third column where you pick `green_ball` or `cone`,
  drag the H/S/V lower and upper plus min_area sliders, and watch the mask clean
  up live (the detector's ranges update each frame), then copy the shown
  `config.yaml` snippet into `detector/hsv`. This is how you fix flaky or
  shadow-triggered detections under the real arena light, since the sim's HSV
  ranges were tuned for Gazebo, not a real camera. Calibration needs
  `detector/backend: hsv`.

How to tune HSV, in order, while holding the object in view and watching the
mask:
1. Raise `S low` (saturation). Shadows and greys are low-saturation, so this is
   the biggest lever for dropping "shadow reads as a ball."
2. Raise `V low` (value/brightness) to cut dark regions, and tighten `H low`/`H
   high` around the object's actual hue (green is roughly 40 to 85, orange
   roughly 5 to 25, on OpenCV's 0 to 179 hue scale).
3. Raise `min_area` to ignore small speckle blobs.

The goal is a clean solid blob on the object and black everywhere else. Then copy
the snippet into `config.yaml` so it persists. If green can't be separated from
the floor or shadows even after tuning, that's the inherent limit of colour rules,
and a learned CNN backend (`detector/backend: cnn` plus a trained `.onnx`) is
scaffolded for that but is out of the current workshop's scope. HSV is the path
here.

### Distance (pixel estimate)
A one-point pinhole estimate lives in the Distance subsection of the Calibration
tab: `distance_m = ref_distance_m × ref_height_px / bbox_height_px`
(`minibunker_real/distance.py`). Calibrate once per object: get a clean
detection, type the known distance, click "Calibrate from view," and it grabs the
live bbox height as the reference; the live estimate and a `config.yaml` snippet
update. The Drive telemetry then shows the target distance in metres.

CLI alternative, no browser needed:
```bash
python tests/distance_calibrate.py --class green_ball --dist 1.0   # medians 20 frames
```
Config lives in `detector/distance` (`ref_height_px: 0` means uncalibrated, so no
estimate is shown). To make the rover stop at a real distance, set
`behavior/approach/collect_distance_m: <metres>`. The FSM then collects at that
distance for a calibrated class, and falls back to `collect_bbox_frac` otherwise.

Detection labels: each box is annotated with a short name and live distance once
calibrated (for example `b 0.85m`), or a name and score otherwise (`b 0.79`). Set
the short names in `detector/display_names` (for example `green_ball: b`, `cone:
c`); this is annotation only and doesn't change `mission/follow_item` or the
HSV/distance class keys.

### Mission completion + proximity (distance-driven behaviour)
Once the classes are distance-calibrated, the rover acts on real metres, and all
thresholds live in `config.yaml` so you can tune them:

| Mission | At distance | Behaviour |
| --- | --- | --- |
| ball | `approach/ball_retrieve_m` (0.7 m) | Retrieved: stop, log "ball retrieved", show the panel's success banner, and switch to mission `none` (stays armed) |
| cone | `approach/cone_danger_m` (0.5 m) | Danger: show the panel's danger banner, back up for `cone_backup_sec` at `cone_backup_speed`, and switch to mission `none` (stays armed) |
| none (teleop) | `proximity_warn_m` (1.0 m) | Any object closer triggers the panel's proximity warning, with no motion change |

The FSM raises the event, and `run.py` handles the mission switch and the
persistent banner, which clears on the next ARM edge or on DISARM. Uncalibrated
classes fall back to `approach/collect_bbox_frac`. (This replaces the old
collect-then-retreat cycle.)

---

## 9. Status summary (2026-06-26, `raspberrypi2`)

Camera: Pi Camera v2 (IMX219) via picamera2 at 640×480. CAN: `can0` at 500 kbps
to the real Bunker Mini. Battery 25.6 V. System libs: numpy 1.24.2, cv2 4.6.0.

| Check | State | Evidence |
| --- | --- | --- |
| `test_fsm` (FSM safety + transitions) | pass | 7/7 |
| `test_detector` (HSV + perception_state) | pass | 4/4 |
| `test_bunker_can` incl. vcan0 loopback | pass | 5/5, `[ok] vcan0 loopback` |
| Camera capture (`run.py --no-can`) | pass | IMX219/picamera2, 640×480, clean DISARMED→STANDBY |
| Live CAN to the real Bunker | pass | candump + decode: batt 25.6 V, vstate=ESTOP, ctrl_mode 0→1 on ARM |
| Autonomous follow dry-run, e-stop engaged | pass | SEARCH→APPROACH, `w` steers/flips with the ball, `actual_v=0` |
| Real motion (e-stop released, armed, mission=ball) | pass | drove under CAN: `ctrl_mode=1`, `actual_v≈+0.20` tracking `cmd v` during APPROACH |
| RC transmitter off for clean CAN control | required | with RC on, `ctrl_mode` flips 1↔3 and the base faults to EXCEPTION (§5.4) |

---

## 10. Workflow log (how this got built, 2026-06-26)

1. SSH'd to `pi@raspberrypi2` (Bookworm 64-bit, Pi 5). Survey: git present, docker
   absent, no `can0`, camera present (`/dev/video0`), 46 GB free, 8 GB RAM.
2. Decision: skip Docker/ROS and go native Python (§0), on branch
   `real-pi-native` in the same repo, so detection and FSM logic stay diffable
   against sim, and `real_pi/` is purely additive so it can't break the validated
   sim.
3. Extracted the Bunker protocol-v2 CAN frames from the vendored `ugv_sdk`
   (motion `0x111`, ctrl-mode `0x421`, state `0x211`/`0x221`) into `bunker_can.py`.
4. Lifted `detector.py`, `perception_state.py`, and `fsm.py` from the ROS nodes
   with rospy removed, then wrote `camera.py`, `config.py`, and `run.py` (the
   loop and its safety logic).
5. Tests: 16/16 pure-logic tests pass on the dev laptop, plus a 60-frame
   end-to-end pipeline smoke test (synthetic camera through HSV, packing, and
   FSM). vcan loopback was added at this point.
6. On the Pi: the repo is private, so git needs a PAT rather than the account
   password. Use apt's numpy/OpenCV, not pip, since piwheels numpy causes the
   `libopenblas.so.0` shadow issue.
7. All hardware-free tests pass (§4): fsm 7/7, detector 4/4, bunker_can 5/5
   including the real `vcan0` loopback.
8. Camera validated: IMX219 via picamera2 at 640×480, capturing 30/30 frames.
9. CAN wired plus a read-only bring-up caught a SystemState decode bug (byte0 is
   `vehicle_state`, not `control_mode`), fixed before any motion (§2 gotcha).
10. Autonomous follow dry-run, e-stop engaged (§6): SEARCH moved to APPROACH,
    steering tracked and sign-flipped on the ball, `ctrl_mode` went 0 to 1, and
    `actual_v=0` throughout, so the full pipeline was validated with zero motion.
11. Real motion achieved (e-stop released): armed autonomous follow drove the
    Bunker under CAN, with `ctrl_mode=1` and `actual_v≈+0.20` tracking the
    APPROACH command. Then we hit the RC-vs-CAN contention lock: the RC
    transmitter was on (`ctrl_mode` flipping 1↔3), and the base faulted to
    `vstate=2` EXCEPTION (§5.4). The fix was RC off plus a power-cycle. The
    panel and console now flag mode-mismatch and EXCEPTION conditions directly.

## 11. Open items / next steps

- First real drive (§7): a teleop sanity nudge, then autonomous follow, with low
  caps in the fenced arena. This is the only remaining gate; everything upstream
  is validated.
- HSV tuning under the actual arena lighting, to reduce the APPROACH-to-SEARCH
  blips.
- Autostart (optional): a systemd unit for `run.py` once drive is trusted.
- CNN (future, out of current scope): drop a trained `.onnx` in and set
  `detector/backend: cnn` for lighting-robust detection (HSV stays the workshop
  default).
- If a unit reports protocol v1, add the v1 frames to `bunker_can.py`.
