# MiniBunker Workshop — Plan (unified)

> **This is the single, living plan for the station.** It consolidates the former
> three-file chain — `plan.md` (Phase-0 design + architecture), `plan2.md`
> (missions/teleop/perception roadmap) and `plan3.md` (mission/teleop delivery +
> arena/physics + dev-mount) — into one document. Consolidated **2026-06-29**.
> **§0–§16 are the original design backbone** and are still referenced by section
> number from code and docs (`plan.md §11`, `§4.4`, `§10`, …) — **do not renumber
> them**. **§17 onward** carry the current status, the delivered missions/teleop
> work, the perception roadmap, the "how it works now" mechanisms, the remaining
> work, and the running changelog.
>
> **Where it stands now (2026-06-29):** the **sim is hands-on validated** — the rover
> SEARCH→APPROACH→COLLECT→RETREAT on a green "ore" ball, AVOIDs/knocks cones, and
> drives under WASD, all live-tunable from the Streamlit UI (HSV backend). Done:
> rename/de-brand, missions (`follow_item`), WASD teleop, a fully config-driven
> dynamic arena, and a dev bind-mount for no-rebuild edits. **Not done:** real-robot
> bring-up on the Pi 5 (CAN + Pi camera), the pluggable modern-perception stages
> (LocateAnything / SAM / Depth Anything), and the CNN actually being trained.
> Full status in **§17**; remaining work in **§21**.

> **De-brand note.** The project was renamed and fully de-branded (2026-06-24): the
> "SpaceMine" station name was dropped for plain **"MiniBunker"**, the repo is
> **`minibunker-workshop`**, the Docker image is **`minibunker`**, and the Gazebo world
> is **`minibunker_arena.world`**. References below to *"SpaceMine"*,
> `minibunker-spacemine-workshop`, or `spacemine_arena.world` are **historical** — the
> §12 `gh repo create` commands were actually run under the old names. The genuine
> theme (the event "Space Summer School", the "space-mining" flavour, the "ore" ball,
> 🛰️) is kept; only the "SpaceMine" brand was removed. Rename record: §15 + §17.

> **Purpose & scope.** A self-contained build + status plan for the workshop station:
> an AgileX **Bunker Mini 2.0** tracked rover that uses a **Raspberry Pi 5 + Raspberry
> Pi Camera** to run a **small CNN** recognising a **green ball** (the "ore sample" to
> approach) and **construction cones** (hazards to avoid), driving a *space-mining*
> behaviour on top. It runs in **two interchangeable modes** — a **Gazebo simulation**
> and the **real MiniBunker 2.0** — driven by a single **YAML config** and a
> **Streamlit UI**. Written so a fresh session can act from this one file without prior
> context; read §0 + §17 first, then dive where needed.

---

## 0. TL;DR

Build a non-coder workshop station where a tracked MiniBunker rover **chases a green
ball and avoids construction cones**, using a Raspberry Pi camera + a lightweight CNN
detector — runnable identically in **Gazebo sim** and on the **real robot**, fully
**YAML-configurable**, fronted by a **Streamlit knob panel**.

- **Stand on three AgileX repos** (vendored as git submodules):
  `ugv_gazebo_sim` (Gazebo models incl. Bunker), `bunker_ros` (ROS driver:
  `bunker_base`/`bunker_bringup`/`bunker_msgs`), `ugv_sdk` (C++ CAN layer the driver
  links against). All three are **ROS1 Noetic + CAN**.
- **One ROS graph, two backends.** The *same* detector + behaviour nodes run in both
  modes. Only two things swap: the **camera source** (Gazebo camera plugin ↔ Pi
  Camera) and the **velocity sink** (Gazebo diff/skid-steer plugin ↔ `bunker_base`
  over CAN). A single `platform: sim | real` key flips both.
- **Two perception backends behind one topic seam.** A **CNN** detector (YOLOv8-nano
  trained on a Roboflow dataset, exported for the Pi 5) is the headline; a **classic
  HSV** detector (the existing v0 exercise loop) is kept as a selectable baseline for
  the CNN-vs-classic-CV teaching moment. Both publish the *same* detection topic, so
  the behaviour node is detector-agnostic.
- **Space-mining behaviour** = a small state machine: `SEARCH → APPROACH (green ball)
  → AVOID (cone) → STOP/COLLECT`, emitting `/cmd_vel`.
- **Everything configurable** through one `config/minibunker.yaml`; a **Streamlit UI**
  shows the live annotated camera + telemetry and live-tunes the knobs (thresholds,
  gains, speed caps, detector backend, platform).
- Ship as a **new private GitHub repo** `billisandr/minibunker-spacemine-workshop`
  (local + online), Docker-based like the existing `ros_z1_*` stations.
- **Supersedes** `ExerciseHandouts/MiniB2_rover/Exercise_MiniBunker2_v0.pdf` (the HSV
  color-chase) — that loop survives as the classic-CV baseline inside this station.

---

## 1. Source material (what already exists)

### 1.1 The three AgileX upstream repos

| Repo | Role | What we use |
| --- | --- | --- |
| [`agilexrobotics/ugv_gazebo_sim`](https://github.com/agilexrobotics/ugv_gazebo_sim) | Gazebo Classic models for AgileX chassis (Scout, Hunter, Bunker, Tracer, Ranger, Limo…) | The **Bunker** model/URDF + a sim launch as the base of our sim world |
| [`agilexrobotics/bunker_ros`](https://github.com/agilexrobotics/bunker_ros) | ROS1 driver for the real robot; packages `bunker_base`, `bunker_bringup`, `bunker_msgs` | The **real-robot driver**: `/cmd_vel` in, `/odom` out, over CAN `can0`. Explicitly supports **Bunker Mini** |
| [`agilexrobotics/ugv_sdk`](https://github.com/agilexrobotics/ugv_sdk) | C++ CAN protocol library (Weston Robot + AgileX); ProtocolV2 | Build-time dependency of `bunker_base`; no direct edits |

**Confirmed from upstream:**
- All three are **ROS1** (Noetic-era) and talk to hardware over **CAN bus** (default
  `can0`, via a CAN-to-USB adapter or a Pi CAN HAT). `ugv_sdk` lists **CAN-only** for
  active robots; UART is not active for Bunker.
- `ugv_gazebo_sim` **includes a Bunker model** (tracked / skid-steer). It does **not**
  ship a Bunker *Mini* sim by default — confirm whether the Bunker model is close
  enough or if the Mini needs a scaled URDF (see §15 open question Q2).
- `bunker_ros` start command for the real base:
  `roslaunch bunker_bringup bunker_robot_base.launch`. Publishes `/odom`
  (param `odom_topic_name`), subscribes `/cmd_vel`.

> **ROS version decision is load-bearing** — see §4.1. Default is **ROS1 Noetic in
> Docker**, matching both the upstream repos and every existing `ros_z1_*` station in
> this workspace.

### 1.2 The existing MiniB2 exercise (what we supersede)

`2.CodeRepos/ExerciseHandouts/MiniB2_rover/Exercise_MiniBunker2_v0.pdf` is a ~10-min
*coder* mini-exercise: a 25-line `bunker_follow.py` that does **HSV color chasing** —
`cv2.inRange` mask → moments → centroid error → `set_speed(left, right)` via a lab
helper `bunker_motors.set_speed`. Knobs: `LOWER`/`UPPER` HSV bounds, `GAIN`, area
threshold. Objects used: blue ball, red cone, yellow cube.

**How it relates to this plan (resolved):** the new workshop **supersedes** v0 as the
station's primary deliverable, but **reuses v0's HSV loop verbatim** as the
`detector.backend: hsv` baseline (§4.3). This gives the workshop its core pedagogical
contrast: *classic color thresholding* vs *a trained CNN*, on the same robot, same
arena, same behaviour code. The lab helper `set_speed(left,right)` maps onto our
`/cmd_vel` publisher (skid-steer mixing) so the v0 code drops in with minimal change.

### 1.3 Workspace conventions to honour (read these, don't re-derive)

- **Plan style:** model this document on `2.CodeRepos/ros_z1_teleop/PLAN.md` (phased
  roadmap, decisions section, risk table) — that's the format the user expects.
- **Docker + Noetic pattern:** every `ros_z1_*` station runs ROS1 Noetic in a
  container with a one-shot `start_*.sh`; mirror that. See
  `[[project_ros_z1_zed_docker]]` for the Windows/Docker gotchas (`-it` terminal
  contention, `MSYS_NO_PATHCONV`, CRLF in CMakeLists, device passthrough) that will
  recur here.
- **Handouts / instructor cards:** when we reach docs (§14), follow
  `2.CodeRepos/ExerciseHandouts/HANDOUT_STYLE_GUIDE.md` and
  `INSTRUCTOR_CARDS_STYLE_GUIDE.md` exactly (maroon/gold palette, A4, `Takeaway:`
  label, versioned `*_vN.html`, don't over-iterate screenshot QA). See
  `[[feedback_handout_card_decks]]`.
- **Station tracking:** this station is row #3 in
  `Summer_School_Station_Plan.md` ("MiniB2 spacemine rover — placeholder"). Update
  that tracker and `Workshop_Repos_Overview.md` once content lands.

---

## 2. Mission & non-goals

**Mission.** A Space-Summer-School station where a MiniBunker rover demonstrates a full
**sense → think → act** loop: a Pi camera + CNN spots a green "ore" ball and
construction-cone hazards, and the rover autonomously approaches the ore while avoiding
cones — shown first in **Gazebo** (zero hardware, always works), then on the **real
MiniBunker 2.0**, with a **Streamlit** panel for non-coders to watch and tune.

**In scope (v1)**
- New repo `minibunker-spacemine-workshop`, local + GitHub (private, `billisandr`).
- Gazebo sim of the Bunker + green ball + cones in a "spacemine arena."
- Real-robot bringup on Pi 5 (camera + CAN to `bunker_base`).
- Two detector backends (CNN default, HSV baseline) behind one topic.
- Space-mining behaviour state machine → `/cmd_vel`.
- One master YAML + a Streamlit UI (live view + knobs + start/stop + mode switch).
- Roboflow → YOLOv8n training pipeline, exported to run on the Pi 5.
- Docs: README/QUICKSTART, participant handout (v1 HTML), instructor cards (v1 HTML).

**Non-goals (explicitly out for v1)**
- No SLAM / mapping / global navigation. Reactive behaviour only (image-frame steering),
  like v0. (Cone *avoidance* is reactive, not planned.)
- No metric depth / 3D pose of the ball. Distance is **estimated from apparent size**
  (bounding-box height) and used only for a stop threshold — no calibration required.
- No GPU requirement. The CNN runs on the Pi 5 CPU (NCNN/ONNX) or an optional AI HAT;
  training is done off-robot.
- No multi-robot, no manipulator, no autonomy beyond the arena.
- No real-robot autonomous run **without** an instructor and a hardware e-stop.

---

## 3. Target architecture

```
                         ┌─────────────── camera source (swappable) ───────────────┐
   SIM  : Gazebo camera plugin ─►/camera/image_raw      REAL: pi_camera_node ─►/camera/image_raw
                         └──────────────────────────┬───────────────────────────────┘
                                                     │  sensor_msgs/Image
                                                     ▼
                                   detector_node   (backend: cnn | hsv)
                                     ├─► /minibunker/detections   (vision_msgs/Detection2DArray)
                                     ├─► /minibunker/debug_image   (sensor_msgs/Image, annotated)
                                     └─► /minibunker/perception_state (custom: target seen? cone seen?)
                                                     │
                                                     ▼
                                   behavior_node   (SEARCH→APPROACH→AVOID→STOP)
                                     └─► /cmd_vel   (geometry_msgs/Twist)
                                                     │
                         ┌──────────────────────────┴───────────────────────────────┐
   SIM  : Gazebo skid-steer/diff plugin            REAL: bunker_base ─► CAN can0 ─► motors
          (consumes /cmd_vel, emits /odom)               (consumes /cmd_vel, emits /odom)
                         └────────────────────────────────────────────────────────────┘

   Streamlit UI  ◄──► rosbridge/roslibpy  ◄──►  rosparam + /minibunker/debug_image + /odom
     (live annotated camera, telemetry, knob panel writing config live, start/stop, platform switch)
```

### 3.1 The two seams that make sim == real

| Seam | SIM provides | REAL provides | Topic (identical both sides) |
| --- | --- | --- | --- |
| **Camera** | Gazebo `libgazebo_ros_camera` plugin on a camera link added to the Bunker model | `pi_camera_node` (libcamera/picamera2 → cv2 → publish) | `/camera/image_raw` (+ `/camera/camera_info`) |
| **Drive** | Gazebo skid-steer/diff-drive plugin (already in `ugv_gazebo_sim` Bunker model) | `bunker_base` over CAN `can0` | `/cmd_vel` in, `/odom` out |

Because `detector_node` and `behavior_node` only ever touch `/camera/image_raw`,
`/minibunker/detections`, and `/cmd_vel`, **they are written once and never change
between sim and real.** This is the same "swappable seam" strategy proven in
`ros_z1_sim_marker-real-camera` (camera-agnostic ArUco node).

### 3.2 Nodes (all new, in our packages)

| Node | Package | Role |
| --- | --- | --- |
| `pi_camera_node` | `minibunker_perception` | REAL only: Pi Camera → `/camera/image_raw` |
| `detector_node` | `minibunker_perception` | CNN **or** HSV (config switch) → detections + debug image |
| `behavior_node` | `minibunker_behavior` | State machine → `/cmd_vel` |
| `image_pub` (optional) | `minibunker_perception` | Publish a bundled demo `.mp4`/webcam to `/camera/image_raw` for hardware-free dev |
| Streamlit app | `minibunker_ui` | Dashboard + knob panel (not a ROS node; talks via rosbridge) |

### 3.3 Topic / message summary

| Topic | Type | Notes |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/Image` | The seam; one source at a time |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Optional; not required by the reactive behaviour |
| `/minibunker/detections` | `vision_msgs/Detection2DArray` | Class id (green_ball / cone), bbox, score |
| `/minibunker/debug_image` | `sensor_msgs/Image` | Annotated frame for Streamlit/RViz |
| `/minibunker/perception_state` | `std_msgs/Float32MultiArray` | **Role-based 7-slot contract** — `target_*` = the followed class, `hazard_*` = nearest hazard (full layout in §20.3). *Originally ball-hardwired; generalised when missions landed (§18).* |
| `/cmd_vel` | `geometry_msgs/Twist` | Behaviour output → both drive backends |
| `/odom` | `nav_msgs/Odometry` | From whichever drive backend is active |

---

## 4. Core design decisions

### 4.1 ROS distro & runtime — **ROS1 Noetic in Docker (default)**
- **Why Noetic:** `ugv_gazebo_sim` and `bunker_ros` are ROS1; Gazebo Classic; and every
  existing station in `2.CodeRepos` is Noetic-in-Docker. Maximum reuse, minimum
  surprise.
- **Why Docker:** reproducibility + the workspace already standardises on it. ARM64
  `osrf/ros:noetic-*` images exist, so the *same* Dockerfile philosophy runs on an
  x86 dev laptop (sim) and on the Pi 5 (real).
- **Alternative considered — ROS2 (Humble/Jazzy):** AgileX ships `bunker_ros2` and
  `ugv_sdk` builds for ROS2; ROS2 is the long-term path and arm64-native on the Pi.
  **Rejected for v1** because `ugv_gazebo_sim` (the sim base) is ROS1, so going ROS2
  would mean rebuilding the sim. Revisit post-v1 if the lab standardises on ROS2.
- **Alternative considered — no-ROS native Python on the Pi (real only):** a single
  Python loop (picamera2 → detector → CAN via `python-can`) like v0's `set_speed`
  helper. Lighter to run live, but **breaks sim↔real parity** (sim has no such path)
  and re-implements drive mixing. Keep as a documented fallback (§9.4), not the
  default. **Decision flagged for the user — see §15 Q1.**

### 4.2 Robot platform — Bunker Mini 2.0 (tracked, skid-steer)
- Differential/skid-steer kinematics. `behavior_node` emits a `Twist` (linear.x,
  angular.z); the drive backend mixes to per-track speeds. The v0 `set_speed(left,
  right)` mixing becomes a tiny `twist_to_tracks()` helper for the HSV-baseline path,
  but the canonical interface is `/cmd_vel` so sim and the real `bunker_base` both work
  unchanged.
- Real connection: **CAN `can0`** via a CAN-to-USB adapter or a Pi CAN HAT (§9.2).

### 4.3 Perception — two backends behind one topic
- **`detector.backend: cnn` (default, headline).** YOLOv8-nano, 2 classes
  (`green_ball`, `cone`), trained on a Roboflow dataset (§10), exported to **NCNN or
  ONNX** for Pi 5 CPU inference (optionally Hailo-8L if an AI HAT is present). Publishes
  `vision_msgs/Detection2DArray`.
- **`detector.backend: hsv` (baseline).** The v0 loop, generalised to two colours:
  green-ball HSV range + cone (orange) HSV range → contours → bboxes → the *same*
  `Detection2DArray`. Zero training, runs anywhere, and is the deliberate teaching
  contrast.
- **Same output contract** for both, so `behavior_node` never knows which ran. Switch
  is a single YAML key, live-flippable from the Streamlit UI.
- **Distance proxy:** estimate range to the green ball from bbox **apparent size**
  (height in px ∝ 1/distance, calibrated once with a tape measure). Drives the
  "close enough → STOP/COLLECT" threshold. No camera calibration / depth sensor needed.

### 4.4 Behaviour — reactive space-mining state machine
```
        no target, no cone
SEARCH ───────────────────────► (rotate slowly, scan)
  │  green ball detected
  ▼
APPROACH ──► steer toward ball centroid (P-control on image-x error, like v0 GAIN),
  │          drive forward; ball bbox big enough ──► COLLECT
  │  cone enters danger zone (bbox in lower-centre / too close)
  ▼
AVOID ────► back off / turn away from cone until clear ──► back to SEARCH/APPROACH
  │  ball lost for N frames
  ▼
STOP (zero Twist; resume to SEARCH when something reappears)
```
- All thresholds/gains are YAML (§5). Cone avoidance is **reactive** (image-frame), not
  planned — matches v1 non-goals.
- **Safety convention:** `behavior_node` boots in `STOP` and publishes zero `Twist`
  until the UI/operator explicitly **arms** it. The robot never lurches on launch
  (same posture as the z1 stations' "start frozen").

> **Implemented (current).** The FSM is now **role-based** — it follows whatever
> `mission/follow_item` selects (ball *or* cone), not a hardwired ball (§18) — and
> gained a **COLLECT → RETREAT** cycle: on reaching the target it stops, pauses
> (`behavior/collect/pause_sec`), turns a random way and drives off before resuming
> SEARCH, so it doesn't re-collect the same item. When `follow_item: none` it enters a
> **TELEOP** state and passes WASD intent through the same ARM gate (§18.2). The live
> 7-slot perception contract is §20.3.

### 4.5 Sim vs real switch — one key, two launch files
- `platform: sim` → `minibunker_sim.launch`: Gazebo world (arena + ball + cones) +
  Bunker model w/ camera plugin + detector + behavior + (optional) rosbridge.
- `platform: real` → `minibunker_real.launch`: `bunker_bringup` + `pi_camera_node` +
  detector + behavior + rosbridge. (`use_sim_time:=false`.)
- The YAML `platform` key is the single source of truth; the Streamlit "Sim/Real"
  toggle just selects which launch the start script runs. Detector + behaviour configs
  are shared across both.

### 4.6 Configuration — one master YAML, loaded to `rosparam`
A single `config/minibunker.yaml` (§5) is the only file a workshop host edits. The
launch files load it into the param server; nodes read params on start; the Streamlit
UI rewrites a live subset and pokes `dynamic_reconfigure`-style updates (or re-reads on
a "Apply" button) so non-coders never touch code.

### 4.7 Streamlit UI — dashboard + knob panel (non-coder front door)
Mirrors the `g1-sim2sim-workshop` live-knobs pattern. Shows the annotated camera, the
current state, `/odom` speed, and detection count; lets a participant drag knobs
(HSV ranges, CNN confidence, follow gain, speed cap, stop distance) and flip
backend/platform. Integration via **rosbridge + roslibpy** (or a thin shared-state
file) — see §6 for the two options and the recommendation.

---

## 5. Master configuration schema (`config/minibunker.yaml`)

The single file that drives everything. Proposed structure (final keys settle during
Phase 2–4):

```yaml
platform: sim                 # sim | real   (§4.5)

camera:
  source: gazebo              # gazebo | picamera | video:/path.mp4 | webcam
  width: 640
  height: 480
  fps: 30
  flip_horizontal: false

detector:
  backend: cnn                # cnn | hsv     (§4.3)
  cnn:
    weights: models/minibunker_yolov8n.onnx   # or .ncnn / .pt
    runtime: onnxruntime      # onnxruntime | ncnn | ultralytics | hailo
    conf_threshold: 0.45
    iou_threshold: 0.50
    class_names: [green_ball, cone]
    input_size: 416
  hsv:                        # the v0 baseline, two colours
    green_ball:
      lower: [40, 80, 40]
      upper: [85, 255, 255]
      min_area: 3000
    cone:
      lower: [5, 120, 80]     # orange
      upper: [25, 255, 255]
      min_area: 4000

behavior:
  arm_on_start: false         # safety: boot in STOP (§4.4)
  search:
    scan_angular_speed: 0.5   # rad/s rotate while searching
  approach:
    steer_gain: 0.003         # P-gain on image-x error (v0 GAIN)
    forward_speed: 0.25       # m/s
    collect_bbox_frac: 0.45   # ball bbox height / frame height to trigger COLLECT
  avoid:
    cone_danger_frac: 0.35    # cone bbox in lower-centre bigger than this => AVOID
    backoff_speed: -0.15
    turn_speed: 0.6
  limits:
    max_linear: 0.4           # hard speed caps (m/s)
    max_angular: 1.0          # rad/s
    lost_frames: 15           # frames without target before STOP

drive:
  cmd_vel_topic: /cmd_vel
  track_width: 0.4            # for twist->tracks mixing (HSV/v0 path)

ui:
  bridge: rosbridge          # rosbridge | sharedfile
  rosbridge_port: 9090
  refresh_hz: 10
```

### 5.1 Schema additions (post-v1)

Blocks added to the same `config/minibunker.yaml` as missions/teleop/arena landed
(see §18–§20). `detector/backend: hsv|cnn` above still works; `perception:` is the
forward-looking superset for §19. The `arena:` values below are illustrative — the
authoritative set lives in the repo's `minibunker.yaml`.

```yaml
mission:
  follow_item: none          # none | ball | cone   (none => manual WASD teleop, §18.2)
  hazard_items: [cone]       # classes to AVOID; the followed class is auto-excluded

behavior:
  collect:
    pause_sec: 5.0           # COLLECT dwell before RETREAT (§4.4)
  teleop:                    # WASD intent passthrough; inherits behavior/limits + watchdog

perception:                  # roadmap (§19) — pluggable stages; NOT yet implemented
  detector: hsv              # hsv | cnn | locate_anything
  segmenter: none            # none | sam
  depth: none                # none | depth_anything   (replaces target_h_frac proxy)
  open_vocab_prompts: ["green ball", "construction cone"]
  device: cpu                # cpu | cuda | tensorrt | hailo
  max_infer_hz: 5            # throttle heavy models off the camera/control loop

arena:                       # single source of truth for every sim object (§20.2)
  objects:
    floor:    { type: plane,     mu: 1.0 }
    sun:      { type: light }
    ore_ball: { type: sphere,    pose: [...], static: false, mass: 0.2, mu: 0.4, kp: 1e5, kd: 10 }
    cone_1:   { type: cone_mesh, pose: [...], scale: 3.0, static: false, mass: 0.5, mu: 0.6, kp: 1e5, kd: 10 }
    # cone_2, cone_3, fence ...   (dynamic props: inertia auto-computed from type+size+mass)
```

---

## 6. Streamlit UI specification

**Goal:** a non-coder can watch the rover think and tune it without opening code.

**Panels**
1. **Live view** — the `/minibunker/debug_image` (annotated frame: boxes, class labels,
   current state badge). Plus a small mono "what the rover sees" mask view when in HSV
   mode (the v0 black-and-white window, recreated in the browser).
2. **Telemetry** — current state (SEARCH/APPROACH/AVOID/STOP), linear/angular speed from
   `/odom`, detections-this-frame, FPS.
3. **Knob panel** — sliders/inputs bound to the YAML subset: detector backend toggle,
   CNN confidence, HSV ranges (per colour), follow gain, forward speed, stop bbox
   fraction, cone danger fraction, speed caps. An **Apply** button pushes to
   `rosparam`/dynamic-reconfigure.
4. **Controls** — **ARM / DISARM** (the safety gate), **Sim ↔ Real** toggle (selects
   the launch), **Start / Stop** the stack.

**Integration options**
- **(A, recommended) rosbridge + roslibpy/`roslib`.** Run `rosbridge_server` in the
  container; Streamlit (Python `roslibpy`) subscribes to `debug_image`/`odom`, calls a
  small ROS service to set params and ARM/DISARM. Clean, ROS-native, works sim+real.
- **(B, fallback) shared-state file + image dump.** Nodes write the latest annotated
  JPEG + a small status JSON to a tmpfs path; Streamlit polls them and writes the YAML
  back; a node watches the file. Simpler, no rosbridge, but laggier and one-way-ish.

Default to **(A)**; keep **(B)** documented for low-resource Pi runs.

---

## 7. Repository layout

```
minibunker-spacemine-workshop/
├── plan.md                       ← this file
├── README.md                     ← short: what/why + pointer to plan.md + quickstart
├── .gitignore                    ← Python/ROS/Docker artifacts, models/*.pt, data/
├── .dockerignore
├── docker/
│   ├── Dockerfile                ← ROS Noetic + sim deps + perception deps (arm64+amd64)
│   └── entrypoint.sh
├── start_sim.sh                  ← one-shot: build+run sim (dev laptop)
├── start_real.sh                 ← one-shot: bringup on the Pi 5
├── config/
│   └── minibunker.yaml           ← the master config (§5)
├── catkin_ws/src/
│   ├── minibunker_perception/    ← pi_camera_node, detector_node, image_pub
│   ├── minibunker_behavior/      ← behavior_node (state machine)
│   ├── minibunker_bringup/       ← launch files, worlds, rviz, params loader
│   └── minibunker_ui/            ← streamlit app + rosbridge launch
├── models/                       ← exported CNN weights (gitignored; fetched/released)
├── training/                     ← Roboflow download + YOLOv8 train/export scripts + notebook
├── sim/
│   ├── worlds/spacemine_arena.world
│   └── models/{green_ball, construction_cone}/   ← SDF/meshes
├── media/                        ← bundled demo .mp4 (hand-free dev), photos
├── docs/
│   ├── QUICKSTART.md
│   ├── HARDWARE_SETUP.md         ← Pi 5 + camera + CAN HAT wiring
│   ├── TRAINING.md               ← Roboflow→YOLOv8→export→deploy
│   └── ARENA.md                  ← physical arena + object spec
└── workshop/                     ← (docs phase) handout + instructor cards (HTML/PDF)
```

Vendored as **submodules** under `catkin_ws/src/`: `ugv_gazebo_sim`, `bunker_ros`,
`ugv_sdk` (pinned — §12).

---

## 8. Detailed component plan

### 8.1 `minibunker_perception/detector_node` (the heart)
- Read `detector.*`, `camera.*` params. Subscribe `/camera/image_raw`.
- If `backend == cnn`: load weights via the configured `runtime` (onnxruntime / ncnn /
  ultralytics / hailo); per frame → boxes/scores/classes; filter by `conf_threshold`.
- If `backend == hsv`: two `cv2.inRange` masks (green, orange) → contours → bboxes;
  reuse v0's moments/area logic per colour.
- Publish `vision_msgs/Detection2DArray`, an annotated `/minibunker/debug_image`, and a
  small `perception_state` (target centroid x-normalised, target bbox-height fraction,
  cone-in-danger bool).
- **Acceptance:** on the demo `.mp4`, both backends publish sensible detections; the
  debug image shows boxes + labels; switching `backend` at runtime works.

### 8.2 `minibunker_behavior/behavior_node`
- Subscribe `perception_state` (+ `detections`). Run the §4.4 state machine. Publish
  `/cmd_vel`, clamped to `behavior.limits`.
- Boots **DISARMED → STOP**; an `arm` service/flag gates motion (§4.4 safety).
- P-control steering = v0's `turn = GAIN * error`; forward speed and avoidance from YAML.
- **Acceptance:** in sim, the Bunker rotates to find the ball, drives to it, stops when
  close; backs/turns away when a cone fills the danger zone; freezes when the ball is
  lost.

### 8.3 `minibunker_perception/pi_camera_node` (REAL only)
- libcamera/`picamera2` → numpy BGR → `cv_bridge` → `/camera/image_raw` at
  `camera.fps`. Optional flip. **Acceptance:** `rqt_image_view` shows the Pi camera in
  the same topic the sim publishes.

### 8.4 `minibunker_bringup` — launch + world + rviz
- `minibunker_sim.launch`: Gazebo (`spacemine_arena.world`) + Bunker model **with an
  added camera link + `libgazebo_ros_camera` plugin** publishing `/camera/image_raw` +
  the stock skid-steer/diff plugin (`/cmd_vel`→motion, `/odom`) + detector + behavior +
  rosbridge. Params from `config/minibunker.yaml`.
- `minibunker_real.launch`: `bunker_bringup` base + `pi_camera_node` + detector +
  behavior + rosbridge. `use_sim_time:=false`.
- `params.launch`: loads the master YAML into `rosparam`.
- RViz config: image display on `/minibunker/debug_image`, robot model, `/odom`.

### 8.5 Sim assets (`sim/`)
- `spacemine_arena.world`: a fenced arena floor matching the real arena footprint.
- **green_ball** model: a green sphere SDF (radius ≈ real ball).
- **construction_cone** model: reuse Gazebo's stock *Construction Cone* model (in the
  OSRF model DB) or a simple orange cone mesh; place 2–4 in the arena.
- The Bunker model needs a **camera link** added (it likely ships without one) —
  a small xacro/SDF edit; this is the only model surgery required.

### 8.6 HSV-baseline drop-in
Port v0's `bunker_follow.py` into the `hsv` branch of `detector_node` (detection only)
+ the P-steer into `behavior_node`. Keep a standalone `reference/bunker_follow_v0.py`
for provenance and for the optional coder extension.

---

## 9. Hardware & Pi 5 setup + dev-machine reality check

### 9.1 Raspberry Pi 5 software stack
- **OS:** Raspberry Pi OS (64-bit, Bookworm) or Ubuntu 24.04 arm64.
- **ROS Noetic** runs via the **arm64 Docker image** (Noetic isn't native on Bookworm).
  **Risk:** Pi Camera (libcamera) + CAN inside Docker need care (§9.2, §9.3, §11).
- **Camera:** Raspberry Pi Camera Module 3 (or v2) via **libcamera/`picamera2`**.

### 9.2 CAN to the Bunker
- Bunker speaks **CAN**. On the Pi use either a **USB-CAN adapter** (`slcan`/`gs_usb`)
  or a **CAN HAT** (e.g. Waveshare 2-CH CAN HAT, MCP2515/MCP2518). Bring up `can0`
  (`ip link set can0 up type can bitrate 500000`) on the host, then pass it into the
  container with **`--network host`** (CAN is a netdev) + `--cap-add NET_ADMIN`.
- `bunker_base` then runs against `can0` exactly as upstream documents.

### 9.3 Pi Camera in Docker
- Easiest: run `pi_camera_node` such that the container can reach the camera. Options:
  (a) mount `/dev/video*` + `/run/udev` and use the libcamera V4L2 path; (b) run the
  camera node **on the host** Python and publish into the container's ROS master via
  `ROS_MASTER_URI`/`ROS_IP`. Document whichever proves reliable on the actual Pi.
- **Fallback for dev/bring-up:** `camera.source: video:/media/demo.mp4` — a bundled
  clip of a green ball + cones, so the whole stack runs with **zero camera hardware**.

### 9.4 Documented no-ROS fallback (real robot only)
If Noetic-in-Docker on the Pi proves too heavy live, a **native Python** loop
(picamera2 → detector → `python-can` to the Bunker, using v0's mixing) can run the real
station. It loses sim parity and re-implements drive — keep it as `reference/` + a
section in `HARDWARE_SETUP.md`, not the default. (See §15 Q1.)

### 9.5 Dev machine (Windows) reality check
- The sim (`start_sim.sh`) runs on the **x86 dev laptop** in Docker, exactly like the
  `ros_z1_*` sims — expect the same Windows/Docker gotchas from
  `[[project_ros_z1_zed_docker]]`: detached-container + foreground-RViz to dodge `-it`
  contention, `export MSYS_NO_PATHCONV=1` for device/path args, CRLF normalisation on
  any vendored CMakeLists, full rebuild after edits (no bind-mount surprises).
- No real Bunker on Windows; the real path is Pi-only.

---

## 10. CNN training pipeline (off-robot)

`training/` + `docs/TRAINING.md`. Reproducible, runs on Colab or any CUDA box.

1. **Dataset (Roboflow).** Search **Roboflow Universe** for existing sets — terms:
   `traffic cone` / `construction cone`, and `green ball` / `tennis ball` / `sports
   ball`. Either (a) merge two single-class public datasets into a 2-class set, or
   (b) capture ~150–300 arena photos with the actual ball + cones and annotate in
   Roboflow (best domain match — the real arena lighting/background). Export in
   **YOLOv8** format; pull with the `roboflow` pip package + API key (key kept out of
   git).
2. **Train.** `ultralytics` YOLOv8-**nano** (`yolov8n.pt` start), 2 classes, ~416 input,
   light augmentation. Target a model that runs **≥10 FPS on the Pi 5 CPU**.
3. **Validate.** mAP + a visual check on held-out arena frames; confirm green ball vs
   cone aren't confused under arena lighting.
4. **Export for the Pi.** `model.export(format='ncnn')` (best CPU on ARM) and/or
   `onnx`; optionally compile for **Hailo-8L** if the AI HAT is used. Drop into
   `models/`, point `detector.cnn.weights` at it.
5. **Release, don't commit weights.** `.gitignore` the binaries; attach to a GitHub
   release or fetch via a `models/download.sh`.

**Pedagogical note for docs:** this pipeline (annotate → train → export → deploy) *is*
the workshop's "how a robot learns to see" story; the HSV baseline is the "before."

---

## 11. Docker & dependency integration (highest-risk area)

- Base `osrf/ros:noetic-desktop-full` (dev/sim) and a slimmer `ros:noetic-ros-base`
  (Pi/real). Build `ugv_sdk` + `bunker_base` from the submodules; `catkin_make` the
  workspace.
- **Perception deps:** `opencv-python`/system `python3-opencv` (must stay compatible
  with `cv_bridge`), `numpy`, plus the chosen CNN runtime (`onnxruntime` /
  `ncnn`/`ultralytics`). **Risk:** the MediaPipe-style opencv/protobuf clashes seen in
  `ros_z1_teleop` — add an in-build **import smoke test**
  (`python3 -c "import cv2, numpy, onnxruntime; from cv_bridge import CvBridge"`) that
  fails the build loudly.
- **Multi-arch:** the same Dockerfile must build for **arm64** (Pi) and **amd64**
  (laptop). Use `--platform`/buildx; pick CNN-runtime wheels that have arm64 builds
  (onnxruntime + ncnn do; ultralytics works but is heavier on the Pi — prefer
  ncnn/onnx at runtime on the Pi).
- **CAN + camera passthrough** flags live in `start_real.sh` (`--network host`,
  `--cap-add NET_ADMIN`, device mounts) — §9.2/§9.3.
- Carry over any **CMakeLists CRLF/`sed` fixes** the vendored submodules need (same
  class of fix as `z1_controller` in the z1 repos — verify per submodule).
- **Acceptance (Phase 1):** image builds on both arches; smoke test passes;
  `roslaunch minibunker_bringup minibunker_sim.launch` brings up Gazebo + nodes with no
  import errors.

---

## 12. Git & GitHub setup

`minibunker-spacemine-workshop/` is **not yet** a git repo. This session does steps 1–5;
submodules/scaffolding (6–7) happen when implementation starts.

1. `git init` in the target folder.
2. Add `.gitignore` (Python `__pycache__/`, `build/ devel/ install/`, `.venv/`,
   `models/*.pt models/*.onnx models/*.ncnn*`, `data/ training/runs/`, `*.mp4` large
   media optional) and `.dockerignore`.
3. Add `plan.md` (this file) + a short `README.md` pointing at it.
4. First commit.
5. Create the **private** remote under `billisandr` and push:
   ```
   gh repo create billisandr/minibunker-spacemine-workshop \
     --private --source . --remote origin --push
   ```
6. **(DONE 2026-06-24)** added the three submodules under `catkin_ws/src/`, pinned to
   these upstream SHAs (recorded like `ros_z1_teleop` §12b):
   ```
   ugv_gazebo_sim  27633a956c845903ee630538afeb17fe70afdd84   (default branch)
   bunker_ros      6ae0a1da92a0cdfb5f679e1cf2c0ad63e75a36d1   (master)
   ugv_sdk         c3dfaf444f9bae10757e546acae055aaf4a13de7   (main)
   ```
   Notable finds: `ugv_gazebo_sim` ships **both** a `bunker_gazebo_sim` (with a
   `libgazebo_ros_planar_move` drive plugin: `/cmd_vel`→motion, `/odom`) **and** a
   `bunker_mini/` URDF (partly answers §15 Q2 — a Mini description exists).
   `bunker_bringup/launch/bunker_robot_base.launch` has an `is_bunker_mini` arg, used
   by `minibunker_real.launch`.
7. **(DONE 2026-06-24)** scaffolded the four `minibunker_*` packages + Docker + configs;
   implemented Phases 1–6 (perception/behaviour/bringup/ui code, sim world + camera
   xacro, master YAML, Streamlit UI, training scripts). Pending: image build + hardware
   validation, and Phase 7 styled handout/instructor cards.

GitHub owner/visibility = **`billisandr`, PRIVATE** (matches the other stations).

---

## 13. Phased implementation roadmap

Each phase ends with a concrete, runnable acceptance check. Do them in order.

> **Status (2026-06-29).** This original build roadmap is largely complete: **Phases
> 0–5 + the sim are done and hands-on validated**. **Phase 6 (real robot)** is the
> outstanding **Phase C** and **Phase 7 (docs)** is the partial **Phase H** in the
> current enhancement roadmap — see §17.2 and §21. Beyond this list, missions, WASD
> teleop and the dynamic arena were added later (§17–§20).

- **Phase 0 — Plan & repo (THIS SESSION).** Write `plan.md`; `git init`; `.gitignore`;
  `README.md`; first commit; create private GitHub repo and push.
  *Accept:* repo exists locally and at `github.com/billisandr/minibunker-spacemine-workshop`,
  contains `plan.md`, branch pushed.

- **Phase 1 — Docker + submodules + sim brings up bare.** Add submodules; Dockerfile
  (multi-arch); build `ugv_sdk`+`bunker_base`; smoke test; bare Gazebo Bunker world.
  *Accept:* image builds; `minibunker_sim.launch` shows the Bunker in Gazebo; smoke test green.

- **Phase 2 — Detector (HSV first, then CNN).** `detector_node` with HSV baseline on the
  demo `.mp4`; then wire the CNN runtime + a placeholder/early-trained model.
  *Accept:* `/minibunker/detections` + `/minibunker/debug_image` sane for both backends;
  YAML backend switch works.

- **Phase 3 — Behaviour in sim.** `behavior_node` state machine → `/cmd_vel`; add camera
  plugin to the Bunker model; add ball + cones to the world.
  *Accept:* in Gazebo the rover searches, approaches the ball, avoids cones, stops when
  close; respects ARM/limits.

- **Phase 4 — Config + Streamlit UI.** Master YAML wired to rosparam; rosbridge; the
  Streamlit dashboard + knob panel + ARM/DISARM + sim/real toggle.
  *Accept:* a non-coder can watch the annotated feed and live-tune knobs; ARM gate works.

- **Phase 5 — Train the real CNN.** Roboflow dataset → YOLOv8n → export (ncnn/onnx);
  drop into `models/`; validate in sim.
  *Accept:* trained model detects green ball + cones on held-out frames at usable FPS.

- **Phase 6 — Real robot bring-up (Pi 5).** `pi_camera_node`; CAN `can0` + `bunker_base`;
  `minibunker_real.launch`; tune for arena lighting.
  *Accept:* on the real MiniBunker, the same stack drives the rover to the green ball and
  avoids cones, instructor-supervised, with e-stop.

- **Phase 7 — Docs.** README/QUICKSTART/HARDWARE_SETUP/TRAINING/ARENA; participant
  handout (v1 HTML per `HANDOUT_STYLE_GUIDE.md`); instructor cards (v1 HTML per
  `INSTRUCTOR_CARDS_STYLE_GUIDE.md`); update `Summer_School_Station_Plan.md` (row #3) and
  `Workshop_Repos_Overview.md`.
  *Accept:* a new host can reproduce sim + real; handout/cards match house style; station
  trackers updated.

---

## 14. Documentation deliverables (Phase 7 detail)

Follow the workspace style guides **exactly** (see `[[feedback_handout_card_decks]]`):

- **Participant handout** → `2.CodeRepos/ExerciseHandouts/MiniB2_rover/` as a new
  versioned `MiniBunker_SpaceMine_Handout_v1.html` (don't edit the v0 PDF in place;
  v0 stays as provenance). A4, maroon/gold palette, robot spec box (Bunker Mini 2.0
  real specs — verify against the datasheet, don't invent), goal box, sense→think→act
  diagram, knob-panel box (the Streamlit knobs), 3 teaching columns (HSV vs CNN; the
  state machine; sim vs real), safety box, standard footer.
- **Instructor cards** → `MiniBunker_SpaceMine_InstructorCards_v1.html`. Card kinds per
  the guide's checklist: Hook, Launch Sequence, the Knob Loop, Health Checks, one card
  per behaviour state, a **HSV-vs-CNN comparison card**, a "why train a CNN at all"
  cost card, the Bigger-Picture closer (reactive vision vs learned perception), Timing
  strip, Safety strip. `Takeaway:` label only.
- **Repo docs:** `README.md`, `docs/QUICKSTART.md`, `docs/HARDWARE_SETUP.md`,
  `docs/TRAINING.md`, `docs/ARENA.md`.
- Render handout/cards once or twice to sanity-check the densest/sparsest cards, then
  hand visual QA to the user — **don't** loop the screenshot/crop cycle.

---

## 15. Decisions

> **The current status of every decision/question below is tracked in §22** — several
> "Open" items are now resolved (branding, follow-class contract, hazard set) and the
> hardware questions are answered in `docs/HARDWARE_SETUP.md`.

**Resolved (this plan's defaults):**
- **D1.** ROS1 **Noetic in Docker**, sim + real share nodes (§4.1).
- **D2.** Two detector backends behind one topic; **CNN default, HSV baseline** (§4.3).
- **D3.** **Reactive** behaviour (no SLAM/nav); distance from bbox size (§2, §4.3, §4.4).
- **D4.** **YOLOv8-nano** via Roboflow, exported **ncnn/onnx** for the Pi (§10).
- **D5.** Streamlit via **rosbridge/roslibpy** (option A), sharedfile as fallback (§6).
- **D6.** GitHub: **`billisandr/minibunker-spacemine-workshop`, PRIVATE** (§12).
- **D7.** This station **supersedes** the v0 HSV PDF; v0 lives on as the baseline (§1.2).

**Open — flag to the user before/while implementing:**
- **Q1.** Real-robot runtime: **ROS-Noetic-in-Docker on the Pi** (parity, default) vs a
  **native-Python no-ROS loop** (lighter, live-friendly, but no sim parity)? (§4.1, §9.4)
- **Q2.** Sim platform: is `ugv_gazebo_sim`'s **Bunker** model close enough, or do we
  need a **scaled Bunker *Mini*** URDF? (§1.1)
- **Q3.** Pi 5 inference: **CPU (ncnn/onnx)** only, or is a **Hailo-8L AI HAT** available
  to target? Changes export + Docker deps (§10, §11).
- **Q4.** CAN interface on the Pi: **USB-CAN adapter** vs **CAN HAT** (which model)? Sets
  the bring-up commands in `HARDWARE_SETUP.md` (§9.2).
- **Q5.** Audience framing: primarily **non-coder** (Streamlit knobs) with an optional
  **coder** "edit the HSV/CNN" extension? (Station plan lists it as non-coder.) (§1.3)
- **Q6.** Dataset: reuse/merge **public Roboflow** sets vs **annotate real arena
  photos** (better domain match)? (§10)

---

## 16. Risks & mitigations

| Risk | Impact | Mitigation |
| --- | --- | --- |
| Noetic + libcamera + CAN all inside Docker on the Pi | Real robot won't bring up | Validate camera/CAN passthrough early (§9.2/9.3); native-Python fallback (§9.4); demo `.mp4` for hardware-free dev |
| CNN runtime ↔ `cv_bridge` opencv/protobuf clash | Build/import breaks | In-build import smoke test (§11), pin a known-good trio |
| CNN too slow on Pi 5 CPU | Laggy/unsafe steering | YOLOv8-**nano** + ncnn export + small input size; optional Hailo (§10); HSV baseline always available |
| `ugv_gazebo_sim` Bunker ≠ Bunker **Mini** | Sim mismatches real | Scale/adjust URDF; document as Q2; behaviour is reactive so exact kinematics aren't critical |
| Real robot lurches / runs into people | Safety | Boot DISARMED→STOP; speed caps in YAML; ARM gate in UI; hardware e-stop + fenced arena + instructor-only real runs (§4.4) |
| Green ball vs orange cone confused under arena light | Wrong behaviour | Train on **real arena** photos (Q6); tune HSV ranges live in UI; validate per §10.3 |
| Windows/Docker sim gotchas recur | Dev friction | Apply `[[project_ros_z1_zed_docker]]` fixes verbatim (detached container, `MSYS_NO_PATHCONV`, CRLF, rebuilds) (§9.5) |
| Submodule build (`ugv_sdk`/`bunker_base`) fails | No driver | Pin SHAs (§12); carry CRLF/`sed` fixes; build in Phase 1 before anything depends on it |

---

*End of the original design backbone (§0–§16). The sections below — §17 onward —
are the consolidated current status and forward plan, folded in from the former
`plan2.md` (2026-06-24) and `plan3.md` (2026-06-25).*

---

## 17. Current status & changelog

> The authoritative "where we are" section. §0–§16 describe the **design**; §17–§23
> describe **reality** as of the last working session (2026-06-25), consolidated
> 2026-06-29.

### 17.1 What works now (sim, hands-on validated)
- **Sim validated end-to-end** — not just "builds": `start_sim.sh` builds the Noetic
  Docker image and launches Gazebo `minibunker_arena.world` + Bunker (with our camera
  seam) + detector + behaviour + rosbridge + Streamlit. The rover
  **SEARCH→APPROACH→COLLECT→RETREAT** on the ore ball, **AVOID**s/knocks cones, and
  drives under **WASD**. Captured in the README Gallery clips.
- **One ROS graph, two seams** (§3.1): camera source + velocity sink swap sim↔real;
  detector + behaviour are identical. The topic contract is the anchor.
- **Two perception backends behind one contract**: `HsvDetector` + `CnnDetector`
  (YOLOv8-nano via onnxruntime/ultralytics/ncnn) in `detector_node.py`; backend re-read
  each frame so the UI flips it live. **HSV is the working default; the CNN is wired but
  untrained.**
- **Role-based reactive FSM** (§18, §20.3): `SEARCH → APPROACH → AVOID → COLLECT →
  RETREAT → (STOP/SEARCH)` in `behavior_node.py`. Boots DISARMED, clamps to
  `behavior/limits`, ARM gate via `/minibunker/arm`.
- **Missions live**: `mission/follow_item: none | ball | cone`, switchable from the UI
  with no relaunch; `none` is the boot default and yields no autonomous motion.
- **WASD teleop**: UI pad, physical browser keys, and a terminal `teleop_node.py` — all
  through the single `/cmd_vel` owner (§18).
- **Fully config-driven, dynamic arena**: every object (`floor`, `sun`, `ore_ball`,
  `cone_1..3`, `fence`) is one `arena:` block in `minibunker.yaml`; props are dynamic,
  knockable, and tunable (§20.2).
- **Fast edit→relaunch** via the dev bind-mount (§20.1): interpreted files go live with a
  plain `bash start_sim.sh`; only C++/msg changes need `--rebuild`.

### 17.2 Phase status (the enhancement roadmap of §21)
| Phase | Scope | Status |
| --- | --- | --- |
| R | Rename / de-brand (repo, image, world, dataset) | ✅ done (2026-06-24/25) |
| A | `follow_item` missions + UI selector (sim) | ✅ done |
| B | WASD teleop via behaviour pass-through + UI pad (sim) | ✅ done |
| C | Real-robot validation on the Pi 5 (CAN + Pi camera) | ❌ not started — needs hardware |
| D | Perception refactor to pluggable stages, HSV/CNN stay green | ❌ not started — gated by Q2 (§22) |
| E | Depth Anything stage (real distance, replaces bbox proxy) | ❌ not started — gated by Q3/Q4 |
| F | Open-vocab ("LocateAnything") + SAM stages + combos | ❌ not started — gated by Q2/Q3 |
| G | Pi-5 perf pass for D–F (edge variants, accel, offload) | ❌ not started — gated by Q3 |
| H | Docs: `PERCEPTION_MODELS.md` + QUICKSTART/HARDWARE updates | 🟡 partial — MISSIONS.md, TELEOP.md, ARENA.md done |

(The original build roadmap §13 — Phases 0–5 + sim — is complete; its Phase 6 maps to C
above and Phase 7 to H.)

### 17.3 Known not-done / rough edges
- **Real robot (Phase C):** never run through the ROS stack on hardware (CAN + Pi
  camera). A separate no-ROS `real_pi/` native path exists — see the §21 note.
- **CNN backend:** wired but untrained/unvalidated in sim; HSV is the working default.
- **Physics tuning is "good enough", still iterable.** If a cone seems stuck bobbing,
  first check whether the rover is driving *over* a short cone (~0.135 m at `scale 3.0`)
  rather than hitting its side — raise `cone_*/scale`. Floor `mu` combines with each
  prop's `mu` (ODE ≈ min), so a prop's low `mu` usually dominates.
- **`st.iframe` keyboard hack:** the WASD-key listener relies on same-origin
  `window.parent` inside the iframe — verified to render; confirm keys actually drive in
  the browser (fallback: `st.html(..., unsafe_allow_javascript=True)`).

### 17.4 Changelog
- **2026-06-23/24 — Phase 0 (design + scaffold).** Wrote the design plan (§0–§16);
  `git init`; private repo (under the old `minibunker-spacemine-workshop` name); added
  the three AgileX submodules pinned (§12); scaffolded the four `minibunker_*` packages +
  Docker; implemented Phases 1–6 (perception/behaviour/bringup/ui, sim world + camera
  xacro, master YAML, Streamlit UI, training scripts).
- **2026-06-24 — sim validated + full de-brand.** Sim came up clean end-to-end (HSV).
  Dropped "SpaceMine" everywhere (titles/UI, Docker image → `minibunker`, world →
  `minibunker_arena.world`, dataset → `minibunker`, clone URLs → `minibunker-workshop`).
- **2026-06-25 — missions, teleop, arena, dev-mount.** Delivered Phases R/A/B: GitHub
  rename to `billisandr/minibunker-workshop` + local dir move; role-based
  `perception_state` (option B); `follow_item` missions live in the UI; WASD teleop (UI
  pad + physical keys + terminal node) through the single `/cmd_vel` owner with watchdog.
  Beyond roadmap: COLLECT→RETREAT cycle; HSV cone-detection fix (close-then-open
  per-colour kernels); all arena props dynamic + physically tuned; single-source
  `arena:` block + generic `arena_setup.py` SDF builder; dev bind-mount in
  `start_sim.sh`; Streamlit deprecation fixes; README Gallery. Docs: MISSIONS.md,
  TELEOP.md, ARENA.md.

---

## 18. Missions, teleop & /cmd_vel arbitration (delivered)

**Status: implemented in sim (Phases A+B).** Generalises the originally ball-hardwired
behaviour of §4.4.

### 18.1 Missions — `follow_item`
- Config: `mission/follow_item: none | ball | cone`, `mission/hazard_items: [cone]`
  (§5.1). The user picks what the rover hunts; `none` is the default (no autonomous
  drive → teleop, §18.2). Settable in YAML **and** live from the UI selectbox with no
  relaunch (re-read each tick, same pattern as the detector backend).
- **Role-based `perception_state` (option B, implemented):** instead of hardwiring
  ball=target / cone=hazard, the detector reads `mission/follow_item` +
  `mission/hazard_items` each frame and packs **target_*** = the followed class and
  **hazard_*** = nearest hazard, auto-excluding the followed class from its own hazard
  set. The 7-slot layout is the contract in §20.3 — the same contract the §19 open-vocab
  backends want.
- The FSM gained a **COLLECT → RETREAT** cycle (§4.4) so it doesn't re-collect the same
  item.

### 18.2 WASD teleop (when `follow_item: none`)
- **Safety-critical design:** `behavior_node` stays the **single owner of `/cmd_vel`**.
  Teleop publishes *intent* on `/minibunker/teleop_cmd` (Twist), passed through the
  **same ARM gate + `behavior/limits` clamp**. No second publisher races on `/cmd_vel`;
  DISARM stays authoritative.
- Three input paths: the UI **WASD pad**, **physical W/A/S/D keys** in the browser
  (X = stop), and a terminal `teleop_node.py` (run via `teleop.sh`; needs a TTY, so it is
  **not** auto-launched).
- **Watchdog:** if no `teleop_cmd` arrives for N ms, command zero (don't latch motion).

### 18.3 /cmd_vel arbitration + ARM safety (do not regress)
- **Single `/cmd_vel` owner = behaviour_node.** Autonomous follow, teleop pass-through,
  and the DISARM zero-Twist all flow through it, so the ARM gate + `behavior/limits`
  clamp are unconditional.
- Boots **DISARMED**; DISARM publishes zero every tick and wins over any latched/teleop
  input.
- On the real robot, software caps are a backstop, **not** a substitute for the hardware
  e-stop.

---

## 19. Modern perception backends — roadmap (not started)

**Goal:** let the user opt into NVIDIA open-vocabulary localization, Meta SAM, and Depth
Anything — individually or combined — without breaking the one-topic contract or the
HSV/CNN baselines. The biggest remaining piece; its own mini-roadmap. **Gated by Q2–Q4
(§22).**

### 19.0 Verify upstreams first (do NOT assume)
Pin a real, licensed, edge-viable source for each before coding; record repo +
commit/tag + license + runtime in a new `docs/PERCEPTION_MODELS.md`:
- **"LocateAnything" (NVIDIA)** — likely open-vocab / promptable localization. Evaluate
  **NanoOWL** (OWL-ViT, TensorRT, Jetson), Grounding-DINO-class detectors, or the
  specific release if it exists. Confirm an ONNX path (no NVIDIA GPU on a Pi).
- **Segment Anything (Meta)** — SAM/SAM2 are heavy; for edge use **NanoSAM / MobileSAM /
  EdgeSAM** + ONNX export.
- **Depth Anything (V2)** — monocular depth; **Depth-Anything-V2-Small** → ONNX. Confirm
  license (some variants non-commercial) and a Pi-viable size.

### 19.1 Architecture — a perception pipeline with pluggable stages
Refactor `detector_node.py` from "two backends" to a **stage registry** that composes:
1. **Detector/localizer** → boxes(+labels): `hsv`, `cnn`, `locate_anything`.
2. **Segmenter** (optional) → masks: `sam`.
3. **Depth** (optional) → per-object depth: `depth_anything`.

Fuse into the **same `perception_state`** (+ richer debug topics
`/minibunker/seg_mask`, `/minibunker/depth_image`). Depth, if present, **replaces
`target_h_frac`** as the distance signal (real distance vs bbox proxy — a big teaching
win). Config under the `perception:` block (§5.1). Each stage loads lazily and **fails
soft** (log + skip), exactly like `CnnDetector` today, so the validated baselines never
break. Heavy models run on a worker thread at `perception/max_infer_hz` (1–5 Hz), never
in the camera callback; the 15 Hz FSM uses cached results.

### 19.2 Pi-5 compute reality (the real risk)
The Pi 5 has **no NVIDIA GPU**; naive PyTorch CPU inference of these models is far below
real-time. Design for it: prefer distilled/edge variants exported to ONNX; consider a
**Hailo-8L AI HAT / Coral** (`perception/device: hailo`); decouple inference from
control; reduce input resolution; offer a **laptop-offload** fallback (heavy perception
on a networked laptop via rosbridge/ROS master, Pi does drive + camera only); always
keep **HSV/CNN** as the guaranteed-real-time baseline so a demo never hard-depends on the
heavy stack.

---

## 20. How the project works now — mechanisms a new agent MUST know

1. **Dev mount = no rebuild for interpreted files.** `start_sim.sh` bind-mounts host
   `catkin_ws/src` → container `/home/rosuser/catkin_ws/src` (read-only), so the world
   file, `arena_setup.py`, launch files, `minibunker.yaml`, and **all Python nodes** are
   read live — just `bash start_sim.sh` to apply. **Only** C++ / custom-message changes
   need `--rebuild`. The image build `COPY`s + `catkin_make`s the source, and deleting
   the image does **not** clear Docker's build cache (which previously made rebuilds
   silently stale) — the mount sidesteps that. `MB_NO_MOUNT=1` disables it.
2. **One arena config block.** All props live under `arena:` at the bottom of
   `minibunker.yaml`. Each entry: `type`, size, `pose [x,y(,yaw)]`, `static`, `color`,
   and for dynamic props `mass / mu / kp / kd`. **Inertia is auto-computed** from
   type+size+mass (a mismatched inertia fights motion — don't hand-set unless via
   `inertia:`). `arena_setup.py` is a generic SDF builder
   (`sphere|box|cylinder|cone_mesh|plane|fence|light`) that spawns floor+lights first,
   then props one at a time (Ogre race). The `.world` file holds **only** physics + a
   baseline ambient `<scene>`.
3. **Role-based `perception_state` contract** — 7-slot `std_msgs/Float32MultiArray`:

   | idx | name | meaning |
   | --- | --- | --- |
   | 0 | `target_seen` | followed class present (0/1) |
   | 1 | `target_cx_norm` | target centre x, −1..1 (0 = centre, + = right) |
   | 2 | `target_cy_norm` | target centre y, −1..1 (+ = down) |
   | 3 | `target_h_frac` | target bbox height / image height (**distance proxy**) |
   | 4 | `hazard_seen` | nearest hazard present (0/1) |
   | 5 | `hazard_danger` | hazard is big + low-centre (in danger zone) |
   | 6 | `hazard_cx_norm` | nearest hazard centre x, −1..1 |

   Slots 0–3 = the **followed class**, 4–6 = the **nearest hazard**. Detector and
   behaviour share this; the layout comment lives at the top of both nodes. **Distance is
   still a bbox-height proxy** (no depth) — exactly what Phase E (Depth Anything)
   replaces.
4. **`/cmd_vel` has one owner: behaviour_node** (§18.3). Never add a second publisher;
   teleop feeds `/minibunker/teleop_cmd`.
5. **Contact-tuning intuition:** ground contact = spring (`kp`) + damper (`kd`). High
   `kp` = low sink but bouncy if underdamped → raise `kd`. Low `mu` = skids farther. The
   rover plows via `PlanarMovePlugin` (velocity forced), so a prop's mass mainly sets
   skid distance, not whether it gets knocked.

---

## 21. Remaining work & next-session phasing

| Phase | Scope | Gate |
| --- | --- | --- |
| **C** | Real-robot bring-up of A+B on the Pi (CAN + Pi cam), e-stop in hand, low caps first | drives + teleops on hardware |
| **D** | Detector → pluggable stage registry (§19.1); HSV/CNN become stages, stay green | baselines unaffected, UI selector |
| **E** | Add Depth Anything stage; COLLECT/stop on real distance behind a config switch | works in sim, proxy path still selectable |
| **F** | Add open-vocab ("LocateAnything") + SAM stages + (detector, segmenter, depth) combos | UI selectors, fail-soft |
| **G** | Pi-5 perf pass for D–F (edge variants, accelerator, laptop-offload fallback) | documented fps on Pi |
| **H** | `docs/PERCEPTION_MODELS.md`; update QUICKSTART/HARDWARE | — |

C is independent (needs hardware); D–G are the heavy perception work and need Q2–Q4
(§22) answered first — before any of them, do §19.0. The fastest high-value next step is
**Phase C** if hardware is available, else **D+E** (perception refactor + Depth
Anything).

> **Note — a no-ROS real-robot path exists.** Since this plan was first written, a
> separate `real_pi/` native-Python stack (picamera2 → detector → `python-can`, the
> §9.4 fallback) was added for the Pi; see `docs/REAL_PI_NATIVE.md`. It does not replace
> Phase C's ROS bring-up but is the current real-robot route; the CAN adapter is the last
> gate there.

---

## 22. Open questions (current)

Supersedes the "Open" list in §15; resolved items marked.

1. **Branding:** ✅ resolved + applied — full de-brand; rename complete.
2. **"LocateAnything" identity:** open — confirm the exact NVIDIA model/repo (NanoOWL? a
   specific release? Grounding-DINO-class?) + license + an ONNX/edge path. **Gates D/F.**
3. **Pi-5 accelerator:** open — Hailo AI HAT / Coral, or CPU-only? Decides on-device vs
   laptop-offload. **Gates E/F/G.**
4. **Depth meaning:** open — metric (needs calibration) vs relative + tuned threshold for
   COLLECT. **Gates E.**
5. **Class set for follow:** ✅ effectively resolved — role-based contract (option B)
   implemented; currently `{ball, cone}`. Open-vocab (arbitrary prompts) is the Phase-F
   extension of the same contract.
6. **`cone` as target — hazard set:** ✅ resolved — the followed class is auto-excluded
   from `hazard_items` (default `[cone]`); following the cone currently leaves no active
   hazard (set `hazard_items: []` to be explicit).

(The original hardware questions Q1/Q3/Q4/Q6 from §15 are resolved in
`docs/HARDWARE_SETUP.md` and `docs/TRAINING.md`.)

---

## 23. Housekeeping / gotchas

- **Dev-mount workflow:** edit interpreted files → `bash start_sim.sh` (live); only
  C++/msg → `--rebuild`. If something *still* looks stale, you're probably off the mount
  (`MB_NO_MOUNT` set) or it's a genuinely compiled change.
- **`.sh` on Windows** run as `& "C:\Program Files\Git\bin\bash.exe" ./script.sh`.
- **Repo git root** is `2.CodeRepos/minibunker-workshop/` (the subdir), **not** the
  SpaceSmSc workspace root — use `git -C <repo>` or run from inside.
- **Removable SanDisk (E:).** The drive can lock/unmount mid-session — `E:` then shows
  only a ~6 MB "SanDisk Drive Unlock" partition and the project "disappears". Not data
  loss; re-run the unlock. Don't panic-`git` against a missing path.
- **Submodule case-collision** (`box_link.STL` / `box_Link.STL`) in `ugv_gazebo_sim` can
  never be fully "clean" on Windows; silenced locally with `git update-index
  --skip-worktree` (local-only). Harmless on Linux/Docker — don't "fix" it by deleting a
  file.
- **Videos in git:** `assets/*.mp4` (~36 MB) are committed as plain binaries
  (`.gitattributes` marks `*.mp4 binary`, no LFS). Consider Git LFS if history bloat
  matters.

---

*Space Summer School · Technical University of Crete · SenseLAB — unified plan,
consolidated 2026-06-29 from `plan.md` / `plan2.md` / `plan3.md`. Phases 5–6/C need the
real MiniBunker 2.0 + Pi 5.*
