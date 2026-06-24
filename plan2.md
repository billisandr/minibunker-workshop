# MiniBunker Workshop — Plan 2 (status + handover)

> Continuation of [plan.md](plan.md). That file is the original Phase 0 design and
> still holds the architecture rationale (§3 seams, §4 decisions, §5 config schema).
> **This file is the live handover**: where we are now, and the concrete next items
> to add. A new session should be able to act from this document without re-deriving
> context. Convert any "today" references to absolute dates — this was written
> **2026-06-24**.

---

## 0. TL;DR for the next session

The sim path is **built and validated end-to-end** (Gazebo + detector + behaviour +
rosbridge + Streamlit UI, HSV backend). Four things are queued:

1. **Rename + full de-brand** — ✅ done in-repo (2026-06-24): "SpaceMine" dropped everywhere
   (titles/UI, Docker image → `minibunker`, world → `minibunker_arena.world`, training
   dataset → `minibunker`, clone URLs → `minibunker-workshop`). **Remaining:** the
   GitHub-side repo rename + git remote + local dir move (manual / network) — see §2. — §2
2. **Missions** — a `follow_item: ball | cone | none` knob (default `none`) so the rover
   finds-and-follows the chosen item, tunable from YAML **and** live from the UI, working
   in **sim first**, then on the **real Pi 5 + Pi Camera**. — §3
3. **Teleop** — when `follow_item: none`, the user drives with **WASD** keys. — §4
4. **Modern perception backends** — make the detector pluggable so the user can opt into
   **NVIDIA open-vocabulary localization ("LocateAnything")**, **Meta Segment Anything
   (SAM/SAM2)**, and **Depth Anything**, individually or in combination. — §5

Cross-cutting concerns that gate all of the above: **/cmd_vel arbitration + the ARM
safety gate** (§6) and the **Pi 5 compute reality** for the heavy models (§7).

---

## 1. Current status snapshot

### 1.1 What exists and works
- **Sim validated** (2026-06-24): `start_sim.sh` builds the Noetic Docker image, launches
  Gazebo `minibunker_arena.world` + Bunker (with our camera seam) + detector + behaviour +
  rosbridge. Confirmed clean: `arena_setup` spawns cones sequentially, no `boost::lock_error`,
  detector comes up on `backend = hsv`, behaviour boots **DISARMED**.
- **One ROS graph, two seams** (plan.md §3.1): camera source and velocity sink swap between
  sim and real; detector + behaviour are identical. Topic contract is the anchor.
- **Two perception backends** behind one contract: `HsvDetector` and `CnnDetector`
  (YOLOv8-nano via onnxruntime/ultralytics/ncnn) in
  [detector_node.py](catkin_ws/src/minibunker_perception/src/detector_node.py). Backend is
  re-read every frame so the UI can flip it live.
- **Reactive FSM**: `SEARCH → APPROACH → AVOID → COLLECT → STOP` in
  [behavior_node.py](catkin_ws/src/minibunker_behavior/src/behavior_node.py). Boots DISARMED,
  clamps to `behavior/limits`, ARM gate via `/minibunker/arm`.
- **Streamlit UI** ([app.py](catkin_ws/src/minibunker_ui/app.py)) over rosbridge: live
  annotated image, telemetry, live knobs (backend, CNN conf, HSV ranges, gains, caps),
  ARM/DISARM.
- **One master config**
  [config/minibunker.yaml](catkin_ws/src/minibunker_bringup/config/minibunker.yaml) → rosparam.
- **Arena composition is config-driven**: `arena/fence_enabled` (default off),
  `arena/cone_scale` via [arena_setup.py](catkin_ws/src/minibunker_bringup/scripts/arena_setup.py),
  cones use the real vendored `construction_cone` mesh.

### 1.2 The contract everything hangs off (read before changing perception/behaviour)
`/minibunker/perception_state` is a `std_msgs/Float32MultiArray`, layout shared by
detector ([detector_node.py:17-25](catkin_ws/src/minibunker_perception/src/detector_node.py#L17-L25))
and behaviour ([behavior_node.py:18-21](catkin_ws/src/minibunker_behavior/src/behavior_node.py#L18-L21)):

| idx | name | meaning |
| --- | --- | --- |
| 0 | `target_seen` | green ball present (0/1) |
| 1 | `target_cx_norm` | ball centre x, −1..1 (0 = centre, + = right) |
| 2 | `target_cy_norm` | ball centre y, −1..1 (+ = down) |
| 3 | `target_h_frac` | ball bbox height / image height (the **distance proxy**) |
| 4 | `cone_seen` | cone present (0/1) |
| 5 | `cone_danger` | cone is big + low-centre (in danger zone) |
| 6 | `cone_cx_norm` | nearest cone centre x, −1..1 |

Two facts that shape §3 and §5:
- "**target**" is currently hardwired to the **green ball**; "**hazard**" to the **cone**.
  `follow_item` (§3) generalises *which class is target*.
- There is **no depth** — distance is proxied by `target_h_frac`. **Depth Anything** (§5)
  is the natural upgrade to a real depth signal here.

### 1.3 Known not-done (pre-existing, from plan.md roadmap)
- CNN not trained/validated in sim (HSV is the working default).
- Real-robot bring-up on the Pi (CAN + Pi camera) not run on hardware.
- Phase 7 docs (participant handout + instructor cards) not written.

### 1.4 Housekeeping notes for the next session
- **Windows case-collision** in submodule `ugv_gazebo_sim`: two tracked paths
  `box_link.STL` and `box_Link.STL` differ only by case, so the submodule can never be
  fully "clean" on Windows. Silenced locally with `git update-index --skip-worktree
  scout/pro/meshes/box_Link.STL` (local-only, not pushed). It is harmless — Docker/Linux
  is case-sensitive and builds fine. Don't "fix" it by deleting a file.
- `.sh` scripts on Windows are run from PowerShell as
  `& "C:\Program Files\Git\bin\bash.exe" ./script.sh` (workspace convention).
- Repo lives at `2.CodeRepos/minibunker-spacemine-workshop/` (the git root is this subdir,
  **not** the `SpaceSmSc` workspace root).

---

## 2. Task 1 — rename to `minibunker-workshop` (local + online)

**Decision (RESOLVED 2026-06-24): full de-brand.** "SpaceMine" is dropped everywhere; the
station is plain *"MiniBunker"*. This was applied across code, docs, UI, the Docker image
name, the Gazebo world, and the training dataset (see §2.4). What remains is purely the
GitHub-side rename + local working-dir move below (§2.1–2.3).

GitHub keeps a redirect from the old name, but we update the remote anyway for cleanliness.

### 2.1 Online (GitHub)
```bash
# from inside the repo, gh is authed as billisandr
gh repo rename minibunker-workshop --repo billisandr/minibunker-spacemine-workshop --yes
```

### 2.2 Local git remote
```bash
git remote set-url origin https://github.com/billisandr/minibunker-workshop.git
git remote -v   # verify
```

### 2.3 Local working-directory rename
The folder is `2.CodeRepos/minibunker-spacemine-workshop`. On Windows this **must be done
with the IDE and any shells closed on that path** (open handles block the move), so it is a
**manual step for the user**, not something to run mid-session from inside the dir:
```powershell
# from 2.CodeRepos, with nothing open inside the folder
Rename-Item minibunker-spacemine-workshop minibunker-workshop
```

### 2.4 In-repo de-brand — ✅ DONE (2026-06-24)
The full de-brand has been applied in-repo. Changes made:
- **Clone URLs / repo name** → `minibunker-workshop`: `README.md`, `docs/QUICKSTART.md`.
- **Docker image** `minibunker-spacemine` → `minibunker`: `start_sim.sh`, `start_real.sh`,
  `docs/HARDWARE_SETUP.md`, `docs/QUICKSTART.md`.
- **Gazebo world** `spacemine_arena.world` → `minibunker_arena.world` (file `git mv`d +
  `<world name>` + the launch path in `minibunker_sim.launch` + the `Dockerfile` / `ARENA.md`
  references).
- **Titles / prose** "MiniBunker SpaceMine" → "MiniBunker": `README.md`, `docs/QUICKSTART.md`,
  `app.py` (page title + header), `detector_node.py`, both `package.xml`, `minibunker.yaml`,
  `arena_setup.py`, the `start_*.sh` / `Dockerfile` headers.
- **Training** dataset/run `spacemine` / `spacemine_yolov8n` → `minibunker` /
  `minibunker_yolov8n`: `training/data.yaml`, `training/train.py`, `training/export.py`,
  `docs/TRAINING.md`.

The genuine theme is kept (the event "Space Summer School", the "space-mining" flavour, the
"ore" ball, 🛰️) — only the "SpaceMine" *brand* was removed.

`plan.md` keeps its historical Phase-0 references (commands actually run under the old name);
a de-brand note was added at its top instead of rewriting that frozen record.

### 2.5 Acceptance
- `gh repo view billisandr/minibunker-workshop` resolves; old URL redirects.
- `git push`/`pull` work against the new remote.
- Fresh `git clone --recurse-submodules <new-url>` + `start_sim.sh` still builds & runs.

---

## 3. Task 2a — missions: `follow_item: ball | cone | none`

**Goal:** the user picks what the rover hunts. `none` is the default (rover does not
auto-drive; see §4 teleop). Settable in YAML **and** live from the UI. Sim first, then real.

### 3.1 Config (add to `minibunker.yaml`, new `mission:` block)
```yaml
mission:
  follow_item: none        # none | ball | cone   (none = manual WASD teleop, see §4)
  hazard_items: [cone]     # classes treated as obstacles to AVOID (can be empty)
  # 'none' => behaviour yields /cmd_vel to teleop; ball/cone => autonomous follow
```
Rationale: decoupling **target** (`follow_item`) from **hazards** (`hazard_items`) keeps the
door open for the open-vocab backends in §5, where the class set is no longer just
{ball, cone}.

### 3.2 Behaviour changes ([behavior_node.py](catkin_ws/src/minibunker_behavior/src/behavior_node.py))
- Read `mission/follow_item` (re-read each tick so the UI flips it live, same pattern as the
  detector backend).
- Map `follow_item` → which perception_state slots are the **target**:
  - `ball` → use slots [0..3] as today.
  - `cone` → use the cone as the *approach target* (needs detector to also export a
    cone cx/cy/h_frac as a target; see §3.3 — currently only ball has full target slots).
  - `none` → enter a new **TELEOP** state: stop autonomous control, do **not** publish
    autonomous Twist; consume the teleop intent instead (§4). ARM gate + clamps still apply.
- Keep AVOID driven by `hazard_items` (today: cone danger). If the followed item *is* the
  cone, it should not simultaneously be its own hazard — exclude `follow_item` from the
  active hazard set.

### 3.3 Detector change (generalise target slots)
Today only the green ball gets full `target_*` slots; the cone only gets seen/danger/cx.
To let `follow_item: cone` actually be *followed*, either:
- **(A, recommended, minimal)** Add a second full target group, or
- **(B)** Make `perception_state` role-based: detector reads `mission/follow_item` +
  `mission/hazard_items` and packs **target_* = the followed class** and **hazard_* = nearest
  hazard**, regardless of which physical class each is.

Option **B** is cleaner long-term (it's the same contract the §5 open-vocab backends want)
but changes the contract — do it once, carefully, and update both the layout comment blocks
and the UI telemetry reader. Recommend B, but A is acceptable if time-boxed.

### 3.4 UI ([app.py](catkin_ws/src/minibunker_ui/app.py))
- Add a **Mission** control (selectbox `none | ball | cone`) in the Controls tab that
  `set_param("/mission/follow_item", …)` live.
- Show the active target/hazard in telemetry.
- When `none`, reveal the WASD pad (§4).

### 3.5 Sim-first test, then real
1. **Sim**: launch, ARM, set `follow_item: ball` → rover SEARCH→APPROACH→COLLECT on the ore
   ball while AVOIDing cones. Then `follow_item: cone` → follows a cone. Then `none` → stops
   (and teleop works, §4).
2. **Real**: same knobs on the Pi launch; validate with e-stop in hand, fenced arena, low
   speed caps first.

### 3.6 Acceptance
- Changing `follow_item` live in the UI changes behaviour with no relaunch.
- `none` is the boot default and yields no autonomous motion.
- Works identically in sim and (subsequently) on the real robot.

---

## 4. Task 2b — WASD keyboard teleop (when `follow_item: none`)

**Design decision (safety-critical):** keep **behaviour_node as the single owner of
/cmd_vel** so the ARM gate and speed clamps always apply. Teleop publishes *intent*, not
raw velocity. This avoids a two-publisher race on /cmd_vel and keeps DISARM authoritative.

### 4.1 New node: `teleop_node.py` (in `minibunker_perception`? no — new small pkg or
`minibunker_behavior/scripts/`)
- Captures WASD (W=fwd, S=back, A=turn-left, D=turn-right; space=stop), publishes
  `/minibunker/teleop_cmd` (`geometry_msgs/Twist`) at a fixed rate while keys are held.
- Terminal keyboard capture in a container is fiddly; provide **two input paths**:
  - **(a)** a ROS teleop node for a terminal (`docker exec -it … rosrun … teleop_node.py`),
    style of `teleop_twist_keyboard`.
  - **(b)** **UI WASD pad** in Streamlit (hold-to-move buttons / key handler) that publishes
    `/minibunker/teleop_cmd` over rosbridge — this is the non-coder front door and the
    primary path; (a) is for power users.
- Note Streamlit's latency/key-repeat limits: implement as press-and-publish with a UI
  refresh tick, and a prominent **STOP** button. Document the latency caveat.

### 4.2 Behaviour wiring
- In TELEOP state (`follow_item: none`), behaviour subscribes to `/minibunker/teleop_cmd`,
  passes it through the **same** ARM gate + `behavior/limits` clamp, and republishes to
  `/cmd_vel`. DISARM → zero Twist, exactly as today.
- A teleop watchdog: if no teleop_cmd for N ms, command zero (don't latch motion).

### 4.3 Sim-first, then real; same as §3.5. On real, WASD must respect the same caps and the
hardware e-stop is still the real stop.

### 4.4 Acceptance
- With `follow_item: none` + ARMED, WASD drives the rover in sim; release → stops.
- DISARM overrides teleop instantly. Watchdog stops on input loss.

---

## 5. Task 2c — modern perception backends (LocateAnything / SAM / Depth Anything)

**Goal:** let the user opt into NVIDIA open-vocabulary localization, Meta SAM, and Depth
Anything — individually or combined — without breaking the one-topic contract or the
HSV/CNN baselines. This is the biggest piece; treat it as its own mini-roadmap.

### 5.0 ⚠️ Verify upstreams first (next-session action, do NOT assume)
These names must be pinned to real, licensed, edge-viable sources before coding. Confirm
repo/model/license/runtime for each (the exact "NVIDIA LocateAnything" identity especially):
- **"LocateAnything" (NVIDIA)** — likely an **open-vocabulary detection / promptable
  localization** model. Candidates to evaluate: NVIDIA **NanoOWL** (OWL-ViT, TensorRT,
  Jetson-optimized), Grounding-DINO-style open-vocab detectors, or the specific
  "LocateAnything" release if it exists. **Action:** confirm the canonical repo + license +
  whether there's an ONNX/TensorRT path that fits Pi 5 (no NVIDIA GPU on a Pi!).
- **Segment Anything (Meta)** — **SAM / SAM2** are heavy. For edge use the distilled
  variants: **NanoSAM** (NVIDIA-AI-IOT), **MobileSAM**, or **EdgeSAM**. **Action:** pick the
  variant + ONNX export.
- **Depth Anything (V2)** — monocular depth. Use **Depth-Anything-V2-Small** exported to
  ONNX. **Action:** confirm license (some variants are non-commercial) and a Pi-viable size.

Record the chosen repo + commit/tag + license for each in a new `docs/PERCEPTION_MODELS.md`.

### 5.1 Architecture — a perception **pipeline** with pluggable stages
Refactor `detector_node.py` from "two backends" to a small **stage registry** so stages
compose. Conceptual stages:
1. **Detector / localizer** → boxes (+labels): `hsv`, `cnn`, `locate_anything` (open-vocab).
2. **Segmenter** (optional) → masks for the detected boxes: `sam`.
3. **Depth** (optional) → per-pixel/per-object depth: `depth_anything`.
The node fuses stage outputs into the **same `perception_state`** (and richer debug topics).
Depth, if present, **replaces `target_h_frac` as the distance signal** (huge teaching win:
real distance instead of a bbox proxy).

### 5.2 Config (new `perception:` / extend `detector:`)
```yaml
perception:
  detector: hsv            # hsv | cnn | locate_anything
  segmenter: none          # none | sam            (overlays masks; refines centroid)
  depth: none              # none | depth_anything  (replaces bbox-frac distance proxy)
  open_vocab_prompts: ["green ball", "construction cone"]   # for locate_anything
  device: cpu              # cpu | cuda | tensorrt | hailo   (see §7)
  max_infer_hz: 5          # throttle heavy models independently of camera fps
```
Keep `detector/backend: hsv|cnn` working as an alias for back-compat, or migrate the UI/launch
to the new keys in one pass. Each stage is independently selectable → "combinations" = any
valid (detector, segmenter, depth) triple.

### 5.3 Code changes
- `detector_node.py`: introduce a `Stage` interface (`load()`, `infer(bgr, prior) -> result`)
  and a registry keyed by config. Existing `HsvDetector`/`CnnDetector` become detector stages.
  New stages: `LocateAnythingDetector`, `SamSegmenter`, `DepthAnythingEstimator`. Each loads
  lazily and **fails soft** (log + skip), exactly like `CnnDetector` does today
  ([detector_node.py:118-122](catkin_ws/src/minibunker_perception/src/detector_node.py#L118-L122)).
- New debug topics: `/minibunker/seg_mask`, `/minibunker/depth_image` (+ compressed twins for
  the UI, mirroring the existing `/minibunker/debug_image/compressed` pattern).
- Behaviour: if depth is active, consume real distance instead of `target_h_frac` for the
  COLLECT/stop trigger (config switch so the proxy path still works).
- Heavy models run on a **separate thread at `max_infer_hz`**, not in the camera callback,
  so the 15 Hz control loop never blocks. Cache last result between inferences.

### 5.4 Docker / deps
- Big new dependencies (torch, model runtimes). Keep them **optional / opt-in** so the base
  sim image stays light: a separate build stage or a `--build-arg WITH_HEAVY_MODELS=1`, and/or
  a second `Dockerfile.perception`. Vendor model weights at build time (offline-run principle,
  like the construction_cone mesh) where licensing allows; otherwise document a fetch step.
- amd64 (dev laptop, maybe CUDA) vs arm64 (Pi 5, CPU/Hailo) diverge here — plan per-arch.

### 5.5 UI
- Add **Detector / Segmenter / Depth** selectors + open-vocab prompt text box in a new
  "Perception" tab. Show seg-mask overlay and a depth colormap next to the live view.
- Flag which switches are live vs need-relaunch (model loads are heavy → likely relaunch or a
  guarded reload, unlike the HSV sliders which are per-frame live).

### 5.6 Sim-first, then real
- Validate each stage in Gazebo on the synthetic/sim camera first (correctness, topic shapes,
  UI overlays), **then** measure on the Pi (§7). Expect to drop resolution / `max_infer_hz`
  for the real robot.

### 5.7 Acceptance
- User can select any (detector, segmenter, depth) combo from YAML or UI; invalid/missing
  models fail soft (baseline keeps running).
- With `depth_anything` on, COLLECT triggers on real distance.
- `docs/PERCEPTION_MODELS.md` records each model's source/license/runtime/edge variant.

---

## 6. Cross-cutting: /cmd_vel arbitration + ARM safety (do not regress)

- **Single /cmd_vel owner = behaviour_node.** Autonomous follow, teleop pass-through, and the
  DISARM zero-Twist all flow through it so the ARM gate + `behavior/limits` clamp are
  unconditional. No second publisher on /cmd_vel.
- Boots **DISARMED**; DISARM publishes zero every tick and wins over any latched/teleop input.
- Teleop and every new mission mode inherit these guarantees (§4.2).
- On the real robot, software caps are a backstop, **not** a substitute for the hardware
  e-stop (docs/HARDWARE_SETUP.md, docs/ARENA.md).

---

## 7. Cross-cutting: Pi 5 compute reality (the real risk)

The Pi 5 has **no NVIDIA GPU**. SAM2 / open-vocab detectors / Depth Anything are heavy; naive
PyTorch CPU inference will be far below real-time. This is the single biggest risk in §5 and
must be designed for, not discovered later:
- Prefer **distilled/edge variants** (NanoOWL, NanoSAM/MobileSAM/EdgeSAM,
  Depth-Anything-V2-Small) exported to **ONNX**; consider an **AI accelerator** (Hailo-8L AI
  HAT, or Coral) and the matching runtime (`device: hailo`).
- **Decouple inference from control**: run heavy models at `max_infer_hz` (e.g. 1–5 Hz) on a
  worker thread; keep the 15 Hz FSM on cached results.
- **Reduce input resolution** for heavy stages.
- **Offload option**: document running the heavy perception on a laptop on the same network
  (rosbridge/ROS master), Pi does drive + camera only — good fallback for the workshop.
- Always keep **HSV/CNN** as the guaranteed-real-time baseline so a demo never hard-depends on
  the heavy stack.

---

## 8. Suggested phasing for the next session(s)

| Phase | Scope | Gate |
| --- | --- | --- |
| **R** | Rename repo (§2) | clone+sim still works |
| **A** | `follow_item` + mission config + UI selector (§3), sim-only | live switch, none=default |
| **B** | WASD teleop via behaviour pass-through + UI pad (§4), sim-only | ARM/DISARM authoritative |
| **C** | Real-robot validation of A+B on the Pi (CAN + Pi cam) | e-stop, low caps first |
| **D** | Perception refactor to pluggable stages (§5.1-5.3), keep HSV/CNN green | baselines unaffected |
| **E** | Add Depth Anything stage (best ROI: real distance) | COLLECT on real depth, sim |
| **F** | Add open-vocab ("LocateAnything") + SAM stages + combos | UI selectors, sim |
| **G** | Pi 5 perf pass for D–F (edge variants, accel, offload) | documented fps on Pi |
| **H** | Docs: PERCEPTION_MODELS.md, MISSIONS.md, TELEOP.md, update QUICKSTART/HARDWARE | — |

Phases A–C deliver the user-visible mission/teleop value quickly and are low-risk. D–G are
the heavier perception work; keep each opt-in and fail-soft so the validated sim never breaks.

---

## 9. New / changed files map (forward reference)

- **Rename:** `README.md`, `docs/QUICKSTART.md`, `plan.md` (refs); git remote; local dir.
- **Config:** `config/minibunker.yaml` (+`mission:`, +`perception:`).
- **Behaviour:** `behavior_node.py` (follow_item, TELEOP state, teleop pass-through, watchdog).
- **Detector:** `detector_node.py` (role-based perception_state; pluggable stage registry;
  new stages; new debug topics).
- **New nodes:** `teleop_node.py`; stage modules for locate-anything / sam / depth-anything.
- **Launch:** `minibunker_sim.launch` / `minibunker_real.launch` (teleop node, perception args).
- **UI:** `app.py` (Mission selector, WASD pad, Perception tab with mask/depth overlays).
- **Docker:** optional heavy-models build stage / `Dockerfile.perception`; per-arch weights.
- **Docs (new):** `docs/MISSIONS.md`, `docs/TELEOP.md`, `docs/PERCEPTION_MODELS.md`; update
  `docs/QUICKSTART.md`, `docs/HARDWARE_SETUP.md`, `README.md`.

---

## 10. Open questions for the user / next session

1. **Branding:** ✅ RESOLVED — full de-brand. "SpaceMine" dropped everywhere; the station is
   plain "MiniBunker". Applied in-repo 2026-06-24 (§2.4); only the GitHub rename + dir move
   remain (§2.1–2.3).
2. **"LocateAnything" identity:** confirm the exact NVIDIA model/repo intended (NanoOWL?
   a specific "Locate Anything" release? Grounding-DINO-class?). Drives §5.0/§5.3 + licensing.
3. **Pi 5 accelerator:** is a Hailo AI HAT / Coral available, or CPU-only? Decides whether the
   heavy models run on-device or via the laptop-offload fallback (§7).
4. **Depth meaning:** Depth Anything gives *relative* depth by default; do we need *metric*
   distance (requires calibration/scale) for COLLECT, or is relative + a tuned threshold ok?
5. **Class set for follow:** stick to {ball, cone}, or let open-vocab prompts define arbitrary
   followable items (drives perception_state generalisation option A vs B, §3.3)?
6. **`cone` as target:** when following the cone, what's the hazard set — empty, or other
   cones? (§3.1 `hazard_items`.)

---

*Space Summer School · Technical University of Crete · SenseLAB — handover plan, 2026-06-24.*
