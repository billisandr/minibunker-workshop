# MiniBunker Workshop — Plan 3 (status + handover)

> Continuation of [plan.md](plan.md) (original Phase-0 design + architecture rationale)
> and [plan2.md](plan2.md) (the missions/teleop/perception roadmap, with the canonical
> §-references this file points back to). **This file is the live handover**: what got
> done since plan2, what changed about *how the project works*, and the concrete next
> items. A new session should be able to act from this document. Written **2026-06-25**;
> all relative dates converted to absolute.

---

## 0. TL;DR for the next session

Since plan2, **Phases R, A and B are done and the sim is hands-on validated** (the rover
finds/follows the ball, avoids/knocks cones, drives under WASD — see the clips in
[README.md](README.md) Gallery). Beyond plan2's scope, this session also: made all arena
props **dynamic + physically tuned**, **consolidated the entire arena into one config
block**, added a **dev bind-mount so edits go live without a rebuild**, fixed Streamlit
deprecations, and added **real-keyboard WASD** in the UI.

**What's left (unchanged from plan2's back half):**
- **Phase C** — real-robot bring-up on the Pi 5 (CAN + Pi camera). Needs hardware. Not started.
- **Phases D–G** — pluggable perception stages (LocateAnything / SAM / Depth Anything),
  then a Pi-5 perf pass. **Gated by open questions Q2–Q4 (§7).** Not started.
- **Phase H** — `docs/PERCEPTION_MODELS.md` + QUICKSTART/HARDWARE updates (MISSIONS.md and
  TELEOP.md are done).

The fastest high-value next step is still **Phase C** (real robot) if hardware is available,
or **Phase D+E** (perception refactor + Depth Anything) once Q2–Q4 are answered.

---

## 1. What got done this session (2026-06-25)

### 1.1 Plan2 phases delivered
- **Phase R — rename/de-brand: ✅ COMPLETE.** Remote is `billisandr/minibunker-workshop`,
  local dir is `minibunker-workshop`. (plan2 listed the GitHub rename + dir move as the only
  remainder; both are done.)
- **Phase A — missions (`follow_item`): ✅ DONE (sim).** Implemented the **role-based
  `perception_state`** (plan2 §3.3 **option B**): the detector reads `mission/follow_item`
  + `mission/hazard_items` each frame and packs **target_\*** = the followed class and
  **hazard_\*** = nearest hazard, auto-excluding the followed class from its own hazard set.
  `none | ball | cone`, live-switchable from the UI. The same FSM now follows a ball *or* a cone.
- **Phase B — WASD teleop: ✅ DONE (sim).** behaviour stays the single `/cmd_vel` owner;
  teleop publishes *intent* on `/minibunker/teleop_cmd`, gated through the **same ARM gate +
  limits clamp**, with a **watchdog** (zeroes on stale input). Three input paths: the UI
  **WASD pad**, **physical W/A/S/D keys** in the browser (X = stop), and a terminal
  `teleop_node.py` (run via `teleop.sh`).
- **Phase H (partial): ✅** `docs/MISSIONS.md`, `docs/TELEOP.md` written; `docs/ARENA.md`
  updated for the new arena schema.

### 1.2 Beyond plan2 (this session's extra work)
- **Collect → retreat behaviour cycle** ([behavior_node.py](catkin_ws/src/minibunker_behavior/src/behavior_node.py)):
  on reaching the target the rover now **stops → pauses (`behavior/collect/pause_sec`, 5 s) →
  turns a random way → drives off → resumes SEARCH** (new `RETREAT` state), so it doesn't
  re-collect the same item.
- **HSV cone detection fixed** for the black/orange/white striped cone: `_detect_colour` now
  CLOSEs (tall, narrow kernel `[61,9]`) before OPENing so the stacked orange bands merge into
  one full-cone blob even up close. Kernels are per-colour config.
- **Physics overhaul — all props dynamic + tuned.** Ball and cones are now `static: false`
  with auto-computed inertia, tunable `mass / mu / kp / kd`. The ~24 kg rover (PlanarMove
  forces velocity) knocks them aside. Learned the hard way: a stiff contact (`kp`) with low
  damping (`kd`) makes a prop **bob vertically in place** — raise `kd`.
- **Single-source arena config (big one).** *Every* arena object — `floor`, `sun` light,
  `ore_ball`, `cone_1..3`, `fence` — is now defined in **one `arena:` block at the bottom of
  [minibunker.yaml](catkin_ws/src/minibunker_bringup/config/minibunker.yaml)**.
  [arena_setup.py](catkin_ws/src/minibunker_bringup/scripts/arena_setup.py) is a generic SDF
  builder (`sphere | box | cylinder | cone_mesh | plane | fence | light`) that spawns them;
  the `.world` file holds **only** physics + a baseline `<scene>` ambient.
- **Dev bind-mount in [start_sim.sh](start_sim.sh).** The host `catkin_ws/src` is mounted over
  the image's copy, so edits to the world/yaml/launch/Python nodes go **live on a plain
  relaunch — no `--rebuild`, no Docker build-cache traps** (`MB_NO_MOUNT=1` to disable). This
  ended a long rebuild-confusion loop; see §6.
- **UI fixes:** `get_param` now falls back on a missing param (was returning `None` → crash);
  Streamlit deprecations fixed (`use_container_width` → `width="stretch"`, `components.html`
  → `st.iframe`); image monitors are side-by-side, half-width.
- **`run_ui.sh`** no longer sources `activate` (avoids the Git-Bash `uname: command not found`);
  it calls the venv python directly.
- **README Gallery** with the two sim clips (`assets/*.mp4`) + screenshots (`assets/*.png`).

---

## 2. Current status — what works now

- **Sim is hands-on validated** (not just "builds"): Gazebo + detector + behaviour +
  rosbridge + Streamlit, HSV backend. The rover SEARCH→APPROACH→COLLECT→RETREAT on the ball,
  AVOIDs/knocks cones, and drives under WASD. Captured in the README Gallery clips.
- **Mission switching is live** (UI selectbox, no relaunch). `none` is the boot default and
  yields no autonomous motion.
- **Teleop** works via UI pad, physical keys, and the terminal node; ARM/DISARM authoritative;
  watchdog stops on link loss.
- **Arena is fully config-driven and dynamic**; props are knockable and tunable from one block.
- **Edit→relaunch loop** is fast thanks to the dev mount.

### Known not-done / still rough
- **Real robot (Phase C):** never run on hardware (CAN + Pi camera).
- **CNN backend:** still untrained/unvalidated in sim; HSV is the working default.
- **Physics tuning is "good enough", still iterable.** If a cone seems stuck bobbing, first
  check whether the **rover is driving *over* a short cone** (cone is ~0.135 m tall at
  `scale 3.0`) rather than hitting its side — raise `cone_*/scale` so it's struck side-on.
- **Floor friction (`arena/objects/floor/mu`)** combines with each prop's `mu` (ODE ≈ min),
  so a prop's low `mu` usually dominates; the floor knob is secondary.
- **`st.iframe` keyboard hack:** the WASD-key listener relies on same-origin `window.parent`
  access inside the `st.iframe`. Verified to render but **confirm keys actually drive in the
  browser**; fallback is `st.html(..., unsafe_allow_javascript=True)` (inline, no iframe).

---

## 3. How the project works now — mechanisms a new agent MUST know

1. **Dev mount = no rebuild for interpreted files.** `start_sim.sh` bind-mounts host
   `catkin_ws/src` → container `/home/rosuser/catkin_ws/src` (read-only). So the world file,
   `arena_setup.py`, launch files, `minibunker.yaml`, and **all Python nodes** are read live —
   just `bash start_sim.sh` to apply. **Only** C++ / custom-message changes need
   `bash start_sim.sh --rebuild`. The image build `COPY`s + `catkin_make`s the source, and
   deleting the image does **not** clear Docker's build cache, which previously made rebuilds
   silently stale — the mount sidesteps all of that.
2. **One arena config block.** All props live under `arena:` at the bottom of
   `minibunker.yaml`. Each entry: `type`, size, `pose [x,y(,yaw)]`, `static`, `color`, and for
   dynamic props `mass / mu / kp / kd`. **Inertia is auto-computed** from type+size+mass (a
   mismatched inertia fights motion — that bug bit the ball early; don't hand-set it unless via
   `inertia: [ixx,iyy,izz]`). `arena_setup.py` spawns floor+lights first, then props, one at a
   time (Ogre race).
3. **Role-based `perception_state` contract** (7-slot `Float32MultiArray`): `[0..3]` =
   target_seen/cx/cy/h_frac of the **followed class**, `[4..6]` = hazard_seen/danger/cx of the
   **nearest hazard**. Detector and behaviour share this; the layout comment lives at the top
   of both nodes. **Distance is still a bbox-height proxy** (no depth) — this is exactly what
   Phase E (Depth Anything) replaces.
4. **`/cmd_vel` has one owner: behaviour_node.** Autonomous follow, teleop pass-through, and
   the DISARM zero-Twist all flow through the one ARM gate + `behavior/limits` clamp. Never add
   a second `/cmd_vel` publisher. Teleop feeds `/minibunker/teleop_cmd` instead.
5. **Contact tuning intuition:** ground contact = spring (`kp`) + damper (`kd`). High `kp` =
   low sink but bouncy if underdamped → raise `kd`. Low `mu` = skids farther. The rover plows
   via `PlanarMovePlugin` (velocity forced), so a prop's mass mainly sets skid distance, not
   whether it gets knocked.

---

## 4. What remains (mapped to plan2 §8)

| Phase | Scope | Status |
| --- | --- | --- |
| R | Rename/de-brand | ✅ done |
| A | `follow_item` + mission + UI (sim) | ✅ done |
| B | WASD teleop (sim) | ✅ done |
| **C** | **Real-robot validation on the Pi (CAN + Pi cam)** | ❌ not started — needs hardware |
| **D** | **Perception refactor to pluggable stages** (plan2 §5.1–5.3), keep HSV/CNN green | ❌ not started — gated by Q2 |
| **E** | **Depth Anything stage** (best ROI: real distance, replaces the bbox proxy) | ❌ not started — gated by Q3/Q4 |
| **F** | **Open-vocab ("LocateAnything") + SAM stages + combos** | ❌ not started — gated by Q2/Q3 |
| **G** | **Pi-5 perf pass** for D–F (edge variants, accel, offload) | ❌ not started — gated by Q3 |
| H | Docs | 🟡 partial — MISSIONS.md, TELEOP.md, ARENA.md done; **PERCEPTION_MODELS.md + QUICKSTART/HARDWARE updates remain** |

**Before any of D–G, do plan2 §5.0:** pin the real repo + license + edge/ONNX viability for
each model (NanoOWL / Grounding-DINO-class for "LocateAnything"; NanoSAM/MobileSAM/EdgeSAM for
SAM; Depth-Anything-V2-Small) and record them in a new `docs/PERCEPTION_MODELS.md`. Keep every
stage **opt-in and fail-soft** so the validated HSV/CNN sim never breaks (plan2 §5.3).

---

## 5. Suggested phasing for the next session(s)

| Phase | Scope | Gate |
| --- | --- | --- |
| **C** | Real-robot bring-up of A+B on the Pi (CAN + Pi cam), e-stop in hand, low caps first | drives + teleops on hardware |
| **D** | Detector → pluggable stage registry (plan2 §5.1–5.3); HSV/CNN become stages, stay green | baselines unaffected, UI selector |
| **E** | Add Depth Anything stage; COLLECT/stop on real distance behind a config switch | works in sim, proxy path still selectable |
| **F** | Add open-vocab + SAM stages + (detector, segmenter, depth) combos | UI selectors, fail-soft |
| **G** | Pi-5 perf pass for D–F (edge variants, accelerator, laptop-offload fallback) | documented fps on Pi |
| **H** | `docs/PERCEPTION_MODELS.md`; update QUICKSTART/HARDWARE | — |

C is independent (hardware). D–G are the heavy perception work and need Q2–Q4 first.

---

## 6. Files map (this session)

- **Config:** `config/minibunker.yaml` — `mission:` block; `behavior/collect` + `behavior/teleop`;
  per-colour HSV morphology; and the consolidated **`arena:`** block (floor/sun/ball/cones/fence).
- **Behaviour:** `behavior_node.py` — role-based slots, `TELEOP` + `RETREAT` states, teleop
  pass-through + watchdog, collect→retreat cycle.
- **Detector:** `detector_node.py` — role-based `perception_state`; per-colour close/open kernels.
- **New node:** `teleop_node.py` (terminal WASD); registered in `minibunker_behavior/CMakeLists.txt`.
- **Arena:** `arena_setup.py` — generic SDF builder (all object types); `worlds/minibunker_arena.world`
  now physics + ambient only.
- **Launch:** `minibunker_sim.launch` comment; (teleop node is **not** auto-launched — needs a TTY).
- **UI:** `app.py` — Mission selector, role-based telemetry, WASD pad + physical-key listener,
  side-by-side monitors, `get_param` null-safe, Streamlit-deprecation fixes.
- **Scripts:** `start_sim.sh` (dev mount), `teleop.sh` (new), `run_ui.sh` (no `activate`).
- **Docs:** new `docs/MISSIONS.md`, `docs/TELEOP.md`; updated `docs/ARENA.md`; `README.md` Gallery.
- **Assets:** `assets/minibunker_sim_{gz,ui}.mp4`, `assets/mb_sim_{gz,ui}.png`.

---

## 7. Open questions (updated from plan2 §10)

1. **Branding:** ✅ resolved + applied (rename complete).
2. **"LocateAnything" identity:** still open — confirm the exact NVIDIA model/repo (NanoOWL? a
   specific release? Grounding-DINO-class?) + license + an ONNX/edge path. **Gates D/F.**
3. **Pi-5 accelerator:** still open — Hailo AI HAT / Coral, or CPU-only? Decides on-device vs
   laptop-offload. **Gates E/F/G.**
4. **Depth meaning:** still open — metric (needs calibration) vs relative + tuned threshold for
   COLLECT. **Gates E.**
5. **Class set for follow:** ✅ effectively resolved — role-based contract (option B) is
   implemented; currently `{ball, cone}` only. Open-vocab (arbitrary prompts) is the Phase-F
   extension of the same contract.
6. **`cone` as target — hazard set:** ✅ resolved — the followed class is auto-excluded from
   `hazard_items` (default `[cone]`); following the cone currently leaves no active hazard
   (set `hazard_items: []` explicitly if desired).

---

## 8. Housekeeping / gotchas for the next session

- **Dev mount workflow:** edit interpreted files → `bash start_sim.sh` (live). Only C++/msg
  changes → `--rebuild`. If something *still* looks stale, you're probably not on the mount
  (`MB_NO_MOUNT` set) or it's a genuinely compiled change.
- **Drive is a removable SanDisk (E:).** It can **lock/unmount mid-session** — `E:` then shows
  only a ~6 MB "SanDisk Drive Unlock" partition and the whole project "disappears". It's not
  data loss; re-run the unlock and the repo returns. Don't panic-`git` against a missing path.
- **Repo git root** is `2.CodeRepos/minibunker-workshop/` (the subdir), **not** the SpaceSmSc
  workspace root. Run git with `git -C <repo>` or from inside.
- **`.sh` on Windows** run as `& "C:\Program Files\Git\bin\bash.exe" ./script.sh`.
- **Videos in git:** `assets/*.mp4` (~36 MB) are committed as plain binaries (`.gitattributes`
  marks `*.mp4 binary`, no LFS). Fine for now; consider Git LFS if history bloat matters.
- **Uncommitted at handover:** the README H1 was edited to **"AgileX MiniBunker 2.0 Workshop"**
  (working-tree only, not yet committed) — commit or revert as you prefer.
- **Submodule case-collision** (`box_link.STL` / `box_Link.STL`) in `ugv_gazebo_sim` is still
  silenced locally with `git update-index --skip-worktree`; harmless on Linux/Docker.

---

*Space Summer School · Technical University of Crete · SenseLAB — handover plan, 2026-06-25.*
