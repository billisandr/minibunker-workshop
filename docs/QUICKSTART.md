# QUICKSTART — MiniBunker

Three ways to run this, smallest first. All three use the same detector and
behaviour nodes; only the camera source and drive backend change.

> Status: code-complete through Phase 4 (sim), with Phase 5/6 scaffolding. The
> Docker image build and the real-robot bring-up have not been validated on
> hardware yet, so expect to iterate the Dockerfile and launches on first run.

---

## 0. Prerequisites

- Docker Desktop (Windows/macOS/Linux).
- For the sim GUI on Windows: VcXsrv (the `start_sim.sh` script starts it with
  `-wgl`).
- For the UI: host Python 3.9+ (a venv is created automatically by `run_ui.sh`).

Running the `.sh` scripts on Windows: the commands below are shown in the
PowerShell form that calls Git's bundled bash explicitly,
`& "C:\Program Files\Git\bin\bash.exe" ./start_sim.sh`. If your bash lives
elsewhere, adjust the path. From a Git Bash prompt you can drop the prefix and
just run `bash ./start_sim.sh`. The real-robot commands run on the Pi (Linux),
where it's always plain `bash ./start_real.sh`.

Clone with submodules, since the three AgileX repos are vendored:

```bash
git clone --recurse-submodules https://github.com/billisandr/minibunker-workshop
# already cloned without --recurse-submodules?
git submodule update --init --recursive
```

---

## 1. Gazebo simulation (no robot, no camera)

```powershell
& "C:\Program Files\Git\bin\bash.exe" ./start_sim.sh                 # builds the image first time, then launches
# HSV baseline is the default; for the CNN backend (needs a model, see TRAINING):
& "C:\Program Files\Git\bin\bash.exe" ./start_sim.sh "mb_sim backend:=cnn"
```

You should see Gazebo with the Bunker in the minibunker arena, complete with a
green ore ball and orange cones. The rover boots disarmed, so it won't move until
you arm it (see §3). Gazebo's camera plugin publishes `/camera/image_raw`, and the
Bunker's planar-move plugin consumes `/cmd_vel` and emits `/odom`.

Quick checks, in another terminal:

```bash
docker exec -it minibunker-sim bash -ic "rostopic hz /minibunker/detections"
docker exec -it minibunker-sim bash -ic "mb_cam"     # annotated debug image
```

---

## 2. Hardware-free dev loop (no Gazebo either)

The `image_pub` node can feed a synthetic scene (a moving green ball and a cone)
or a video file into `/camera/image_raw`, so the whole detect-then-behave stack
runs on any laptop with nothing attached:

```bash
docker run -it --rm minibunker bash -ic "\
  roscore & sleep 2; \
  rosparam set camera/source synthetic; \
  rosrun minibunker_perception image_pub_node.py & \
  rosrun minibunker_perception detector_node.py & \
  rosrun minibunker_behavior behavior_node.py"
```

Set `camera/source: video:/abs/path/to/demo.mp4` to loop a real clip instead.

---

## 3. The Streamlit control panel (non-coder front door)

With a station running, sim or real (both start rosbridge on `:9090`):

```powershell
& "C:\Program Files\Git\bin\bash.exe" ./catkin_ws/src/minibunker_ui/run_ui.sh      # -> http://localhost:8501
```

The panel shows the live annotated camera and telemetry, and lets you:

- Arm or disarm the safety gate. The rover publishes zero velocity until armed.
- Switch the detector backend, HSV or CNN, live.
- Tune HSV ranges, CNN confidence, follow gain, speed, stop distance, cone danger
  size, and the speed caps, all applied live (the nodes re-read these "soft" knobs
  every frame).

The single source of truth for defaults is
[`config/minibunker.yaml`](../catkin_ws/src/minibunker_bringup/config/minibunker.yaml).

---

## 4. Real robot

See [HARDWARE_SETUP.md](HARDWARE_SETUP.md) for the Pi 5, camera, and CAN setup,
then on the Pi (Linux, plain `bash`, since the Windows `bash.exe` path doesn't
apply here):

```bash
bash ./start_real.sh
```

Safety: only arm with the hardware e-stop in hand, inside the fenced arena.

---

## Option B — shared-file UI fallback (low-resource Pi)

If rosbridge is too heavy on the Pi, option B describes a shared-state file
instead: nodes dump the latest annotated JPEG and a status JSON to a tmpfs
path, and the Streamlit app polls them. This isn't wired up by default, since
option A (rosbridge) is the default, but it's documented here as the fallback to
implement if you need it.
