# AgileX MiniBunker 2.0 Workshop

An AgileX Bunker Mini 2.0 tracked rover that uses a Raspberry Pi 5 with a Pi
Camera and a small CNN to recognise a green ball (the target to approach) and
construction cones (hazards to avoid), then drives a reactive behaviour. It
runs identically in Gazebo simulation and on the real robot, fully
YAML-configurable, fronted by a Streamlit knob panel.

> Status: implemented (Phases 1 through 6 in code), pending a build and
> hardware validation. The sim path (detector, behaviour, Gazebo) and the
> Streamlit UI are written; the real-robot path (CAN and Pi camera) and the
> CNN training pipeline are scaffolded. The Docker image build and the real
> robot haven't been run on hardware yet. Start with
> [docs/QUICKSTART.md](docs/QUICKSTART.md).

## Gallery

| Gazebo simulation | Streamlit control panel |
| --- | --- |
| ![Gazebo arena](assets/mb_sim_gz.png) | ![Control panel](assets/mb_sim_ui.png) |
| [Watch video](assets/minibunker_sim_gz.mp4): the rover, cones, and the ball in the Gazebo arena | [Watch video](assets/minibunker_sim_ui.mp4): live camera, telemetry, ARM/DISARM, mission and WASD knobs |

## What it is

- One ROS graph, two backends. The same
  [detector](catkin_ws/src/minibunker_perception/src/detector_node.py) and
  [behaviour](catkin_ws/src/minibunker_behavior/src/behavior_node.py) nodes
  run in sim and on the real Pi; only the camera source and the velocity sink
  swap (`platform: sim | real`).
- Two perception backends behind one `vision_msgs/Detection2DArray` topic: a
  YOLOv8-nano CNN (Roboflow-trained) and a classic HSV colour detector (the
  v0 baseline), giving a CNN-vs-classic-CV teaching contrast. Flip between
  them live.
- Reactive state machine: SEARCH, then APPROACH, then AVOID, followed by
  COLLECT or STOP. Distance to the target is proxied by bounding-box size,
  since there's no depth sensor. It boots disarmed.
- Streamlit UI with live annotated camera, telemetry, live-tunable knobs, and
  an ARM/DISARM gate. Runs in a host venv over rosbridge
  ([app.py](catkin_ws/src/minibunker_ui/app.py)).
- One config file: [config/minibunker.yaml](catkin_ws/src/minibunker_bringup/config/minibunker.yaml)
  is the only file a host edits.

## Quick start

On Windows, run from PowerShell (the `.sh` scripts are invoked through Git's
bundled bash by full path; from a Git Bash prompt drop the prefix and use
`bash ./start_sim.sh`):

```powershell
git clone --recurse-submodules https://github.com/billisandr/minibunker-workshop
cd minibunker-workshop
& "C:\Program Files\Git\bin\bash.exe" ./start_sim.sh                              # Gazebo sim (HSV baseline)
& "C:\Program Files\Git\bin\bash.exe" ./catkin_ws/src/minibunker_ui/run_ui.sh     # http://localhost:8501
```

The full walkthrough, including a hardware-free synthetic loop, is in
[docs/QUICKSTART.md](docs/QUICKSTART.md).

## Repo layout

```
catkin_ws/src/
├── minibunker_perception/   detector_node (CNN+HSV), pi_camera_node, image_pub_node
├── minibunker_behavior/     behavior_node (state machine)
├── minibunker_bringup/      launch + minibunker world + camera xacro + master YAML
├── minibunker_ui/           Streamlit app + rosbridge launch
├── ugv_gazebo_sim/          (submodule) Bunker / Bunker-Mini Gazebo description
├── bunker_ros/              (submodule) real driver: bunker_base/bringup/msgs
└── ugv_sdk/                 (submodule) C++ CAN layer
docker/   start_sim.sh   start_real.sh   training/   docs/
```

## Built on

- [`agilexrobotics/ugv_gazebo_sim`](https://github.com/agilexrobotics/ugv_gazebo_sim) — Gazebo models (Bunker and Bunker-Mini)
- [`agilexrobotics/bunker_ros`](https://github.com/agilexrobotics/bunker_ros) — ROS1 driver for the real Bunker Mini
- [`agilexrobotics/ugv_sdk`](https://github.com/agilexrobotics/ugv_sdk) — C++ CAN layer

## Docs

- [docs/QUICKSTART.md](docs/QUICKSTART.md) — running the sim, the hardware-free loop, the UI, and the real robot
- [docs/HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md) — Pi 5, camera, and CAN setup
- [docs/TRAINING.md](docs/TRAINING.md) — Roboflow to YOLOv8n, training and export
- [docs/ARENA.md](docs/ARENA.md) — the physical arena and object spec

## Next steps

1. Build the image and bring the sim up (`start_sim.sh`); fix any submodule build issues.
2. Train the CNN ([docs/TRAINING.md](docs/TRAINING.md)) and validate it in sim.
3. Real-robot bring-up on the Pi ([docs/HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md)).
4. Docs: a participant handout and instructor cards, best done once the sim
   is validated so they can carry real screenshots.
