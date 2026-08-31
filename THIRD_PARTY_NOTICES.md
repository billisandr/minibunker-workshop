# Third-Party Notices

This repository's own code (`real_pi/`, the `minibunker_*` ROS packages under
`catkin_ws/src/`, `docs/`, and the config files) is licensed under the MIT
License (see [LICENSE](LICENSE)).

This project vendors the following third-party repositories as git submodules
under `catkin_ws/src/`. Each keeps its own license, unmodified, and none of
those licenses are changed by this repo's own MIT license.

## ugv_sdk

- Source: [agilexrobotics/ugv_sdk](https://github.com/agilexrobotics/ugv_sdk)
- License: Apache License 2.0
- Used for: the AgileX CAN protocol reference. The real-Pi native path
  (`real_pi/minibunker_real/bunker_can.py`) reimplements the protocol-v2 CAN
  frames independently in Python, based on the frame layout documented in this
  submodule's C++ source.

## bunker_ros

- Source: [agilexrobotics/bunker_ros](https://github.com/agilexrobotics/bunker_ros)
- License: BSD 3-Clause License, Copyright (c) 2020, WestonRobot
- Used for: the ROS1 driver (`bunker_base`, `bunker_bringup`, `bunker_msgs`)
  that the Gazebo-sim/Docker path (`start_real.sh`) drives the real Bunker
  Mini through.

## ugv_gazebo_sim

- Source: [agilexrobotics/ugv_gazebo_sim](https://github.com/agilexrobotics/ugv_gazebo_sim)
- License: no license file is present in the upstream repository at the time
  of writing. Used here as an unmodified git submodule (Gazebo models and
  world assets for the Bunker and Bunker Mini) per the terms available in that
  public repository; consult the upstream repository directly for current
  licensing.

## Trained CNN weights

No trained model weights are bundled in this repository (see
`.gitignore` and `docs/TRAINING.md`). If you train and distribute your own
weights, apply whatever license you choose to that artifact separately; it
isn't covered by this notice.
