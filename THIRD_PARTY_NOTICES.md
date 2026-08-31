# Third-Party Notices

This repository's own code (the `minibunker_*` ROS packages under
`catkin_ws/src/`, `docs/`, and the config files) is licensed under the MIT
License (see [LICENSE](LICENSE)).

This project vendors the following third-party repositories as git submodules
under `catkin_ws/src/`. Each keeps its own license, unmodified, and none of
those licenses are changed by this repo's own MIT license.

## ugv_sdk

- Source: [agilexrobotics/ugv_sdk](https://github.com/agilexrobotics/ugv_sdk)
- License: Apache License 2.0
- Used for: a build-time dependency of `bunker_base`, providing the AgileX
  CAN protocol layer that the real-robot driver links against.

## bunker_ros

- Source: [agilexrobotics/bunker_ros](https://github.com/agilexrobotics/bunker_ros)
- License: BSD 3-Clause License, Copyright (c) 2020, WestonRobot
- Used for: the ROS1 driver (`bunker_base`, `bunker_bringup`, `bunker_msgs`)
  that `start_real.sh` uses to drive the real Bunker Mini over CAN.

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
