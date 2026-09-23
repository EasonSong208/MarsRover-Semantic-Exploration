
# Hiwonder Jetson Robot

This repository contains the bringup, logging, ROS2 integration, and learning pipeline for a Hiwonder robot based on Jetson Orin NX.

## Current Goal

M1: Complete an approximately 20 m closed-loop route, return near the start,
and use a color marker for final visual homing.

The current engineering task is to stabilize interfaces and staged verification;
it is not yet the final 20 m motion test. See:

- [Project context](docs/PROJECT_CONTEXT.md)
- [Current status](docs/CURRENT_STATUS.md)
- [Roadmap](docs/ROADMAP.md)
- [M1 test plan](docs/M1_20M_RETURN_HOME.md)

## Hardware

- Main controller: Jetson Orin NX 16GB
- Robot platform: Hiwonder robot
- Camera: TBD
- Motor interface: TBD
- Encoder / joint-state interface: TBD

## Current Status

- [X] Official demo runs
- [ ] Joint / encoder state readable
- [ ] ROS2 `/joint_states` available
- [ ] Encoder logging script available
- [ ] Camera + action + joint-state recording available

## Directory Structure

```text
docs/           Project notes, architecture records, decisions
ros2_ws/        ROS2 workspace (robot_mission, semantic_perception)
configs/        Machine-local config template (local.example.yaml)
vendor_patches/ Documented vendor-driver patches
```


## First Milestone

M0: Make the robot observable.

The first target is not model training.

The first target is:

Run official demo.
Read joint / encoder state.
Save timestamped joint states.
Publish joint states to ROS2.
Record one reproducible demo.

## License

Original code authored for this repository is licensed under the MIT License.
See [LICENSE](LICENSE) for details.

Third-party and vendor source code included in this repository, including the
Hiwonder ROS 2 packages under `ros2_ws/src/`, remains subject to its original
licensing terms and is not relicensed under the MIT License by this repository.

See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for details.
