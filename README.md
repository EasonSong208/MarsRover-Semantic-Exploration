
# Hiwonder Jetson Robot

This repository contains the bringup, logging, ROS2 integration, and learning pipeline for a Hiwonder robot based on Jetson Orin NX.

## Current Goal

M0: Robot bringup and joint-state observation.

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
docs/           Project notes, hardware docs, decisions
ros2_ws/        ROS2 workspace
robot/          Low-level robot scripts and configs
data_tools/     Demonstration recording and replay
models/         Learning / inference code
experiments/    Experiment logs
configs/        Machine-specific configs
scripts/        Setup and utility scripts
tests/          Basic tests
```


# First Milestone

M0: Make the robot observable.

The first target is not model training.

The first target is:

Run official demo.
Read joint / encoder state.
Save timestamped joint states.
Publish joint states to ROS2.
Record one reproducible demo.