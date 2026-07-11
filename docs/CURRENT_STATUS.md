# Current Status

Last updated: 2026-07-11

## Milestone

M1 is active: approximately 20 m closed-loop travel, return near the start, then
color-marker visual homing. The current focus is engineering readiness, not the
final 20 m run.

## Confirmed capabilities

- The official Hiwonder demo and navigation stack can start on the Jetson.
- A single `navigation navigation.launch.py map:=map_01` launch can bring up the
  base hardware interfaces and Nav2 without separately running vendor bringup.
- In the most recent read-only audit, critical topics had one publisher each:
  `/scan_raw`, `/scan`, `/imu`, `/odom_raw`, and `/odom`.
- A1 lidar produced roughly 13 Hz data after a clean single-instance restart.
- `map -> odom`, `odom -> base_footprint`, and
  `base_footprint -> lidar_frame` were available.
- AMCL, map server, planner, controller, BT navigator, and velocity smoother
  reached the active lifecycle state.
- The global costmap published a map-frame costmap, and planner/navigation action
  servers were present.

## Important limitations

- `/odom_raw` is command-integrated, not encoder-derived. Distance accuracy over
  20 m is therefore unvalidated and likely affected by slip and command tracking.
- The servo controller's joint-state stream has not yet been established as
  periodic physical feedback rather than cached commanded positions.
- No global planning request or motion trial was performed during the interface
  audit, so successful action endpoints do not yet prove route execution.
- Global costmap continuous publication rate still needs a controlled observation.
- Color marker specification, camera exposure, detection range, and final docking
  tolerances are not yet frozen.

## Current risks

1. Duplicate bringup can create competing motor, odometry, IMU, lidar, camera,
   and TF publishers.
2. Command-derived odometry can look internally consistent while diverging from
   physical displacement.
3. Multiple direct publishers can target `/controller/cmd_vel`, bypassing the
   Nav2 velocity smoother and any future mission-level arbitration.
4. A 20 m test is not interpretable until short straight and turning tests have
   repeatable measurements and stop criteria.

## Immediate engineering priority

Create a project-owned `robot_mission` package skeleton and document interfaces,
test records, and configuration boundaries. Do not implement or run motion control
until the staged M1 tests and approval protocol are ready.
