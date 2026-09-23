# Project Context

## Purpose

This is a long-term robotics-learning project built around a Hiwonder robot with
an NVIDIA Jetson Orin NX 16GB. The near-term priority is a reliable, observable,
and repeatable robot platform. Imitation learning, reinforcement learning, and
model distillation come after the physical interfaces and evaluation procedures
are stable.

## Source of truth and machines

- GitHub repository `EasonSong208/hiwonder-jetson-robot` is the sole source of truth.
- The Windows + WSL2 checkout is the primary development environment.
- A separate checkout on the Jetson is a deployment target for ROS2 compilation
  and real-hardware verification.
- Changes flow through feature branches and pull requests; Jetson-only edits are
  not authoritative and must not become an undocumented fork.
- Generated ROS2 directories (`build/`, `install/`, and `log/`) remain local.

## Hardware and software context

- Robot: Hiwonder JetRover-class platform, currently configured as mecanum.
- Onboard computer: NVIDIA Jetson Orin NX 16GB, not Raspberry Pi.
- Middleware: ROS2 Humble on the Jetson deployment environment.
- Sensors observed so far: controller-board IMU, A1 lidar, and Dabai RGB-D camera.
- Navigation stack: Nav2 with AMCL, static map, global/local costmaps, planner,
  controller, behavior tree navigator, and velocity smoother.

## Known interface facts

- `/odom_raw` is produced by `odom_publisher` by integrating commanded velocity;
  it is not verified wheel-encoder odometry.
- `/odom` is produced by `ekf_filter_node` from command-derived odometry and IMU.
- Vendor `odom_publisher` accepts both `/cmd_vel` and `/controller/cmd_vel` and
  forwards motor commands to the controller board.
- Nav2 normally uses `/cmd_vel_nav` into `velocity_smoother`, whose output is
  `/cmd_vel` in the imported hardware launch.
- Launching both vendor bringup and the navigation launch can duplicate hardware
  drivers, EKF, lidar filters, camera nodes, and TF publishers. A single composed
  launch restored one publisher per critical topic during the M1 audit.

## Safety boundary

Repository work and read-only ROS2 inspection are distinct from real-robot motion.
No tool or agent should publish motion commands or navigation goals without the
operator's immediate, explicit approval.
