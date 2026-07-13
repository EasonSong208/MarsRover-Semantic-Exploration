# Project status

Last updated: 2026-07-13

## Current objective

M1 aims to complete an approximately 20 m closed route, return near the origin,
then perform color-marker visual homing. The current engineering substage is M1-A:

```text
Drive(10.0 m) -> Turn(180.0 deg left) -> Drive(10.0 m)
```

M1-A is an odometry-terminated baseline, not final visual homing and not proof of
physical 20 m accuracy.

## Environments and authority

- GitHub `EasonSong208/hiwonder-jetson-robot` is the source of truth.
- Primary development checkout: WSL2,
  `/home/song_eason/codep/hiwonder-jetson-robot`.
- Jetson deployment checkout: `/home/ubuntu/hiwonder-jetson-robot`.
- Vendor ROS2 workspace on Jetson: `/home/ubuntu/ros2_ws`.
- Robot computer: NVIDIA Jetson Orin NX 16GB; ROS2 Humble; Hiwonder JetRover
  currently identified as `JetRover_Mecanum` with A1 LiDAR and Dabai RGB-D camera.
- Generated `build/`, `install/` and `log/` directories are machine-local.

Do not assume WSL2 and Jetson are currently identical. Before a target build,
compare the Git revision or explicitly selected file hashes. Targeted `rsync -aR`
has been used for temporary deployment, followed by SHA-256 verification.

## Completed and verified

- Vendor demos and navigation interfaces have run on the Jetson.
- Read-only audits established the main control, odometry, IMU, LiDAR, Nav2 and TF
  relationships summarized in `architecture.md`.
- Project-owned `robot_mission` package exists with read-only `preflight` checks.
- Minimal bringup was composed statically with camera optional and disabled by
  default. It excludes joystick, `init_pose`, servo controller, app demos and Nav2.
- Wheels-raised pure straight motion smoke test passed: 0.03 m/s, no more than
  0.3 s nonzero, automatic zero tail, no observed arm motion.
- The user reports that separate pure straight and pure in-place turn tests have
  passed on hardware.
- DDS discovery delay was measured: a new node required about 2.7 s to discover
  both `ros_robot_controller` and `odom_publisher`. Motion tests now require two
  consecutive satisfactory graph snapshots within a fixed five-second window.
- A Jetson vendor-source syntax pollution at
  `src/driver/controller/controller/odom_publisher_node.py` was isolated and
  minimally corrected from `0.0codexcodex` to `0.0`; only the `controller` package
  was cleaned and rebuilt. The pre-fix backup is outside the vendor workspace.
- M1-A `out_and_back_test` is implemented locally with pure policy tests and static
  source-safety tests. Local results: 91 tests, 0 errors, 0 failures; local colcon
  build passed.
- A bounded camera-only runtime audit established the actual Dabai RGB-D topics,
  rates, QoS, calibration, alignment and timestamp behavior. The message chain is
  healthy, but base-to-camera TF is disconnected and GUI visual checks remain open.
- The vendor RTAB-Map launch was audited without executing it. Its default tree
  starts LiDAR, joystick, controller/servo and an arm `horizontal` action, deletes
  the default RTAB-Map database, and provides no visual-odometry node; it is blocked
  as the no-LiDAR RGB-D baseline.

## Current interfaces

| Interface | Current understanding |
|---|---|
| `/cmd_vel` | Final direct mission/Nav2-smoothed chassis command consumed by vendor `odom_publisher` |
| `/controller/cmd_vel` | Direct alternative used by joystick/app paths; bypasses Nav2 smoother |
| `/cmd_vel_nav` | Nav2 controller output into `velocity_smoother` |
| `/odom_raw` | Produced by `odom_publisher` by integrating received commands; not independent encoder feedback |
| `/odom` | EKF output using command-derived odometry and filtered IMU; frame `odom`, child `base_footprint` |
| `/imu` | Filtered `sensor_msgs/msg/Imu`, from controller-board raw IMU through calibration/Madgwick |
| `/scan_raw` -> `/scan` | A1 driver output through scan filter |
| RGB-D | Dabai wrapper publishes RGB `/depth_cam/rgb/image_raw`, hardware-aligned depth `/depth_cam/depth/image_raw`, CameraInfo and XYZ cloud `/depth_cam/depth/points` at approximately 30 Hz |

## Open risks and UNKNOWN items

1. `/odom_raw` cannot observe slip, stall or external displacement. M1-A may appear
   successful in odometry while physical return error is large.
2. STM32 command-loss watchdog behavior remains UNKNOWN. Stopping a publisher or
   killing a process is not accepted as a stop mechanism.
3. Vendor `odom_publisher` shutdown does not itself guarantee a motor-zero command.
4. Duplicate bringup can create competing hardware, camera, TF and command paths.
5. The audited RGB-D TF graph is split at arm joints `joint1` through `joint4`
   because `/joint_states` has no publisher. RGB, depth and cloud visual correctness
   also remain manually unverified.
6. The vendor RTAB-Map entry is not a safe no-LiDAR RGB-D baseline: it composes
   motion-capable and LiDAR paths, deletes its default database, and has no visual
   odometry node or safe top-level disable arguments.
7. M1-A deployment parity with the Jetson is not recorded after the latest local
   implementation. It must be synchronized and rebuilt before even an unconfirmed
   invocation check.
8. Color target, illumination envelope, detection range, confidence threshold,
   loss behavior and final stand-off tolerances are not frozen.

## Next gated work

1. Review and deploy only the M1-A package files; verify hashes on Jetson.
2. Build and test `robot_mission` on Jetson without running the mission.
3. Run `out_and_back_test` with its default `confirmed:=false`; it must print the
   plan and create no publisher, subscription or timer.
4. Resolve authoritative arm joint-state/TF ownership, then repeat both
   base-to-camera TF checks and complete the manual RGB, depth and cloud views.
5. Design and statically review a project-owned observation-only RGB-D RTAB-Map
   wrapper before any SLAM runtime test; exclude LiDAR, joystick, arm actions,
   duplicate controller bringup and database deletion.
6. Only after a fresh physical-motion approval, clear-area check and stop plan,
   run M1-A at 1 m. Review physical displacement, turn, zero cleanup, odometry and
   logs before progressing through 3 m, 5 m and 10 m.

No 10 m run is currently authorized merely because the code and offline tests pass.
