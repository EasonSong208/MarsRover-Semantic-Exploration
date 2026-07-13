# robot_mission

Project-owned ROS2 utilities for M1. The first executable, `preflight`, performs
read-only graph, lifecycle, action-interface, and TF checks. It does not create a
motion publisher or navigation action client.

Build on the Jetson deployment checkout:

```bash
cd ~/hiwonder-jetson-robot/ros2_ws
colcon build --packages-select robot_mission
source install/setup.bash
```

Run after the approved, single-instance robot/navigation bringup is already active:

```bash
ros2 run robot_mission preflight --ros-args \
  --params-file "$(ros2 pkg prefix robot_mission)/share/robot_mission/config/preflight.yaml"
```

The process returns `1` when a hard readiness check fails. A successful report is
still not permission to move the robot; real motion requires explicit user approval.

## Wheels-raised straight smoke test

`motion_smoke_test` is hard-limited to `linear.x=0.03 m/s` for no more than
0.3 seconds at 20 Hz, followed by at least two seconds of zero commands. Its
default unconfirmed mode prints the plan without creating a publisher:

```bash
ros2 run robot_mission motion_smoke_test
```

Only add `--ros-args -p confirmed:=true` after a fresh approval for that specific
wheels-raised test, with the mechanism clear and an operator at emergency power.

## Turn-test order

`turn_step_test` is a separate fixed approximately 30-degree ground test. It does
not replace the short-pulse, wheels-raised `turn_smoke_test`. At the time this
document was written, `turn_smoke_test` was not present in this package and must be
implemented/validated separately before any ground turn.

Required order:

1. Run the wheels-raised `turn_smoke_test` left and verify opposite wheel motion
   plus complete zero cleanup.
2. Run the wheels-raised `turn_smoke_test` right and verify the wheel directions
   are exactly reversed plus complete zero cleanup.
3. Only after both pass, place the robot on the ground for `turn_step_test`.
4. Execute only one direction on the first ground attempt. Keep an operator beside
   the emergency stop and power switch, with clear space around the robot.
5. Wait for a complete stop and inspect the full log before executing the opposite
   direction.

Without explicit confirmation the command below only prints the fixed plan and
does not create a publisher:

```bash
ros2 run robot_mission turn_step_test --ros-args -p direction:=left
```

Do not add `confirmed:=true` until all physical prerequisites above are satisfied
and the user gives immediate approval for that specific direction.

## M1-A odometry-closed-loop out and back

`out_and_back_test` represents the mission as one reusable segment sequence:

```text
Drive(distance_m) -> Turn(180 deg left) -> Drive(distance_m)
```

It publishes only to `/cmd_vel`, observes `/odom`, and never combines linear and
angular velocity. `/odom` ultimately depends on the vendor command-integrated
`/odom_raw`; it is not independent encoder feedback, so staged tests and generous
operator stop access remain mandatory. The executable defaults to
`confirmed:=false`, in which mode it prints the plan without creating its publisher,
subscription, or timer.

After a specific on-site motion approval, the first ground test must be 1 m:

```bash
ros2 run robot_mission out_and_back_test --ros-args \
  -p confirmed:=true \
  -p distance_m:=1.0 \
  -p linear_speed_mps:=0.15 \
  -p turn_angle_deg:=180.0 \
  -p angular_speed_radps:=0.20
```

Only after reviewing the complete stop, odometry, cross-track and final-error logs
at each distance may testing progress to 3 m, 5 m and finally 10 m by changing only
`distance_m`. Never begin with the 10 m default.

## Minimal bringup

`launch/minimal_bringup.launch.py` is the statically audited base/sensor
composition. It excludes joystick, servo controller, `init_pose`, application
demos, Nav2 and mission executables; camera is disabled by default. The launch is
still hardware-facing because it initializes the controller board, so do not run
it merely to inspect interfaces or without the required onsite approval.

Its exact composition, defaults, exclusions and remaining shutdown risks are in
`docs/minimal_bringup_design.md` and `docs/controller_shutdown_safety.md`.

## Gated RGB-D RTAB-Map bringup

`rgbd_rtabmap_bringup.launch.py` composes the no-LiDAR camera, external odometry,
fixed-arm TF and RTAB-Map path. It defaults `fixed_pose_confirmed:=false`; in that
state the initial read-only gate exits before any hardware-facing include starts.
Do not set it true unless the arm physically matches the documented vendor
`horizontal` pose and the ROS domain is clear of duplicate bringup nodes.

The launch never passes RTAB-Map `-d`, so its configurable database is not deleted
on startup. See `docs/static_camera_tf_provenance.md` and
`docs/rgbd_rtabmap_bringup_design.md` before use.
