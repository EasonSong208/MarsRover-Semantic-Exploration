# robot_mission

## Standalone red-marker homing experiment

`red_marker_homing_test` is an isolated RGB-D perception and bounded-motion
experiment. Its launch file starts only this node: it never starts a camera,
controller, robot description, TF, arm, joystick, Nav2, LiDAR, or RTAB-Map.
Defaults are `dry_run:=true`, `confirmed:=false`, and
`enable_base_motion:=false`. The last setting independently prevents creation of
the `/cmd_vel` publisher even if the other two are enabled. See
`docs/red_marker_homing_test_plan.md` for the safety gates and
offline workflow. RGB/Depth pairing supports bounded `direct_slop` and
`fixed_offset` modes; comparison diagnostics never invoke control twice. Missing
or stale Depth is recoverable and always suppresses linear motion.

## Guarded navigation-camera pose

`camera_guarded_red_marker_homing.launch.py` adds a project-owned
`camera_pose_guard`, the four existing `vendor_init` fixed-joint TF edges and the
red-marker node. It does not start a controller, camera, joystick, Nav2,
`init_pose`, action-group player or vendor demo. The guard waits for the already
running vendor board controller and refuses a second arm-command owner.
It also requires exactly one instance of each selected fixed-joint TF publisher;
missing or duplicate fixed TF ownership keeps readiness false.

The torque/state endpoint has a restricted passive-publisher compatibility rule.
The node declares a typed string-array parameter with an effective empty default;
the pose config does not override it. Only this guarded launch passes the exact
`/odom_publisher` identity because the audited
`JetRover_Mecanum` vendor node advertises the topic generically while its sole
publish call is confined to the Ackermann steering branch. The guard enumerates
namespace-qualified endpoint owners; it rejects duplicates, similar names,
unknown publishers, unresolved identities and graph-query failures. This is not
a `publisher_count <= 2` rule and does not relax position-command ownership.

The audited command endpoint is
`/ros_robot_controller/bus_servo/set_position` with
`ros_robot_controller_msgs/msg/ServosPosition`; duration is seconds and positions
are pulse units. Defaults are `dry_run:=true`, `confirmed:=false`, and
`arm_torque_confirmed:=false`, so the guard does not create a servo command
publisher. For a later separately authorized arm operation, the third gate
permits a fail-closed sequence: read current pulses, preload them through the sole
controller owner, enable torque, verify all four torque states and reject a
position jump, then send `vendor_init`. The pose command limit defaults to one,
FAULT never automatically retries, and node exit does not unload the servos.
Readiness requires measured position feedback; time-only readiness is disabled by
default.

`camera_pose_guard_only.launch.py` starts only the guard and selected four
fixed-joint TF publishers. It never starts red-marker homing, a chassis
controller, or a `/cmd_vel` publisher. Its defaults are non-actuating:

```bash
ros2 launch robot_mission camera_pose_guard_only.launch.py
```

Do not add all three real-arm confirmations until a separately approved hardware
test. Starting `ros_robot_controller` is hardware-facing and is intentionally
outside this launch.

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

## First ground motion pulse

`ground_motion_pulse` reuses the same hard-limited publisher, graph conflict
checks, signal handling and mandatory zero cleanup as `motion_smoke_test`, but is
an explicitly named ground-test entry point. It commands only forward
`linear.x=0.03 m/s` for no more than 0.3 seconds, then publishes zero for at least
two seconds. Unconfirmed invocation only prints the plan:

```bash
ros2 run robot_mission ground_motion_pulse
```

Only add `--ros-args -p confirmed:=true` immediately after approval for that one
ground pulse, with a clear floor and the operator controlling the physical stop.

If that 0.03 m/s pulse completes in software but produces no visible ground
motion, `ground_deadband_probe` is the next diagnostic stage. It is independently
hard-limited to forward `linear.x=0.08 m/s` for at most 0.25 seconds (ideal travel
at most 0.02 m), followed by the same two-second zero tail. Capture
`/ros_robot_controller/set_motor` during the probe to distinguish ROS translation
failure from a downstream driver, power, or mechanical issue:

```bash
ros2 run robot_mission ground_deadband_probe
```

The unconfirmed command above only prints and refuses. A confirmed invocation is
a new physical test and requires its own immediate approval.

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
fixed-arm TF and RTAB-Map path. `camera_pose:=vendor_init` is the default and is
the versioned `SLAM_POSE_V1`; `vendor_horizontal` remains available only for a
robot deliberately placed in the `horizontal.d6a` pose. The launch defaults
`fixed_pose_confirmed:=false`, so the initial read-only gate exits before any
hardware-facing include starts.

There is no automatic real-servo position verification. Do not set
`fixed_pose_confirmed:=true` unless an operator has manually confirmed that the
physical arm matches the selected `camera_pose` and the ROS domain is clear of
duplicate bringup nodes. The ready gate compares the composed
`base_link -> depth_cam_link` TF with the selected YAML, but that proves only TF
configuration consistency. The values are nominal vendor-URDF/FK extrinsics, not
a camera extrinsic calibration. Stop SLAM immediately if the arm moves.

The launch never passes RTAB-Map `-d`, so its configurable database is not deleted
on startup. See `docs/static_camera_tf_provenance.md` and
`docs/rgbd_rtabmap_bringup_design.md` before use.
