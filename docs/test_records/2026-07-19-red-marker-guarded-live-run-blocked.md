# Guarded red-marker live run blocked by arm-topic ownership

Date: 2026-07-19 (Asia/Shanghai)

## Purpose and authorization

Attempt the user-approved guarded red-marker closed-loop run with a fixed red
marker, charger disconnected, cleared test area, operator-controlled physical
stop, and the following reviewed limits:

```text
forward: 0.03 m/s, nominal 0.009 m
left/right open-loop yaw: 0.10 rad/s, nominal 1.7 degrees
visual yaw maximum: 0.08 rad/s
backup: -0.03 m/s for at most 0.30 s per pulse
backup and segment zero settle: 0.75 s
total timeout: 60 s
final zero tail: at least 2 s
```

The approved run included arm torque arming and movement to `vendor_init` before
base motion.

## Pre-run gate

- `start_app_node.service`: inactive
- `button_scan.service`: inactive
- `/dev/rrc`: initially unowned, then exactly one owner,
  `ros_robot_controller` PID `8392`
- one controller, odometry, IMU/EKF, robot-description and Dabai camera chain
- `/cmd_vel`: 0 publishers, 1 subscriber
- `/controller/cmd_vel`: 0 publishers, 1 subscriber
- `/cmd_vel_nav`: absent
- RGB CameraInfo and aligned Depth messages received
- battery: `10544 mV`

Minimal bringup:

```bash
ros2 launch robot_mission minimal_bringup.launch.py \
  enable_lidar:=false \
  enable_camera:=true \
  enable_imu:=true \
  enable_odom:=true \
  enable_ekf:=true
```

## Attempted guarded command

```bash
ros2 launch robot_mission camera_guarded_red_marker_homing.launch.py \
  dry_run:=false confirmed:=true arm_torque_confirmed:=true \
  enable_base_motion:=true \
  forward_speed:=0.03 forward_distance_nominal:=0.009 \
  turn_speed:=0.10 turn_angle_deg:=1.7 yaw_max_speed:=0.08 \
  backup_speed:=-0.03 backup_pulse_duration:=0.30 \
  backup_settle_duration:=0.75 segment_stop_duration:=0.75 \
  total_timeout:=60.0 \
  output_directory:=/tmp/red_marker_homing_live_20260719_2042
```

## Result

The run was blocked before arm torque enable, arm movement, or chassis movement.
`camera_pose_guard` reported:

```text
WAIT_FOR_CONTROLLER: bus-servo torque topic already has publisher(s):
/odom_publisher
```

Consequently `/camera_pose_ready` remained false. The red-marker node created its
permitted `/cmd_vel` publisher after its command-graph check, but the final camera
pose safety gate suppressed every command. It reached `WAIT_FOR_MARKER`, had not
collected a reference, and was manually terminated after the ownership failure
persisted.

Persisted Jetson output:

```text
/tmp/red_marker_homing_live_20260719_2042/state_log.csv
/tmp/red_marker_homing_live_20260719_2042/sync_samples.csv
/tmp/red_marker_homing_live_20260719_2042/summary.json
/tmp/red_marker_homing_live_20260719_2042/sync_summary.json
```

`summary.json` recorded `camera_pose_ready=false`,
`camera_pose_gate_ready=false`, `reference=null`, and `backup_pulse_count=0`.
An independent CSV check found `nonzero_cmd_rows=0` for the actual
`cmd_linear_x` and `cmd_angular_z` columns.

## Root cause boundary

Deployed vendor `odom_publisher_node.py` unconditionally creates a
`SetBusServoState` publisher on
`/ros_robot_controller/bus_servo/set_state`. Its command callback only publishes
servo state when the selected chassis calculation returns a steering-servo
command; the Mecanum path normally returns no such command. Nevertheless, DDS
ownership alone is deliberately sufficient for the project guard to reject the
graph. This conflicts with the guard's sole arm-torque publisher rule.

Do not bypass `camera_pose_ready` or weaken the guard during a live run. The
Mecanum odometry node's unused steering-servo publisher must be explicitly
isolated or removed through a reviewed launch/vendor boundary, followed by
offline graph tests and an arm-only hardware validation.

Follow-up: the approved restricted passive-publisher allowlist was implemented
offline without modifying vendor source. See
`2026-07-19-camera-pose-guard-passive-publisher-allowlist.md`. It has not yet been
deployed or exercised on hardware.

## Cleanup

Both launches were stopped. The red-marker and camera-guard processes exited
cleanly; the vendor controller and odometry nodes emitted their known shutdown
context/thread exceptions and exited. Final checks found:

- no relevant controller, odometry, camera, guard or mission process;
- `/dev/rrc` unowned;
- both automatic services inactive.

No commit or push was made.
