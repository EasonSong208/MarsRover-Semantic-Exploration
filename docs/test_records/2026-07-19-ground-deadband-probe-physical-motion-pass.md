# Ground deadband probe: physical motion passed

Date: 2026-07-19 (Asia/Shanghai)

## Purpose

Repeat exactly one bounded `0.08 m/s` ground probe after identifying the
controller's independent motor-control switch as the leading stationary check.
Determine whether the previously verified ROS motor-command path can produce
operator-visible physical motion when that hardware precondition is confirmed.

## Authorization and safety envelope

Immediately before the test, the operator approved the following exact command
after being asked to confirm that the independent motor-control switch was on:

```text
forward only
/cmd_vel linear.x = 0.08 m/s
nonzero duration <= 0.25 s at 20 Hz
all other Twist components = 0
zero-command tail >= 2.0 s
ideal displacement <= 0.020 m
```

The operator confirmed a clear area and retained responsibility for the physical
emergency stop.

## Software and pre-test gate

- WSL checkout: `/home/song_eason/codep/hiwonder-jetson-robot`
- Jetson checkout: `/home/ubuntu/hiwonder-jetson-robot`
- No commit or push was made.
- `start_app_node.service`: inactive
- `button_scan.service`: inactive
- `/dev/rrc`: initially unowned, then exactly one owner after minimal bringup,
  `ros_robot_controller` PID `7653`
- no residual controller, odometry, motion-probe, or topic-publisher process
- unconfirmed probe returned `2` and reported that it created no publisher and
  sent no command
- final ROS graph: only `joint_state_publisher`, `robot_state_publisher`,
  `ros_robot_controller`, and `odom_publisher`
- `/odom_publisher` was the sole subscriber of both `/cmd_vel` and
  `/controller/cmd_vel`
- battery before motion: `11097 mV`

Minimal bringup command:

```bash
ros2 launch robot_mission minimal_bringup.launch.py \
  enable_lidar:=false \
  enable_camera:=false \
  enable_imu:=false \
  enable_odom:=true \
  enable_ekf:=false
```

## Physical command and result

Exactly one approved command was executed:

```bash
ros2 run robot_mission ground_deadband_probe \
  --ros-args -p confirmed:=true
```

Software result:

- graph discovery required two stable snapshots and completed in `0.202 s`;
- process return code: `0`;
- after the zero tail, `/cmd_vel` had 0 publishers and 1 subscriber;
- after the zero tail, `/controller/cmd_vel` had 0 publishers and 1 subscriber;
- `/odom_raw` reported `x=0.019765338897705077 m` and an all-zero twist;
- post-command battery: `11128 mV`.

`/odom_raw` remains command-integrated and is not the physical evidence for this
result.

The operator's direct physical observation was:

```text
可以的，动了。没问题
```

The bounded ground probe therefore **passed as a physical-motion test**.

## Cleanup

The minimal chain was stopped after the state checks. The controller and odometry
nodes emitted their known shutdown context/thread exceptions and exited. Final
checks established:

- `/dev/rrc`: no owner;
- no minimal-bringup, controller, odometry, motion-probe, or topic-publisher
  process remained;
- `start_app_node.service`: inactive;
- `button_scan.service`: inactive.

## Interpretation

The project-owned bounded executor, `/cmd_vel -> odom_publisher` translation,
`ros_robot_controller` serial path, STM32 motor handling, enabled motor power
stage, and drivetrain jointly produced visible motion for this envelope.

This successful run is consistent with the earlier failures having occurred
while the independent motor-control hardware path was not enabled. Because no
direct before/after switch-state measurement was recorded during the failed
runs, that historical root cause remains an inference rather than a proven
observation.

This record authorizes no additional motion. A longer motion or mission remains a
separate gated hardware test requiring a new exact approval.
