# Ground deadband probe: motor messages present, no physical motion observed

Date: 2026-07-19 (Asia/Shanghai)

## Purpose

Run the separately approved, bounded `0.08 m/s` ground probe proposed after the
`0.03 m/s` test produced no visible motion. Capture the translated motor command
so that a ROS-side failure can be distinguished from a failure downstream of the
motor-command topic.

## Software and deployment

- WSL checkout: `/home/song_eason/codep/hiwonder-jetson-robot`
- WSL Git revision: `05c68621ef1cb27765c7c876bf6133102ddd480c`
- Jetson checkout: `/home/ubuntu/hiwonder-jetson-robot`
- Jetson Git revision: `6eac6a390d801dd689b9a98eebc26a8a8767546e`
- Nine selected uncommitted files were deployed with targeted `rsync -aR`; their
  WSL and Jetson SHA-256 values matched before the build.
- No commit or push was made.

The immutable probe envelope was:

```text
/cmd_vel linear.x = 0.08 m/s
nonzero duration <= 0.25 s at 20 Hz
all other Twist components = 0
zero-command tail >= 2.0 s
ideal displacement <= 0.020 m
```

Offline verification before the run:

- WSL package test suite: `207 passed`
- Jetson `colcon build --packages-select robot_mission --symlink-install`: passed
- Jetson package test suite: `197 passed`

## Pre-test state

The Jetson had rebooted during the test session, causing
`start_app_node.service` and `button_scan.service` to reactivate. An attempted
project minimal chain immediately encountered the duplicate `/dev/rrc` access
and was interrupted before any motion command. Both automatic services were then
stopped and the graph was cleaned before this probe.

At the final gate:

- `start_app_node.service`: inactive
- `button_scan.service`: inactive
- `/dev/rrc`: exactly one owner, PID `6232`, the project minimal-bringup
  `ros_robot_controller`
- `/ros_robot_controller` node count: 1
- `/odom_publisher` node count: 1
- `/cmd_vel`: 0 publishers, 1 subscriber
- `/controller/cmd_vel`: 0 publishers, 1 subscriber
- `/ros_robot_controller/set_motor`: 1 publisher, 1 subscriber
- battery before the final run: `11027 mV`

The minimal chain used:

```bash
ros2 launch robot_mission minimal_bringup.launch.py \
  enable_lidar:=false \
  enable_camera:=false \
  enable_imu:=false \
  enable_odom:=true \
  enable_ekf:=false
```

## Approved physical command

The operator requested the retry and retained responsibility for the physical
emergency stop. Exactly one retry command was executed:

```bash
ros2 run robot_mission ground_deadband_probe \
  --ros-args -p confirmed:=true
```

Software result:

- stable graph discovery completed in `0.203 s`;
- process return code: `0`;
- five nonzero `ros_robot_controller_msgs/msg/MotorsState` frames were captured;
- each nonzero frame contained motor values
  `+0.26252361747116754,+0.26252361747116754,`\
  `-0.26252361747116754,-0.26252361747116754 rps` for motors 1-4;
- the nonzero frames were followed by repeated all-zero motor frames;
- post-command `/odom_raw` position: `x=0.019750900268554687 m`;
- post-command twist: all zero;
- post-command battery: `11055 mV`.

The persistent Jetson-side text capture is:

```text
/tmp/ground_deadband_probe_retry_20260719_2004_motor.txt
```

It contains 460 lines. `/odom_raw` is command-integrated and is not physical
wheel or displacement feedback.

## Physical result

The operator's direct observation was:

```text
好像没有反应
```

No physical chassis or wheel motion was observed. Because the wording was
tentative and no independent physical sensor was present, the exact physical
displacement remains `UNKNOWN`; this test did not pass as a physical-motion test.

## Cleanup and additional observations

- the project minimal chain was stopped;
- no minimal-bringup, controller, odometry, probe, or topic-recording process
  remained;
- `/dev/rrc` was released;
- both automatic services remained inactive;
- the kernel journal for the test window contained no `ttyACM` or USB
  disconnect, reset, or error event;
- controller logs showed normal startup and no error during the command window;
- the known vendor shutdown context/thread errors occurred only during shutdown.

## Interpretation and next gate

This run proves that the project executor published the bounded `/cmd_vel`
command and that the deployed `odom_publisher` translated it into the expected
nonzero `/ros_robot_controller/set_motor` values before sending a zero tail. It
does **not** prove that the `ros_robot_controller` callback wrote the command over
serial or that the STM32 motor handler and motor power stage acted on it.

The remaining fault boundary is downstream of the observed motor-command topic:

1. the controller callback or serial write;
2. STM32 motor-command handling or a firmware deadband at `0.2625 rps`;
3. motor-driver power/enable state or wiring;
4. motor or drivetrain hardware.

Do not infer physical actuation from `/odom_raw`, and do not blindly increase the
speed.

## Follow-up generic write-path check

With both automatic services still inactive and `/dev/rrc` initially unowned,
only `ros_robot_controller` was started. One message was published to
`/ros_robot_controller/set_buzzer` with `freq=1000`, `on_time=0.1`,
`off_time=0.0`, and `repeat=1`. The operator heard an approximately ten-second
continuous sound and requested that it be stopped immediately. A second message
with `freq=0`, zero times and `repeat=0` was sent, after which the controller node
was stopped.

This observation proves that the tested ROS subscription, controller callback,
serial write, and STM32 command handling path can actuate at least one board
output. Follow-up static inspection found that the Python SDK correctly converts
`0.1 s` to the protocol value `100 ms`; this was not a time-unit conversion
error. Official Hiwonder firmware documentation states that a zero silent
interval selects continuous buzzing. The diagnostic command therefore used an
unsafe special value: `off_time=0.0`. The approximately ten seconds were the
operator-observed interval before the explicit stop, not a requested or bounded
duration. No further buzzer test is authorized by this record.

Post-test cleanup confirmed both automatic services inactive, `/dev/rrc`
unowned, and no controller, odometry, probe, or topic-publisher process present.

Official Hiwonder controller documentation identifies a separate motor-control
switch: when it is off, motors are not controlled even though the controller can
remain otherwise operational. The next gate is a stationary visual check of that
independent switch and the motor/battery wiring. Any further wheel command
requires a new, exact approval.
