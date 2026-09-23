# Ground motion pulse: no physical motion observed

Date: 2026-07-19 (Asia/Shanghai)

## Purpose

Run the first explicitly ground-scoped, bounded chassis command after reducing the
Jetson graph to one hardware owner. Determine whether the known ROS command chain
can produce visible physical motion at the existing wheels-raised smoke-test
envelope.

## Software and deployment

- WSL checkout: `/home/song_eason/codep/hiwonder-jetson-robot`
- WSL Git revision: `05c68621ef1cb27765c7c876bf6133102ddd480c`
- Jetson checkout: `/home/ubuntu/hiwonder-jetson-robot`
- Jetson Git revision: `6eac6a390d801dd689b9a98eebc26a8a8767546e`
- The working trees were not assumed identical. Five selected uncommitted files
  were deployed with targeted `rsync -aR`; their WSL and Jetson SHA-256 values
  matched before the build.
- No commit or push was made.

The deployed ground entry point reused the existing bounded smoke executor and
added no independent publisher implementation. Its immutable command was:

```text
/cmd_vel linear.x = 0.03 m/s
nonzero duration <= 0.30 s at 20 Hz
all other Twist components = 0
zero-command tail >= 2.0 s
```

## Pre-test state

The original graph was unsafe and was not used: `start_app_node.service`,
`button_scan.service`, a second controller chain, and two duplicate `init_pose`
launches were present. Before motion:

- `start_app_node.service`: inactive
- `button_scan.service`: inactive
- `/dev/rrc`: exactly one owner, project minimal-bringup
  `ros_robot_controller`
- `/ros_robot_controller` node count: 1
- `/odom_publisher` node count: 1
- `/cmd_vel` publisher count: 0
- `/controller/cmd_vel` publisher count: 0
- `/cmd_vel_nav`: absent
- joystick and `init_pose`: absent
- battery before the final gate: `10737 mV`
- controller parameter service: responsive

The minimal chain used:

```bash
ros2 launch robot_mission minimal_bringup.launch.py \
  enable_lidar:=false \
  enable_camera:=false \
  enable_imu:=false \
  enable_odom:=true \
  enable_ekf:=false
```

## Offline and target verification

- WSL package test suite: `198 passed`
- Jetson `colcon build --packages-select robot_mission --symlink-install`: passed
- Jetson package test suite: `188 passed`
- Unconfirmed invocation returned `2` after printing
  `NOT CONFIRMED: no publisher created; no command sent`.

## Approved physical command

The operator confirmed the area was safe and retained responsibility for the
physical emergency stop. Exactly one command was executed:

```bash
ros2 run robot_mission ground_motion_pulse \
  --ros-args -p confirmed:=true
```

Software result:

- stable graph discovery completed in `0.605 s`;
- process result: `GROUND_PULSE_RC=0`;
- post-command `/cmd_vel` publisher count: 0;
- post-command `/controller/cmd_vel` publisher count: 0;
- post-command battery: `10704 mV`;
- command-integrated `/odom_raw` position: `x=0.008675272464752198 m`.

The `/odom_raw` value is not wheel or encoder feedback and is not evidence of
physical travel.

## Physical result

The operator's direct observation was:

```text
现场没有任何动作。
```

Therefore this test **did not pass as a physical-motion test**. It only proved that
the ROS-side bounded executor completed and that command-integrated odometry
advanced.

## Cleanup

The minimal bringup was stopped after the zero tail. The vendor controller and
odometry nodes emitted their known shutdown-thread/context exceptions and exited
with code `1`; afterward:

- `/dev/rrc` had no owner;
- no `minimal_bringup`, `ros_robot_controller`, `odom_publisher`, or
  `ground_motion_pulse` process remained;
- both automatic services remained inactive.

## Interpretation and next gate

Static deployed-source inspection showed that `0.03 m/s` becomes approximately
`+0.09845,+0.09845,-0.09845,-0.09845 rps` for motors 1-4 and is published on
`/ros_robot_controller/set_motor`; the code does not clamp it to zero. Deployed
vendor ground applications use explicit linear speeds beginning at `0.08 m/s`,
with many using `0.15-0.30 m/s`.

The leading hypothesis is ground static friction or a firmware/driver deadband at
the 0.03 m/s, 0.30 s envelope. A downstream motor-driver, power, or serial-write
failure is still possible. The next physical gate must be a separately approved,
one-shot `0.08 m/s` for at most `0.25 s` while capturing the actual translated
`/ros_robot_controller/set_motor` message. No turn or autonomous mission is
authorized by this record.
