# Camera pose guard arming implementation verification — 2026-07-19

## Scope and safety boundary

- Primary checkout: `/home/song_eason/codep/hiwonder-jetson-robot`, local revision
  `05c68621ef1cb27765c7c876bf6133102ddd480c` with pre-existing uncommitted work.
- Jetson checkout: `/home/ubuntu/hiwonder-jetson-robot`, deployed revision
  `6eac6a390d801dd689b9a98eebc26a8a8767546e` plus targeted uncommitted files.
- This pass implemented and verified the arming policy without actuating hardware.
  It did not enable torque, publish a servo-position command, create a chassis
  command publisher, or run a physical pose test.
- `start_app_node.service` remained inactive. The temporary guard and controller
  processes were stopped after verification; `button_scan.service` was active at
  completion.

## Implemented contract

The guard is fail-closed unless `dry_run=false`, `confirmed=true`, and the new
`arm_torque_confirmed=true` gate are all present. An authorized run must then:

1. discover one `/ros_robot_controller` subscriber on both the bus-servo state and
   position command endpoints and reject competing ROS owners;
2. read the current pulse of every configured arm servo;
3. preload those exact pulses before enabling torque;
4. enable torque through the controller-owned state endpoint;
5. re-read torque and position, require torque state `1` for every servo, and
   reject any pulse jump beyond the configured tolerance;
6. publish at most one pose command by default.

Any arming or pose-feedback failure enters terminal `FAULT`. The node does not
automatically resend from `FAULT`, and shutdown does not disable servo torque.

The red-marker node independently defaults `enable_base_motion=false`; in that
state it does not create `/cmd_vel`. The new
`camera_pose_guard_only.launch.py` starts only the guard and its four static TF
publishers, with no red-marker or chassis path.

## Vendor-driver isolation

`vendor_patches/ros_robot_controller_get_bus_servo_state.patch` changes only the
two invalid service-wrapper calls:

```text
bus_servo_read_voltage -> bus_servo_read_vin
bus_servo_read_torque -> bus_servo_read_torque_state
```

The patch was dry-run checked, applied to the Jetson vendor workspace, and
`ros_robot_controller` rebuilt successfully. The repaired torque/voltage service
branches were not invoked against live hardware in this pass; runtime validation
therefore remains UNKNOWN.

## Verification evidence

- Local `colcon build --packages-select robot_mission --symlink-install`: passed.
- Local package tests: 196 passed, 0 errors, 0 failures.
- Jetson `ros_robot_controller` targeted build after the patch: passed.
- Jetson `robot_mission` targeted build and package tests: 186 passed, 0 errors,
  0 failures.
- Both guarded and arm-only launch files parsed with `--show-args` under ROS 2
  Humble. Their defaults exposed `arm_torque_confirmed=false`,
  `max_pose_command_attempts=1`, and `enable_base_motion=false` where applicable.
- A bounded Jetson default dry-run started only `camera_pose_guard` and four static
  TF nodes. `/ros_robot_controller/bus_servo/set_position`,
  `/ros_robot_controller/bus_servo/set_state`, and `/cmd_vel` were all absent.
- SIGINT cleanup completed without a Python traceback after the shutdown handling
  fix.

## Remaining live gate

Before a future hardware arming test, obtain fresh physical-motion approval and
stop `button_scan.service`, because it also opens `/dev/rrc`; this is required for
`ros_robot_controller` to be the sole OS-level serial owner, not merely the sole
ROS command endpoint. Run only the controller and arm-only launch, observe the
preload and torque verification, and keep the chassis path disabled.
