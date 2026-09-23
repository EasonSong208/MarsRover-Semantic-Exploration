# Camera pose guard passive set-state publisher allowlist

Date: 2026-07-19 (Asia/Shanghai)

## Scope

Implement the approved, restricted compatibility policy for
`/ros_robot_controller/bus_servo/set_state` without changing vendor source or the
red-marker state machine. This pass was offline-only: no ROS2 hardware node was
started and neither the arm nor chassis was commanded.

## Ownership policy

`camera_pose_guard` now retains every publisher endpoint returned by the ROS graph
instead of reducing the result to a count or a set. It constructs an owner only
from a canonical `node_namespace` and `node_name`; missing, empty, relative,
slash-containing or otherwise ambiguous identities are rejected.

For the torque/state topic, the permitted endpoint identities are:

1. the guard's own exact, namespace-qualified node identity, at most once;
2. each exact identity in `allowed_passive_set_state_publishers`, at most once.

The parameter is statically declared as a ROS string array and is initialized to
an empty string array when no launch override exists. This avoids ROS Humble's
ambiguous inference of a bare empty Python list as a byte array. The pose YAML
does not override it. The audited set of values accepted by code is itself
restricted to `/odom_publisher`; an APP, `init_pose`, controller, similar name,
or arbitrary operator-provided exception fails parameter validation. Only
`camera_guarded_red_marker_homing.launch.py` explicitly passes:

```yaml
allowed_passive_set_state_publishers:
  - /odom_publisher
```

The arm-only launch does not opt in this exception.

Unknown publishers, duplicate guard or passive endpoints, two
`/odom_publisher` endpoints, unresolved identities, graph API exceptions, and
subscriber mismatches all reset the graph-stability count and return not-ready.
If the pose runner was already ready, loss of graph readiness transitions it to
`DEGRADED`, publishes `camera_pose_ready=false`, and the existing red-marker final
gate continues forcing chassis commands to zero.

## Static vendor evidence

The deployed source was inspected read-only at:

```text
/home/ubuntu/ros2_ws/src/driver/controller/controller/odom_publisher_node.py
SHA-256: 412092729c5ca86897ee2f8502c3b70098250e76a9d9f2410c8667cbdefe8903
```

The Jetson environment reported `MACHINE_TYPE=JetRover_Mecanum`. Relevant source
paths in that exact file are:

- lines 97 and 103 select `machine_type` from `MACHINE_TYPE`;
- line 127 unconditionally creates `servo_state_pub` on
  `ros_robot_controller/bus_servo/set_state`;
- lines 195-210 implement `JetRover_Mecanum` and publish only the Mecanum
  `MotorsState` result through `motor_pub`;
- lines 215-234 implement `JetRover_Acker`; the sole
  `self.servo_state_pub.publish(data)` call is line 234 inside that Ackermann
  branch and is conditional on a non-`None` steering-servo command.

Therefore the current Mecanum node advertises the set-state endpoint but cannot
reach its publish call through the deployed `cmd_vel_callback`. The exception is
specific to that static code and machine selection; it is not a general trust of
nodes named like odometry publishers.

The read-only check also confirmed both automatic services inactive and
`/dev/rrc` unowned. No vendor file was changed.

## Tests

Focused guard, state-machine and torque-arming tests:

```bash
cd ros2_ws/src/robot_mission
python3 -m pytest -q \
  test/test_camera_pose_guard_static.py \
  test/test_camera_pose_guard_state_machine.py \
  test/test_arm_torque_arming.py
```

Result: `41 passed`.

The focused policy coverage includes:

- guard alone: allowed;
- guard plus one exact allowed `/odom_publisher`: allowed;
- empty allowlist plus `/odom_publisher`: rejected;
- allowed `/odom_publisher` plus an unknown or similar publisher: rejected;
- two `/odom_publisher` endpoints: rejected;
- unresolved endpoint identity: rejected;
- unreviewed allowlist configuration: rejected;
- graph loss after readiness: transitions to not-ready.

Complete package suite:

```bash
cd ros2_ws/src/robot_mission
python3 -m pytest -q
```

Result: `219 passed`.

## Jetson read-only graph verification commands

After a future reviewed deployment and while the intended nodes are already
running, use a separate terminal without publishing anything:

```bash
source /opt/ros/humble/setup.zsh
source /home/ubuntu/ros2_ws/install/setup.zsh
source /home/ubuntu/hiwonder-jetson-robot/ros2_ws/install/setup.zsh
export ROS_DOMAIN_ID=0

ros2 topic info /ros_robot_controller/bus_servo/set_state -v
ros2 node info /odom_publisher
ros2 param get /camera_pose_guard allowed_passive_set_state_publishers
ros2 topic echo --once /camera_pose_ready
ros2 topic info /cmd_vel -v
ros2 topic info /controller/cmd_vel -v
ros2 topic info /cmd_vel_nav -v
```

The set-state publisher endpoints must be exactly one `/odom_publisher` plus, only
after the guard creates its lazy publisher, exactly one `/camera_pose_guard`.
Any additional endpoint or unresolved identity is a failure.

## Next hardware gates

1. Deploy only the modified project files with hash comparison; build and run the
   complete `robot_mission` tests on Jetson without starting hardware nodes.
2. Parse both guard launches with `--show-args` and confirm the arm-only launch has
   an empty allowlist while the guarded homing launch supplies only
   `/odom_publisher`.
3. With fresh arm-motion approval, start the sole vendor controller and run
   `camera_pose_guard_only.launch.py`; validate read/preload/torque enable,
   torque=1, bounded pulse jump, the one pose command, measured arrival and
   `camera_pose_ready=true`. No odometry or chassis node belongs in this gate.
4. Stop and clean that graph. With a separate fresh approval, start the intended
   Mecanum minimal bringup and guarded homing in a non-actuating configuration;
   run the read-only commands above and confirm the precise endpoint allowlist.
5. Only after those results are recorded may a separately approved full guarded
   homing attempt enable both arm and base motion.

No commit or push was made.
