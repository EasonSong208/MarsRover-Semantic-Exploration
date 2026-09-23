# Camera pose guard no-motion root-cause audit — 2026-07-19

## Scope and safety

- Hardware: JetRover, Jetson Orin NX, ROS 2 Humble.
- Jetson checkout: `/home/ubuntu/hiwonder-jetson-robot`.
- Revision: `6eac6a390d801dd689b9a98eebc26a8a8767546e`, with pre-existing uncommitted project files.
- The vendor APP service remained inactive. Nav2, joystick, init pose, and chassis controllers were not started.
- No `/cmd_vel` or `/controller/cmd_vel` publisher was created and neither topic existed at test completion.
- The user explicitly authorized hardware-node control and a bounded arm test after confirming that the work area was safe.
- The only actuation test targeted bus servo 2, from measured pulse 398 to 430 over 1.0 s. Because torque was disabled, no movement occurred and the measured position remained 398; therefore no recovery command was required.

Raw terminal captures from the audit were retained on the Jetson under
`/tmp/camera_pose_guard_root_cause_20260719/`.

## Runtime graph and command evidence

`ros_robot_controller.launch.py` started one hardware node,
`/ros_robot_controller`. It opened `/dev/rrc`, which resolved to
`/dev/ttyACM0`, at 1,000,000 baud. The launch has no namespace or remap and does
not require an additional servo-controller node.

With the guard and controller running, the command interface was:

- topic: `/ros_robot_controller/bus_servo/set_position`
- type: `ros_robot_controller_msgs/msg/ServosPosition`
- publisher: `/camera_pose_guard` (one)
- subscriber: `/ros_robot_controller` (one)
- QoS: reliable and volatile at both endpoints

The position feedback service was
`/ros_robot_controller/bus_servo/get_state`, provided by
`/ros_robot_controller` with type
`ros_robot_controller_msgs/srv/GetBusServoState`.

The guard ran from the project overlay, resolving to
`/home/ubuntu/hiwonder-jetson-robot/ros2_ws/build/robot_mission/robot_mission/camera_pose_guard.py`.
The guard and controller had matching `ROS_DOMAIN_ID=0`,
`ROS_LOCALHOST_ONLY=0`, Cyclone DDS configuration, and overlays.

The guard initially waited for the absent controller. Once the controller
appeared, it published the configured command and entered `SETTLING`. Feedback
then caused `FAULT: feedback_out_of_tolerance`. The current retry logic returned
from `FAULT` to `SETTLING` and republished the pose approximately every 2.4 s;
the launch was stopped to prevent further repeated commands.

The configured command was:

```yaml
duration: 1.0
position:
  - {id: 1, position: 500}
  - {id: 2, position: 765}
  - {id: 3, position: 15}
  - {id: 4, position: 150}
```

The duration is converted from seconds to milliseconds by the deployed vendor
SDK. IDs and pulse values fit the message's `uint16` fields.

## Hardware feedback and bounded test

A correct position-only `GetBusServoState` request returned `success=True` and
the following measured pulses:

| Servo | Measured | `vendor_init` target | Difference |
| --- | ---: | ---: | ---: |
| 1 | 506 | 500 | 6 |
| 2 | 398 | 765 | -367 |
| 3 | 253 | 15 | 238 |
| 4 | 152 | 150 | 2 |

The arm was therefore not already at `vendor_init`.

An independent listener observed the following single bounded ROS test message:

```yaml
duration: 1.0
position:
  - {id: 2, position: 430}
```

After two seconds, servo 2 still read 398. With the ROS controller stopped and
the SDK owning the serial device exclusively, the same SDK reported:

```text
SDK_POS_BEFORE [398]
SDK_TORQUE [0]
SDK_COMMAND_SENT 2->430 duration=1.0
SDK_POS_AFTER [398]
```

Read-only SDK checks then showed all four arm servos unloaded:

```text
[(1, [506], [0]), (2, [398], [0]), (3, [253], [0]), (4, [152], [0])]
```

This proves that the immediate no-motion cause was disabled servo torque. The
serial/STM32/bus path itself was able to return real positions and torque state.
Shell history and the available system journal did not identify who disabled the
four servos. The journal only showed that stopping `start_app_node.service` at
14:13:37 forcibly killed several vendor `servo_controller` processes; it does
not establish that those processes issued the unload command. The upstream
trigger for the disabled state is therefore UNKNOWN.

## Additional defects and non-causes

- The deployed vendor service calls nonexistent SDK methods for `get_voltage`
  (`bus_servo_read_voltage`) and `get_torque_state` (`bus_servo_read_torque`). A
  request using the voltage field raised `AttributeError` and terminated the
  controller. The actual SDK names are `bus_servo_read_vin` and
  `bus_servo_read_torque_state`. The guard's position-only request does not hit
  these branches.
- `button_scan.service` also opens `/dev/ttyACM0` through `Board()`. Its source
  does not enable SDK reception, so this audit did not establish it as the cause
  of the ignored position command. Multiple owners of a hardware serial port
  remain a latent integration risk.
- No namespace, remap, QoS, DDS-domain, stale-install, competing ROS arm
  publisher, or wrong-message-type fault was found.
- `red_marker_homing_test` rejected zero speed parameters because its validation
  requires positive values. This was independent of the arm fault and confirms
  that the combined launch cannot currently express arm-only operation cleanly.

## Root-cause classification and next gate

Classification: **M — all four bus servos had torque disabled**, with a second
M-class guard safety defect: feedback failure causes indefinite low-rate command
republication. Categories A–L were not the direct no-motion cause.

No source fix was made during this audit. Before another real pose command:

1. Add an explicit, fail-closed arm arming contract in project-owned code. Do
   not silently enable torque as a side effect of guard startup.
2. Provide a one-shot, explicitly confirmed torque-enable step for servos 1–4,
   then verify torque state and current positions before commanding a pose.
3. Cap or remove automatic pose-command retries after feedback failure.
4. Correct or wrap the two broken vendor feedback method mappings before relying
   on voltage or torque through the ROS service. A vendor-driver edit requires
   explicit agreement.
5. Add an arm-only launch path, or an `enable_base_motion=false` gate, so arm
   verification cannot create a chassis command publisher.

At audit completion `button_scan.service` was restored to active,
`start_app_node.service` remained inactive, the manually started
`ros_robot_controller` and guard were stopped, and both chassis velocity topics
were absent.
