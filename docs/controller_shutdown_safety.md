# Controller shutdown safety analysis

Source basis: current Jetson files
`driver/ros_robot_controller/.../ros_robot_controller_node.py`,
`ros_robot_controller_sdk.py` and
`driver/controller/controller/odom_publisher_node.py`, inspected read-only on
2026-07-12. No driver was imported or connected to a real serial device.

## Exit-path matrix

| Event | Four-motor zero guaranteed by visible host code? | Reason |
|---|---|---|
| Board-node initialization | Yes | Constructor calls `set_motor_speed([[1,0], ... [4,0]])` |
| `KeyboardInterrupt` while in `rclpy.spin` | Yes, if the serial write succeeds | Explicit zero call in `except KeyboardInterrupt` before node destruction |
| Normal return from `rclpy.spin` without `KeyboardInterrupt` | No | `finally` only prints `shutdown finish` |
| SIGINT | Usually maps to ROS/Python signal handling, but not proven equivalent to the explicit `KeyboardInterrupt` branch in every launch context | Requires an offline process-level signal test and later hardware-isolated observation |
| SIGTERM/systemd stop | No visible guarantee | No SIGTERM handler and zero is not in `finally` or node destruction callback |
| Uncaught exception | No | It bypasses `except KeyboardInterrupt`; `finally` does not zero motors |
| Serial write failure/disconnect during stop | No | `port.write` exceptions are not caught/retried; the zero packet may never reach STM32 |
| SIGKILL | Impossible at application level | Kernel terminates the process immediately; Python handlers, `finally`, DDS cleanup and serial writes cannot run |

The same constructor also calls `pwm_servo_set_offset(1, 0)`. It is not a servo
position command, but it is a real RRC PWM-servo configuration packet and must be
included in the later hardware-isolated startup observation.

## Interaction with `odom_publisher`

`odom_publisher` converts each received `/cmd_vel` or `/controller/cmd_vel` into a
motor command. It has no command-age timeout. Its signal handler calls only
`rclpy.shutdown()` and does not publish motor zero. The official motion PDF also
warns that Ctrl+C may leave the robot moving and instructs sending zero first.

Nav2's velocity smoother has a one-second timeout, but that protection covers only
the `/cmd_vel_nav -> /cmd_vel` route. Joystick, keyboard and app publishers can use
`/controller/cmd_vel` directly and bypass it.

## Serial failure behavior

RRC uses `/dev/rrc` at 1,000,000 baud. `set_motor_speed` serializes RRC function 3
and immediately calls `port.write`. There is no retry, acknowledgement, stop latch
or exception-to-zero fallback in the audited host code. A disconnected serial link
can therefore raise into the caller, and the physical outcome depends on STM32
firmware state.

## Watchdog conclusion

The visible host safety is insufficient to guarantee stop under SIGTERM, crash,
SIGKILL or serial failure. Safe behavior in those cases would require an STM32-side
command watchdog or an independent hardware stop. The supplied PDF and audited
host source do not define that watchdog, so its presence and timeout remain
**UNKNOWN**.

## Offline verification performed

The test harness uses fake objects and Python AST only. It verifies that an
explicit four-motor-zero call can be recognized, that a cleanup print is not a
stop, and that a non-KeyboardInterrupt exception bypasses a KeyboardInterrupt-only
zero branch. It does not import the vendor driver or open `/dev/rrc`.

## Required follow-up before motion

1. Obtain STM32 firmware/protocol evidence for command timeout and motor stop.
2. Build a subprocess harness around a copied/minimal control-flow model to compare
   SIGINT and SIGTERM without importing hardware code.
3. With motors electrically isolated, observe actual serial packets for startup and
   each shutdown path using a mock/PTY or serial proxy.
4. Treat emergency stop and physical isolation as mandatory until watchdog behavior
   is proven.
