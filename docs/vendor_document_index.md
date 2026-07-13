# Vendor document index for M1

This index cross-checks official PDFs against the deployed Jetson source at
`/home/ubuntu/ros2_ws/src`. Source was read on 2026-07-12; project deployment HEAD
was `6eac6a390d801dd689b9a98eebc26a8a8767546e`, with environment
`JetRover_Mecanum`, `A1`, `Dabai`. The vendor workspace already contained unrelated
local changes; none were modified.

## Official startup entries

| Scope | PDF statement | Current source confirmation | Safety meaning |
|---|---|---|---|
| Base robot | `运动控制.pdf` uses `ros2 launch controller controller.launch.py`; tutorials often first stop `start_app_node.service` | `driver/controller/launch/controller.launch.py` composes controller board, odometry, IMU filter, EKF and servo controller | Hardware-facing; not a read-only launch |
| Full vendor app | Manuals repeatedly manage `start_app_node.service` | Service executes `ros2 launch bringup bringup.launch.py`; source also includes camera, LiDAR, app, joystick and `init_pose` | Starting/restarting can initialize actuators |
| Navigation | `2 导航教程.pdf`: `ros2 launch navigation navigation.launch.py map:=map_01` | `navigation/launch/navigation.launch.py` exists and composes base robot plus localization and Nav2 | Includes hardware and initialization, not Nav2-only |
| RGB-D | `深度相机基础课程.pdf`: direct Orbbec launch and `ros2 launch peripherals depth_camera.launch.py` | `peripherals/launch/depth_camera.launch.py` selects Dabai by environment | Sensor-facing; direct tutorial names use `/camera`, wrapper remaps to `/depth_cam` |
| LiDAR demos | `激光雷达课程.pdf`: `ros2 launch app lidar_node.launch.py debug:=true` plus `/lidar_controller/set_running` services | App source publishes `/controller/cmd_vel`; base `peripherals/launch/lidar.launch.py` is the sensor/filter entry | Demo commands can move the chassis and are prohibited in preflight |

## Jetson ↔ STM32 / RRC interface

The use manual states that the RRC controller uses an STM32F407, performs motor
PID with Hall encoder feedback, samples IMU, receives target chassis velocity and
sends calculated real-time velocity, IMU and battery data over USB serial.

Current ROS2 source confirms:

- device `/dev/rrc`, 1,000,000 baud, serial timeout 5 s;
- RRC frame `0xAA 0x55`, function byte, length byte, data and CRC-8;
- functions 0–10: system, LED, buzzer, motor, PWM servo, bus servo, key, IMU,
  gamepad, SBUS and OLED;
- motor output uses function 3 with motor index and float speed;
- ROS node publishes `~/imu_raw`, `~/joy`, `~/sbus`, `~/button`, `~/battery`;
- it subscribes to motor and servo command topics.

The current Python receive parser has no encoder, motor-speed or odometry report
handler. STM32 can use encoders internally for motor PID, but current ROS2 source
does not expose that feedback to ROS.

## Chassis velocity and odometry chain

Confirmed current Nav2 route:

```text
controller_server
  -- cmd_vel remap --> /cmd_vel_nav
velocity_smoother (OPEN_LOOP, 20 Hz, timeout 1.0 s)
  -- cmd_vel_smoothed remap --> /cmd_vel
odom_publisher
  -- kinematic conversion --> /ros_robot_controller/set_motor
ros_robot_controller
  -- RRC function 3 --> STM32 motor targets
```

Manual/direct alternatives publish `/controller/cmd_vel`:

- joystick controller;
- keyboard teleop;
- vendor app nodes such as line following and LiDAR controller;
- the motion-control tutorial's explicit `ros2 topic pub` example (prohibited in
  this project without per-command motion approval).

`odom_publisher_node.py` subscribes to both `/controller/cmd_vel` and `/cmd_vel`.
It stores those commanded velocities, integrates them at roughly 50 Hz and
publishes `/odom_raw`. It has no encoder subscription. EKF consumes `odom_raw` and
`imu`, publishes `/odom` and the `odom -> base_footprint` transform.

## Stop, timeout and startup behavior

- The motion-control PDF explicitly warns that stopping the publisher with
  Ctrl+C may leave the robot moving; it instructs sending zero first.
- `odom_publisher` has no command-age timeout and its signal shutdown handler does
  not publish zero. Its last velocity state also continues feeding `/odom_raw`.
- `ros_robot_controller_node.py` sends four motor zeros during initialization and
  on its handled `KeyboardInterrupt` path.
- That zero is not proven for crashes, `SIGTERM`, power loss, serial disconnect or
  forced termination. No host-side motor watchdog was found.
- Nav2 `velocity_smoother` has `velocity_timeout: 1.0`, but this protects only the
  Nav2 path and does not cover direct `/controller/cmd_vel` publishers.
- STM32 firmware watchdog behavior is **UNKNOWN**: neither the supplied PDFs nor
  the audited host source proves it.

## Startup actuator paths

- `bringup.launch.py` includes `init_pose.launch.py` and joystick control.
- `init_pose.yaml` selects `type: action`, action name `init`.
- `init_pose.py` runs `/home/ubuntu/software/arm_pc/ActionGroups/init` and publishes
  servo positions. This is a real arm/servo action, not merely TF initialization.
- Joystick subscribes to RRC gamepad data and publishes `controller/cmd_vel` when
  axes change; deadband is 0.1. For mecanum, its configured maximums are applied to
  x/y/yaw.
- The arm PDF documents 115200-baud half-duplex UART bus servos and warns against
  opening another servo tool while ROS servo nodes own the serial interface.

## Sensor interfaces

| Sensor | Document index | Current implementation |
|---|---|---|
| IMU | `运动控制.pdf` documents `/imu`, `sensor_msgs/msg/Imu` | RRC `/ros_robot_controller/imu_raw` → calibration → Madgwick filter → `/imu` |
| Odometry | Document identifies `/odom_raw` as `nav_msgs/msg/Odometry` and `/odom` as EKF output | `/odom_raw` is command-integrated; EKF inputs are `odom_raw` and `imu` |
| A1 LiDAR | LiDAR PDF focuses on moving demos | `sllidar_a1.launch.py` remaps driver `scan` → `/scan_raw`; `scan_to_scan_filter_chain` publishes `/scan` |
| Dabai RGB-D | Camera PDF shows direct `/camera/color`, `/camera/depth`, IR and points | Runtime audit observed `/depth_cam/rgb/image_raw`, hardware-aligned `/depth_cam/depth/image_raw`, both CameraInfo topics and `/depth_cam/depth/points` |

## Visual homing relevance

`2D视觉.pdf` provides examples for color detection, sorting and tracking, but its
launch files compose controller/servo/kinematics and its demos physically move the
arm or chassis. For M1, reuse only the perception concepts and verified camera
interface in a project-owned, observation-only node before adding any approved
alignment controller.

## Remaining static work

1. Inspect the actual STM32 firmware/source package, if supplied separately, for
   encoder telemetry and motor watchdog behavior.
2. Resolve RGB-D base-to-camera TF ownership and complete the manual RGB, depth and
   point-cloud visual checks recorded in `rgbd_chain_audit.md`.
3. Audit systemd signal delivery versus the Python `KeyboardInterrupt`-only zero
   path.
4. Add the discovered stop-path risks to preflight/design documentation before the
   first 0.5 m test. No vendor source change is proposed in this round.
