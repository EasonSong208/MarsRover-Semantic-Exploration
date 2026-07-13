# Minimal safe bringup design

## Purpose and boundary

`robot_mission/launch/minimal_bringup.launch.py` composes only the vendor hardware,
state and sensor chain needed for M1 observation. It has no mission controller and
is not permission to move the robot. This round did not execute the launch.

## Arguments

| Argument | Default | Effect |
|---|---|---|
| `enable_lidar` | `true` | Include vendor A1/selected LiDAR plus scan filter |
| `enable_camera` | `false` | Include the selected RGB-D camera wrapper |
| `enable_imu` | `true` | Include calibration and Madgwick filter; raw IMU still originates in the always-required board node |
| `enable_odom` | `true` | Start command-integrating `odom_publisher` |
| `enable_ekf` | `true` | Fuse `odom_raw` and `imu`, publish `/odom` and TF |

`enable_ekf:=true` assumes both IMU and odometry inputs are enabled. Disabling an
input while leaving EKF enabled is a diagnostic configuration, not a readiness
configuration.

## Exact composition

```text
minimal_bringup.launch.py
├── include jetrover_description/robot_description.launch.py
│   ├── joint_state_publisher
│   ├── robot_state_publisher
│   ├── joint_state_publisher_gui [forced off]
│   └── rviz.launch.py [forced off, therefore not included]
├── ros_robot_controller/ros_robot_controller
├── include peripherals/imu_filter.launch.py [enable_imu]
│   ├── imu_calib/apply_calib
│   └── imu_filter_madgwick/imu_filter_madgwick_node
├── controller/odom_publisher [enable_odom]
├── robot_localization/ekf_node [enable_ekf]
├── include peripherals/lidar.launch.py [enable_lidar]
│   ├── driver selected by LIDAR_TYPE (A1 on audited robot)
│   └── laser_filters/scan_to_scan_filter_chain
└── include peripherals/depth_camera.launch.py [enable_camera=false]
    └── driver selected by DEPTH_CAMERA_TYPE (Dabai on audited robot)
```

The four include files above were recursively inspected. Vendor
`controller.launch.py`, `odom_publisher.launch.py`, `bringup.launch.py`,
`navigation.launch.py` and `slam/include/robot.launch.py` are deliberately not
included because their composition is broader and can bring in servo, joystick or
initialization actions.

## Default exclusions

- joystick and keyboard teleop;
- `init_pose` and action group `init`;
- servo controller and all servo command nodes;
- `start_app_node.service`, app server and demos;
- Nav2, localization/map server and navigation actions;
- autonomous driving, tracking, line following and transport;
- any node whose purpose is to publish a non-zero velocity.

The launch itself creates no ROS publisher or action client. The vendor
`odom_publisher` necessarily owns a motor-command publisher and velocity
subscriptions, but it publishes motor commands only after receiving a velocity.
This latent actuation path is why launch execution still requires onsite approval.

The vendor board node also calls `pwm_servo_set_offset(1, 0)` in its constructor.
That is a servo-protocol configuration write, although it is not a position target.
Consequently this composition contains no project-generated servo command and no
servo-position node, but it cannot honestly guarantee that the hardware driver
sends zero servo-protocol traffic. Removing that write would require an approved
vendor change or a vendor-supported parameter that does not currently exist.

## Confirmed and unconfirmed safety properties

Confirmed statically:

- board initialization sends four motor zeros;
- no selected launch publishes a startup non-zero chassis velocity;
- no selected include contains `init_pose` or servo controller;
- the board node performs one PWM-servo offset write at startup; no position
  command was found in this path;
- camera defaults off;
- no Nav2 goal client exists.

Not confirmed:

- STM32 command-loss watchdog behavior;
- behavior after process crash, SIGTERM, SIGKILL or serial failure;
- absence of stale commands in DDS at real startup;
- physical response of the controller board and attached gamepad;
- physical side effects, if any, of the PWM-servo offset write;
- runtime node uniqueness, topics, TF, QoS and rates.

## Later read-only hardware verification

Before launch: wheels raised or motor power safely isolated, emergency stop tested,
gamepad neutral/removed, arm clearance confirmed, and no existing equivalent nodes.
After explicit user approval, start exactly one instance and perform only node,
topic, lifecycle and TF inspection plus `robot_mission preflight`. Do not publish a
velocity or send a navigation goal. Stop immediately if any actuator initializes
unexpectedly. Passing software checks still requires separate approval for motion.
