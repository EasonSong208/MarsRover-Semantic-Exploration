# Official document versus deployed source

PDF statements below were extracted with `pdftotext -layout`; implementation was
checked read-only against `/home/ubuntu/ros2_ws/src` on 2026-07-12. Current source
wins when behavior differs.

| Area | Official documentation | Current Jetson source | Impact on M1 |
|---|---|---|---|
| Encoder role | Use manual says STM32 collects encoders, closes motor PID and sends calculated real-time velocity/odometry; motion course describes odometry generally as encoder + kinematics | Host RRC parser exposes no encoder/motor-speed report; `odom_publisher` integrates received commands | `/odom_raw` is not independent motion feedback; slip, stall and external displacement are invisible |
| `/odom_raw` implementation | Motion course points to `odom_publisher_node.py`, `/controller/cmd_vel` and `cal_odom_fun`, but also uses encoder language | Code directly integrates `linear_x`, `linear_y`, `angular_z` set by command callbacks | Code resolves the ambiguity: command-derived odometry |
| Topic direction wording | Motion PDF calls several `create_subscription` entries “发布话题” | Code subscribes to `set_odom`, `controller/cmd_vel`, `cmd_vel`; it publishes motors, servo state, pose and odometry | Use source API direction, not the mistranslated prose |
| Source filenames | PDF alternates between `odom_publisher.py`, `odom_publisher_node.py` and split line/path formatting | Current file is `driver/controller/controller/odom_publisher_node.py` | Automated path extraction must normalize document typos |
| Base launch name | PDF prose mentions older-looking `hiwonder_controller.launch`; commands use `controller controller.launch.py` | Current launch is `driver/controller/launch/controller.launch.py` | Use current command/package; do not rely on screenshot captions |
| Recommended velocity | Motion PDF suggests x within `-0.6..0.6` and demonstrates 0.1/0.3 m/s | Current `/cmd_vel` callback clamps x/y to ±0.2; `/controller/cmd_vel` bypasses this clamp; Nav2 smoother max x is 0.26 | Limits depend on entry topic; M1 needs one controlled path and conservative limits |
| Stop behavior | Motion PDF warns Ctrl+C may not stop and requires a separate zero command first | `odom_publisher` shutdown emits no zero and has no timeout; board node zeros only on init and handled KeyboardInterrupt | Critical safety gap. Do not rely on stopping a publisher/launch to stop motion |
| Watchdog | No definitive motor watchdog behavior found | Nav2 smoother timeout is 1 s; no host board/odom watchdog found; STM32 behavior not in audited source | Direct topic paths can retain last command; STM32 watchdog remains UNKNOWN |
| `start_app_node.service` | Tutorials stop/restart it around exercises | Service launches full `bringup`; source includes controller, camera, LiDAR, app, joystick and `init_pose` | Service state change is hardware-facing and can initialize actuators |
| `init_pose` meaning | Navigation PDF calls `bringup_launch` an initialization action; 2D vision examples visibly initialize arm actions | Current config runs action group `init` and publishes servo commands | It is physical arm motion, not harmless pose metadata |
| Navigation velocity route | Navigation PDF explains Nav2 architecture but not the exact remap chain | `controller_server` → `/cmd_vel_nav` → open-loop `velocity_smoother` → `/cmd_vel` → `odom_publisher` | `/cmd_vel_nav` is upstream; `/cmd_vel` is final Nav2-smoothed command into vendor controller |
| Joystick path | Use manual says PS2 receiver is attached to STM32 | RRC publishes Joy; joystick node publishes `/controller/cmd_vel` when axes change, with 0.1 deadband | Connected gamepad can bypass Nav2 smoother and must be neutral/controlled during tests |
| IMU filter | Motion PDF starts `peripherals imu_filter.launch.py` | Current launch uses calibration plus `imu_filter_madgwick`; complementary filter code is commented | Interpret orientation/tuning as Madgwick output |
| Camera naming | Camera course directly launches Orbbec as `/camera/...` | Vendor wrapper renames to `depth_cam` and remaps color to `/depth_cam/rgb/...` | M1 must use wrapper/runtime names, not direct-driver tutorial names |
| LiDAR tutorial | LiDAR PDF starts an app demo and calls `set_running` services | App LiDAR controller publishes `/controller/cmd_vel`; sensor-only path is `peripherals/lidar.launch.py` | Tutorial demo is a motion command path, unsuitable for interface audit |
| Servo serial layer | Arm PDF says servos use half-duplex UART at 115200 | Current ROS driver sends bus-servo RRC commands through STM32 over `/dev/rrc` at 1,000,000 baud | These are two serial layers: servo bus versus Jetson–STM32 USB serial; do not conflate baud rates |

## Confirmed RRC details

- Frame: `0xAA 0x55`, function, length, data, CRC-8.
- Host transport: `/dev/rrc`, 1,000,000 baud, five-second serial read timeout.
- Motor function: 3; bus servo: 5; IMU: 7.
- Current receive dispatch has no encoder/motor velocity packet parser.
- Manual's Hall encoders can still close STM32-local motor PID; this does not make
  the ROS `/odom_raw` encoder-derived.

## Still UNKNOWN after PDF and source review

- STM32 firmware's command-loss watchdog duration and exact stop behavior.
- Whether a separate, unaudited STM32 firmware build contains encoder telemetry
  that the current ROS2 host driver ignores.
- Whether systemd `SIGTERM` always reaches a path that transmits motor zero; the
  visible Python code guarantees zero only for its handled `KeyboardInterrupt`.
- Runtime QoS/rates and actual sensor timestamps until one approved, single-instance
  bringup is observed.

These UNKNOWN items require either STM32 firmware evidence or a later controlled
runtime test. They must not be guessed and they block treating the current stop
chain or odometry as independently fail-safe.
