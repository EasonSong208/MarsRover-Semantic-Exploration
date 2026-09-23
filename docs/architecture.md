# Project architecture

## Authority and deployment

```text
GitHub (authoritative)
        |
        v
WSL2 development checkout
  /home/song_eason/codep/hiwonder-jetson-robot
        |
        | feature branch + PR (normal)
        | targeted rsync + SHA-256 (temporary, explicit request only)
        v
Jetson deployment checkout
  /home/ubuntu/hiwonder-jetson-robot
        |
        +-- project ROS2 workspace: ros2_ws/
        +-- overlays vendor workspace: /home/ubuntu/ros2_ws
```

Project mission code belongs in `ros2_ws/src/robot_mission`. Vendor packages under
`/home/ubuntu/ros2_ws/src` are upstream dependencies and are not normal edit targets.

## Runtime layers

```text
Mission / diagnostics
  robot_mission: preflight, bounded smoke tests, M1-A state machine
        |
ROS2 command and observation interfaces
        |
Vendor controller, sensor, localization and navigation packages
        |
Jetson <-> STM32 RRC serial protocol (/dev/rrc, 1,000,000 baud)
        |
Motors, local motor PID, IMU, servos and other hardware
```

Perception, planning and actuation should remain separable. Observation-only nodes
must not acquire a motion publisher as an implementation convenience.

## Velocity paths

```text
Nav2 controller_server
  -> /cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel
  -> odom_publisher
  -> motor command publisher
  -> ros_robot_controller / STM32

Direct alternatives (joystick, keyboard, app demos)
  -> /controller/cmd_vel
  -> odom_publisher
  -> motor command publisher
```

Only one intended command producer may be active. `/controller/cmd_vel` bypasses
the Nav2 smoother and must be treated as a competing actuation path.

## State and localization paths

```text
STM32 IMU packet
  -> ros_robot_controller/imu_raw
  -> imu_calib
  -> imu_filter (Madgwick)
  -> /imu

/cmd_vel or /controller/cmd_vel
  -> odom_publisher command integration
  -> /odom_raw

/odom_raw + /imu
  -> ekf_filter_node
  -> /odom
  -> odom -> base_footprint TF

static map + /scan + odom TF
  -> AMCL
  -> map -> odom TF
```

The decisive limitation is that `/odom_raw` is command-derived. The ROS host code
does not expose encoder or measured wheel-speed feedback. `/odom` is useful for
state-machine integration but is not independent proof of physical travel.

## Sensor paths

```text
A1 LiDAR -> /scan_raw -> scan_to_scan_filter_chain -> /scan

Dabai RGB-D -> peripherals/depth_camera.launch.py
             -> node/camera name depth_cam
             -> /depth_cam/rgb/image_raw + camera_info
             -> /depth_cam/depth/image_raw + camera_info
             -> /depth_cam/depth/points
             -> depth_cam frames and partial TF
```

The semantic interface is isolated in the project-owned `semantic_perception`
package. The fake source remains available for interface testing; the deployed V3
source is independent and uses the same downstream contract:

```text
existing RGB Image -> fake_semantic_node (test only)
                   \-> pidnet_semantic_node (PIDNet-S hazard5 V3)
                         -> /semantic/mask (Image, reliable mono8 IDs 0-4)
                         -> /semantic/info (String, ratios/timing/FPS)
                         -> /semantic/color + /semantic/overlay (debug Image)

/semantic/mask + aligned PointCloud2 + CameraInfo + /map_cropped
  -> project-owned semantic_fusion_compat entry
  -> existing navigation.SemanticObstacleFusion implementation
  -> /semantic_cost -> existing map_cropper merge path
```

Neither semantic source starts a camera, publishes TF or exposes an actuation
interface. The PIDNet launch defaults to the audited Dabai RGB endpoint
`/depth_cam/rgb/image_raw`; it preserves the 640 x 360 input header and size. The
compatibility entry overrides only the existing backend's XYZ array conversion so
Humble's plain `N x 3` result is accepted; projection, confidence and cost-map
logic remain owned by the existing navigation implementation.

The standalone red-marker experiment consumes the already-running RGB, aligned
Depth and RGB CameraInfo streams. Two independent bounded nearest-neighbour
matchers may inspect the same messages; only the selected matcher feeds detection
and control. Sensor loss is a recoverable observation state. Stale Depth cannot
produce linear motion, and the experiment never launches a camera or controller.

The guarded variant adds this gate without adding a second vendor control stack:

```text
camera_pose_guard
  -> read current Servo1-4 pulse through GetBusServoState
  -> preload those pulses through SetBusServoState
  -> enable torque through the same ros_robot_controller owner
  -> verify torque=1 and bounded position change
  -> at most one ros_robot_controller_msgs/ServosPosition command by default
  -> /ros_robot_controller/bus_servo/set_position
  -> existing ros_robot_controller / STM32 / bus servos
  -> /camera_pose_ready (Bool, reliable + transient local)
  -> red_marker_homing final command safety gate
```

The command is authorized only when `dry_run=false`, `confirmed=true`,
`arm_torque_confirmed=true`, both vendor arm endpoints have the sole subscriber
`/ros_robot_controller`, and no other arm-command publisher, `init_pose`,
joystick or vendor `controller_manager` is present. The set-state publisher check
enumerates exact namespace-qualified endpoint owners. Its compatibility allowlist
defaults empty; only the JetRover guarded launch opts in exactly one audited,
passive `/odom_publisher`. Duplicate, unknown, similarly named or unresolved
endpoints and graph-query failures keep readiness false. The deployed Mecanum
vendor source advertises this endpoint for all chassis but calls its publish only
inside the Ackermann branch. The exception does not permit that node to command
the arm and does not apply to the position topic. Arming is fail-closed;
FAULT is terminal and process exit does not unload the servos. The separate
`camera_pose_guard_only.launch.py` contains no red-marker or chassis node. The
red-marker path independently defaults `enable_base_motion=false`, which prevents
creation of its `/cmd_vel` publisher. The guarded launch reuses the same
`vendor_init.yaml` four-joint TF values as RGB-D mapping;
it does not publish a competing direct `base_link -> depth_cam_link` edge. Each
expected `fixed_joint1_tf` through `fixed_joint4_tf` node must appear exactly once
before the guard can consider the controller path ready.

These are runtime-observed endpoints. Depth is hardware-aligned to color; the
statically suggested `/depth_cam/depth_registered/points` was absent. The camera
tree was disconnected in the original audit because four arm-joint transforms were
missing. The project RGB-D launch suppresses the vendor zero-default joint-state
publisher and supplies exactly four static transforms from the selected named
camera pose. `vendor_init` is the default `SLAM_POSE_V1`; `vendor_horizontal` is
retained as a non-default option. Vendor RSP retains fixed-URDF ownership and the
driver retains camera optical-frame ownership. A two-phase gate checks the pose
selection, graph uniqueness, sensor frames, nominal camera-link TF and completed
TF chain before RTAB-Map starts.

## Minimal bringup boundary

`robot_mission/launch/minimal_bringup.launch.py` composes:

- robot description/state publisher;
- `ros_robot_controller`;
- optional IMU calibration/filter;
- optional `odom_publisher` and EKF;
- optional LiDAR, enabled by default;
- optional RGB-D camera, disabled by default.

It excludes joystick, keyboard, `init_pose`, servo controller, app demos, Nav2 and
all project motion nodes. Starting it remains hardware-facing because vendor
drivers initialize serial/controller state.

## M1-A mission architecture

The action model is deliberately reusable:

```text
[Drive(distance_m), Turn(180 deg), Drive(distance_m)]
```

One ROS2 node owns exactly one `/cmd_vel` publisher, one `/odom` subscriber and one
fixed-rate timer. A ROS-independent policy implements:

```text
WAIT_ODOM -> START_SEGMENT -> EXECUTE_DRIVE/EXECUTE_TURN
          -> SETTLE -> FINAL_STOP -> DONE
                              \-> ABORT
```

Drive completion uses projection along the segment-start heading; cross-track
projection is monitored but not PID-corrected. Turn completion accumulates
normalized per-sample yaw changes across the `-pi/pi` boundary. Linear and angular
commands are mutually exclusive. Segment timeout, total timeout, odometry timeout,
cross-track limit and zero-command terminal states are mandatory.

## Safety boundary

Software readiness is not motion permission. Every physical command needs fresh
operator approval, a clear area, an expected trajectory, a tested stop method and
an observer near emergency power. `SIGKILL`, power loss and an unknown STM32
watchdog cannot be mitigated by Python cleanup guarantees.

## Maintained document map

- `AGENTS.md`: rules and required reading order.
- `project_status.md`: volatile status and next gate.
- `architecture.md`: stable structure and chains.
- `rgbd_chain_audit.md`: perception interface evidence and UNKNOWN items.
- `decisions/`: durable design decisions.
- Topic-specific legacy documents: detailed evidence and historical context.
