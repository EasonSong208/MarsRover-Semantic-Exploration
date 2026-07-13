# Vendor RTAB-VSLAM Baseline

Last updated: 2026-07-13

## Audit status

The vendor launch was fully audited, but the requested runtime entry was **not
executed** because static and graph preflight found multiple direct violations of
the task's safety boundary. This is a failed startup gate, not a successful static
RTAB-Map run. No LiDAR, joystick, controller, camera, RTAB-Map or arm action was
started by this audit.

The requested command is:

```bash
ros2 launch slam rtabmap_slam.launch.py
```

It is not currently a safe no-LiDAR RGB-D baseline. It composes LiDAR input,
external wheel/IMU odometry, joystick control and an arm initialization action.
It does not start an RGB-D visual-odometry node.

## 1. Launch Structure

Actual source entry:

```text
slam/launch/rtabmap_slam.launch.py
  -> GroupAction / PushRosNamespace(robot_name; current HOST="/")
     -> slam/launch/include/robot.launch.py
        -> driver/controller/launch/controller.launch.py
           -> peripherals/launch/imu_filter.launch.py
              -> imu_calib/apply_calib (node: imu_calib)
              -> imu_filter_madgwick_node (node: imu_filter)
           -> driver/controller/launch/odom_publisher.launch.py
              -> jetrover_description/launch/robot_description.launch.py
                 -> joint_state_publisher
                 -> robot_state_publisher
                 -> joint_state_publisher_gui [disabled by use_gui=false]
                 -> description RViz [disabled by use_rviz=false]
              -> ros_robot_controller/launch/ros_robot_controller.launch.py
                 -> ros_robot_controller
              -> controller/odom_publisher
           -> robot_localization/ekf_node (node: ekf_filter_node)
           -> servo_controller/launch/servo_controller.launch.py
              -> servo_controller
        -> peripherals/launch/depth_camera.launch.py [unconditional]
           -> peripherals/launch/include/dabai_dcw.launch.py
              -> /depth_cam/camera_container
              -> /depth_cam/depth_cam
        -> peripherals/launch/lidar.launch.py
           [enabled because use_depth_camera defaults false]
           -> peripherals/launch/include/sllidar_a1.launch.py
              -> sllidar_node
           -> laser_filters/scan_to_scan_filter_chain
        -> peripherals/launch/joystick_control.launch.py
           [enabled because use_joy defaults true]
           -> joystick_control
        -> controller/launch/init_pose.launch.py [unconditional]
           -> init_pose with action_name rewritten to horizontal
     -> after 10 seconds: slam/launch/include/rtabmap.launch.py
        -> rtabmap_sync/rgbd_sync
        -> rtabmap_slam/rtabmap with argument -d
```

`slam/launch/include/slam_base.launch.py` is not in this RTAB-Map include tree; it
is the separate `slam_toolbox` laser-SLAM path. No vendor README exists under
`/home/ubuntu/ros2_ws/src/slam` or elsewhere in the vendor source with matching
RTAB-Map/RGB-D SLAM content.

The top-level source constructs launch configurations inside an `OpaqueFunction`,
but:

```text
ros2 launch slam rtabmap_slam.launch.py --show-args
```

reported `No arguments`. There is no documented top-level launch argument that
safely disables the LiDAR, joystick, controller/servo stack or `init_pose` action.

The separate visualization entry is:

```text
slam/launch/rviz_rtabmap.launch.py
  -> ExecuteProcess(['rviz2', 'rviz2', '-d', slam/rviz/rtabmap.rviz])
```

The duplicated `rviz2` token is suspicious and was not runtime-tested because
`DISPLAY` was unset.

## 2. Node Graph

### Nodes the vendor launch would create

| Node | Role | Startup status |
|---|---|---|
| `/depth_cam/depth_cam` | Dabai RGB-D driver component | NOT STARTED |
| `/depth_cam/camera_container` | Component container | NOT STARTED |
| `rgbd_sync` | Approximate RGB + depth + CameraInfo synchronization | NOT STARTED |
| `rtabmap` | RTAB-Map graph/map backend | NOT STARTED |
| `ros_robot_controller` | Hardware controller/STM32 interface | NOT STARTED |
| `odom_publisher` | Integrates commanded velocity to `/odom_raw` | NOT STARTED |
| `ekf_filter_node` | Fuses `/odom_raw` and `/imu` to `/odom` | NOT STARTED |
| `imu_calib` | IMU calibration | NOT STARTED |
| `imu_filter` | Madgwick IMU filtering | NOT STARTED |
| `joint_state_publisher` | Republishes configured joint sources | NOT STARTED |
| `robot_state_publisher` | URDF TF | NOT STARTED |
| `servo_controller` | Arm servo control and joint-state path | NOT STARTED |
| `init_pose` | Immediately runs the `horizontal` arm action | BLOCKED |
| `sllidar_node` | A1 LiDAR driver | BLOCKED |
| `scan_to_scan_filter_chain` | `/scan_raw -> /scan` | BLOCKED |
| `joystick_control` | Can publish `/controller/cmd_vel` | BLOCKED |

There is no `rgbd_odometry`, `visual_odometry` or another visual-odometry node in
the launch tree.

### Pre-existing graph

Before any launch, domain 0 already exposed:

```text
/odom_publisher
/robot_state_publisher
/robot_state_publisher   (duplicate exact name)
/ros_robot_controller
/sim_servo_bridge
```

`/tf` and `/tf_static` each had two publishers named `robot_state_publisher`.
`/joint_states` had zero publishers and two subscribers. No matching local Jetson
process appeared in `ps`, so these endpoints appear to come from external DDS
participants on the shared `ROS_DOMAIN_ID=0`. Starting the vendor launch would
introduce duplicate control and TF identities.

## 3. Data Flow

The configured flow, not runtime-verified in this audit, is:

```text
/depth_cam/rgb/image_raw --------------------+
/depth_cam/rgb/camera_info ------------------+-> rgbd_sync
/depth_cam/depth/image_raw ------------------+     -> unremapped RGBD output
                                                     -> rtabmap (subscribe_rgbd=true)

/cmd_vel or /controller/cmd_vel
  -> odom_publisher (command integration)
  -> /odom_raw -------------------------------+
                                                -> ekf_filter_node -> /odom
STM32 IMU -> imu_calib -> imu_filter -> /imu --+
                                                     |
                                                     +-> rtabmap odom input

A1 LiDAR -> /scan_raw -> scan filter -> /scan -------> rtabmap

rtabmap
  -> configured RViz consumers: /map, /map_updates, /mapData, /mapGraph
  -> expected map/odom TF ownership is not explicitly configured or runtime-proven
```

Wheel odometry participates directly through `/odom`. It is not measured wheel
feedback: the project audit established that `/odom_raw` integrates commanded
velocity. IMU participates indirectly because EKF fuses `/imu` into `/odom`.
RTAB-Map itself does not configure `subscribe_imu` and disables gravity constraints
with `Optimizer/GravitySigma=0`.

This is therefore a scan-assisted RGB-D mapping configuration with external
command-derived/IMU odometry, not an RGB-D visual-odometry baseline.

## 4. Topic Table

The table distinguishes source configuration from runtime evidence. No listed
RTAB-Map endpoint was created by this audit.

| Role | Topic | Type | Publisher | Subscriber | Rate | `frame_id` | Status |
|---|---|---|---|---|---|---|---|
| RGB input | `/depth_cam/rgb/image_raw` | `sensor_msgs/msg/Image` | Dabai driver | `rgbd_sync` | Prior audit: 29.517 Hz | `depth_cam_color_optical_frame` | CONFIGURED; not observed in this run |
| Depth input | `/depth_cam/depth/image_raw` | `sensor_msgs/msg/Image` | Dabai driver | `rgbd_sync` | Prior audit: 29.609 Hz | `depth_cam_color_optical_frame` | CONFIGURED; not observed in this run |
| RGB CameraInfo | `/depth_cam/rgb/camera_info` | `sensor_msgs/msg/CameraInfo` | Dabai driver | `rgbd_sync` | Prior audit: 29.586 Hz | `depth_cam_color_optical_frame` | CONFIGURED; not observed in this run |
| Depth CameraInfo | `/depth_cam/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Dabai driver | Not explicitly remapped into `rgbd_sync` | Prior audit: 29.611 Hz | `depth_cam_color_optical_frame` | NOT USED by source remapping |
| RGB-D sync output | unremapped default | RTAB-Map RGBD message | `rgbd_sync` | `rtabmap` | UNKNOWN | UNKNOWN | NOT OBSERVED |
| Raw odometry | `/odom_raw` | `nav_msgs/msg/Odometry` | `odom_publisher` | `ekf_filter_node` | UNKNOWN | configured `odom -> base_footprint` | NOT OBSERVED from this launch |
| Filtered odometry | `/odom` | `nav_msgs/msg/Odometry` | `ekf_filter_node` in intended tree | `rtabmap` | UNKNOWN | configured `odom -> base_footprint` | Pre-existing external publisher only |
| Filtered IMU | `/imu` | `sensor_msgs/msg/Imu` | `imu_filter` | `ekf_filter_node` | UNKNOWN | `imu_link` | NOT OBSERVED |
| LiDAR input | `/scan` | `sensor_msgs/msg/LaserScan` | scan filter | `rtabmap` | UNKNOWN | `lidar_frame` | CONFIGURED in violation of scope |
| Occupancy map | `/map` | `nav_msgs/msg/OccupancyGrid` | RTAB-Map candidate | RViz | UNKNOWN | likely map; runtime UNKNOWN | RViz-configured, NOT OBSERVED |
| Map updates | `/map_updates` | map update type UNKNOWN | RTAB-Map candidate | RViz | UNKNOWN | UNKNOWN | RViz-configured, NOT OBSERVED |
| Map cloud data | `/mapData` | RTAB-Map map-data type | RTAB-Map candidate | MapCloud plugin | UNKNOWN | UNKNOWN | RViz-configured, NOT OBSERVED |
| Pose graph | `/mapGraph` | RTAB-Map graph type | RTAB-Map candidate | MapGraph plugin | UNKNOWN | UNKNOWN | RViz-configured, NOT OBSERVED |
| TF | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | RSP/EKF/camera/RTAB-Map candidates | TF consumers | UNKNOWN | multiple | Duplicate pre-existing publishers |

The names `/map`, `/map_updates`, `/mapData` and `/mapGraph` are evidence from the
vendor RViz configuration, not claims of successful runtime publication.

## 5. TF Tree

The intended structure is:

```text
map
  -> odom                         intended RTAB-Map correction; runtime UNKNOWN
  -> base_footprint               EKF dynamic TF
  -> base_link                    URDF fixed TF
  -> link1                        joint1 dynamic TF
  -> servo_link1                  fixed TF
  -> link2                        joint2 dynamic TF
  -> link3                        joint3 dynamic TF
  -> link4                        joint4 dynamic TF
  -> camera_connect_link          fixed TF
  -> depth_cam_link               fixed TF
  -> depth_cam_depth_frame        camera driver TF
  -> depth_cam_color_frame        camera driver TF
  -> depth_cam_color_optical_frame camera driver TF
```

The earlier RGB-D audit proved this chain was split because `/joint_states` had no
publisher and joint1 through joint4 were absent. The current preflight again found
zero `/joint_states` publishers, plus two exact-name robot-state publishers on TF.
Thus `base_link -> camera` remains unproven and duplicate TF ownership already
exists before startup.

No `map -> odom`, `odom -> base`, or base-to-camera `tf2_echo` result is reported
for this vendor run because the run was not started.

## 6. Parameters

Values below are only those explicitly present in
`slam/launch/include/rtabmap.launch.py`.

| Parameter | Vendor value | Evidence status |
|---|---|---|
| `frame_id` | `base_footprint` | Configured |
| `odom_frame_id` | not configured | Do not infer default |
| `map_frame_id` | not configured | Do not infer default |
| `subscribe_rgb` | not configured | `rgbd_sync` is used instead |
| `subscribe_depth` | not configured | `rgbd_sync` is used instead |
| `subscribe_rgbd` | `True` | Configured |
| `subscribe_odom` | not configured | `/odom` remapping is present |
| `subscribe_imu` | not configured | Direct IMU use not enabled |
| `subscribe_scan` | `True` | Configured; violates no-LiDAR scope |
| `approx_sync` | `True` on `rgbd_sync` | Configured |
| `approx_sync_max_interval` | `0.008` s | Configured |
| `queue_size` | `50` on `rtabmap` | Configured |
| `sync_queue_size` | not configured | Do not infer default |
| `RGBD/LinearUpdate` | not configured | Do not infer default |
| `RGBD/AngularUpdate` | not configured | Do not infer default |
| `Mem/IncrementalMemory` | not configured | Mapping invocation uses `-d`; do not infer parameter default |
| `Reg/Strategy` | `1` | Configured |
| `Reg/Force3DoF` | `true` | Configured |
| `Vis/FeatureType` | not configured | Do not infer default |
| `Grid/FromDepth` | not configured | Do not infer default |
| `Grid/RangeMax` | not configured | Do not infer default |
| `Grid/RangeMin` | `0.2` | Configured |
| `Grid/Sensor` | `true` | Configured |
| `Optimizer/GravitySigma` | `0` | IMU constraints disabled in RTAB-Map |
| image/scan/IMU QoS | launch argument `qos`, default `2` | Configured |

RTAB-Map is invoked with `-d`, which deletes its default database before startup.
This conflicts with the instruction not to modify database files and is another
reason the entry was not executed.

## 7. Static Test

**FAIL — SAFETY PREFLIGHT BLOCKED; RTAB-MAP NOT STARTED.**

| Check | Result |
|---|---|
| Source/include audit | PASS |
| No duplicate controller/TF before launch | FAIL |
| No LiDAR in launch | FAIL |
| No arm action in launch | FAIL |
| No command-capable joystick path | FAIL |
| Pure RGB-D visual odometry present | FAIL |
| Camera topics during this run | NOT OBSERVED |
| RTAB-Map nodes healthy | NOT EXECUTED |
| RTAB-Map map/cloud/graph outputs | NOT OBSERVED |
| TF chain complete | FAIL based on prior and current preflight evidence |
| Stationary drift | NOT MEASURED |
| Tracking lost messages | NOT OBSERVED; process never ran |

No startup command was run, so no rate, drift, synchronization-error or RTAB-Map
log claim is made. The full machine-local preflight record is intentionally ignored
at `logs/rtabmap_baseline_startup.log`; it is not part of the repository.

## 8. Motion Test

No real-robot motion was authorized immediately before a test, and the static gate
did not pass. No velocity publisher or control process was created by this audit.

| Phase | Result |
|---|---|
| Static 20 seconds | NOT EXECUTED |
| Forward approximately 0.5 m | NOT EXECUTED |
| Rotate approximately 30 degrees | NOT EXECUTED |
| Small 1-2 m loop | NOT EXECUTED |
| Tracking-lost count | NOT AVAILABLE |
| Loop closure | NOT OBSERVED |
| Obvious map tearing | NOT OBSERVED |
| Camera/arm vibration | NOT OBSERVED |
| Post-stop map drift | NOT OBSERVED |

## 9. Problems

Ordered by severity:

1. **CRITICAL — arm motion on startup.** `init_pose` is unconditional and the top
   launch selects action group `horizontal`. The implementation publishes servo
   actions after controller initialization.
2. **CRITICAL — LiDAR is enabled by default.** Camera launch is unconditional,
   while `use_depth_camera=false` also enables A1 LiDAR and scan filtering.
3. **CRITICAL — duplicate control/TF graph already exists.** Domain 0 exposed an
   odom controller, hardware controller and duplicate robot-state publishers before
   startup, apparently from external participants.
4. **HIGH — not a visual-odometry baseline.** There is no visual-odometry node.
   RTAB-Map consumes `/odom` and `/scan`; `/odom` is based on command integration
   plus IMU fusion.
5. **HIGH — base-to-camera TF is incomplete.** Arm joint TF remains absent because
   `/joint_states` has zero publishers.
6. **HIGH — startup deletes the default RTAB-Map database.** The `-d` argument
   conflicts with the no-database-modification boundary.
7. **HIGH — unsafe features cannot be officially disabled at the top level.**
   `--show-args` exposes no launch arguments.
8. **MEDIUM — joystick is enabled by default.** It creates a command-capable path
   with configured maxima 0.2 m/s and 0.5 rad/s, both above this task's limits.
9. **MEDIUM — visualization is scan-oriented and unverified.** The RViz file
   enables `/scan`, a slam_toolbox plugin, `/map`, `/mapData` and `/mapGraph`; its
   launch command also contains a duplicated `rviz2` token.
10. **MEDIUM — output topics and TF ownership remain runtime UNKNOWN.** Static RViz
    consumers are not evidence that RTAB-Map successfully publishes them.

## 10. Comparison with RGB-D Audit

| RGB-D audit fact | Vendor SLAM use |
|---|---|
| RGB is `/depth_cam/rgb/image_raw` | Used by `rgbd_sync` |
| Aligned depth is `/depth_cam/depth/image_raw` | Used by `rgbd_sync` |
| RGB CameraInfo is `/depth_cam/rgb/camera_info` | Used by `rgbd_sync` |
| Depth CameraInfo is `/depth_cam/depth/camera_info` | Not explicitly used |
| Actual cloud is `/depth_cam/depth/points` | Not consumed by the RTAB-Map launch |
| RGB/depth nearest stamps averaged 1.696 ms | Compatible with configured 8 ms approximate-sync interval in the earlier sample, but not re-tested here |
| Device frame sync is disabled | Vendor compensates with approximate sync, not exact sync |
| Base-to-camera TF is incomplete | Still a startup blocker because RTAB-Map uses `frame_id=base_footprint` |

The unused `depth_camera_info`, RGB and depth topic variables constructed in
`rtabmap_slam.launch.py` use `/depth_cam/color/...`, but they are never passed into
the included RTAB-Map launch. The effective remappings are the `/depth_cam/rgb/...`
paths in `launch/include/rtabmap.launch.py`, matching the measured RGB-D audit.

## 11. Next Step

**Decision: A — first learn and decompose the vendor launch.**

The immediate blocker is unsafe composition, not an RTAB-Map tuning parameter.
Before any runtime baseline, create or review a project-owned observation-only
wrapper that explicitly excludes LiDAR, joystick, `init_pose`, duplicate controller
bringup and database deletion. This audit did not create that wrapper because the
task forbids vendor changes and asks only for audit/reporting.

After the launch boundary is made safe, **B — fix TF** is the next runtime gate:
provide authoritative fixed arm joint states or a reviewed fixed mapping-pose
extrinsic so `base_footprint -> depth_cam_color_optical_frame` is complete.

Do not tune RTAB-Map parameters yet. There is no valid static baseline from which
to judge parameter changes, RGB-D synchronization was already within the configured
8 ms approximate interval in the previous audit, and no motion/GUI evidence exists.

The three most important files to study next are:

1. `/home/ubuntu/ros2_ws/src/slam/launch/rtabmap_slam.launch.py`
2. `/home/ubuntu/ros2_ws/src/slam/launch/include/robot.launch.py`
3. `/home/ubuntu/ros2_ws/src/slam/launch/include/rtabmap.launch.py`
