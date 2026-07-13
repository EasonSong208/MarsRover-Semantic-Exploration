# Safe RGB-D RTAB-Map bringup design

Last updated: 2026-07-13

## Structure

```text
rgbd_rtabmap_bringup.launch.py
  -> pre-start read-only gate
     -> fixed_pose_confirmed must be true
     -> no duplicate controller, EKF, RSP, camera or topic publishers
     -> no LiDAR, scan, joystick, teleop, init_pose or servo nodes
  -> minimal_bringup.launch.py
     -> fixed_arm_pose=true: vendor Xacro + robot_state_publisher only
     -> ros_robot_controller
     -> IMU calibration + Madgwick filter
     -> odom_publisher
     -> EKF -> /odom
     -> Dabai depth camera
     -> LiDAR explicitly disabled
  -> four static vendor_horizontal joint transforms
  -> ready read-only gate
     -> unique required nodes and topic publishers
     -> observed odom/RGB/depth/CameraInfo frame IDs
     -> odom -> base and base -> optical TF
     -> no joint-state, scan or prohibited-node publishers
  -> rtabmap_sync/rgbd_sync
  -> rtabmap_slam/rtabmap
  -> optional plain RViz2, disabled by default
```

Either gate exits nonzero on failure and shuts the launch down. RTAB-Map actions
are returned only after the ready process exits successfully. Neither preflight
creates a publisher, action client or motion interface.

## Reused vendor components

- the installed JetRover Xacro through `robot_state_publisher`;
- `ros_robot_controller`, `odom_publisher`, IMU filter and EKF composition from
  project minimal bringup;
- `peripherals/depth_camera.launch.py` and its measured Dabai topics;
- vendor approximate-sync interval of 8 ms and queue size 50;
- 2D force constraint and gravity-disabled RTAB-Map assumptions.

The unsafe vendor `slam/launch/include/rtabmap.launch.py` is not included because
it enables scan subscription, fixes the base frame and passes `-d`.

## Explicit exclusions

No LiDAR launch, `/scan` filter, joystick, teleop, `init_pose`, `horizontal` action,
servo controller, Nav2, slam_toolbox or RGB-D visual odometry node is created. The
first version continues to consume external `/odom`, whose raw source is
command-integrated and therefore not independent encoder feedback.

## Database behavior

`database_path` is a launch argument and defaults to `$HOME/.ros/rtabmap.db`.
There is no `-d` argument and no `reset_database` option, so startup does not
automatically delete or clear an existing database.

## Important arguments

| Argument | Default | Meaning |
|---|---|---|
| `fixed_pose_confirmed` | `false` | Hard gate requiring operator confirmation of `vendor_horizontal` |
| `use_static_camera_tf` | `true` | Publish the four audited fixed joint transforms |
| `base_frame` | `base_footprint` | RTAB-Map body frame and odometry child |
| `odom_frame` | `odom` | External odometry frame |
| `map_frame` | `map` | RTAB-Map map frame |
| `camera_frame` | `depth_cam_color_optical_frame` | Measured RGB/aligned-depth frame |
| `rgb_topic` | `/depth_cam/rgb/image_raw` | Measured RGB topic |
| `depth_topic` | `/depth_cam/depth/image_raw` | Measured aligned-depth topic |
| `camera_info_topic` | `/depth_cam/rgb/camera_info` | Measured RGB calibration topic |
| `publish_tf` | `true` | Allow RTAB-Map map correction TF |
| `use_rviz` | `false` | Optional GUI; no vendor scan-oriented RViz config |

## Runtime status

Static implementation and offline tests are complete. Stationary bringup was not
executed: the current DDS domain exposed multiple controller, EKF and
robot-state-publisher identities before startup, so the pre-start gate would
correctly refuse to initialize another hardware chain. The current camera was also
not running, and `/odom` produced no message during the bounded read-only sample.

GUI RGB/depth/map/point-cloud validation remains **MANUAL REQUIRED**. No motion or
closed-loop mapping was performed.
