# RGB-D Chain Audit

Last updated: 2026-07-13

Runtime audit window: 2026-07-13 19:19-19:26 CST

## 1. Environment

| Item | Observed value |
|---|---|
| Host | `ubuntu-desktop` (deployment Jetson; network address intentionally omitted) |
| User | `ubuntu` |
| Jetson OS | Ubuntu 22.04.5 LTS, Linux `5.15.148-tegra`, aarch64 |
| ROS distribution | Humble; the interactive Jetson profile set `ROS_DISTRO=humble` |
| ROS domain | `ROS_DOMAIN_ID=0` |
| RMW selection | `RMW_IMPLEMENTATION` was unset; the environment does not explicitly pin an RMW implementation |
| Camera selection | `DEPTH_CAMERA_TYPE=Dabai`, `need_compile=False` |
| Display | `DISPLAY` was unset |
| APP service before audit | `start_app_node.service`: `inactive` |
| Initial ROS graph | No camera node or camera topic; only `/parameter_events` and `/rosout` were listed |
| Camera start command | `ros2 launch peripherals depth_camera.launch.py` from the ROS2-configured Jetson login shell |
| Camera launch PID | `16396` (audit-only instance, stopped at the end) |
| Camera driver node | `/depth_cam/depth_cam`, composable node in `/depth_cam/camera_container` |
| Wrapper package | `peripherals`, prefix `/home/ubuntu/ros2_ws/install/peripherals` |
| Driver package | `orbbec_camera`, prefix `/home/ubuntu/third_party/orbbec_ws/install/orbbec_camera` |
| Device | DaBai DCW, firmware `RD2460`, wrapper `1.5.15`, SDK `1.10.35`, USB 2.0 |
| Git state before audit | Jetson deployment checkout already contained unrelated modified and untracked project files; they were preserved |

The APP service was already inactive, so it was not stopped. Static inspection of
`peripherals/launch/depth_camera.launch.py` and
`peripherals/launch/include/dabai_dcw.launch.py` confirmed that the selected path
starts only the Dabai/Orbbec camera container. No LiDAR, RTAB-Map, SLAM, Nav2,
chassis-motion or arm-motion node was started. No rosbag was recorded and no
vendor source or configuration was changed.

After collection, PID `16396` was sent `SIGINT`. The launch and component container
exited, no matching camera process remained, and `start_app_node.service` was still
`inactive`.

## 2. RGB-D Data Flow

The observed runtime data path was:

```text
DaBai DCW hardware (USB 2.0)
  -> peripherals/depth_camera.launch.py
  -> /depth_cam/camera_container
  -> /depth_cam/depth_cam (orbbec_camera::OBCameraNodeDriver)
       -> RGB Image       /depth_cam/rgb/image_raw
       -> RGB CameraInfo  /depth_cam/rgb/camera_info
       -> aligned Depth   /depth_cam/depth/image_raw
       -> Depth CameraInfo /depth_cam/depth/camera_info
       -> XYZ PointCloud2 /depth_cam/depth/points
       -> camera TF       /tf and /tf_static
```

The static candidate `/depth_cam/depth_registered/points` was **not** present.
The actual point-cloud topic was `/depth_cam/depth/points`. No separate registered
depth image topic was published. Instead, the actual depth image, both CameraInfo
messages and the point cloud all used `depth_cam_color_optical_frame`; together
with `depth_registration=True` and `align_mode=HW`, this shows that
`/depth_cam/depth/image_raw` was the hardware-aligned D2C depth output.

IR was disabled (`enable_ir=False`) and no IR image topic was present. The driver
published uncolored XYZ points because `enable_point_cloud=True` and
`enable_colored_point_cloud=False`.

## 3. Topic Audit

Every core topic had exactly one publisher, `/depth_cam/depth_cam`. QoS was
`RELIABLE`, `VOLATILE`, automatic liveliness; the CLI reported history depth as
`UNKNOWN`. Rates below are the final values from concurrent 12-second samples.

| Role | Topic | Type | Publisher node | Rate | Encoding / payload | Resolution | `frame_id` | QoS | Status |
|---|---|---|---|---:|---|---|---|---|---|
| RGB | `/depth_cam/rgb/image_raw` | `sensor_msgs/msg/Image` | `/depth_cam/depth_cam` | 29.517 Hz | `rgb8`, step 1920 | 640 x 360 | `depth_cam_color_optical_frame` | Reliable / Volatile | PASS |
| Depth (HW aligned to color) | `/depth_cam/depth/image_raw` | `sensor_msgs/msg/Image` | `/depth_cam/depth_cam` | 29.609 Hz | `16UC1`, step 1280 | 640 x 360 | `depth_cam_color_optical_frame` | Reliable / Volatile | PASS |
| RGB CameraInfo | `/depth_cam/rgb/camera_info` | `sensor_msgs/msg/CameraInfo` | `/depth_cam/depth_cam` | 29.586 Hz | calibrated intrinsics | 640 x 360 | `depth_cam_color_optical_frame` | Reliable / Volatile | PASS |
| Depth CameraInfo | `/depth_cam/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | `/depth_cam/depth_cam` | 29.611 Hz | aligned color intrinsics | 640 x 360 | `depth_cam_color_optical_frame` | Reliable / Volatile | PASS |
| PointCloud2 | `/depth_cam/depth/points` | `sensor_msgs/msg/PointCloud2` | `/depth_cam/depth_cam` | 29.373 Hz | XYZ `FLOAT32`; uncolored; `is_dense=True` | unordered: height 1, sampled width 200164 | `depth_cam_color_optical_frame` | Reliable / Volatile | PASS |

The 12-second interval statistics were:

| Role | Min interval | Max interval | Std dev | Samples in final window | Observation |
|---|---:|---:|---:|---:|---|
| RGB | 0.003 s | 0.182 s | 0.00991 s | 298 | Continuous; no sustained outage observed |
| Depth | 0.008 s | 0.153 s | 0.00763 s | 300 | Continuous; no sustained outage observed |
| RGB CameraInfo | 0.007 s | 0.182 s | 0.00929 s | 300 | Continuous; no sustained outage observed |
| Depth CameraInfo | 0.007 s | 0.153 s | 0.00761 s | 300 | Continuous; no sustained outage observed |
| PointCloud2 | 0.007 s | 0.068 s | 0.00467 s | 297 | Continuous; no sustained outage observed |

The short minimum intervals and occasional 153-182 ms maximum gaps indicate
delivery jitter/bursting under concurrent CLI subscriptions, but all streams
continued throughout the bounded sample.

### CameraInfo

Both CameraInfo messages matched their corresponding 640 x 360 image and used
`rational_polynomial` distortion.

RGB calibration:

```text
D = [-0.01443379745, 0.003655878128, -0.0001475558092,
     -0.0003942815820, 0, 0, 0, 0]
K = [358.034912109, 0, 320.660430908,
     0, 358.034912109, 178.374984741,
     0, 0, 1]
R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
P = [358.034912109, 0, 320.660430908, 0,
     0, 358.034912109, 178.374984741, 0,
     0, 0, 1, 0]
```

Aligned-depth calibration used the same `K`, `R` and `P`, with all eight `D`
coefficients equal to zero.

## 4. TF Audit

### Designed chain from the deployed robot description

```text
base_footprint
  -> base_link                         fixed: base_joint
  -> link1                             dynamic: joint1 (MISSING at runtime)
  -> servo_link1                       fixed: servo_joint1
  -> link2                             dynamic: joint2 (MISSING at runtime)
  -> link3                             dynamic: joint3 (MISSING at runtime)
  -> link4                             dynamic: joint4 (MISSING at runtime)
  -> camera_connect_link               fixed: camera_connect_joint
  -> depth_cam_link                    fixed: depth_cam_joint
  -> depth_cam_depth_frame             camera driver, published at about 10 Hz
  -> depth_cam_color_frame             camera driver, published at about 10 Hz
  -> depth_cam_color_optical_frame     camera driver, published at about 10 Hz
```

The camera driver also published `depth_cam_depth_optical_frame`. The URDF had a
separate fixed `depth_cam_link -> depth_cam_frame` simulation link.

### Runtime result: FAIL

`view_frames` showed two disconnected TF trees. The chassis tree contained
`base_footprint -> base_link`; the camera-side tree was rooted at `link4` and
contained `link4 -> camera_connect_link -> depth_cam_link` plus the driver frames.
The four revolute arm transforms needed to connect these trees were absent.

- `/joint_states` existed only as an interface: publisher count 0, subscriber
  count 1 (`/robot_state_publisher`). A bounded 5-second echo received no message.
- `tf2_echo base_link depth_cam_color_optical_frame` failed for the full six-second
  sample with “two or more unconnected trees”.
- `tf2_echo base_footprint depth_cam_color_optical_frame` failed the same way.
- No complete base-to-camera translation, quaternion or RPY exists in the audited
  runtime graph; inventing one would be unsafe.
- The available partial `link4 -> depth_cam_color_optical_frame` transform was
  stable: translation `[-0.051, -0.012, 0.066]` m; quaternion approximately
  `[-0.001, -0.001, -0.709, 0.705]`; RPY approximately
  `[-0.065, -0.171, -90.282]` degrees.
- The camera driver's `depth_cam_link -> depth_cam_color_optical_frame` partial
  transform was stable: translation `[0.001, -0.012, 0.000]` m; quaternion
  approximately `[0.502, -0.498, 0.500, -0.499]`; RPY approximately
  `[-90.066, -0.282, -89.829]` degrees.

The camera-internal transforms were published dynamically at about 10.17 Hz even
though their sampled values were stable. URDF fixed segments appeared with the
usual `view_frames` static rate notation.

The generated TF artifacts are on the Jetson at:

```text
/tmp/rgbd_chain_audit_tf/frames_2026-07-13_19.24.09.pdf
/tmp/rgbd_chain_audit_tf/frames_2026-07-13_19.24.09.gv
```

Classification: **C — the arm-to-camera transform is not correctly complete in
the audited runtime**. Keeping the arm physically fixed does not fill revolute
joint transforms. The system still needs valid positions for at least `joint1`
through `joint4`, or a separately reviewed fixed-extrinsic strategy for the frozen
mapping pose. Do not add an ad-hoc static transform without resolving ownership
and the actual arm pose.

The `/robot_state_publisher` participant and its static interfaces remained visible
after the audit-only camera process stopped, but no local process with that command
name was found by the bounded process check. With `ROS_DOMAIN_ID=0`, its runtime
ownership should be confirmed before relying on it or launching another instance.

## 5. Synchronization

Current relevant settings and observations:

- `depth_registration=True`
- `align_mode=HW`
- `enable_depth_scale=True`
- driver log: `set align mode to ALIGN_D2C_HW_MODE`
- driver log: `Disable frame sync`
- color and aligned depth both configured for 640 x 360 at 30 fps
- `use_hardware_time` was reported as `Parameter not set`; the driver log stated
  `current time domain: system`
- Exact sync and approximate sync were not provided by a downstream synchronizer
  in this camera-only launch.

Both stamps advanced continuously and were in the same system-time domain. A
concurrent four-second capture produced 77 RGB and 76 depth stamps. For 70 depth
samples in the overlapping time range, the nearest RGB stamp delta was 0.055 to
4.879 ms, mean 1.696 ms and median 1.468 ms. There was no second-scale offset.

This nearest-neighbor measurement shows close timestamps but does not prove exact
one-to-one synchronization, especially because device frame sync was explicitly
disabled. A later RGB-D consumer should record its actual pairing behavior and use
an appropriate approximate-time policy unless exact pairing is independently
demonstrated.

## 6. Visual Verification

| Check | Result | Reason |
|---|---|---|
| RGB in `rqt_image_view` | **MANUAL REQUIRED** | Jetson SSH session had no `DISPLAY`; no GUI image was observed |
| Depth in `rqt_image_view` | **MANUAL REQUIRED** | Jetson SSH session had no `DISPLAY`; no depth response could be visually judged |
| Point cloud in RViz | **MANUAL REQUIRED** | PointCloud2 exists and is healthy at the message level, but RViz could not be observed |

No screenshots were created, and none of these checks is claimed as PASS.

For manual verification, first ensure exactly one camera instance is running. In a
Jetson desktop or correctly forwarded GUI shell:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/depth_cam/rgb/image_raw`, then `/depth_cam/depth/image_raw`. Confirm live,
correctly oriented RGB without freeze or corruption, and depth that changes with
object distance rather than remaining black, zero or fixed.

For RViz:

```bash
rviz2
```

Use `depth_cam_color_optical_frame` as Fixed Frame, add `PointCloud2`, select
`/depth_cam/depth/points`, and use a non-RGB transformer such as Axis because the
cloud contains only XYZ. Confirm continuous updates, correct orientation, stable
geometry while the camera is stationary, and sensible motion when an object moves
closer. A base-fixed RViz frame will not work until the TF defect above is fixed.

## 7. Summary and Follow-up

| Gate | Result |
|---|---|
| Actual RGB, depth and CameraInfo endpoints discovered | PASS |
| Actual PointCloud2 endpoint discovered | PASS |
| Unique publishers and stable bounded rates | PASS |
| Matching image/CameraInfo dimensions and nonempty intrinsics | PASS |
| Hardware D2C alignment and close system-time stamps | PASS, with frame sync disabled |
| Complete `base_link`/`base_footprint` to camera TF | **FAIL** |
| RGB/depth/point-cloud visual correctness | **MANUAL REQUIRED** |

Overall result: **NOT READY for SLAM or base-frame semantic fusion**. The RGB-D
message chain itself is healthy, but the required base-to-camera TF is incomplete
and all three visual checks remain unverified.

Next actions, without changing vendor camera configuration in this audit:

1. Identify the intended owner of `/robot_state_publisher` and provide valid
   `/joint_states` for `joint1` through `joint4` while the arm remains fixed. Then
   re-run both base-to-camera `tf2_echo` checks and `view_frames`.
2. If the mapping pose is intentionally frozen instead of driven by joint states,
   review and document one authoritative fixed-extrinsic design based on measured
   arm joint values; do not publish an guessed transform alongside the URDF chain.
3. Complete the exact `rqt_image_view` and RViz checks above and save evidence only
   after a human has actually observed the displays.
4. In the later RGB-D consumer, validate approximate-time pairing under load; the
   current driver publishes close stamps but explicitly disables device frame sync.
