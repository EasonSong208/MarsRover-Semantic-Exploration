# PIDNet-S V3 semantic deployment

Date: 2026-08-06

## What changed

The project-owned `semantic_perception` package now contains an independent
`pidnet_semantic_node`. It retains the original fake node and does not modify or
start the camera, RTAB-Map, Nav2, chassis or arm control. The node loads the
five-class hazard5 V3 checkpoint, subscribes to the existing RGB stream and
publishes the established single-frame semantic mask interface.

The Jetson-only process notes were read from
`/home/ubuntu/ros2_ws/docs/debug_notes.md`. The existing downstream implementation
is `/home/ubuntu/ros2_ws/src/navigation/navigation/semantic_obstacle_fusion.py`.
On this Humble host its PointCloud2 helper returns plain `N x 3` arrays, while the
backend expects structured records. The project-owned `semantic_fusion_compat`
entry inherits that backend and overrides only XYZ conversion; no navigation
source was changed.

## Storage

The added SSD is mounted at:

```text
/media/ubuntu/c94b5031-05d7-4533-925a-ad37f0350b10
```

- Model: `jetrover_models/pidnet/best_miou_v3.pt`
- Static test data/results: `jetrover_data/pidnet_v3_test/`
- Isolated colcon build/install: `jetrover_data/pidnet_v3_test/colcon/`
- ROS and colcon logs: `jetrover_logs/semantic_perception/`

No checkpoint, inference image set or large log was added to the Git repository.

## Interfaces

| Direction | Topic | Type | Runtime contract |
|---|---|---|---|
| Input | `/depth_cam/rgb/image_raw` | `sensor_msgs/msg/Image` | `rgb8`, 640 x 360 |
| Output | `/semantic/mask` | `sensor_msgs/msg/Image` | reliable/volatile `mono8`, IDs 0-4, input header and size |
| Output | `/semantic/info` | `std_msgs/msg/String` | class ratios, latency, precision and FPS |
| Debug | `/semantic/color` | `sensor_msgs/msg/Image` | fixed-palette `rgb8` |
| Debug | `/semantic/overlay` | `sensor_msgs/msg/Image` | RGB/prediction overlay |

Classes are `0=other`, `1=hill_candidate`, `2=crater_candidate`,
`3=step_candidate`, and `4=rover`. Training-only ID 255 is rejected from model
output.

## Build and start

The verified build used SSD-backed generated directories and did not install or
upgrade dependencies:

```bash
cd /home/ubuntu/hiwonder-jetson-robot/ros2_ws
source /opt/ros/humble/setup.zsh
source /home/ubuntu/ros2_ws/install/setup.zsh
colcon --log-base \
  /media/ubuntu/c94b5031-05d7-4533-925a-ad37f0350b10/jetrover_logs/semantic_perception/colcon \
  build --base-paths src --packages-select semantic_perception \
  --build-base /media/ubuntu/c94b5031-05d7-4533-925a-ad37f0350b10/jetrover_data/pidnet_v3_test/colcon/build \
  --install-base /media/ubuntu/c94b5031-05d7-4533-925a-ad37f0350b10/jetrover_data/pidnet_v3_test/colcon/install \
  --symlink-install
```

Start only PIDNet against an already-running camera:

```bash
source /media/ubuntu/c94b5031-05d7-4533-925a-ad37f0350b10/jetrover_data/pidnet_v3_test/colcon/install/local_setup.zsh
export ROS_LOG_DIR=/media/ubuntu/c94b5031-05d7-4533-925a-ad37f0350b10/jetrover_logs/semantic_perception
ros2 launch semantic_perception pidnet_semantic.launch.py \
  model_path:=/media/ubuntu/c94b5031-05d7-4533-925a-ad37f0350b10/jetrover_models/pidnet/best_miou_v3.pt \
  input_topic:=/depth_cam/rgb/image_raw
```

Start the existing downstream through the minimal compatibility entry:

```bash
ros2 run semantic_perception semantic_fusion_compat --ros-args \
  -r __node:=semantic_obstacle_fusion_compat \
  -p semantic_mask_topic:=/semantic/mask \
  -p pointcloud_topic:=/depth_cam/depth/points \
  -p camera_info_topic:=/depth_cam/rgb/camera_info \
  -p map_ref_topic:=/map_cropped \
  -p output_topic:=/semantic_cost \
  -p map_frame:=map
```

## Verification result

- Jetson build passed; 17 package tests passed with no failure or skip.
- Three static images passed CUDA FP16 inference. Output was 360 x 640, contained
  multiple classes, contained no 255, and saved raw/mask/color/overlay artifacts.
- RGB orientation and overlay alignment were visually checked. FP16 steady static
  inference was about 34 ms after the first-frame CUDA warm-up.
- Live `/semantic/mask` was reliable/volatile `mono8`, 640 x 360, with
  `depth_cam_color_optical_frame` and the input timestamp copied by the node.
- The isolated existing backend produced a 200 x 200 semantic cost grid through
  the compatibility entry with no QoS, encoding or point-array error.
- A 180.03-second continuous run averaged 49.93 ms inference, 73.92 ms complete
  callback and 11.73 output FPS. The simultaneous monitor saw 4,000 RGB inputs and
  1,936 outputs, an estimated 51.6% latest-frame drop. FP16 remained active; no
  CUDA OOM or inference error occurred. The backend emitted 553 cost grids over
  175.1 seconds and 1,940 debug clouds over 175.8 seconds.

## Current issues and next step

The Python point-cloud backend competes for compute and reduces PIDNet throughput;
the depth-one RGB queue intentionally discards stale frames instead of building
latency. This is acceptable for the current observation gate but is the recorded
performance limitation.

The real production `/map_cropped` and `map` TF chain were not running, so full
navigation-map fusion remains unverified. The next stationary gate is to review
the saved overlays and then validate the same compatibility entry against the real
`/map_cropped` without sending a navigation goal or publishing velocity.

This was a targeted uncommitted Jetson deployment. Move the source changes through
the normal feature-branch and pull-request flow before treating it as authoritative.
