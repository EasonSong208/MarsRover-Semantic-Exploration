# semantic_perception

`semantic_perception` is a project-owned ROS2 Humble package with two independent
nodes. The retained `fake_semantic_node` provides the Phase 0 random interface.
The `pidnet_semantic_node` loads the five-class PIDNet-S V3 checkpoint, consumes
an existing RGB stream and publishes real same-size semantic results. Neither node
starts a camera or exposes an actuation interface.

## Build

From the Jetson deployment checkout, after sourcing the vendor workspace:

```bash
cd /home/ubuntu/hiwonder-jetson-robot/ros2_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
rosdep install --from-paths src/semantic_perception --ignore-src -r -y
colcon build --symlink-install --packages-select semantic_perception
source install/setup.bash
```

If this Jetson reports that rosdep has never been initialized, complete the
one-time host setup before the `rosdep install` command:

```bash
sudo rosdep init
rosdep update
```

The PIDNet node additionally requires the Jetson's existing CUDA-enabled PyTorch.
This deployment does not install or upgrade PyTorch.

## PIDNet-S V3

The checkpoint is a runtime parameter and is intentionally not stored in this
repository. On the current Jetson it is kept on the SSD:

```text
/media/ubuntu/c94b5031-05d7-4533-925a-ad37f0350b10/jetrover_models/pidnet/best_miou_v3.pt
```

Run only the semantic node against an already-running Dabai RGB stream:

```bash
export ROS_LOG_DIR=/media/ubuntu/c94b5031-05d7-4533-925a-ad37f0350b10/jetrover_logs/semantic_perception
ros2 launch semantic_perception pidnet_semantic.launch.py \
  model_path:=/media/ubuntu/c94b5031-05d7-4533-925a-ad37f0350b10/jetrover_models/pidnet/best_miou_v3.pt \
  input_topic:=/depth_cam/rgb/image_raw
```

The deployment default is CUDA FP16. A CUDA runtime error during the first FP16
inference switches the node once to FP32 and logs the reason. Model-load or
checkpoint-contract failures terminate the node; they never produce a fake mask.

Static inference writes all artifacts to an explicitly selected output directory:

```bash
ros2 run semantic_perception pidnet_static_test -- \
  --input /media/ubuntu/c94b5031-05d7-4533-925a-ad37f0350b10/jetrover_data/pidnet_v3_test/input \
  --model-path /media/ubuntu/c94b5031-05d7-4533-925a-ad37f0350b10/jetrover_models/pidnet/best_miou_v3.pt \
  --output /media/ubuntu/c94b5031-05d7-4533-925a-ad37f0350b10/jetrover_data/pidnet_v3_test/static_output
```

### PIDNet interface

| Direction | Topic | Type | Contract |
|---|---|---|---|
| Input | `/depth_cam/rgb/image_raw` | `sensor_msgs/msg/Image` | `rgb8`, 640 x 360; configurable |
| Output | `/semantic/mask` | `sensor_msgs/msg/Image` | reliable/volatile `mono8`; IDs 0-4; input dimensions and header |
| Output | `/semantic/info` | `std_msgs/msg/String` | JSON class ratios, latency, precision and output FPS |
| Debug | `/semantic/color` | `sensor_msgs/msg/Image` | fixed-palette `rgb8`; input dimensions and header |
| Debug | `/semantic/overlay` | `sensor_msgs/msg/Image` | RGB/prediction overlay; input dimensions and header |

Class IDs are `0=other`, `1=hill_candidate`, `2=crater_candidate`,
`3=step_candidate`, and `4=rover`. Training-only ignore ID `255` is rejected from
inference output.

The existing backend is
`/home/ubuntu/ros2_ws/src/navigation/navigation/semantic_obstacle_fusion.py`.
It can consume the new mask without code changes by setting
`semantic_mask_topic:=/semantic/mask`. On this robot the audited point-cloud
override is `/depth_cam/depth/points`, not the backend's legacy default.

ROS2 Humble on this Jetson returns the uniform XYZ cloud as a plain `N x 3`
NumPy array, while the existing backend assumes named structured records. The
project-owned `semantic_fusion_compat` entry inherits that backend and overrides
only its XYZ conversion; confidence, projection and map-cost logic remain the
existing implementation, and no vendor source is changed:

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

## Run

The required Phase 0 input topic is the launch default:

```bash
ros2 launch semantic_perception semantic_perception.launch.py
```

The Dabai camera topic observed on this robot is different. If that existing
camera stream is already running, select it without changing the camera driver:

```bash
ros2 launch semantic_perception semantic_perception.launch.py \
  input_topic:=/depth_cam/rgb/image_raw
```

The launch file does not start the camera or any hardware-facing node.

## Verify

```bash
ros2 topic list
ros2 topic type /semantic/mask
ros2 topic type /semantic/info
ros2 topic echo /semantic/info
ros2 topic hz /semantic/mask
```

For a GUI check, run:

```bash
ros2 run rqt_image_view rqt_image_view
```

Then select `/semantic/mask`. Class IDs `0` through `4` are dark in a raw mono8
view, while `255` (ignore) appears white.

## Interface

| Direction | Topic | Type | Contract |
|---|---|---|---|
| Input | `/camera/color/image_raw` | `sensor_msgs/msg/Image` | Default RGB input; configurable with `input_topic` |
| Output | `/semantic/mask` | `sensor_msgs/msg/Image` | `mono8`, same width, height, stamp and frame as input |
| Output | `/semantic/info` | `std_msgs/msg/String` | JSON ratios for hill, crater, step and rover |

Mask class IDs are `0=other`, `1=hill`, `2=crater`, `3=step`, `4=rover`, and
`255=ignore`. Each ratio uses the total mask pixel count as its denominator, so
the four published ratios do not sum to one when `other` or `ignore` pixels are
present. The result is random test data and must not be treated as a perception
measurement.
