# Phase 0 semantic perception interface validation

Date: 2026-08-05 (Asia/Shanghai)

## Scope and safety

The test covered only the project-owned `semantic_perception` observation node.
It did not deploy PIDNet-S, install a deep-learning dependency, publish a motion
or servo command, start or stop a camera/controller node, or modify a vendor
driver. The final live check subscribed to the Dabai RGB node that was already
running. No fake semantic process remained after the bounded checks.

## Software and environment

- WSL source checkout: branch `feat/rgbd-rtabmap-bringup`, with unrelated
  pre-existing changes preserved.
- Jetson deployment checkout: `/home/ubuntu/hiwonder-jetson-robot`, branch `main`,
  revision `6eac6a390d801dd689b9a98eebc26a8a8767546e` before the temporary sync.
- Jetson: Ubuntu 22.04, ROS2 Humble, Python 3.10.
- Transfer: the ten files under `ros2_ws/src/semantic_perception` were copied with
  targeted `rsync -aR`; WSL and Jetson SHA-256 values matched. The final node-file
  SHA-256 after the Humble shutdown fix was
  `5873bb4c2bd1d38895b721277a2cde486725b8945ae2adad123437b24faed9a8`.
- Build outputs: isolated under `/tmp/semantic-phase0-colcon.Jbtpr6` on the
  Jetson; the repository workspace `build/`, `install/` and `log/` were not used.

The Jetson had `cv_bridge`, `rclpy`, `sensor_msgs`, `std_msgs`, `launch_ros`,
NumPy and colcon available. `rosdep check` could not run because rosdep had never
been initialized on that host. No `rosdep init`, update or install was performed.

## Commands and results

The package was built and tested with explicitly isolated build paths:

```bash
colcon --log-base /tmp/semantic-phase0-colcon.Jbtpr6/log build \
  --base-paths /home/ubuntu/hiwonder-jetson-robot/ros2_ws/src \
  --packages-select semantic_perception \
  --build-base /tmp/semantic-phase0-colcon.Jbtpr6/build \
  --install-base /tmp/semantic-phase0-colcon.Jbtpr6/install \
  --symlink-install

colcon --log-base /tmp/semantic-phase0-colcon.Jbtpr6/test-log test \
  --build-base /tmp/semantic-phase0-colcon.Jbtpr6/build \
  --install-base /tmp/semantic-phase0-colcon.Jbtpr6/install \
  --packages-select semantic_perception

colcon test-result \
  --test-result-base \
  /tmp/semantic-phase0-colcon.Jbtpr6/build/semantic_perception --verbose
```

Results:

- Jetson colcon build: 1 package finished.
- Jetson tests: 5 tests, 0 errors, 0 failures, 0 skipped.
- `ament_flake8`: 6 files checked, no problems.
- `ament_pep257`: no problems.
- `ament_xmllint package.xml`: valid, no problems.
- A five-second launch check started `/fake_semantic_node` on the default
  `/camera/color/image_raw` input and exited cleanly on SIGINT.

The synthetic runtime check published one 3 x 2 `rgb8` image with stamp
`123.000000456` and `frame_id=test_camera`. The observed mask retained that
header, had `height=2`, `width=3`, `encoding=mono8`, `step=3`, and contained only
allowed IDs. One random result was `[255, 1, 1, 0, 0, 0]`; the paired info was:

```json
{"hill_ratio":0.333333,"crater_ratio":0.0,"step_ratio":0.0,"rover_ratio":0.0}
```

The bounded live launch selected the already-running robot endpoint:

```bash
ros2 launch semantic_perception semantic_perception.launch.py \
  input_topic:=/depth_cam/rgb/image_raw
```

Observed live mask metadata was `mono8`, 640 x 360, with
`frame_id=depth_cam_color_optical_frame`. The launch exited cleanly and process
checks found no remaining fake node. A GUI was not available, so
`rqt_image_view` remains a manual check. A separate second live info echo was not
captured because the CLI queried before DDS rediscovery; the synthetic paired
info check above passed.

## Result

Phase 0 package build, launch, fake-mask generation and declared topic contracts
passed on the Jetson. This is not a PIDNet-S deployment or a visual-quality test.
The temporary copied package remains uncommitted in the Jetson checkout and does
not replace the required feature-branch/PR deployment flow.
