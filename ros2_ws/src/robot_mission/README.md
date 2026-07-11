# robot_mission

Project-owned ROS2 utilities for M1. The first executable, `preflight`, performs
read-only graph, lifecycle, action-interface, and TF checks. It does not create a
motion publisher or navigation action client.

Build on the Jetson deployment checkout:

```bash
cd ~/hiwonder-jetson-robot/ros2_ws
colcon build --packages-select robot_mission
source install/setup.bash
```

Run after the approved, single-instance robot/navigation bringup is already active:

```bash
ros2 run robot_mission preflight --ros-args \
  --params-file "$(ros2 pkg prefix robot_mission)/share/robot_mission/config/preflight.yaml"
```

The process returns `1` when a hard readiness check fails. A successful report is
still not permission to move the robot; real motion requires explicit user approval.
