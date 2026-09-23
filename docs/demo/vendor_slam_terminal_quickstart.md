# 官方 SLAM Demo：纯终端调用指南

本文说明如何在 JetRover 的 Jetson 终端中直接启动官方二维 SLAM Demo、
确认建图数据正在产生、保存地图，以及停止程序。整个流程不需要 RViz 或
其他图形界面。

这里使用的入口是：

```bash
ros2 launch slam slam.launch.py
```

它启动官方 `slam` 包中的 `slam_toolbox` 建图流程。默认输入是 A1 LiDAR
的 `/scan`，底盘位姿链使用 `/odom` 和 TF。这个 launch 还会启动厂商的
机器人底层、传感器、手柄控制和初始化姿态组件，因此不要同时启动另一套
bringup、导航或 SLAM launch。

## 1. 登录 Jetson 并加载环境

打开第一个终端，登录 Jetson，然后执行：

```bash
cd /home/ubuntu/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

确认厂商 launch 依赖的环境变量存在：

```bash
printenv need_compile
printenv MASTER
printenv HOST
```

在当前已知的单机器人配置中，通常应看到：

```text
True
/
/
```

如果登录环境没有设置这些值，可仅在当前终端补充：

```bash
export need_compile=True
export MASTER=/
export HOST=/
```

## 2. 启动 SLAM

仍在第一个终端中执行：

```bash
ros2 launch slam slam.launch.py slam_method:=slam_toolbox sim:=false
```

保持这个终端运行。该命令本身不会打开 RViz，不需要设置 `DISPLAY`。

启动过程会先拉起机器人和传感器节点，约 5 秒后再启动
`sync_slam_toolbox_node`。看到 `slam_toolbox` 进入运行状态后，即可通过
厂商 Demo 原有的手柄控制机器人移动并采集地图。

> 启动时厂商 Demo 会执行机械臂初始化姿态，并启用手柄控制路径。开始前
> 保证机器人周围无障碍物，并确保没有其他程序在控制底盘。

## 3. 在第二个终端检查运行状态

打开第二个 Jetson 终端并加载同一环境：

```bash
cd /home/ubuntu/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

查看节点和关键话题：

```bash
ros2 node list
ros2 topic list
```

关键节点/接口应包括：

```text
/slam_toolbox
/scan
/odom
/map
/tf
/tf_static
```

分别检查雷达和地图是否持续更新；每条命令观察数秒后按 `Ctrl+C` 退出，
不会停止第一个终端中的 SLAM：

```bash
ros2 topic hz /scan
```

```bash
ros2 topic hz /map
```

读取一次地图元数据：

```bash
ros2 topic echo /map --once --field info
```

检查 `map -> base_footprint` TF：

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

如果 `/scan` 有数据但 `/map` 长时间没有消息，优先查看第一个终端中的
`slam_toolbox` 报错，以及 TF 查询是否成功。

确认没有启动可视化进程：

```bash
pgrep -af rviz2
```

没有输出即表示 RViz 未运行。

## 4. 保存地图

完成建图后，在第二个终端创建地图目录：

```bash
mkdir -p /home/ubuntu/hiwonder_maps/demo_slam
cd /home/ubuntu/hiwonder_maps/demo_slam
```

保存供 Nav2/map_server 使用的二维栅格地图：

```bash
ros2 run nav2_map_server map_saver_cli -f demo_map \
  --ros-args -p map_subscribe_transient_local:=true
```

成功后应得到：

```text
/home/ubuntu/hiwonder_maps/demo_slam/demo_map.yaml
/home/ubuntu/hiwonder_maps/demo_slam/demo_map.pgm
```

通过终端确认：

```bash
ls -lh /home/ubuntu/hiwonder_maps/demo_slam/demo_map.yaml \
  /home/ubuntu/hiwonder_maps/demo_slam/demo_map.pgm
```

如果之后还需要由 `slam_toolbox` 继续编辑同一张图，可以另外保存其序列化
位姿图。先确认服务存在：

```bash
ros2 service list | grep /slam_toolbox/serialize_map
```

存在时调用：

```bash
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/ubuntu/hiwonder_maps/demo_slam/demo_map'}"
```

`.yaml` 和 `.pgm` 用于常规地图加载；序列化位姿图用于
`slam_toolbox` 的继续建图，两者用途不同。

## 5. 停止 Demo

先确保机器人已经停止，再回到运行 launch 的第一个终端，按一次：

```text
Ctrl+C
```

等待 launch 输出所有节点退出信息并返回 shell。不要使用 `kill -9` 作为
正常停止方式。

停止后可以在第二个终端确认关键节点已经消失：

```bash
ros2 node list
```

## 6. 一页式命令摘要

终端 1：

```bash
cd /home/ubuntu/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch slam slam.launch.py slam_method:=slam_toolbox sim:=false
```

终端 2：

```bash
cd /home/ubuntu/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic hz /scan
ros2 topic hz /map
mkdir -p /home/ubuntu/hiwonder_maps/demo_slam
cd /home/ubuntu/hiwonder_maps/demo_slam
ros2 run nav2_map_server map_saver_cli -f demo_map \
  --ros-args -p map_subscribe_transient_local:=true
```

保存完成后，回到终端 1 按 `Ctrl+C` 停止。

## 7. 与 RTAB-Map Demo 的区别

本文使用的是二维 LiDAR `slam_toolbox` Demo。厂商还提供 RGB-D/激光融合的
RTAB-Map 入口：

```bash
ros2 launch slam rtabmap_slam.launch.py
```

它同样不会自动启动 RViz，但启动组成与地图格式不同，并且厂商实现会用
`-d` 清空 RTAB-Map 默认数据库后启动。不要把两种 SLAM launch 同时运行。
