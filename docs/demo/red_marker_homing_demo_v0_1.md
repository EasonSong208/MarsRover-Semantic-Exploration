# JetRover 红色标记回归实机演示 SOP v0.1

> 当前审计结论（2026-07-19）：**NO-GO，禁止运行完整实机闭环（阶段 5）**。
>
> 已有源码足以执行阶段 0～3 的只读、机械臂和感知 dry-run 检查，但还不满足
> v0.1 对底盘微动作和独立紧急停车的要求。具体缺口是：
>
> 1. 当前红块状态机默认先以 `0.05 m/s` 连续前进 5 秒，再以
>    `0.15 rad/s` 连续左、右转各约 3.49 秒；即使覆盖初始段参数，后续视觉
>    对齐仍可能连续输出角速度，代码没有 0.30 秒脉冲上限。
> 2. 仓库只有轮子悬空专用的 0.03 m/s、0.30 秒直行测试；没有符合
>    `|angular.z| <= 0.10 rad/s`、单次不超过 0.30 秒的左右微转入口。
> 3. 仓库没有经过验证的独立持续零速/紧急停车程序。任务自身的 Ctrl+C
>    清零是 best effort，不能覆盖进程崩溃、串口断开或 SIGKILL。
> 4. 当前完整 guard 只支持 `vendor_init`，而本 SOP 的观察姿态是
>    `horizontal`；不能把两者拼成一个已验证的 horizontal 闭环入口。
> 5. red-marker 节点的 `/odom` 仅可选记录，不参与 freshness/停车门禁；深度
>    比参考更近时当前策略是倒车恢复，而不是“过近立即终止并停车”。
>
> 本文中的“通过标准”是现场放行条件，不代表这些项目已经通过。任何标记为
> “待现场确认”或“缺失”的项目都不得按 PASS 处理。

## 1. 演示目标

本次演示最终只计划验证以下完整链路：

```text
机械臂姿态切换
  -> 深度相机工作
  -> 红色标记检测
  -> 视觉误差生成
  -> 小范围底盘修正
  -> 接近目标后停车
  -> 机械臂恢复 init
```

当前仓库可以安全审查到“视觉误差生成”，但底盘微动作和完整闭环仍是
NO-GO。只有本文阶段 4 和阶段 5 的缺口被代码和实机记录关闭后，才能把上面
整条链路宣称为演示通过。

本次不包含：

- 长距离导航；
- 机械臂抓取；
- SLAM；
- Nav2；
- 自动充电；
- 大角度高速转向；
- 无人看守运行。

红块程序恢复的是相机相对一个固定红块的水平像素位置和深度，不是全局
`x/y/yaw` 定位。当前实现还是“先建立参考、主动离开参考、再倒车恢复参考”
实验，不是从任意远处持续向红块前进的通用 approach 控制器。

## 2. 成功标准

一次“完整演示成功”必须同时满足：

1. 机械臂 `init` 与 `horizontal` 都有可见物理动作并到位。
2. `horizontal` 姿态下相机视野、线缆和机械结构正常。
3. 红色标记可连续稳定检测，RGB/Depth 配对和距离趋势合理。
4. dry-run 只记录决策，`/cmd_vel` 发布者数量仍为 0。
5. 经独立微脉冲验证，左右修正的物理方向正确。
6. 每次非零动作满足本文速度、时长和停车间隔限制。
7. 达到停止阈值后，任务至少以 20 Hz 发布零速度 2 秒，且机器人确实停车。
8. Ctrl+C、任务异常、相机超时和检测丢失均有已验证停车结果。
9. 最后机械臂恢复 `init`，没有碰撞、拉线或持续异响。
10. 相同配置完整连续成功 3 次，且每次均保留记录和输出目录。

当前第 5～8 项尚不完整，因此不能宣称完整演示成功。

## 3. 现场人员与环境要求

- [ ] 11.1V 电池已经充满；控制板 `/battery` 读数至少 10000 mV。
- [ ] 充电器已经拔掉，禁止边充电边运行。
- [ ] 机器人放在平整、干燥、防滑地面。
- [ ] 前方和两侧至少保留 1 米空旷区域。
- [ ] 机器人前方没有台阶。
- [ ] 机械臂运动范围内没有人手、工具或线缆。
- [ ] 一人操作终端，一人观察机器人。
- [ ] 观察人员站在物理电源旁，可随时断电；不要用身体阻挡运动中的机器人。
- [ ] 首次演示不放在桌面、台架或任何边缘附近。
- [ ] 红色标记固定，不由人手持。
- [ ] 手机、遥控器、APP、手柄和键盘遥控不会同时控制机器人。
- [ ] `start_app_node.service` 与 `button_scan.service` 均已确认不占用本轮控制链。
- [ ] 每个实机动作前重新获得现场负责人对“该一次动作”的明确许可。

## 4. 演示参数上限

### 4.1 当前源码参数与 v0.1 建议

| 项目 | 仓库默认值 | 代码硬限制 | v0.1 演示建议 | 当前结论 |
|---|---:|---:|---:|---|
| 初始前进速度 | `0.05 m/s` | `<=0.05` | `0.03 m/s` | 参数可覆盖 |
| 初始前进距离 | `0.25 m` | `<=0.25` | `0.009 m`，与 0.03 m/s 组合为 0.30 s | 参数可覆盖 |
| 初始左右转速 | `0.15 rad/s` | `<=0.15` | `0.10 rad/s` | 参数可覆盖 |
| 初始左右转角 | `30 deg` | `<=30` | `1.7 deg`，与 0.10 rad/s 组合约 0.297 s | 参数可覆盖 |
| 视觉对齐最大角速度 | `0.08 rad/s` | `<=0.08` | `<=0.08 rad/s` 且单脉冲 `<=0.30 s` | **缺少脉冲时限** |
| 倒车速度 | `-0.03 m/s` | 不得小于 `-0.03` | `-0.03 m/s` | 满足 |
| 倒车脉冲 | `0.35 s` | `<=0.35` | `0.30 s` | 参数可覆盖 |
| 段间停车 | `0.50 s` | `>=0.50` | `0.75 s` | 参数可覆盖 |
| 倒车后停车 | `0.50 s` | `>=0.50` | `0.75 s` | 参数可覆盖 |
| RGB/CameraInfo 超时 | `0.50 s` | `<=0.50` | `0.50 s` | 满足 |
| 目标丢失超时 | `0.50 s` | `<=0.50` | `0.50 s` | 满足 |
| 总超时 | `90 s` | `<=90` | `60 s` | 参数可覆盖 |
| 最终清零 | `2.0 s` | `>=2.0` | `2.0 s` | 任务正常退出路径满足 |
| 控制频率 | `20 Hz` | 固定 `20 Hz` | `20 Hz` | 满足 |

源码拒绝同时非零的 `linear.x` 和 `angular.z`。`dry_run`、`confirmed`、
`enable_base_motion` 是三个独立门禁，只有
`dry_run=false && confirmed=true && enable_base_motion=true` 才可能创建
`/cmd_vel` 发布者。

### 4.2 当前阻断项

**当前代码不满足 v0.1 实机演示安全约束，禁止进入实机闭环阶段。**

即使把初始前进、初始转向和倒车段改成上表建议值，
`VISUAL_ALIGN_YAW` 仍可在状态超时前连续输出角速度，没有“非零 0.30 秒后
强制停车并重新观察”的状态。不能通过 launch 参数补上这个缺失的状态边界。
此外，`log_odom` 只保存 `/odom` 坐标；源码没有 odom 超时停车条件。当前过近
处理会进入 `BACKUP_PULSE`，也不符合 v0.1 的“目标过近立即终止停车”。

## 5. 终端规划

建议使用 6 个终端。不要用多个终端重复启动同一个 launch。

| 终端 | 唯一职责 | 保持运行的进程 |
|---|---|---|
| A | 最小硬件、里程计和相机 bringup | `minimal_bringup.launch.py`，只启动一次 |
| B | 只读 preflight、graph、topic、TF 和电池检查 | 无长期硬件节点 |
| C | 最窄机械臂中间层 | `servo_controller.launch.py`，只启动一次 |
| D | 单次 `init` 或 `horizontal` 动作 | 每次只运行一个 `init_pose`，到位后 Ctrl+C |
| E | 红块检测、dry-run、输出目录 | 每次只运行一个 red-marker 任务 |
| F | `/cmd_vel` 监视、现场记录和物理急停协调 | 只读监视；物理电源是最终停止手段 |

每个终端先执行完全相同的环境加载：

```bash
source /opt/ros/humble/setup.zsh
source /home/ubuntu/ros2_ws/install/setup.zsh
source /home/ubuntu/hiwonder-jetson-robot/ros2_ws/install/setup.zsh
export ROS_DOMAIN_ID=0
```

三个 `setup.zsh` 和两个动作文件已在 Jetson 上静态确认存在。项目 overlay 必须
最后 source，否则可能运行到旧安装。

## 6. 阶段 0：开机与只读检查

本阶段不得移动机器人。任一步失败都停在本阶段。

### 6.1 物理电源与仓库

- [ ] 充电器已拔掉。
- [ ] 主电源打开，启动过程没有异常动作。
- [ ] 操作人与观察人均已就位。

终端 B：

```bash
test -r /home/ubuntu/hiwonder-jetson-robot/ros2_ws/install/setup.zsh
git -C /home/ubuntu/hiwonder-jetson-robot rev-parse HEAD
git -C /home/ubuntu/hiwonder-jetson-robot status --short
```

- 预期输出：setup 文件存在；显示本次使用的 commit 和工作区状态。
- 通过标准：命令退出码为 0，现场记录 commit；未提交文件必须和预期部署清单一致。
- 失败处理：停止，不要猜测安装路径或继续使用未知构建。

### 6.2 服务、旧进程和串口

```bash
systemctl is-active start_app_node.service
systemctl is-active button_scan.service
pgrep -af 'ros2|ros_robot_controller|odom_publisher|controller_manager|init_pose|depth_cam|joystick|nav2|velocity_smoother'
ros2 node list
readlink -f /dev/rrc
fuser -v /dev/rrc
```

- 预期输出：两个服务均为 `inactive`；没有旧 ROS 节点；`/dev/rrc` 解析为
  `/dev/ttyACM0`；启动 bringup 前串口没有占用者。
- 通过标准：没有 APP、joystick、Nav2、旧 controller、旧相机或旧任务。
- 失败处理：在原启动终端停止对应进程。不要直接用 `pkill` 猜测清理范围。
- 不得继续原因：重复串口所有者和重复速度发布者可能绕过任务门禁。

若服务处于 active，先确认机器人物理静止，再由现场负责人授权后执行：

```bash
sudo systemctl stop start_app_node.service
sudo systemctl stop button_scan.service
systemctl is-active start_app_node.service
systemctl is-active button_scan.service
```

复查结果必须为 `inactive`。停止服务本身不是软件急停；如果机器人有任何异常，
观察人立即断开物理电源。

### 6.3 启动唯一的最小 bringup 和相机

终端 A：

```bash
ros2 launch robot_mission minimal_bringup.launch.py \
  enable_lidar:=false \
  enable_camera:=true \
  enable_imu:=true \
  enable_odom:=true \
  enable_ekf:=true
```

- 预期输出：启动一个 `ros_robot_controller`、一个 `odom_publisher`、IMU/EKF、
  robot description 和一个 Dabai 相机；不启动 joystick、Nav2、`init_pose` 或
  `servo_controller`。
- 通过标准：没有执行器意外动作，终端无串口异常，相机容器保持运行。
- 失败处理：终端 A 按 Ctrl+C；若有任何物理运动，立即物理断电。
- 不得继续原因：后续所有检查都依赖唯一、稳定的 controller 和相机实例。

### 6.4 电池与串口唯一性

终端 B：

```bash
ros2 topic echo --once /ros_robot_controller/battery
fuser -v /dev/rrc
ros2 node list | sort
```

- 预期输出：`battery.data >= 10000`（单位为 mV）；`/dev/rrc` 只有
  `ros_robot_controller` 一个进程占用。
- 通过标准：电压至少 10V，没有串口断连或第二占用者。
- 失败处理：低于 10V、约 4V、串口无 owner 或多 owner 都立即 NO-GO。
- 不得继续原因：低压时舵机可能拒绝启扭矩；多 owner 会破坏反馈和命令确定性。

### 6.5 运行项目只读 preflight

```bash
ros2 run robot_mission preflight --ros-args \
  --params-file "$(ros2 pkg prefix robot_mission)/share/robot_mission/config/preflight.yaml"
```

- 预期输出：逐项打印 observation topics、唯一节点、TF、Nav2 lifecycle 和 action
  服务结果。
- 重要限制：该 preflight 面向 M1/Nav2，不是红块演示专用。由于本 SOP 故意不启
  LiDAR、map、AMCL 和 Nav2，最终 `SUMMARY` 预计会包含 FAIL。
- 通过标准：只把与本演示相关的 `ros_robot_controller`、`odom_publisher`、IMU、
  odom、RGB 和 CameraInfo 唯一性作为证据；不能把整体 FAIL 改写为 PASS。
- 失败处理：相关项目缺失或重复时停止。Nav2 项缺失是预期结果，但也证明当前
  仓库缺少红块专用 preflight。

### 6.6 速度控制图

```bash
ros2 topic info /cmd_vel -v
ros2 topic info /controller/cmd_vel -v
ros2 topic info /cmd_vel_nav -v
```

- 预期输出：`/cmd_vel` 有且只有一个订阅者 `odom_publisher`，发布者为 0；
  `/controller/cmd_vel` 不得有发布者；`/cmd_vel_nav` 不得有发布者。
- 通过标准：三条路径发布者都为 0，`/cmd_vel` controller subscriber 至少 1。
- 失败处理：任一发布者未知或重复都停止并查明 owner。
- 不得继续原因：red-marker 只检查并使用 `/cmd_vel`，APP/手柄可从
  `/controller/cmd_vel` 绕过它。

### 6.7 相机话题和频率

依次运行，每条观察约 12 秒：

```bash
timeout 12s ros2 topic hz /depth_cam/rgb/image_raw
timeout 12s ros2 topic hz /depth_cam/depth/image_raw
timeout 12s ros2 topic hz /depth_cam/rgb/camera_info
timeout 12s ros2 topic hz /depth_cam/depth/points
timeout 12s ros2 topic hz /odom
```

- 预期输出：历史审计约为 29～30 Hz；短时抖动可以存在，但不能持续断流。
- 通过标准：四个相机话题都有数据；RGB、Depth 和 CameraInfo 持续接近
  30 Hz；`/odom` 持续存在。
- 失败处理：停止在阶段 0，先排除重复相机、USB 断连和相机驱动错误。
- 限制：`/odom` 有频率不代表 red-marker 会在 odom 超时后停车；当前节点不把
  odom 用作完成条件或安全门禁。

### 6.8 TF 检查

```bash
timeout 8s ros2 run tf2_ros tf2_echo \
  depth_cam_link depth_cam_color_optical_frame
timeout 8s ros2 run tf2_ros tf2_echo \
  base_link depth_cam_color_optical_frame
```

- 预期输出：相机内部 `depth_cam_link -> depth_cam_color_optical_frame` 可用。
- `base_link -> camera` 只做诊断：minimal bringup 的 joint-state 路径不等于对
  `horizontal` 的实测反馈，不能据此证明物理外参正确。
- 通过标准：红块检测只依赖对齐的 RGB/Depth/CameraInfo，因此相机内部链必须
  连通；完整 base TF 标记为“待现场确认”，不得作为标定结果。
- 失败处理：相机内部 TF 缺失时停止；base TF 不可信时禁止把结果用于 SLAM/Nav2。

阶段 0 放行：

- [ ] 电压至少 10V。
- [ ] 串口只有一个 owner。
- [ ] `/cmd_vel`、`/controller/cmd_vel`、`/cmd_vel_nav` 均无发布者。
- [ ] controller、odom 和相机各只有一套。
- [ ] RGB/Depth/CameraInfo 连续。
- [ ] 没有意外物理动作。

## 7. 阶段 1：机械臂姿态验证

底盘不得运动。ROS launch 成功不代表舵机已经执行，必须记录物理结果。

### 7.1 启动唯一的机械臂中间层

终端 C：

```bash
ros2 launch servo_controller servo_controller.launch.py
```

- 预期输出：`/controller_manager` 出现，并提供
  `/controller_manager/init_finish`；它复用终端 A 的 `ros_robot_controller`，
  不打开第二个 `/dev/rrc`。
- 通过标准：终端无等待服务错误，串口仍只有 `ros_robot_controller` 一个 owner。
- 失败处理：Ctrl+C 停止终端 C；不要改用完整 `controller.launch.py`，否则会
  重复启动 controller、odom 和 robot description。

只读核对：

```bash
ros2 service list | grep '/controller_manager/init_finish'
ros2 topic info /ros_robot_controller/bus_servo/set_position -v
fuser -v /dev/rrc
```

### 7.2 单次动作命令

每次只在终端 D 运行一条。动作完成并人工确认后按 Ctrl+C，因为 vendor
`init_pose` 节点执行动作后仍会继续 spin，不能遗留后再启动下一实例。

`init`（动作文件 `init.d6a`，首行记录时长 1000 ms）：

```bash
ros2 launch controller init_pose.launch.py action_name:=init
```

`horizontal`（动作文件 `horizontal.d6a`，首行记录时长 1500 ms）：

```bash
ros2 launch controller init_pose.launch.py action_name:=horizontal
```

每条命令的共同检查：

- 预期动作：机械臂平滑到对应 vendor 动作组姿态，底盘保持静止。
- 通过标准：人工确认实际到位、无碰撞、无拉线、无持续异响；动作期间
  `/dev/rrc` 不断连。
- 失败处理：先在终端 D Ctrl+C；如果动作没有立即安全停止或有碰撞风险，物理
  断电。不得以 `/controller_manager/servo_states` 代替物理确认，因为它是发送
  目标缓存，不是实测位置。
- 收尾：到位后 Ctrl+C，再运行 `ros2 node list | grep '/init_pose'`；必须无输出。

按 `init -> horizontal -> init` 完整重复 3 轮：

| 轮次 | init 成功 | horizontal 成功 | 串口正常 | 无碰撞/拉线/异响 | 备注 |
|---:|---|---|---|---|---|
| 1 | [ ] | [ ] | [ ] | [ ] | |
| 2 | [ ] | [ ] | [ ] | [ ] | |
| 3 | [ ] | [ ] | [ ] | [ ] | |

任一轮失败：NO-GO，不进入阶段 2。

## 8. 阶段 2：相机和红色标记感知验证

底盘不得运动。

1. 用阶段 1 的真实命令将机械臂置于 `horizontal`，到位后停止 `init_pose`。
2. 不要再启动相机；终端 A 已通过 `enable_camera:=true` 启动唯一实例。
3. 终端 B 复查：

```bash
ros2 topic info /depth_cam/rgb/image_raw -v
ros2 topic info /depth_cam/depth/image_raw -v
ros2 topic info /depth_cam/rgb/camera_info -v
ros2 topic info /depth_cam/depth/points -v
```

- 预期输出：每个话题恰好一个相机 publisher。
- 失败处理：发现 0 或多个 publisher 时停止，禁止再开第二个相机尝试掩盖问题。

4. 有 Jetson 桌面或正确的 GUI 转发时：

```bash
ros2 run rqt_image_view rqt_image_view
```

依次选择 `/depth_cam/rgb/image_raw` 和 `/depth_cam/depth/image_raw`。无 GUI 时
只能把图像检查标记为“待现场确认”，不能因为 topic 有频率就判定画面正常。

5. 终端 E 启动纯感知 dry-run。该 launch 只启动 red-marker 节点，不启动相机、
controller 或机械臂；`dry_run=true` 且 `enable_base_motion=false` 时不会创建
`/cmd_vel` publisher：

```bash
DEMO_OUT=/tmp/red_marker_homing_demo_v0_1_perception_$(date +%Y%m%d_%H%M%S)
ros2 launch robot_mission red_marker_homing.launch.py \
  dry_run:=true \
  confirmed:=false \
  enable_base_motion:=false \
  output_directory:="$DEMO_OUT"
```

6. 另一个 GUI 终端打开项目调试图：

```bash
ros2 run rqt_image_view rqt_image_view
```

选择 `/red_marker_homing/debug_image`。同时可只读观察：

```bash
ros2 topic echo /red_marker_homing/metrics
```

7. 固定红块后，依次测试偏左、居中、偏右、较远、较近、移出画面。不要移动
机器人或手持红块。

| 场景 | 是否检测 | 水平误差方向正确 | 距离趋势正确 | 调试图正常 | 备注 |
|---|---|---|---|---|---|
| 偏左 | [ ] | [ ] | 不适用 | [ ] | |
| 居中 | [ ] | [ ] | [ ] | [ ] | |
| 偏右 | [ ] | [ ] | 不适用 | [ ] | |
| 较远 | [ ] | [ ] | [ ] | [ ] | |
| 较近 | [ ] | [ ] | [ ] | [ ] | |
| 暂时移出 | [ ] | [ ] | [ ] | [ ] | |

磁盘输出规则：

- 持续文件：`state_log.csv`、`sync_samples.csv`；
- 退出时：`summary.json`、`sync_summary.json`；
- 状态快照前缀：`reference`、`after_forward`、`after_left`、`after_right`、
  `final`；
- 快照后缀：`_rgb.png`、`_depth_preview.png`、`_debug.png`；16 位深度另有
  `_depth_raw_16uc1.png`；
- 节点不会把每一帧调试图连续写盘，实时调试图只在 ROS topic 上发布。

检查文件：

```bash
find "$DEMO_OUT" -maxdepth 1 -type f -printf '%f\n' | sort
tail -n 20 "$DEMO_OUT/state_log.csv"
```

## 9. 阶段 3：dry-run 决策验证

本阶段必须保证没有 `/cmd_vel` publisher。先 Ctrl+C 结束阶段 2 的 red-marker
进程，等待其写完 JSON，再以新目录启动。

终端 E：

```bash
DEMO_OUT=/tmp/red_marker_homing_demo_v0_1_dry_$(date +%Y%m%d_%H%M%S)
ros2 launch robot_mission red_marker_homing.launch.py \
  dry_run:=true \
  confirmed:=true \
  enable_base_motion:=false \
  forward_speed:=0.03 \
  forward_distance_nominal:=0.009 \
  turn_speed:=0.10 \
  turn_angle_deg:=1.7 \
  yaw_max_speed:=0.08 \
  backup_speed:=-0.03 \
  backup_pulse_duration:=0.30 \
  backup_settle_duration:=0.75 \
  segment_stop_duration:=0.75 \
  total_timeout:=60.0 \
  output_directory:="$DEMO_OUT"
```

- 预期日志：明确打印 `dry_run=True confirmed=True enable_base_motion=False`。
- 通过标准：节点推进决策状态，但实际 `cmd_linear_x/cmd_angular_z` 永远为 0；
  `/cmd_vel` 发布者为 0。
- 失败处理：任何 publisher 或非零实际命令都立即 Ctrl+C 并 NO-GO。

终端 B/F 在任务运行期间检查：

```bash
ros2 topic info /cmd_vel -v
ros2 topic echo /red_marker_homing/state
ros2 topic echo /red_marker_homing/metrics
```

按下表操作固定红块并对照 `decision_*` 与 `cmd_*`：

- 红块偏左：`pixel_error < 0`，当前公式的修正意图应为正 `angular.z`。
- 红块偏右：`pixel_error > 0`，修正意图应为负 `angular.z`。
- 红块居中：角速度意图为 0，进入深度判断。
- 红块丢失或传感器超时：实际命令必须为 0。
- 深度比参考更远：当前实现标记 overshoot 并中止；它不会向前补偿。
- 深度比参考更近：当前实现用短倒车脉冲恢复参考距离。

方向正负号不能只看代码变量名判断，必须在未来通过一次符合 v0.1 的极小实机
脉冲验证。当前仓库缺少该微转入口，所以 dry-run 方向正确不能放行阶段 5。

至少连续运行 30 秒，再 Ctrl+C。检查：

```bash
python3 -m json.tool "$DEMO_OUT/summary.json"
python3 -m json.tool "$DEMO_OUT/sync_summary.json"
tail -n 40 "$DEMO_OUT/state_log.csv"
```

- [ ] `dry_run` 为 true。
- [ ] `enable_base_motion` 为 false。
- [ ] `cmd_linear_x` 和 `cmd_angular_z` 全为 0。
- [ ] 没有 `/cmd_vel` publisher。
- [ ] `had_exception` 为 false。
- [ ] 输出目录无写入错误。

## 10. 阶段 4：底盘独立微动作验证

### 当前状态：NO-GO，禁止执行地面微动作

项目现有入口审计结果：

- `motion_smoke_test`：固定 `linear.x=0.03 m/s`、最长 0.30 秒、随后至少
  2 秒零速，但源码和 README 明确限定为**轮子悬空测试**，不能直接作为地面
  前进演示。
- `turn_step_test`：固定 `|angular.z|=0.20 rad/s`，52 条非零消息、积分窗口
  2.60 秒，约 30 度；明显超过 v0.1 的 0.10 rad/s 和 0.30 秒上限。
- 仓库 README 明确记录 `turn_smoke_test` 尚不存在。

以下命令只打印计划且不创建 publisher，可用于确认当前缺口，不是实机动作：

```bash
ros2 run robot_mission motion_smoke_test
ros2 run robot_mission turn_step_test --ros-args -p direction:=left
ros2 run robot_mission turn_step_test --ros-args -p direction:=right
```

预期分别输出 `NOT CONFIRMED` 或 `TEST_ABORTED reason=NOT_CONFIRMED`。

在新增并测试左右微转入口、地面前进微脉冲入口和独立停车入口之前，下列结果表
只能记录“未执行”：

| 动作 | 计划指令 | 实际方向 | 停车成功 | 是否允许继续 |
|---|---|---|---|---|
| 极小左转 | `<=+0.10 rad/s, <=0.30 s` | 未执行 | 未验证 | 否 |
| 极小右转 | `>=-0.10 rad/s, <=0.30 s` | 未执行 | 未验证 | 否 |
| 极短前进 | `+0.03 m/s, <=0.30 s` | 仅轮空版本 | 未验证地面 | 否 |

未来入口存在后，每次仍必须口头倒计时、单次授权、一次动作后至少停车
0.5～1.0 秒，并观察物理方向和停车结果。任一方向相反或 Ctrl+C 后不停，结论
立即为 NO-GO。

## 11. 阶段 5：红色标记闭环演示

### 当前状态：NO-GO，禁止执行

目标 v0.1 状态机应为：

```text
WAIT_FOR_CAMERA
  -> SEARCH_OR_WAIT_TARGET
  -> ALIGN_LEFT / ALIGN_RIGHT
  -> STOP_AND_REOBSERVE
  -> APPROACH
  -> STOP_AND_REOBSERVE
  -> TARGET_REACHED
  -> FINAL_STOP
```

当前源码实际状态机是：

```text
WAIT_FOR_SENSORS
  -> WAIT_FOR_MARKER
  -> COLLECT_REFERENCE
  -> WAIT_FOR_CONFIRMATION
  -> DRIVE_FORWARD_NOMINAL
  -> STOP_AFTER_FORWARD
  -> TURN_LEFT_NOMINAL
  -> STOP_AFTER_LEFT
  -> TURN_RIGHT_NOMINAL
  -> STOP_AFTER_RIGHT
  -> VISUAL_ALIGN_YAW
  -> CHECK_DEPTH
  -> BACKUP_PULSE / STOP_AFTER_BACKUP
  -> FINAL_VERIFY
  -> FINAL_STOP
```

二者不是同一控制策略。尤其当前实现含连续 yaw 对齐、先前进后倒车的参考恢复
实验，不能改名为“小步靠近红块”。

源码确实暴露 `dry_run=false`、`confirmed=true`、
`enable_base_motion=true` 实机门禁，但**不存在符合本文 v0.1 条件且已验证的
完整实机命令**。因此本文不提供可复制执行的 confirmed 闭环命令。

解除阶段 5 阻断至少需要：

- [ ] 实现左右微转 `<=0.10 rad/s`、每次 `<=0.30 s`。
- [ ] 所有视觉 yaw 和线速度都改为“脉冲 -> 零速 -> 重新观察”。
- [ ] 明确定义真正 approach 的距离方向和过近行为。
- [ ] 增加 odom freshness 门禁，或正式论证并记录本演示不依赖 odom。
- [ ] 提供独立、经过实机验证的持续零速入口。
- [ ] 用 horizontal 姿态建立一致的 pose/TF/ready 契约，或正式改为 vendor_init。
- [ ] 阶段 4 三个方向和 Ctrl+C 停车均有测试记录。
- [ ] 相机/目标丢失和串口异常的停车结果有实机证据。

## 12. 紧急停止流程

> **警告：仓库目前缺少经过验证的独立紧急停车入口，禁止进行阶段 5。**

若阶段 0～3 出现异常，或未来底盘测试获得放行后出现异常，按以下优先级处理：

1. 在当前任务终端按一次 Ctrl+C。red-marker 节点在 SIGINT/SIGTERM、异常和正常
   退出的 `finally` 中，会尝试以 20 Hz 发布零速度至少 2 秒。
2. 观察人立即确认机器人是否真的停止；不要只根据终端退出判断。
3. 终端 F 检查控制图：

   ```bash
   ros2 topic info /cmd_vel -v
   ros2 topic info /controller/cmd_vel -v
   ros2 topic info /cmd_vel_nav -v
   ```

4. 若软件清零无效、串口断开、进程崩溃或运动无法确认，立即使用物理电源停止。
5. 停止后不得直接重启。检查：

   ```bash
   ros2 node list
   pgrep -af 'ros2|ros_robot_controller|odom_publisher|controller_manager|init_pose|depth_cam|joystick|nav2|velocity_smoother'
   fuser -v /dev/rrc
   systemctl is-active start_app_node.service
   systemctl is-active button_scan.service
   ```

6. 记录电池电压、速度发布者、串口 owner 和相关终端日志。

`ros2 topic info` 只能证明当前 DDS endpoint，不能证明 STM32 已经清除最后命令。
当前可见 host 源码没有已证明的 STM32 watchdog，也没有独立 stop latch。缺少经过
验证的紧急停车入口时，阶段 5 必须保持 NO-GO。

## 13. 正常收尾流程

1. 人工确认底盘物理静止。
2. 若 red-marker 任务仍运行，在终端 E 按一次 Ctrl+C，等待至少 2 秒清零尾段和
   JSON 文件写完。
3. 保存并记录 `$DEMO_OUT`：

   ```bash
   find "$DEMO_OUT" -maxdepth 1 -type f -printf '%f %s bytes\n' | sort
   python3 -m json.tool "$DEMO_OUT/summary.json"
   python3 -m json.tool "$DEMO_OUT/sync_summary.json"
   ```

4. 终端 D 单独执行 `init`，人工确认实际到位后 Ctrl+C：

   ```bash
   ros2 launch controller init_pose.launch.py action_name:=init
   ```

5. 先 Ctrl+C 停止终端 C 的 `servo_controller`，再 Ctrl+C 停止终端 A 的
   minimal bringup。不要同时发送多个 Ctrl+C。
6. 终端 B 检查清理结果：

   ```bash
   ros2 node list
   pgrep -af 'ros2|ros_robot_controller|odom_publisher|controller_manager|init_pose|depth_cam'
   fuser -v /dev/rrc
   ```

7. 通过标准：没有遗留相关节点或进程，`fuser` 无输出，机器人保持静止。
8. 关闭机器人主电源。
9. 填写下一节记录。任何异常都保留为失败，不得通过重跑覆盖原记录。

## 14. 演示记录表

复制以下模板到新的 test record：

```markdown
# Red-marker homing demo record

- 日期/时间：
- 操作人：
- 观察人：
- Git commit：
- 工作区是否有未提交文件：
- ROS_DOMAIN_ID：
- 电池电压（mV）：
- 充电器已拔掉：[ ]
- 使用的 bringup：
- 使用的机械臂 controller：
- 使用的 red-marker launch：
- 完整参数：
- 输出目录：
- ROS 日志目录：

## 分阶段结果

- 阶段 0 只读检查：PASS / FAIL
- init 物理到位：PASS / FAIL
- horizontal 物理到位：PASS / FAIL
- 机械臂 3 轮结果：
- RGB 频率：
- Depth 频率：
- CameraInfo 频率：
- 红块检测：PASS / FAIL
- dry-run 无 /cmd_vel publisher：PASS / FAIL
- dry-run decision 方向：PASS / FAIL
- 左转微脉冲：PASS / FAIL / 未执行
- 右转微脉冲：PASS / FAIL / 未执行
- 前进微脉冲：PASS / FAIL / 未执行
- Ctrl+C 停车：PASS / FAIL / 未执行
- 独立紧急停车：PASS / FAIL / 缺失
- 完整闭环：PASS / FAIL / 禁止执行
- 最终物理停车：PASS / FAIL
- 最终恢复 init：PASS / FAIL

## 数据

- state_log.csv：
- sync_samples.csv：
- summary.json：
- sync_summary.json：
- 调试图片目录：

## 异常与改进项

- （填写）

## 最终结论

GO / NO-GO：
签字：
```

完整成功必须用同一版本和参数连续记录 3 次，不能只填写一次结果表。

## 15. Go / No-Go 检查表

- [ ] 电池充满、读数至少 10V，且充电器已拔掉。
- [ ] 最小 bringup 正常且没有意外动作。
- [ ] `/dev/rrc` 只有 `ros_robot_controller` 一个 owner。
- [ ] 三条速度话题没有发布者争抢。
- [ ] `init` 动作物理通过。
- [ ] `horizontal` 动作物理通过。
- [ ] init/horizontal 连续 3 轮通过。
- [ ] RGB、Depth、CameraInfo 和点云正常。
- [ ] 红块检测和误差方向正确。
- [ ] dry-run 没有创建 `/cmd_vel` publisher。
- [ ] 左转微脉冲方向验证正确。
- [ ] 右转微脉冲方向验证正确。
- [ ] 前进微脉冲方向验证正确。
- [ ] 所有非零段均不超过 0.30 秒。
- [ ] Ctrl+C 能使物理底盘停车。
- [ ] 独立紧急停车入口已经实机验证。
- [ ] 相机超时会停车。
- [ ] 目标丢失会停车。
- [ ] 目标过近按 v0.1 规则终止停车，而不是自动倒车。
- [ ] odom 超时策略已经实现或有正式的不依赖论证。
- [ ] horizontal pose/ready 契约与闭环一致。
- [ ] 现场人员对本次单次运行明确许可。

当前至少“微转”“地面微前进”“独立急停”“连续视觉脉冲限制”和
“horizontal 闭环契约”未通过，因此结论必须为：

**NO-GO：禁止运行完整实机闭环。**

## 16. 命令速查区

速查区不能替代前述分阶段检查和单次动作许可。

### 环境

```bash
source /opt/ros/humble/setup.zsh
source /home/ubuntu/ros2_ws/install/setup.zsh
source /home/ubuntu/hiwonder-jetson-robot/ros2_ws/install/setup.zsh
export ROS_DOMAIN_ID=0
```

### 最小 bringup（唯一实例）

```bash
ros2 launch robot_mission minimal_bringup.launch.py \
  enable_lidar:=false enable_camera:=true \
  enable_imu:=true enable_odom:=true enable_ekf:=true
```

### 只读 preflight

```bash
ros2 run robot_mission preflight --ros-args \
  --params-file "$(ros2 pkg prefix robot_mission)/share/robot_mission/config/preflight.yaml"
```

### 机械臂 controller、init 和 horizontal

```bash
ros2 launch servo_controller servo_controller.launch.py
```

另一个终端每次只运行一条，到位后 Ctrl+C：

```bash
ros2 launch controller init_pose.launch.py action_name:=init
ros2 launch controller init_pose.launch.py action_name:=horizontal
```

### 相机检查

```bash
timeout 12s ros2 topic hz /depth_cam/rgb/image_raw
timeout 12s ros2 topic hz /depth_cam/depth/image_raw
timeout 12s ros2 topic hz /depth_cam/rgb/camera_info
ros2 run rqt_image_view rqt_image_view
```

### 红块 dry-run

```bash
DEMO_OUT=/tmp/red_marker_homing_demo_v0_1_dry_$(date +%Y%m%d_%H%M%S)
ros2 launch robot_mission red_marker_homing.launch.py \
  dry_run:=true confirmed:=true enable_base_motion:=false \
  forward_speed:=0.03 forward_distance_nominal:=0.009 \
  turn_speed:=0.10 turn_angle_deg:=1.7 \
  yaw_max_speed:=0.08 \
  backup_speed:=-0.03 backup_pulse_duration:=0.30 \
  backup_settle_duration:=0.75 segment_stop_duration:=0.75 \
  total_timeout:=60.0 output_directory:="$DEMO_OUT"
```

### 红块 confirmed 实机模式

**不存在符合 v0.1 安全约束且已验证的可用命令。禁止执行阶段 5。**

虽然 launch 暴露 `dry_run`、`confirmed` 和 `enable_base_motion`，但拼出
`false/true/true` 只会绕过授权门禁，不能补上 0.30 秒视觉脉冲、独立急停或
horizontal ready 契约，因此本 SOP 不提供该命令。

### 紧急停车

**仓库缺少经过验证的独立停车命令。** 当前唯一任务内路径是在任务终端按一次
Ctrl+C，等待 red-marker 的 2 秒零速 cleanup，并由观察人确认物理停车；失败时
立即物理断电。此缺口关闭前禁止阶段 5。

### 日志和图片

```bash
find "$DEMO_OUT" -maxdepth 1 -type f -printf '%f %s bytes\n' | sort
tail -n 40 "$DEMO_OUT/state_log.csv"
python3 -m json.tool "$DEMO_OUT/summary.json"
python3 -m json.tool "$DEMO_OUT/sync_summary.json"
ros2 run rqt_image_view rqt_image_view
```

在 `rqt_image_view` 中选择 `/red_marker_homing/debug_image`。

### ROS2 进程检查与清理确认

先在各自原终端按 Ctrl+C，再只读确认：

```bash
ros2 node list
pgrep -af 'ros2|ros_robot_controller|odom_publisher|controller_manager|init_pose|depth_cam|joystick|nav2|velocity_smoother'
fuser -v /dev/rrc
```

不要把通配 `pkill` 或 `kill -9` 当作停车方法。

## 审计依据

本文命令和结论来自以下当前仓库文件及 Jetson 已部署源码的静态核对：

- `ros2_ws/src/robot_mission/setup.py`；
- `ros2_ws/src/robot_mission/launch/minimal_bringup.launch.py`；
- `ros2_ws/src/robot_mission/launch/red_marker_homing.launch.py`；
- `ros2_ws/src/robot_mission/launch/camera_guarded_red_marker_homing.launch.py`；
- `ros2_ws/src/robot_mission/config/red_marker_homing.yaml`；
- `ros2_ws/src/robot_mission/robot_mission/red_marker_homing_node.py`；
- `ros2_ws/src/robot_mission/robot_mission/homing_state_machine.py`；
- `ros2_ws/src/robot_mission/robot_mission/motion_smoke_test.py`；
- `ros2_ws/src/robot_mission/robot_mission/motion_smoke_policy.py`；
- `ros2_ws/src/robot_mission/robot_mission/turn_step_policy.py`；
- `docs/minimal_bringup_design.md`、`docs/rgbd_chain_audit.md`、
  `docs/red_marker_homing_test_plan.md` 和已有实机记录；
- Jetson 部署文件 `controller/launch/init_pose.launch.py`、
  `controller/controller/init_pose.py`、
  `servo_controller/launch/servo_controller.launch.py`、
  `servo_controller/controller_manager.py`、
  `servo_controller/action_group_controller.py`、
  `peripherals/launch/depth_camera.launch.py`；
- Jetson 动作文件 `/home/ubuntu/software/arm_pc/ActionGroups/init.d6a` 和
  `horizontal.d6a` 的存在性检查。

本轮只生成本 SOP；没有启动 ROS2、没有发布命令、没有修改业务代码。
