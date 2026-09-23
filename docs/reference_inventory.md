# Official reference inventory

Audit date: 2026-07-12. The 20 local PDFs under `references/official/` were
read-only. They are large vendor-distributed binary reference material and are
intentionally not versioned; this text inventory is the repository record.
Metadata was collected with `pdfinfo`; layout-preserving text was written only to
`/tmp/hiwonder-reference-text` with `pdftotext -layout`. The PDF Title field is
empty in every file, so the filename is the usable title.

Priority: **P0** before M1 hardware bringup/motion, **P1** directly useful during
M1, **P2** later engineering reference, **P3** outside the M1 critical path.

| Filename / relative path | Pages | Size | PDF metadata | Main content | M1 value | Priority |
|---|---:|---:|---|---|---|---|
| `references/official/JetRover使用手册.pdf` | 155 | 11,787,040 B | Author Administrator; WPS | Hardware, setup, RRC serial protocol, ROS/ROS2 directory layout, STM32 architecture | Hardware safety, Jetson/STM32 boundary, encoders and startup context | P0 |
| `references/official/运动控制.pdf` | 34 | 2,212,770 B | Author Administrator; WPS | IMU and odometry calibration, EKF, `/odom_raw`, chassis velocity control and source walkthrough | Defines the documented control/odometry chain and shutdown warning | P0 |
| `references/official/1 运动学分析.pdf` | 24 | 2,297,289 B | Author 简单三叶草; WPS | Mecanum/tank/Ackermann geometry and controller source mapping | Validate wheel geometry, signs and command conversion | P0 |
| `references/official/2 导航教程.pdf` | 37 | 2,531,000 B | Author Administrator; WPS | Nav2 architecture, TF/localization, single/multi-point and RTAB navigation | Official navigation launch and `map -> odom -> base_footprint` model | P0 |
| `references/official/2D视觉.pdf` | 51 | 3,321,454 B | Author Administrator; WPS | Hand tracking, color sorting/tracking, line clearing, classification and navigation transport | Color perception index, but examples deliberately move arm/chassis | P0 |
| `references/official/1.机械臂基础控制.pdf` | 38 | 2,638,873 B | Author Administrator; WPS | UART bus-servo hardware, PC tool, action groups, editing/import/export | Identifies servo serial ownership and startup-motion hazards | P0 |
| `references/official/激光雷达课程.pdf` | 14 | 726,349 B | Author Administrator; WPS | LiDAR principles and obstacle/follow/guard demos | Demo is unsafe for audit; source launch helps locate scan chain | P0 |
| `references/official/深度相机基础课程.pdf` | 34 | 2,682,270 B | Author Administrator; WPS | ROS2 basics, Orbbec Dabai, RGB/depth/IR/point cloud and LDP | Camera launch, topic tree and model-specific behavior | P0 |
| `references/official/1 建图教程.pdf` | 41 | 1,355,052 B | Author Administrator; WPS | GMapping, Cartographer and RTAB-VSLAM mapping workflows | Map production and SLAM-side TF ownership | P1 |
| `references/official/2.SDK管理工具烧写固件.pdf` | 30 | 2,436,080 B | Author Administrator; WPS | SDK manager and firmware flashing | Platform recovery reference; flashing is not part of M1 audit | P1 |
| `references/official/4.远程连接开发板.pdf` | 20 | 1,248,461 B | Author Administrator; WPS | Network and remote Jetson access | Supports WSL2/Jetson deployment and diagnostics | P1 |
| `references/official/ROS+OpenCV课程.pdf` | 39 | 2,816,800 B | Author 简单三叶草; WPS | ROS image transport and OpenCV examples | Foundation for a project-owned marker detector | P1 |
| `references/official/Gazebo仿真.pdf` | 31 | 2,092,288 B | Author 简单三叶草; WPS | Gazebo setup and robot simulation | Later non-hardware mission rehearsal | P2 |
| `references/official/Moveit2仿真.pdf` | 61 | 4,055,219 B | Author Administrator; WPS | MoveIt 2 arm simulation | Arm simulation, not chassis M1 critical path | P2 |
| `references/official/MediaPipe人机交互.pdf` | 72 | 4,713,013 B | Author 简单三叶草; WPS | MediaPipe perception and interaction demos | Later perception reference; demos can actuate hardware | P2 |
| `references/official/无人驾驶课程.pdf` | 34 | 2,077,774 B | Author Administrator; WPS | Autonomous driving examples | Later autonomy comparison; do not run during audit | P2 |
| `references/official/机器学习应用.pdf` | 89 | 4,531,122 B | Author Administrator; WPS | ML application examples | Deferred until stable M1 interfaces | P3 |
| `references/official/3 语音交互应用.pdf` | 74 | 4,721,913 B | Author Administrator; WPS | Voice interaction applications | Outside M1 | P3 |
| `references/official/4 具身智能应用之智能管家.pdf` | 27 | 1,903,911 B | Author 15317; WPS | Embodied assistant application | Outside M1; likely combines perception and actuation | P3 |
| `references/official/14.群控课程.pdf` | 9 | 3,538,010 B | Creator Typora/Electron | Multi-robot control | Outside single-robot M1 | P3 |

Creation/modification timestamps are available in the adjacent temporary
`*.pdfinfo` files but are not treated as publication versions: most contain only
authoring-tool timestamps and no explicit manual revision identifier.
