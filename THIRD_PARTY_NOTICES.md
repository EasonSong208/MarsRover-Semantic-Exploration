# Third-Party Notices

This repository contains or integrates third-party software. Those files remain
subject to the licensing and copyright terms of their original authors and are
**not** relicensed under this repository's MIT License. The MIT License applies
only to original code authored for this repository (see [LICENSE](LICENSE)).

## Original project code (MIT)

Original code authored for this repository includes, for example:

- `ros2_ws/src/robot_mission/` — mission readiness checks, camera pose guard,
  red-marker homing, and RGB-D bringup work.
- `ros2_ws/src/semantic_perception/` — PIDNet-S semantic inference and
  compatibility nodes.
- `training/` — PIDNet-S training/validation stage (wraps the upstream PIDNet
  project; see below).
- `vendor_patches/` — project-authored patch files against vendor drivers (the
  vendor sources they modify are not distributed here).

## Hiwonder robot software (`ros2_ws/src/`)

The following source packages were imported from Hiwonder robot software and
remain subject to the licensing and copyright terms of their original authors.
They are not relicensed under this repository's MIT License.

| Package | License status found in this repository |
| --- | --- |
| `hiwonder_moveit_config` | BSD (declared in `package.xml`; no license text bundled) |
| `holonomic_sim` | Apache-2.0 (full license text included at `ros2_ws/src/holonomic_sim/LICENSE`) |
| `jetrover_description` | ⚠️ No license declared |
| `robot_gazebo` | ⚠️ No license declared |
| `ros_robot_controller_msgs` | ⚠️ No license declared |
| `servo_controller` | ⚠️ No license declared |
| `servo_controller_msgs` | ⚠️ No license declared |

Notes:

- ⚠️ Where no license is declared, no permission to copy, modify, or
  redistribute may be assumed merely because the code is publicly visible.
  These packages are included as part of this robot's development history;
  please contact the original authors (Hiwonder) regarding reuse.
- Some files inside the imported packages carry their own third-party headers
  (for example Apache-2.0 headers on portions of `robot_gazebo` launch files,
  and standard ROS test templates). Those terms take precedence for the
  corresponding files.
- Upstream reference: <https://github.com/Hiwonder/JetRover>

## Other third-party components

- **PIDNet** — `training/` wraps <https://github.com/XuJiacong/PIDNet>
  (MIT License, pinned commit `4c158cf24ce432f0a8cb43364fae38d93cee0dc3`),
  cloned separately at setup time; its source is not vendored into this
  repository.

If you believe any material is mis-attributed, please open an issue.
