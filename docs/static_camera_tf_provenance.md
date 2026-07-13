# Fixed camera transform provenance

Last updated: 2026-07-13

## Scope and validity

The only supported mapping pose is the vendor action group `horizontal`, renamed
`vendor_horizontal` in project configuration. The arm must be physically placed
in this pose before launch and must not move while SLAM is running. Any arm motion
immediately invalidates this transform; stop SLAM and restore the pose before
continuing.

The numeric transform is source-derived and reproducible. Runtime conformance of
the physical arm to `vendor_horizontal` was not asserted in this round because no
servo action or feedback node was started.

## Pose evidence and joint angles

A read-only SQLite query of
`/home/ubuntu/software/arm_pc/ActionGroups/horizontal.d6a` returned one row:

```text
Time=1500 ms, Servo1=500, Servo2=750, Servo3=0, Servo4=375,
Servo5=500, Servo10=500
```

The action loader maps these columns directly to pulse-unit servo commands
(`servo_controller/action_group_controller.py:27-55`). The ROS joint-state
conversion uses a 500-centred, reversed 240-degree/1000-pulse mapping
(`joint_position_controller.py:11-55`):

```text
q = (500 - pulse) * ((240 / 360) * 2*pi / 1000)
```

| URDF joint | Pulse | Angle (rad) | Angle (deg) |
|---|---:|---:|---:|
| `joint1` | 500 | 0 | 0 |
| `joint2` | 750 | -1.047197551196598 | -60 |
| `joint3` | 0 | 2.094395102393195 | 120 |
| `joint4` | 375 | 0.523598775598299 | 30 |

The separate vendor kinematics map expresses joints 2 and 4 with a -90-degree DH
offset. Its pulse results are therefore -150 and -60 degrees respectively; adding
the documented model offset gives the same URDF angles above. Pulses were not
treated directly as radians.

## URDF chain and forward kinematics

Source joint origins:

- `connect.urdf.xacro:4-20`: `base_link -> link1`, joint1 axis `0 0 -1`;
- `arm.urdf.xacro:127-143`: `servo_link1 -> link2`, joint2 axis `0 1 0`;
- `arm.urdf.xacro:181-197`: `link2 -> link3`, joint3 axis `0 1 0`;
- `arm.urdf.xacro:235-251`: `link3 -> link4`, joint4 axis `0 1 0`;
- `depth_camera.urdf.xacro:40-105`: fixed camera connector and
  `camera_connect_link -> depth_cam_link`.

Applying each URDF origin followed by its joint-axis rotation gives:

```text
base_link
  -> link1              q1 =   0 deg
  -> servo_link1        fixed identity
  -> link2              q2 = -60 deg
  -> link3              q3 = 120 deg
  -> link4              q4 =  30 deg
  -> camera_connect_link fixed
  -> depth_cam_link      fixed
```

Final `base_link -> depth_cam_link`:

| Field | Value |
|---|---:|
| x | 0.090170699365528 m |
| y | 0 m |
| z | 0.291404054800367 m |
| roll | 0 rad |
| pitch | 0 rad |
| yaw | 0 rad |
| quaternion `(x,y,z,w)` | `(0,0,0,1)` |

The mecanum URDF fixes `base_footprint -> base_link` at
`(0, 0, 0.116091082157675)` m with zero rotation. Therefore the derived
`base_footprint -> depth_cam_link` translation is
`(0.090170699365528, 0, 0.407495136958042)` m, also with zero rotation.

ROS body axes are x forward, y left and z up. The identity final rotation means
the physical camera link axes are aligned with `base_link` in this pose. The
camera driver owns `depth_cam_link -> depth_cam_color_optical_frame`; the prior
runtime audit observed the optical convention and all RGB/aligned-depth messages
in `depth_cam_color_optical_frame`. This project does not duplicate that internal
camera transform.

## TF ownership

The vendor robot-description launch starts `joint_state_publisher`, which publishes
zero defaults and would conflict with the fixed pose. Fixed-arm mode instead starts
only `robot_state_publisher` with the same vendor Xacro. The project publishes the
four missing joint transforms as static edges. Vendor RSP retains ownership of all
fixed URDF edges, and the camera driver retains ownership of its internal frames.

A direct static `base_link -> depth_cam_link` edge is deliberately not published:
it would give `depth_cam_link` two parents because the vendor URDF already owns
`camera_connect_link -> depth_cam_link`.

## Error sources and runtime gate

- servo manufacturing zero and linkage assembly tolerances;
- mechanical backlash or sag;
- the physical arm not actually matching the database pulses;
- model-to-hardware mounting tolerances;
- camera-driver internal extrinsic calibration.

The launch defaults `fixed_pose_confirmed:=false`, so it refuses hardware bringup
until an operator confirms the physical pose. The ready gate also requires no
`/joint_states` publisher and a complete `base_footprint ->
depth_cam_color_optical_frame` TF before RTAB-Map can start.
