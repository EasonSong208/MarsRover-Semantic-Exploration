# Fixed camera transform provenance

Last updated: 2026-07-13

## Pose contract

The first supported fixed SLAM pose is:

```text
SLAM_POSE_V1 = vendor_init
```

`vendor_init` is the `init.d6a` pose automatically requested by the deployed
vendor boot chain. It is the default `camera_pose` because it matches the robot's
normal software startup target and does not require claiming that the visibly
similar `horizontal.d6a` pose was reached.

Two named configurations are retained:

| Pose | Action | Servo1..4 | Joint1..4 | Role |
|---|---|---|---|---|
| `vendor_init` | `init.d6a`, 1000 ms | `500,765,15,150` | `0,-63.6,116.4,84.0 deg` | `SLAM_POSE_V1`, default |
| `vendor_horizontal` | `horizontal.d6a`, 1500 ms | `500,750,0,375` | `0,-60,120,30 deg` | optional, non-default |

`vendor_horizontal` is valid only after that exact action has been played and an
operator has confirmed the physical pose. Merely booting the robot does not meet
that condition.

Neither pose is automatically verified from real servo feedback. The vendor
controller's normal `servo_states` path is a sent-target cache, not measured
position feedback, and no uniquely owned hardware feedback node was available in
the audit. `fixed_pose_confirmed=true` therefore means manual confirmation only.

## Evidence and pulse conversion

Read-only SQLite inspection returned:

```text
init.d6a:       Time=1000, Servo1=500, Servo2=765, Servo3=15, Servo4=150
horizontal.d6a: Time=1500, Servo1=500, Servo2=750, Servo3=0,  Servo4=375
```

The boot path is:

```text
start_app_node.service
  -> bringup.launch.py
  -> init_pose.launch.py action_name=init
  -> init_pose
  -> ActionGroupController
  -> init.d6a
```

For joints 1 through 4, the deployed controller uses a 500-centred reversed
240-degree/1000-pulse conversion:

```text
q = (500 - pulse) * ((240 / 360) * 2*pi / 1000)
```

This gives `vendor_init` radians:

```text
(0, -1.110029404268394, 2.031563249321400, 1.466076571675237)
```

## Full URDF forward kinematics

`robot_mission.fixed_camera_tf` multiplies the complete deployed URDF chain. It
does not modify the old horizontal transform by the 54-degree joint4 difference:

```text
base_link -> link1 -> servo_link1 -> link2 -> link3 -> link4
          -> camera_connect_link -> depth_cam_link
```

Evidence sources:

- `connect.urdf.xacro:4-20`: `base_link -> link1`, joint1 axis `0 0 -1`;
- `arm.urdf.xacro:127-251`: joint2, joint3 and joint4 origins and axes;
- `depth_camera.urdf.xacro:40-105`: camera connector and depth-camera mount;
- `joint_position_controller.py:11-55`: pulse conversion and reversed direction.

### `vendor_init` — `SLAM_POSE_V1`

`base_link -> depth_cam_link`:

| Field | Value |
|---|---:|
| x | 0.093787390048285 m |
| y | 0 m |
| z | 0.234390577130383 m |
| roll | 0 rad / 0 deg |
| pitch | 0.816814089933346 rad / 46.8 deg |
| yaw | 0 rad / 0 deg |
| quaternion `(x,y,z,w)` | `(0, 0.397147890634780, 0, 0.917754625683981)` |

The fixed URDF edge `base_footprint -> base_link` is
`(0,0,0.116091082157675)` m. Therefore
`base_footprint -> depth_cam_link` is:

```text
xyz = (0.093787390048285, 0, 0.350481659288058) m
rpy = (0, 0.816814089933346, 0) rad
quaternion = (0, 0.397147890634780, 0, 0.917754625683981)
```

### Retained `vendor_horizontal`

The previous full-chain result is preserved as a selectable non-default pose.

```text
base_link -> depth_cam_link:
  xyz = (0.090170699365528, 0, 0.291404054800367) m
  rpy = (0,0,0) rad
  quaternion = (0,0,0,1)

base_footprint -> depth_cam_link:
  xyz = (0.090170699365528, 0, 0.407495136958042) m
  rpy = (0,0,0) rad
  quaternion = (0,0,0,1)
```

## Configuration and TF ownership

The versioned pose files are:

```text
config/camera_poses/vendor_init.yaml
config/camera_poses/vendor_horizontal.yaml
```

The RGB-D launch loads exactly one file selected by `camera_pose` and publishes
only that file's four frozen revolute-joint edges. It does not publish a direct
`base_link -> depth_cam_link` edge, because the vendor robot-state publisher owns
the intervening fixed URDF edges and `depth_cam_link` must have only one parent.
The camera driver continues to own its internal optical-frame transforms.

The ready gate compares the resulting runtime `base_link -> depth_cam_link`
translation and quaternion with the selected YAML. It also rejects a joint-state
publisher, duplicate graph owners, or a missing base-to-optical chain.

## Accuracy boundary

These values are nominal extrinsics derived from action targets and the vendor
URDF. They are not a camera-to-base extrinsic calibration result. Unmeasured error
sources include servo zero, backlash, sag, linkage assembly, camera mounting and
the camera driver's internal calibration. If the arm moves, stop SLAM immediately;
do not continue using either fixed transform.
