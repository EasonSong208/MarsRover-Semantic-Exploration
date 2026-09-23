# Red marker RGB-D homing experiment

## Scope and accuracy boundary

This experiment restores the camera's observed distance and horizontal bearing
relative to one fixed matte-red block. One marker does not uniquely recover global
`x`, `y`, and `yaw`, so success must not be described as global localization or a
complete return-to-pose result. `/odom` is optional logging only and is never a
completion condition.

The node subscribes to the already-running aligned streams
`/depth_cam/rgb/image_raw`, `/depth_cam/depth/image_raw`, and
`/depth_cam/rgb/camera_info`. It does not launch or configure any dependency.

## Detector and reference

The detector treats the incoming image as `rgb8`, converts it with
`COLOR_RGB2HSV`, combines parameterized low- and high-hue red bands, applies open
then close morphology, and selects the largest in-range contour. Depth is sampled
inside an eroded contour mask. Zero and non-finite samples are removed and the
native-unit median is retained; no millimetre assumption is made.

After at least two continuous seconds and 20 distinct valid observations,
reference values are
the medians of `u`, `v`, depth, bounding-box width, and area. The robust completion
tolerances are:

```text
yaw_tolerance_px = max(configured yaw_pixel_tolerance, 3 * u_MAD)
depth_tolerance = max(3 * depth_MAD, depth_ref * depth_relative_tolerance)
```

MAD makes the threshold respond to observed jitter while the configured/relative
floors avoid unrealistically tight thresholds when the reference happens to be
nearly constant. Collection restarts when `u_MAD` exceeds 4 px or `depth_MAD`
exceeds 2% of median depth. RGB and depth are paired by a project-owned bounded
nearest-neighbour matcher. It never combines two independently overwritten
`latest` messages and never changes a ROS message header.

Two runtime-selectable modes are available:

- `direct_slop`: compare original stamps with a default 70 ms tolerance;
- `fixed_offset`: compare RGB against an internal logical Depth stamp using
  `corrected_depth_stamp = original_depth_stamp + depth_stamp_offset_ms`, with a
  default 25 ms tolerance.

`compare_sync_modes:=true` feeds both independent matchers, but only `sync_mode`
drives detection and the state machine. The other matcher is diagnostic-only.
Buffers are limited by `sync_queue_size` and `sync_buffer_max_age_ms`. Matched and
evicted samples go to `sync_samples.csv`; aggregate match/delta statistics and the
diagnostic-only median offset estimate go to `sync_summary.json`.

The state log records raw/corrected pair delta, detection age, reason, sequence,
sensor health and both proposed and actually published commands.

## Control sequence

The first three segments are explicitly open loop: nominal 0.25 m forward at
0.05 m/s, nominal 30 degree left turn, and nominal 30 degree right turn at
0.15 rad/s. These values are command integrals, not measured physical travel.
Every segment is pure translation or pure yaw and is followed by zero velocity.

Yaw correction uses CameraInfo `fx`:

```text
pixel_error = u_current - u_ref
angle_error = atan(pixel_error / fx)
angular_z = clamp(-yaw_kp * angle_error, -yaw_max_speed, yaw_max_speed)
```

A target right of its reference therefore commands negative `angular.z` (right
turn); a target left commands positive `angular.z` (left turn). A configured
minimum magnitude is applied outside the tolerance band.

Distance recovery alternates yaw alignment with 0.35 second reverse-only pulses
at -0.03 m/s and 0.50 second zero settles. If depth is still below
`depth_ref - tolerance`, another pulse is permitted. In-range depth advances to
final verification. Depth above `depth_ref + tolerance` is an overshoot abort;
the node never drives forward to compensate.

## Safety gates

- Nonzero chassis commands require all three independent gates:
  `dry_run=false`, `confirmed=true`, and `enable_base_motion=true`.
- `enable_base_motion` defaults to false. While false, the node does not create
  the chassis command publisher even if the other two gates are open.
- With `require_camera_pose_ready=true`, every final command also requires the
  transient-local `/camera_pose_ready` value to be true. A false transition
  immediately zeros both command axes without terminating the node; recovery is
  automatic. Dry-run records the pose gate but never creates `/cmd_vel`.
- A real run waits for two clean graph snapshots, no competing publisher on
  `/cmd_vel`, `/controller/cmd_vel`, or `/cmd_vel_nav`, and exactly one each of
  `ros_robot_controller` and `odom_publisher`.
- Publisher conflicts discovered after startup abort the run.
- RGB, depth, CameraInfo, marker, area, and valid-depth gates stop the state
  machine. Total, per-state, segment, pulse-count, and parameter-envelope limits
  are enforced.
- Linear and angular commands cannot be nonzero together.
- Normal completion, abort, exception, SIGINT, and SIGTERM all reach a `finally`
  cleanup that publishes zero at 20 Hz for at least two seconds when the command
  publisher exists.

Synchronization loss is a recoverable soft fault. Recent unmatched RGB still runs
2-D red detection and produces a debug box. It may produce a yaw decision, but
Depth older than `depth_freshness_limit_ms` always suppresses `linear.x`. RGB or
Depth beyond `sensor_stop_timeout_ms` suppresses all motion while the node remains
alive and continues matching. States `RGB_ONLY`, `DEPTH_STALE`, `NO_RGB` and
`SENSOR_SILENCE` return automatically to `SYNC_OK` when valid pairs recover.
Warnings are throttled by `sensor_log_throttle_ms`; no per-frame INFO log is used.

Application cleanup remains best effort. It cannot protect against SIGKILL,
power/serial failure, or an unproven STM32 watchdog, so an operator at physical
power/stop control remains mandatory for any later real test.

## Offline-only invocation

With externally supplied recorded or synthetic RGB-D topics, the safe launch is:

```bash
ros2 launch robot_mission red_marker_homing.launch.py \
  dry_run:=true confirmed:=true \
  sync_mode:=direct_slop direct_slop_ms:=70.0 \
  output_directory:=/tmp/red_marker_homing_test
```

`confirmed:=true` lets the dry-run state machine advance, but `dry_run:=true`
forces every command to zero and does not create a command publisher.

To compare a fixed correction while keeping all motion disabled:

```bash
ros2 launch robot_mission red_marker_homing.launch.py \
  dry_run:=true confirmed:=true \
  sync_mode:=fixed_offset fixed_offset_slop_ms:=25.0 \
  depth_stamp_offset_ms:=-55.0 compare_sync_modes:=true \
  output_directory:=/tmp/red_marker_homing_fixed_offset
```

A negative value is appropriate when Depth stamps are later than RGB stamps. The
automatic estimator reports a suggestion only; it never modifies the active
offset during a run.

## Future reviewed real-run proposal — do not execute without new approval

```bash
ros2 launch robot_mission red_marker_homing.launch.py \
  dry_run:=false confirmed:=true enable_base_motion:=true \
  output_directory:=/tmp/red_marker_homing_test_real
```

Before that future command, manually confirm the physical arm is exactly
`vendor_init`; fix the matte-red block with no other large red background;
keep camera and arm immobile; clear the test area; place an operator at emergency
power; verify `start_app_node.service` is inactive; verify exactly one intended
velocity path and healthy RGB-D topics; complete and review a dry run; and obtain
fresh explicit physical-motion approval.

Outputs include debug/state/metrics topics plus PNG snapshots, an unmodified
16UC1 PNG when the input is 16-bit, `state_log.csv`, and `summary.json` under the
selected `/tmp` directory.
