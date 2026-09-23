# Project status

Last updated: 2026-08-05

## Current objective

M1 aims to complete an approximately 20 m closed route, return near the origin,
then perform color-marker visual homing. The current engineering substage is M1-A:

```text
Drive(10.0 m) -> Turn(180.0 deg left) -> Drive(10.0 m)
```

M1-A is an odometry-terminated baseline, not final visual homing and not proof of
physical 20 m accuracy.

## Environments and authority

- GitHub `EasonSong208/hiwonder-jetson-robot` is the source of truth.
- Primary development checkout: WSL2,
  `/home/song_eason/codep/hiwonder-jetson-robot`.
- Jetson deployment checkout: `/home/ubuntu/hiwonder-jetson-robot`.
- Vendor ROS2 workspace on Jetson: `/home/ubuntu/ros2_ws`.
- Robot computer: NVIDIA Jetson Orin NX 16GB; ROS2 Humble; Hiwonder JetRover
  currently identified as `JetRover_Mecanum` with A1 LiDAR and Dabai RGB-D camera.
- Generated `build/`, `install/` and `log/` directories are machine-local.

Do not assume WSL2 and Jetson are currently identical. Before a target build,
compare the Git revision or explicitly selected file hashes. Targeted `rsync -aR`
has been used for temporary deployment, followed by SHA-256 verification.

## Completed and verified

- Vendor demos and navigation interfaces have run on the Jetson.
- Read-only audits established the main control, odometry, IMU, LiDAR, Nav2 and TF
  relationships summarized in `architecture.md`.
- Project-owned `robot_mission` package exists with read-only `preflight` checks.
- Minimal bringup was composed statically with camera optional and disabled by
  default. It excludes joystick, `init_pose`, servo controller, app demos and Nav2.
- Wheels-raised pure straight motion smoke test passed: 0.03 m/s, no more than
  0.3 s nonzero, automatic zero tail, no observed arm motion.
- The first explicitly ground-scoped repetition of that same envelope completed
  in software on 2026-07-19 but produced no operator-visible chassis or wheel
  motion. Command-integrated `/odom_raw` advanced by 0.008675 m, which is not
  physical feedback. The ground test therefore failed; static inspection makes
  low-speed ground deadband/static friction the leading hypothesis, while the
  downstream motor/serial path remains unproven. See
  `test_records/2026-07-19-ground-motion-pulse-no-physical-motion.md`.
- A separately approved `0.08 m/s`, `0.25 s` ground deadband probe also produced
  no operator-visible motion. Five expected nonzero motor frames at
  `+0.262524,+0.262524,-0.262524,-0.262524 rps` were captured on
  `/ros_robot_controller/set_motor`, followed by repeated zeros. This proves the
  project command and odometry translation path, but not the controller serial
  write or physical actuation. The remaining fault boundary is downstream of the
  observed motor-command topic. See
  `test_records/2026-07-19-ground-deadband-probe-no-physical-motion.md`.
- A subsequent non-motion buzzer check produced an operator-audible response,
  proving that the tested ROS controller callback, serial write and STM32 command
  handling can actuate a board output. It sounded continuously for approximately
  ten seconds until the operator requested an immediate stop; an explicit zero
  buzzer message was sent and the controller was shut down. Static audit showed
  that `0.1 s` was correctly encoded as `100 ms`, but `off_time=0` selects the
  firmware's continuous-buzzer mode. This was a diagnostic-command error, not a
  protocol time-unit error. Official Hiwonder hardware documentation identifies
  an independent motor-control switch, now the leading stationary check before
  any further wheel command.
- After the operator confirmed the independent motor-control switch precondition,
  one exact repetition of the `0.08 m/s`, `0.25 s` probe produced visible forward
  motion and passed. Software returned `0`, the zero tail completed, both command
  topics had no publishers afterward, `/odom_raw` twist was zero, and the minimal
  chain was fully removed. The earlier no-motion results are consistent with the
  motor hardware path being disabled at that time, but this remains an inference
  because the earlier switch state was not directly recorded. See
  `test_records/2026-07-19-ground-deadband-probe-physical-motion-pass.md`.
- The first user-approved guarded red-marker live invocation was attempted with
  the reviewed reduced motion parameters, but fail-closed before any arm or base
  motion. `camera_pose_guard` found `/odom_publisher` advertising the bus-servo
  torque topic and therefore kept `/camera_pose_ready=false`. The red-marker log
  contained zero nonzero actual command rows. Deployed vendor source confirms
  that `odom_publisher` unconditionally creates this steering-servo publisher,
  even though the Mecanum command branch normally does not use it. The conflict
  must be isolated without weakening the pose gate. See
  `test_records/2026-07-19-red-marker-guarded-live-run-blocked.md`.
- A restricted passive set-state publisher allowlist now resolves that graph
  mismatch offline without modifying vendor code. The node/config default remains
  empty and the accepted configuration value is code-limited to the exact
  `/odom_publisher`; only the guarded JetRover launch opts it in. Every DDS
  endpoint identity is namespace-qualified and retained, so duplicates, unknown
  or similar names, unresolved identities and graph-query failures remain
  fail-closed. Static deployed-source evidence confirms the Mecanum branch only
  publishes motor state; the sole bus-servo publish call is inside the Ackermann
  branch. Focused tests passed 41 cases and the full package passed 219. This
  revision has not been deployed or hardware-tested. See
  `test_records/2026-07-19-camera-pose-guard-passive-publisher-allowlist.md`.
- The user reports that separate pure straight and pure in-place turn tests have
  passed on hardware.
- DDS discovery delay was measured: a new node required about 2.7 s to discover
  both `ros_robot_controller` and `odom_publisher`. Motion tests now require two
  consecutive satisfactory graph snapshots within a fixed five-second window.
- A Jetson vendor-source syntax pollution at
  `src/driver/controller/controller/odom_publisher_node.py` was isolated and
  minimally corrected from `0.0codexcodex` to `0.0`; only the `controller` package
  was cleaned and rebuilt. The pre-fix backup is outside the vendor workspace.
- M1-A `out_and_back_test` is implemented locally with pure policy tests and static
  source-safety tests. Local results: 91 tests, 0 errors, 0 failures; local colcon
  build passed.
- A bounded camera-only runtime audit established the actual Dabai RGB-D topics,
  rates, QoS, calibration, alignment and timestamp behavior. The message chain is
  healthy, but base-to-camera TF is disconnected and GUI visual checks remain open.
- The vendor RTAB-Map launch was audited without executing it. Its default tree
  starts LiDAR, joystick, controller/servo and an arm `horizontal` action, deletes
  the default RTAB-Map database, and provides no visual-odometry node; it is blocked
  as the no-LiDAR RGB-D baseline.
- The audited boot chain selects `init.d6a`, so `SLAM_POSE_V1` is now
  `vendor_init`: Servo1-4 `500,765,15,150`, joints
  `0,-63.6,116.4,84.0` degrees. Full URDF FK gives
  `base_link -> depth_cam_link` translation
  `(0.093787390, 0, 0.234390577)` m and pitch `46.8` degrees.
  `vendor_horizontal` remains selectable but is no longer the default.
- A project-owned no-LiDAR RGB-D RTAB-Map launch is implemented with pre-start and
  ready gates, fixed-arm TF ownership, external `/odom`, and non-destructive
  database behavior. It has not passed stationary runtime validation.
- A standalone red-marker homing experiment now has a bounded dual-mode RGB/Depth
  synchronizer, fixed-offset comparison diagnostics and recoverable sensor-loss
  handling. The complete WSL offline suite, including synthetic OpenCV detector
  cases, passed after NumPy/OpenCV became available. A previous Jetson no-sensor
  dry-run completed without a chassis publisher; no live RGB-D homing or physical
  homing run has validated this revision.
- A `camera_pose_guard` offline implementation now gates red-marker chassis
  commands on the audited `vendor_init` arm target. It uses the existing vendor
  bus-servo command endpoint, publishes latched `/camera_pose_ready`, rejects
  competing arm owners and defaults to no servo publisher in dry-run/unconfirmed
  mode. On 2026-07-19, the WSL suite passed 182 tests, Jetson package build and
  tests passed, launch arguments parsed under Humble, and a bounded Jetson
  no-sensor dry-run showed no `/cmd_vel` or bus-servo command topic. All six
  dry-run processes then exited cleanly. This does not prove the physical arm
  pose was reached. A later stationary hardware audit on the same date proved
  that the guard published the correct `ServosPosition` command and that the
  position-only feedback service returned real pulses, but all four arm servos
  reported torque disabled. The arm did not move and remained at
  `506,398,253,152`, not `vendor_init`. See
  `test_records/2026-07-19-camera-pose-guard-root-cause.md`.
- The root-cause follow-up is now implemented in project-owned code. Arm motion
  requires the separate `arm_torque_confirmed` gate in addition to
  `dry_run=false` and `confirmed=true`. The guard reads and preloads every current
  pulse through the sole `/ros_robot_controller` ROS owner, enables torque, then
  requires torque state `1` and no pulse jump beyond the configured bound before
  it can publish the one allowed pose command. `FAULT` is terminal, automatic
  pose resend is disabled by the default `max_pose_command_attempts=1`, and node
  shutdown leaves torque unchanged. Red-marker base motion now has the independent
  fail-closed `enable_base_motion=false` default, and
  `camera_pose_guard_only.launch.py` excludes the red-marker and chassis paths.
  The vendor voltage/torque service method names were narrowly corrected and the
  vendor package rebuilt on Jetson. Local and Jetson offline verification passed;
  no torque, servo-position, or chassis command was sent in this implementation
  pass. See `test_records/2026-07-19-camera-pose-guard-arming-implementation.md`.
- The project-owned `semantic_perception` package retains its Phase 0
  `fake_semantic_node` and now adds an independent five-class PIDNet-S V3 node.
  The real node consumes the already-running Dabai RGB stream and publishes a
  same-size, header-preserving `mono8` mask on `/semantic/mask`, JSON ratios and
  timing on `/semantic/info`, plus optional color and overlay images. The V3
  checkpoint and all test artifacts live on the added SSD. Three static images,
  ROS interface checks and a 180-second live observation run passed on Jetson;
  FP16 stayed active with no CUDA OOM. Under concurrent Python point-cloud fusion,
  output averaged 11.73 Hz and intentionally dropped about 51.6% of frames seen
  by the monitor. A project-owned compatibility entry adapts the existing
  navigation backend's PointCloud2 NumPy return shape without modifying that
  backend. The isolated backend produced semantic costs throughout the run. This
  remains a temporary uncommitted deployment. See
  `pidnet_v3_deployment.md` and
  `test_records/2026-08-06-pidnet-v3-semantic-deployment.md`.

## Current interfaces

| Interface | Current understanding |
|---|---|
| `/cmd_vel` | Final direct mission/Nav2-smoothed chassis command consumed by vendor `odom_publisher` |
| `/controller/cmd_vel` | Direct alternative used by joystick/app paths; bypasses Nav2 smoother |
| `/cmd_vel_nav` | Nav2 controller output into `velocity_smoother` |
| `/odom_raw` | Produced by `odom_publisher` by integrating received commands; not independent encoder feedback |
| `/odom` | EKF output using command-derived odometry and filtered IMU; frame `odom`, child `base_footprint` |
| `/imu` | Filtered `sensor_msgs/msg/Imu`, from controller-board raw IMU through calibration/Madgwick |
| `/scan_raw` -> `/scan` | A1 driver output through scan filter |
| RGB-D | Dabai wrapper publishes RGB `/depth_cam/rgb/image_raw`, hardware-aligned depth `/depth_cam/depth/image_raw`, CameraInfo and XYZ cloud `/depth_cam/depth/points` at approximately 30 Hz |
| `/semantic/mask` | PIDNet or retained Phase 0 fake `sensor_msgs/msg/Image`; reliable/volatile `mono8` class IDs, input dimensions and input header |
| `/semantic/info` | `std_msgs/msg/String`; PIDNet class ratios, timing, precision and output FPS, or Phase 0 fake ratios when that node is selected |

## Open risks and UNKNOWN items

1. `/odom_raw` cannot observe slip, stall or external displacement. M1-A may appear
   successful in odometry while physical return error is large.
2. STM32 command-loss watchdog behavior remains UNKNOWN. Stopping a publisher or
   killing a process is not accepted as a stop mechanism.
3. Vendor `odom_publisher` shutdown does not itself guarantee a motor-zero command.
4. Duplicate bringup can create competing hardware, camera, TF and command paths.
5. The fixed camera transform is valid only when the physical arm exactly matches
   the selected `camera_pose` (`vendor_init` by default). The guard now implements
   an explicit read/preload/enable/verify arming sequence, but that complete
   sequence has not yet been exercised on hardware. The last hardware observation
   still had all four servos unloaded. Servo zero, backlash, sag and mounting
   tolerances remain. The URDF/FK result is nominal, not a calibrated camera
   extrinsic. RGB, depth and cloud visual correctness also remain manually
   unverified.
6. The vendor RTAB-Map entry remains unsafe: it composes
   motion-capable and LiDAR paths, deletes its default database, and has no visual
   odometry node or safe top-level disable arguments.
7. M1-A deployment parity with the Jetson is not recorded after the latest local
   implementation. It must be synchronized and rebuilt before even an unconfirmed
   invocation check.
8. Color target, illumination envelope, detection range, confidence threshold,
   loss behavior and final stand-off tolerances are not frozen.
9. RGB/Depth fixed offset remains UNKNOWN until `sync_samples.csv` from a Jetson
   dry-run is reviewed. The estimator is diagnostic-only and must not silently
   change the active offset.
10. The low-level bus-servo service's position-only path is hardware-validated.
    The voltage and torque method-name defects are patched in the Jetson vendor
    source and the package rebuild passed, but the repaired branches have not been
    called against live hardware after the rebuild. Their runtime result therefore
    remains UNKNOWN. `allow_time_based_ready=true` remains an explicit degraded
    mode, not measured pose confirmation.
11. The unbounded feedback-failure resend defect is fixed offline: the default is
    one pose-command attempt and `FAULT` is terminal. This policy has not yet been
    verified during a live armed pose run.
12. The `0.03 m/s` pulse and first `0.08 m/s` probe produced no visible motion,
    but a later exact `0.08 m/s` repetition after confirmation of the independent
    motor-control-switch precondition passed physically. The full chassis
    actuation path is therefore hardware-validated for this short envelope. The
    earlier failures are consistent with a disabled motor hardware path, but the
    historical switch state was not directly recorded. Do not use
    command-integrated `/odom_raw` as physical feedback for longer runs.
13. The guarded red-marker launch and vendor Mecanum `odom_publisher` graph
    mismatch now has an offline-only restricted allowlist implementation. The
    first live invocation was fail-closed with zero physical commands; the new
    exact endpoint policy has not yet been deployed or observed on the Jetson.
    Arm-only arming and pose arrival also remain unvalidated. Do not treat the
    offline pass as permission to retry full homing.
14. PIDNet-S V3 is runtime-validated against `/depth_cam/rgb/image_raw`, but the
    complete Python semantic-fusion chain averages 11.73 Hz and drops queued
    camera frames by design. The existing backend also needs the project-owned
    PointCloud2 compatibility entry on this Humble host. Isolated cost publication
    passed with a test map reference; production `/map_cropped` and full
    base/map-frame fusion were not started or validated in this stationary pass.

## Next gated work

1. Review and deploy only the M1-A package files; verify hashes on Jetson.
2. Build and test `robot_mission` on Jetson without running the mission.
3. Run `out_and_back_test` with its default `confirmed:=false`; it must print the
   plan and create no publisher, subscription or timer.
4. For a future arm-only hardware validation, first obtain fresh physical-motion
   approval, stop `button_scan.service` so the controller is also the sole OS-level
   `/dev/rrc` owner, verify the clear area and stop plan, and start only the vendor
   controller plus `camera_pose_guard_only.launch.py`. Review read/preload ordering,
   torque-state confirmation, pulse-jump bounds, the single pose attempt, and
   terminal `FAULT`; do not involve the chassis.
5. Repeat base-to-camera TF checks and complete manual RGB, depth, cloud and map
   views without chassis or arm motion.
6. Only after a fresh physical-motion approval, clear-area check and stop plan,
   run M1-A at 1 m. Review physical displacement, turn, zero cleanup, odometry and
   logs before progressing through 3 m, 5 m and 10 m.
7. Preserve the now-validated motor-control-switch precondition in every chassis
   test gate. The `0.08 m/s`, `0.25 s` ground probe has passed physically; any
   longer or different wheel command still needs a new, exact physical-motion
   approval and must retain the single-owner and zero-tail checks.
8. Deploy the restricted allowlist files with hash comparison, build and run the
   package tests on Jetson without hardware nodes, then inspect the live endpoint
   identities read-only. A separately approved arm-only torque/pose validation
   remains mandatory before any guarded homing retry.
9. Move the PIDNet V3 `semantic_perception` changes through the feature-branch/PR
   source-of-truth flow. Review the saved SSD overlays, then validate the real
   `/map_cropped` integration while stationary. Treat 11.73 Hz and intentional
   frame dropping as the current performance baseline before deciding whether the
   existing Python point-cloud backend needs a separately scoped optimization.

No 10 m run is currently authorized merely because the code and offline tests pass.
