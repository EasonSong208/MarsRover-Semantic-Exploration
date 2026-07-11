# M1: 20 m Return Home with Visual Homing

## Objective

The robot should follow an approximately 20 m closed-loop route, return to the
start area, find a predefined color marker, align to it, and approach a safe final
pose. M1 is an engineering validation milestone, not a learning-policy milestone.

## Safety and execution policy

- Every physical motion stage requires fresh user confirmation.
- No script may publish `/cmd_vel` or send a Nav2 goal by default.
- Before each test, define the clear area, maximum speed, expected path, observer,
  emergency stop method, timeout, and success/failure thresholds.
- Begin with the smallest test. Do not skip a failed gate or extrapolate a result
  from simulation to hardware.
- Store results in `docs/test_records/`; do not commit bags or bulk sensor data.

## System boundaries

Project-owned mission logic belongs in `ros2_ws/src/robot_mission/`. Vendor drivers
remain upstream dependencies. The mission package should eventually coordinate
Nav2 goals and visual homing through explicit states, but this repository skeleton
does not yet implement actuation.

Expected high-level state flow:

```text
preflight -> navigate route -> return region -> search marker
          -> align marker -> approach -> stop -> report
```

## Stage 1 — Interface audit

Purpose: establish one authoritative publisher for each critical command,
observation, and TF edge.

Verify at minimum:

- Velocity chain: `/cmd_vel_nav -> velocity_smoother -> /cmd_vel -> base driver`.
- Direct-control publishers on `/controller/cmd_vel` are identified and inactive.
- Lidar chain: `/scan_raw -> /scan`.
- Localization chain: `/map + /scan + odom TF -> AMCL -> map -> odom`.
- Odometry provenance and limitations are recorded.
- Camera topics and camera-to-base TF are stable.
- Nav2 lifecycle nodes and actions are available.

Gate: no duplicated hardware/TF publishers, all required frames resolve, and a
read-only diagnostic run is repeatable.

## Stage 2 — 0.5 m straight-line test

Purpose: validate command routing, stopping, direction, short-distance odometry,
and physical displacement measurement.

Record commanded distance, independent measured distance, odometry distance,
lateral error, yaw error, speed, surface, and stop behavior.

Gate: thresholds must be agreed before execution and met repeatedly.

## Stage 3 — 90-degree turn test

Purpose: validate angular direction, yaw convention, IMU/EKF behavior, overshoot,
and settling.

Record commanded angle, independently measured angle, odometry/TF angle,
overshoot, settling time, and repeatability in both directions.

## Stage 4 — 1 m × 1 m square

Purpose: combine short straight segments and turns, then quantify closure error.

Record final position/yaw error, corner behavior, replanning events, localization
confidence, and safety interventions.

## Stage 5 — 5 m × 5 m square

Purpose: expose accumulated odometry/localization error over an approximately
20 m perimeter before adding visual homing.

This is the first test with M1-scale travel distance, but it is not the final
acceptance test. It requires successful prior stages and a mapped, controlled area.

## Stage 6 — Color marker search, alignment, and approach

Develop and validate three separable behaviors:

1. Search: detect the configured color marker and report confidence/bearing.
2. Align: reduce horizontal/angular error without uncontrolled forward motion.
3. Approach: move toward the marker with bounded speed and a positive stop rule.

Define marker color space, dimensions, placement, lighting range, detection range,
loss-of-target behavior, alignment tolerance, approach distance, and timeout.
Test perception without motion before testing any actuation.

## Stage 7 — Final 20 m acceptance

Purpose: combine navigation return and visual homing in one controlled run.

Acceptance criteria must be frozen before execution and should include:

- Route length and allowed deviation.
- Navigation success and intervention count.
- Return-region position/yaw tolerance before visual homing.
- Marker acquisition time and false-detection behavior.
- Final lateral, angular, and stand-off error.
- Maximum speed, total duration, timeout, and safe-stop behavior.
- Required number of successful repeated runs.

## Test record template

Each stage record should contain:

- Date, operator, location, floor/surface, and safety setup.
- Git commit and branch deployed to the Jetson.
- ROS2 launch command and relevant configuration hashes.
- Topic/TF/lifecycle preflight results.
- Expected result and predeclared pass/fail thresholds.
- Actual measurements, anomalies, and interventions.
- Links to external artifacts without committing bulk runtime data.
- Decision: pass, fail, or repeat, with the next permitted stage.
