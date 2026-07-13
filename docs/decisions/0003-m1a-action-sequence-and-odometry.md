# ADR 0003: M1-A action sequence and odometry baseline

- Status: Accepted
- Date: 2026-07-13

## Context

M1 needs a measurable baseline before adding rectangular routes, Nav2 or visual
homing. Existing straight and turn tests validate directions and bounded commands,
but chaining executables through a shell would fragment state, cleanup and metrics.

## Decision

M1-A is represented as data:

```text
[Drive(distance_m), Turn(180 deg left), Drive(distance_m)]
```

A single project-owned ROS2 node uses one `/cmd_vel` publisher, one `/odom`
subscriber and one fixed-rate non-blocking timer. The policy is ROS-independent.
Drive completion uses along-track projection from each segment start, with a
cross-track abort threshold. Turn completion accumulates normalized yaw deltas.
Pure translation and pure rotation are mutually exclusive; no heading PID is added
in this baseline.

`/odom` is accepted only as the current integration interface, with the explicit
limitation that its `/odom_raw` source integrates commands rather than independent
encoder feedback. Physical distance and closure error must therefore be measured
separately during staged tests.

## Consequences

- The same executor can later represent additional drive/turn sequences without
  shelling out or duplicating state branches.
- Offline policy tests cover geometry, wraparound, timeouts and terminal zeros.
- M1-A does not validate slip detection, obstacle handling, navigation, visual
  homing or precise physical return.
- Hardware progression is fixed at 1 m, then 3 m, 5 m and finally 10 m after review
  of each prior result.
