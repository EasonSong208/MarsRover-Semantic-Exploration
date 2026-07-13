# ADR 0002: Motion safety and authorization boundary

- Status: Accepted
- Date: 2026-07-13

## Context

The JetRover exposes multiple velocity paths, vendor launches may initialize
hardware, host-side shutdown does not guarantee motor zero, and the STM32 watchdog
is unknown. An executable or successful preflight cannot establish that the
physical area and operator are ready.

## Decision

1. Every physical motion test requires fresh, explicit user confirmation.
2. Motion executables default to a non-actuating mode such as `confirmed:=false`.
3. `/cmd_vel`, `/controller/cmd_vel`, `/cmd_vel_nav`, Nav2 goals and servo commands
   are never published merely for diagnosis.
4. Tests check for competing command publishers and required single-instance nodes
   before nonzero commands when that protocol is implemented.
5. Motion is bounded by hard speed/count/time limits, odometry or phase timeouts,
   and an explicit zero-command cleanup path.
6. Zero cleanup is best effort, not a guarantee against SIGKILL, power failure,
   serial failure or controller firmware behavior.
7. Hardware-facing launch/service state changes require authorization even if no
   intentional nonzero velocity is published.

## Consequences

- Offline tests and unconfirmed invocation checks can proceed without motion.
- The operator remains responsible for physical clearance, emergency power and the
  final go/no-go decision.
- Longer missions are gated by shorter recorded tests rather than code readiness.
- Safety UNKNOWN items remain visible instead of being hidden behind cleanup code.
