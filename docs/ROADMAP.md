# Roadmap

## M0 — Observable platform

Goal: understand vendor bringup, identify real and command-derived observations,
and establish repeatable diagnostics.

Status: substantially audited, with known gaps in physical encoder and joint
feedback semantics.

Exit evidence:

- Documented topic/node/TF graph.
- Stable single-instance bringup.
- Identified IMU and odometry provenance.
- Repeatable read-only observation commands.

## M1 — 20 m return-home mission

Goal: travel an approximately 20 m closed-loop route, return near the origin, and
use a color marker for final alignment and approach.

Stages and gates are defined in [M1_20M_RETURN_HOME.md](M1_20M_RETURN_HOME.md).
Short tests must pass before longer tests are authorized.

Exit evidence:

- Repeatable 20 m route completion under documented conditions.
- Quantified odometry drift and return-position error.
- Successful color-marker acquisition, alignment, and final approach.
- Complete test record containing software revision, configuration, metrics, and
  operator safety confirmation.

## M2 — Demonstration data

Goal: record synchronized observations and actions with stable schemas and
provenance suitable for imitation-learning experiments.

## M3 — Learned policies

Goal: evaluate imitation learning on bounded tasks with classical safety and
fallback controls retained.

## M4 — Policy improvement and compression

Goal: investigate reinforcement learning, distillation, and edge deployment only
after repeatable baselines and evaluation metrics exist.
