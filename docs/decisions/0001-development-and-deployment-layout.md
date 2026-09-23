# ADR 0001: Development and Deployment Layout

- Status: Accepted
- Date: 2026-07-11

## Context

The project is developed across a Windows + WSL2 workstation and a Hiwonder robot
with a Jetson Orin NX 16GB. ROS2 hardware compilation and real-robot validation
must occur on the Jetson, while most editing and review should occur in WSL2.
Uncoordinated edits on both machines would create divergent sources and make test
results difficult to reproduce.

## Decision

1. GitHub repository `EasonSong208/hiwonder-jetson-robot` is the only authoritative
   source.
2. The WSL2 checkout is the primary development workspace.
3. The Jetson has a separate deployment checkout used for ROS2 builds and hardware
   tests.
4. Work is developed on feature branches and merged through pull requests.
5. Deployment changes move through GitHub; the Jetson checkout is not an independent
   source of truth.
6. ROS2 `build/`, `install/`, and `log/` directories and other machine-generated
   artifacts are never committed.
7. Machine-local configuration and credentials remain ignored. Shareable defaults
   use sanitized example files or documented environment variables.
8. Vendor packages are imported dependencies. Project mission code is isolated in
   project-owned packages such as `robot_mission`.

## Consequences

- Test records can identify an exact Git revision deployed to hardware.
- WSL2 and Jetson may have different generated artifacts without repository noise.
- Hot fixes made directly on the Jetson must be reproduced on a feature branch or
  they will be lost; direct Jetson commits are discouraged.
- ROS2 builds are validated on the target architecture before merge when hardware
  integration is affected.

## Verification

On WSL2 and Jetson, compare:

```bash
git remote -v
git branch --show-current
git rev-parse HEAD
git status --short
```

Confirm generated directories are ignored:

```bash
git check-ignore -v ros2_ws/build ros2_ws/install ros2_ws/log
```
