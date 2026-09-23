# Repository Working Agreement

## Project and hardware

This repository supports a Hiwonder robot whose onboard computer is an NVIDIA
Jetson Orin NX 16GB. The host development copy normally runs in Windows with
WSL2; a separate Jetson checkout is used for ROS2 builds and hardware tests.
GitHub (`EasonSong208/hiwonder-jetson-robot`) is the only authoritative source.

The current milestone is M1: complete an approximately 20 m closed-loop route,
return near the start, and use a color marker for final visual homing.

## Safety rules

- Treat every command that can move the physical robot as a hardware operation.
- Obtain explicit user confirmation immediately before every real-robot motion
  command or test, even when a broader development task has already been approved.
- Never publish to `/cmd_vel`, `/controller/cmd_vel`, `/cmd_vel_nav`, or another
  motion topic by default. Do not send a navigation goal without confirmation.
- Keep the robot stopped while performing interface audits and diagnostics.
- State the expected motion, test area, stop method, and operator responsibility
  before requesting approval for a motion test.
- Do not start, stop, or restart hardware-facing ROS2 nodes unless the user has
  authorized that state change.

## Source and deployment rules

- Work in the existing repository; never create a nested or second Git repository.
- Develop primarily in the WSL2 checkout. Transfer changes through GitHub and
  deploy them to the Jetson checkout for ROS2 builds and real-hardware tests.
- Use feature branches and pull requests. Do not commit or push unless requested.
- Never commit ROS2 `build/`, `install/`, or `log/` directories, local environment
  files, credentials, machine-specific secrets, datasets, bags, or generated models.
- Treat imported Hiwonder/vendor drivers as upstream code. Do not casually modify
  them. Prefer wrappers, launch composition, configuration, or a project-owned ROS2
  package. A vendor-driver change requires a documented reason, minimal diff, and
  explicit user agreement.
- Do not install dependencies unless the user approves it.

## Engineering expectations

- Preserve existing valid content and unrelated user changes.
- Keep mission logic in project-owned packages such as `robot_mission`; do not mix
  experimental mission behavior into vendor packages.
- Separate observation, planning, and actuation. Diagnostic tools should be
  read-only unless their mutating behavior is explicit and approved.
- Document topic types, publishers, subscribers, frames, QoS, and whether a value
  is measured or command-derived before relying on it.
- Record each hardware test in `docs/test_records/` with software revision,
  environment, command, expected result, actual result, and safety notes.
- Every change must be accompanied by concrete verification commands. If a command
  was not run, label it as recommended rather than claiming success.

## Minimum verification handoff

For documentation-only or skeleton changes, report at least:

```bash
git status --short
git diff --check
git diff --stat
```

For later ROS2 package changes, also provide an appropriately scoped build/test
command, normally from the Jetson deployment checkout, without committing generated
workspace directories.
