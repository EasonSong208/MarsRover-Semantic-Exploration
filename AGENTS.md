# Repository Working Agreement

## Project and hardware

This repository supports a Hiwonder robot whose onboard computer is an NVIDIA
Jetson Orin NX 16GB. The host development copy normally runs in Windows with
WSL2; a separate Jetson checkout is used for ROS2 builds and hardware tests.
GitHub (`EasonSong208/hiwonder-jetson-robot`) is the only authoritative source.

The current milestone is M1: complete an approximately 20 m closed-loop route,
return near the start, and use a color marker for final visual homing.

## Start here

Before changing code or proposing a hardware command, read these maintained entry
documents in order:

1. `AGENTS.md` — non-negotiable working and safety rules.
2. `docs/project_status.md` — current verified state, open risks and next gate.
3. `docs/architecture.md` — machine layout, ROS2 chains and software boundaries.
4. `docs/rgbd_chain_audit.md` — RGB-D facts and remaining runtime checks when
   working on perception or visual homing.
5. Relevant records in `docs/decisions/` — accepted design constraints.

Older uppercase context/status files and topic-specific audit documents remain
supporting evidence. If they conflict with the maintained entry documents, verify
against current code and Jetson runtime before editing either account.

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
- Never run `start_app_node.service`, a vendor demo, `minimal_bringup`, or a mission
  executable merely to inspect an interface. Static and already-running graph
  inspection come first.
- A parameter such as `confirmed:=true` is not a substitute for immediate user
  approval of that specific physical test.

## Source and deployment rules

- Work in the existing repository; never create a nested or second Git repository.
- Develop primarily in the WSL2 checkout. Transfer changes through GitHub and
  deploy them to the Jetson checkout for ROS2 builds and real-hardware tests.
- For explicitly requested temporary deployment of uncommitted files, targeted
  `rsync -aR` over SSH is allowed. List every path and compare SHA-256 on both
  machines. This does not replace the feature-branch/PR source-of-truth workflow.
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
- Keep `docs/project_status.md` current when a capability, risk, deployment state,
  or next test gate changes. Update `docs/architecture.md` only for structural
  changes, and create an ADR when a durable design or safety decision changes.
- Mark a runtime or hardware property `UNKNOWN` unless it is supported by current
  deployed source or a recorded observation. Vendor PDFs are indexes, not final
  authority; deployed Jetson source wins when they disagree.

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
