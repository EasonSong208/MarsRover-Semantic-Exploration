# Architecture decision records

Read accepted ADRs when changing a durable boundary rather than relying on chat
history. Do not rewrite an accepted ADR to hide a changed decision; supersede it
with a new record and link both directions.

| ADR | Status | Decision |
|---|---|---|
| `0001-development-and-deployment-layout.md` | Accepted | GitHub authority, WSL2 development and Jetson deployment layout |
| `0001_repo_structure.md` | Legacy | Early repository layout record; retained for history |
| `0002-motion-safety-and-authorization.md` | Accepted | Physical motion approval and best-effort cleanup boundary |
| `0003-m1a-action-sequence-and-odometry.md` | Accepted | Drive-turn-drive action model and command-derived odometry limitation |

Two historical files use number 0001. They are retained without renaming to avoid
breaking existing references. New ADR numbers must be unique and monotonically
increasing.
