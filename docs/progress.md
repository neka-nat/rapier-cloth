# Implementation evidence

The scope is PR0, PR1a, PR1b, PR2a, PR2b, PR3a, PR3b, PR3c and R0 in [the implementation plan](implementation-plan.ja.md). A phase is complete only when its stated gates have evidence. GitHub publication of the packages is outside R0.

| Phase | State | Evidence |
|---|---|---|
| PR0 | In progress | Workspace, precision features, consumer fixtures and CI authored; verification pending |
| PR1a | Pending | |
| PR1b | Pending | |
| PR2a | Pending | |
| PR2b | Pending | |
| PR3a | Pending | |
| PR3b | Pending | |
| PR3c | Pending | |
| R0 | Pending | |

PR0 dependency finding: the resolved Rapier 0.34 graph includes nalgebra 0.35.0 (Rust 1.89), safe_arch 1.2.0 (1.89), wide 1.7.0 (1.89), and ordered-float 5.5.0 (1.90). Therefore Rapier's own 1.86 declaration cannot establish this workspace's MSRV. The workspace targets 1.90; actual checks are pending. Toolchain 1.93.0 remains the development baseline.
