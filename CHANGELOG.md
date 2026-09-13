# Changelog

## Unreleased

- Add a live Rust CPU demo with a Three.js UI, per-connection worlds, demand-driven WebSocket frames, draping/wind scenes, bounded obstacle controls, pause/reset/release and reconnection. Keep transport dependencies in an independent unpublished demo crate.
- Add f32/f64 live browser tests against actual Rust responses and a one-command launcher.

- Reduce CPU cloth cost with closed-form dihedral gradients, a fast path for unwrapped angle differences, angle-only diagnostics and linear-time strain quantiles.
- Merge sorted contact states and cached sweep planes in linear time while preserving contact order, budgets, friction, atomic failure and attachment exclusions.
- Add a 32×32 benchmark measuring four actual sequential physics substeps per frame, with before/after data for stationary contact, hanging cloth and a moving sphere.
- Include retained contact-state arrays in scratch memory diagnostics; historical tree nodes were excluded from that metric.

## 0.1.0 candidate — 2026-09-13

- Add an engine-independent CPU XPBD core with validated fixed triangle meshes, area-based masses, stretch and signed dihedral constraints, pins and compliant targets.
- Add one-way Rapier 0.34 integration with static primitives, particle sweeps, bounded kinematic motion, contact filters, kinetic friction and local-anchor attachments.
- Preserve cloth state on failed substeps; provide checkpoint/restore and generational cloth/attachment handles.
- Add f32/f64 reference tests, headless pick-and-place recording, a Three.js replay viewer, scaling benchmarks and extracted-package consumer checks.

Self-collision, two-way coupling, arbitrary collision meshes and language bindings are outside this candidate. Crates.io publication has not been performed.
