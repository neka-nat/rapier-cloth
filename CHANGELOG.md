# Changelog

## 0.1.0 candidate — 2026-09-13

- Add an engine-independent CPU XPBD core with validated fixed triangle meshes, area-based masses, stretch and signed dihedral constraints, pins and compliant targets.
- Add one-way Rapier 0.34 integration with static primitives, particle sweeps, bounded kinematic motion, contact filters, kinetic friction and local-anchor attachments.
- Preserve cloth state on failed substeps; provide checkpoint/restore and generational cloth/attachment handles.
- Add f32/f64 reference tests, headless pick-and-place recording, a Three.js replay viewer, scaling benchmarks and extracted-package consumer checks.

Self-collision, two-way coupling, arbitrary collision meshes and language bindings are outside this candidate. Crates.io publication has not been performed.
