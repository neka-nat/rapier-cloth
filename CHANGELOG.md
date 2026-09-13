# Changelog

## Unreleased

- Add experimental persistent static/kinetic surface friction with material witnesses, physical normal-load limits, equal-and-opposite deformable updates and continuous checks for position corrections. Preserve legacy particle friction and avoid applying the position-level kinetic load twice.
- Transport Rapier material anchors with the complete rigid transform; add a defaulted `ContactSource::transport_surface_anchor` callback for custom sources and `Cloth::clear_contact_history` for external model changes. Include anchor/context state in checkpoints and invalidate it on separation, pin/grasp changes, teleports, shapes and collision-setting changes.
- Compute all three closest-triangle face weights from signed areas, avoiding a negative weight caused by cancellation near a vertex. Keep strict public witness validation.
- Tighten self-collision broad-phase bounds with six diagonal projection intervals and outward rounding. Preserve swept convex-hull coverage and physical activation margins while rejecting separated diagonal features.
- Accelerate exactly inextensible cloth attached to hard anchors with redundant rest-edge-path distance bounds. Keep compliant stretch and soft targets unchanged, allow folded rest meshes to unfold, and clear bounds on release. Include the corrections in existing continuous motion checks and memory diagnostics.
- Complete elastic and contact projections as one trial before continuous motion checks. Restore contact thickness after a shortened trial with a separately checked correction, fixing edge-length recovery for compressed cloth supported by a surface.
- Reuse exact unchanged self-contact queries and certified initial geometry. Invalidate derived caches on changed inputs, topology or settings, and preserve initial-intersection checks, retained-contact limits and scratch-memory accounting.
- Add opt-in `continuous_rigid_collision` for bounded sphere/box/capsule/fixed-plane sweeps, including all solver correction stages with self-collision disabled. Add defaulted external motion callbacks, immutable shape snapshots, shared work limits and atomic failure tests. Update explicit contact-settings literals for the new field.
- Validate the declared rigid trajectory against Rapier 0.34's normalized quaternion integration and damping. Reject ambiguous velocity-based rotations reaching half a turn per external step; existing motion limits still apply. Add transformed analytical crossings, offset arcs and moving/rotating support regressions.
- Add `--continuous-rigid-collision` to the folding diagnostic, implying rigid-surface contacts and recording the actual swept minimum. Full folding and real-time performance remain unqualified.
- Add folding fixture version 2, capturing grasp anchors after the final approach pose to avoid commanding settled cloth into the table. Version 1 remains available/default for historical replay; every trajectory sample, physical parameter and acceptance limit is preserved.
- Add opt-in discrete whole-triangle contact against Rapier spheres, boxes, capsules and fixed half-spaces, with half-thickness offsets, canonical shared-feature witnesses, scoped patch exclusions and a shared self/rigid work budget.
- Add `ClothContactSettings::rigid_surface_collision`, `SurfaceWitness`, shared immutable mesh access and a defaulted budget-aware surface callback. Initial rigid intersections and unresolved final separation return typed errors; update exhaustive error matches and contact-settings literals as needed.
- Fix cached static-sweep contact duplication when continuous prediction retries a trial pose. Repeated queries update the witness for each retained key without consuming extra contact capacity.
- Add experimental continuous self-contact checks for prediction, accepted constraint corrections and the final substep sweep, with swept witnesses, explicit work/convergence failures and a friction-free tangential-motion regression. Folding qualification remains under development.
- Add opt-in discrete, thickness-aware self-collision with refitted triangle/edge hierarchies, canonical contact features, bounded work and atomic failures.
- Add generalized four-particle surface contacts and per-cloth contact history restored by checkpoints. Existing particle contact sources remain supported through a defaulted surface callback.
- Pre-release API additions include `ClothContactSettings`, collision work/error types, and `StepReport::surface_collision`; update exhaustive error matches and report struct literals as needed.
- Use the MIT license for the libraries and demos.
- Add English documentation, example entry points and demo controls, with automated documentation and example checks.
- Soften bending in the live demo while retaining inextensible edges and the existing solver budget; add smooth spatial gusts for local billowing.
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
