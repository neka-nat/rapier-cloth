# Changelog

## Unreleased

- Make the shared folding task transactional across Rapier, cloth/contact history, grasp/release commands, events and time. Failed substeps restore the last accepted state before returning the error; task checkpoint costs remain inside benchmark timings. Add grasp, partial-attachment, release and retry regressions in both precisions.

- Add schema-2 folding diagnostics with per-frame timing samples, initial and per-step audits, both attachment kinds, grasp/release coverage and explicit final-settling observations. Add a strict correctness report checker and six-variant f32/f64 suite validation. Full folding, CPU performance and live qualification remain open.

- Differentiate self-friction in material coordinates, including support rotation during slip. Use the full tangent effective mass and retain actual per-vertex corrections through frame refresh, motion shortening and unloading. Add independent virtual-work, force/moment, weak-direction and atomic-error tests; complete folding and CPU qualification remain open.

- Transport sticking self-contact anchors with the supporting material triangle, including rotation about the contact normal and incident vertex/edge supports. Preserve actual tangent impulse increments through frame changes and shortened motion, exclude overlap recovery from physical slip, and retain frame state through checkpoints. Degenerate supporting triangles fail atomically; dense folding and CPU qualification remain open.
- Use coupled rigid supports during initial surface-gap restoration, with fresh normal multipliers and no friction/history updates. This prevents unresolved overlap from producing a spurious velocity in the subsequent physical prediction.
- Couple deforming surface normal contacts with independent single-particle rigid supports. Solve their local complementarity conditions together, preserve free particle motion and retract friction when a support unloads. This repairs supported-fold prediction regressions without increasing substeps or solver iterations; full folding and CPU qualification remain open.
- Bound fixed-plane CCD endpoint distances independently so movement away from a plane cannot consume the starting point's numerical clearance. Zero-clearance starts still fail atomically.
- Add bounded ray, sphere-approach and closest-surface queries, generational cloth-associated material points, rest-edge-path patch selection and a runnable `surface_grasp` example. Preserve nearby-layer ordering for long rays in f32 and reject invalid or over-budget queries instead of returning partial selections.
- Add weighted surface targets and body-local point grasps alongside existing vertex attachments. Preserve support masses, integrate corrections/multipliers into CCD and reject overlapping supports. Both attachment kinds participate in release/removal events, filtering and atomic checkpoints.
- Keep cloth and attachment allocation generations monotonic across checkpoint clones so handles from discarded simulation time cannot reconnect to later allocations. Add surface-target and surface-query budget error variants; update exhaustive matches as needed.
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
