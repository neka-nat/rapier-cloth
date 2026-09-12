# Implementation evidence

Scope: PR0, PR1a, PR1b, PR2a, PR2b, PR3a, PR3b, PR3c and R0 from [the plan](implementation-plan.ja.md). Registry publication is outside R0. The labels are implementation units; the review branch groups them into coherent commits.

| Phase | State | Evidence |
|---|---|---|
| PR0 | Implemented; local checks passed | Both precision graphs, independent direct-Rapier consumers, rejected feature combinations, core dependency boundary |
| PR1a | Implemented; local tests passed | Grid topology, invalid meshes, surface density mass, generational handles |
| PR1b | Implemented; local tests passed | Analytical integration/distance references, dihedral finite differences, 10,000-step drape in f32/f64 |
| PR2a | Implemented; local tests passed | Four primitives, corner contacts, penetration stabilization, atomic contact budget failure |
| PR2b | Implemented; local tests passed | Thin-box sweep, filters and BVH lifecycle, world/h/step contracts, checkpoint replay |
| PR3a | Implemented; local tests passed | Local anchors, rotation/translation, release, deletion/reuse, selective exclusions, kinematic motion limits |
| PR3b | Implemented; local tests passed | Coulomb slowdown and moving-surface velocity; canonical pick-and-place and exact JSON roundtrip in f32/f64 |
| PR3c | Verification in progress | Actual f64 Rust recording matched browser GPU buffers/body poses; playback/seek/camera/upload tests passed; final benchmarks pending |
| R0 | Verification in progress | Archives and extracted consumers exercised; final package script run and platform CI pending |

The resolved dependency graph requires Rust 1.90, despite Rapier's own 1.86 declaration: nalgebra 0.35, safe_arch 1.2 and wide 1.7 need 1.89; ordered-float 5.5 needs 1.90. Development uses 1.93.0. Linux local MSRV checks passed; remote platform completion is recorded separately after checking the actual runs.

The horizontal 32×32 hanging fixture completed 10,000 substeps at h=1/240 and 8 iterations. The maximum of the final 1,000 steps' p95 stretch was 0.016481519 (f32) and 0.01647869953082659 (f64), below 0.05. The free corner dropped below y=0.2, so this checks actual deformation.

The canonical manipulation fixture uses 16×16 vertices, 0.3m size, 1/240s substeps, 8 iterations, default material. Release is at 4s with 0.2m/s horizontal velocity; the final maximum height is approximately 0.005012m. Capture happens after settling, and transport distance is measured from the captured position. All recorded steps passed finite-value checks. The maximum single-edge stretch during the whole operation was about 18.22% for f32 and 5.67% for f64 in the local runs; the hanging fixture's 5% p95 gate is a different measure and is not a blanket deformation guarantee for this operation.

The replay viewer passed two Chromium tests against the actual f64 recording: first/middle/last frames and backward seeking, GPU f32 conversion, normals/bounds, body poses/local offsets, anchors/pins, numerical labels, playback/pause, wireframe, marker visibility, camera drag/reset, and valid/invalid file input. This is replay validation, not browser-side cloth physics.
