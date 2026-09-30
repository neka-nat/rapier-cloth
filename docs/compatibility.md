# Compatibility and limitations

## Toolchain and dependencies

| Component | Supported configuration |
|---|---|
| Minimum Rust | 1.90, edition 2024 |
| Repository toolchain | Rust 1.93.0, pinned in `rust-toolchain.toml` |
| Rapier | `rapier3d 0.34` for f32, `rapier3d-f64 0.34` for f64 |
| Math | glam 0.33; `Vec3` for f32 and `DVec3` for f64 |
| Precision | Exactly one of `f32` and `f64`; f32 is the default |
| Experimental shell solver | Optional `implicit` feature; f64 runtime, serial or opt-in four-worker execution; Linux test job |
| Library CI | Linux, Windows and macOS, both precisions |
| Browser demos | Node.js >=22.12, WebGL; Three.js 0.186 and Vite 8.3 |

Cargo.lock and the viewer's package-lock.json pin the repository's resolved
versions. The default core's only normal dependency is glam. Enabling `implicit`
adds faer 0.24, nalgebra 0.35 and rayon 1.10. The core has no Rapier, Parry, renderer or serde
dependency; the bridge re-exports the selected Rapier crate.
The resolved graph requires Rust 1.90 even though some dependencies allow older Rust.

Precision features are mutually exclusive after Cargo feature unification. Do not
use `--all-features` or combine a default-f32 dependency with an f64 dependency.
The project does not promise bitwise determinism across CPUs or precisions.

## Simulation limits

| Area | Supported | Not implemented |
|---|---|---|
| Cloth mesh | Fixed, oriented triangle topology | Tearing, remeshing and automatic repair |
| Material | Edge-length and dihedral XPBD constraints; optional isotropic Neo-Hookean shell | Calibrated directional fabric |
| Fixed obstacles | Spheres, boxes, capsules, half-spaces, convex hulls, and compounds of those solids (for example convex decompositions) | Triangle meshes, height fields and other composite shapes |
| Moving obstacles | Kinematic spheres, boxes, capsules, convex hulls and their compounds; within the motion budget for discrete queries, certified by sweeps for continuous ones | Dynamic obstacle contacts and unrestricted fast motion |
| Coupling | One-way obstacle-to-cloth interaction | Cloth reaction forces on dynamic rigid bodies |
| Cloth collision | Particle contacts/static particle sweeps, opt-in self-contact and whole-triangle rigid contacts, separate optional continuous checks | Cloth-to-cloth collision and unrestricted continuous motion |
| Garments | A parametric sewn T-shirt as one mesh (shared or stitched seams, flat-folded rest state, notch or round neck) with landmarks, layer patches, seam and sleeve stiffness and warp/weft material axes; see [garments](garments.md) | Seam allowance geometry, sliding threads, set-in sleeves, several garments colliding |
| Grasping | Pins and body-local attachment targets | Grasping based only on static friction |
| Recovery | In-memory checkpoint of one cloth world | Public serialized checkpoints or automatic Rapier rollback |

Unsupported collision candidates return errors. Filter unrelated colliders when
necessary. A thin obstacle can pass between vertices of a coarse cloth mesh;
particle collision does not test entire triangle interiors. The separate
`rigid_surface_collision` setting covers triangle interiors against the supported
primitives with exact vertex, edge and face distances, using half the physical
thickness as the rigid offset. It replaces
particle contacts for that cloth. Those queries are discrete unless
`continuous_rigid_collision` is also enabled. The latter checks primitive sweeps
and correction batches, independently of self-collision. It uses a declared
COM-linear/angular endpoint trajectory and rejects velocity-based commands reaching
half a turn per external step. The kinematic motion budget applies only while some
cloth in the world relies on discrete or particle contacts.
Changed collider shapes/local poses, unsupported dynamic obstacles, unresolvable
initial gaps and unresolved final separation return typed errors. See the
[continuous rigid contact contract](integration.md#continuous-rigid-surface-collision)
for numerical clearance, supported motion and retry requirements.

Discrete self-contact
checks nonincident vertex-face and edge-edge proximity, with a physical thickness
independent of particle radius. It refreshes bounds after constraint iterations,
rejects preexisting intersections and fails on exhausted work/contact limits.
Fast motion or constraint corrections can still cross between discrete queries.
The additional `continuous_self_collision` option bounds every accepted
self-motion batch, with explicit convergence/work failures.
The [implicit solver guide](implicit.md) describes a separate 32×32 towel task
and its acceptance checks. These experiments do not qualify arbitrary robotic
folding, moving rigid contact with the implicit solver, or real-time performance.

Inconsistent winding, non-manifold edges, isolated vertices and zero-area triangles
are rejected. Compliance values depend on the discrete setup; evaluate strain and
contact error when changing resolution, substep size or iteration count.

See [integration](integration.md) for filters, kinematic motion limits, friction
rules and recovery, and [performance](performance.md) for measurement boundaries.
