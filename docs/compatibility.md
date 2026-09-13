# Compatibility and limitations

## Toolchain and dependencies

| Component | Supported configuration |
|---|---|
| Minimum Rust | 1.90, edition 2024 |
| Repository toolchain | Rust 1.93.0, pinned in `rust-toolchain.toml` |
| Rapier | `rapier3d 0.34` for f32, `rapier3d-f64 0.34` for f64 |
| Math | glam 0.33; `Vec3` for f32 and `DVec3` for f64 |
| Precision | Exactly one of `f32` and `f64`; f32 is the default |
| Library CI | Linux, Windows and macOS, both precisions |
| Browser demos | Node.js >=22.12, WebGL; Three.js 0.186 and Vite 8.3 |

Cargo.lock and the viewer's package-lock.json pin the repository's resolved
versions. The core's only normal dependency is glam. It has no Rapier, Parry,
renderer or serde dependency; the bridge re-exports the selected Rapier crate.
The resolved graph requires Rust 1.90 even though some dependencies allow older Rust.

Precision features are mutually exclusive after Cargo feature unification. Do not
use `--all-features` or combine a default-f32 dependency with an f64 dependency.
The project does not promise bitwise determinism across CPUs or precisions.

## Simulation limits

| Area | Supported | Not implemented |
|---|---|---|
| Cloth mesh | Fixed, oriented triangle topology | Tearing, remeshing and automatic repair |
| Material | Edge-length and dihedral XPBD constraints | Calibrated continuum fabric or independent shear model |
| Fixed obstacles | Spheres, boxes, capsules, half-spaces | Arbitrary triangle meshes and compound shapes |
| Moving obstacles | Kinematic spheres, boxes and capsules within the motion budget | Dynamic obstacle contacts and unrestricted fast motion |
| Coupling | One-way obstacle-to-cloth interaction | Cloth reaction forces on dynamic rigid bodies |
| Cloth collision | Particle contacts and static particle sweeps | Self-collision, cloth-to-cloth collision and edge/face CCD |
| Grasping | Pins and body-local attachment targets | Grasping based only on static friction |
| Recovery | In-memory checkpoint of one cloth world | Public serialized checkpoints or automatic Rapier rollback |

Unsupported collision candidates return errors. Filter unrelated colliders when
necessary. A thin obstacle can pass between vertices of a coarse cloth mesh;
particle collision does not test entire triangle interiors. Cloth can intersect
itself because self-collision is absent.

Inconsistent winding, non-manifold edges, isolated vertices and zero-area triangles
are rejected. Compliance values depend on the discrete setup; evaluate strain and
contact error when changing resolution, substep size or iteration count.

See [integration](integration.md) for filters, kinematic motion limits, friction
rules and recovery, and [performance](performance.md) for measurement boundaries.
