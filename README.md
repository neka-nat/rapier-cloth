# rapier-cloth

CPU cloth simulation for Rust applications using Rapier 3D. Add deformable triangle
meshes to an existing physics world, with contacts, pinned vertices and attachments
to rigid bodies. The solver uses extended position-based dynamics (XPBD).

`rapier-cloth-core` provides the engine-independent solver; `rapier-cloth` adds
Rapier collision queries, attachments and time synchronization.

## Try the live demo

From a repository checkout, with Rust and Node.js 22.12 or newer:

```bash
npm --prefix demos/viewer ci
npm --prefix demos/viewer run live
```

Open **http://127.0.0.1:5173/live.html** to drape cloth over a sphere, adjust wind,
move the obstacle and release pinned vertices. The launcher builds a local Rust
CPU server and starts the browser UI. See the [live demo guide](docs/live-demo.md)
for controls and requirements.

## Use from Rust

This is a pre-release project. Use a checkout as a path dependency:

```toml
[dependencies]
rapier-cloth = { path = "../rapier-cloth" }
```

Rust 1.90 or newer is required. The default is `f32`, compatible with `rapier3d 0.34`.
For `rapier3d-f64 0.34`, set `default-features = false, features = ["f64"]`.
Select exactly one precision; do not use `--all-features`.

```rust
use rapier_cloth::{rapier::prelude::*, prelude::*};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let id = WorldId::new();
    let mut rigid = PhysicsWorld::new();
    let h = 1.0 / 240.0;
    rigid.integration_parameters.dt = h;
    let mut cloths = RapierClothWorld::new(id);
    let cloth = Cloth::new(
        GridBuilder::new(32, 32).origin(Vec3::Y).build()?,
        ClothMaterial::default(),
    )?;
    let handle = cloths.add_cloth(cloth);

    // Run this sequence once per substep, using the same h for both worlds.
    let before = SceneSnapshot::capture(id, 0, &rigid.bodies, &rigid.colliders);
    // Set any kinematic targets here, after taking the snapshot.
    rigid.step();
    let query = rigid.broad_phase.as_query_pipeline(
        rigid.narrow_phase.query_dispatcher(),
        &rigid.bodies, &rigid.colliders, QueryFilter::default(),
    );
    cloths.step_substep(h, &RapierScene::new(query, &before, h, rigid.gravity))?;
    let surface = cloths.cloth(handle)?.surface();
    assert_eq!(surface.positions.len(), 1024);
    Ok(())
}
```

See [integration](docs/integration.md) for step ordering, rendering, materials,
collision filters, attachments and recovery after a failed step.

## Capabilities and limits

The solver supports fixed triangle topology, surface density, stretch and dihedral
bending constraints, pins and attachments with body-local anchors. Collisions
support spheres, boxes, capsules and fixed half-spaces. Kinematic obstacle motion
is bounded per substep.

Coupling is one-way. Self-collision, cloth-to-cloth collision, reactions on dynamic
rigid bodies, arbitrary collision meshes and edge/face CCD are not implemented.
Use explicit attachments for grasping; static friction alone is not a grasp model.
Read the [compatibility and limitations](docs/compatibility.md) before integrating.

## Documentation

- [Documentation index](docs/README.md)
- [Runnable examples and recording playback](docs/examples.md)
- [Performance measurement](docs/performance.md)
- [Contributing](CONTRIBUTING.md) · [Changelog](CHANGELOG.md)

Licensed under the [MIT License](LICENSE-MIT).
