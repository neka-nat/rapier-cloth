# rapier-cloth

CPU cloth simulation for Rust applications using Rapier 3D. Add deformable triangle
meshes to an existing physics world, with contacts, pinned vertices and attachments
to rigid bodies. The default solver uses extended position-based dynamics (XPBD);
an optional [implicit shell solver](docs/implicit.md) supports folding experiments.

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
CPU server and starts the browser UI. For implicit towel folding and Rapier
end-effector pose commands, run `npm --prefix demos/viewer run live:implicit` and
open `/live.html?scene=implicit_towel&paused=1`. Strict stopping is the default;
the scene also offers an explicit validated-approximation policy applied on Reset. Use 5 mm moves, 5 degree rotations
and independent releases. See [robot control](docs/robot-control.md) and the
[live demo guide](docs/live-demo.md). General folding and wall-clock real-time
performance remain under development.

## Use from Rust

```toml
[dependencies]
rapier-cloth = "0.2"
```

A repository checkout also works as a path dependency
(`rapier-cloth = { path = "../rapier-cloth" }`). The engine-independent solver is
available on its own as [`rapier-cloth-core`](https://crates.io/crates/rapier-cloth-core).
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
support spheres, boxes, capsules, convex hulls, compounds of those solids (such as
convex decompositions) and fixed half-spaces. Kinematic obstacle motion is bounded
per substep.

Coupling is one-way. Experimental [self-collision](docs/integration.md#discrete-self-collision)
and [whole-triangle rigid contact](docs/integration.md#rigid-surface-contact) are
available as opt-in settings. Cloth-to-cloth collision, reactions on dynamic rigid
bodies and arbitrary collision meshes are not implemented. Experimental
[continuous rigid-surface checks](docs/integration.md#continuous-rigid-surface-collision)
cover bounded primitive motion and solver corrections. The optional
[implicit towel example](docs/implicit.md#run-the-folding-example) tests a separate
fold, release and settle trajectory at 0.04 or 0.1 s per physical step. General
folding and its real-time CPU budget remain under development.
Use [surface queries and grasp attachments](docs/grasping.md) to select exposed
cloth layers and command a point or patch; static friction alone is not a grasp model.
Read the [compatibility and limitations](docs/compatibility.md) before integrating.

## Documentation

- [Documentation index](docs/README.md)
- [Runnable examples and recording playback](docs/examples.md)
- [Experimental implicit shell solver](docs/implicit.md)
- [Performance measurement](docs/performance.md)
- [Contributing](CONTRIBUTING.md) · [Changelog](CHANGELOG.md)

Licensed under the [MIT License](LICENSE-MIT).
