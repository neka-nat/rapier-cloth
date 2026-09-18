# rapier-cloth-core

An engine-independent CPU cloth solver using extended position-based dynamics
(XPBD). It provides triangle meshes, area-based masses, edge-length and dihedral
bending constraints, fixed vertices, external forces and collision-source interfaces.
The default solver's only normal dependency is glam. The optional `implicit`
feature adds faer and nalgebra for an experimental global shell solve (f64 only).

For Rapier collision queries, attachments and time synchronization, use
[rapier-cloth](https://github.com/neka-nat/rapier-cloth).

## Use from a checkout

```toml
[dependencies]
rapier-cloth-core = { path = "../rapier-cloth/crates/rapier-cloth-core" }
```

Requires Rust 1.90 or newer. Select exactly one precision: `f32` is the default;
for f64, add `default-features = false, features = ["f64"]`. Do not use
`--all-features`.

```rust
use rapier_cloth_core::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mesh = GridBuilder::new(32, 32).origin(Vec3::Y).build()?;
    let mut cloth = Cloth::new(mesh, ClothMaterial::default())?;
    for i in 0..32 {
        cloth.pin(i, cloth.positions()[i as usize])?;
    }
    let mut solver = Solver::new();
    solver.step(
        &mut cloth,
        1.0 / 240.0,
        -Vec3::Y * 9.81,
        &SolverSettings::default(),
    )?;
    assert_eq!(cloth.surface().positions.len(), 1024);
    Ok(())
}
```

This example has gravity and pins but no external collision source. The caller owns
the substep loop and rendering. Mesh topology is fixed. Experimental discrete
self-collision can be enabled with
`cloth.set_contact_settings(Some(ClothContactSettings::default()))?`.
This separates nonincident vertex-face and edge-edge features using a physical
thickness (default 1 mm). It rejects initial intersections and uses bounded,
refitted candidate searches. Setting `continuous_self_collision: true` also checks
prediction, accepted constraint corrections and the final substep sweep. This
experimental mode can return a typed failure when a safe advance cannot be found;
it has not qualified the complete folding task or its real-time performance.
Experimental XPBD surface friction retains static/kinetic contact history.
The Rapier adapter offers opt-in discrete
whole-triangle rigid contacts through the core's generalized surface interface.
Its candidates share the configured work budget with self-collision. The core
contains no rigid-shape queries; `rigid_surface_collision` selects this behavior
only in an adapter that supports it.
Tearing is not implemented. Material compliance describes discrete constraints and requires
tuning with the chosen grid, time step and iteration count.

For a separate continuum shell model, enable `f64,implicit` and call
`cloth.set_implicit_solver(Some(ImplicitSettings::default()))?`. This selects a
global backward-Euler solve with a Neo-Hookean membrane, dihedral bending,
positive-gap barrier and lagged smoothed Coulomb friction. Hard particle targets
are supported; soft and weighted targets are rejected. It requires positive
initial clearance, a positive activation margin and continuous contact checks.
Its smoothed kinetic friction permits creep and does not enforce exact sticking.
See the repository's [implicit solver guide](https://github.com/neka-nat/rapier-cloth/blob/main/docs/implicit.md)
for the optional `ImplicitExecution::Parallel4` CPU budget (serial by default),
checkpoint/recovery behavior, physical settings, the complete towel fixture and
qualification limits.

See the repository's [documentation](https://github.com/neka-nat/rapier-cloth/tree/main/docs)
and [examples](https://github.com/neka-nat/rapier-cloth/tree/main/examples).

Licensed under the [MIT License](LICENSE-MIT).
