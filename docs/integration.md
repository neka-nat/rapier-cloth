# Integrating with Rapier

`rapier_cloth::rapier` re-exports the Rapier crate for the selected precision.
Applications using the same Rapier 0.34 series and precision can pass
`RigidBodyHandle`, `ColliderHandle`, `SharedShape` and `Pose` directly between them.
Both preludes export `Real`; explicitly import `rapier_cloth::Real` when combining
both with glob imports. See the [minimal example](../README.md#use-from-rust).

## Own the time step

The application owns the Rapier world and the external substep loop. For a 1/60 s
application tick, a typical configuration advances both worlds four times at
1/240 s. Cloth does not add hidden substeps.

1. Keep one `WorldId` for a Rapier world and its cloth world.
2. Capture the starting poses with `SceneSnapshot::capture(id, step, &bodies, &colliders)`.
   Step indices start at zero and increase by one per successful cloth substep.
3. Set the next poses of kinematic bodies.
4. Set `integration_parameters.dt = h` and step Rapier once.
5. Create a query from the updated broad phase and borrow it through
   `RapierScene::new(query, &before, h, gravity)`.
6. Call `RapierClothWorld::step_substep(h, &scene)` once.

The entry point checks the world token, step index and `h`. The token is an
application-supplied identity: it cannot detect incorrectly attaching the same
identity to a different Rapier world, or count how many times the application
actually stepped Rapier. After inserting, removing or directly moving colliders,
step Rapier to update its query structures. Recreating a query alone is insufficient.

## Render a cloth

`cloth.surface()` returns a borrowed `SurfaceView` containing positions and
triangle indices. Physics and display vertices correspond one-to-one. Copy these
buffers to your renderer and end the borrow before the next simulation step.
Use `surface.write_normals(&mut normals)` to fill a normal buffer. A renderer may
instead recompute normals and bounds after updating its geometry, as the
[live viewer](https://github.com/neka-nat/rapier-cloth/blob/main/demos/viewer/src/live.js) does.

## Materials

`ClothMaterial` uses metres, kilograms and seconds. Compliance belongs to discrete
constraints; values are not calibrated continuum fabric moduli. Reassess behavior
when changing mesh resolution, time step or solver iterations.

| Field | Default | Meaning |
|---|---:|---|
| `surface_density` | 0.2 | Mass per square metre; vertex masses use triangle areas |
| `stretch_compliance` | 0 | Edge-length compliance; zero requests inextensible edges |
| `bend_compliance` | 1e-4 | Dihedral bending compliance; larger values allow easier folding |
| `damping` | 0.1 | Exponential velocity damping coefficient, in inverse seconds |
| `friction` | 0.5 | Cloth contribution to contact friction |
| `contact_radius` | 0.005 | Numerical particle radius, not necessarily physical fabric thickness |

For softer folds in the 1 m, 32×32 live scene, the demo sets
`bend_compliance: 1e3` while retaining the other defaults. Tune bending separately
from stretch so that softness does not require elastic edges. Zero stretch
compliance does not guarantee zero numerical strain with a finite iteration budget.
Diagonal triangle edges are not an independent shear material model.

`pin(particle, world_position)` creates a fixed target; `unpin(particle)` releases
it without replacing its current position or velocity. `set_force` sets a persistent
force in newtons; set it to zero to clear it. The live demo multiplies its wind
acceleration by vertex mass.

## Contacts and filters

Supported contacts are fixed spheres, boxes, capsules and half-spaces, and kinematic
spheres, boxes and capsules. Each vertex is treated as a sphere with
`contact_radius`. Candidates use the cloth AABB and are reevaluated after constraint
projection. An unsupported shape or dynamic body that becomes a selected contact
candidate returns `UnsupportedCollision`; its mere presence far away does not
normally make it a contact candidate.

Use Rapier's `QueryFilter` groups/predicate and
`CollisionSettings::excluded_colliders`. Sensors and disabled colliders are also
excluded. These settings filter cloth queries; they do not execute Rapier rigid-body
contact hooks or solver groups. `ignored_colliders` counts enumerated candidates
excluded by the adapter, not colliders already excluded by the query or broad phase.

The kinematic motion budget bounds translation plus rotation-induced surface motion
for all selected kinematic shapes. It includes the shape radius, the collider's
offset from the body origin and Rapier angular velocity over `h`, so a complete
rotation is not mistaken for zero motion. The default budget is half the smallest
cloth particle radius. It also applies to distant selected kinematic shapes to avoid
missing fast crossings; filter out unrelated ones explicitly. On `MotionBudget`, use
`required_substeps` to plan smaller external steps. This is not a full relative-CCD
guarantee.

Static sweeps retain the hit tangent plane for the remainder of the substep and
solve remaining motion along the surface, with zero restitution. Moving surfaces
use discrete contacts at their ending poses. Recovery from preexisting penetration
is separated from physical velocity and friction; penetration introduced by new
kinematic motion is treated as physical contact correction.

Friction acts on relative velocity at the contact point and uses the arithmetic
mean of cloth and collider coefficients, or `friction_override` when set. Rapier's
coefficient combination rule is not used. A Coulomb limit bounds the impulse and
prevents reversing the remaining slip. There is no static-friction history.

## Attachments and grasping

Call `world.attach(desc, &bodies, &colliders)` with an `AttachmentDesc` containing a
cloth handle, body handle, compliance and a list of
`AttachmentPoint { particle, local_anchor }`. Anchors use the body's local frame,
not the collider's. To grasp the current position, compute the anchor with
`body.position().inverse_transform_point(particle_position)`.

- Zero compliance creates a hard target. Its velocity is the displacement from the
  previous particle position to the ending anchor position, divided by `h`.
- Positive compliance creates a soft XPBD target with correction-consistent velocity.
- Multiple attachments on one particle, or an attachment conflicting with a pin,
  are rejected.
- Attachment `excluded_colliders` must belong to its body. Exclusions apply only to
  the attachment's particle/collider pairs.
- `release(handle)` preserves the most recently computed particle velocity.
  `drain_attachment_events()` reports releases and removals. Body removal or disabling
  is reported on the next successful step.
- Handles carry arena identity and generation to avoid reconnecting to reused indices.

See [moving_anchor](../examples/moving_anchor.rs) for a minimal example and
[pick-and-place](examples.md#pick-and-place-recording) for multi-vertex grasping.

## Failures and recovery

The core commits particle state only on success. The Rapier bridge stages all cloths
and commits them together. Contact budgets bound both query counts and retained
contact keys; excess contacts are rejected rather than silently dropped. Degenerate
constraints, non-finite values and inconsistent hard targets also return errors.
`next_step_index()` remains at the failed substep.

A numerical failure can leave cloth at `t` while Rapier has already reached `t+h`.
When `is_desynchronized()` becomes true, stop stepping. To retry with a different `h`:

1. Save `cloth.checkpoint()` and the application's complete Rapier state before the step.
2. Restore Rapier to the saved time, including colliders, contacts, islands, joints
   and query structures, not just body poses.
3. Call `cloth.restore(&checkpoint)` and restore the application clock, step index
   and inputs.
4. Advance both worlds again with the new `h`, using handles from the restored state.

Checkpoints are in-memory state for the same cloth world, not a public serialization
format. Discard handles and events created after the checkpoint. The repository's
checkpoint tests use a fixed-world fixture; a general application must also preserve
controllers and other external state. A demo may instead recreate both worlds on reset.

Invalid world tokens or `h` values are rejected before numerical work begins.
Corrected metadata can be retried, but the application must ensure Rapier's actual
time remains correct.

See [limitations](compatibility.md#simulation-limits) for unsupported collisions and
mesh configurations.
