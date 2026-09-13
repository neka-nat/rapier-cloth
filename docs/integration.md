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

With zero stretch compliance and hard pins or attachments, the solver also bounds
each free vertex's distance to a reachable hard anchor in each connected hard-target
region by a shortest rest-edge path length. Corrections from different regions are
averaged so that selecting one gripper does not open a gap at their boundary.
These one-sided bounds accelerate propagation from the grasp: they
follow from inextensible edge lengths and leave vertices inside the bound free to
buckle. Local edge constraints remain necessary; the bounds do not guarantee a
maximum local strain. The approach follows
[long-range attachments](https://matthias-research.github.io/pages/publications/sca2012cloth.pdf),
using edge-path upper bounds rather than distances through the rest surface's
interior.

The bounds use rest topology, so folded rest meshes can unfold. They are disabled
for positive stretch compliance and soft targets, and are rebuilt when topology
or the set of hard anchors changes. Releasing all anchors removes the bounds.
Their projections are part of the same trial and continuous-collision checks as
other constraints; they are not additional physical time steps or force sensors.

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

## Discrete self-collision

Self-collision is experimental and disabled until a cloth opts in:

```rust
use rapier_cloth::{ClothContactSettings, CollisionLimits};

cloth.set_contact_settings(Some(ClothContactSettings {
    thickness: 0.001,           // Full physical thickness in metres.
    activation_margin: 0.0001,  // Extra candidate range, not extra thickness.
    self_collision: true,
    continuous_self_collision: false,
    rigid_surface_collision: false,
    continuous_rigid_collision: false,
    static_friction: 0.6,       // Reserved; static stick/slip is not implemented yet.
    kinetic_friction: 0.5,
    limits: CollisionLimits::default(),
}))?;
```

The core constrains nonincident vertex-face and edge-edge features on either side
of the sheet. Shared vertices/edges are excluded by topology; connected components
and pinned regions are not excluded wholesale. Physical self-contact separation
is `thickness`; Rapier's current external particle contacts still use
`ClothMaterial::contact_radius`. This option does not add rigid triangle-surface
collision or continuous self-collision. Use small-motion fixtures and inspect
penetration/strain; the mode is not yet a qualified robotic folding system.

`CollisionLimits` separates cumulative candidate-pair work (default 2,000,000 per
substep), retained contact keys (65,536), and CCD distance-evaluation work
(2,000,000). The CCD counter is zero in the discrete implementation. Candidate
work includes repeated refreshes and topologically incident candidates examined
by the broad phase. Retained keys are bounded by maximum capacity, not summed
across iterations. `SolverSettings::max_contacts` additionally limits the combined
legacy and surface contacts. Limits return typed errors; contacts are never silently
dropped. `StepReport::surface_collision` reports successful-step work/high-water
counts, and its scratch-byte estimate includes reusable hierarchy/contact buffers.

Contact history belongs to each cloth and participates in world checkpoints and
atomic stepping. `set_positions` and changed contact settings invalidate it; an
invalid setting or failed substep preserves the previous physical state. Reapplying
identical settings preserves history. `contact_history_len()` reports retained
touching surface contacts. The `static_friction` coefficient is validated but its
static-cone/history behavior remains unimplemented; kinetic friction currently
acts on relative velocity with equal-and-opposite updates on the two cloth features.

Custom core collision adapters can implement the defaulted
`ContactSource::surface_contacts` method, returning `SurfaceContact` constraints
with up to four particles and stable, unique `SurfaceContactKey` values. The
existing particle-contact callback remains available. Invalid geometry, unresolved
initial self-intersections and infeasible all-fixed surface contacts return errors
without committing the cloth state.

For bounded external surface queries, override
`ContactSource::surface_contacts_with_work` and charge each primitive candidate
through `CollisionWork::charge` using the owning cloth's limits. These counters
are shared with built-in self-contact/CCD; the default callback forwards to
`surface_contacts`. Every contact stage can be queried repeatedly within one
substep, including prediction. Keep cached keys unique and update their witnesses
when retrying a trial pose.

## Rigid surface contact

Enable the experimental Rapier surface adapter per cloth:

```rust
cloth.set_contact_settings(Some(ClothContactSettings {
    rigid_surface_collision: true,
    self_collision: true,
    continuous_self_collision: true,
    ..Default::default()
}))?;
```

The adapter queries complete triangles against spheres, boxes and capsules, and
uses vertex inequalities to cover triangles against a fixed half-space. It finds
small obstacles inside a triangle even when every cloth vertex misses them.
Canonical vertex/edge/face witnesses avoid counting a shared boundary twice;
collider identities include their handle generation. Gripper exclusions apply
only when all nonzero support particles belong to the selected excluded patch.
An adjacent unselected feature remains eligible for collision.

Required rigid-surface separation is `thickness / 2`; activation adds only a
candidate margin. For example, 1 mm full thickness rests with its midsurface
0.5 mm above a plane. The legacy particle contact/sweep path is disabled for that
cloth, so its numerical `contact_radius` does not inflate the surface offset.
Kinetic friction combines the new cloth coefficient with collider friction using
their arithmetic mean; `friction_override` overrides both coefficients. The
static coefficient remains reserved until static stick/slip is implemented.

This configuration performs discrete rigid queries. The continuous flag in the
example applies to **self-collision only**. Rigid CCD has a separate opt-in below.
Kinematic sphere/box/capsule poses use the existing motion-budget contract, and
pre-step poses are used for stabilization. Moving contact-point velocities include
body rotation. Dynamic bodies and moving half-spaces remain unsupported.

Initial midsurface intersections deeper than the precision length tolerance
(10 micrometres for f32, 1 nanometre for f64) return
`IntegrationError::InitialRigidIntersection`. A final separation below 90% of the
half-thickness target returns `ClothError::UnresolvedSurfaceContact`. Contact/work
overflow and infeasible fixed targets also fail atomically. The adapter has small
primitive and moving-support regression tests; the complete fold and CPU frame
budget remain unqualified.

## Continuous rigid-surface collision

Enable external motion checks independently of cloth self-collision:

```rust
cloth.set_contact_settings(Some(ClothContactSettings {
    rigid_surface_collision: true,
    continuous_rigid_collision: true,
    self_collision: false,
    ..Default::default()
}))?;
```

Fixed half-spaces and fixed/kinematic spheres, boxes and capsules participate in
prediction, stabilization, elastic/target/contact correction and the final sweep.
The swept minimum is 90% of `thickness / 2`, with additional numerical clearance;
contact projection still targets the full half-thickness. Swept contact anchors
are transported to the ending rigid pose before solving the complete inertial
prediction. The application still advances both worlds by exactly the same `h`.

The declared continuous trajectory uses linear body center-of-mass translation
and constant angular interpolation between the actual endpoint orientations;
collider offsets therefore follow arcs. Rapier 0.34's serial velocity solver uses
normalized linear quaternion increments, so its ending angle can differ from
`angular_velocity * h`. The bridge validates that difference against the possible
solver increment angles, including final velocity damping. Velocity-based angular
commands reaching half a turn per external substep are rejected: endpoint
quaternions cannot identify their path unambiguously. Reduce the application step
and restore both worlds before retrying. The existing, normally much tighter
kinematic motion budget also applies. These checks certify the declared path,
not an arbitrary trajectory sharing the same endpoint poses.

Keep collider geometry and its body-local pose unchanged between snapshot and
solve. A changed shape, a moved fixed collider or inconsistent kinematic motion
returns an error. Dynamic bodies and moving half-spaces remain unsupported;
exclude unrelated unsupported colliders with the query filter.

Initial swept gaps must exceed the minimum plus numerical clearance. Infeasible
commands, unresolvable separation and exhausted work limits fail without committing
cloth positions, velocities, contact history or attachment events. The primitive
checks use at most 256 advancement iterations, charged distance queries and the
shared collision budget. Small analytical, rotating-support, zero-friction and
rollback tests exercise this experimental mode; they do not qualify a complete
fold or a CPU frame budget.

Custom core sources opt in through `ContactSource::continuous_motion` and implement
`motion_fraction(ContactMotion, CollisionWork)`. Return a certified fraction in
`[0, 1]`, charge work, and return an error if certification fails. Stabilization
holds obstacles at their previous pose; iteration corrections hold their current
pose. Prediction/final checks span the physical trajectory. The default methods
preserve existing sources that only supply contact queries.

## Continuous self-collision

Set `continuous_self_collision: true` together with `self_collision: true` to
enable experimental continuous checks. The core bounds linear motion during
prediction, stabilization, elastic/target projection and both contact projection
paths. Internal trial corrections form batches; each accepted batch and the final
substep's linear endpoint sweep are checked. No additional physical time steps are
hidden inside this procedure.

Sweeps retain at least 90% of physical thickness, while contact constraints target
the full thickness. Thus a 1 mm cloth uses a 0.9 mm minimum swept separation.
Initial geometry must have a resolvable gap above the swept minimum wherever
advancement is needed. Conservative advancement uses a 10% clearance reserve,
a 256-distance-evaluation limit per query and the configured cumulative CCD budget.
Numerical clearance and convergence failures return `UnresolvedContinuousCollision`;
budget exhaustion returns `CollisionBudgetExceeded`. Neither commits the cloth.

Prediction uses swept witnesses to solve contacts against the full inertial
prediction, preserving tangential motion at zero friction and normal support for
kinetic friction. Elastic/contact trial corrections can be shortened together with
their multiplier increments. Each solver iteration completes elastic, target and
contact projections before checking their combined displacement. If that trial
must be shortened, contacts are refreshed at the accepted pose and their thickness
correction is checked separately. This lets a supported patch recover its edge
lengths without accepting a penetrated intermediate elastic pose. Hard target
commands are still required to be reached within the precision's length tolerance;
infeasible commands fail atomically.
`surface_collision.limited_advances` counts motion checks requesting a reduction.

This option covers cloth self-contact. The Rapier adapter uses particle contacts
unless `rigid_surface_collision` separately enables discrete triangle contacts.
Enable `continuous_rigid_collision` as described above for bounded external checks.
Static friction, the complete folding task and its CPU budget are not yet qualified.

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
