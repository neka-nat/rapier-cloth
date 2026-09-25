# Surface selection and grasping

Use surface queries to select material points and ideal attachments to command
their motion. This API does not infer fingertip forces or simulate a frictional
pinch. See [integration](integration.md) for physical contacts and step ordering.

## Select the exposed layer

```rust
use rapier_cloth::{SurfaceRay, SurfaceQueryLimits, Vec3};

let limits = SurfaceQueryLimits::default();
let hit = world.raycast_cloth(SurfaceRay {
    origin: Vec3::new(0.04, 0.8, 0.03),
    direction: -Vec3::Y,
    max_distance: 1.0,
}, limits)?;
```

`raycast_cloth` returns the first cloth surface across the world. Queries are
double-sided, use the current cloth pose and include triangle interiors, edges
and vertices. An occupied or pinned top layer still occludes layers underneath;
creating a conflicting grasp returns an error. Rigid obstacles are outside this
cloth query: compare against a Rapier query with the appropriate gripper filter
when the environment must also occlude the selection.

`sweep_cloth_sphere(ray, radius, limits)` provides a finite-radius approach against
static triangle faces, edges and vertices. Initial overlap returns zero travel.
Radius zero has ray semantics; a coplanar ray has no unique hit. This selection
query does not predict future deforming cloth or replace physical CCD.
`closest_cloth_point(position, max_distance, limits)` finds the nearest point
irrespective of visibility, so it can select a covered layer.

The core equivalents are `SurfaceView::{raycast, sweep_sphere, closest_point}`.
Their `SurfaceHit` contains a local `SurfacePoint`, position, oriented triangle
normal and distance. `SurfacePoint::new(triangle, barycentric)` validates and
normalizes the three nonnegative weights. The integration result wraps it in a
`ClothSurfacePoint` with a generational cloth handle. Use
`world.surface_point_position(point)` to evaluate the point after deformation.
Long-distance comparisons retain f64 ordering even in the f32 build; the public
`distance` field uses the selected precision.

The default query limit is 65,536 triangle tests across all cloths. Queries scan
the supplied geometry; this is intended for bounded interaction queries, not
high-throughput rendering. Invalid/degenerate geometry and exceeded limits return
errors, not a partial selection that might expose an occluded layer. Ray edge
tests use a shear projection; see [PBRT's triangle intersection discussion](https://pbr-book.org/4ed/Shapes/Triangle_Meshes).

## Grasp a point or a patch

```rust
use rapier_cloth::GraspOptions;

if let Some(hit) = hit {
    let grasp = world.grasp_surface(
        hit.point, GraspOptions::new(gripper), &bodies, &colliders,
    )?;
    // Set bounded kinematic targets and step both worlds as usual.
    world.release(grasp)?;
}
```

`grasp_surface` captures one barycentric point's current body-local offset.
Individual support vertices keep their physical inverse masses. Their weighted
position follows the anchor while unconstrained rotation/deformation remains
possible. Positive `GraspOptions::compliance` creates a soft XPBD target; zero
requires the final material-point error to meet the precision's length tolerance.
Corrections and accumulated multipliers pass through the existing CCD trial
boundaries. Infeasible or unconverged hard commands fail atomically. The core
entry point is `Solver::step_with_surface_targets`, taking `SurfaceTarget` values.
Hard-particle rest-path acceleration remains an internal elastic correction;
surface targets project afterward and do not become hidden pinned vertices.

For a finite vertex patch, use `world.select_visible_grasp_patch(ray, radius,
limits)`, then `world.grasp_patch(&patch, options, &bodies, &colliders)` for a
returned patch. The selector starts at the first cloth hit and checks every
selected vertex's visibility from the ray origin. If a patch wraps onto the
hidden back of a fold, the entire selection fails; it does not attach a partial
patch or choose an underlying layer. As with the initial ray, rigid-obstacle
occlusion requires the host's Rapier query.

Patch membership follows shortest rest-edge paths starting from the hit triangle.
It cannot jump between disconnected layers just because they are close in world
space. This is an edge path metric, not an exact continuous surface geodesic.
Each vertex retains its own body-local offset; the patch is not collapsed to one
point. `world.select_grasp_patch(hit.point, radius, limits)` exposes the
material-only selection when the application handles visibility itself.

`limits.patch_vertices` defaults to 256. A radius too small to include a vertex
returns an empty-patch error; use a point grasp or choose a suitable radius.
Exceeding the vertex cap fails instead of truncating. Visible-patch selection
charges the cumulative triangle tests: one initial scan and one scan for every
selected vertex, across all cloths. It fails if this exceeds `limits.triangles`.
Re-query after cloth motion, and use a radius appropriate for the exposed
material neighborhood. Selection and attachment creation reject conflicts
instead of silently removing occupied vertices.

For explicit anchors, use `attach_surface(SurfaceAttachmentDesc, ...)` with
`SurfaceAttachmentPoint` values. Existing `AttachmentDesc`, `attach` and
`attachments()` remain the vertex API. `surface_attachment(handle)` and
`surface_attachments()` access weighted attachments; both kinds share
`AttachmentHandle`, `release` and release/removal events.

## Conflicts, filtering and lifetime

- Nonzero support vertices cannot overlap another vertex/surface target or pin.
  Zero-weight vertices do not create a conflict. This explicit policy rejects
  redundant or competing weighted constraints instead of choosing an order.
- Excluded colliders must belong to the grasp's body. Weighted exclusions cover
  only their nonzero support vertices against those colliders, using the existing
  feature eligibility rule. Self-contact and the other gripper stay enabled.
- Creation evaluates current cloth/body poses. Re-query visibility if selection
  and creation occur at different times. Fixed topology keeps material indices
  stable; removed/foreign cloth handles fail validation.
- Release preserves velocity and clears affected friction history. Removing a
  cloth, removing/disabling its gripper, or restoring a checkpoint applies to
  both attachment kinds. All-cloth failure preserves grasps and events.
- Allocation generations are shared across checkpoint clones and never rewound.
  A handle created in discarded simulation time cannot alias a later allocation.
  Physical state and existing handles restore normally; newly allocated handle
  numbers after a replay need not match the discarded timeline.

Run the [surface_grasp example](../examples/surface_grasp.rs) for a complete query,
weighted lift and release. Full towel-folding quality and CPU performance remain
unqualified; these APIs provide selection and commanded grasp constraints.
