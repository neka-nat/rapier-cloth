# Garments

A garment is one cloth mesh whose panels are sewn together. `rapier-cloth-core`
builds a parametric T-shirt from a flat pattern (`TShirtPattern` → `Garment`),
and the `fold_shirt_implicit` example folds it with two ideal grippers on the
[implicit shell solver](implicit.md). This guide describes the mesh, how to place
and grasp it, the folding task and its limits.

## Pattern and mesh

`TShirtPattern` holds the dimensions in metres: body width and length, sleeve
length and width, neck width and the front and back neck depth, plus the grid
`spacing` and the `layers` variant. `TShirtPattern::default()` is a medium adult
T-shirt on 2 cm cells (0.50 m × 0.70 m body, 0.22 m sleeves, sewn). All lengths
are rounded to whole cells, and `TShirtPattern::cells()` reports the result.

The outline is aligned to the grid: a body rectangle, a sleeve bar along the
shoulder line and a rectangular neck notch (deeper on the front). Every triangle
is a right isosceles triangle with legs of one cell, so the mesh has no slivers
and the membrane rest shapes stay well conditioned.

Pattern frame: `x` runs across the garment (`Side::Left` is `-x`, the wearer's
right), `z` runs from the hem (`-z`) to the shoulder line (`+z`), and the layers
are separated along `y` with the front panel on the `+y` side. The pattern is
centred at the origin.

Two variants come from one pattern:

- `GarmentLayers::Single` is the outline as one open sheet with the front
  neckline: the cheapest garment-shaped cloth, and the shape of the classic
  single-layer shirt folding demonstrations.
- `GarmentLayers::Sewn` has a front and a back panel. Both lie coincident in
  the pattern plane, which is their flat rest geometry. The vertices along a
  seam (the shoulders, the sleeve tops and bottoms, the sides) are shared by
  both panels, and the back panel's triangles are wound the other way, so the
  surface is consistently oriented and every seam edge has exactly two faces.
  The neck, the hem and the two cuffs stay open: the sewn mesh has one
  connected component and four boundary loops.

Panel interiors rest flat. Seam hinges rest fully folded (an angle of ±π),
which is the shape of a garment lying on a table; the bending energy measures
angle differences on the circle, so a seam that opens or flips is safe.

`Garment` keeps the mesh (`mesh()`, `into_mesh()`) and the pattern metadata:
`vertices()` gives each vertex's layer (`Front`, `Back` or `Seam`), region
(`Body`, `LeftSleeve`, `RightSleeve`) and grid cell; `vertex(layer, column, row)`
looks a corner up; `landmark(Landmark)` returns named points (hem corners and
centre, cuff top, bottom and centre, shoulders, underarms, front and back neck,
chest); `seam_vertices()` lists the shared vertices.

## Placement

Rest geometry never separates the layers. Before the first step, call
`Garment::placed_positions(origin, layer_gap)` and apply it with
`Cloth::set_positions`: the front panel is lifted by half the gap, the back
panel lowered by half, and the seams stay midway, all offset by `origin`. A
single panel ignores the gap.

Two stacked layers need at least the contact `thickness` between their
midsurfaces, and they start free of contact force at `thickness +
activation_margin`; use that as the gap. Place the lowest layer's midsurface
half a thickness plus one band above the table top, as the towel example does.
The garment tests (`tests/garment.rs`) rest a single-panel and a sewn shirt on
a table this way: no stretch, no crossings, layers apart.

Stacked layers report many self-contacts (about twenty per vertex inside the
band). Raise `CollisionLimits::retained_contacts` and
`SolverSettings::max_contacts` for full-size garments; the folding example
uses 2^20.

## Grasping a garment

The implicit solver takes hard vertex attachments only (`RapierClothWorld::attach`
with compliance 0, `grasp_patch`, or `Cloth::pin`). `Garment::patch(landmark,
radius)` returns every vertex of every layer within `radius` of a landmark's rest
position: a pinch through the cuff or the hem. `Garment::top_patch` returns the
front-panel and seam vertices only, as a gripper closing on the top layer of a
garment lying front up. Use a radius of at least one and a half cells so a coarse
mesh still holds the landmark's neighbours.

A pinch through layers moves pinned cloth into free cloth that rests on it. The
implicit seed handles this the way it handles an arriving kinematic obstacle:
when the swept path of the pinned vertices is limited by self-collision, the
limiting self-contacts serve as witnesses and the free cloth they met is pushed
ahead of the pinned vertices before the path is certified again
(`docs/implicit.md`). Without this a two-layer pinch failed its initialization
sweep at the first fold step.

## Folding example

```bash
cargo run --locked --release --no-default-features --features f64,implicit \
    --example fold_shirt_implicit -- --layers sewn --spacing 0.02 --workers 4 \
    --output target/fold_shirt.json
```

The task (`examples/support/garment_task.rs`) lays the shirt on a half-space
table with the towel example's shell (0.318 mm thickness, mu 0.5, surface density
0.1503 kg/m², default `ShellMaterial`) and a 1 mm barrier band, then runs a script
with two kinematic grippers that carry no colliders:

1. settle for 0.5 s;
2. pinch both cuffs (a 4 cm patch through all layers) and turn each sleeve over
   the body about the body's side, like a page, in 3 s; release;
3. travel to the hem corners while the sleeves settle for 1.5 s;
4. pinch both hem corners and turn the lower half over the shoulder line in
   12 s, moving the grippers 5 mm towards each other at mid-flight so the
   sagging flap between them is not stretched across; release;
5. settle for 2 s.

A turned patch travels a circular arc about the fold line and rotates with it,
so the grasped fabric stays consistent with a rigidly turning flap. The arc
keeps its radius, which is the fabric length between the fold line and the
patch: it stops short of a half circle so the patch is released
`release_height` (5 cm) above the plane. Adding that height radially instead
stretched the taut two-layer flap against the hard pins by several percent and
broke the factorization of the full-size shirt near the vertical. Every step is
transactional: a rejected step restores both worlds and the attachments and stops
the task. The recording (schema 2, see [recording format](recording-format.md))
holds every accepted step, the two grippers and the table as boxes, and the
landmark indices under `config.landmarks`; open it in the
[replay viewer](examples.md#replay-in-the-browser), whose camera scales its
preset to the recording's extent.

Gates, checked on the completed run: maximum edge extension ≤ 5% (the towel gate is 3%; a pinched two-layer cuff stretches more); settled over
the final 0.5 s; the bounding rectangle in the table plane at most 45% of its
initial area; both cuff centres over the body's original footprint; both hem
corners within 8 cm of the shoulder line; no crossing triangle pairs in an
independent audit; nothing below the table clearance. `tests/garment_fold.rs`
runs the script on coarse (5 cm) single-panel and sewn shirts.

Flags: `--layers single|sewn`, `--spacing m`, `--workers 1|4`, `--cap-policy
strict|approximate`, `--h s`, `--max-iterations n`, `--pinch top|all`,
`--band m`, `--sleeve-fold s`, `--hem-fold s`, `--hem-inset m`,
`--hem-arc-height k` (1 is a semicircle; less drags the flap over the body),
`--release-height m`, `--friction mu`, `--patch-radius m`, `--steps n`
(partial run), `--output` and `--summary` paths.

Results with the defaults on four workers: the sewn 2 cm shirt (2202
vertices, 4290 triangles, up to 46 k contacts) completes its 19 s script in
about 7 minutes of solve time with 3.4% extension, a footprint
ratio of 0.27 and the hem within 1 cm of the shoulder line; the single-panel
2 cm shirt completes in about a minute with 2.7% extension. The coarse 5 cm
variants take seconds.

The script is sensitive to three settings, all found by running the full-size
shirts:

- Pinching only the top layer (`--pinch top`) leaves the back of the sleeve
  hanging from its seams; it whips during the turn and the sewn task fails with
  a factorization error around the release, so the default pinches all layers.
- The hem turn must be slow for the two-layer flap: at 6 s (0.37 m/s peak) the
  full-size sewn shirt fails near the vertical with a contact jam (one gap
  collapses to 1e-10 m inside a Newton step and the factorization breaks);
  at 12 s it completes. Friction, a 2 mm band, a 0.05 s step and a 512-iteration
  cap do not change that. Making the solver tolerate such jams is follow-up work.
- The hem inset must match how much the flap sags between the two grippers:
  the 2 cm shirts want about 5 mm (2 cm buckles the fine single sheet and stalls
  Newton), the coarse single sheet wants 2 cm (5 mm stretches its hem by 20%).
  A compliant or slipping grasp would remove this dependence; the implicit
  solver only has hard pins today.

With the towel's 0.318 mm band the hem fold of the sewn shirt exhausts the
Newton budget; the 1 mm default band avoids that.

## Limits

- One isotropic material per cloth: no warp/weft anisotropy, no seam stiffness
  or seam allowance, no stitch constraints; seams are shared vertices.
- The silhouette is grid aligned with a rectangular neck notch; curved
  necklines and set-in sleeves are not modelled.
- Cloth-to-cloth collision between separate cloths is not implemented, so a
  garment must be one mesh; several garments in one world do not collide with
  each other.
- Grasps are ideal vertex attachments; there is no physical pinching or force
  feedback (see the implicit solver's limits).
- Cost grows with the stacked area: the sewn 2 cm shirt (2202 vertices) runs
  1–5 s per 0.1 s step on four workers, about 21× slower than real time;
  it is meant for baked timelines. The single-panel 2 cm shirt runs close to
  real time. See [performance](performance.md) for measurement practice.
