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
shoulder line and a neck opening (deeper on the front) that is either a
rectangular notch (`NeckShape::Notch`, the default) or a half ellipse
(`NeckShape::Round`). With the notch every triangle is a right isosceles
triangle with legs of one cell. With the round neck the cells whose centre
falls inside the ellipse are removed and the vertices left inside the curve are
moved out onto it; a move that would leave a triangle thinner than three tenths
of a cell is undone, so the mesh keeps well-conditioned rest shapes either way.

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
`TShirtPattern::seam_stiffness` multiplies the bending stiffness of the hinges
across seam edges (`ClothMesh::scale_hinge_stiffness`); a stitched seam with
its allowance is a few times stiffer than the fabric, and 1 (the default) keeps
the seams as soft as the panels.

`TShirtPattern::seams` chooses how the panels are joined. `SeamJoin::Shared`
(the default) shares the seam vertices as described above. `SeamJoin::Stitched`
keeps each panel's own seam vertices and threads them with stitches
(`ClothMesh::stitches`, `Stitch { vertices, rest_length }`) of length
`stitch_length`: two open sheets that hinge freely at the seam, the model for
panels meshed separately. The XPBD solver keeps a stitch at its rest length
(`ClothMaterial::stitch_compliance`), the implicit solver holds it with a
spring (`ShellMaterial::stitch_stiffness`, 500 N/m by default). Stitched
vertices stay distinct for collision, so place the layers `stitch_length`
apart with `stitch_length` at least the contact thickness; the folding task
places them thickness plus band apart, which is its stitch length.

`TShirtPattern::sleeve_stiffness` scales the membrane stiffness of the sleeve
triangles (`ClothMesh::scale_triangle_stiffness`), a per-region material for
the implicit solver; `Garment::triangle_regions` tells which triangles belong
to which region for other schemes. XPBD has no triangle membrane; its analogue
is `ClothMesh::scale_edge_compliance`, and
`scale_edge_compliance_by_material_axes` derives the per-edge scale from the
material axes (a compliance factor along the axis and one across it, blended
by the squared cosine), which is how XPBD gets its warp/weft difference.

`Garment` keeps the mesh (`mesh()`, `into_mesh()`) and the pattern metadata:
`vertices()` gives each vertex's layer (`Front`, `Back` or `Seam`), region
(`Body`, `LeftSleeve`, `RightSleeve`) and grid cell; `vertex(layer, column, row)`
looks a corner up; `landmark(Landmark)` returns named points (hem corners and
centre, cuff top, bottom and centre, shoulders, underarms, front and back neck,
chest); `seam_vertices()` lists the shared vertices.

The mesh carries a material axis per triangle (`ClothMesh::material_axes`):
the warp runs from the hem to the shoulders on both panels. With
`ShellMaterial::warp_stiffness` and `weft_stiffness` set, the implicit solver
adds a stretch energy along and across that axis, so the garment is stiffer
along its threads than on the bias, as woven cloth is; the defaults of zero
keep the isotropic shell.

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

The implicit solver takes vertex attachments (`RapierClothWorld::attach`,
`grasp_patch`, or `Cloth::pin`); an attachment with positive compliance holds its
vertices with springs of stiffness `1 / compliance` instead of fixing them.
Weighted surface grasps remain XPBD-only. `Garment::patch(landmark,
radius)` returns every vertex of every layer within `radius` of a landmark's rest
position: a pinch through the cuff or the hem. `Garment::top_patch` returns the
front-panel and seam vertices only, as a gripper closing on the top layer of a
garment lying front up. Use a radius of at least one and a half cells so a coarse
mesh still holds the landmark's neighbours.

To hand a grasp from one gripper to another without letting go,
`RapierClothWorld::transfer_attachment` re-anchors an attachment to another
kinematic body at the vertices' current positions (`docs/grasping.md`).

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
4. pinch both hem corners with compliant grasps (springs of 50 N/m, so the
   flap sagging between the two grippers extends the springs instead of
   stretching the hem) and turn the lower half over the shoulder line in 12 s;
   release;
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

`--usd stage.usda` also writes a USD ASCII stage: the garment as a `Mesh` with
time-sampled `points`, the table and the grippers as `Cube` prims under
time-sampled `Xform`s, one time code per accepted step at `1 / h` codes per
second, Y up, metres. Any USD viewer with time samples plays it (checked with
OpenUSD: the mesh, its samples and the gripper transforms resolve), and it is
the interim path into USD-based hosts such as botrail. botrail's own adapter
crate, `botrail-cloth`, wraps a garment, a table and kinematic gripper bodies
as a `ClothCell` stepped next to a bake, converts between botrail's Z-up frame
and the simulation's Y-up frame at its boundary, and records a per-sample
vertex track that botrail's timeline carries and its studio draws; driving the
grippers from the robot's own tracks is the next step there.

Flags: `--layers single|sewn`, `--spacing m`, `--neck notch|round`,
`--seam-stiffness k`, `--warp Pa`, `--weft Pa`, `--workers 1|4`, `--cap-policy
strict|approximate`, `--h s`, `--max-iterations n`, `--pinch top|all`,
`--seams shared|stitched`, `--stitch-length m`, `--sleeve-stiffness k`,
`--band m`, `--sleeve-fold s`, `--hem-fold s`, `--sleeve-dwell s`, `--hem-dwell s`,
`--hem-release-ramp n`, `--hem-compliance m/N`, `--hem-inset m`,
`--hem-arc-height k` (1 is a semicircle; less drags the flap over the body),
`--release-height m`, `--friction mu`, `--patch-radius m`, `--steps n`
(partial run), `--output` and `--summary` paths.

Results with the defaults on four workers: the sewn 2 cm shirt (2202
vertices, 4290 triangles, up to 46 k contacts) completes its 19 s script in
about six minutes of solve time with 2.6% extension, a footprint ratio of 0.28
and the hem 3 cm past the shoulder line; the single-panel 2 cm shirt completes
in under a minute with 2.1% extension. The coarse 5 cm variants take seconds.
The stitched 2 cm shirt (`--seams stitched`, stitches thickness plus band long)
completes as well, in about two minutes with 3% extension, and keeps doing so
with stitch springs of 100 or 2000 N/m instead of 500, stitches two bands
longer, or a 0.05 s step; the hem then lands 3 cm short of the shoulder line
instead of past it, because the stitched flap hinges more freely.

The script is sensitive to three settings, all found by running the full-size
shirts:

- Pinching only the top layer (`--pinch top`) leaves the back of the sleeve
  hanging from its seams; it whips during the turn and the sewn task fails with
  a factorization error around the release, so the default pinches all layers.
- With hard pins the hem turn had to be slow for the two-layer flap: at 6 s
  (0.37 m/s peak) the full-size sewn shirt jammed near the vertical (one gap
  collapsed to 1e-10 m inside a Newton step and the factorization broke), and
  friction, a 2 mm band, a 0.05 s step or a 512-iteration cap did not change
  that. The pins were the cause: they hold both layers of the flap rigidly
  while the turn tensions them against each other. With compliant hem grasps a
  4 s turn (0.55 m/s peak) completes; a 6 s turn fails on the release step,
  when the flap drops onto the body, whether the gripper first holds still for
  0.5–1 s or releases from 2 cm instead of 5 cm. A soft release fixes that:
  `--hem-release-ramp 3` re-anchors the hem grasp with ten times the compliance
  on each of the last three steps, so the flap descends onto the body before
  it is let go, and the 6 s turn then completes and settles (with
  `--final-settle 3`). A host can soften a grasp the same way without
  re-anchoring through `RapierClothWorld::set_attachment_compliance`. 12 s
  without a ramp remains the default. The
  solver now also keeps its factorization under such jams (a line-search gap
  floor and adaptive barrier stiffness), turning them into a clean
  iteration-budget failure.
- The hem grasps must give: with hard pins the two corners hold the flap at
  its full width while it sags between them, and the hem stretches (20% on the
  coarse single sheet) unless the grippers are scripted to move towards each
  other by an amount that depends on the mesh and the layers. Compliant grasps
  (`hem_compliance`, default 0.02 m/N) remove that dependence for the 2 cm
  shirts and the coarse sewn shirt. The window is not wide: at 0.01 m/N the
  full-size sewn shirt failed its final self sweep while landing, at 0.002 the
  grasp behaves like a hard pin again, and the coarse single sheet lands
  unevenly and creeps at 0.015–0.02 but settles at 0.005–0.01 (its test uses
  0.01). `--hem-inset` remains available for hard grasps.

With the towel's 0.318 mm band the hem fold of the sewn shirt exhausts the
Newton budget; the 1 mm default band avoids that.

## Limits

- One base material per cloth, scaled per triangle (implicit) or per edge
  (XPBD), with warp and weft stiffness along per-triangle axes; seams are
  shared vertices with a bending stiffness scale, or stitches with a rest
  length. No seam allowance geometry, and stitches are springs, not threads
  that slide.
- The silhouette is grid aligned; the neck is a notch or a half ellipse, and
  set-in sleeves are not modelled.
- Cloth-to-cloth collision between separate cloths is not implemented, so a
  garment must be one mesh; several garments in one world do not collide with
  each other.
- Grasps are ideal vertex attachments; there is no physical pinching or force
  feedback (see the implicit solver's limits).
- Cost grows with the stacked area: the sewn 2 cm shirt (2202 vertices) runs
  1–5 s per 0.1 s step on four workers, about 21× slower than real time;
  it is meant for baked timelines. The single-panel 2 cm shirt runs close to
  real time. See [performance](performance.md) for measurement practice.
