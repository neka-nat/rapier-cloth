# Experimental implicit shell solver

The optional `implicit` feature adds a global backward-Euler position solve for
folding with finite-thickness contact. It uses a Neo-Hookean membrane, dihedral
bending, a positive-gap contact barrier and lagged, smoothed Coulomb friction.
The default solver remains XPBD. Enable the experimental solver per cloth:

```toml
[dependencies]
rapier-cloth = { version = "0.3", default-features = false, features = ["f64", "implicit"] }
```

```rust
use rapier_cloth::{ClothContactSettings, ImplicitSettings};

cloth.set_contact_settings(Some(ClothContactSettings {
    thickness: 0.000318,
    activation_margin: 0.000318,
    continuous_self_collision: true,
    rigid_surface_collision: true,
    continuous_rigid_collision: true,
    ..Default::default()
}))?;
cloth.set_implicit_solver(Some(ImplicitSettings::default()))?;
```

The initial supported runtime requires **f64**. Enabling this solver in an f32
build returns an error. `set_implicit_solver(None)` restores XPBD. Checkpoints
preserve the selected solver and its settings. The application still advances
Rapier and cloth with the same physical time step; Newton and line-search
iterations do not insert physical substeps. Set `SolverSettings::max_substep`
to permit the requested step size.

## Iteration limits and accepted outcomes

Newton starts from `ImplicitSettings::seed`: with the default
`ImplicitSeed::Velocity`, free vertices begin at their previous position plus
`velocity × h`, shortened until the continuous checks pass and every contact
keeps at least half the barrier band as clearance (falling back to the previous
positions otherwise); `ImplicitSeed::Previous` always starts at the previous
positions. Both seeds converge to the same objective; the velocity seed saves
about 10–20% of the iterations on the towel task and lets a residual velocity
below the tolerance persist, where the previous-position seed would zero it.

A kinematic obstacle that reaches resting cloth within the step (a gripper
pad pressing down, a pusher sliding in) cannot be certified against cloth that
stays where it is. The step then moves the cloth it meets ahead of the obstacle,
using the witnesses of the obstacle's physical sweep
(`ContactSource::swept_witnesses`), until the joint motion of cloth and obstacle
is certified. Pinned cloth that reaches resting free cloth within the step (a
pinch through the layers of a garment lifting them) is handled the same way: the
self-contacts that limited the sweep serve as witnesses, and the free cloth they
met is pushed ahead of the pinned vertices. Inside a step, the line search does
not let a trial drive the smallest contact gap below one ten-thousandth of the
band while it is shrinking, and a contact that stays jammed below one thousandth
of the band doubles the barrier stiffness for the rest of the step (up to ten
thousand times); both keep the barrier Hessian within the range the sparse
factorization can handle. A sweep certifies the path down to a
fraction of the contact separation, so contacts found at the certified positions
that still sit inside the separation are pushed out the same way before Newton
starts. A box pushing a
sheet along a table at 0.1 m/s (1 cm per 0.1 s step) runs this way, whether the
sheet slides or buckles. Sources without swept witnesses fail such a step with
`ImplicitSolverFailed { phase: "grasp initialization sweep" }`, as before.

The barrier band is `ClothContactSettings::activation_margin`. With coarse cloth
sliding around the edges of boxes or convex hulls within one 0.1 s step, a band of
about 1 mm lets the barrier steer the cloth before continuous collision cuts the
Newton steps short; with the 0.1 mm default such a step can exhaust the Newton
budget. Smaller steps also work. The towel example, which only meets a flat
floor, uses a band equal to its 0.318 mm thickness.

A step converges when `convergence_window` consecutive Newton directions (default
3) have an RMS displacement per time step at or below `velocity_tolerance`
(default 0.001 m/s), or when the first direction of the step already does: a
state at rest, or a consistent extrapolated state, then stays where it is. The step is limited to `max_iterations`
Newton iterations (default 128). A smaller window stops sooner but can accept the
low point of an oscillating iteration far from the solution; on the 5 mm lift
condition, a window of 1 left one step about 10 mm from its converged state.

The implicit solver defaults to `ImplicitCapPolicy::Strict`: an unconverged
iteration cap rejects the step. To explicitly accept a bounded approximation:

```rust
use rapier_cloth::{ImplicitCapPolicy, ImplicitSettings};

cloth.set_implicit_solver(Some(ImplicitSettings {
    cap_policy: ImplicitCapPolicy::ApproximateWithFinalValidation,
    ..Default::default()
}))?;
```

This policy requires the default iteration cap, tolerance and convergence window.
It returns the last accepted Newton iterate only after every update was
accepted and the final self sweep, contact/budget, finite-state, hard-target
and less-than-3% edge-extension checks pass. Line-search, factorization, contact,
CCD, worker and budget failures still reject the step. It adds no hidden
physical substeps and does not enlarge the work budgets. Checkpoints retain the
policy. Acceptance of a configuration is separate from task qualification. With
the default settings, all seven documented towel conditions converge in strict
mode, so none of them currently exercises this policy.

For every accepted implicit step, `StepReport::implicit` contains an
`ImplicitOutcome` with typed `termination`, `converged`, objective `energy` (J),
and free-particle residual `force_rms` / `force_max` (N). The RMS is over particle
force magnitudes, with fixed particles excluded. `Converged` means the configured
Newton-displacement criterion was met; it does not bound shape or trajectory
error. `ApproximateIterationCap` always has `converged = false`, including after
successful physical validation. A rejected step has no accepted outcome.
`StepReport::iterations` and `surface_collision` contain iteration/work counts.
XPBD reports have no implicit outcome.

## CPU execution

Contact energy and forces are exact. The Newton matrix uses each contact's
Gauss-Newton curvature `m b'' ∇g ∇gᵀ` (barrier second derivative times the gap
gradient outer product), which is positive semidefinite without a per-contact
eigen-decomposition; the omitted curvature terms are small inside the barrier
band. Matrix blocks are summed in assembly order into a pattern that only grows
during a physical step, so its symbolic factorization is usually reused. The
self-contact sweep of each Newton direction records the pairs within contact
range, and line-search trial points evaluate only those pairs.

Serial execution is the default. To use the calling thread and up to three
additional native threads for material and parent-contact evaluation and for
self-contact queries and sweeps, select `Parallel4` explicitly. On the towel
task the serial sparse factorization dominates, so four workers save about
0.5 s of the 8 s solve under light load and nothing on a busy host (see
[performance.md](performance.md#implicit-solver-timing)):

```rust
use rapier_cloth::{ImplicitExecution, ImplicitSettings};

cloth.set_implicit_solver(Some(ImplicitSettings {
    execution: ImplicitExecution::Parallel4,
    ..Default::default()
}))?;
```

Execution settings are per cloth and preserved by checkpoints. Each physical
step creates its own three-thread [rayon](https://crates.io/crates/rayon) pool
at its first parallel phase and joins it when the step ends; the library does
not install a global pool or set CPU affinity. The application controls
scheduling across cloths. Budget these workers alongside the robot simulator's
own threads. Small meshes and nonparallel phases run on the caller. Parallel work
preserves the serial contribution and summation order, so `Serial` and `Parallel4`
give identical results on the same build and host. Parent contacts and friction
contacts are evaluated in parallel only for batches of at least 1,024 and 512
contacts, and self-contact queries only for meshes of at least 1,024 vertices.
Those queries and sweeps traverse the vertex, edge and triangle hierarchies
pairwise from a fixed set of subtree pairs, which the lanes take in turn; contacts,
recorded pairs, work counts and numerical errors return to that fixed order before
they are consumed. Temporary results do
not survive a physical step, except the Newton matrix pattern and its symbolic
analysis, which the next step reuses only when its first matrix has the same
pattern (the analysis it would otherwise recompute), so results are unchanged. The four-thread budget does not extend determinism
guarantees across CPUs, compilers or precisions.

The target must support native thread creation. A pool creation failure returns
`ClothError::ImplicitWorkerSpawnFailed` and leaves that cloth's physical state
unchanged. Retry after resources become available, or select `Serial`. A world
containing multiple cloths still follows the
[world checkpoint and recovery contract](integration.md#failures-and-recovery);
Rapier state and application commands are separate. Unexpected programmer panics
remain panics, as on the serial path. No numerical factor, worker or optimizer
state survives a physical step.

## Material and contact configuration

`ImplicitSettings::material` contains Young's modulus in Pa, Poisson's ratio and
shell thickness in metres. Defaults are 821000 Pa, 0.243 and 0.000318 m. Areal
mass, forces and velocity damping still come from `ClothMaterial`; its XPBD
stretch and bending compliances do not control this solver. Collision thickness
is configured separately in `ClothContactSettings`.

The contact activation margin is the barrier width and must be positive.
The default barrier stiffness is 30 N/m per contact. Implicit self-contact uses
a coherent unsigned-distance potential for each parent vertex-face or edge-edge
pair, retaining parent multiplicity at shared features. Edge-edge contacts use
a rest-geometry mollifier consistently in energy and forces. This differs
from the XPBD contact feature reduction. Scalar energy accumulation is compensated,
and the membrane energy uses a stable near-rest expression. Initial geometry must have
positive clearance; this solver does not recover initially overlapping layers.
Enable continuous checks for each active collision mode. Existing cumulative
candidate, retained-contact and CCD limits apply to the whole physical step.

`ShellMaterial::warp_stiffness` and `weft_stiffness` add a stretch energy
`k / 2 (|F a| - 1)^2` per unit volume along each triangle's material axis
(`ClothMesh::set_material_axes`, projected into the rest plane) and across it,
with a positive semidefinite Hessian; triangles without an axis and the
defaults of zero keep the isotropic Neo-Hookean membrane. Hinges scaled with
`ClothMesh::scale_hinge_stiffness` bend accordingly, triangles scaled with
`ClothMesh::scale_triangle_stiffness` stretch accordingly, and the mesh's
stitches (`ClothMesh::stitches`) are springs of `ShellMaterial::stitch_stiffness`
between their two vertices with a positive semidefinite Hessian.

Friction uses `kinetic_friction` and a smooth transition velocity, default
0.001 m/s. It permits slow creep under tangential load and does not implement
exact static sticking or use `static_friction`. Compliant particle targets
(`Target::compliance` > 0, or an attachment with positive compliance) hold their
particle with a spring of stiffness `1 / compliance` newtons per metre instead of
fixing it; weighted surface targets remain unsupported. Hard particle pins and hard
vertex attachments are supported; compliant targets and weighted surface
attachments currently return an error. Coupling remains one-way.

Sparse Newton solves use the optional faer and nalgebra dependencies, and
`Parallel4` the optional rayon dependency. Iteration and line-search work are bounded. In strict mode, numerical exhaustion returns
`ClothError::ImplicitSolverFailed`; collision budgets retain their existing error
types. A failed solve leaves the cloth's physical state unchanged. Application
commands and the separate Rapier state still follow the
[recovery contract](integration.md#failures-and-recovery).

## Run the folding example

```bash
cargo run --locked --release --no-default-features --features f64,implicit --example fold_towel_implicit -- --dt 0.1
cargo run --locked --release --no-default-features --features f64,implicit --example fold_towel_implicit -- --case lift_5mm --cap-policy approximate --workers 4 --output fold-lift.jsonl
cargo run --locked --release --no-default-features --features f64,implicit --example fold_towel_implicit -- --dt 0.1 --workers 4 --output fold-parallel.jsonl
```

The [example](../examples/fold_towel_implicit.rs) uses one 0.5 m square towel with
32×32 vertices, a fixed floor, four simulated seconds of prescribed folding and
four seconds with all grasps released. The command fixtures provide
0.04 and 0.1 s steps, with exactly one cloth solve per step. The current contact
model and approximate policy were requalified at 0.1 s; historical 0.04 s results
below concern the preceding implementation. All 53 selected vertices
are released together in the nominal case. The fixture uses areal density 0.1503 kg/m², no extra
velocity damping, friction 0.5 and 10 million candidate/CCD queries per step.
Its [command data and provenance](../examples/assets/README.md) are included.

The floor represents the top of a shell of the same thickness. Its Rapier
halfspace is at y=0.159 mm; cloth contributes another half thickness, so the
required cloth midsurface height is 0.318 mm. All accepted vertices are checked
against the original 2 m floor footprint.

An optional output path must be new and its parent directory must already exist.
JSONL records contain configuration, every accepted state, step timings, the
typed `outcome`, cumulative `approximate_steps`, and a terminal completion or
failure record. The completed summary lists approximate step numbers.
`--cap-policy strict|approximate` defaults to `strict`. Convert this diagnostic format for
[viewer playback](#watch-the-motion). `--workers` accepts `1` (default) or `4`;
the configuration record includes the requested worker count, including the
caller. The summary checks full release, maximum
extension below 3%, planar fold error below 3 cm, and a final half-second window
with RMS speed below 1 mm/s, maximum speed below 5 mm/s and drift below 1 mm.
Failure returns a nonzero exit status. These are fixture checks, not universal
accuracy guarantees.

Use `--case NAME` for the bounded variants: `nominal`, `grasp_inset`, `lift_5mm`,
`left_early`, `right_late`, `friction_low`, and `friction_high`. See the
[condition screen and robot adapter](robot-control.md#headless-reproduction-and-evidence)
for results. With the default settings all seven conditions converge in strict mode. The recording converter
below validates the nominal pin-command fixture; changed grasp/motion/release
commands use diagnostic JSONL rather than that converter.

For actual Rapier end-effector bodies and browser pose commands, use the
[robot-control example and live demo](robot-control.md).

## Watch the motion

Start the viewer and open **http://127.0.0.1:5173/?sample=implicit&play=1**:

```bash
npm --prefix demos/viewer ci
npm --prefix demos/viewer run dev
```

The bundled f64 recording is a historical recording from the preceding contact
implementation. It shows the complete 8 s task at h=0.1 s. Pink points
indicate the 53 pinned vertices while folding; all disappear in the first
accepted released state at t=4.1 s. Pause, scrub the timeline, orbit or enable
wireframe to inspect the fold. The inspector shows measured edge extension and
speeds. This is recorded playback; the recording took about 14.80 s to compute
on its host with four workers. Display refresh does not change the physical
step size, and motion is not interpolated between saved states.

To inspect a new simulation, run these commands from the repository root.
Both output paths must be new:

```bash
cargo run --locked --release --no-default-features --features f64,implicit --example fold_towel_implicit -- --dt 0.1 --workers 4 --output fold.jsonl
npm --prefix demos/viewer run convert:implicit -- ../../fold.jsonl ../../fold-view.json.gz
```

Choose **Open recording** in the viewer and select `fold-view.json.gz`.
The converter resolves its arguments from `demos/viewer` (npm's working
directory). It also supports h=0.04 and gzip-compressed JSONL input. It checks the
initial geometry and pin commands against the included fixture, preserves all
accepted positions and timestamps, and supplies topology from that fixture.
For new records it also preserves solver outcomes and approximate counts; the
phase label retains the approximation history.
Incomplete or failed runs remain visibly incomplete or failed. Neither conversion
nor rendering reruns the physics or certifies collision clearance.

## Qualification boundary

The reference is the author implementation accompanying
[Effective cloth folding trajectories in simulation with only two parameters](https://doi.org/10.3389/fnbot.2022.989702),
using [Codimensional IPC](https://ipc-sim.github.io/C-IPC/). This example uses the
same captured towel manipulation, material constants and physical time step.
The barrier, friction smoothing and approximate bending Hessian differ from
C-IPC; this library is not a port of its source or a claim of identical physics.

The two command files retain the author's final-frame convention independently
at each sample rate. Comparing them establishes task behavior at each step size;
it is not a temporal-convergence test with an identical continuous input.

The current qualification concerns this flat towel and fixed floor. The
[T-shirt folding example](garments.md) runs a sewn garment through sleeve and
hem folds with its own gates, but it is not qualified against a reference. Moving
obstacles, weighted grasps, multiple garments, force feedback and arbitrary
materials require further validation. The default live drape/wind
scenes and older `fold_towel` example continue to use XPBD and their own fixtures.
The optional `implicit_towel` live scene uses the shared Rapier pose adapter.
Measure complete-task CPU cost separately from numerical success at a large
physical step; a 0.1 s step does not by itself establish wall-clock real time.

The current implementation completes the seven h=0.1 s conditions in strict
mode: all 560 accepted steps converge, the maximum edge extension is 1.48% and
the planar fold error is 12.5–12.9 mm. At every step, the vertex RMS position
difference from the preceding implementation's trajectories stays within
1.50 mm; for the 5 mm lift, the difference from a fully converged solve of that
implementation stays within 0.81 mm. Repeated runs and `Serial`/`Parallel4` give
identical results on the same build and host. Continuous safety depends on the
runtime CCD, whose certified pairs are unchanged; the independent endpoint
geometry audits of the preceding implementation have not been repeated. These
results do not qualify arbitrary manual commands or other cloths.

Task completion does not imply reference-solution accuracy: `Converged` is a
Newton-displacement criterion, not a bound on the distance to the exact
backward-Euler trajectory.

See [implicit solver timing](performance.md#implicit-solver-timing) for the
current cost of the 8 s task: about 7.2–8.4 s of solve time with four workers
on this laptop under light background load, and more in sustained runs that
throttle. Wall-clock real time is not guaranteed; budget CPU time, worker count
and process memory alongside the rest of the simulator. Older timings in that
guide describe preceding implementations.
