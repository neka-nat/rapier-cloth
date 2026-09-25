# Experimental implicit shell solver

The optional `implicit` feature adds a global backward-Euler position solve for
folding with finite-thickness contact. It uses a Neo-Hookean membrane, dihedral
bending, a positive-gap contact barrier and lagged, smoothed Coulomb friction.
The default solver remains XPBD. Enable the experimental solver per cloth:

```toml
[dependencies]
rapier-cloth = { path = "../rapier-cloth", default-features = false, features = ["f64", "implicit"] }
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

The implicit solver defaults to `ImplicitCapPolicy::Strict`: an unconverged
iteration cap rejects the step. To explicitly accept a bounded approximation:

```rust
use rapier_cloth::{ImplicitCapPolicy, ImplicitSettings};

cloth.set_implicit_solver(Some(ImplicitSettings {
    cap_policy: ImplicitCapPolicy::ApproximateWithFinalValidation,
    ..Default::default()
}))?;
```

This policy requires `max_iterations = 80` and `velocity_tolerance = 0.001` m/s.
It returns the last accepted Newton iterate only after all 80 updates were
accepted and the final physical sweep, contact/budget, finite-state, hard-target
and less-than-3% edge-extension checks pass. Line-search, factorization, contact,
CCD, worker and budget failures still reject the step. It adds no hidden
physical substeps and does not enlarge the work budgets. Checkpoints retain the
policy. Acceptance of a configuration is separate from task qualification: the
current approximation evidence covers the documented f64, 32×32 towel at h=0.1 s.

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

Trial contact evaluations store only scalar energy and its gradient in their
internal differentiation values. Newton assemblies retain full second-order
curvature. This reduces intermediate storage without changing contact formulas,
solver tolerances, result ordering or the execution settings below.

Serial execution is the default. To use the calling thread and up to three
additional native threads for material, parent-contact and sparse matrix assembly, select
`Parallel4` explicitly:

```rust
use rapier_cloth::{ImplicitExecution, ImplicitSettings};

cloth.set_implicit_solver(Some(ImplicitSettings {
    execution: ImplicitExecution::Parallel4,
    ..Default::default()
}))?;
```

Execution settings are per cloth and preserved by checkpoints. The application
controls scheduling across cloths; the library does not install a global pool
or set CPU affinity. Budget these workers alongside the robot simulator's own
threads. Small meshes and nonparallel phases run on the caller. Parallel
assembly preserves the serial contribution and summation order. Parent contacts
are evaluated in parallel only for batches of at least 1,024 parent pairs; their
energy, gradient, curvature and numerical errors are consumed in original contact
order. Temporary per-contact results are discarded after each assembly. Material,
contact and sparse workers run in separate scopes within the same four-thread
budget; this does not extend determinism guarantees across CPUs, compilers or precisions.

The target must support native thread creation. A creation failure returns
`ClothError::ImplicitWorkerSpawnFailed`, joins workers already started, and leaves
that cloth's physical state unchanged. Retry after resources become available,
or select `Serial`. A world containing multiple cloths still follows the
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
a rest-geometry mollifier consistently in energy and derivatives. This differs
from the XPBD contact feature reduction. Scalar energy accumulation is compensated,
and the membrane energy uses a stable near-rest expression. Initial geometry must have
positive clearance; this solver does not recover initially overlapping layers.
Enable continuous checks for each active collision mode. Existing cumulative
candidate, retained-contact and CCD limits apply to the whole physical step.

Friction uses `kinetic_friction` and a smooth transition velocity, default
0.001 m/s. It permits slow creep under tangential load and does not implement
exact static sticking or use `static_friction`. Hard particle pins and hard
vertex attachments are supported; compliant targets and weighted surface
attachments currently return an error. Coupling remains one-way.

Sparse Newton solves use optional faer and nalgebra dependencies. Iteration and
line-search work are bounded. In strict mode, numerical exhaustion returns
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
for results. The 5 mm lift reaches the iteration cap in strict mode; explicit
approximate mode completes the seven screened conditions. The recording converter
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

The current qualification concerns this flat towel and fixed floor. Moving
obstacles, garment meshes, weighted grasps, multiple garments, force feedback
and arbitrary materials require further validation. The default live drape/wind
scenes and older `fold_towel` example continue to use XPBD and their own fixtures.
The optional `implicit_towel` live scene uses the shared Rapier pose adapter.
Measure complete-task CPU cost separately from numerical success at a large
physical step; a 0.1 s step does not by itself establish wall-clock real time.

The current public implementation replays the seven h=0.1 s conditions with
explicit approximation: 560 accepted steps, 559 converged and one approximate
return (`lift_5mm`, step 24). Position, velocity, residuals and work counts match
the qualified solver on this Linux host. Independent endpoint checks cover all
567 states including initial states; continuous safety still depends on runtime
CCD. Serial and four-worker saved-input controls match on the same build/host.
These results do not qualify arbitrary manual commands or other cloths.

Task completion does not imply reference-solution accuracy. For the lift case,
the final RMS position difference from a tighter, converged reference trajectory
was about 17.52 mm, despite passing the fold and settling checks. The capped
step remains explicitly unconverged.

The earlier implementation completed both fixture step sizes and supported the
historical [timing comparisons](performance.md#implicit-towel-folding), including
about 14.81 s for an 8 s h=0.1 task with four workers. Those timings do not describe
the current parent-primitive contact model. The [current matched CPU and browser comparison](performance.md#current-implicit-contact-performance)
measures first-order contact storage specialization on top of parallel contact
evaluation, with exact states/status/work on the qualified paths. Whole-task
gains are smaller than the first-order kernel improvement and vary with host
load. Wall-clock real time remains unmet; budget CPU time, worker count and
process memory alongside the rest of the simulator.
