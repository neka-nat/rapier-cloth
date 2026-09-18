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

## CPU execution

Serial execution is the default. To use the calling thread and up to three
additional native threads for material and sparse matrix assembly, select
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
assembly preserves the serial contribution and summation order; this does not
extend determinism guarantees across CPUs, compilers or precisions.

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
The default barrier stiffness is 30 N/m per contact. Initial geometry must have
positive clearance; this solver does not recover initially overlapping layers.
Enable continuous checks for each active collision mode. Existing cumulative
candidate, retained-contact and CCD limits apply to the whole physical step.

Friction uses `kinetic_friction` and a smooth transition velocity, default
0.001 m/s. It permits slow creep under tangential load and does not implement
exact static sticking or use `static_friction`. Hard particle pins and hard
vertex attachments are supported; compliant targets and weighted surface
attachments currently return an error. Coupling remains one-way.

Sparse Newton solves use optional faer and nalgebra dependencies. Iteration and
line-search work are bounded. Numerical exhaustion returns
`ClothError::ImplicitSolverFailed`; collision budgets retain their existing error
types. A failed solve leaves the cloth's physical state unchanged. Application
commands and the separate Rapier state still follow the
[recovery contract](integration.md#failures-and-recovery).

## Run the folding example

```bash
cargo run --locked --release --no-default-features --features f64,implicit --example fold_towel_implicit -- --dt 0.1
cargo run --locked --release --no-default-features --features f64,implicit --example fold_towel_implicit -- --dt 0.04 --output fold-25hz.jsonl
cargo run --locked --release --no-default-features --features f64,implicit --example fold_towel_implicit -- --dt 0.1 --workers 4 --output fold-parallel.jsonl
```

The [example](../examples/fold_towel_implicit.rs) uses one 0.5 m square towel with
32×32 vertices, a fixed floor, four simulated seconds of prescribed folding and
four seconds with all grasps released. The two supported physical step sizes are
0.04 and 0.1 s, with exactly one cloth solve per step. All 53 selected vertices
are released together in the nominal case. The fixture uses areal density 0.1503 kg/m², no extra
velocity damping, friction 0.5 and 10 million candidate/CCD queries per step.
Its [command data and provenance](../examples/assets/README.md) are included.

The floor represents the top of a shell of the same thickness. Its Rapier
halfspace is at y=0.159 mm; cloth contributes another half thickness, so the
required cloth midsurface height is 0.318 mm. All accepted vertices are checked
against the original 2 m floor footprint.

An optional output path must be new and its parent directory must already exist.
JSONL records contain configuration, every accepted state, step timings and a
terminal completion or failure record. Convert this diagnostic format for
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
for results, including a failed 5 mm lift perturbation. The recording converter
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

The bundled f64 recording shows the complete 8 s task at h=0.1 s. Pink points
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

Local Linux validation of this fixture on an Intel Core i9-13900H completed both
step sizes. Earlier measurements after sparse-structure reuse took approximately 18 s for
8 simulated seconds, with
about 1.12% maximum edge extension. Independent checks found no intersections at
the 81 saved states or the 240 quarter-step samples. Sampling does not prove
continuous nonintersection; runtime CCD checks provide the solver's motion guard.
The last half-second had maximum RMS speed about 0.069 mm/s and maximum drift
about 0.094 mm. See the [timing and memory comparison](performance.md#implicit-towel-folding).
Later assembly optimization preserves these states; variable host load prevents
claiming a further serial whole-task speedup from those repetitions.
The optional four-worker public example was subsequently verified at both step
sizes with identical physical states and work counts. At h=0.1, a local paired
comparison reduces mean cost from 18.05 to 14.81 s, with 11.26 MiB additional
peak RSS. At h=0.04, overlapping timings do not establish a useful speedup and
additional peak RSS reaches 29.61 MiB. See the
[four-worker measurements](performance.md#optional-four-worker-execution).
This is task-specific evidence, and wall-clock real time remains unmet.
