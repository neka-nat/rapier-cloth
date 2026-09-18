# Measuring performance

Measure a release build on the intended CPU, at the intended mesh resolution,
substep size and iteration count. Record both time and simulation errors. Browser
render fps is not the same as physics keeping pace with wall time.

## Four-substep physics frames

```bash
cargo bench --locked --bench realtime -- --output target/realtime/run-01
cargo bench --locked --bench realtime --no-default-features --features f64 -- --output target/realtime/run-01
```

[benches/realtime.rs](../benches/realtime.rs) measures one 32×32, 1 m cloth with the
library's default material, 1/240 s substeps and eight iterations. After 100 warmup
substeps, it measures 1,000 substeps as 250 actual groups of four. Use a new output
directory for each comparison; the f32 and f64 files have distinct names.

| Fixture | Setup |
|---|---|
| `flat-halfspace` | Horizontal cloth at y=0.005 m, contacting a fixed floor |
| `hanging` | One edge pinned at y=1 m; gravity and no external collider |
| `moving-sphere` | Cloth falling onto a floor and a horizontally moving kinematic sphere |

`physics_frame` includes Rapier stepping, snapshots, cloth staging, queries,
constraints and diagnostics. It excludes rendering, GPU uploads, normal updates,
JSON output and pacing. Frame percentiles are measured over the four-step groups,
not calculated by multiplying a substep percentile by four. `frames_over_budget`
counts measured groups exceeding the 16.67 ms budget for 60 Hz.

Reports include CPU, OS, compiler, commit, dirty-checkout flag, precision, conditions,
p50/p95/maximum timing, maximum errors and the number of frames over budget. Compare
runs with the same conditions and inspect outliers as well as percentiles.

## Folding development trajectory

The folding development benchmark can also exercise the discrete self-contact
path with the frozen dual-gripper trajectory:

```bash
cargo bench --locked --bench folding -- --self-collision --repeats 1 --output target/folding/discrete-01
```

This is a diagnostic. Experimental persistent static/kinetic friction is implemented;
full-task convergence and performance qualification remain open. Replace `--self-collision` with
`--continuous-self-collision` to exercise the experimental self-CCD path. The report
records partial failures, exact collision settings, per-substep work high-water
marks, geometry errors and four-substep timings. `--verify` enables an expensive
independent geometry audit between timed substeps; run that separately from
performance qualification. Add `--rigid-surface-collision` to use discrete triangle
contacts against the supported rigid shapes. It is independent of the self-contact
flags, and reports the actual half-thickness offset. Use
`--continuous-rigid-collision` to enable bounded rigid CCD as well; it implies
rigid-surface contacts and records `rigid_ccd_minimum_separation`. Omit all collision
flags to use particle-contact mode. Historical numerical results also require
their recorded solver revision: current hard-attached inextensible meshes use
the [attachment distance bounds](integration.md#materials).
An exit code of zero means the
diagnostic wrote its report; inspect `failure`, `steps` and geometry/task metrics
before interpreting the result as a completed trajectory.

Each shared folding-task substep checkpoints both worlds before moving grippers
or changing grasps. A failed substep restores Rapier state, cloth/contact history,
attachment handles, queued events and task time to the last accepted state.
After correcting the cause, a retry consumes that same substep. These task
checkpoint costs are included in the folding benchmark's physics-frame time;
the core scratch counter does not include checkpoint or Rapier memory.

The [headless folding example](examples.md#experimental-towel-folding) shares the
benchmark's execution and audit helper. Its `stop_reason` distinguishes completed,
requested partial and solver-error results. Recording callbacks run outside the
physics stopwatch. Their allocations and the interleaved task/oracle work can
still affect the process workload, so recorded example timings are diagnostic.
The source identity is taken from the checkout used to build the helper,
independently of the launch directory. An extracted crate without that checkout
reports unknown/unqualified source metadata; verify its archive provenance
separately instead of presenting an unrelated enclosing checkout as its source.

Use `--fixture-version 2` with surface-rigid contact. Version 2 captures grasp
anchors after the final approach movement and before solving the cloth, avoiding
a downward target jump into the table when grasping. Mesh, material, trajectories,
phase times and acceptance limits are identical to version 1. The default remains
version 1 for reproducing historical particle-contact baselines. Reports identify
the fixture version; compare equivalent versions when attributing solver changes.

```bash
cargo bench --locked --bench folding -- --fixture-version 2 --rigid-surface-collision --continuous-self-collision --verify --repeats 1 --output target/folding/surface-v2-01

# Experimental continuous self and rigid checks; correctness replay.
cargo bench --locked --bench folding -- --fixture-version 2 --continuous-rigid-collision --continuous-self-collision --verify --repeats 1 --output target/folding/continuous-v2-01

# Separate timing run without the interleaved geometric oracle.
cargo bench --locked --bench folding -- --fixture-version 2 --continuous-rigid-collision --continuous-self-collision --repeats 5 --output target/folding/continuous-timing-v2-01
```

Omitting `--verify` removes the per-substep geometric oracle; task metrics still
run outside the stopwatch. Dedicated CPU qualification must account for this
remaining interleaved work and measure complete runs on the target host.

### Check a folding correctness report

From a repository checkout, use Python 3.10 or newer to check an audited report:

```bash
bash scripts/check-folding.sh target/folding/continuous-v2-01/folding-f32.json

# All six variants in both precisions from the same clean source revision.
bash scripts/check-folding.sh --suite target/folding/correctness-*/folding-*.json
```

The checker requires schema 2, fixture 2, both continuous collision modes, and a
complete `--verify` replay. It compares settings and limits with the checked-in
fixture, rejects recorded failures and missing or non-finite measurements, and
checks geometry, strain, tracking, fold shape and settling. A suite must include
the nominal and all five perturbations in f32 and f64. Every supplied repetition
must pass; incomplete reports and dirty-source reports are rejected.

Schema 2 retains individual four-substep times in `physics_samples_ms`; the checker
recomputes frame and phase percentiles from them. `task_audit` records penetration
throughout the task, coverage of both grasps and the entire final five seconds,
and target-free observations after release. `final_targets` includes vertex
attachments, weighted surface attachments, pins, their target points and task
grasp handles. Target counts cover the world inputs used by this task.

A passing checker result establishes the recorded correctness gates only. The
interleaved oracle affects the workload, so its times do not qualify CPU speed.
Dedicated timing repetitions, total application memory, continuous-crossing
regressions and a paced live run require their own evidence. Schema 1 reports lack
the required coverage fields and must be regenerated. The current folding
diagnostic may still stop before completion and be rejected by the checker.

## Resolution scaling

```bash
cargo bench --locked --bench cloth_scaling -- --output target/benchmarks/run-01
cargo bench --locked --bench cloth_scaling --no-default-features --features f64 -- --output target/benchmarks/run-01
```

[benches/cloth_scaling.rs](../benches/cloth_scaling.rs) measures 32×32, 64×64 and
128×128 grids, each recreated as a 1 m square on a fixed half-space. It uses the
default material, no pins, gravity `(0, -9.81, 0)`, 1/240 s steps and eight iterations.
There are 100 warmup and 1,000 measured substeps per resolution. JSON contains the
conditions and diagnostics; CSV provides comparison rows.

| Metric | Scope |
|---|---|
| `core` | Solver time excluding the contact callback |
| `query` | Candidate bounds/enumeration, Parry contact/sweep and contact conversion |
| `total` | Cloth-world entry, kinematic checks, staging, core and queries |
| `scratch_array_bytes` | Retained core array capacities, including contact-state arrays; excludes bridge staging and Rapier memory |
| Error fields | Maximum penetration/strain and maximum per-step p95 strain during measurement |

The scaling benchmark's `total` excludes Rapier's rigid-body step. It is therefore
a different scope from `physics_frame` above. Timings are serial CPU measurements;
larger grids are not automatically suitable for real-time use.

## Implicit towel folding

For the experimental f64 shell solver, measure the complete fold, release and
settle trajectory separately from the XPBD benchmarks above:

```bash
cargo run --locked --release --no-default-features --features f64,implicit --example fold_towel_implicit -- --dt 0.1 --output implicit-fold.jsonl
```

Use a new output path for each repetition. Its summary reports the mean, p95 and
maximum physical-step time, plus the cost of all 80 steps over 8 simulated seconds.
Step timing includes checkpoints, commands and Rapier/cloth stepping; it excludes
record serialization, rendering and independent geometry audits. Read the
[implicit solver guide](implicit.md) for the fixture and qualification limits.

The implicit solver reuses sparse ordering for unchanged columns and symbolic
factorization for an unchanged complete matrix pattern. Numerical values and
factors are recomputed each iteration. These caches expire at the physical-step
boundary, so changing grasp targets, restoring a checkpoint or retrying a failed
step does not reuse derived data from the previous attempt.

### Optional four-worker execution

`ImplicitSettings::execution` defaults to `ImplicitExecution::Serial`. Use
`ImplicitExecution::Parallel4`, or `--workers 4` in the example, to include up to
three additional native workers for material and sparse-column assembly.
Numerical accumulation order is preserved; factorization and contact queries
remain serial. The application owns its overall thread budget, including other
cloths and the robot simulator. See [CPU execution](implicit.md#cpu-execution)
for failure and checkpoint behavior.

```bash
cargo run --locked --release --no-default-features --features f64,implicit --example fold_towel_implicit -- --dt 0.1 --workers 4 --output implicit-parallel.jsonl
```

Local Linux measurements of the actual public example on an Intel Core i9-13900H
used the same affinity to four distinct physical cores, with serial/parallel/
parallel/serial order at h=0.1 and reversed mode order at h=0.04. Each run simulates
eight seconds, including full grasp release and settling. No builds or tests ran
concurrently; unrelated host load and CPU frequency were uncontrolled.

| Metric | Serial | Four workers |
|---|---:|---:|
| h=0.1 task, two observations | 17.11 / 18.98 s | 14.80 / 14.81 s |
| h=0.1 mean task cost | 18.05 s | 14.81 s |
| h=0.1 p95 physical step | 469–519 ms | 392–396 ms |
| h=0.1 mean process CPU time | 18.09 s | 18.27 s |
| h=0.1 peak process RSS | 79.10 MiB | 90.36 MiB |
| h=0.04 task, two observations | 36.43 / 34.98 s | 37.92 / 32.36 s |
| h=0.04 mean task cost | 35.70 s | 35.14 s |
| h=0.04 mean process CPU time | 35.80 s | 42.16 s |
| h=0.04 peak process RSS | 80.57 MiB | 110.18 MiB |

The h=0.1 mean wall cost decreases by **17.96%**, while all physical states,
iterations and collision-work counters remain exactly equal to the serial
reference. Across eight runs, 1,120 accepted steps and all release/settling
metrics match. Additional peak RSS reaches 11.26 MiB at h=0.1 and 29.61 MiB at
h=0.04. Process CPU and memory usage include recording/allocator overhead;
step wall timings exclude serialization as described above.

The h=0.04 ranges overlap substantially: its 1.59% mean difference does not
establish a useful speedup at that step size. Two repetitions per mode are local
observations, not a cross-platform guarantee. Measure the chosen mode in the
host application. At h=0.1 the four-worker runtime still takes about **1.85 wall
seconds per simulated second**, so wall-clock real time remains unmet.

### Earlier serial optimizations

Earlier measurements on a local Intel Core i9-13900H, before and after this reuse
change measured the following complete-task costs. The run order was before,
after, after, before, with no concurrent build or experiment.

| Metric | Before reuse | With reuse |
|---|---:|---:|
| 8 simulated seconds at h=0.1 | 23.78–24.17 s | 17.67–17.99 s |
| Mean physical step | 297–302 ms | 221–225 ms |
| p95 physical step | 606–612 ms | 476–480 ms |
| Peak process RSS | 49.9–50.3 MiB | 75.1–75.3 MiB |

Mean total cost decreased by about 25.6%, with additional temporary/cache memory.
All accepted positions, velocities, solver iteration counts and collision-work
counts matched the previously audited run exactly. These are local measurements
of this fixture, not a cross-platform timing guarantee. CPU cost remains about
2.2 times simulated time, so wall-clock real time is not yet achieved.

Subsequent assembly optimization skips local Hessian blocks for fixed vertices
and the unused upper triangle, and avoids building these blocks during trial
energy/gradient evaluation. Full h=0.1 and h=0.04 regressions retain identical
physical states and work counts. A later CPU-0 before/after comparison varied
from 46–57 s before to 48–54 s after under changing host frequency and load.
These overlapping ranges do not establish an additional whole-task speedup;
they also should not be compared directly with the earlier 18 s measurement.

In the h=0.1 fixture, the first released step requires 35 Newton iterations,
compared with three in each adjacent step. Its cost is spread across sparse
factorization, assembly, contact queries and continuous motion checks. Evaluate
improvements over the complete task and preserve the release and settling
checks; reducing the physical step count alone does not address this work.

## Measure the full application

The [live demo](live-demo.md#read-the-metrics) exposes render fps, physics time,
response time and simulated-time/wall-time ratio separately. Its softer material
and wind differ from the benchmark fixtures. Budget the full application, including
transport, geometry uploads, normals and rendering, on the target hardware.

Keep time step and iteration count visible alongside timing results. Reducing them
changes accuracy and can require different material parameters. CI checks functional
behavior and error bounds; it does not impose absolute timing thresholds on shared
runners or certify a universal 60 fps target.

## Implicit live response

The [Rapier pose demo](robot-control.md) uses the same 80 accepted h=0.1 states
as its headless example. On the local Intel Core i9-13900H with headless Chromium
153 software WebGL (1440×1050), continuous rendering competed with CPU physics.
Rendering only after physics or view changes reduced the observed full-task wait:

| Rendering | Two runs, 8 simulated seconds | Mean wall time | Per-run response p95 | Maximum response |
|---|---|---|---|---|
| Continuous scene and shadow redraw | 34.29 / 35.96 s | 35.12 s | 929 / 888 ms | 2.66 s |
| Redraw on change, reuse shadows | 20.27 / 19.16 s | 19.71 s | 525 / 518 ms | 1.28 s |

These four runs used an ABBA order and identical layout, viewport, physical
settings and reference commands. Every accepted cloth state and hand pose matched
the headless robot example exactly (320 step comparisons). Timing uses browser
`performance.now()` from the first step request to the last state arrival;
individual responses include serialization, transport and browser scheduling.
Startup/build time and final screenshot capture are excluded.

The 43.9% wall-time reduction is an application-level result on this software
renderer. It does not establish a new native solver speedup or predict the
improvement on hardware WebGL. A separate headless robot run took 15.61 s of
physics time; it was not part of the paired rendering comparison. Rendering
remains responsive to camera input while a solve runs. An unchanged paused view
issues no redraws, and at most one physics request is outstanding.

At about 0.41× simulated/wall time, the measured live demo still falls short of
real-time progress. No time step, Newton budget, collision guard or physical
trajectory was changed for this optimization. The largest response remains
around release. Target-device performance and robustness to changed trajectories
remain separate follow-up work.
