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

## Implicit solver timing

These measurements use the current implementation on an Intel Core i9-13900H
laptop (Linux x86_64, Rust 1.93.0, release, f64): one 32×32 towel, h=0.1 s,
80 steps (8 simulated seconds), `ImplicitExecution::Parallel4` and the strict
iteration policy. Times are the example's summed per-step solve time
(`simulation_wall_seconds`).

| Configuration | 8 s task | Notes |
|---|---:|---|
| Preceding implementation, four workers | 34–35 s | Nominal; same host, not pinned |
| Current, pinned to four P-cores, load average 3–3.6 | 7.2–7.7 s | Nominal; interleaved single runs |
| Current, seven conditions, two repeats each, load average about 3 | 7.6–8.4 s | Medians per condition; `right_late`, `grasp_inset` and `left_early` exceed 8 s |
| Current, sustained or busy host | above 8 s | Laptop clocks fall to 0.4–3.8 GHz at 94–101 °C |
| Current, `Serial`, pinned to one P-core | 7.9–8.4 s | Nominal 7.9 s, `grasp_inset` 8.2–8.4 s; interleaved four-worker runs took 8.0–8.6 s at load average 4–6 |

The seven towel conditions take 567–604 Newton iterations each; the slowest single
step (the lift condition's step 24, 79 iterations) takes about 0.7 s. In an
instrumented nominal run, sparse Cholesky factorization and solves take about 50%
of the time, continuous self-contact sweeps about 12%, material and contact
assembly about 13%, matrix loading about 9% and line-search energies about 6%.
One factorization of the roughly 3,000-unknown system takes 4–7 ms and is
compute-bound; parallel, single-precision and nested-dissection variants were
slower on this host. Because that serial share dominates, four workers now save
about 0.5 s over `Serial` under light load and nothing on a busy host, while
using 11.3–12.1 s of CPU time instead of 7.9 s; a single core is a reasonable
budget for one towel.

The target of finishing the 8 s task within 8 s of solve time on four threads is
met on this laptop for the nominal, lift and friction conditions at a load
average of about 3, and missed by 0.1–0.4 s for the three conditions that hold
the fold longer; sustained thermal throttling changes throughput by about 40%.
Measure on the intended host under its intended load, and compare configurations
with interleaved runs.

## Earlier implicit contact performance

The current implementation omits unused Hessian storage from first-order contact
differentiation. It retains the second-order formulas, contact result buffers,
four-worker scheduler and original accumulation order. In f64, the first-order
intermediate shrinks from 1,256 to 104 bytes; second-order values remain 1,256
bytes. Intermediate size is not an equivalent reduction in process memory.

A local matched comparison on September 21, 2026 used an Intel Core i9-13900H,
Linux x86_64, Rust 1.93.0, release/f64, one 32×32 towel, four workers, actual
`h=0.1` and the explicit approximate cap policy. Each trajectory simulated eight
seconds. Both versions already used parallel material, sparse and parent-contact
assembly; this comparison isolates the derivative storage change.

Each entrypoint/case had one warmup per version and three measured runs per
version, alternating before/after order. No project builds, tests or geometry
audits ran concurrently. Desktop load, CPU frequency and affinity were
uncontrolled. All measured rows are retained. These are local observations,
not a guaranteed speedup or a controlled estimate of performance on other hosts.

| Entrypoint / case | Before, mean wall seconds (range) | After, mean wall seconds (range) | Reduction of means | Median paired reduction |
|---|---:|---:|---:|---:|
| Rapier hands / nominal | 68.36 (54.15–95.52) | 52.63 (52.33–53.06) | 23.0% | 4.2% |
| Rapier hands / lift +5 mm | 54.95 (46.98–61.63) | 52.19 (48.66–55.44) | 5.0% | 6.7% |
| Browser / nominal | 55.29 (43.71–77.28) | 38.15 (36.71–40.74) | 31.0% | 18.2% |
| Browser / lift +5 mm | 45.75 (37.15–59.15) | 47.88 (35.66–65.53) | −4.7% | −3.7% |

A negative reduction means more time. The slow nominal control runs raise its
mean gains substantially; the paired medians describe the same three comparison
pairs without discarding those runs. Browser lift is slower in two of three
pairs and in the overall mean. A consistent improvement across live conditions
has **not** been established. Headless lift also has one pair that is 18.0%
slower despite its lower overall mean.

Headless wall time includes recording output; summed physics time is within
0.11 s of it. Browser timing starts at the first automatic step request and ends
at the final response, excluding startup, manual steps, screenshots and output
validation. Both native servers use the same built viewer, headless Chrome 125,
software WebGL and a 1440×1050 viewport. Browser physics means are 0.8–1.0 s less
than total wait. All 80 step states, outcomes and work counts match in every
headless run; all 1,280 timed/warmup browser frames match the headless Rapier
positions and typed outcomes. Nominal converges throughout; lift step 24 remains
explicitly approximate.

| Case | Headless process CPU mean, before → after | Headless mean peak RSS, before → after | Browser native-server CPU mean, before → after |
|---|---:|---:|---:|
| nominal | 104.44 → 94.76 s | 144.09 → 142.59 MiB | 88.27 → 67.87 s |
| lift +5 mm | 98.23 → 91.80 s | 127.31 → 129.15 MiB | 80.40 → 80.46 s |

CPU is process user+system time across its threads and may exceed wall time.
Browser CPU covers the native server only; it excludes Chrome and software
rendering. RSS includes allocator and recording costs. Smaller AD values do not
remove the existing contact staging buffers or the need to budget four workers.

| Entrypoint / case | Mean per-run step/response p95, before → after | Mean per-run slowest step/response, before → after |
|---|---:|---:|
| Rapier hands / nominal | 2.08 → 1.53 s | 6.00 → 5.23 s |
| Rapier hands / lift +5 mm | 1.56 → 1.48 s | 5.12 → 5.11 s |
| Browser / nominal | 1.56 → 1.05 s | 7.63 → 3.42 s |
| Browser / lift +5 mm | 1.34 → 1.39 s | 4.47 → 3.67 s |

One-minute host load at run start ranged from 3.63 to 17.79 in the headless block
and 4.15 to 17.58 in the browser block. The blocks ran at different times;
software rendering also shares CPU resources. Their absolute differences do not
isolate a rendering effect or qualify hardware GPU rendering.

The separate captured-contact kernel comparison reduces first-order elapsed
loop time by about 90%, but second-order vertex-face / edge-edge evaluation takes
9.8% / 13.5% more time. A matched four-input solver screen lowers combined saved
release-plus-settle time by 11.6% and process CPU by 16.2%; its free/cap controls
stay within the preset 5% regression limit. These narrower results do not imply
a 90% task speedup. Remaining second-order cost and the browser lift regression
need further investigation.

Eight simulated seconds still require about 52 s headless and 38–48 s in the
measured browser means, with individual responses taking several seconds.
Wall-clock real time remains unmet. See the
[qualification boundary](implicit.md#qualification-boundary) for task and
accuracy limits.

Reproduce the current Rapier case with:

```sh
cargo run --locked --release --no-default-features --features f64,implicit --example robot_towel_implicit -- --case nominal --cap-policy approximate --workers 4 --output robot-fold.jsonl
```

## Implicit towel folding

The implicit timings below are historical measurements of the preceding contact
implementation. Use the [earlier matched comparison](#earlier-implicit-contact-performance)
above for the coherent parent-primitive contact model and explicit approximate
cap policy. The following numbers are not current performance claims.

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
