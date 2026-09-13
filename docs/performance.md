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

This is a diagnostic, not a qualified folding example: persistent static friction
and full-task qualification are still missing. Replace `--self-collision` with
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

## Measure the full application

The [live demo](live-demo.md#read-the-metrics) exposes render fps, physics time,
response time and simulated-time/wall-time ratio separately. Its softer material
and wind differ from the benchmark fixtures. Budget the full application, including
transport, geometry uploads, normals and rendering, on the target hardware.

Keep time step and iteration count visible alongside timing results. Reducing them
changes accuracy and can require different material parameters. CI checks functional
behavior and error bounds; it does not impose absolute timing thresholds on shared
runners or certify a universal 60 fps target.
