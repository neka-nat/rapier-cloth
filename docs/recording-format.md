# Recording formats

The generic `Recording<Config>` and pick-and-place `Summary` types in
[examples/support/recording.rs](../examples/support/recording.rs) define the replay
format. [Folding recording support](../examples/support/folding_recording.rs) adds
the folding configuration and outcome. Serde is a development dependency for examples, tests and benchmarks, not
a normal library dependency. Recordings support viewing and analysis, not restarting
a solver.

Units are metres, seconds and kilograms. Coordinates use the Rapier world frame;
examples are Y-up. Quaternions use `[x, y, z, w]`. f64 results remain f64 through JSON
serialization with `serde_json`'s `float_roundtrip` feature. The viewer retains the
received numbers and converts only rendering positions to `Float32Array`.

| Field | Meaning |
|---|---|
| `schema_version` | 1 for XPBD recordings; 2 for the implicit replay profile below |
| `precision` | `f32` or `f64` |
| `config` | Task-specific configuration; the viewer requires a positive `h` |
| `triangles` | Zero-based index triples shared by all frames |
| `shapes` | Box half-extents, body-local offset and display color, keyed by recording body ID |
| `frames[].step` / `time` | External substep index / elapsed seconds |
| `frames[].positions` | World-space `[x, y, z]` triples in physics vertex order |
| `frames[].bodies` | Stable recording IDs, world translation and rotation |
| `frames[].attached_particles` / `anchors` | Attached vertex indices and corresponding world-space anchors |
| `frames[].pinned_particles` | Pinned vertex indices |
| `frames[].diagnostics` | Maximum/p95 strain, penetration, target error and contact count |
| `outcome` | Optional task termination metadata, supplied by `fold_towel` |

For pick-and-place, `phase` is `settle`, `lift`, `transport`, `release` or `drop`.
Its configuration includes the recording stride. Attachment is created
from the ending state of `attach_step`, before the next substep. Release occurs at
the end of `release_step`; that frame contains no attachment. Velocities immediately
before release are saved in the summary.

The summary records release time and velocities, lifted height, transport endpoint,
final center and maximum height, and maximum errors over the simulation. Transport
distance is evaluated from the actual grasp position, including deformation during
settling. `finite: true` is returned only after all substeps complete their finite
checks. A failed example exits nonzero instead of producing a success summary.

For folding, `config` is the selected
[folding fixture](../examples/support/fold_fixture_v2.json), with variant-adjusted
phase steps. Phase strings are `Settle`, `Approach`, `Lift`, `Fold`, `Lower`,
`Hold`, `Retract`, `Released` and `Finished`. Frames are emitted every four substeps, with
initial and final accepted states also retained. Shape/body IDs 0, 1 and 2 denote
the table display proxy and the two grippers. The table box illustrates an infinite
physical half-space. Anchors are in world space; each gripper's collider offset
remains in `shapes[].local_translation`.

Folding's `outcome` contains `stop_reason` (`completed`, `step_limit` or
`solver_error`), `steps` (the final accepted frame), `end_step` (the complete task)
and `failure` (a nonempty error string for a solver error, otherwise null).
The viewer rejects inconsistent outcomes and displays partial/failed runs as such.
Older v1 files without this field remain supported. Outcome metadata describes
task termination; it does not certify physical fold quality. Folding's separate
schema-2 summary contains the [full audit](performance.md#check-a-folding-correctness-report).
Frame diagnostics describe the most recently accepted substep, not an aggregate
over the four-substep recording interval. Weighted attachments are not represented
by the vertex/anchor arrays; the folding recorder rejects them.

The viewer's shape format currently supports boxes only; that display format does
not define the library's collision support. The viewer validates indices, finite
values, time ordering and body correspondence before replacing the current display.
The live demo uses a [separate request/response protocol](live-demo.md#architecture).

## Implicit replay profile (v2)

The viewer's converter (`npm --prefix demos/viewer run convert:implicit`, see the
[implicit guide](implicit.md#watch-the-motion)) produces a separate v2 profile
from `fold_towel_implicit` JSONL. Geometry, bodies, frame ordering and
outcome fields follow v1. The required `config.solver` is `implicit` and
`precision` is `f64`. Other v2 solvers and unknown schema versions are rejected.

- Frame diagnostics are `max_edge_extension` (fraction), `rms_speed` and
  `max_speed` (m/s). They are null at the initial state, where no step was solved.
  XPBD penetration, attachment-error and contact-count fields are not fabricated.
- Pinned vertex IDs and triangles come from the matching public command fixture.
  The converter checks initial geometry, accepted step ordering and pin targets.
  The initial state has no pins; commands apply before the first solve.
- Body 0 is a finite display proxy for the fixed halfspace, with its top at
  half the recorded thickness. No robot/gripper geometry is implied by pins.
- Completed runs include the original `summary`, including `settled`, simulated
  duration, computation time and final-window drift. Outcome describes the whole
  recording, independently of the currently displayed frame.
- `provenance` records the input file's SHA-256 and command fixture name.
  Compression is optional: the viewer accepts `.json` and `.json.gz`.

Every accepted position and timestamp is retained without resampling. The viewer
holds each state until the next recorded timestamp; it does not interpolate
motion. These files are visualization data, not solver checkpoints.
