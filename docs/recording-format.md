# Recording format v1

The `Recording` and `Summary` types in
[examples/support/recording.rs](../examples/support/recording.rs) define the replay
format. Serde is a development dependency for examples, tests and benchmarks, not
a normal library dependency. Recordings support viewing and analysis, not restarting
a solver.

Units are metres, seconds and kilograms. Coordinates use the Rapier world frame;
examples are Y-up. Quaternions use `[x, y, z, w]`. f64 results remain f64 through JSON
serialization with `serde_json`'s `float_roundtrip` feature. The viewer retains the
received numbers and converts only rendering positions to `Float32Array`.

| Field | Meaning |
|---|---|
| `schema_version` | Currently 1; other versions are rejected by the viewer |
| `precision` | `f32` or `f64` |
| `config` | Grid, time step, iterations, material, action step indices and recording stride |
| `triangles` | Zero-based index triples shared by all frames |
| `shapes` | Box half-extents, body-local offset and display color, keyed by recording body ID |
| `frames[].step` / `time` | External substep index / elapsed seconds |
| `frames[].positions` | World-space `[x, y, z]` triples in physics vertex order |
| `frames[].bodies` | Stable recording IDs, world translation and rotation |
| `frames[].attached_particles` / `anchors` | Attached vertex indices and corresponding world-space anchors |
| `frames[].pinned_particles` | Pinned vertex indices |
| `frames[].diagnostics` | Maximum/p95 strain, penetration, target error and contact count |

`phase` is `settle`, `lift`, `transport`, `release` or `drop`. Attachment is created
from the ending state of `attach_step`, before the next substep. Release occurs at
the end of `release_step`; that frame contains no attachment. Velocities immediately
before release are saved in the summary.

The summary records release time and velocities, lifted height, transport endpoint,
final center and maximum height, and maximum errors over the simulation. Transport
distance is evaluated from the actual grasp position, including deformation during
settling. `finite: true` is returned only after all substeps complete their finite
checks. A failed example exits nonzero instead of producing a success summary.

The viewer's shape format currently supports boxes only; that display format does
not define the library's collision support. The viewer validates indices, finite
values, time ordering and body correspondence before replacing the current display.
The live demo uses a [separate request/response protocol](live-demo.md#architecture).
