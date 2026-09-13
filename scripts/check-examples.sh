#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
example_check_dir="$(mktemp -d "${TMPDIR:-/tmp}/rapier-cloth-examples-XXXXXX")"
for precision in f32 f64; do
  for example in hanging_cloth drape_static moving_anchor; do
    cargo run --locked --release --no-default-features --features "$precision" --example "$example" > "$example_check_dir/$example-$precision.log"
    test -s "$example_check_dir/$example-$precision.log"
  done
  cargo run --locked --release --no-default-features --features "$precision" --example pick_and_place -- --help > "$example_check_dir/help-$precision.log"
  cargo run --locked --release --no-default-features --features "$precision" --example pick_and_place -- --record "$example_check_dir/recording-$precision.json" --summary "$example_check_dir/summary-$precision.json" > "$example_check_dir/stdout-$precision.json"
  python3 - "$example_check_dir" "$precision" <<'PY'
import json, math, pathlib, sys
folder, precision = pathlib.Path(sys.argv[1]), sys.argv[2]
recording = json.loads((folder / f'recording-{precision}.json').read_text())
summary = json.loads((folder / f'summary-{precision}.json').read_text())
assert summary == json.loads((folder / f'stdout-{precision}.json').read_text())
assert summary['finite'] is True and summary['precision'] == precision
assert recording['schema_version'] == 1 and recording['precision'] == precision
assert len(recording['frames']) == 391
assert recording['frames'][0]['positions'] != recording['frames'][-1]['positions']
assert all(math.isfinite(x) for frame in recording['frames'] for p in frame['positions'] for x in p)
assert any(frame['attached_particles'] for frame in recording['frames'])
assert recording['frames'][-1]['attached_particles'] == []
print(f'All four {precision} examples ran; recording and stdout/file summary verified.')
PY
done
printf 'Example outputs: %s\n' "$example_check_dir"
