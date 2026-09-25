#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
package_check_dir="$(mktemp -d "${TMPDIR:-/tmp}/rapier-cloth-package-XXXXXX")"
echo "Package evidence: $package_check_dir"
version="$(cargo metadata --no-deps --format-version 1 --locked | python3 -c 'import json, sys; print(next(p["version"] for p in json.load(sys.stdin)["packages"] if p["name"] == "rapier-cloth"))')"
echo "Package version: $version"
# Exercise exclusion even on clean CI checkouts with no local work records.
mkdir -p .internal
package_probe_dir="$(mktemp -d "$PWD/.internal/package-probe-XXXXXX")"
trap 'rm -rf -- "$package_probe_dir"' EXIT
for probe in README.md CHANGELOG.md CONTRIBUTING.md LICENSE-PROBE; do
  printf 'Generated package exclusion probe.\n' > "$package_probe_dir/$probe"
done
for precision in f32 f64; do
  package_target="$package_check_dir/target-$precision"
  cargo package --workspace --locked --allow-dirty --target-dir "$package_target" --no-default-features --features "rapier-cloth/$precision,rapier-cloth-core/$precision"
  unpacked="$package_check_dir/unpacked-$precision"
  mkdir -p "$unpacked"
  for name in rapier-cloth-core rapier-cloth; do
    tar -xzf "$package_target/package/$name-$version.crate" -C "$unpacked"
    test -f "$unpacked/$name-$version/src/lib.rs"
    cmp LICENSE-MIT "$unpacked/$name-$version/LICENSE-MIT"
    test ! -e "$unpacked/$name-$version/LICENSE-APACHE"
    tar -tzf "$package_target/package/$name-$version.crate" > "$package_check_dir/$name-$precision-contents.txt"
    if grep -E '/(\.internal|evidence|node_modules|target)/|/(progress\.md|package-design\.ja\.md|implementation-plan\.ja\.md)$' "$package_check_dir/$name-$precision-contents.txt"; then
      echo "Internal work records and build dependencies must not be distributed" >&2
      exit 1
    fi
    python3 scripts/check-docs.py --root "$unpacked/$name-$version"
    python3 - "$name" "$unpacked/$name-$version" <<'PY'
import pathlib, sys
name, artifact = sys.argv[1], pathlib.Path(sys.argv[2])
source = pathlib.Path('crates/rapier-cloth-core' if name == 'rapier-cloth-core' else '.')
for directory in ['src', 'examples']:
    expected = {p.relative_to(source) for p in (source / directory).rglob('*.rs')}
    actual = {p.relative_to(artifact) for p in (artifact / directory).rglob('*.rs')}
    assert expected == actual, (name, expected ^ actual)
    for relative in expected:
        assert (source / relative).read_bytes() == (artifact / relative).read_bytes(), relative
print(f'Verified: {name} library and example Rust sources match the workspace.')
PY
    sha256sum "$package_target/package/$name-$version.crate" >> "$package_check_dir/SHA256SUMS"
  done
  consumer="$package_check_dir/consumer-$precision"
  mkdir -p "$consumer/src"
  cp "tests/consumers/$precision/src/main.rs" "$consumer/src/main.rs"
  rapier_package=rapier3d
  if [[ "$precision" == f64 ]]; then rapier_package=rapier3d-f64; fi
  cat > "$consumer/Cargo.toml" <<TOML
[package]
name = "packaged-consumer-$precision"
version = "0.0.0"
edition = "2024"
[dependencies]
rapier-cloth = { version = "=$version", default-features = false, features = ["$precision"] }
$rapier_package = "0.34"
[patch.crates-io]
rapier-cloth = { path = "$unpacked/rapier-cloth-$version" }
rapier-cloth-core = { path = "$unpacked/rapier-cloth-core-$version" }
[workspace]
TOML
  cargo run --offline --manifest-path "$consumer/Cargo.toml"
  cargo test --offline --manifest-path "$consumer/Cargo.toml"
  cargo run --offline --manifest-path "$unpacked/rapier-cloth-$version/Cargo.toml" \
    --config "patch.crates-io.rapier-cloth-core.path=\"$unpacked/rapier-cloth-core-$version\"" \
    --target-dir "$consumer/target" --no-default-features --features "$precision" \
    --example surface_grasp > "$package_check_dir/surface-grasp-$precision.json"
  python3 - "$package_check_dir/surface-grasp-$precision.json" "$precision" <<'PY'
import json, math, pathlib, sys
report = json.loads(pathlib.Path(sys.argv[1]).read_text())
assert report['precision'] == sys.argv[2] and report['steps'] == 240
assert report['finite'] and report['released']
assert math.isfinite(report['max_target_error']) and report['max_target_error'] <= 1e-5
assert all(math.isfinite(b) and b > 0 for b in report['barycentric'])
print(f'Verified: extracted {sys.argv[2]} surface_grasp example lifted and released.')
PY
  cargo build --offline --manifest-path "$unpacked/rapier-cloth-$version/Cargo.toml" \
    --config "patch.crates-io.rapier-cloth-core.path=\"$unpacked/rapier-cloth-core-$version\"" \
    --target-dir "$consumer/target" --no-default-features --features "$precision" --example fold_towel
  python3 scripts/check-folding-example.py --binary "$consumer/target/debug/examples/fold_towel" \
    --precision "$precision" --output "$package_check_dir/fold-$precision"
  cargo metadata --offline --format-version 1 --manifest-path "$consumer/Cargo.toml" > "$package_check_dir/metadata-$precision.json"
  python3 - "$package_check_dir/metadata-$precision.json" "$unpacked" "$version" <<'PY'
import json, pathlib, sys
metadata=json.loads(pathlib.Path(sys.argv[1]).read_text())
base=pathlib.Path(sys.argv[2]).resolve()
version=sys.argv[3]
for name in ['rapier-cloth','rapier-cloth-core']:
    packages=[p for p in metadata['packages'] if p['name']==name]
    assert len(packages)==1, packages
    manifest=pathlib.Path(packages[0]['manifest_path']).resolve()
    assert manifest.is_relative_to(base), manifest
    assert packages[0]['version']==version, (name, packages[0]['version'])
    assert packages[0]['license']=='MIT', (name, packages[0]['license'])
print('Verified: both library sources are extracted package artifacts licensed under MIT.')
PY
done
printf 'Package and independent consumer checks passed. Evidence: %s\n' "$package_check_dir"
