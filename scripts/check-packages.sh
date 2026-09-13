#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
package_check_dir="$(mktemp -d "${TMPDIR:-/tmp}/rapier-cloth-package-XXXXXX")"
echo "Package evidence: $package_check_dir"
for precision in f32 f64; do
  package_target="$package_check_dir/target-$precision"
  cargo package --workspace --locked --allow-dirty --target-dir "$package_target" --no-default-features --features "rapier-cloth/$precision,rapier-cloth-core/$precision"
  unpacked="$package_check_dir/unpacked-$precision"
  mkdir -p "$unpacked"
  for name in rapier-cloth-core rapier-cloth; do
    tar -xzf "$package_target/package/$name-0.1.0.crate" -C "$unpacked"
    test -f "$unpacked/$name-0.1.0/src/lib.rs"
    test -f "$unpacked/$name-0.1.0/LICENSE-MIT"
    test -f "$unpacked/$name-0.1.0/LICENSE-APACHE"
    tar -tzf "$package_target/package/$name-0.1.0.crate" > "$package_check_dir/$name-$precision-contents.txt"
    sha256sum "$package_target/package/$name-0.1.0.crate" >> "$package_check_dir/SHA256SUMS"
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
rapier-cloth = { version = "=0.1.0", default-features = false, features = ["$precision"] }
$rapier_package = "0.34"
[patch.crates-io]
rapier-cloth = { path = "$unpacked/rapier-cloth-0.1.0" }
rapier-cloth-core = { path = "$unpacked/rapier-cloth-core-0.1.0" }
[workspace]
TOML
  cargo run --offline --manifest-path "$consumer/Cargo.toml"
  cargo test --offline --manifest-path "$consumer/Cargo.toml"
  cargo metadata --offline --format-version 1 --manifest-path "$consumer/Cargo.toml" > "$package_check_dir/metadata-$precision.json"
  python3 - "$package_check_dir/metadata-$precision.json" "$unpacked" <<'PY'
import json, pathlib, sys
metadata=json.loads(pathlib.Path(sys.argv[1]).read_text())
base=pathlib.Path(sys.argv[2]).resolve()
for name in ['rapier-cloth','rapier-cloth-core']:
    packages=[p for p in metadata['packages'] if p['name']==name]
    assert len(packages)==1, packages
    manifest=pathlib.Path(packages[0]['manifest_path']).resolve()
    assert manifest.is_relative_to(base), manifest
    assert packages[0]['version']=='0.1.0'
print('Verified: both library sources are extracted package artifacts.')
PY
done
printf 'Package and independent consumer checks passed. Evidence: %s\n' "$package_check_dir"
