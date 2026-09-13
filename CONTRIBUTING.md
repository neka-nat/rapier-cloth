# Contributing

Bug reports, focused fixes, documentation improvements and runnable examples are
welcome. Include the reproduction, expected behavior and actual result. For
simulation issues, provide precision, mesh resolution, material, time step,
iterations and the relevant error or diagnostic values.

## Set up a checkout

```bash
git clone https://github.com/neka-nat/rapier-cloth.git
cd rapier-cloth
```

The repository pins Rust 1.93.0; rustup selects it automatically. Library code must
remain compatible with Rust 1.90. Install Node.js 22.12 or newer for browser work,
and Python 3 for documentation/package checks. The package-check shell script
requires a Unix-compatible environment.

## Rust checks

Select one precision at a time. `--all-features` is intentionally unsupported.

```bash
cargo fmt --all -- --check
cargo test --locked --workspace
cargo test --locked --workspace --no-default-features --features f64
cargo clippy --locked --workspace --all-targets -- -D warnings
cargo clippy --locked --workspace --all-targets --no-default-features --features f64 -- -D warnings
bash scripts/check-features.sh
```

The live server is a separate workspace, so check it explicitly when changing it:

```bash
cargo fmt --manifest-path demos/live-server/Cargo.toml -- --check
cargo test --locked --manifest-path demos/live-server/Cargo.toml
cargo test --locked --manifest-path demos/live-server/Cargo.toml --no-default-features --features f64
cargo clippy --locked --manifest-path demos/live-server/Cargo.toml --all-targets -- -D warnings
cargo clippy --locked --manifest-path demos/live-server/Cargo.toml --all-targets --no-default-features --features f64 -- -D warnings
```

## Browser checks

```bash
npm --prefix demos/viewer ci
npm --prefix demos/viewer exec -- playwright install chromium
npm --prefix demos/viewer run build
npm --prefix demos/viewer run test
npm --prefix demos/viewer run test:live
CLOTH_LIVE_PRECISION=f64 npm --prefix demos/viewer run test:live
```

The last command uses shell environment-variable syntax; set the same variable
through your shell on Windows. An existing Chromium executable can be selected with
`CHROME_PATH`, for example `/usr/bin/google-chrome` on Linux. Live browser tests start
the actual Rust server on port 9174 and a production preview on port 4174. Screenshots
and reports are written under `demos/viewer/test-results/`.

## Documentation, examples and package checks

```bash
python3 scripts/check-docs.py
cargo test --locked --workspace --doc
cargo test --locked --workspace --doc --no-default-features --features f64
cargo doc --locked --workspace --no-deps
bash scripts/check-examples.sh
bash scripts/check-packages.sh
```

Keep English as the primary language for public documentation and examples. Keep the
root README concise, link detailed usage from `docs/`, and document user-visible
behavior, supported inputs and limitations. Run the documented example commands
when changing their code or usage. The documentation checker validates local links
and anchors; the package check inspects both crate archives and runs independent
consumers against their extracted sources. These checks do not publish a release.

For performance changes, use the [benchmark guide](docs/performance.md) and preserve
accuracy conditions. Do not turn rendering screenshots or shared-runner timings into
hardware performance guarantees.

## Submit a change

Open a pull request with the problem, resulting behavior and relevant checks. Keep
unrelated changes separate, preserve the selected precision and dependency boundaries,
and include regression coverage for changes to solver or integration contracts.
Documentation and example corrections should stay focused on the public workflow.

Unless explicitly stated otherwise, contributions are licensed under the project's
[MIT License](LICENSE-MIT).
