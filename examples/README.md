# Rust examples

Run from the repository root in release mode. Examples are headless and print their
results; they do not open a window.

| Example | Start here for |
|---|---|
| [hanging_cloth.rs](hanging_cloth.rs) | The engine-independent solver and fixed vertices |
| [drape_static.rs](drape_static.rs) | Rapier contact queries and explicit substep ordering |
| [moving_anchor.rs](moving_anchor.rs) | A body-local attachment and velocity-preserving release |
| [pick_and_place.rs](pick_and_place.rs) | Multi-vertex attachments and JSON recording |

```bash
cargo run --locked --release --example hanging_cloth
cargo run --locked --release --example drape_static
cargo run --locked --release --example moving_anchor
cargo run --locked --release --example pick_and_place -- --help
```

Add `--no-default-features --features f64` before `--example` to use f64. Do not use
`--all-features`. See [the example guide](../docs/examples.md) for recording commands,
expected output and the [live demo](../docs/live-demo.md) for an interactive window.
