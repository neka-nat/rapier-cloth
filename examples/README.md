# Rust examples

Run from the repository root in release mode. Examples are headless and print their
results; they do not open a window.

| Example | Start here for |
|---|---|
| [hanging_cloth.rs](hanging_cloth.rs) | The engine-independent solver and fixed vertices |
| [drape_static.rs](drape_static.rs) | Rapier contact queries and explicit substep ordering |
| [moving_anchor.rs](moving_anchor.rs) | A body-local attachment and velocity-preserving release |
| [surface_grasp.rs](surface_grasp.rs) | Visible surface selection, weighted lift and release |
| [pick_and_place.rs](pick_and_place.rs) | Multi-vertex attachments and JSON recording |
| [fold_towel.rs](fold_towel.rs) | Experimental dual-gripper fold with audited partial/failure recordings |
| [fold_towel_implicit.rs](fold_towel_implicit.rs) | Experimental global shell solve; fold, release and settle at 0.04 or 0.1 s |

```bash
cargo run --locked --release --example hanging_cloth
cargo run --locked --release --example drape_static
cargo run --locked --release --example moving_anchor
cargo run --locked --release --example surface_grasp
cargo run --locked --release --example pick_and_place -- --help
cargo run --locked --release --example fold_towel -- --help
cargo run --locked --release --no-default-features --features f64,implicit --example fold_towel_implicit -- --dt 0.1
cargo run --locked --release --no-default-features --features f64,implicit --example fold_towel_implicit -- --dt 0.1 --workers 4
```

Add `--no-default-features --features f64` before `--example` to use f64. Do not use
`--all-features`. See [the example guide](../docs/examples.md) for recording commands,
expected output and the [live demo](../docs/live-demo.md) for an interactive window.
