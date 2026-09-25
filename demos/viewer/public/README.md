# Bundled implicit towel recording

`towel-fold-implicit.json.gz` is a compressed replay of the public
[`fold_towel_implicit` example](../../../examples/fold_towel_implicit.rs), recorded
with f64, h=0.1 s and four workers. It contains 81 saved states (initial state plus
80 accepted steps), 1024 vertices and 1922 triangles. Four seconds of prescribed
folding are followed by four seconds with every grasp released.

The 8 s simulation took 14.804513308 s on the recording host. Playback is not a
real-time solver benchmark. The `settled` flag reports the example's final-window
checks, not a general stability guarantee. The [guide](../../../docs/implicit.md)
documents the fixture and supported scope.

Reproduce from the repository root (use new output names):

```bash
cargo run --locked --release --no-default-features --features f64,implicit --example fold_towel_implicit -- --dt 0.1 --workers 4 --output fold.jsonl
npm --prefix demos/viewer run convert:implicit -- ../../fold.jsonl ../../fold-view.json.gz
```

The bundled source was recorded on 2026-09-18. Input compressed JSONL SHA-256:
`484a13485f8d26da352c5c8d92a5b72e165345337efebe7113ca263072b73793`.
Bundled replay SHA-256:
`4ea3cc49deb90aac127b86a18da888e8765aaecce715882f9827bb953ba69416`.
The converter preserves decoded positions exactly; a fresh run has different
timings and therefore need not produce the same file hash.
