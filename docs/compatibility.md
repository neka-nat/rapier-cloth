# Compatibility

| Component | Contract |
|---|---|
| Development toolchain | Rust 1.93.0, edition 2024 |
| Minimum Rust | 1.90; see actual CI results in progress.md |
| Rapier / Parry | Rapier 0.34.0; Parry 0.29.0 resolved in Cargo.lock |
| Math | glam 0.33.7; f32 Vec3 or f64 DVec3 |
| Precision | Exactly one of f32/f64; f32 is default |
| CI targets | Linux, Windows, macOS, both precisions; completion evidence in progress.md |
| Viewer | Node >=22.12; Three.js 0.186.0, Vite 8.3.0, Playwright 1.63.0 |
| Distribution | Two Rust crate archives; registry publication is separate from R0 |

Core has no Rapier, Parry, renderer or serde normal dependency. Rapier is re-exported by the bridge. Rust 1.86 was an initial candidate based on Rapier's own metadata, but the resolved graph includes nalgebra/wide/safe_arch requiring 1.89 and ordered-float requiring 1.90. Both precision graphs must pass the actual 1.90 job.

The two precisions are mutually exclusive even through feature unification. Do not use `--all-features`, or enable a consumer's default f32 dependency together with f64. This workspace does not declare cross-CPU or cross-precision bitwise determinism. Replaying the same checkpoint on the same build is tested.

Before release, run `bash scripts/check-packages.sh`: workspace packaging, archive inspection, then independent consumers using only extracted crate sources. This validates local distribution artifacts, not installation from crates.io. See [integration](integration.ja.md) for supported shapes, step ordering and explicit exclusions.
