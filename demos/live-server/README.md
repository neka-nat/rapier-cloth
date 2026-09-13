# Live CPU server

An unpublished example application that runs one `RapierClothWorld` per WebSocket
connection. It is a separate Cargo workspace so networking dependencies do not
become library dependencies.

For the combined UI/server launcher, see the
[live demo guide](../../docs/live-demo.md). To run only the server from the repository root:

```bash
cargo run --locked --release --manifest-path demos/live-server/Cargo.toml
cargo run --locked --release --manifest-path demos/live-server/Cargo.toml -- --help
```

The default listener is `127.0.0.1:9100`; `/health` reports readiness and `/live/ws`
accepts configured browser origins. Use `--port` and `--origin` after Cargo's `--`
separator to override them. Add `--no-default-features --features f64` before that
separator for f64. Only one precision can be enabled.

The server advances four 1/240 s substeps per `step` command and does not advance
while idle. This is a local demo transport, not a hosted multi-user service.
