# Documentation

Start with the [project README](../README.md) for installation and a minimal Rust
example. These guides describe supported behavior and runnable workflows.

| Guide | Use it to |
|---|---|
| [Live demo](live-demo.md) | Interact with a cloth simulated by a local Rust CPU server |
| [Examples](examples.md) | Run headless simulations, generate recordings and inspect them |
| [Integration](integration.md) | Add cloth to a Rapier world, tune materials and handle errors |
| [Rapier end-effector control](robot-control.md) | Command two ideal grasp frames in headless and live implicit simulations |
| [Implicit shell solver](implicit.md) | Run the experimental f64 fold, release and settle example |
| [Surface selection and grasping](grasping.md) | Select exposed material points or patches and attach them to grippers |
| [Compatibility and limitations](compatibility.md) | Check precision, toolchain and collision support |
| [Recording format](recording-format.md) | Read the versioned JSON used by the replay viewer |
| [Performance](performance.md) | Measure physics cost and interpret frame timings |

For API documentation, run `cargo doc --no-deps --open` from the repository.
For development and checks, see [Contributing](../CONTRIBUTING.md).
