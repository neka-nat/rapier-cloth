//! Folds a parametric T-shirt with two ideal grippers on the implicit solver
//! and writes a schema-2 recording for `demos/viewer`.
//!
//! ```bash
//! cargo run --locked --release --no-default-features --features f64,implicit \
//!     --example fold_shirt_implicit -- --layers sewn --spacing 0.02 --workers 4 \
//!     --output target/fold_shirt.json
//! ```
#[path = "support/folding_oracle.rs"]
mod folding_oracle;
#[path = "support/garment_task.rs"]
#[allow(dead_code)]
mod garment_task;

use garment_task::{ShirtTask, ShirtTaskConfig};
use rapier_cloth::core::garment::{GarmentLayers, TShirtPattern};
use rapier_cloth::{ImplicitCapPolicy, ImplicitExecution};
use std::io::Write;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut config = ShirtTaskConfig::default();
    let mut output = None;
    let mut summary_path = None;
    let mut steps = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or(format!("{arg} requires a value"));
        match arg.as_str() {
            "--layers" => {
                config.pattern.layers = match value()?.as_str() {
                    "single" => GarmentLayers::Single,
                    "sewn" => GarmentLayers::Sewn,
                    other => return Err(format!("unknown layers {other}").into()),
                }
            }
            "--spacing" => config.pattern.spacing = value()?.parse()?,
            "--workers" => {
                config.execution = match value()?.as_str() {
                    "1" => ImplicitExecution::Serial,
                    "4" => ImplicitExecution::Parallel4,
                    other => return Err(format!("--workers requires 1 or 4, got {other}").into()),
                }
            }
            "--cap-policy" => {
                config.cap_policy = match value()?.as_str() {
                    "strict" => ImplicitCapPolicy::Strict,
                    "approximate" => ImplicitCapPolicy::ApproximateWithFinalValidation,
                    other => return Err(format!("unknown cap policy {other}").into()),
                }
            }
            "--friction" => config.friction = value()?.parse()?,
            "--pinch" => {
                config.pinch = match value()?.as_str() {
                    "top" => garment_task::Pinch::TopLayer,
                    "all" => garment_task::Pinch::AllLayers,
                    other => return Err(format!("unknown pinch {other}").into()),
                }
            }
            "--sleeve-fold" => config.sleeve_fold = value()?.parse()?,
            "--band" => config.band = value()?.parse()?,
            "--h" => config.h = value()?.parse()?,
            "--hem-inset" => config.hem_inset = value()?.parse()?,
            "--hem-arc-height" => config.hem_arc_height = value()?.parse()?,
            "--release-height" => config.release_height = value()?.parse()?,
            "--max-iterations" => config.max_iterations = value()?.parse()?,
            "--hem-fold" => config.hem_fold = value()?.parse()?,
            "--patch-radius" => config.patch_radius = value()?.parse()?,
            "--steps" => steps = Some(value()?.parse::<usize>()?),
            "--output" => output = Some(value()?),
            "--summary" => summary_path = Some(value()?),
            "--help" | "-h" => {
                println!(
                    "fold_shirt_implicit [--layers single|sewn] [--spacing m] [--workers 1|4] \
                     [--cap-policy strict|approximate] [--friction mu] [--patch-radius m] [--pinch top|all] \
                     [--sleeve-fold s] [--hem-fold s] [--band m] [--h s] [--max-iterations n] [--hem-inset m] [--hem-arc-height k] [--release-height m] \
                     [--steps n] [--output recording.json] [--summary summary.json]"
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument {other}").into()),
        }
    }
    let pattern: TShirtPattern = config.pattern;
    let mut task = ShirtTask::new(config)?;
    let schedule = task.schedule();
    let garment = task.garment();
    eprintln!(
        "shirt: {:?}, spacing {} m, {} vertices, {} triangles; steps: grasp {} release {} grasp {} release {} end {}",
        pattern.layers,
        pattern.spacing,
        garment.mesh().rest_positions().len(),
        garment.mesh().triangles().len(),
        schedule.sleeve_grasp,
        schedule.sleeve_release,
        schedule.hem_grasp,
        schedule.hem_release,
        schedule.end
    );
    let limit = steps.unwrap_or(schedule.end).min(schedule.end);
    let mut failure = None;
    while task.step() < limit {
        match task.tick() {
            Ok(sample) => {
                if sample.step % 10 == 0 || sample.step == limit {
                    eprintln!(
                        "step {:3} t {:5.1} s  {:6.3} s/step  iterations {:3}  contacts {:6}  extension {:.4}  max speed {:.4}  held {}",
                        sample.step,
                        sample.time,
                        sample.seconds,
                        sample.iterations,
                        sample.contacts,
                        sample.max_edge_extension,
                        sample.max_speed,
                        sample.held
                    );
                }
            }
            Err(error) => {
                eprintln!("{error}");
                failure = Some(error);
                break;
            }
        }
    }
    let summary = task.summary();
    let summary_json = summary.to_json();
    println!("{summary_json}");
    if let Some(path) = summary_path {
        std::fs::write(&path, serde_json::to_string_pretty(&summary_json)?)?;
    }
    if let Some(path) = output {
        let recording = task.recording(&summary);
        let mut file = std::io::BufWriter::new(std::fs::File::create(&path)?);
        serde_json::to_writer(&mut file, &recording)?;
        file.flush()?;
        eprintln!("recording written to {path}");
    }
    if let Some(error) = failure {
        return Err(error.into());
    }
    if steps.is_none() && !summary.passed {
        return Err("the completed fold did not meet the task gates".into());
    }
    Ok(())
}
