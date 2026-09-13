//! Complete dual-gripper task timing, with optional independent offline geometry checks.
#[path = "../examples/support/folding.rs"]
mod folding;
#[path = "../examples/support/folding_report.rs"]
mod folding_report;
#[path = "../examples/support/folding_run.rs"]
mod folding_run;
#[path = "../examples/support/folding_oracle.rs"]
mod oracle;
use folding::*;
use folding_run::*;
use serde_json::json;
use std::fs;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let mut directory = None;
    let mut repeats = 5;
    let mut verify = false;
    let mut mode = CollisionMode::default();
    let mut config = Config::default();
    let mut fixture_version = 1;
    let mut bend_override = None;
    let mut variant = 0;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--output" => directory = Some(args.next().ok_or("--output requires a directory")?),
            "--repeats" => {
                repeats = args
                    .next()
                    .ok_or("--repeats requires a count")?
                    .parse::<usize>()?
            }
            "--variant" => {
                variant = args
                    .next()
                    .ok_or("--variant requires an index")?
                    .parse::<usize>()?
            }
            "--bend-compliance" => {
                bend_override = Some(
                    args.next()
                        .ok_or("--bend-compliance requires a value")?
                        .parse::<f64>()?,
                );
            }
            "--fixture-version" => {
                fixture_version = args
                    .next()
                    .ok_or("--fixture-version requires 1 or 2")?
                    .parse()?;
            }
            "--verify" => verify = true,
            "--self-collision" => mode.self_collision = true,
            "--rigid-surface-collision" => mode.rigid_surface = true,
            "--continuous-self-collision" => {
                mode.self_collision = true;
                mode.continuous_self = true;
            }
            "--continuous-rigid-collision" => {
                mode.rigid_surface = true;
                mode.continuous_rigid = true;
            }
            "--bench" => (),
            _ => return Err(format!("unknown argument {arg}").into()),
        }
    }
    if repeats == 0 || repeats > 100 {
        return Err("repeats must be in 1..=100".into());
    }
    if fixture_version != 1 {
        config = Config::with_version(fixture_version)?;
    }
    if let Some(bend) = bend_override {
        config.bend_compliance = bend;
    }
    // Capture source identity before a long run, so later workspace edits do
    // not relabel the already-built benchmark at report-writing time.
    let (source_commit, source_dirty) = source_identity();
    let mut warmup = configured_world(config.clone(), variant, mode)?;
    for _ in 0..120 {
        warmup.tick()?;
    }
    let mut runs = vec![];
    for _ in 0..repeats {
        runs.push(run(
            config.clone(),
            variant,
            verify,
            mode,
            None,
            |_, _, _| Ok(()),
        )?);
    }
    let cpu = fs::read_to_string("/proc/cpuinfo")
        .ok()
        .and_then(|s| {
            s.lines()
                .find(|l| l.starts_with("model name"))
                .and_then(|l| l.split_once(':'))
                .map(|(_, v)| v.trim().to_string())
        })
        .unwrap_or_else(|| output("sysctl", &["-n", "machdep.cpu.brand_string"]));
    let precision = if cfg!(feature = "f64") { "f64" } else { "f32" };
    let result = json!({"schema_version":2,"fixture_version":config.version,"precision":precision,"cpu":cpu,
        "os":output("uname",&["-a"]),"rust":output("rustc",&["--version"]),"commit":source_commit,
        "dirty":source_dirty,"substeps_per_frame":4,"warmup_substeps":120,
        "scope":"Rapier, snapshots, task transitions and cloth; excludes oracle, task metrics, rendering and output",
        "runs":runs});
    if let Some(directory) = directory {
        fs::create_dir_all(&directory)?;
        let file = fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(format!("{directory}/folding-{precision}.json"))?;
        serde_json::to_writer_pretty(std::io::BufWriter::new(file), &result)?;
    }
    println!("{}", serde_json::to_string_pretty(&result)?);
    Ok(())
}
