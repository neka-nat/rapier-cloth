//! Experimental dual-gripper towel fold with audited summaries and replay data.
#[path = "support/folding.rs"]
mod folding;
#[path = "support/folding_recording.rs"]
mod folding_recording;
#[path = "support/folding_report.rs"]
mod folding_report;
#[path = "support/folding_run.rs"]
mod folding_run;
#[path = "support/folding_oracle.rs"]
mod oracle;
#[path = "support/recording.rs"]
mod recording;

use folding::{CollisionMode, Config, configured_world};
use folding_run::{output, run, source_identity};
use std::{fs, io::Write, path::Path, process::ExitCode};

fn reserve(path: Option<&str>) -> Result<Option<fs::File>, Box<dyn std::error::Error>> {
    path.map(|path| {
        if let Some(parent) = Path::new(path)
            .parent()
            .filter(|p| !p.as_os_str().is_empty())
        {
            fs::create_dir_all(parent)?;
        }
        Ok(fs::OpenOptions::new()
            .create_new(true)
            .write(true)
            .open(path)?)
    })
    .transpose()
}
fn write_json(
    file: Option<fs::File>,
    value: &impl serde::Serialize,
) -> Result<(), Box<dyn std::error::Error>> {
    if let Some(file) = file {
        let mut writer = std::io::BufWriter::new(file);
        serde_json::to_writer(&mut writer, value)?;
        writer.flush()?;
    }
    Ok(())
}
fn execute() -> Result<ExitCode, Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let mut record = None;
    let mut summary = None;
    let mut variant = 0;
    let mut limit = None;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--record" => record = Some(args.next().ok_or("--record requires a new path")?),
            "--summary" => summary = Some(args.next().ok_or("--summary requires a new path")?),
            "--variant" => variant = args.next().ok_or("--variant requires an index")?.parse()?,
            "--max-steps" => {
                limit = Some(
                    args.next()
                        .ok_or("--max-steps requires a count")?
                        .parse::<u64>()?,
                )
            }
            "--help" => {
                println!(
                    "fold_towel [--variant 0..5] [--record NEW_FILE.json] [--summary NEW_FILE.json] [--max-steps N]\n\nRuns frozen fixture 2 with both continuous collision modes and independent audits.\nExit 1: simulation/error; exit 2: requested partial run; exit 0: trajectory completed.\nUse scripts/check-folding.sh from a checkout to check correctness; completion alone is not qualification.\nThe current solver may stop before releasing the towel. A solver failure during the task preserves accepted frames and the failure report."
                );
                return Ok(ExitCode::SUCCESS);
            }
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }
    let config = Config::with_version(2)?;
    let actual = config.for_variant(variant)?;
    if limit.is_some_and(|n| n == 0 || n > actual.end_step) {
        return Err("max-steps must be between 1 and the selected task end step".into());
    }
    let record_file = reserve(record.as_deref())?;
    let summary_file = reserve(summary.as_deref())?;
    let (source_commit, source_dirty) = source_identity();
    let mode = CollisionMode {
        self_collision: true,
        continuous_self: true,
        rigid_surface: true,
        continuous_rigid: true,
    };
    let mut warmup = configured_world(config.clone(), variant, mode)?;
    for _ in 0..120 {
        warmup.tick()?;
    }
    drop(warmup);
    let mut recorder = folding_recording::Recorder::default();
    let result = run(
        config.clone(),
        variant,
        true,
        mode,
        limit,
        |task, report, last| {
            if record_file.is_some() {
                recorder.observe(task, report, last)?;
            }
            Ok(())
        },
    )?;
    let exit = if !result["failure"].is_null() {
        ExitCode::FAILURE
    } else if result["steps"].as_u64() != Some(actual.end_step) {
        ExitCode::from(2)
    } else {
        ExitCode::SUCCESS
    };
    let precision = if cfg!(feature = "f64") { "f64" } else { "f32" };
    let cpu = fs::read_to_string("/proc/cpuinfo")
        .ok()
        .and_then(|s| {
            s.lines()
                .find(|l| l.starts_with("model name"))
                .and_then(|l| l.split_once(':'))
                .map(|(_, v)| v.trim().to_string())
        })
        .unwrap_or_else(|| output("sysctl", &["-n", "machdep.cpu.brand_string"]));
    let recording = recorder.finish(&result)?;
    let report = serde_json::json!({
        "schema_version":2,"fixture_version":config.version,"precision":precision,
        "cpu":cpu,"os":output("uname", &["-a"]),"rust":output("rustc", &["--version"]),
        "commit":source_commit,"dirty":source_dirty,"substeps_per_frame":4,"warmup_substeps":120,
        "scope":"Rapier, snapshots, task transitions and cloth; excludes oracle, task metrics, rendering and output",
        "runs":[result],"recording_enabled":record_file.is_some()
    });
    if let Some(recording) = recording {
        write_json(record_file, &recording)?;
    }
    write_json(summary_file, &report)?;
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(exit)
}
fn main() -> ExitCode {
    match execute() {
        Ok(exit) => exit,
        Err(error) => {
            eprintln!("fold_towel: {error}");
            ExitCode::FAILURE
        }
    }
}
