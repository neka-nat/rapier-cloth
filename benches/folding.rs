//! Complete dual-gripper task timing, with optional independent offline geometry checks.
#[path = "../examples/support/folding.rs"]
mod folding;
#[path = "../examples/support/folding_oracle.rs"]
mod oracle;
use folding::*;
use serde_json::json;
use std::{fs, process::Command, time::Instant};
fn output(command: &str, args: &[&str]) -> String {
    Command::new(command)
        .args(args)
        .output()
        .ok()
        .filter(|r| r.status.success())
        .map(|r| String::from_utf8_lossy(&r.stdout).trim().to_string())
        .unwrap_or_else(|| "unknown".into())
}
fn timing(mut ms: Vec<f64>) -> serde_json::Value {
    if ms.is_empty() {
        return serde_json::Value::Null;
    }
    ms.sort_by(f64::total_cmp);
    let q = |p: f64| {
        ms[((p * ms.len() as f64).ceil() as usize)
            .saturating_sub(1)
            .min(ms.len() - 1)]
    };
    json!({"p50_ms":q(0.5),"p95_ms":q(0.95),"p99_ms":q(0.99),"max_ms":ms.last(),"frames":ms.len(),
        "frames_over_budget":ms.iter().filter(|&&x|x>1000.0/60.0).count()})
}
fn run(
    config: Config,
    variant: usize,
    verify: bool,
) -> Result<serde_json::Value, Box<dyn std::error::Error>> {
    let mut simulation = FoldingWorld::new(config, variant)?;
    let rest: Vec<_> = simulation
        .world
        .cloth(simulation.cloth)?
        .mesh()
        .rest_positions()
        .iter()
        .copied()
        .map(point)
        .collect();
    let triangles = simulation
        .world
        .cloth(simulation.cloth)?
        .mesh()
        .triangles()
        .to_vec();
    let mut frames = vec![];
    let mut frame_ms = 0.0;
    let mut frame_by_phase = std::collections::BTreeMap::<String, Vec<f64>>::new();
    let mut max_strain: rapier_cloth::Real = 0.0;
    let mut max_p95: rapier_cloth::Real = 0.0;
    let mut max_target: rapier_cloth::Real = 0.0;
    let mut max_penetration: rapier_cloth::Real = 0.0;
    let mut max_contacts = 0;
    let mut max_scratch = 0;
    let mut error = None;
    let mut settle_center = None;
    let mut drift = 0.0_f64;
    let mut audit = oracle::SurfaceAudit::default();
    let mut audit_steps = 0;
    while simulation.step < simulation.config.end_step {
        let begin = Instant::now();
        let result = simulation.tick();
        frame_ms += begin.elapsed().as_secs_f64() * 1000.0;
        match result {
            Ok(report) => {
                let r = &report.cloths[0].1;
                max_strain = max_strain.max(r.max_stretch);
                max_p95 = max_p95.max(r.p95_stretch);
                max_target = max_target.max(r.max_target_error);
                max_penetration = max_penetration.max(r.max_penetration);
                max_contacts = max_contacts.max(r.contacts);
                max_scratch = max_scratch.max(r.scratch_bytes);
            }
            Err(e) => {
                error = Some(e.to_string());
                break;
            }
        }
        if simulation.step % 4 == 0 {
            frames.push(frame_ms);
            frame_by_phase
                .entry(format!(
                    "{:?}",
                    phase(&simulation.config, simulation.step - 1)
                ))
                .or_default()
                .push(frame_ms);
            frame_ms = 0.0;
        }
        if verify {
            let next = oracle::audit_surface(
                &simulation.positions(),
                &triangles,
                simulation.config.thickness,
            );
            audit.crossing_pairs = audit.crossing_pairs.max(next.crossing_pairs);
            audit.max_separation_deficit = audit
                .max_separation_deficit
                .max(next.max_separation_deficit);
            audit.tested_pairs += next.tested_pairs;
            audit_steps += 1;
        }
        if simulation.step >= simulation.config.retract_end {
            let center = simulation.center_of_mass();
            let initial = settle_center.get_or_insert(center);
            drift = drift.max((center[0] - initial[0]).hypot(center[2] - initial[2]));
        }
    }
    let positions = simulation.positions();
    let final_audit = oracle::audit_surface(&positions, &triangles, simulation.config.thickness);
    let metrics = oracle::fold_metrics(
        &rest,
        &positions,
        &triangles,
        simulation.config.grid,
        simulation.config.size,
    );
    let table_penetration = positions
        .iter()
        .map(|p| (simulation.config.thickness * 0.5 - p[1]).max(0.0))
        .fold(0.0, f64::max);
    Ok(json!({
        "status":"unqualified_particle_baseline", "config":simulation.config,"variant":simulation.variant,
        "steps":simulation.step,"failure":error,"failure_next_step":error.as_ref().map(|_|simulation.step+1),
        "physics_frame":timing(frames),"physics_phases":frame_by_phase.into_iter().map(|(k,v)|(k,timing(v))).collect::<std::collections::BTreeMap<_,_>>(),
        "max_strain":max_strain,"max_p95_strain":max_p95,"max_target_error":max_target,"max_particle_penetration":max_penetration,
        "final_table_penetration":table_penetration,"max_contacts":max_contacts,"max_scratch_array_bytes":max_scratch,
        "fold":metrics,"settle_drift":drift,"final_surface_audit":final_audit,
        "all_substeps_audit":if verify {Some(audit)} else {None},"audited_substeps":audit_steps,
        "remaining_attachments":simulation.world.attachments().count(),"remaining_pins":simulation.world.cloth(simulation.cloth)?.pins().len(),
        "finite":positions.iter().flatten().all(|x|x.is_finite())
    }))
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let mut directory = None;
    let mut repeats = 5;
    let mut verify = false;
    let mut config = Config::default();
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
                config.bend_compliance = args
                    .next()
                    .ok_or("--bend-compliance requires a value")?
                    .parse()?
            }
            "--verify" => verify = true,
            "--bench" => (),
            _ => return Err(format!("unknown argument {arg}").into()),
        }
    }
    if repeats == 0 || repeats > 100 {
        return Err("repeats must be in 1..=100".into());
    }
    let mut warmup = FoldingWorld::new(config.clone(), variant)?;
    for _ in 0..120 {
        warmup.tick()?;
    }
    let mut runs = vec![];
    for _ in 0..repeats {
        runs.push(run(config.clone(), variant, verify)?);
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
    let result = json!({"schema_version":1,"fixture_version":config.version,"precision":precision,"cpu":cpu,
        "os":output("uname",&["-a"]),"rust":output("rustc",&["--version"]),"commit":output("git",&["rev-parse","HEAD"]),
        "dirty":!output("git",&["status","--porcelain"]).is_empty(),"substeps_per_frame":4,"warmup_substeps":120,
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
