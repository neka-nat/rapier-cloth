use rapier::prelude::*;
use rapier_cloth::{Real, *};
use serde::Serialize;
use std::{fs, process::Command};
#[derive(Serialize)]
struct Timings {
    p50_ms: f64,
    p95_ms: f64,
}
fn percentile(mut data: Vec<f64>) -> Timings {
    data.sort_by(f64::total_cmp);
    Timings {
        p50_ms: data[(data.len() / 2).saturating_sub(1)] * 1000.0,
        p95_ms: data[((data.len() as f64 * 0.95).ceil() as usize).saturating_sub(1)] * 1000.0,
    }
}
#[derive(Serialize)]
struct Sample {
    grid: usize,
    vertices: usize,
    core: Timings,
    query: Timings,
    total: Timings,
    max_penetration: Real,
    max_stretch: Real,
    max_p95_stretch: Real,
    max_contacts: usize,
    scratch_array_bytes: usize,
}
#[derive(Serialize)]
struct Benchmark {
    schema_version: u32,
    cpu: String,
    os: String,
    rust: String,
    commit: String,
    dirty: bool,
    precision: String,
    h: Real,
    iterations: usize,
    warmup: usize,
    measured: usize,
    fixture: String,
    samples: Vec<Sample>,
}
fn output(command: &str, args: &[&str]) -> String {
    Command::new(command)
        .args(args)
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .unwrap_or_else(|| "unknown".into())
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let mut output_path = None;
    while let Some(arg) = args.next() {
        if arg == "--output" {
            output_path = Some(args.next().ok_or("--output needs a directory")?);
        } else if arg != "--bench" {
            return Err(format!("unknown argument {arg}").into());
        }
    }
    let cpu = fs::read_to_string("/proc/cpuinfo")
        .ok()
        .and_then(|s| {
            s.lines()
                .find(|l| l.starts_with("model name"))
                .and_then(|l| l.split_once(':'))
                .map(|(_, v)| v.trim().into())
        })
        .unwrap_or_else(|| {
            std::env::var("PROCESSOR_IDENTIFIER")
                .unwrap_or_else(|_| output("sysctl", &["-n", "machdep.cpu.brand_string"]))
        });
    let mut result = Benchmark {
        schema_version: 1,
        cpu,
        os: format!("{} {}", std::env::consts::OS, output("uname", &["-r"])),
        rust: output("rustc", &["--version"]),
        commit: output("git", &["rev-parse", "HEAD"]),
        dirty: !output("git", &["status", "--porcelain"]).is_empty(),
        precision: if cfg!(feature = "f64") { "f64" } else { "f32" }.into(),
        h: 1.0 / 240.0,
        iterations: 8,
        warmup: 100,
        measured: 1000,
        fixture:
            "1m square flat on fixed halfspace; default ClothMaterial; no pins; gravity=(0,-9.81,0)"
                .into(),
        samples: vec![],
    };
    for grid in [32, 64, 128] {
        let id = WorldId::new();
        let mut rigid = PhysicsWorld::new();
        rigid.integration_parameters.dt = result.h;
        rigid
            .colliders
            .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)));
        let mut world = RapierClothWorld::new(id);
        let cloth = world.add_cloth(Cloth::new(
            GridBuilder::new(grid, grid)
                .size(1.0, 1.0)
                .origin(Vec3::Y * 0.005)
                .build()?,
            ClothMaterial::default(),
        )?);
        let mut core = vec![];
        let mut query = vec![];
        let mut total = vec![];
        let mut depth: Real = 0.0;
        let mut stretch: Real = 0.0;
        let mut p95: Real = 0.0;
        let mut contacts = 0;
        let mut scratch = 0;
        for step in 0..result.warmup + result.measured {
            let before = SceneSnapshot::capture(id, step as u64, &rigid.bodies, &rigid.colliders);
            rigid.step();
            let q = rigid.broad_phase.as_query_pipeline(
                rigid.narrow_phase.query_dispatcher(),
                &rigid.bodies,
                &rigid.colliders,
                QueryFilter::default(),
            );
            let report = world.step_substep(
                result.h,
                &RapierScene::new(q, &before, result.h, rigid.gravity),
            )?;
            if step >= result.warmup {
                core.push(report.core_time_seconds);
                query.push(report.query_time_seconds);
                total.push(report.total_time_seconds);
                let r = &report.cloths[0].1;
                depth = depth.max(r.max_penetration);
                stretch = stretch.max(r.max_stretch);
                p95 = p95.max(r.p95_stretch);
                contacts = contacts.max(r.contacts);
                scratch = scratch.max(r.scratch_bytes);
            }
        }
        let sample = Sample {
            grid,
            vertices: world.cloth(cloth)?.positions().len(),
            core: percentile(core),
            query: percentile(query),
            total: percentile(total),
            max_penetration: depth,
            max_stretch: stretch,
            max_p95_stretch: p95,
            max_contacts: contacts,
            scratch_array_bytes: scratch,
        };
        eprintln!(
            "{} {}x{}: total p50={:.3}ms p95={:.3}ms; stretch={:.5} penetration={:.6}",
            result.precision,
            grid,
            grid,
            sample.total.p50_ms,
            sample.total.p95_ms,
            sample.max_p95_stretch,
            sample.max_penetration
        );
        result.samples.push(sample);
    }
    let json = serde_json::to_string_pretty(&result)?;
    if let Some(directory) = output_path {
        fs::create_dir_all(&directory)?;
        let stem = format!("{directory}/cloth-scaling-{}", result.precision);
        fs::write(format!("{stem}.json"), &json)?;
        let mut csv = String::from(
            "precision,grid,vertices,core_p50_ms,core_p95_ms,query_p50_ms,query_p95_ms,total_p50_ms,total_p95_ms,max_penetration,max_stretch,max_p95_stretch,max_contacts,scratch_array_bytes\n",
        );
        for s in &result.samples {
            csv += &format!(
                "{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n",
                result.precision,
                s.grid,
                s.vertices,
                s.core.p50_ms,
                s.core.p95_ms,
                s.query.p50_ms,
                s.query.p95_ms,
                s.total.p50_ms,
                s.total.p95_ms,
                s.max_penetration,
                s.max_stretch,
                s.max_p95_stretch,
                s.max_contacts,
                s.scratch_array_bytes
            );
        }
        fs::write(format!("{stem}.csv"), csv)?;
    }
    println!("{json}");
    Ok(())
}
