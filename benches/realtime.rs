//! Four actual sequential physics substeps per frame, including Rapier and
//! scene snapshots. Rendering, pacing, and serialization are excluded.
use rapier::prelude::*;
use rapier_cloth::{Real, *};
use serde::Serialize;
use std::{fs, process::Command, time::Instant};

#[derive(Serialize)]
struct Timing {
    p50_ms: f64,
    p95_ms: f64,
    max_ms: f64,
}
fn timing(mut values: Vec<f64>) -> Timing {
    values.sort_by(f64::total_cmp);
    Timing {
        p50_ms: values[(values.len() / 2).saturating_sub(1)] * 1000.0,
        p95_ms: values[(values.len() as f64 * 0.95).ceil() as usize - 1] * 1000.0,
        max_ms: values.last().unwrap() * 1000.0,
    }
}
#[derive(Serialize)]
struct Sample {
    fixture: &'static str,
    core_substep: Timing,
    query_substep: Timing,
    cloth_substep: Timing,
    physics_frame: Timing,
    frames_over_budget: usize,
    max_penetration: Real,
    max_stretch: Real,
    max_p95_stretch: Real,
    max_target_error: Real,
    max_contacts: usize,
    final_corner: [Real; 3],
}
#[derive(Serialize)]
struct Benchmark {
    schema_version: u32,
    cpu: String,
    os: String,
    rust: String,
    commit: String,
    dirty: bool,
    precision: &'static str,
    grid: usize,
    cloths: usize,
    h: Real,
    iterations: usize,
    warmup_substeps: usize,
    measured_substeps: usize,
    substeps_per_frame: usize,
    frame_budget_ms: f64,
    samples: Vec<Sample>,
}
fn output(command: &str, args: &[&str]) -> String {
    Command::new(command)
        .args(args)
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().into())
        .unwrap_or_else(|| "unknown".into())
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let mut directory = None;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--output" => directory = Some(args.next().ok_or("--output needs a directory")?),
            "--bench" => (),
            _ => return Err(format!("unknown argument {arg}").into()),
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
        precision: if cfg!(feature = "f64") { "f64" } else { "f32" },
        grid: 32,
        cloths: 1,
        h: 1.0 / 240.0,
        iterations: 8,
        warmup_substeps: 100,
        measured_substeps: 1000,
        substeps_per_frame: 4,
        frame_budget_ms: 1000.0 / 60.0,
        samples: vec![],
    };
    for fixture in ["flat-halfspace", "hanging", "moving-sphere"] {
        let id = WorldId::new();
        let mut rigid = PhysicsWorld::new();
        rigid.integration_parameters.dt = result.h;
        let mut sphere = None;
        let origin = match fixture {
            "flat-halfspace" => {
                rigid
                    .colliders
                    .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)));
                Vec3::Y * 0.005
            }
            "hanging" => Vec3::Y,
            _ => {
                let body = rigid.bodies.insert(
                    RigidBodyBuilder::kinematic_position_based()
                        .translation(Vec3::new(0.5, 0.25, 0.5)),
                );
                rigid.colliders.insert_with_parent(
                    ColliderBuilder::ball(0.3),
                    body,
                    &mut rigid.bodies,
                );
                rigid
                    .colliders
                    .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)));
                sphere = Some(body);
                Vec3::Y * 0.56
            }
        };
        let mut cloth = Cloth::new(
            GridBuilder::new(32, 32).origin(origin).build()?,
            ClothMaterial::default(),
        )?;
        if fixture == "hanging" {
            for i in 0..32 {
                cloth.pin(i, cloth.positions()[i as usize])?;
            }
        }
        let mut world = RapierClothWorld::new(id);
        let handle = world.add_cloth(cloth);
        let mut core = vec![];
        let mut query = vec![];
        let mut total = vec![];
        let mut frames = vec![];
        let mut frame_start = Instant::now();
        let mut depth: Real = 0.0;
        let mut stretch: Real = 0.0;
        let mut p95: Real = 0.0;
        let mut target: Real = 0.0;
        let mut contacts = 0;
        for step in 0..result.warmup_substeps + result.measured_substeps {
            if step % 4 == 0 {
                frame_start = Instant::now();
            }
            let before = SceneSnapshot::capture(id, step as u64, &rigid.bodies, &rigid.colliders);
            if let Some(body) = sphere {
                let t = (step + 1) as Real * result.h;
                rigid.bodies[body].set_next_kinematic_translation(Vec3::new(
                    0.5 + 0.08 * (1.5 * t).sin(),
                    0.25,
                    0.5,
                ));
            }
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
            if step >= result.warmup_substeps {
                if step % 4 == 3 {
                    frames.push(frame_start.elapsed().as_secs_f64());
                }
                core.push(report.core_time_seconds);
                query.push(report.query_time_seconds);
                total.push(report.total_time_seconds);
                let r = &report.cloths[0].1;
                depth = depth.max(r.max_penetration);
                stretch = stretch.max(r.max_stretch);
                p95 = p95.max(r.p95_stretch);
                target = target.max(r.max_target_error);
                contacts = contacts.max(r.contacts);
            }
        }
        let sample = Sample {
            fixture,
            core_substep: timing(core),
            query_substep: timing(query),
            cloth_substep: timing(total),
            frames_over_budget: frames
                .iter()
                .filter(|&&s| s * 1000.0 > result.frame_budget_ms)
                .count(),
            physics_frame: timing(frames),
            max_penetration: depth,
            max_stretch: stretch,
            max_p95_stretch: p95,
            max_target_error: target,
            max_contacts: contacts,
            final_corner: world.cloth(handle)?.positions()[1023].to_array(),
        };
        eprintln!(
            "{} {fixture}: frame p50={:.3}ms p95={:.3}ms; missed={}/250; p95 stretch={:.5}",
            result.precision,
            sample.physics_frame.p50_ms,
            sample.physics_frame.p95_ms,
            sample.frames_over_budget,
            sample.max_p95_stretch
        );
        result.samples.push(sample);
    }
    let json = serde_json::to_string_pretty(&result)?;
    if let Some(directory) = directory {
        fs::create_dir_all(&directory)?;
        fs::write(
            format!("{directory}/realtime-{}.json", result.precision),
            &json,
        )?;
    }
    println!("{json}");
    Ok(())
}
