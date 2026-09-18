//! Run the retained 32x32 towel commands through the experimental shell solver.
//! See docs/implicit.md for physical parameters, scope and reproduction.
use rapier_cloth::rapier::prelude::*;
use rapier_cloth::*;
#[path = "support/implicit_fixture.rs"]
mod implicit_fixture;
use implicit_fixture::Input;
use serde_json::json;
use std::{
    fs,
    io::{BufWriter, Write},
    time::Instant,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let mut h = 0.1;
    let mut execution = ImplicitExecution::Serial;
    let mut output = None;
    let mut variant = String::from("nominal");
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--case" => variant = args.next().ok_or("--case requires a name")?,
            "--dt" => h = args.next().ok_or("--dt requires 0.04 or 0.1")?.parse()?,
            "--workers" => {
                execution = match args.next().as_deref() {
                    Some("1") => ImplicitExecution::Serial,
                    Some("4") => ImplicitExecution::Parallel4,
                    _ => return Err("--workers requires 1 or 4".into()),
                };
            }
            "--output" => output = Some(args.next().ok_or("--output requires a new file path")?),
            "--help" => {
                println!(
                    "fold_towel_implicit [--dt 0.04|0.1] [--workers 1|4] [--case NAME] [--output NEW_RECORDING.jsonl]"
                );
                println!("Cases: {}", implicit_fixture::CASES.join(", "));
                return Ok(());
            }
            _ => return Err(format!("unknown option {arg}").into()),
        }
    }
    let input = Input::load(h, &variant)?;
    let friction = Input::friction(&variant);
    let writer: Box<dyn Write> = if let Some(path) = output {
        Box::new(
            fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(path)?,
        )
    } else {
        Box::new(std::io::sink())
    };
    let mut out = BufWriter::new(writer);
    let id = WorldId::new();
    let mut rigid = PhysicsWorld::new();
    rigid.integration_parameters.dt = input.h;
    rigid.gravity = Vec3::new(0.0, -9.81, 0.0);
    // The reference floor is a finite-thickness shell. Its top surface is half
    // the thickness above its y=0 midsurface; cloth supplies the other half.
    rigid.colliders.insert(
        ColliderBuilder::new(SharedShape::halfspace(Vec3::Y))
            .translation(Vec3::Y * (input.thickness * 0.5))
            .friction(friction),
    );
    let mut world = RapierClothWorld::new(id);
    world.solver_settings.max_substep = input.h;
    let mesh = ClothMesh::new(
        input.x.iter().copied().map(Vec3::from_array).collect(),
        input.faces.clone(),
    )?;
    let mut cloth = Cloth::new(
        mesh,
        ClothMaterial {
            surface_density: 0.1503,
            damping: 0.0,
            friction,
            contact_radius: input.thickness * 0.5,
            ..Default::default()
        },
    )?;
    cloth.set_contact_settings(Some(ClothContactSettings {
        thickness: input.thickness,
        activation_margin: input.thickness,
        static_friction: friction,
        kinetic_friction: friction,
        self_collision: true,
        continuous_self_collision: true,
        rigid_surface_collision: true,
        continuous_rigid_collision: true,
        limits: CollisionLimits {
            candidate_pairs: 10_000_000,
            ccd_checks: 10_000_000,
            ..Default::default()
        },
    }))?;
    cloth.set_implicit_solver(Some(ImplicitSettings {
        execution,
        ..Default::default()
    }))?;
    let handle = world.add_cloth(cloth);
    writeln!(
        out,
        "{}",
        json!({"kind":"config","solver":"implicit","h":input.h,"substeps":1,"case":variant,"friction":friction,
        "workers":match execution { ImplicitExecution::Serial => 1, ImplicitExecution::Parallel4 => 4 },
        "youngs_modulus":821000.0,"poisson_ratio":0.243,"thickness":input.thickness,
        "density":0.1503,"damping":0.0,"barrier_stiffness":30.0,"friction_velocity":0.001,
        "newton_iterations":80,"velocity_tolerance":0.001,"candidate_pair_limit":10000000,
        "ccd_check_limit":10000000,"x":input.x,"masses":world.cloth(handle)?.masses()})
    )?;
    out.flush()?;
    let started = Instant::now();
    let mut times = Vec::new();
    let mut maximum_extension: f64 = 0.0;
    let mut window = Vec::new();
    let duration = input.targets.len() as f64 * input.h;
    for step in 0..input.targets.len() {
        let start = Instant::now();
        let checkpoint = world.checkpoint()?;
        let cloth = world.cloth_mut(handle)?;
        for (selected, &particle) in input.grasp.iter().enumerate() {
            if let Some(p) = input.target(step, selected, &variant) {
                cloth.pin(particle, Vec3::from_array(p))?;
            } else {
                cloth.unpin(particle)?;
            }
        }
        let previous = SceneSnapshot::capture(id, step as u64, &rigid.bodies, &rigid.colliders);
        rigid.step();
        let query = rigid.broad_phase.as_query_pipeline(
            rigid.narrow_phase.query_dispatcher(),
            &rigid.bodies,
            &rigid.colliders,
            QueryFilter::default(),
        );
        let result = world.step_substep(
            input.h,
            &RapierScene::new(query, &previous, input.h, rigid.gravity),
        );
        let seconds = start.elapsed().as_secs_f64();
        let report = match result {
            Ok(report) => report,
            Err(error) => {
                world.restore(&checkpoint)?;
                writeln!(
                    out,
                    "{}",
                    json!({"kind":"failure","step":step+1,
                "t":(step+1) as f64*input.h,"seconds":seconds,"error":format!("{error:?}"),"committed":false})
                )?;
                out.flush()?;
                return Err(error.into());
            }
        };
        times.push(seconds);
        let cloth = world.cloth(handle)?;
        let x: Vec<_> = cloth.positions().iter().map(|p| p.to_array()).collect();
        if x.iter().any(|p| p[0].abs() >= 1.0 || p[2].abs() >= 1.0) {
            return Err("cloth left the reference floor footprint".into());
        }
        let v: Vec<_> = cloth.velocities().iter().map(|v| v.to_array()).collect();
        let extension = report.cloths[0].1.max_stretch - 1.0;
        maximum_extension = maximum_extension.max(extension);
        let rms_speed = (cloth
            .velocities()
            .iter()
            .map(|v| v.length_squared())
            .sum::<f64>()
            / x.len() as f64)
            .sqrt();
        let max_speed = cloth
            .velocities()
            .iter()
            .map(|v| v.length())
            .fold(0.0, f64::max);
        let t = (step + 1) as f64 * input.h;
        if t >= duration - 0.5 - 1e-9 {
            window.push((
                cloth.positions().to_vec(),
                rms_speed,
                max_speed,
                cloth.pins().len(),
            ));
        }
        writeln!(
            out,
            "{}",
            json!({"kind":"step","step":step+1,"t":t,"seconds":seconds,
            "pins":cloth.pins().len(),"pinned_particles":cloth.pins().keys().copied().collect::<Vec<_>>(),"rms_speed":rms_speed,"max_speed":max_speed,"max_edge_extension":extension,
            "iterations":report.cloths[0].1.iterations,"candidate_pairs":report.cloths[0].1.surface_collision.candidate_pairs,
            "ccd_checks":report.cloths[0].1.surface_collision.ccd_checks,"x":x,"v":v})
        )?;
        out.flush()?;
    }
    let final_cloth = world.cloth(handle)?;
    let drift = final_cloth
        .positions()
        .iter()
        .zip(&window[0].0)
        .map(|(a, b)| a.distance(*b))
        .fold(0.0, f64::max);
    let settled = window
        .iter()
        .all(|(_, rms, max, pins)| *rms < 0.001 && *max < 0.005 && *pins == 0)
        && drift < 0.001;
    let fold_rms = (final_cloth
        .positions()
        .iter()
        .zip(&input.x)
        .map(|(p, rest)| (p.x - rest[0]).powi(2) + (p.z + rest[2].abs()).powi(2))
        .sum::<f64>()
        / input.x.len() as f64)
        .sqrt();
    times.sort_by(f64::total_cmp);
    let simulation_wall: f64 = times.iter().sum();
    let summary = json!({"kind":"completed","steps":input.targets.len(),"simulated_seconds":duration,
        "simulation_wall_seconds":simulation_wall,"total_wall_seconds":started.elapsed().as_secs_f64(),
        "mean_step_seconds":simulation_wall/times.len() as f64,"p95_step_seconds":times[(times.len()-1)*95/100],
        "max_step_seconds":times[times.len()-1],"max_edge_extension":maximum_extension,
        "planar_fold_rms_error":fold_rms,"settled":settled,"final_window_drift":drift});
    writeln!(out, "{summary}")?;
    out.flush()?;
    println!("{summary}");
    if !settled || maximum_extension > 0.03 || fold_rms > 0.03 {
        return Err("completed commands did not meet the example's task checks".into());
    }
    Ok(())
}
