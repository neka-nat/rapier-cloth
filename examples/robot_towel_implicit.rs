//! Rapier end-effector command example using the same task as the implicit live scene.
#[path = "support/implicit_fixture.rs"]
mod implicit_fixture;
#[path = "support/implicit_robot.rs"]
mod implicit_robot;
use implicit_robot::RobotTowel;
use rapier_cloth::{ImplicitExecution, Real};
use serde_json::json;
use std::{
    fs::OpenOptions,
    io::{BufWriter, Write},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let mut variant = String::from("nominal");
    let mut output = None;
    let mut workers = 4;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--case" => variant = args.next().ok_or("--case requires a name")?,
            "--output" => output = Some(args.next().ok_or("--output requires a new path")?),
            "--workers" => {
                workers = args
                    .next()
                    .ok_or("--workers requires 1 or 4")?
                    .parse::<usize>()?;
                if ![1, 4].contains(&workers) {
                    return Err("--workers requires 1 or 4".into());
                }
            }
            "--help" => {
                println!("robot_towel_implicit [--case NAME] [--workers 1|4] [--output NEW.jsonl]");
                println!("Cases: {}", implicit_fixture::CASES.join(", "));
                return Ok(());
            }
            _ => return Err(format!("unknown argument {arg}").into()),
        }
    }
    let execution = if workers == 4 {
        ImplicitExecution::Parallel4
    } else {
        ImplicitExecution::Serial
    };
    let mut task = RobotTowel::new(&variant, execution)?;
    let writer: Box<dyn Write> = if let Some(path) = output {
        Box::new(OpenOptions::new().write(true).create_new(true).open(path)?)
    } else {
        Box::new(std::io::sink())
    };
    let mut out = BufWriter::new(writer);
    writeln!(
        out,
        "{}",
        json!({"kind":"config","solver":"implicit","control":"rapier_end_effectors","case":variant,"workers":workers,
        "h":task.input.h,"thickness":task.input.thickness,"x":task.input.x,"faces":task.input.faces,"grasp":task.input.grasp})
    )?;
    let mut window = vec![];
    while !task.finished() {
        if let Err(error) = task.tick() {
            writeln!(
                out,
                "{}",
                json!({"kind":"failure","step":task.step+1,"accepted_step":task.step,"committed":false,"error":error})
            )?;
            out.flush()?;
            return Err(error.into());
        }
        let x = task.positions();
        if task.step as Real * task.input.h >= 7.5 - 1e-9 {
            window.push(x.clone());
        }
        let poses = [task.pose(0), task.pose(1)].map(
            |p| json!({"translation":p.translation.to_array(),"rotation":p.rotation.to_array()}),
        );
        writeln!(
            out,
            "{}",
            json!({"kind":"step","sample":task.sample(),"x":x.iter().map(|p|p.to_array()).collect::<Vec<_>>(),
            "v":task.world.cloth(task.cloth)?.velocities().iter().map(|v|v.to_array()).collect::<Vec<_>>(),
            "held_particles":task.held_particles(),"grippers":poses,
            "candidate_pairs":task.report.surface_collision.candidate_pairs,"ccd_checks":task.report.surface_collision.ccd_checks})
        )?;
        out.flush()?;
    }
    let x = task.positions();
    let drift = x
        .iter()
        .zip(&window[0])
        .map(|(a, b)| a.distance(*b))
        .fold(0.0, Real::max);
    let settled = task
        .samples
        .iter()
        .filter(|s| s.time >= 7.5 - 1e-9)
        .all(|s| s.held_vertices == 0 && s.rms_speed < 0.001 && s.max_speed < 0.005)
        && drift < 0.001;
    let fold = (x
        .iter()
        .zip(&task.input.x)
        .map(|(p, r)| (p.x - r[0]).powi(2) + (p.z + r[2].abs()).powi(2))
        .sum::<Real>()
        / x.len() as Real)
        .sqrt();
    let extension = task
        .samples
        .iter()
        .map(|s| s.max_edge_extension)
        .fold(0.0, Real::max);
    let mut times: Vec<_> = task.samples.iter().map(|s| s.physics_ms).collect();
    times.sort_by(f64::total_cmp);
    let summary = json!({"kind":"completed","steps":task.step,"settled":settled,"final_window_drift":drift,
        "planar_fold_rms_error":fold,"max_edge_extension":extension,"simulation_wall_seconds":times.iter().sum::<f64>()/1000.0,
        "p95_step_ms":times[(times.len()-1)*95/100],"max_step_ms":times.last(),"held_vertices":task.held_particles().len()});
    writeln!(out, "{summary}")?;
    out.flush()?;
    println!("{summary}");
    if !settled || fold > 0.03 || extension > 0.03 {
        return Err("robot towel did not meet the task checks".into());
    }
    Ok(())
}
