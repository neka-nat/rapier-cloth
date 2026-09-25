//! Shared folding execution and observations for benchmarks and headless examples.
#![allow(dead_code)]
use super::{folding::*, folding_report, oracle};
use serde_json::json;
use std::{process::Command, time::Instant};

pub fn output(command: &str, args: &[&str]) -> String {
    Command::new(command)
        .args(args)
        .output()
        .ok()
        .filter(|r| r.status.success())
        .map(|r| String::from_utf8_lossy(&r.stdout).trim().to_string())
        .unwrap_or_else(|| "unknown".into())
}
/// Report the checkout that built this example, independently of the launch cwd.
/// Extracted crates without their own checkout remain unqualified source records.
pub fn source_identity() -> (String, bool) {
    let manifest = env!("CARGO_MANIFEST_DIR");
    let root = output("git", &["-C", manifest, "rev-parse", "--show-toplevel"]);
    if std::fs::canonicalize(manifest).ok() != std::fs::canonicalize(&root).ok()
        || root == "unknown"
    {
        return ("unknown".into(), true);
    }
    (
        output("git", &["-C", manifest, "rev-parse", "HEAD"]),
        !output("git", &["-C", manifest, "status", "--porcelain"]).is_empty(),
    )
}
pub fn timing(samples: &[f64]) -> serde_json::Value {
    let mut ms = samples.to_vec();
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
pub fn run(
    config: Config,
    variant: usize,
    verify: bool,
    mode: CollisionMode,
    step_limit: Option<u64>,
    observe: impl FnMut(
        &FoldingWorld,
        Option<&rapier_cloth::StepReport>,
        bool,
    ) -> Result<(), Box<dyn std::error::Error>>,
) -> Result<serde_json::Value, Box<dyn std::error::Error>> {
    run_world(
        configured_world(config, variant, mode)?,
        verify,
        mode,
        step_limit,
        observe,
    )
}

pub fn run_world(
    mut simulation: FoldingWorld,
    verify: bool,
    mode: CollisionMode,
    step_limit: Option<u64>,
    mut observe: impl FnMut(
        &FoldingWorld,
        Option<&rapier_cloth::StepReport>,
        bool,
    ) -> Result<(), Box<dyn std::error::Error>>,
) -> Result<serde_json::Value, Box<dyn std::error::Error>> {
    if simulation.step != 0 {
        return Err("run must begin at the initial task state".into());
    }
    let limit = step_limit.unwrap_or(simulation.config.end_step);
    if limit == 0 || limit > simulation.config.end_step {
        return Err("step limit must be between 1 and the task end step".into());
    }
    observe(&simulation, None, false)?;
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
    let initial_audit = verify.then(|| {
        oracle::audit_surface(
            &simulation.positions(),
            &triangles,
            simulation.config.thickness,
        )
    });
    let mut frames = vec![];
    let mut frame_ms = 0.0;
    let mut frame_by_phase = std::collections::BTreeMap::<String, Vec<f64>>::new();
    let mut max_strain: rapier_cloth::Real = 0.0;
    let mut max_p95: rapier_cloth::Real = 0.0;
    let mut max_target: rapier_cloth::Real = 0.0;
    let mut max_penetration: rapier_cloth::Real = 0.0;
    let mut diagnostics_finite = true;
    let mut max_contacts = 0;
    let mut max_scratch = 0;
    let mut collision_work = rapier_cloth::CollisionWork::default();
    let mut error = None;
    let mut task_audit = folding_report::TaskAudit::default();
    let mut audit = oracle::SurfaceAudit::default();
    let mut audit_steps = 0;
    while simulation.step < limit {
        let begin = Instant::now();
        let result = simulation.tick();
        frame_ms += begin.elapsed().as_secs_f64() * 1000.0;
        match result {
            Ok(report) => {
                let r = &report.cloths[0].1;
                diagnostics_finite &= [
                    r.max_stretch,
                    r.p95_stretch,
                    r.max_target_error,
                    r.max_penetration,
                ]
                .iter()
                .all(|v| v.is_finite());
                max_strain = max_strain.max(r.max_stretch);
                max_p95 = max_p95.max(r.p95_stretch);
                max_target = max_target.max(r.max_target_error);
                max_penetration = max_penetration.max(r.max_penetration);
                max_contacts = max_contacts.max(r.contacts);
                max_scratch = max_scratch.max(r.scratch_bytes);
                collision_work.candidate_pairs = collision_work
                    .candidate_pairs
                    .max(r.surface_collision.candidate_pairs);
                collision_work.retained_contacts = collision_work
                    .retained_contacts
                    .max(r.surface_collision.retained_contacts);
                collision_work.ccd_checks = collision_work
                    .ccd_checks
                    .max(r.surface_collision.ccd_checks);
                collision_work.limited_advances = collision_work
                    .limited_advances
                    .max(r.surface_collision.limited_advances);
                observe(&simulation, Some(r), false)?;
            }
            Err(e) => {
                error = Some(e.to_string());
                break;
            }
        }
        if simulation.step.is_multiple_of(4) {
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
        task_audit.observe(&simulation)?;
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
    }
    observe(&simulation, None, true)?;
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
    let final_targets = folding_report::target_counts(&simulation)?;
    let mass: rapier_cloth::Real = simulation
        .world
        .cloth(simulation.cloth)?
        .masses()
        .iter()
        .sum();
    Ok(json!({
        "status":if mode.continuous_rigid {"unqualified_continuous_rigid_collision"} else if mode.rigid_surface {"unqualified_rigid_surface_collision"} else if mode.continuous_self {"unqualified_continuous_self_collision"} else if mode.self_collision {"unqualified_discrete_self_collision"} else {"unqualified_particle_baseline"},
        "collision_settings":simulation.world.cloth(simulation.cloth)?.contact_settings().map(|s|json!({
            "thickness":s.thickness,"activation_margin":s.activation_margin,"self_collision":s.self_collision,
            "continuous_self_collision":s.continuous_self_collision,"ccd_minimum_separation":if s.continuous_self_collision {Some(s.thickness*0.9)} else {None},
            "rigid_surface_collision":s.rigid_surface_collision,"rigid_separation":if s.rigid_surface_collision {Some(s.thickness*0.5)} else {None},
            "continuous_rigid_collision":s.continuous_rigid_collision,"rigid_ccd_minimum_separation":if s.continuous_rigid_collision {Some(s.thickness*0.45)} else {None},
            "static_friction":s.static_friction,"kinetic_friction":s.kinetic_friction,
            "limits":{"candidate_pairs":s.limits.candidate_pairs,"retained_contacts":s.limits.retained_contacts,"ccd_checks":s.limits.ccd_checks}})),
        "collision_work_max_per_substep":{"candidate_pairs":collision_work.candidate_pairs,"retained_contacts":collision_work.retained_contacts,"ccd_checks":collision_work.ccd_checks,"limited_advances":collision_work.limited_advances},
        "config":simulation.config,"variant":simulation.variant,"verification_enabled":verify,
        "physics_settings":{"h":simulation.rigid.integration_parameters.dt,"iterations":simulation.world.solver_settings.iterations,
            "friction_model":if mode.self_collision || mode.rigid_surface {"persistent_material_coordinate"} else {"legacy_particle_kinetic"}},
        "mesh":{"vertices":positions.len(),"triangles":triangles.len(),"total_mass":mass},
        "stop_reason":if error.is_some() {"solver_error"} else if simulation.step < simulation.config.end_step {"step_limit"} else {"completed"},
        "steps":simulation.step,"failure":error,"failure_next_step":error.as_ref().map(|_|simulation.step+1),
        "physics_frame":timing(&frames),"physics_samples_ms":frames,"physics_phases":frame_by_phase.into_iter().map(|(k,v)|(k,timing(&v))).collect::<std::collections::BTreeMap<_,_>>(),
        "max_strain":max_strain,"max_p95_strain":max_p95,"max_target_error":max_target,"max_particle_penetration":max_penetration,
        "final_table_penetration":table_penetration,"max_contacts":max_contacts,"max_scratch_array_bytes":max_scratch,
        "fold":metrics,"settle_drift":task_audit.settle_drift,"task_audit":task_audit,"initial_surface_audit":initial_audit,"final_surface_audit":final_audit,
        "all_substeps_audit":if verify {Some(audit)} else {None},"audited_substeps":audit_steps,
        "remaining_attachments":simulation.world.attachments().count(),"remaining_pins":simulation.world.cloth(simulation.cloth)?.pins().len(),
        "final_targets":final_targets,
        "finite":positions.iter().flatten().all(|x|x.is_finite()),"diagnostics_finite":diagnostics_finite
    }))
}
