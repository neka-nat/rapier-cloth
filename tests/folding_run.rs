#[path = "../examples/support/folding.rs"]
mod folding;
#[path = "../examples/support/folding_recording.rs"]
mod folding_recording;
#[path = "../examples/support/folding_report.rs"]
mod folding_report;
#[path = "../examples/support/folding_run.rs"]
mod folding_run;
#[path = "../examples/support/folding_oracle.rs"]
mod oracle;
#[path = "../examples/support/recording.rs"]
mod recording;
use folding::*;
use folding_recording::Recorder;
use folding_run::*;
use rapier_cloth::*;

fn config() -> Config {
    let mut config = Config::with_version(2).unwrap();
    config.grid = 4;
    config
}

#[test]
fn recording_observes_both_grippers_and_preserves_runner_physics_and_audits() {
    let config = config();
    let limit = config.attach_step + 5;
    let mode = CollisionMode::default();
    let mut plain = run(config.clone(), 0, true, mode, Some(limit), |_, _, _| Ok(())).unwrap();
    let mut recorder = Recorder::default();
    let mut recorded = run(config, 0, true, mode, Some(limit), |task, report, last| {
        recorder.observe(task, report, last)
    })
    .unwrap();
    for key in ["physics_frame", "physics_samples_ms", "physics_phases"] {
        plain.as_object_mut().unwrap().remove(key);
        recorded.as_object_mut().unwrap().remove(key);
    }
    assert_eq!(plain, recorded);
    assert_eq!(recorded["stop_reason"], "step_limit");
    assert_eq!(recorded["audited_substeps"], limit);
    assert_eq!(recorded["final_targets"]["vertex_attachments"], 2);
    let finished = recorder.finish(&recorded).unwrap().unwrap();
    assert_eq!(finished.outcome.stop_reason, "step_limit");
    assert_eq!(finished.outcome.steps, limit);
    assert!(finished.outcome.steps < finished.outcome.end_step);
    assert!(finished.outcome.failure.is_none());
    let record = finished.data;
    assert_eq!(
        record.shapes.iter().map(|s| s.id).collect::<Vec<_>>(),
        [0, 1, 2]
    );
    assert_eq!(record.frames.first().unwrap().step, 0);
    assert_eq!(record.frames.last().unwrap().step, limit);
    assert!(
        record
            .frames
            .windows(2)
            .all(|pair| pair[0].step < pair[1].step && pair[0].time < pair[1].time)
    );
    assert_eq!(record.frames.len(), limit as usize / 4 + 2);
    let frame = record.frames.last().unwrap();
    assert_eq!(frame.bodies.len(), 3);
    assert_eq!(frame.attached_particles.len(), 8);
    for (&i, anchor) in frame.attached_particles.iter().zip(&frame.anchors) {
        assert!(
            Vec3::from_array(frame.positions[i as usize]).distance(Vec3::from_array(*anchor))
                < real(1e-6)
        );
    }
    assert_eq!(
        serde_json::to_value(record.config).unwrap(),
        recorded["config"]
    );
}

#[test]
fn solver_failure_is_reported_with_one_initial_record_and_no_phantom_advance() {
    let mode = CollisionMode::default();
    let mut task = configured_world(config(), 0, mode).unwrap();
    task.world.solver_settings.max_contacts = 1;
    let original = task.positions();
    let mut recorder = Recorder::default();
    let report = run_world(task, true, mode, None, |task, report, last| {
        recorder.observe(task, report, last)
    })
    .unwrap();
    assert_eq!(report["stop_reason"], "solver_error");
    assert_eq!(report["steps"], 0);
    assert_eq!(report["failure_next_step"], 1);
    assert_eq!(report["audited_substeps"], 0);
    assert!(report["failure"].as_str().unwrap().contains("budget"));
    let finished = recorder.finish(&report).unwrap().unwrap();
    assert_eq!(finished.outcome.stop_reason, "solver_error");
    assert_eq!(finished.outcome.steps, 0);
    assert_eq!(
        finished.outcome.failure.as_deref(),
        report["failure"].as_str()
    );
    let record = finished.data;
    assert_eq!(record.frames.len(), 1);
    assert_eq!(record.frames[0].step, 0);
    assert_eq!(
        record.frames[0]
            .positions
            .iter()
            .copied()
            .map(Vec3::from_array)
            .map(point)
            .collect::<Vec<_>>(),
        original
    );
}

#[test]
fn recording_rejects_unrepresentable_surface_grasps_and_invalid_run_limits() {
    let mut task = configured_world(config(), 0, CollisionMode::default()).unwrap();
    task.world
        .grasp_surface(
            ClothSurfacePoint {
                cloth: task.cloth,
                point: SurfacePoint::new(0, [0.3, 0.3, 0.4]).unwrap(),
            },
            GraspOptions::new(task.grippers[0]),
            &task.rigid.bodies,
            &task.rigid.colliders,
        )
        .unwrap();
    assert!(Recorder::default().observe(&task, None, false).is_err());
    for limit in [0, task.config.end_step + 1] {
        assert!(
            run(
                config(),
                0,
                true,
                CollisionMode::default(),
                Some(limit),
                |_, _, _| panic!("invalid run observed")
            )
            .is_err()
        );
    }
}

#[test]
fn completed_legacy_task_records_release_and_all_final_settling_samples() {
    // Small lifecycle fixture only; this does not qualify surface folding.
    let mut recorder = Recorder::default();
    let report = run(
        config(),
        0,
        true,
        CollisionMode::default(),
        None,
        |t, r, last| recorder.observe(t, r, last),
    )
    .unwrap();
    assert_eq!(report["stop_reason"], "completed");
    assert_eq!(report["steps"], 3600);
    assert_eq!(report["final_targets"]["vertex_attachments"], 0);
    assert_eq!(report["final_targets"]["surface_attachments"], 0);
    let finished = recorder.finish(&report).unwrap().unwrap();
    assert_eq!(finished.outcome.steps, finished.outcome.end_step);
    assert!(finished.outcome.failure.is_none());
    assert_eq!(finished.data.frames.len(), 901);
    assert!(finished.data.frames.iter().any(|f| f.anchors.len() == 8));
    assert!(
        finished
            .data
            .frames
            .iter()
            .filter(|f| f.step >= 2280)
            .all(|f| {
                f.attached_particles.is_empty()
                    && f.anchors.is_empty()
                    && f.pinned_particles.is_empty()
            })
    );
}
