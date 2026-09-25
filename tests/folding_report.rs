#[path = "../examples/support/folding.rs"]
mod folding;
#[path = "../examples/support/folding_report.rs"]
mod folding_report;
use folding::*;
use folding_report::*;
use rapier_cloth::*;

fn task() -> FoldingWorld {
    let mut config = Config::with_version(2).unwrap();
    config.grid = 4;
    FoldingWorld::new(config, 0).unwrap()
}

#[test]
fn release_observations_count_both_attachment_kinds_pins_and_stale_commands() {
    let mut task = task();
    let vertex = task
        .world
        .attach(
            AttachmentDesc {
                cloth: task.cloth,
                body: task.grippers[0],
                points: vec![AttachmentPoint {
                    particle: 0,
                    local_anchor: Vec3::ZERO,
                }],
                compliance: 0.0,
                excluded_colliders: vec![],
            },
            &task.rigid.bodies,
            &task.rigid.colliders,
        )
        .unwrap();
    let surface = task
        .world
        .grasp_surface(
            ClothSurfacePoint {
                cloth: task.cloth,
                point: SurfacePoint::new(17, [0.2, 0.3, 0.5]).unwrap(),
            },
            GraspOptions::new(task.grippers[1]),
            &task.rigid.bodies,
            &task.rigid.colliders,
        )
        .unwrap();
    task.world
        .cloth_mut(task.cloth)
        .unwrap()
        .pin(3, Vec3::Y)
        .unwrap();
    task.attachments = [Some(vertex), Some(surface)];
    assert_eq!(
        target_counts(&task).unwrap(),
        TargetCounts {
            vertex_attachments: 1,
            surface_attachments: 1,
            pins: 1,
            target_points: 3,
            commanded_grasps: 2,
        }
    );
    task.step = task.config.release_step;
    let mut audit = TaskAudit::default();
    audit.observe(&task).unwrap();
    assert_eq!(audit.released_substeps_without_targets, 0);
    task.world.release(vertex).unwrap();
    task.world.release(surface).unwrap();
    task.world.cloth_mut(task.cloth).unwrap().unpin(3).unwrap();
    // A dropped world attachment does not erase the task's command handle.
    task.step += 1;
    audit.observe(&task).unwrap();
    assert_eq!(audit.released_substeps_without_targets, 0);
    assert_eq!(target_counts(&task).unwrap().commanded_grasps, 2);
    task.attachments = [None; 2];
    task.step += 1;
    audit.observe(&task).unwrap();
    assert_eq!(audit.released_substeps, 3);
    assert_eq!(audit.released_substeps_without_targets, 1);
    assert_eq!(target_counts(&task).unwrap(), TargetCounts::default());
}

#[test]
fn observations_retain_transient_penetration_and_drift_with_explicit_settle_coverage() {
    let mut task = task();
    let original = task.world.cloth(task.cloth).unwrap().positions().to_vec();
    let mut moved = original.clone();
    moved[0].y = real(-0.002);
    task.world
        .cloth_mut(task.cloth)
        .unwrap()
        .set_positions(&moved)
        .unwrap();
    let mut audit = TaskAudit::default();
    task.step = task.config.retract_end - 1;
    audit.observe(&task).unwrap();
    assert_eq!(audit.settle_samples, 0);
    assert_eq!(audit.settle_start_step, None);
    task.world
        .cloth_mut(task.cloth)
        .unwrap()
        .set_positions(&original)
        .unwrap();
    task.step += 1;
    audit.observe(&task).unwrap();
    assert_eq!(audit.settle_samples, 1);
    // Known translated snapshots exercise the observer, not a physics replay.
    for (p, initial) in moved.iter_mut().zip(&original) {
        *p = *initial + Vec3::X * real(0.003);
    }
    task.world
        .cloth_mut(task.cloth)
        .unwrap()
        .set_positions(&moved)
        .unwrap();
    for step in task.config.retract_end + 1..task.config.end_step {
        task.step = step;
        audit.observe(&task).unwrap();
    }
    task.world
        .cloth_mut(task.cloth)
        .unwrap()
        .set_positions(&original)
        .unwrap();
    task.step = task.config.end_step;
    audit.observe(&task).unwrap();
    assert_eq!(audit.settle_samples, 1201);
    assert_eq!(audit.settle_start_step, Some(task.config.retract_end));
    assert_eq!(audit.settle_end_step, Some(task.config.end_step));
    assert!((audit.settle_drift - 0.003).abs() < 1e-7);
    assert!((audit.max_table_penetration - 0.0025).abs() < 1e-9);
    assert!(audit.all_substeps_finite);
}

#[test]
fn grasp_coverage_requires_both_four_point_patches_during_the_grasp_interval() {
    let mut task = task();
    for g in 0..2 {
        let points = task
            .corner_patch(g)
            .map(|particle| AttachmentPoint {
                particle,
                local_anchor: Vec3::ZERO,
            })
            .to_vec();
        task.attachments[g] = Some(
            task.world
                .attach(
                    AttachmentDesc {
                        cloth: task.cloth,
                        body: task.grippers[g],
                        points,
                        compliance: 0.0,
                        excluded_colliders: vec![],
                    },
                    &task.rigid.bodies,
                    &task.rigid.colliders,
                )
                .unwrap(),
        );
    }
    let mut audit = TaskAudit::default();
    task.step = task.config.attach_step - 1;
    audit.observe(&task).unwrap();
    assert_eq!(audit.grasped_substeps, 0);
    task.step += 1;
    audit.observe(&task).unwrap();
    assert_eq!(audit.grasped_substeps, 1);
    task.world
        .release(task.attachments[1].take().unwrap())
        .unwrap();
    task.step += 1;
    audit.observe(&task).unwrap();
    assert_eq!(audit.grasped_substeps, 1);
}
