#[path = "../examples/support/folding.rs"]
mod folding;
use folding::*;
use rapier_cloth::{rapier::prelude::RigidBodyType, *};

// A small lifecycle fixture; this does not qualify the 32 x 32 folding task.
fn task() -> FoldingWorld {
    let mut config = Config::with_version(2).unwrap();
    config.grid = 4;
    let mut task = FoldingWorld::new(config, 0).unwrap();
    task.world
        .cloth_mut(task.cloth)
        .unwrap()
        .set_contact_settings(Some(ClothContactSettings {
            thickness: real(task.config.thickness),
            activation_margin: real(task.config.activation_margin),
            static_friction: real(task.config.static_friction),
            kinetic_friction: real(task.config.kinetic_friction),
            rigid_surface_collision: true,
            ..Default::default()
        }))
        .unwrap();
    task
}

fn advance_to(task: &mut FoldingWorld, step: u64) {
    while task.step < step {
        task.tick().unwrap();
    }
}

fn assert_same_physical_state(a: &FoldingWorld, b: &FoldingWorld) {
    assert_eq!(a.step, b.step);
    assert_eq!(a.world.next_step_index(), b.world.next_step_index());
    assert!(!a.world.is_desynchronized());
    assert_eq!(a.positions(), b.positions());
    let ac = a.world.cloth(a.cloth).unwrap();
    let bc = b.world.cloth(b.cloth).unwrap();
    assert_eq!(ac.velocities(), bc.velocities());
    assert_eq!(ac.contact_history_len(), bc.contact_history_len());
    assert_eq!(a.world.attachments().count(), b.world.attachments().count());
    assert_eq!(
        a.attachments.map(|x| x.is_some()),
        b.attachments.map(|x| x.is_some())
    );
    for g in 0..2 {
        let ab = &a.rigid.bodies[a.grippers[g]];
        let bb = &b.rigid.bodies[b.grippers[g]];
        assert_eq!(ab.position(), bb.position());
        assert_eq!(ab.next_position(), bb.next_position());
        assert_eq!(ab.linvel(), bb.linvel());
        assert_eq!(ab.angvel(), bb.angvel());
        assert_eq!(
            a.rigid.colliders[a.gripper_colliders[g]].position(),
            b.rigid.colliders[b.gripper_colliders[g]].position()
        );
    }
}

#[test]
fn failed_grasp_substep_restores_history_rigid_motion_and_task_events_before_retry() {
    let mut task = task();
    let mut control = self::task();
    let before_grasp = task.config.attach_step - 1;
    advance_to(&mut task, before_grasp);
    advance_to(&mut control, before_grasp);
    assert!(task.world.cloth(task.cloth).unwrap().contact_history_len() > 0);
    let old_budget = task.world.solver_settings.max_contacts;
    task.world.solver_settings.max_contacts = 1;
    let error = task.tick().unwrap_err();
    assert!(error.to_string().contains("budget"), "{error}");
    assert_same_physical_state(&task, &control);
    assert_eq!(task.attachments, [None; 2]);
    assert_eq!(task.world.drain_attachment_events().count(), 0);
    assert!(task.world.checkpoint().is_ok());
    task.world.solver_settings.max_contacts = old_budget;
    for _ in 0..16 {
        task.tick().unwrap();
        control.tick().unwrap();
        assert_same_physical_state(&task, &control);
    }
    assert_eq!(task.world.attachments().count(), 2);
    assert_eq!(
        task.world.drain_attachment_events().count(),
        control.world.drain_attachment_events().count()
    );
}

#[test]
fn second_grasp_error_retracts_the_first_grasp_and_its_history_invalidation() {
    let mut task = task();
    let before_grasp = task.config.attach_step - 1;
    advance_to(&mut task, before_grasp);
    let positions = task.positions();
    let history = task.world.cloth(task.cloth).unwrap().contact_history_len();
    let body_poses = task.grippers.map(|g| *task.rigid.bodies[g].position());
    task.rigid.bodies[task.grippers[1]].set_body_type(RigidBodyType::Dynamic, true);
    assert!(matches!(
        task.tick(),
        Err(IntegrationError::InvalidAttachment(_))
    ));
    assert_eq!(task.step, before_grasp);
    assert_eq!(task.world.next_step_index(), before_grasp);
    assert_eq!(task.positions(), positions);
    assert_eq!(
        task.world.cloth(task.cloth).unwrap().contact_history_len(),
        history
    );
    assert_eq!(
        task.grippers.map(|g| *task.rigid.bodies[g].position()),
        body_poses
    );
    assert_eq!(task.attachments, [None; 2]);
    assert_eq!(task.world.attachments().count(), 0);
    assert_eq!(task.world.drain_attachment_events().count(), 0);
    assert!(task.world.checkpoint().is_ok());
}

#[test]
fn failed_release_keeps_original_handles_and_emits_release_events_once() {
    let mut task = task();
    // Use the complete commanded trajectory with legacy contact for this
    // release-lifecycle check. Surface-folding quality has separate gates.
    task.world
        .cloth_mut(task.cloth)
        .unwrap()
        .set_contact_settings(None)
        .unwrap();
    let before_release = task.config.release_step - 1;
    advance_to(&mut task, before_release);
    let handles = task.attachments;
    let positions = task.positions();
    let poses = task.grippers.map(|g| *task.rigid.bodies[g].position());
    task.world.drain_attachment_events().for_each(drop);
    let prior_release = task
        .world
        .attach(
            AttachmentDesc {
                cloth: task.cloth,
                body: task.grippers[0],
                points: vec![AttachmentPoint {
                    particle: 12,
                    local_anchor: Vec3::ZERO,
                }],
                compliance: 0.0,
                excluded_colliders: vec![],
            },
            &task.rigid.bodies,
            &task.rigid.colliders,
        )
        .unwrap();
    task.world.release(prior_release).unwrap();
    let iterations = task.world.solver_settings.iterations;
    task.world.solver_settings.iterations = 0;
    assert!(task.tick().is_err());
    assert_eq!(task.step, before_release);
    assert_eq!(task.world.next_step_index(), before_release);
    assert_eq!(task.attachments, handles);
    assert_eq!(task.world.attachments().count(), 2);
    assert_eq!(task.positions(), positions);
    assert_eq!(
        task.grippers.map(|g| *task.rigid.bodies[g].position()),
        poses
    );
    assert_eq!(
        task.world.drain_attachment_events().collect::<Vec<_>>(),
        vec![AttachmentEvent {
            handle: prior_release,
            cloth: task.cloth,
            body: task.grippers[0],
            kind: AttachmentEventKind::Released,
        }]
    );
    for handle in handles.into_iter().flatten() {
        assert!(task.world.attachment(handle).is_ok());
    }
    task.world.solver_settings.iterations = iterations;
    task.tick().unwrap();
    assert_eq!(task.attachments, [None; 2]);
    assert_eq!(task.world.attachments().count(), 0);
    assert_eq!(task.world.drain_attachment_events().count(), 2);
    task.tick().unwrap();
    assert_eq!(task.world.drain_attachment_events().count(), 0);
}
