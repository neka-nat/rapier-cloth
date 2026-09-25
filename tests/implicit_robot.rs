#![cfg(all(feature = "f64", feature = "implicit"))]
#[path = "../examples/support/implicit_fixture.rs"]
mod implicit_fixture;
#[path = "../examples/support/implicit_options.rs"]
mod implicit_options;
#[path = "../examples/support/implicit_robot.rs"]
mod implicit_robot;
use implicit_robot::RobotTowel;
use rapier_cloth::*;

#[test]
fn end_effector_poses_reproduce_every_prescribed_patch() {
    for case in implicit_fixture::CASES {
        let task = RobotTowel::new(case, ImplicitExecution::Serial).unwrap();
        for step in 0..40 {
            for side in 0..2 {
                let pose = task.scripted_pose(step, side);
                let first = *task
                    .input
                    .grasp
                    .iter()
                    .find(|&&i| task.input.side(i) == side)
                    .unwrap();
                for (j, &particle) in task
                    .input
                    .grasp
                    .iter()
                    .enumerate()
                    .filter(|(_, p)| task.input.side(**p) == side)
                {
                    if let Some(expected) = task.input.target(step, j, case) {
                        let local = Vec3::from_array(task.input.x[particle as usize])
                            - Vec3::from_array(task.input.x[first as usize]);
                        assert!(
                            pose.transform_point(local)
                                .distance(Vec3::from_array(expected))
                                < 1e-12
                        );
                    }
                }
            }
        }
    }
}
#[test]
fn commands_are_timestamped_atomic_and_releases_preserve_motion() {
    let mut task = RobotTowel::new("nominal", ImplicitExecution::Parallel4).unwrap();
    let before = task.positions();
    let mut target = task.pose(0).translation.to_array();
    target[1] += 0.005;
    assert!(task.set_pose(0, target, [0.0, 0.0, 0.0, 1.0], 0).is_err());
    assert!(task.automatic);
    task.set_pose(0, target, [0.0, 0.0, 0.0, 1.0], 1).unwrap();
    assert_eq!(task.positions(), before);
    assert_eq!(task.step, 0);
    task.tick().unwrap();
    assert!(task.pose(0).translation.distance(Vec3::from_array(target)) < 1e-12);
    let before = task.positions();
    let velocity = task.world.cloth(task.cloth).unwrap().velocities().to_vec();
    task.release(0).unwrap();
    assert_eq!(task.held_particles().len(), 26);
    assert!(task.attachments[1].is_some());
    assert_eq!(task.positions(), before);
    assert_eq!(task.world.cloth(task.cloth).unwrap().velocities(), velocity);
    task.grasp(0).unwrap();
    assert_eq!(task.held_particles().len(), 53);
    assert_eq!(task.positions(), before);
}
#[test]
fn rejected_physics_restores_rigid_cloth_attachments_and_clock() {
    for policy in [
        implicit_options::CapPolicy::Strict,
        implicit_options::CapPolicy::Approximate,
    ] {
        let mut task =
            RobotTowel::with_policy("nominal", ImplicitExecution::Parallel4, policy).unwrap();
        let before = task.positions();
        let poses = [task.pose(0), task.pose(1)];
        let mut pending = task.pose(0).translation.to_array();
        pending[1] += 0.005;
        task.set_pose(0, pending, [0.0, 0.0, 0.0, 1.0], 1).unwrap();
        let desired = task.desired;
        let held = task.attachments;
        let original = task
            .world
            .cloth(task.cloth)
            .unwrap()
            .contact_settings()
            .unwrap();
        let mut bounded = original;
        bounded.limits.candidate_pairs = 1;
        task.world
            .cloth_mut(task.cloth)
            .unwrap()
            .set_contact_settings(Some(bounded))
            .unwrap();
        assert!(task.tick().is_err());
        assert_eq!(task.step, 0);
        assert_eq!(task.world.next_step_index(), 0);
        assert_eq!(task.positions(), before);
        assert_eq!([task.pose(0), task.pose(1)], poses);
        assert_eq!(task.desired, desired);
        assert_eq!(task.attachments, held);
        assert!(!task.world.is_desynchronized());
        assert!(task.tick().is_err());
        task.world
            .cloth_mut(task.cloth)
            .unwrap()
            .set_contact_settings(Some(original))
            .unwrap();
        task.stopped = None;
        task.tick().unwrap();
        let mut control = RobotTowel::new("nominal", ImplicitExecution::Parallel4).unwrap();
        control
            .set_pose(0, pending, [0.0, 0.0, 0.0, 1.0], 1)
            .unwrap();
        control.tick().unwrap();
        assert_eq!(task.positions(), control.positions());
        assert_eq!(task.approximate_steps, 0);
        assert!(task.sample().outcome.unwrap().converged);
    }
}
