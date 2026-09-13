mod support;
use rapier::prelude::*;
use rapier_cloth::*;
use support::TestWorld;

#[test]
fn restore_both_worlds_after_failure_and_replay_same_build() {
    let mut w = TestWorld::new();
    w.rigid
        .colliders
        .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)));
    let cloth = w.grid(2, 0.1, Vec3::Y * 0.01);
    w.tick().unwrap();
    let cp = w.cloth.checkpoint().unwrap();
    // For this fixed-only fixture, snapshot every persistent Rapier component.
    // PhysicsPipeline is scratch state and can be recreated.
    let bodies = w.rigid.bodies.clone();
    let colliders = w.rigid.colliders.clone();
    let broad = w.rigid.broad_phase.clone();
    let narrow = w.rigid.narrow_phase.clone();
    let islands = w.rigid.islands.clone();
    let impulses = w.rigid.impulse_joints.clone();
    let multibodies = w.rigid.multibody_joints.clone();
    let step = w.step;
    for _ in 0..60 {
        w.tick().unwrap();
    }
    let expected = w.cloth.cloth(cloth).unwrap().positions().to_vec();
    w.cloth.solver_settings.max_contacts = 1;
    assert!(w.tick().is_err());
    assert!(w.cloth.is_desynchronized());
    assert!(w.cloth.checkpoint().is_err());
    assert!(RapierClothWorld::new(w.id).restore(&cp).is_err());
    w.rigid.bodies = bodies;
    w.rigid.colliders = colliders;
    w.rigid.broad_phase = broad;
    w.rigid.narrow_phase = narrow;
    w.rigid.islands = islands;
    w.rigid.impulse_joints = impulses;
    w.rigid.multibody_joints = multibodies;
    w.rigid.physics_pipeline = PhysicsPipeline::new();
    w.rigid.ccd_solver = CCDSolver::new();
    w.cloth.restore(&cp).unwrap();
    w.step = step;
    for _ in 0..60 {
        w.tick().unwrap();
    }
    assert_eq!(w.cloth.cloth(cloth).unwrap().positions(), expected);
}
