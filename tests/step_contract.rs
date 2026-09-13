mod support;
use rapier::prelude::*;
use rapier_cloth::*;
use support::TestWorld;

#[test]
fn token_duration_world_and_duplicate_steps_are_checked() {
    let mut w = TestWorld::new();
    w.grid(2, 0.1, Vec3::Y);
    let previous = SceneSnapshot::capture(w.id, 0, &w.rigid.bodies, &w.rigid.colliders);
    let wrong = SceneSnapshot::capture(WorldId::new(), 0, &w.rigid.bodies, &w.rigid.colliders);
    w.rigid.step();
    let query = w.rigid.broad_phase.as_query_pipeline(
        w.rigid.narrow_phase.query_dispatcher(),
        &w.rigid.bodies,
        &w.rigid.colliders,
        QueryFilter::default(),
    );
    assert!(
        w.cloth
            .step_substep(w.h, &RapierScene::new(query, &wrong, w.h, Vec3::ZERO))
            .is_err()
    );
    let scene = RapierScene::new(query, &previous, w.h, Vec3::ZERO);
    assert!(w.cloth.step_substep(w.h * 0.5, &scene).is_err());
    w.cloth.step_substep(w.h, &scene).unwrap();
    assert!(w.cloth.step_substep(w.h, &scene).is_err());
}

#[test]
fn contact_budget_and_later_cloth_failure_are_atomic_for_whole_world() {
    let mut w = TestWorld::new();
    w.rigid
        .colliders
        .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)));
    let high = w.grid(2, 0.1, Vec3::Y);
    let low = w.grid(2, 0.1, Vec3::ZERO);
    let before = w.cloth.cloth(high).unwrap().positions().to_vec();
    let below = w.cloth.cloth(low).unwrap().positions().to_vec();
    w.cloth.solver_settings.max_contacts = 1;
    assert!(w.tick().is_err());
    assert_eq!(w.cloth.cloth(high).unwrap().positions(), before);
    assert_eq!(w.cloth.cloth(low).unwrap().positions(), below);
    assert!(w.cloth.is_desynchronized());
    assert!(w.tick().is_err());
}
