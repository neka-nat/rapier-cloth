use rapier::prelude::*;
use rapier_cloth::{Real, Vec3, rapier};

#[test]
fn math_and_parry_types_match() {
    let position: rapier::math::Vector = Vec3::new(1.0, 2.0, 3.0);
    let anchor = Pose::translation(2.0, 0.0, 0.0).transform_point(position);
    assert_eq!(anchor, Vec3::new(3.0, 2.0, 3.0));
    let shape: rapier::parry::shape::SharedShape = SharedShape::ball(1.0);
    assert!(shape.as_ball().is_some());
}

#[test]
fn updated_query_contact_sweep_and_off_center_impulse() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vec3::ZERO;
    let body = world.bodies.insert(RigidBodyBuilder::fixed());
    let collider =
        world
            .colliders
            .insert_with_parent(ColliderBuilder::ball(1.0), body, &mut world.bodies);
    world.step();
    let query = world.broad_phase.as_query_pipeline(
        world.narrow_phase.query_dispatcher(),
        &world.bodies,
        &world.colliders,
        QueryFilter::default(),
    );
    assert_eq!(
        query
            .intersect_aabb_conservative(Aabb::new(Vec3::splat(-2.0), Vec3::splat(2.0)))
            .count(),
        1
    );
    let particle = rapier::parry::shape::Ball::new(0.1);
    let contact = rapier::parry::query::contact(
        &Pose::IDENTITY,
        world.colliders[collider].shape(),
        &Pose::translation(1.05, 0.0, 0.0),
        &particle,
        0.1,
    )
    .unwrap()
    .unwrap();
    assert!((contact.dist + 0.05).abs() < 1.0e-5);
    assert!(contact.normal1.x > 0.99);
    let hit = query
        .cast_shape(
            &Pose::translation(3.0, 0.0, 0.0),
            -Vec3::X,
            &particle,
            rapier::parry::query::ShapeCastOptions::with_max_time_of_impact(3.0),
        )
        .unwrap();
    assert_eq!(hit.0, collider);
    assert!((hit.1.time_of_impact - 1.9).abs() < 1.0e-4);
    let dynamic = world
        .bodies
        .insert(RigidBodyBuilder::dynamic().additional_mass(1.0));
    world
        .colliders
        .insert_with_parent(ColliderBuilder::ball(0.2), dynamic, &mut world.bodies);
    world.step();
    world.bodies[dynamic].apply_impulse_at_point(Vec3::X, Vec3::Y, true);
    assert!(world.bodies[dynamic].linvel().x > 0.0 as Real);
    assert!(world.bodies[dynamic].angvel().z < 0.0 as Real);
}
