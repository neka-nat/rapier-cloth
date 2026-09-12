mod support;
use rapier::prelude::*;
use rapier_cloth::*;
use support::TestWorld;

#[test]
fn finite_primitives_push_cloth_within_motion_budget() {
    for shape in [
        SharedShape::ball(0.5),
        SharedShape::cuboid(0.5, 0.5, 0.5),
        SharedShape::capsule_y(0.2, 0.3),
    ] {
        let mut w = TestWorld::new();
        let body = w
            .rigid
            .bodies
            .insert(RigidBodyBuilder::kinematic_position_based().translation(Vec3::Y));
        w.rigid.colliders.insert_with_parent(
            ColliderBuilder::new(shape),
            body,
            &mut w.rigid.bodies,
        );
        let cloth = w.grid(2, 0.01, Vec3::new(0.0, 1.505, 0.0));
        for i in 1..=20 {
            w.rigid.bodies[body]
                .set_next_kinematic_translation(Vec3::Y * (1.0 + i as rapier_cloth::Real * 0.001));
            let report = w.tick().unwrap();
            assert!(report.cloths[0].1.max_penetration <= 0.001);
        }
        assert!(w.cloth.cloth(cloth).unwrap().positions()[0].y > 1.524);
        assert!(w.cloth.cloth(cloth).unwrap().velocities()[0].y > 0.23);
    }
}

#[test]
fn fast_crossings_and_rotation_are_rejected_even_outside_final_cloth_aabb() {
    for rotation in [false, true] {
        let mut w = TestWorld::new();
        let body = w
            .rigid
            .bodies
            .insert(RigidBodyBuilder::kinematic_position_based().translation(Vec3::X * -10.0));
        let collider = w.rigid.colliders.insert_with_parent(
            ColliderBuilder::cuboid(2.0, 0.05, 0.05),
            body,
            &mut w.rigid.bodies,
        );
        let cloth = w.grid(2, 0.1, Vec3::ZERO);
        let before = w.cloth.cloth(cloth).unwrap().positions().to_vec();
        let mut pose = *w.rigid.bodies[body].position();
        if rotation {
            pose.rotation = rapier::math::Rotation::from_rotation_z(1.0);
        } else {
            pose.translation.x = 10.0;
        }
        w.rigid.bodies[body].set_next_kinematic_position(pose);
        match w.tick() {
            Err(IntegrationError::MotionBudget {
                collider: c,
                required_substeps,
                ..
            }) => {
                assert_eq!(c, collider);
                assert!(required_substeps > 100);
            }
            other => panic!("expected motion budget failure: {other:?}"),
        }
        assert_eq!(w.cloth.cloth(cloth).unwrap().positions(), before);
    }
}

#[test]
fn excluded_fast_kinematic_object_does_not_consume_motion_budget() {
    let mut w = TestWorld::new();
    let body = w
        .rigid
        .bodies
        .insert(RigidBodyBuilder::kinematic_position_based());
    let c =
        w.rigid
            .colliders
            .insert_with_parent(ColliderBuilder::ball(1.0), body, &mut w.rigid.bodies);
    w.cloth.collision_settings.excluded_colliders.push(c);
    w.grid(2, 0.1, Vec3::ZERO);
    w.rigid.bodies[body].set_next_kinematic_translation(Vec3::splat(10.0));
    w.tick().unwrap();
}
