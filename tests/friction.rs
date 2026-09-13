mod support;
use rapier::prelude::*;
use rapier_cloth::{Real, *};
use support::TestWorld;

#[test]
fn horizontal_slowdown_matches_coulomb_law_and_zero_friction_preserves_slip() {
    for override_mu in [Some(0.0), None, Some(0.1)] {
        let mut w = TestWorld::new();
        w.cloth.collision_settings.friction_override = override_mu;
        w.rigid
            .colliders
            .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)).friction(0.4));
        let mesh = GridBuilder::new(2, 2)
            .size(0.1, 0.1)
            .origin(Vec3::Y * 0.005)
            .build()
            .unwrap();
        let mut cloth = Cloth::new(
            mesh,
            ClothMaterial {
                friction: 0.2,
                damping: 0.0,
                ..Default::default()
            },
        )
        .unwrap();
        for i in 0..4 {
            cloth.set_velocity(i, Vec3::X).unwrap();
        }
        let handle = w.cloth.add_cloth(cloth);
        for _ in 0..48 {
            w.tick().unwrap();
        }
        let expected = 1.0 - override_mu.unwrap_or(0.3) * 9.81 * 0.2;
        for v in w.cloth.cloth(handle).unwrap().velocities() {
            assert!(
                (v.x - expected).abs() < 0.001,
                "v={v:?}, expected={expected}"
            );
            assert!(v.y.abs() < 0.001);
        }
    }
}

#[test]
fn moving_and_rotating_surface_transfers_relative_tangential_velocity() {
    for rotate in [false, true] {
        let mut w = TestWorld::new();
        w.cloth.collision_settings.friction_override = Some(0.3);
        let body = w
            .rigid
            .bodies
            .insert(RigidBodyBuilder::kinematic_position_based());
        w.rigid.colliders.insert_with_parent(
            ColliderBuilder::cuboid(1.0, 0.1, 1.0),
            body,
            &mut w.rigid.bodies,
        );
        let cloth = w.grid(2, 0.01, Vec3::new(0.5, 0.105, 0.0));
        let mut pose = Pose::IDENTITY;
        if rotate {
            pose.rotation = rapier::math::Rotation::from_rotation_y(0.1 * w.h);
        } else {
            pose.translation.x = 0.1 * w.h;
        }
        w.rigid.bodies[body].set_next_kinematic_position(pose);
        w.tick().unwrap();
        let v = w.cloth.cloth(cloth).unwrap().velocities()[0];
        let expected: Real = 0.3 * 9.81 * w.h;
        if rotate {
            assert!((v.z + expected).abs() < 0.0001, "{v:?}");
        } else {
            assert!((v.x - expected).abs() < 0.0001, "{v:?}");
        }
    }
}

#[test]
fn friction_never_reverses_slip_or_uses_overlap_stabilization_as_support() {
    let v = core::contact::friction_velocity(Vec3::X, Vec3::X * 0.9, Vec3::Y, 100.0, 1.0);
    assert!((v.x - 0.9).abs() < 1.0e-6);
    let mut w = TestWorld::new();
    w.rigid.gravity = Vec3::ZERO;
    w.cloth.collision_settings.friction_override = Some(100.0);
    w.rigid
        .colliders
        .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)));
    let cloth = w.grid(2, 0.1, Vec3::Y * -0.2);
    for i in 0..4 {
        w.cloth
            .cloth_mut(cloth)
            .unwrap()
            .set_velocity(i, Vec3::X)
            .unwrap();
    }
    w.tick().unwrap();
    assert!((w.cloth.cloth(cloth).unwrap().velocities()[0].x - 1.0).abs() < 1.0e-4);
}
