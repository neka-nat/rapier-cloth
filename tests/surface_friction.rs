mod support;
use rapier::prelude::*;
use rapier_cloth::{Real, *};
use support::TestWorld;

fn surface_patch(world: &mut TestWorld) -> ClothHandle {
    let mut cloth = Cloth::new(
        GridBuilder::new(2, 2)
            .size(0.02, 0.02)
            .origin(Vec3::new(0.25, 0.1005, 0.0))
            .build()
            .unwrap(),
        ClothMaterial {
            damping: 0.0,
            ..Default::default()
        },
    )
    .unwrap();
    cloth
        .set_contact_settings(Some(ClothContactSettings {
            self_collision: false,
            rigid_surface_collision: true,
            continuous_rigid_collision: true,
            static_friction: 0.8,
            kinetic_friction: 0.4,
            ..Default::default()
        }))
        .unwrap();
    world.cloth.add_cloth(cloth)
}

#[test]
fn static_anchor_follows_translation_and_rotation_about_the_contact_normal() {
    for rotate in [false, true] {
        let mut world = TestWorld::new();
        let body = world
            .rigid
            .bodies
            .insert(RigidBodyBuilder::kinematic_position_based());
        world.rigid.colliders.insert_with_parent(
            ColliderBuilder::cuboid(0.5, 0.1, 0.5).friction(0.8),
            body,
            &mut world.rigid.bodies,
        );
        let handle = surface_patch(&mut world);
        world.tick().unwrap();
        let original = world.cloth.cloth(handle).unwrap().positions().to_vec();
        for frame in 1..=120 {
            let t = frame as Real * world.h;
            // Start from rest and stay well within the Coulomb acceleration
            // limit. A normal-only rotation would completely miss this yaw.
            let mut pose = Pose::IDENTITY;
            if rotate {
                pose.rotation = rapier::math::Rotation::from_rotation_y(0.2 * t * t);
            } else {
                pose.translation.x = 0.1 * t * t;
            }
            world.rigid.bodies[body].set_next_kinematic_position(pose);
            world
                .tick()
                .unwrap_or_else(|e| panic!("rotate={rotate} frame={frame}: {e:?}"));
            for (&p, &rest) in world
                .cloth
                .cloth(handle)
                .unwrap()
                .positions()
                .iter()
                .zip(&original)
            {
                let expected = pose.transform_point(rest);
                assert!(
                    p.distance(expected) < 2.0e-5,
                    "rotate={rotate} frame={frame}: {p:?} != {expected:?}"
                );
            }
        }
    }
}

#[test]
fn collider_removal_discards_static_anchors_and_releases_inertial_motion() {
    let mut world = TestWorld::new();
    let collider = world
        .rigid
        .colliders
        .insert(ColliderBuilder::cuboid(0.5, 0.1, 0.5).friction(0.8));
    let handle = surface_patch(&mut world);
    world.tick().unwrap();
    assert!(world.cloth.cloth(handle).unwrap().contact_history_len() > 0);
    world.rigid.colliders.remove(
        collider,
        &mut world.rigid.islands,
        &mut world.rigid.bodies,
        false,
    );
    world.rigid.gravity = Vec3::ZERO;
    for i in 0..4 {
        world
            .cloth
            .cloth_mut(handle)
            .unwrap()
            .set_velocity(i, Vec3::X)
            .unwrap();
    }
    world.tick().unwrap();
    let cloth = world.cloth.cloth(handle).unwrap();
    assert_eq!(cloth.contact_history_len(), 0);
    for velocity in cloth.velocities() {
        assert!((velocity.x - 1.0).abs() < 1.0e-5);
    }
}
