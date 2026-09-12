mod support;
use rapier::prelude::*;
use rapier_cloth::{Real, *};
use support::TestWorld;

fn attach(
    w: &mut TestWorld,
    cloth: ClothHandle,
    body: RigidBodyHandle,
    particles: &[u32],
) -> AttachmentHandle {
    let pose = w.rigid.bodies[body].position();
    let points = particles
        .iter()
        .map(|&particle| AttachmentPoint {
            particle,
            local_anchor: pose.inverse_transform_point(
                w.cloth.cloth(cloth).unwrap().positions()[particle as usize],
            ),
        })
        .collect();
    w.cloth
        .attach(
            AttachmentDesc {
                cloth,
                body,
                points,
                compliance: 0.0,
                excluded_colliders: vec![],
            },
            &w.rigid.bodies,
            &w.rigid.colliders,
        )
        .unwrap()
}

#[test]
fn local_anchors_translate_rotate_and_keep_release_velocity() {
    let mut w = TestWorld::new();
    w.rigid.gravity = Vec3::ZERO;
    let body = w
        .rigid
        .bodies
        .insert(RigidBodyBuilder::kinematic_position_based().translation(Vec3::Y));
    let cloth = w.grid(2, 0.1, Vec3::Y + Vec3::X * 0.3);
    let a = attach(&mut w, cloth, body, &[0, 1, 2, 3]);
    let points = w.cloth.attachment(a).unwrap().points.clone();
    for step in 1..=50 {
        let previous = w.cloth.cloth(cloth).unwrap().positions().to_vec();
        let mut pose = Pose::translation(step as Real * 0.001, 1.0, 0.0);
        pose.rotation = rapier::math::Rotation::from_rotation_y(step as Real * 0.002);
        w.rigid.bodies[body].set_next_kinematic_position(pose);
        w.tick().unwrap();
        for p in &points {
            let i = p.particle as usize;
            let c = w.cloth.cloth(cloth).unwrap();
            let expected = pose.transform_point(p.local_anchor);
            assert!(c.positions()[i].distance(expected) < 2.0e-6);
            assert!(c.velocities()[i].distance((expected - previous[i]) / w.h) < 2.0e-4);
        }
    }
    let v = w.cloth.cloth(cloth).unwrap().velocities().to_vec();
    w.cloth.release(a).unwrap();
    assert_eq!(w.cloth.cloth(cloth).unwrap().velocities(), v);
    assert!(w.cloth.release(a).is_err());
    assert_eq!(
        w.cloth.drain_attachment_events().next().unwrap().kind,
        AttachmentEventKind::Released
    );
    let other = w.rigid.bodies.insert(RigidBodyBuilder::fixed());
    let b = attach(&mut w, cloth, other, &[0]);
    assert_eq!(a.index(), b.index());
    assert_ne!(a.generation(), b.generation());
    assert!(w.cloth.attachment(a).is_err());
}

#[test]
fn deleted_bodies_do_not_retarget_reused_handles_and_cloth_removal_emits_events() {
    let mut w = TestWorld::new();
    w.rigid.gravity = Vec3::ZERO;
    let body = w.rigid.bodies.insert(RigidBodyBuilder::fixed());
    let cloth = w.grid(2, 0.1, Vec3::Y);
    let a = attach(&mut w, cloth, body, &[0]);
    w.rigid.bodies.remove(
        body,
        &mut w.rigid.islands,
        &mut w.rigid.colliders,
        &mut w.rigid.impulse_joints,
        &mut w.rigid.multibody_joints,
        true,
    );
    let replacement = w
        .rigid
        .bodies
        .insert(RigidBodyBuilder::fixed().translation(Vec3::splat(20.0)));
    w.tick().unwrap();
    assert!(w.cloth.attachment(a).is_err());
    assert_eq!(
        w.cloth.drain_attachment_events().next().unwrap().kind,
        AttachmentEventKind::BodyRemoved
    );
    assert!(w.cloth.cloth(cloth).unwrap().positions()[0].distance(Vec3::Y) < 1.0e-5);
    let b = attach(&mut w, cloth, replacement, &[0]);
    w.cloth.remove_cloth(cloth).unwrap();
    assert!(w.cloth.attachment(b).is_err());
    assert_eq!(
        w.cloth.drain_attachment_events().next().unwrap().kind,
        AttachmentEventKind::ClothRemoved
    );
}

#[test]
fn exclusions_apply_only_to_attached_particles_and_their_own_collider() {
    let mut w = TestWorld::new();
    w.rigid.gravity = Vec3::ZERO;
    let body = w.rigid.bodies.insert(RigidBodyBuilder::fixed());
    let grip = w.rigid.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.02, 0.01, 0.02),
        body,
        &mut w.rigid.bodies,
    );
    let cloth = w.grid(2, 0.01, Vec3::ZERO);
    let desc = AttachmentDesc {
        cloth,
        body,
        points: vec![AttachmentPoint {
            particle: 0,
            local_anchor: Vec3::ZERO,
        }],
        compliance: 0.0,
        excluded_colliders: vec![grip],
    };
    let a = w
        .cloth
        .attach(desc.clone(), &w.rigid.bodies, &w.rigid.colliders)
        .unwrap();
    assert!(
        w.cloth
            .attach(desc, &w.rigid.bodies, &w.rigid.colliders)
            .is_err()
    );
    w.tick().unwrap();
    let p = w.cloth.cloth(cloth).unwrap().positions();
    assert_eq!(p[0], Vec3::ZERO);
    assert!(
        p[1..]
            .iter()
            .all(|p| p.y.abs() >= 0.014 || p.x.abs() >= 0.024 || p.z.abs() >= 0.024)
    );
    w.rigid
        .colliders
        .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)));
    assert!(
        w.tick().is_err(),
        "other environment must still conflict with attached particle"
    );
    assert!(w.cloth.attachment(a).is_ok());
}

#[test]
fn soft_attachment_is_compliant_and_foreign_handles_are_rejected() {
    let mut w = TestWorld::new();
    w.rigid.gravity = Vec3::ZERO;
    let body = w
        .rigid
        .bodies
        .insert(RigidBodyBuilder::fixed().translation(Vec3::Y));
    let cloth = w.grid(2, 0.1, Vec3::ZERO);
    let points = w
        .cloth
        .cloth(cloth)
        .unwrap()
        .positions()
        .iter()
        .enumerate()
        .map(|(i, &p)| AttachmentPoint {
            particle: i as u32,
            local_anchor: p,
        })
        .collect();
    let a = w
        .cloth
        .attach(
            AttachmentDesc {
                cloth,
                body,
                points,
                compliance: 0.01,
                excluded_colliders: vec![],
            },
            &w.rigid.bodies,
            &w.rigid.colliders,
        )
        .unwrap();
    w.tick().unwrap();
    assert!(
        w.cloth
            .cloth(cloth)
            .unwrap()
            .positions()
            .iter()
            .all(|p| p.y > 0.0 && p.y < 1.0)
    );
    assert!(TestWorld::new().cloth.attachment(a).is_err());
}
