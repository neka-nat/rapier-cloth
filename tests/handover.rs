//! Handing a grasp over from one gripper to another without releasing it.
mod support;

use rapier_cloth::Real;
use rapier_cloth::{rapier::prelude::*, *};
use support::TestWorld;

#[test]
fn a_transferred_attachment_follows_the_new_body_from_where_it_is() {
    let mut world = TestWorld::new();
    world.rigid.gravity = Vec3::ZERO;
    let cloth = world.grid(4, 0.3, Vec3::new(0.0, 0.5, 0.0));
    let first = world
        .rigid
        .bodies
        .insert(RigidBodyBuilder::kinematic_position_based().translation(Vec3::new(0.0, 0.5, 0.0)));
    let second = world
        .rigid
        .bodies
        .insert(RigidBodyBuilder::kinematic_position_based().translation(Vec3::new(1.0, 0.5, 0.0)));
    let particles = [0u32, 1];
    // Rounding allowance for both precisions.
    let tol = 1.0e3 * Real::EPSILON;
    let start: Vec<Vec3> = particles
        .iter()
        .map(|&i| world.cloth.cloth(cloth).unwrap().positions()[i as usize])
        .collect();
    let anchor = |pose: Pose, p: Vec3| pose.inverse_transform_point(p);
    let handle = world
        .cloth
        .attach(
            AttachmentDesc {
                cloth,
                body: first,
                points: particles
                    .iter()
                    .zip(&start)
                    .map(|(&particle, &p)| AttachmentPoint {
                        particle,
                        local_anchor: anchor(*world.rigid.bodies[first].position(), p),
                    })
                    .collect(),
                compliance: 0.0,
                excluded_colliders: vec![],
            },
            &world.rigid.bodies,
            &world.rigid.colliders,
        )
        .unwrap();
    // The first gripper carries the patch 1 cm sideways.
    world.rigid.bodies[first].set_next_kinematic_translation(Vec3::new(0.01, 0.5, 0.0));
    world.tick().unwrap();
    let carried: Vec<Vec3> = particles
        .iter()
        .map(|&i| world.cloth.cloth(cloth).unwrap().positions()[i as usize])
        .collect();
    for (c, s) in carried.iter().zip(&start) {
        assert!((c.x - s.x - 0.01).abs() < tol, "{c:?} vs {s:?}");
    }
    // Hand over: the second gripper takes it where it is, then moves up.
    world
        .cloth
        .transfer_attachment(
            handle,
            second,
            vec![],
            &world.rigid.bodies,
            &world.rigid.colliders,
        )
        .unwrap();
    assert_eq!(world.cloth.attachment(handle).unwrap().body, second);
    let second_pose = *world.rigid.bodies[second].position();
    for (point, c) in world
        .cloth
        .attachment(handle)
        .unwrap()
        .points
        .iter()
        .zip(&carried)
    {
        assert!(second_pose.transform_point(point.local_anchor).distance(*c) < tol);
    }
    world.rigid.bodies[first].set_next_kinematic_translation(Vec3::new(0.5, 0.5, 0.0));
    world.rigid.bodies[second].set_next_kinematic_translation(Vec3::new(1.0, 0.52, 0.0));
    world.tick().unwrap();
    for (&i, c) in particles.iter().zip(&carried) {
        let p = world.cloth.cloth(cloth).unwrap().positions()[i as usize];
        assert!((p - *c - Vec3::Y * 0.02).length() < tol, "{p:?} vs {c:?}");
    }
    // A dynamic body cannot take a grasp; the attachment is unchanged.
    let dynamic = world
        .rigid
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vec3::new(2.0, 0.5, 0.0)));
    assert!(
        world
            .cloth
            .transfer_attachment(
                handle,
                dynamic,
                vec![],
                &world.rigid.bodies,
                &world.rigid.colliders
            )
            .is_err()
    );
    assert_eq!(world.cloth.attachment(handle).unwrap().body, second);
}

#[test]
fn softening_an_attachment_keeps_its_anchors_and_lets_the_patch_sag() {
    let mut world = TestWorld::new();
    let cloth = world.grid(4, 0.3, Vec3::new(0.0, 0.5, 0.0));
    let gripper = world
        .rigid
        .bodies
        .insert(RigidBodyBuilder::kinematic_position_based().translation(Vec3::new(0.0, 0.5, 0.0)));
    let particles = [0u32, 1];
    let tol = 1.0e3 * Real::EPSILON;
    let pose = *world.rigid.bodies[gripper].position();
    let points: Vec<AttachmentPoint> = particles
        .iter()
        .map(|&particle| AttachmentPoint {
            particle,
            local_anchor: pose.inverse_transform_point(
                world.cloth.cloth(cloth).unwrap().positions()[particle as usize],
            ),
        })
        .collect();
    let handle = world
        .cloth
        .attach(
            AttachmentDesc {
                cloth,
                body: gripper,
                points: points.clone(),
                compliance: 0.0,
                excluded_colliders: vec![],
            },
            &world.rigid.bodies,
            &world.rigid.colliders,
        )
        .unwrap();
    for _ in 0..3 {
        world.tick().unwrap();
    }
    // Hard: the held particles sit on their anchors.
    for p in &points {
        let x = world.cloth.cloth(cloth).unwrap().positions()[p.particle as usize];
        assert!(
            x.distance(pose.transform_point(p.local_anchor)) < tol,
            "{x:?}"
        );
    }
    // Invalid compliances are rejected and change nothing.
    assert!(world.cloth.set_attachment_compliance(handle, -1.0).is_err());
    assert!(
        world
            .cloth
            .set_attachment_compliance(handle, Real::NAN)
            .is_err()
    );
    assert_eq!(world.cloth.attachment(handle).unwrap().compliance, 0.0);
    // Soft: the anchors stay and the particles hang below them under gravity.
    world.cloth.set_attachment_compliance(handle, 0.2).unwrap();
    let desc = world.cloth.attachment(handle).unwrap();
    assert_eq!(desc.compliance, 0.2);
    assert_eq!(desc.body, gripper);
    for (a, b) in desc.points.iter().zip(&points) {
        assert_eq!(a.particle, b.particle);
        assert_eq!(a.local_anchor, b.local_anchor);
    }
    for _ in 0..5 {
        world.tick().unwrap();
    }
    for p in &points {
        let x = world.cloth.cloth(cloth).unwrap().positions()[p.particle as usize];
        let sag = pose.transform_point(p.local_anchor).y - x.y;
        assert!(sag > 1.0e-4 && sag < 0.05, "sag {sag}");
    }
    // A released handle is stale.
    world.cloth.release(handle).unwrap();
    assert!(world.cloth.set_attachment_compliance(handle, 0.0).is_err());
}
