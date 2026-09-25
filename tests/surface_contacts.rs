#[path = "support/surface_oracle.rs"]
mod oracle;
mod support;
use rapier_cloth::Real;
use rapier_cloth::{rapier::prelude::*, *};
use support::TestWorld;

fn point(p: Vec3) -> [f64; 3] {
    #[cfg(feature = "f32")]
    {
        p.to_array().map(f64::from)
    }
    #[cfg(feature = "f64")]
    {
        p.to_array()
    }
}

fn triangle(height: Real) -> Cloth {
    let mesh = ClothMesh::new(
        vec![
            Vec3::new(-0.4, height, -0.3),
            Vec3::new(0.4, height, -0.3),
            Vec3::new(0.0, height, 0.4),
        ],
        vec![[0, 2, 1]],
    )
    .unwrap();
    let mut cloth = Cloth::new(
        mesh,
        ClothMaterial {
            damping: 0.0,
            friction: 0.0,
            ..Default::default()
        },
    )
    .unwrap();
    cloth
        .set_contact_settings(Some(ClothContactSettings {
            self_collision: false,
            rigid_surface_collision: true,
            static_friction: 0.0,
            kinetic_friction: 0.0,
            ..Default::default()
        }))
        .unwrap();
    cloth
}

#[test]
fn triangle_interior_is_supported_where_every_legacy_particle_misses() {
    for kind in 0..3 {
        let mut world = TestWorld::new();
        world.rigid.gravity = Vec3::ZERO;
        let shape = match kind {
            0 => SharedShape::ball(0.02),
            1 => SharedShape::cuboid(0.02, 0.02, 0.02),
            _ => SharedShape::capsule_x(0.015, 0.02),
        };
        world
            .rigid
            .colliders
            .insert(ColliderBuilder::new(shape).friction(0.0));
        let h = world.cloth.add_cloth(triangle(0.02045));
        assert!(
            world
                .cloth
                .cloth(h)
                .unwrap()
                .positions()
                .iter()
                .all(|p| p.length() > 0.25)
        );
        let report = world.tick().unwrap_or_else(|e| panic!("shape {kind}: {e}"));
        assert!(report.cloths[0].1.stabilized_contacts > 0, "shape {kind}");
        assert!(report.cloths[0].1.surface_collision.retained_contacts > 0);
        assert!(
            world
                .cloth
                .cloth(h)
                .unwrap()
                .velocities()
                .iter()
                .all(|v| v.length() < 0.001)
        );
        if kind == 0 {
            let cloth = world.cloth.cloth(h).unwrap();
            let points = cloth.mesh().triangles()[0].map(|i| point(cloth.positions()[i as usize]));
            let distance = oracle::point_triangle_distance_squared([0.0; 3], points).sqrt();
            assert!(
                distance >= 0.02045,
                "independent triangle distance {distance}"
            );
        }
        // The same posed mesh cannot reach this tiny obstacle with 5 mm balls.
        let mut control = TestWorld::new();
        control.rigid.gravity = Vec3::ZERO;
        let c = world.rigid.colliders.iter().next().unwrap().1;
        control
            .rigid
            .colliders
            .insert(ColliderBuilder::new(c.shared_shape().clone()));
        let mut cloth = triangle(0.02045);
        cloth.set_contact_settings(None).unwrap();
        control.cloth.add_cloth(cloth);
        assert_eq!(control.tick().unwrap().cloths[0].1.contacts, 0);
    }
}

#[test]
fn plane_uses_half_thickness_and_preserves_physical_friction_load() {
    for mu in [0.0, 0.5] {
        let mut world = TestWorld::new();
        world
            .rigid
            .colliders
            .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)).friction(mu));
        let mut cloth = triangle(0.0005);
        let mut settings = cloth.contact_settings().unwrap();
        settings.static_friction = mu;
        settings.kinetic_friction = mu;
        cloth.set_contact_settings(Some(settings)).unwrap();
        for i in 0..3 {
            cloth.set_velocity(i, Vec3::X * 0.1).unwrap();
        }
        let handle = world.cloth.add_cloth(cloth);
        let report = world.tick().unwrap();
        assert_eq!(report.cloths[0].1.surface_collision.retained_contacts, 3);
        for (&p, &v) in world
            .cloth
            .cloth(handle)
            .unwrap()
            .positions()
            .iter()
            .zip(world.cloth.cloth(handle).unwrap().velocities())
        {
            assert!((p.y - 0.0005).abs() < 1.0e-6, "height {p:?}");
            let expected = 0.1 - mu * 9.81 * world.h;
            assert!(
                (v.x - expected).abs() < 2.0e-5,
                "mu={mu}: {v:?}, expected {expected}"
            );
            assert!(v.y.abs() < 1.0e-5);
        }
    }
}

#[test]
fn initial_rigid_intersection_and_surface_work_overflow_are_atomic() {
    for mode in 0..3 {
        let intersection = mode == 0;
        let mut world = TestWorld::new();
        world.rigid.colliders.insert(if mode == 2 {
            ColliderBuilder::cuboid(0.02, 0.02, 0.02)
        } else {
            ColliderBuilder::ball(0.02)
        });
        let mut cloth = triangle(if intersection { 0.01 } else { 0.0205 });
        if !intersection {
            let mut settings = cloth.contact_settings().unwrap();
            if mode == 1 {
                settings.limits.candidate_pairs = 1;
            } else {
                settings.limits.retained_contacts = 1;
            }
            cloth.set_contact_settings(Some(settings)).unwrap();
        }
        let before = cloth.clone();
        let handle = world.cloth.add_cloth(cloth);
        let error = world.tick().unwrap_err();
        if intersection {
            assert!(
                matches!(error, IntegrationError::InitialRigidIntersection { .. }),
                "{error:?}"
            );
        } else {
            let expected_kind = if mode == 1 {
                CollisionBudgetKind::CandidatePairs
            } else {
                CollisionBudgetKind::RetainedContacts
            };
            assert!(
                matches!(
                    error,
                    IntegrationError::Core(ClothError::CollisionBudgetExceeded {
                        kind,
                        limit: 1,
                    }) if kind == expected_kind
                ),
                "{error:?}"
            );
        }
        let after = world.cloth.cloth(handle).unwrap();
        assert_eq!(after.positions(), before.positions());
        assert_eq!(after.velocities(), before.velocities());
        assert_eq!(after.contact_history_len(), 0);
        assert_eq!(world.cloth.next_step_index(), 0);
    }
}

#[test]
fn pinned_triangle_cannot_hide_infeasible_interior_contact() {
    let mut world = TestWorld::new();
    world.rigid.colliders.insert(ColliderBuilder::ball(0.02));
    let mut cloth = triangle(0.0201);
    for i in 0..3 {
        cloth.pin(i, cloth.positions()[i as usize]).unwrap();
    }
    let before = cloth.positions().to_vec();
    let handle = world.cloth.add_cloth(cloth);
    assert!(matches!(
        world.tick(),
        Err(IntegrationError::Core(ClothError::InfeasibleSurfaceContact))
    ));
    assert_eq!(world.cloth.cloth(handle).unwrap().positions(), before);
}

#[test]
fn incompatible_moving_surfaces_fail_before_committing_a_compressed_triangle() {
    let mut world = TestWorld::new();
    world.rigid.gravity = Vec3::ZERO;
    world
        .rigid
        .colliders
        .insert(ColliderBuilder::cuboid(0.5, 0.1, 0.5).translation(-Vec3::Y * 0.1));
    let body = world.rigid.bodies.insert(
        RigidBodyBuilder::kinematic_velocity_based()
            .translation(Vec3::Y * 0.1012)
            .linvel(-Vec3::Y * 0.096),
    );
    world.rigid.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 0.1, 0.5),
        body,
        &mut world.rigid.bodies,
    );
    let cloth = triangle(0.0006);
    let before = cloth.clone();
    let handle = world.cloth.add_cloth(cloth);
    assert!(matches!(
        world.tick(),
        Err(IntegrationError::Core(ClothError::UnresolvedSurfaceContact))
    ));
    let after = world.cloth.cloth(handle).unwrap();
    assert_eq!(after.positions(), before.positions());
    assert_eq!(after.velocities(), before.velocities());
    assert_eq!(after.contact_history_len(), 0);
}

#[test]
fn moving_sphere_pushes_triangle_interior_with_measured_surface_velocity() {
    let mut world = TestWorld::new();
    world.rigid.gravity = Vec3::ZERO;
    let body = world
        .rigid
        .bodies
        .insert(RigidBodyBuilder::kinematic_velocity_based().linvel(Vec3::Y * 0.01));
    world.rigid.colliders.insert_with_parent(
        ColliderBuilder::ball(0.02).friction(0.0),
        body,
        &mut world.rigid.bodies,
    );
    let handle = world.cloth.add_cloth(triangle(0.0205));
    for _ in 0..12 {
        world.tick().unwrap();
    }
    let cloth = world.cloth.cloth(handle).unwrap();
    let center = point(world.rigid.bodies[body].translation());
    let points = cloth.mesh().triangles()[0].map(|i| point(cloth.positions()[i as usize]));
    assert!(oracle::point_triangle_distance_squared(center, points).sqrt() >= 0.02045);
    assert!(cloth.velocities().iter().any(|v| v.y > 0.001));
}

#[test]
fn rigid_and_self_collision_share_one_cumulative_candidate_budget() {
    let run = |self_collision: bool, limit: usize| {
        let mut world = TestWorld::new();
        world
            .rigid
            .colliders
            .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)));
        let mut cloth = triangle(0.0005);
        let mut settings = cloth.contact_settings().unwrap();
        settings.self_collision = self_collision;
        settings.limits.candidate_pairs = limit;
        cloth.set_contact_settings(Some(settings)).unwrap();
        world.cloth.add_cloth(cloth);
        world.tick()
    };
    let rigid_work = run(false, 100_000).unwrap().cloths[0]
        .1
        .surface_collision
        .candidate_pairs;
    let combined_work = run(true, 100_000).unwrap().cloths[0]
        .1
        .surface_collision
        .candidate_pairs;
    assert!(combined_work > rigid_work);
    assert!(matches!(
        run(true, rigid_work),
        Err(IntegrationError::Core(
            ClothError::CollisionBudgetExceeded {
                kind: CollisionBudgetKind::CandidatePairs,
                ..
            }
        ))
    ));
}

#[test]
fn real_surface_history_restores_and_does_not_commit_when_a_later_cloth_fails() {
    let mut world = TestWorld::new();
    world
        .rigid
        .colliders
        .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)));
    let handle = world.cloth.add_cloth(triangle(0.0005));
    world.tick().unwrap();
    let checkpoint = world.cloth.checkpoint().unwrap();
    let before = world.cloth.cloth(handle).unwrap().clone();
    assert_eq!(before.contact_history_len(), 3);
    world.tick().unwrap();
    let expected = world.cloth.cloth(handle).unwrap().clone();
    world.cloth.restore(&checkpoint).unwrap();
    // The rigid scene is fixed; no moving body/controller state needs replay.
    world.step = 1;
    let mut bad = triangle(1.0);
    bad.set_force(0, Vec3::splat(Real::MAX)).unwrap();
    world.cloth.add_cloth(bad);
    assert!(world.tick().is_err());
    assert_eq!(
        world.cloth.cloth(handle).unwrap().positions(),
        before.positions()
    );
    assert_eq!(world.cloth.cloth(handle).unwrap().contact_history_len(), 3);
    world.cloth.restore(&checkpoint).unwrap();
    world.step = 1;
    world.tick().unwrap();
    assert_eq!(
        world.cloth.cloth(handle).unwrap().positions(),
        expected.positions()
    );
    assert_eq!(
        world.cloth.cloth(handle).unwrap().velocities(),
        expected.velocities()
    );
    assert_eq!(
        world.cloth.cloth(handle).unwrap().contact_history_len(),
        expected.contact_history_len()
    );
}
