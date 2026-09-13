#[path = "support/surface_oracle.rs"]
mod oracle;
mod support;

use rapier_cloth::Real;
use rapier_cloth::{rapier::prelude::*, *};
use support::TestWorld;

fn triangle(height: Real, continuous: bool) -> Cloth {
    let mut cloth = Cloth::new(
        ClothMesh::new(
            vec![
                Vec3::new(-0.4, height, -0.3),
                Vec3::new(0.4, height, -0.3),
                Vec3::new(0.0, height, 0.4),
            ],
            vec![[0, 2, 1]],
        )
        .unwrap(),
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
            continuous_rigid_collision: continuous,
            static_friction: 0.0,
            kinetic_friction: 0.0,
            ..Default::default()
        }))
        .unwrap();
    cloth
}

fn shapes() -> [SharedShape; 3] {
    [
        SharedShape::ball(0.02),
        SharedShape::cuboid(0.02, 0.02, 0.02),
        SharedShape::capsule_x(0.015, 0.02),
    ]
}

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

// Independent f64 surface distances. All fixtures use a triangle extending
// outside the small solid, so box containment cannot masquerade as separation.
fn primitive_gap(kind: usize, pose: Pose, cloth: &Cloth) -> f64 {
    let points = cloth.mesh().triangles()[0]
        .map(|i| point(pose.inverse_transform_point(cloth.positions()[i as usize])));
    match kind {
        0 => oracle::point_triangle_distance_squared([0.0; 3], points).sqrt() - 0.02,
        1 => {
            let vertices: [[f64; 3]; 8] = std::array::from_fn(|i| {
                std::array::from_fn(|axis| if i & (1 << axis) == 0 { -0.02 } else { 0.02 })
            });
            [
                [0, 1, 3, 2],
                [4, 6, 7, 5],
                [0, 4, 5, 1],
                [2, 3, 7, 6],
                [0, 2, 6, 4],
                [1, 5, 7, 3],
            ]
            .into_iter()
            .flat_map(|[a, b, c, d]| [[a, b, c], [a, c, d]])
            .map(|face| oracle::triangle_distance_squared(points, face.map(|i| vertices[i])).sqrt())
            .fold(f64::INFINITY, f64::min)
        }
        2 => {
            // The reference triangle-distance routine explicitly handles
            // degenerate triangles; [a,b,a] represents the capsule's segment.
            let segment = [[-0.015, 0.0, 0.0], [0.015, 0.0, 0.0], [-0.015, 0.0, 0.0]];
            oracle::triangle_distance_squared(points, segment).sqrt() - 0.02
        }
        _ => unreachable!(),
    }
}

#[test]
fn primitive_interior_crossings_are_stopped_without_self_collision() {
    for (kind, shape) in shapes().into_iter().enumerate() {
        for continuous in [false, true] {
            let mut world = TestWorld::new();
            world.rigid.gravity = Vec3::ZERO;
            world
                .rigid
                .colliders
                .insert(ColliderBuilder::new(shape.clone()).friction(0.0));
            let mut cloth = triangle(0.04, continuous);
            for i in 0..3 {
                cloth.set_velocity(i, -Vec3::Y * (0.08 / world.h)).unwrap();
            }
            let handle = world.cloth.add_cloth(cloth);
            let report = world
                .tick()
                .unwrap_or_else(|e| panic!("kind={kind}, CCD={continuous}: {e:?}"));
            let cloth = world.cloth.cloth(handle).unwrap();
            if continuous {
                assert!(report.cloths[0].1.surface_collision.ccd_checks > 0);
                assert!(report.cloths[0].1.surface_collision.limited_advances > 0);
                assert!(
                    primitive_gap(kind, Pose::IDENTITY, cloth) >= 0.00045,
                    "shape {kind}: {:?}",
                    cloth.positions()
                );
                assert!(cloth.positions().iter().map(|p| p.y).sum::<Real>() > 0.0);
            } else {
                assert!(cloth.positions().iter().all(|p| p.y < -0.039));
            }
        }
    }
}

#[test]
fn continuous_plane_contact_keeps_tangent_and_full_gravity_load() {
    for mu in [0.0, 0.5] {
        let mut world = TestWorld::new();
        world
            .rigid
            .colliders
            .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)).friction(mu));
        let mut cloth = triangle(0.0005, true);
        let mut config = cloth.contact_settings().unwrap();
        config.static_friction = mu;
        config.kinetic_friction = mu;
        cloth.set_contact_settings(Some(config)).unwrap();
        for i in 0..3 {
            cloth.set_velocity(i, Vec3::X * 0.1).unwrap();
        }
        let handle = world.cloth.add_cloth(cloth);
        let report = world.tick().unwrap();
        assert!(report.cloths[0].1.surface_collision.limited_advances > 0);
        for (&p, &v) in world
            .cloth
            .cloth(handle)
            .unwrap()
            .positions()
            .iter()
            .zip(world.cloth.cloth(handle).unwrap().velocities())
        {
            assert!((p.y - 0.0005).abs() < 1.0e-6, "{p:?}");
            assert!(
                (v.x - (0.1 - mu * 9.81 * world.h)).abs() < 2.0e-5,
                "mu={mu}: {v:?}"
            );
            assert!(v.y.abs() < 1.0e-5, "{v:?}");
        }
    }
}

#[test]
fn moving_primitives_use_the_whole_substep_and_push_the_surface() {
    for (kind, shape) in shapes().into_iter().enumerate() {
        let mut world = TestWorld::new();
        world.rigid.gravity = Vec3::ZERO;
        let body = world
            .rigid
            .bodies
            .insert(RigidBodyBuilder::kinematic_velocity_based().linvel(Vec3::Y * 0.01));
        world.rigid.colliders.insert_with_parent(
            ColliderBuilder::new(shape).friction(0.0),
            body,
            &mut world.rigid.bodies,
        );
        let handle = world.cloth.add_cloth(triangle(0.0205, true));
        for step in 0..12 {
            world
                .tick()
                .unwrap_or_else(|e| panic!("kind={kind}, step={step}: {e:?}"));
        }
        let cloth = world.cloth.cloth(handle).unwrap();
        assert!(primitive_gap(kind, *world.rigid.bodies[body].position(), cloth) >= 0.00045);
        assert!(cloth.velocities().iter().any(|v| v.y > 0.001));
    }
}

#[test]
fn rotating_supports_work_for_position_and_velocity_based_grippers() {
    for (kind, shape) in shapes().into_iter().enumerate() {
        for position_based in [false, true] {
            let mut world = TestWorld::new();
            world.rigid.gravity = Vec3::ZERO;
            let builder = if position_based {
                RigidBodyBuilder::kinematic_position_based()
            } else {
                RigidBodyBuilder::kinematic_velocity_based()
                    .linvel(Vec3::Y * 0.01)
                    .angvel(Vec3::Z * (0.002 / world.h))
            };
            let body = world.rigid.bodies.insert(builder);
            world.rigid.colliders.insert_with_parent(
                ColliderBuilder::new(shape.clone()).friction(0.0),
                body,
                &mut world.rigid.bodies,
            );
            let handle = world.cloth.add_cloth(triangle(0.0205, true));
            for i in 1..=12 {
                if position_based {
                    world.rigid.bodies[body].set_next_kinematic_position(Pose::from_parts(
                        Vec3::Y * (0.01 * world.h * i as Real),
                        Rotation::from_rotation_z(0.002 * i as Real),
                    ));
                }
                world.tick().unwrap_or_else(|e| {
                    panic!("shape {kind}, position={position_based}, step {i}: {e:?}")
                });
                let cloth = world.cloth.cloth(handle).unwrap();
                assert!(
                    primitive_gap(kind, *world.rigid.bodies[body].position(), cloth) >= 0.00045,
                    "shape {kind}, position={position_based}, step {i}"
                );
            }
        }
    }
}

#[test]
fn a_kinematic_primitive_cannot_cross_a_fixed_surface_between_clear_endpoints() {
    for (kind, shape) in shapes().into_iter().enumerate() {
        for continuous in [false, true] {
            let mut world = TestWorld::new();
            world.rigid.gravity = Vec3::ZERO;
            // Explicitly expand this analytical fixture's motion envelope. The
            // production/M1 motion limit and external time step stay unchanged.
            world.cloth.collision_settings.motion_limit_ratio = 100.0;
            let body = world.rigid.bodies.insert(
                RigidBodyBuilder::kinematic_velocity_based()
                    .translation(-Vec3::Y * 0.04)
                    .linvel(Vec3::Y * (0.08 / world.h)),
            );
            world.rigid.colliders.insert_with_parent(
                ColliderBuilder::new(shape.clone()),
                body,
                &mut world.rigid.bodies,
            );
            let mut cloth = triangle(0.0, continuous);
            for i in 0..3 {
                cloth.pin(i, cloth.positions()[i as usize]).unwrap();
            }
            let before = cloth.clone();
            let handle = world.cloth.add_cloth(cloth);
            let result = world.tick();
            if continuous {
                assert!(
                    matches!(
                        result,
                        Err(IntegrationError::Core(ClothError::InfeasibleSurfaceContact))
                    ),
                    "shape {kind}: {result:?}"
                );
                let after = world.cloth.cloth(handle).unwrap();
                assert_eq!(after.positions(), before.positions());
                assert_eq!(after.previous_positions(), before.previous_positions());
                assert_eq!(after.velocities(), before.velocities());
                assert_eq!(after.contact_history_len(), before.contact_history_len());
            } else {
                result.unwrap();
            }
        }
    }
}

#[test]
fn a_complete_offset_turn_cannot_alias_a_stationary_obstacle() {
    for continuous in [false, true] {
        let mut world = TestWorld::new();
        world.rigid.gravity = Vec3::ZERO;
        world.cloth.collision_settings.motion_limit_ratio = 1000.0;
        let tau = 2.0 * Real::acos(-1.0);
        let body = world.rigid.bodies.insert(
            RigidBodyBuilder::kinematic_velocity_based()
                .angvel(Vec3::Z * (tau / world.h))
                .additional_mass_properties(MassProperties::new(Vec3::ZERO, 1.0, Vec3::ONE)),
        );
        world.rigid.colliders.insert_with_parent(
            ColliderBuilder::ball(0.01)
                .translation(Vec3::X * 0.15)
                .density(0.0),
            body,
            &mut world.rigid.bodies,
        );
        let mut cloth = triangle(0.15, continuous);
        for i in 0..3 {
            cloth.pin(i, cloth.positions()[i as usize]).unwrap();
        }
        let before = cloth.clone();
        let handle = world.cloth.add_cloth(cloth);
        let result = world.tick();
        if continuous {
            assert!(
                matches!(
                    result,
                    Err(IntegrationError::Core(
                        ClothError::InfeasibleSurfaceContact
                            | ClothError::UnresolvedContinuousCollision(_)
                    ))
                ),
                "{result:?}"
            );
            assert_eq!(
                world.cloth.cloth(handle).unwrap().positions(),
                before.positions()
            );
            assert_eq!(
                world.cloth.cloth(handle).unwrap().velocities(),
                before.velocities()
            );
        } else {
            result.unwrap();
        }
    }
}

#[test]
fn thin_rotating_gripper_cannot_pass_through_a_triangle_interior() {
    for continuous in [false, true] {
        let mut world = TestWorld::new();
        world.rigid.gravity = Vec3::ZERO;
        world.cloth.collision_settings.motion_limit_ratio = 100.0;
        let body = world
            .rigid
            .bodies
            .insert(RigidBodyBuilder::kinematic_position_based());
        world.rigid.colliders.insert_with_parent(
            ColliderBuilder::cuboid(0.15, 0.0005, 0.02),
            body,
            &mut world.rigid.bodies,
        );
        let mut cloth = triangle(0.08, continuous);
        for i in 0..3 {
            cloth.pin(i, cloth.positions()[i as usize]).unwrap();
        }
        let before = cloth.clone();
        let handle = world.cloth.add_cloth(cloth);
        world.rigid.bodies[body].set_next_kinematic_position(Pose::from_parts(
            Vec3::ZERO,
            Rotation::from_rotation_z(2.8),
        ));
        // The 1 mm thick bar is below the plane at both endpoints. At pi/2
        // its centerline passes through (0, 0.08, 0), inside the coarse triangle.
        assert!(0.15 * (2.8_f64).sin() + 0.0005 * (2.8_f64).cos().abs() < 0.08);
        assert!(before.positions().iter().all(|p| p.length() > 0.3));
        let result = world.tick();
        if continuous {
            assert!(
                matches!(
                    result,
                    Err(IntegrationError::Core(
                        ClothError::InfeasibleSurfaceContact
                            | ClothError::UnresolvedContinuousCollision(_)
                    ))
                ),
                "{result:?}"
            );
            assert_eq!(
                world.cloth.cloth(handle).unwrap().positions(),
                before.positions()
            );
            assert_eq!(
                world.cloth.cloth(handle).unwrap().velocities(),
                before.velocities()
            );
        } else {
            result.unwrap();
        }
    }
}

#[test]
fn external_ccd_failure_rolls_back_earlier_cloth_and_contact_history() {
    let mut world = TestWorld::new();
    world
        .rigid
        .colliders
        .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)));
    let handle = world.cloth.add_cloth(triangle(0.0005, true));
    world.tick().unwrap();
    let checkpoint = world.cloth.checkpoint().unwrap();
    let before = world.cloth.cloth(handle).unwrap().clone();
    assert_eq!(before.contact_history_len(), 3);
    world.tick().unwrap();
    let expected = world.cloth.cloth(handle).unwrap().clone();
    world.cloth.restore(&checkpoint).unwrap();
    world.step = 1;
    let mut bad = triangle(0.01, true);
    let mut config = bad.contact_settings().unwrap();
    config.limits.ccd_checks = 1;
    bad.set_contact_settings(Some(config)).unwrap();
    world.cloth.add_cloth(bad);
    let result = world.tick();
    assert!(
        matches!(
            result,
            Err(IntegrationError::Core(
                ClothError::CollisionBudgetExceeded {
                    kind: CollisionBudgetKind::CcdChecks,
                    limit: 1
                }
            ))
        ),
        "{result:?}"
    );
    let after = world.cloth.cloth(handle).unwrap();
    assert_eq!(after.positions(), before.positions());
    assert_eq!(after.previous_positions(), before.previous_positions());
    assert_eq!(after.velocities(), before.velocities());
    assert_eq!(after.contact_history_len(), before.contact_history_len());
    world.cloth.restore(&checkpoint).unwrap();
    world.step = 1;
    world.tick().unwrap();
    let replay = world.cloth.cloth(handle).unwrap();
    assert_eq!(replay.positions(), expected.positions());
    assert_eq!(replay.velocities(), expected.velocities());
    assert_eq!(replay.contact_history_len(), expected.contact_history_len());
}

#[test]
fn shape_or_fixed_pose_changes_after_the_snapshot_fail_atomically() {
    for change_shape in [false, true] {
        let mut world = TestWorld::new();
        world.rigid.gravity = Vec3::ZERO;
        let collider = world.rigid.colliders.insert(ColliderBuilder::ball(0.02));
        let handle = world.cloth.add_cloth(triangle(0.04, true));
        let original = world.cloth.cloth(handle).unwrap().clone();
        let before =
            SceneSnapshot::capture(world.id, 0, &world.rigid.bodies, &world.rigid.colliders);
        if change_shape {
            world.rigid.colliders[collider].set_shape(SharedShape::ball(0.021));
        } else {
            world.rigid.colliders[collider].set_translation(Vec3::Y * 0.001);
        }
        world.rigid.step();
        let query = world.rigid.broad_phase.as_query_pipeline(
            world.rigid.narrow_phase.query_dispatcher(),
            &world.rigid.bodies,
            &world.rigid.colliders,
            QueryFilter::default(),
        );
        let scene = RapierScene::new(query, &before, world.h, Vec3::ZERO);
        let result = world.cloth.step_substep(world.h, &scene);
        assert!(
            matches!(result, Err(IntegrationError::InvalidScene(_))),
            "{result:?}"
        );
        let after = world.cloth.cloth(handle).unwrap();
        assert_eq!(after.positions(), original.positions());
        assert_eq!(after.velocities(), original.velocities());
        assert_eq!(after.contact_history_len(), original.contact_history_len());
    }
}
