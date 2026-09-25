mod support;
use rapier::prelude::*;
use rapier_cloth::{Real, *};
use support::TestWorld;

fn point(cloth: ClothHandle, triangle: u32, bary: [Real; 3]) -> ClothSurfacePoint {
    ClothSurfacePoint {
        cloth,
        point: SurfacePoint::new(triangle, bary).unwrap(),
    }
}

#[test]
fn visible_triangle_interior_wins_over_a_nearer_hidden_vertex_and_conflicts_do_not_pick_underneath()
{
    let mut world = TestWorld::new();
    let bottom = world.grid(3, 0.2, Vec3::ZERO);
    let top = world.grid(2, 0.2, Vec3::Y * 0.001);
    let body = world
        .rigid
        .bodies
        .insert(RigidBodyBuilder::kinematic_position_based());
    let ray = SurfaceRay {
        origin: Vec3::new(0.08, 1.0, 0.08),
        direction: -Vec3::Y,
        max_distance: 2.0,
    };
    let hit = world
        .cloth
        .raycast_cloth(ray, SurfaceQueryLimits::default())
        .unwrap()
        .unwrap();
    assert_eq!(hit.point.cloth, top);
    let distant = world
        .cloth
        .raycast_cloth(
            SurfaceRay {
                origin: Vec3::new(0.08, 1.0e6, 0.08),
                max_distance: 2.0e6,
                ..ray
            },
            SurfaceQueryLimits::default(),
        )
        .unwrap()
        .unwrap();
    assert_eq!(
        distant.point.cloth, top,
        "rounding a long ray distance must not reverse layer order"
    );
    assert!(hit.point.point.barycentric().iter().all(|b| *b > 0.0));
    let grasp = world
        .cloth
        .grasp_surface(
            hit.point,
            GraspOptions::new(body),
            &world.rigid.bodies,
            &world.rigid.colliders,
        )
        .unwrap();
    assert_eq!(world.cloth.surface_attachment(grasp).unwrap().cloth, top);
    let repeated = world
        .cloth
        .raycast_cloth(ray, SurfaceQueryLimits::default())
        .unwrap()
        .unwrap();
    assert_eq!(repeated.point.cloth, top);
    assert!(matches!(
        world.cloth.grasp_surface(
            repeated.point,
            GraspOptions::new(body),
            &world.rigid.bodies,
            &world.rigid.colliders
        ),
        Err(IntegrationError::Core(ClothError::ConflictingTarget(_)))
    ));
    assert_eq!(world.cloth.surface_attachments().count(), 1);
    assert!(world.cloth.cloth(bottom).unwrap().pins().is_empty());
    assert!(matches!(
        world.cloth.raycast_cloth(
            ray,
            SurfaceQueryLimits {
                triangles: 8,
                ..Default::default()
            }
        ),
        Err(ClothError::SurfaceQueryBudgetExceeded { limit: 8 })
    ));
    world.cloth.remove_cloth(top).unwrap();
    assert!(world.cloth.surface_point_position(hit.point).is_err());
    assert!(world.cloth.surface_attachment(grasp).is_err());
    assert_eq!(
        world
            .cloth
            .raycast_cloth(ray, SurfaceQueryLimits::default())
            .unwrap()
            .unwrap()
            .point
            .cloth,
        bottom
    );
}

#[test]
fn two_gripper_patches_preserve_offsets_lift_and_release_without_hidden_pins() {
    let mut world = TestWorld::new();
    world.rigid.gravity = Vec3::ZERO;
    let cloth = world.grid(4, 0.3, Vec3::Y * 0.1);
    let last = world.cloth.cloth(cloth).unwrap().mesh().triangles().len() as u32 - 1;
    let bodies = [
        world
            .rigid
            .bodies
            .insert(RigidBodyBuilder::kinematic_position_based()),
        world
            .rigid
            .bodies
            .insert(RigidBodyBuilder::kinematic_position_based()),
    ];
    let points = [
        point(cloth, 0, [1.0, 0.0, 0.0]),
        point(cloth, last, [0.0, 0.0, 1.0]),
    ];
    let mut handles = Vec::new();
    let mut selections = Vec::new();
    let original = world.cloth.cloth(cloth).unwrap().positions().to_vec();
    for (point, body) in points.into_iter().zip(bodies) {
        let patch = world
            .cloth
            .select_grasp_patch(point, 0.10001, SurfaceQueryLimits::default())
            .unwrap();
        assert_eq!(patch.vertices().len(), 3);
        selections.extend_from_slice(patch.vertices());
        handles.push(
            world
                .cloth
                .grasp_patch(
                    &patch,
                    GraspOptions::new(body),
                    &world.rigid.bodies,
                    &world.rigid.colliders,
                )
                .unwrap(),
        );
    }
    for frame in 1..=20 {
        let shift = Vec3::Y * (frame as Real * 0.0005);
        for body in bodies {
            world.rigid.bodies[body].set_next_kinematic_position(Pose::from_translation(shift));
        }
        let report = world.tick().unwrap();
        assert!(report.cloths[0].1.max_target_error < 1.0e-6);
        for &i in &selections {
            assert!(
                world.cloth.cloth(cloth).unwrap().positions()[i as usize]
                    .distance(original[i as usize] + shift)
                    < 1.0e-6
            );
        }
    }
    let velocities = world.cloth.cloth(cloth).unwrap().velocities().to_vec();
    for handle in handles {
        world.cloth.release(handle).unwrap();
    }
    assert_eq!(world.cloth.attachments().count(), 0);
    assert_eq!(world.cloth.surface_attachments().count(), 0);
    assert!(world.cloth.cloth(cloth).unwrap().pins().is_empty());
    assert_eq!(world.cloth.cloth(cloth).unwrap().velocities(), velocities);
    let events: Vec<_> = world.cloth.drain_attachment_events().collect();
    assert_eq!(events.len(), 2);
    assert!(
        events
            .iter()
            .all(|e| e.kind == AttachmentEventKind::Released)
    );
    let before = world.cloth.cloth(cloth).unwrap().positions()[0];
    world.tick().unwrap();
    assert!(world.cloth.cloth(cloth).unwrap().positions()[0].y > before.y);
}

#[test]
fn surface_grasp_tracks_a_moving_body_and_is_released_when_disabled() {
    let mut world = TestWorld::new();
    let cloth = world.grid(2, 0.1, Vec3::Y * 0.1);
    let body = world
        .rigid
        .bodies
        .insert(RigidBodyBuilder::kinematic_position_based());
    let point = point(cloth, 0, [0.2, 0.3, 0.5]);
    let origin = world.cloth.surface_point_position(point).unwrap();
    let handle = world
        .cloth
        .grasp_surface(
            point,
            GraspOptions::new(body),
            &world.rigid.bodies,
            &world.rigid.colliders,
        )
        .unwrap();
    for frame in 1..=20 {
        let shift = Vec3::Y * (frame as Real * 0.0005);
        world.rigid.bodies[body].set_next_kinematic_position(Pose::from_translation(shift));
        world.tick().unwrap();
        assert!(
            world
                .cloth
                .surface_point_position(point)
                .unwrap()
                .distance(origin + shift)
                < 1.0e-6
        );
    }
    assert!(world.cloth.cloth(cloth).unwrap().pins().is_empty());
    world.rigid.bodies[body].set_enabled(false);
    world.tick().unwrap();
    assert!(world.cloth.surface_attachment(handle).is_err());
    let events: Vec<_> = world.cloth.drain_attachment_events().collect();
    assert_eq!(events.len(), 1);
    assert_eq!(events[0].kind, AttachmentEventKind::BodyDisabled);
}

#[test]
fn weighted_target_cannot_cross_a_plane_and_failure_preserves_world_state() {
    let mut world = TestWorld::new();
    world
        .rigid
        .colliders
        .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)).friction(0.0));
    let cloth = world.grid(2, 0.1, Vec3::Y * 0.005);
    world
        .cloth
        .cloth_mut(cloth)
        .unwrap()
        .set_contact_settings(Some(ClothContactSettings {
            self_collision: false,
            rigid_surface_collision: true,
            continuous_rigid_collision: true,
            static_friction: 0.0,
            kinetic_friction: 0.0,
            ..Default::default()
        }))
        .unwrap();
    let body = world
        .rigid
        .bodies
        .insert(RigidBodyBuilder::kinematic_position_based());
    let handle = world
        .cloth
        .grasp_surface(
            point(cloth, 0, [0.2, 0.3, 0.5]),
            GraspOptions::new(body),
            &world.rigid.bodies,
            &world.rigid.colliders,
        )
        .unwrap();
    let before = world.cloth.cloth(cloth).unwrap().clone();
    world.rigid.bodies[body].set_next_kinematic_position(Pose::from_translation(-Vec3::Y * 0.01));
    let result = world.tick();
    assert!(
        matches!(
            result,
            Err(IntegrationError::Core(
                ClothError::ConflictingSurfaceTarget { .. }
                    | ClothError::UnresolvedContinuousCollision(_)
            ))
        ),
        "{result:?}"
    );
    assert_eq!(
        world.cloth.cloth(cloth).unwrap().positions(),
        before.positions()
    );
    assert_eq!(
        world.cloth.cloth(cloth).unwrap().velocities(),
        before.velocities()
    );
    assert!(world.cloth.surface_attachment(handle).is_ok());
    assert_eq!(world.cloth.next_step_index(), 0);
}

#[test]
fn checkpoint_and_slot_reuse_do_not_reconnect_discarded_grasp_handles() {
    let mut world = TestWorld::new();
    let cloth = world.grid(2, 0.1, Vec3::Y * 0.1);
    let bodies = [
        world
            .rigid
            .bodies
            .insert(RigidBodyBuilder::kinematic_position_based()),
        world
            .rigid
            .bodies
            .insert(RigidBodyBuilder::kinematic_position_based()),
    ];
    let point = point(cloth, 0, [0.2, 0.3, 0.5]);
    let original = world
        .cloth
        .grasp_surface(
            point,
            GraspOptions::new(bodies[0]),
            &world.rigid.bodies,
            &world.rigid.colliders,
        )
        .unwrap();
    let checkpoint = world.cloth.checkpoint().unwrap();
    world.cloth.release(original).unwrap();
    let discarded = world
        .cloth
        .grasp_surface(
            point,
            GraspOptions::new(bodies[0]),
            &world.rigid.bodies,
            &world.rigid.colliders,
        )
        .unwrap();
    world.cloth.restore(&checkpoint).unwrap();
    assert!(world.cloth.surface_attachment(original).is_ok());
    assert!(world.cloth.surface_attachment(discarded).is_err());
    world.cloth.release(original).unwrap();
    let replacement = world
        .cloth
        .grasp_surface(
            point,
            GraspOptions::new(bodies[1]),
            &world.rigid.bodies,
            &world.rigid.colliders,
        )
        .unwrap();
    assert_eq!(discarded.index(), replacement.index());
    assert_ne!(discarded.generation(), replacement.generation());
    assert!(world.cloth.release(discarded).is_err());
    assert_eq!(
        world.cloth.surface_attachment(replacement).unwrap().body,
        bodies[1]
    );
    let events: Vec<_> = world.cloth.drain_attachment_events().collect();
    assert_eq!(events.len(), 1);
    assert_eq!(events[0].handle, original);
}

#[test]
fn mixed_grasp_conflicts_and_wrong_owner_exclusions_fail_before_creation() {
    let mut world = TestWorld::new();
    let cloth = world.grid(2, 0.1, Vec3::Y * 0.1);
    let bodies = [
        world
            .rigid
            .bodies
            .insert(RigidBodyBuilder::kinematic_position_based()),
        world
            .rigid
            .bodies
            .insert(RigidBodyBuilder::kinematic_position_based()),
    ];
    let other_collider = world.rigid.colliders.insert_with_parent(
        ColliderBuilder::ball(0.001),
        bodies[1],
        &mut world.rigid.bodies,
    );
    let point = point(cloth, 0, [0.2, 0.3, 0.5]);
    let vertex = AttachmentDesc {
        cloth,
        body: bodies[1],
        points: vec![AttachmentPoint {
            particle: 0,
            local_anchor: Vec3::Y * 0.1,
        }],
        compliance: 0.0,
        excluded_colliders: vec![],
    };
    let handle = world
        .cloth
        .grasp_surface(
            point,
            GraspOptions::new(bodies[0]),
            &world.rigid.bodies,
            &world.rigid.colliders,
        )
        .unwrap();
    assert!(matches!(
        world
            .cloth
            .attach(vertex.clone(), &world.rigid.bodies, &world.rigid.colliders),
        Err(IntegrationError::Core(ClothError::ConflictingTarget(0)))
    ));
    world.cloth.release(handle).unwrap();
    let handle = world
        .cloth
        .attach(vertex, &world.rigid.bodies, &world.rigid.colliders)
        .unwrap();
    assert!(matches!(
        world.cloth.grasp_surface(
            point,
            GraspOptions::new(bodies[0]),
            &world.rigid.bodies,
            &world.rigid.colliders
        ),
        Err(IntegrationError::Core(ClothError::ConflictingTarget(0)))
    ));
    world.cloth.release(handle).unwrap();
    let before = world.cloth.cloth(cloth).unwrap().clone();
    assert!(
        world
            .cloth
            .grasp_surface(
                point,
                GraspOptions {
                    excluded_colliders: vec![other_collider],
                    ..GraspOptions::new(bodies[0])
                },
                &world.rigid.bodies,
                &world.rigid.colliders
            )
            .is_err()
    );
    assert_eq!(world.cloth.surface_attachments().count(), 0);
    assert_eq!(
        world.cloth.cloth(cloth).unwrap().positions(),
        before.positions()
    );
    assert_eq!(
        world.cloth.cloth(cloth).unwrap().velocities(),
        before.velocities()
    );
}

#[test]
fn later_cloth_failure_cannot_commit_a_weighted_grasp_solve() {
    let mut world = TestWorld::new();
    let first = world.grid(2, 0.1, Vec3::Y * 0.1);
    let second = world.grid(2, 0.1, Vec3::Y);
    let body = world
        .rigid
        .bodies
        .insert(RigidBodyBuilder::kinematic_position_based());
    let handle = world
        .cloth
        .grasp_surface(
            point(first, 0, [0.2, 0.3, 0.5]),
            GraspOptions::new(body),
            &world.rigid.bodies,
            &world.rigid.colliders,
        )
        .unwrap();
    let before = world.cloth.cloth(first).unwrap().clone();
    world
        .cloth
        .cloth_mut(second)
        .unwrap()
        .set_force(0, Vec3::splat(Real::MAX))
        .unwrap();
    assert!(world.tick().is_err());
    assert_eq!(
        world.cloth.cloth(first).unwrap().positions(),
        before.positions()
    );
    assert_eq!(
        world.cloth.cloth(first).unwrap().velocities(),
        before.velocities()
    );
    assert!(world.cloth.surface_attachment(handle).is_ok());
    assert_eq!(world.cloth.next_step_index(), 0);
    assert_eq!(world.cloth.drain_attachment_events().count(), 0);
}

#[test]
fn visibility_rejects_a_connected_patch_wrapping_onto_the_back_of_a_fold() {
    let mut world = TestWorld::new();
    let cloth = world.cloth.add_cloth(
        Cloth::new(
            GridBuilder::new(3, 2).size(0.2, 0.1).build().unwrap(),
            ClothMaterial::default(),
        )
        .unwrap(),
    );
    world
        .cloth
        .cloth_mut(cloth)
        .unwrap()
        .set_positions(&[
            Vec3::new(0.0, 0.001, 0.0),
            Vec3::new(0.1, 0.001, 0.0),
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.001, 0.1),
            Vec3::new(0.1, 0.001, 0.1),
            Vec3::new(0.0, 0.0, 0.1),
        ])
        .unwrap();
    let ray = SurfaceRay {
        origin: Vec3::new(0.02, 0.2, 0.03),
        direction: -Vec3::Y,
        max_distance: 1.0,
    };
    let hit = world
        .cloth
        .raycast_cloth(ray, SurfaceQueryLimits::default())
        .unwrap()
        .unwrap();
    let material = world
        .cloth
        .select_grasp_patch(hit.point, 0.21, SurfaceQueryLimits::default())
        .unwrap();
    assert!(material.vertices().contains(&2));
    assert!(matches!(
        world
            .cloth
            .select_visible_grasp_patch(ray, 0.21, SurfaceQueryLimits::default()),
        Err(ClothError::InvalidParameter(
            "grasp patch includes an occluded vertex"
        ))
    ));
    let patch = world
        .cloth
        .select_visible_grasp_patch(ray, 0.05, SurfaceQueryLimits::default())
        .unwrap()
        .unwrap();
    assert_eq!(patch.vertices(), [0]);
    assert!(matches!(
        world.cloth.select_visible_grasp_patch(
            ray,
            0.05,
            SurfaceQueryLimits {
                triangles: 4,
                ..Default::default()
            }
        ),
        Err(ClothError::SurfaceQueryBudgetExceeded { limit: 4 })
    ));
}
