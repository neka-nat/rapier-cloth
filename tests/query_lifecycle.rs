mod support;
use rapier::prelude::*;
use rapier_cloth::*;
use support::TestWorld;

#[test]
fn sensors_exclusions_groups_and_predicate_filter_contacts() {
    for mode in 0..4 {
        let mut w = TestWorld::new();
        w.rigid.gravity = Vec3::ZERO;
        let floor = w.rigid.colliders.insert(
            ColliderBuilder::cuboid(2.0, 0.1, 2.0)
                .sensor(mode == 0)
                .collision_groups(InteractionGroups::new(
                    Group::GROUP_1,
                    Group::GROUP_1,
                    InteractionTestMode::And,
                )),
        );
        if mode == 1 {
            w.cloth.collision_settings.excluded_colliders.push(floor);
        }
        let h = w.grid(2, 0.1, Vec3::ZERO);
        let predicate = |handle: ColliderHandle, _: &Collider| handle != floor;
        let filter = match mode {
            2 => QueryFilter::default().groups(InteractionGroups::new(
                Group::GROUP_2,
                Group::GROUP_2,
                InteractionTestMode::And,
            )),
            3 => QueryFilter {
                predicate: Some(&predicate),
                ..Default::default()
            },
            _ => QueryFilter::default(),
        };
        let before = w.cloth.cloth(h).unwrap().positions().to_vec();
        let report = w.tick_filtered(filter).unwrap();
        assert_eq!(
            w.cloth.cloth(h).unwrap().positions(),
            before,
            "filter mode {mode}"
        );
        if mode <= 1 {
            assert!(report.ignored_colliders.contains(&floor));
        }
    }
}

#[test]
fn collider_insertion_move_and_removal_are_visible_after_rapier_step() {
    let mut w = TestWorld::new();
    w.rigid.gravity = Vec3::ZERO;
    let h = w.grid(2, 0.1, Vec3::ZERO);
    w.tick().unwrap();
    let floor = w
        .rigid
        .colliders
        .insert(ColliderBuilder::cuboid(1.0, 0.1, 1.0).translation(Vec3::new(0.0, -0.1, 0.0)));
    w.tick().unwrap();
    assert!(w.cloth.cloth(h).unwrap().positions()[0].y >= 0.004);
    w.rigid.colliders[floor].set_translation(Vec3::new(0.0, 0.0, 0.0));
    w.tick().unwrap();
    assert!(w.cloth.cloth(h).unwrap().positions()[0].y >= 0.104);
    w.rigid
        .colliders
        .remove(floor, &mut w.rigid.islands, &mut w.rigid.bodies, true);
    for i in 0..4 {
        w.cloth
            .cloth_mut(h)
            .unwrap()
            .set_velocity(i, -Vec3::Y)
            .unwrap();
    }
    w.tick().unwrap();
    assert!(w.cloth.cloth(h).unwrap().positions()[0].y < 0.104);
}

#[test]
fn nearby_unsupported_shapes_fail_but_distant_ones_do_not() {
    for near in [false, true] {
        let mut w = TestWorld::new();
        w.rigid.gravity = Vec3::ZERO;
        w.rigid
            .colliders
            .insert(ColliderBuilder::cylinder(0.5, 0.5).translation(if near {
                Vec3::ZERO
            } else {
                Vec3::splat(100.0)
            }));
        let h = w.grid(2, 0.1, Vec3::ZERO);
        let before = w.cloth.cloth(h).unwrap().positions().to_vec();
        let result = w.tick();
        assert_eq!(result.is_err(), near);
        if near {
            assert!(matches!(
                result,
                Err(IntegrationError::UnsupportedCollision { .. })
            ));
            assert_eq!(w.cloth.cloth(h).unwrap().positions(), before);
        }
    }
}

#[test]
fn disabled_colliders_and_explicitly_excluded_dynamic_bodies_do_not_collide() {
    for disabled in [false, true] {
        let mut w = TestWorld::new();
        w.rigid.gravity = Vec3::ZERO;
        let body = w.rigid.bodies.insert(RigidBodyBuilder::dynamic());
        let c = w.rigid.colliders.insert_with_parent(
            ColliderBuilder::ball(0.5).enabled(!disabled),
            body,
            &mut w.rigid.bodies,
        );
        if !disabled {
            w.cloth.collision_settings.excluded_colliders.push(c);
        }
        let cloth = w.grid(2, 0.1, Vec3::ZERO);
        let previous = w.cloth.cloth(cloth).unwrap().positions().to_vec();
        w.tick().unwrap();
        assert_eq!(w.cloth.cloth(cloth).unwrap().positions(), previous);
    }
    let mut w = TestWorld::new();
    let body = w.rigid.bodies.insert(RigidBodyBuilder::dynamic());
    w.rigid
        .colliders
        .insert_with_parent(ColliderBuilder::ball(0.5), body, &mut w.rigid.bodies);
    w.grid(2, 0.1, Vec3::ZERO);
    assert!(matches!(
        w.tick(),
        Err(IntegrationError::UnsupportedCollision { .. })
    ));
}
