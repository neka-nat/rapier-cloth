mod support;
use rapier::prelude::*;
use rapier_cloth::Real;
use rapier_cloth::*;
use support::TestWorld;

#[test]
fn supported_primitives_stop_a_falling_cloth() {
    for shape in 0..4 {
        let mut w = TestWorld::new();
        let builder = match shape {
            0 => ColliderBuilder::ball(0.5),
            1 => ColliderBuilder::cuboid(0.5, 0.5, 0.5),
            2 => ColliderBuilder::capsule_y(0.3, 0.5),
            _ => ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)),
        };
        w.rigid.colliders.insert(builder.friction(0.0));
        let h = w.grid(4, 0.12, Vec3::new(-0.06, 1.1, -0.06));
        let mut max_depth: Real = 0.0;
        for _ in 0..300 {
            let r = w.tick().unwrap();
            max_depth = max_depth.max(r.cloths[0].1.max_penetration);
        }
        let c = w.cloth.cloth(h).unwrap();
        let min_dist = c
            .positions()
            .iter()
            .map(|p| match shape {
                0 => p.length() - 0.5,
                1 => {
                    let q = p.abs() - Vec3::splat(0.5);
                    q.max(Vec3::ZERO).length() + q.max_element().min(0.0)
                }
                2 => Vec3::new(p.x, (p.y.abs() - 0.3).max(0.0), p.z).length() - 0.5,
                _ => p.y,
            })
            .reduce(Real::min)
            .unwrap();
        assert!(
            min_dist >= 0.004 - 1.0e-5,
            "shape {shape}: distance {min_dist}"
        );
        assert!(
            max_depth <= 0.001 + 1.0e-5,
            "shape {shape}: depth {max_depth}"
        );
        assert!(c.velocities().iter().all(|v| v.is_finite()));
    }
}

#[test]
fn initial_penetration_is_restored_without_artificial_launch() {
    let mut w = TestWorld::new();
    w.rigid.gravity = Vec3::ZERO;
    w.rigid
        .colliders
        .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)));
    let h = w.grid(2, 0.1, Vec3::new(0.0, -0.2, 0.0));
    let r = w.tick().unwrap();
    assert!(r.cloths[0].1.stabilized_contacts > 0);
    let c = w.cloth.cloth(h).unwrap();
    assert!(c.positions().iter().all(|p| (p.y - 0.005).abs() < 1.0e-5));
    assert!(c.velocities().iter().all(|v| v.length() < 1.0e-4));
}

#[test]
fn floor_and_wall_are_both_resolved_without_attraction() {
    let mut w = TestWorld::new();
    w.rigid.gravity = Vec3::ZERO;
    w.rigid
        .colliders
        .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)));
    w.rigid
        .colliders
        .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::X)));
    let h = w.grid(2, 0.1, Vec3::new(-0.04, -0.05, 0.0));
    w.tick().unwrap();
    assert!(
        w.cloth
            .cloth(h)
            .unwrap()
            .positions()
            .iter()
            .all(|p| p.x >= 0.004 && p.y >= 0.004)
    );
    let mut free = TestWorld::new();
    free.rigid.gravity = Vec3::ZERO;
    free.rigid
        .colliders
        .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)));
    let h = free.grid(2, 0.1, Vec3::Y);
    let before = free.cloth.cloth(h).unwrap().positions().to_vec();
    free.tick().unwrap();
    assert_eq!(free.cloth.cloth(h).unwrap().positions(), before);
}

#[test]
fn contradictory_pin_is_reported_and_state_is_not_committed() {
    let mut w = TestWorld::new();
    w.rigid
        .colliders
        .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)));
    let h = w.grid(2, 0.1, Vec3::new(0.0, -0.1, 0.0));
    w.cloth
        .cloth_mut(h)
        .unwrap()
        .pin(0, Vec3::new(0.0, -0.1, 0.0))
        .unwrap();
    let before = w.cloth.cloth(h).unwrap().positions().to_vec();
    assert!(w.tick().is_err());
    assert!(w.cloth.is_desynchronized());
    assert_eq!(w.cloth.cloth(h).unwrap().positions(), before);
}

#[test]
fn sphere_center_overlap_either_recovers_or_reports_a_defined_failure() {
    let mut w = TestWorld::new();
    w.rigid.gravity = Vec3::ZERO;
    w.rigid.colliders.insert(ColliderBuilder::ball(0.5));
    let handle = w.grid(2, 0.1, Vec3::ZERO);
    let before = w.cloth.cloth(handle).unwrap().positions().to_vec();
    match w.tick() {
        Ok(_) => assert!(
            w.cloth
                .cloth(handle)
                .unwrap()
                .positions()
                .iter()
                .all(|p| p.is_finite() && p.length() >= 0.504)
        ),
        Err(IntegrationError::Core(ClothError::DegenerateConstraint)) => {
            assert_eq!(w.cloth.cloth(handle).unwrap().positions(), before)
        }
        other => panic!("unexpected sphere-center result: {other:?}"),
    }
}
