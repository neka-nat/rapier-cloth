#[path = "support/folding.rs"]
mod folding;
#[path = "support/surface_oracle.rs"]
mod oracle;
use folding::*;
use oracle::*;

fn grid(n: usize) -> (Vec<Point>, Vec<[u32; 3]>) {
    let mesh = rapier_cloth::GridBuilder::new(n, n)
        .size(real(0.5), real(0.5))
        .origin(rapier_cloth::Vec3::new(real(-0.25), 0.0, real(-0.25)))
        .build()
        .unwrap();
    (
        mesh.rest_positions().iter().copied().map(point).collect(),
        mesh.triangles().to_vec(),
    )
}
#[test]
fn fixture_is_fixed_and_all_six_trajectories_are_bounded() {
    let config = Config::default();
    config.validate().unwrap();
    assert_eq!((config.grid, config.variants.len()), (32, 6));
    for i in 0..config.variants.len() {
        let c = config.for_variant(i).unwrap();
        let v = &config.variants[i];
        assert!((c.end_step - c.retract_end) as f64 * c.h >= 5.0);
        for g in 0..2 {
            for step in 0..c.end_step {
                let a = gripper_target(&c, v, g, step, c.legacy_contact_radius);
                let b = gripper_target(&c, v, g, step + 1, c.legacy_contact_radius);
                assert!(a.is_finite() && b.is_finite());
                assert!(a.distance(b) < real(c.legacy_contact_radius * 0.5));
            }
        }
    }
    let world = FoldingWorld::new(config, 0).unwrap();
    let cloth = world.world.cloth(world.cloth).unwrap();
    assert_eq!(
        (cloth.positions().len(), cloth.mesh().triangles().len()),
        (1024, 1922)
    );
    let total: rapier_cloth::Real = cloth.masses().iter().sum();
    assert!((total - real(0.05)).abs() < real(1.0e-6));
}

#[test]
fn version_two_preserves_every_physical_parameter_and_commanded_trajectory() {
    let v1 = Config::with_version(1).unwrap();
    let v2 = Config::with_version(2).unwrap();
    let mut old = serde_json::to_value(&v1).unwrap();
    old["version"] = 2.into();
    assert_eq!(old, serde_json::to_value(&v2).unwrap());
    for variant in 0..v1.variants.len() {
        let a = v1.for_variant(variant).unwrap();
        let b = v2.for_variant(variant).unwrap();
        for step in 0..=a.end_step {
            for g in 0..2 {
                assert_eq!(
                    gripper_target(&a, &a.variants[variant], g, step, a.legacy_contact_radius),
                    gripper_target(&b, &b.variants[variant], g, step, b.legacy_contact_radius)
                );
            }
        }
    }
    assert!(Config::with_version(3).is_err());
}

#[test]
fn grasp_after_final_approach_does_not_command_settled_patches_into_the_table() {
    use rapier_cloth::{ClothContactSettings, ClothError, IntegrationError};
    for version in [1, 2] {
        let mut config = Config::with_version(version).unwrap();
        // A small mesh isolates the event-order bug; full 32 x 32 trajectory
        // correctness/performance belongs to the separate folding diagnostic.
        config.grid = 4;
        let mut simulation = FoldingWorld::new(config.clone(), 0).unwrap();
        simulation
            .world
            .cloth_mut(simulation.cloth)
            .unwrap()
            .set_contact_settings(Some(ClothContactSettings {
                self_collision: false,
                rigid_surface_collision: true,
                ..Default::default()
            }))
            .unwrap();
        while simulation.step + 1 < config.attach_step {
            simulation.tick().unwrap();
        }
        let last_approach = gripper_target(
            &config,
            &config.variants[0],
            0,
            config.attach_step - 1,
            config.legacy_contact_radius,
        ) - gripper_target(
            &config,
            &config.variants[0],
            0,
            config.attach_step,
            config.legacy_contact_radius,
        );
        assert!(last_approach.y > real(1.0e-5) && last_approach.y < real(1.1e-5));
        let before = simulation.positions();
        if version == 1 {
            assert!(matches!(
                simulation.tick(),
                Err(IntegrationError::Core(ClothError::InfeasibleSurfaceContact))
            ));
            assert_eq!(simulation.positions(), before);
        } else {
            let report = simulation.tick().unwrap();
            assert_eq!(simulation.step, config.attach_step);
            assert_eq!(simulation.world.attachments().count(), 2);
            assert!(report.cloths[0].1.max_target_error < real(1.0e-6));
            for g in 0..2 {
                for i in simulation.corner_patch(g) {
                    let p = simulation.positions()[i as usize];
                    assert!((p[1] - before[i as usize][1]).abs() < 1.0e-8);
                    assert!(p[1] >= config.thickness * 0.5 - 1.0e-8);
                }
            }
            for _ in 0..12 {
                simulation.tick().unwrap();
            }
        }
    }
}
#[test]
fn oracle_distinguishes_piercing_coplanar_and_separated_triangles() {
    let a = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let crossing = [[0.2, 0.2, -1.0], [0.2, 0.2, 1.0], [0.8, 0.2, 0.5]];
    assert_eq!(triangle_distance_squared(a, crossing), 0.0);
    let coplanar = [[0.2, 0.2, 0.0], [0.4, 0.2, 0.0], [0.2, 0.4, 0.0]];
    assert!(triangle_distance_squared(a, coplanar) < 1.0e-24);
    let b = a.map(|mut p| {
        p[2] = 0.002;
        p
    });
    assert!((triangle_distance_squared(a, b) - 4.0e-6).abs() < 1.0e-18);
    assert!(
        (segment_distance_squared(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            [[0.2, 0.1, 0.0], [0.8, 0.1, 0.0]]
        ) - 0.01)
            .abs()
            < 1.0e-15
    );
    // Analytically known continuous crossing: both endpoints are disjoint.
    for z in [-1.0, 1.0] {
        let moved = coplanar.map(|mut p| {
            p[2] = z;
            p
        });
        assert!((triangle_distance_squared(a, moved) - 1.0).abs() < 1.0e-12);
    }
    assert!(triangle_distance_squared(a, coplanar) < 1.0e-24);
}
#[test]
fn sweep_reference_matches_exhaustive_pairs_and_permutations() {
    let mut seed = 0x12345678_u64;
    let mut random = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        ((seed >> 32) as f64 / u32::MAX as f64) - 0.5
    };
    let positions: Vec<Point> = (0..90).map(|_| [random(), random(), random()]).collect();
    let triangles: Vec<[u32; 3]> = (0..30).map(|i| [i * 3, i * 3 + 1, i * 3 + 2]).collect();
    let audit = audit_surface(&positions, &triangles, 0.01);
    let mut crossings = 0;
    let mut deficit = 0.0_f64;
    for i in 0..triangles.len() {
        for j in i + 1..triangles.len() {
            let a = triangles[i].map(|v| positions[v as usize]);
            let b = triangles[j].map(|v| positions[v as usize]);
            let d = triangle_distance_squared(a, b).sqrt();
            assert!(
                (d - triangle_distance_squared([a[2], a[0], a[1]], [b[1], b[0], b[2]]).sqrt())
                    .abs()
                    < 1.0e-10
            );
            crossings += usize::from(d <= 1.0e-10);
            deficit = deficit.max((0.01 - d).max(0.0));
        }
    }
    assert_eq!(audit.crossing_pairs, crossings);
    assert!((audit.max_separation_deficit - deficit).abs() < 1.0e-12);
}
#[test]
fn fold_area_metric_rejects_flat_offset_and_crumpled_shapes() {
    let (rest, triangles) = grid(3);
    let flat = fold_metrics(&rest, &rest, &triangles, 3, 0.5);
    assert!((flat.projected_union_area - 0.25).abs() < 1.0e-12);
    assert_eq!(flat.overlap_ratio, 0.0);
    let folded: Vec<_> = rest
        .iter()
        .map(|p| [p[0], if p[2] < 0.0 { 0.001 } else { 0.0 }, p[2].abs()])
        .collect();
    let m = fold_metrics(&rest, &folded, &triangles, 3, 0.5);
    assert!((m.projected_union_area - 0.125).abs() < 1.0e-12);
    assert!((m.overlap_ratio - 1.0).abs() < 1.0e-12);
    assert!(m.max_corner_error < 1.0e-12);
    let shifted: Vec<_> = rest
        .iter()
        .zip(&folded)
        .map(|(r, p)| [p[0] + if r[2] < 0.0 { 0.08 } else { 0.0 }, p[1], p[2]])
        .collect();
    let offset = fold_metrics(&rest, &shifted, &triangles, 3, 0.5);
    assert!(offset.max_corner_error > 0.02);
    let crumpled: Vec<_> = folded
        .iter()
        .map(|p| [p[0] * 0.1, p[1], p[2] * 0.1])
        .collect();
    let m = fold_metrics(&rest, &crumpled, &triangles, 3, 0.5);
    assert!(m.overlap_ratio > 0.99);
    assert!(m.relative_area_error > 0.9);
    let (rest, triangles) = grid(32);
    let m = fold_metrics(&rest, &rest, &triangles, 32, 0.5);
    assert!((m.projected_half_areas[0] - 0.125).abs() < 1.0e-7);
    assert!((m.projected_half_areas[1] - 0.125).abs() < 1.0e-7);
}
#[test]
fn polygon_union_counts_overlaps_once_and_splits_crossing_edges() {
    let a = vec![vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]];
    let b = vec![vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]];
    let areas = projected_areas(&a, &b);
    assert_eq!(areas, [0.5, 0.5, 0.25]);
    let duplicate = vec![a[0].clone(), a[0].clone()];
    assert_eq!(projected_areas(&duplicate, &b), areas);
}
