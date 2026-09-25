#[path = "support/folding.rs"]
mod folding;
#[path = "support/surface_oracle.rs"]
mod oracle;
use folding::{point, real};
use rapier_cloth::{collision::geometry::triangles_intersect, *};

fn random(seed: &mut u64) -> Vec3 {
    Vec3::from_array(std::array::from_fn(|_| {
        *seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        real((*seed >> 32) as f64 / u32::MAX as f64 - 0.5)
    }))
}

#[test]
fn initial_intersection_predicate_matches_independent_triangles() {
    let mut seed = 724_u64;
    let mut intersections = 0;
    for _ in 0..3000 {
        let a = std::array::from_fn(|_| random(&mut seed));
        let b = std::array::from_fn(|_| random(&mut seed));
        let distance = oracle::triangle_distance_squared(a.map(point), b.map(point));
        // The production f32 predicate conservatively includes distances up to
        // a scale-dependent rounding tolerance. Keep the oracle comparison
        // outside that boundary; exact geometric intersections still must pass.
        if distance > 0.0 && distance < 1.0e-10 {
            continue;
        }
        let expected = distance == 0.0;
        intersections += usize::from(expected);
        assert_eq!(
            triangles_intersect(a, b),
            Some(expected),
            "{a:?}, {b:?}, reference d2={distance}"
        );
    }
    assert!(intersections > 100);
}

#[test]
fn refitted_self_collision_covers_independent_exhaustive_distance_cases() {
    // Reuse the same topology while completely relocating its vertices; rest
    // boxes cannot accidentally provide correct deformed candidate coverage.
    // Sixteen primitives force multiple hierarchy levels. Only the first and
    // last triangle approach one another, across distinct original branches.
    let rest: Vec<_> = (0..16)
        .flat_map(|i| {
            let origin = Vec3::X * (10.0 * i as Real);
            [origin, origin + Vec3::X, origin + Vec3::Z]
        })
        .collect();
    let triangles = (0..16).map(|i| [3 * i, 3 * i + 1, 3 * i + 2]).collect();
    let mut cloth = Cloth::new(
        ClothMesh::new(rest.clone(), triangles).unwrap(),
        ClothMaterial::default(),
    )
    .unwrap();
    let config = ClothContactSettings {
        thickness: 0.05,
        activation_margin: 0.01,
        ..Default::default()
    };
    cloth.set_contact_settings(Some(config)).unwrap();
    let mut solver = Solver::new();
    let mut seed = 886_u64;
    let mut counts = [0usize; 4];
    for case in 0..2000 {
        let a = std::array::from_fn(|_| random(&mut seed));
        let b = std::array::from_fn(|_| random(&mut seed));
        let mut positions = rest.clone();
        positions[..3].copy_from_slice(&a);
        positions[45..].copy_from_slice(&b);
        let distance = oracle::triangle_distance_squared(a.map(point), b.map(point)).sqrt();
        if (distance - 0.05).abs() < 1.0e-5
            || (distance - 0.06).abs() < 1.0e-5
            || (distance > 0.0 && distance < 1.0e-5)
        {
            continue;
        }
        cloth.set_positions(&positions).unwrap();
        for (i, &p) in positions.iter().enumerate() {
            cloth.pin(i as u32, p).unwrap();
        }
        let result = solver.step(
            &mut cloth,
            1.0 / 240.0,
            Vec3::ZERO,
            &SolverSettings::default(),
        );
        if distance == 0.0 {
            counts[0] += 1;
            assert!(
                matches!(
                    result,
                    Err(ClothError::InitialSelfIntersection { triangles: [0, 15] })
                ),
                "case {case}, {result:?}"
            );
        } else if distance < 0.05 {
            counts[1] += 1;
            assert!(
                matches!(result, Err(ClothError::InfeasibleSurfaceContact)),
                "case {case}, d={distance}, {result:?}"
            );
        } else {
            let report = result.unwrap_or_else(|e| panic!("case {case}, d={distance}: {e}"));
            if distance < 0.06 {
                counts[2] += 1;
                assert!(report.contacts > 0, "case {case}, d={distance}");
            } else {
                counts[3] += 1;
                assert_eq!(report.contacts, 0, "case {case}, d={distance}");
            }
        }
        assert_eq!(cloth.positions(), positions);
        assert!(cloth.velocities().iter().all(|v| *v == Vec3::ZERO));
    }
    assert!(
        counts[0] > 100 && counts[1] > 100 && counts[2] > 10 && counts[3] > 100,
        "coverage classes: {counts:?}"
    );
}
