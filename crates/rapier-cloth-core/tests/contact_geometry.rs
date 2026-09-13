use rapier_cloth_core::{collision::geometry::*, *};

#[test]
fn near_vertex_closest_point_keeps_nonnegative_barycentric_weights() {
    // A rotating support produced these finite coordinates. The point is just
    // inside the third vertex; subtracting two rounded weights from one used
    // to return -5.96e-8 for the first weight in f32.
    let triangle = [
        Vec3::ZERO,
        Vec3::new(-0.019999683, 0.0, 0.020000324),
        Vec3::new(3.2782555e-7, 0.0, 0.02),
    ];
    let p = Vec3::new(3.2697687e-7, 0.0, 0.02);
    let witness = closest_triangle(p, triangle).unwrap();
    assert!(witness.barycentric.iter().all(|w| (0.0..=1.0).contains(w)));
    assert!(witness.point.distance(p) < 1.0e-8);
    SurfaceWitness::from_triangle([0, 1, 2], 0, witness.barycentric).unwrap();
}

#[test]
fn canonical_surface_witnesses_match_shared_edges_and_validate_barycentrics() {
    let a = SurfaceWitness::from_triangle([7, 3, 2], 0, [0.25, 0.75, 0.0]).unwrap();
    let b = SurfaceWitness::from_triangle([3, 7, 9], 1, [0.75, 0.25, 0.0]).unwrap();
    assert_eq!(a, b);
    assert_eq!(a.feature, SurfaceFeature::Edge([3, 7]));
    assert_eq!(a.particles, [3, 7, 0]);
    assert_eq!(a.weights, [0.75, 0.25, 0.0]);
    let vertex = SurfaceWitness::from_triangle([7, 3, 2], 0, [0.0, 1.0, 0.0]).unwrap();
    assert_eq!(vertex.feature, SurfaceFeature::Vertex(3));
    let interior = SurfaceWitness::from_triangle([7, 3, 2], 5, [0.25, 0.5, 0.25]).unwrap();
    assert_eq!(interior.feature, SurfaceFeature::Face(5));
    for barycentric in [[Real::NAN, 0.0, 1.0], [-0.1, 0.6, 0.5], [0.0; 3], [0.5; 3]] {
        assert!(SurfaceWitness::from_triangle([7, 3, 2], 0, barycentric).is_err());
    }
    assert!(SurfaceWitness::from_triangle([7, 7, 2], 0, [0.25, 0.5, 0.25]).is_err());
}

fn contact() -> SurfaceContact {
    SurfaceContact {
        key: SurfaceContactKey {
            other_cloth: None,
            features: [SurfaceFeature::Vertex(0), SurfaceFeature::Face(0)],
        },
        particles: [0, 1, 2, 3],
        weights: [1.0, -0.5, -0.25, -0.25],
        normal: Vec3::Y,
        offset: Vec3::ZERO,
        surface_velocity: Vec3::ZERO,
        separation: 0.01,
        static_friction: 0.6,
        kinetic_friction: 0.5,
    }
}
#[test]
fn triangle_and_segment_witnesses_cover_feature_regions() {
    let triangle = [Vec3::ZERO, Vec3::X, Vec3::Z];
    let interior = closest_triangle(Vec3::new(0.2, 0.3, 0.3), triangle).unwrap();
    assert!((interior.point - Vec3::new(0.2, 0.0, 0.3)).length() < 1.0e-6);
    for (p, w) in [
        (Vec3::new(-1.0, 0.2, -1.0), [1.0, 0.0, 0.0]),
        (Vec3::new(0.5, 0.1, -1.0), [0.5, 0.5, 0.0]),
        (Vec3::new(1.0, 0.1, 1.0), [0.0, 0.5, 0.5]),
    ] {
        let found = closest_triangle(p, triangle).unwrap();
        for (a, b) in found.barycentric.into_iter().zip(w) {
            assert!((a - b).abs() < 1.0e-6);
        }
    }
    assert!(closest_triangle(Vec3::ZERO, [Vec3::ZERO; 3]).is_none());
    assert!(closest_triangle(Vec3::splat(Real::NAN), triangle).is_none());
    let crossing = closest_segments(
        [Vec3::ZERO, Vec3::X],
        [Vec3::new(0.5, 0.0, -1.0), Vec3::new(0.5, 0.0, 1.0)],
    )
    .unwrap();
    assert!(crossing.a.distance(crossing.b) < 1.0e-6);
    let parallel = closest_segments([Vec3::ZERO, Vec3::X], [Vec3::Z, Vec3::X + Vec3::Z]).unwrap();
    assert!((parallel.a.distance(parallel.b) - 1.0).abs() < 1.0e-6);
    let point = closest_segments([Vec3::ZERO; 2], [Vec3::X, Vec3::X * 2.0]).unwrap();
    assert_eq!(point.parameters, [0.0, 0.0]);
}
#[test]
fn distance_gradient_matches_finite_difference_away_from_feature_boundaries() {
    let triangle = [Vec3::ZERO, Vec3::X, Vec3::Z];
    let point = Vec3::new(0.2, 0.3, 0.3);
    let witness = closest_triangle(point, triangle).unwrap();
    let n = (point - witness.point).normalize();
    let gradients = [
        n,
        -n * witness.barycentric[0],
        -n * witness.barycentric[1],
        -n * witness.barycentric[2],
    ];
    let positions = [point, triangle[0], triangle[1], triangle[2]];
    let eps: Real = if cfg!(feature = "f64") {
        1.0e-6
    } else {
        1.0e-3
    };
    for particle in 0..4 {
        for axis in 0..3 {
            let mut a = positions;
            let mut b = positions;
            a[particle][axis] += eps;
            b[particle][axis] -= eps;
            let da = a[0].distance(closest_triangle(a[0], [a[1], a[2], a[3]]).unwrap().point);
            let db = b[0].distance(closest_triangle(b[0], [b[1], b[2], b[3]]).unwrap().point);
            assert!(((da - db) / (2.0 * eps) - gradients[particle][axis]).abs() < 1.0e-4);
        }
    }
}
#[test]
fn projection_respects_mass_ratios_and_pair_momentum() {
    let c = contact();
    let mut p = [
        Vec3::ZERO,
        Vec3::new(-0.25, 0.0, 0.0),
        Vec3::new(0.25, 0.0, -0.25),
        Vec3::new(0.25, 0.0, 0.25),
    ];
    let weights = [1.0, 0.5, 0.25, 2.0];
    let before = p;
    let mut lambda = 0.0;
    c.project(&mut p, &weights, &mut lambda).unwrap();
    let relative: Vec3 = p.iter().zip(c.weights).map(|(p, w)| *p * w).sum();
    assert!((relative.y - c.separation).abs() < 1.0e-7);
    let momentum: Vec3 = p
        .iter()
        .zip(before)
        .zip(weights)
        .map(|((p, b), w)| (*p - b) / w)
        .sum();
    assert!(momentum.length() < 1.0e-7);
    let angular: Vec3 = p
        .iter()
        .zip(before)
        .zip(weights)
        .map(|((p, b), w)| b.cross((*p - b) / w))
        .sum();
    assert!(angular.length() < 1.0e-7);
    for i in 0..4 {
        assert!((p[i].y - lambda * c.weights[i] * weights[i]).abs() < 1.0e-7);
    }
    let mut fixed = before;
    let mut l = 0.0;
    assert_eq!(
        c.project(&mut fixed, &[0.0; 4], &mut l),
        Err(ClothError::InfeasibleSurfaceContact)
    );
    assert_eq!(fixed, before);
    assert_eq!(l, 0.0);
    let old = p;
    assert!(c.project(&mut p, &[1.0; 3], &mut lambda).is_err());
    assert_eq!(p, old);
}

#[test]
fn edge_distance_gradient_and_invalid_projection_inputs() {
    let p = [
        -Vec3::X,
        Vec3::X,
        Vec3::new(0.0, 0.3, -1.0),
        Vec3::new(0.0, 0.3, 1.0),
    ];
    let witness = closest_segments([p[0], p[1]], [p[2], p[3]]).unwrap();
    let n = (witness.a - witness.b).normalize();
    let [s, t] = witness.parameters;
    let gradients = [n * (1.0 - s), n * s, -n * (1.0 - t), -n * t];
    let eps: Real = if cfg!(feature = "f64") {
        1.0e-6
    } else {
        1.0e-3
    };
    for i in 0..4 {
        for axis in 0..3 {
            let mut a = p;
            let mut b = p;
            a[i][axis] += eps;
            b[i][axis] -= eps;
            let wa = closest_segments([a[0], a[1]], [a[2], a[3]]).unwrap();
            let wb = closest_segments([b[0], b[1]], [b[2], b[3]]).unwrap();
            let derivative = (wa.a.distance(wa.b) - wb.a.distance(wb.b)) / (2.0 * eps);
            assert!((derivative - gradients[i][axis]).abs() < 1.0e-4);
        }
    }
    for invalid in 0..4 {
        let mut c = contact();
        match invalid {
            0 => c.particles[0] = u32::MAX,
            1 => c.normal = Vec3::splat(Real::NAN),
            2 => c.particles[1] = c.particles[0],
            _ => c.weights[0] = Real::INFINITY,
        }
        let mut positions = p;
        let mut lambda = 0.0;
        assert!(c.project(&mut positions, &[1.0; 4], &mut lambda).is_err());
        assert_eq!(positions, p);
        assert_eq!(lambda, 0.0);
    }
}

#[test]
fn surface_contact_budget_does_not_discard_contacts_or_commit_state() {
    struct TooMany;
    impl ContactSource for TooMany {
        fn contacts(
            &mut self,
            _: &[Vec3],
            _: &[Vec3],
            _: Real,
            _: ContactStage,
            _: &mut Vec<Contact>,
        ) -> Result<(), ClothError> {
            Ok(())
        }
        fn surface_contacts(
            &mut self,
            _: &[Vec3],
            _: &[Vec3],
            _: ContactStage,
            out: &mut Vec<SurfaceContact>,
        ) -> Result<(), ClothError> {
            let a = particle_plane();
            let mut b = a;
            b.key.features[1] = SurfaceFeature::External {
                object: 2,
                feature: 0,
            };
            out.extend([a, b]);
            Ok(())
        }
    }
    let mut cloth = cloth();
    let before = cloth.positions().to_vec();
    let settings = SolverSettings {
        max_contacts: 1,
        ..Default::default()
    };
    assert!(matches!(
        Solver::new().step_with_contacts(
            &mut cloth,
            1.0 / 240.0,
            -Vec3::Y * 9.81,
            &settings,
            &[],
            &mut TooMany
        ),
        Err(ClothError::ContactBudgetExceeded { limit: 1 })
    ));
    assert_eq!(cloth.positions(), before);
    assert_eq!(cloth.contact_history_len(), 0);
}

struct Source {
    contact: SurfaceContact,
    fail: bool,
    active: bool,
}
impl ContactSource for Source {
    fn contacts(
        &mut self,
        _: &[Vec3],
        _: &[Vec3],
        _: Real,
        _: ContactStage,
        _: &mut Vec<Contact>,
    ) -> Result<(), ClothError> {
        Ok(())
    }
    fn surface_contacts(
        &mut self,
        _: &[Vec3],
        _: &[Vec3],
        stage: ContactStage,
        out: &mut Vec<SurfaceContact>,
    ) -> Result<(), ClothError> {
        if self.fail && stage == ContactStage::Final {
            return Err(ClothError::External("injected late failure".into()));
        }
        if self.active {
            out.push(self.contact);
        }
        Ok(())
    }
}
fn particle_plane() -> SurfaceContact {
    SurfaceContact {
        particles: [0, 0, 0, 0],
        weights: [1.0, 0.0, 0.0, 0.0],
        key: SurfaceContactKey {
            other_cloth: None,
            features: [
                SurfaceFeature::Vertex(0),
                SurfaceFeature::External {
                    object: 1,
                    feature: 0,
                },
            ],
        },
        ..contact()
    }
}
fn cloth() -> Cloth {
    Cloth::new(
        GridBuilder::new(2, 2)
            .origin(Vec3::Y * 0.01)
            .build()
            .unwrap(),
        ClothMaterial {
            damping: 0.0,
            ..Default::default()
        },
    )
    .unwrap()
}
#[test]
fn history_is_owned_by_cloth_and_failure_or_teleport_does_not_leak_state() {
    let mut solver = Solver::new();
    let settings = SolverSettings::default();
    let mut a = cloth();
    let mut b = cloth();
    let mut source = Source {
        contact: particle_plane(),
        fail: false,
        active: true,
    };
    solver
        .step_with_contacts(
            &mut a,
            1.0 / 240.0,
            -Vec3::Y * 9.81,
            &settings,
            &[],
            &mut source,
        )
        .unwrap();
    assert_eq!(a.contact_history_len(), 1);
    assert_eq!(b.contact_history_len(), 0);
    let checkpoint = a.clone();
    let before = a.positions().to_vec();
    let velocity = a.velocities().to_vec();
    source.fail = true;
    assert!(
        solver
            .step_with_contacts(
                &mut a,
                1.0 / 240.0,
                -Vec3::Y * 9.81,
                &settings,
                &[],
                &mut source
            )
            .is_err()
    );
    assert_eq!(a.positions(), before);
    assert_eq!(a.velocities(), velocity);
    assert_eq!(a.contact_history_len(), 1);
    solver
        .step(&mut b, 1.0 / 240.0, Vec3::ZERO, &settings)
        .unwrap();
    assert_eq!(b.contact_history_len(), 0);
    assert_eq!(a.contact_history_len(), 1);
    source.fail = false;
    let mut replay = checkpoint.clone();
    solver
        .step_with_contacts(
            &mut a,
            1.0 / 240.0,
            -Vec3::Y * 9.81,
            &settings,
            &[],
            &mut source,
        )
        .unwrap();
    Solver::new()
        .step_with_contacts(
            &mut replay,
            1.0 / 240.0,
            -Vec3::Y * 9.81,
            &settings,
            &[],
            &mut source,
        )
        .unwrap();
    assert_eq!(a.positions(), replay.positions());
    assert_eq!(a.velocities(), replay.velocities());
    assert!(a.set_positions(&[Vec3::ZERO]).is_err());
    assert_eq!(a.contact_history_len(), 1);
    a.set_positions(&before).unwrap();
    assert_eq!(a.contact_history_len(), 0);
    assert_eq!(checkpoint.contact_history_len(), 1);
    source.active = false;
    solver
        .step_with_contacts(
            &mut replay,
            1.0 / 240.0,
            -Vec3::Y * 9.81,
            &settings,
            &[],
            &mut source,
        )
        .unwrap();
    assert_eq!(replay.contact_history_len(), 0);
    assert!(
        replay.velocities()[0].y < 0.0,
        "a disappeared contact must not support the cloth"
    );
}
