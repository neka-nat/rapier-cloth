use rapier_cloth_core::{collision::ccd::*, *};

fn vertex_face() -> [Vec3; 4] {
    [
        Vec3::Y,
        Vec3::new(-1.0, 0.0, -1.0),
        Vec3::new(1.0, 0.0, -1.0),
        Vec3::Z,
    ]
}
fn query(feature: CcdFeature, start: [Vec3; 4], end: [Vec3; 4]) -> (CcdResult, CollisionWork) {
    let mut work = CollisionWork::default();
    let result = conservative_advance(
        feature,
        start,
        end,
        0.001,
        &mut work,
        CollisionLimits::default(),
    )
    .unwrap();
    (result, work)
}

#[test]
fn crossing_vertex_face_has_a_positive_conservative_prefix() {
    let start = vertex_face();
    let mut end = start;
    end[0] = -Vec3::Y;
    let (result, work) = query(CcdFeature::VertexFace, start, end);
    let fraction = result.fraction();
    assert!(matches!(result, CcdResult::Limited { .. }));
    // Both endpoints are a metre from the triangle, but the exact first
    // minimum-separation event lies at (1 - 0.001) / 2.
    assert!(fraction > 0.4 && fraction < 0.4995, "{result:?}");
    assert!(work.ccd_checks > 1 && work.ccd_checks <= 256);
    for i in 0..=100 {
        let t = fraction * i as Real / 100.0;
        assert!(1.0 - 2.0 * t > 0.001);
    }
    let (reverse, _) = query(CcdFeature::VertexFace, start.map(|p| -p), end.map(|p| -p));
    assert!((reverse.fraction() - fraction).abs() < 1.0e-6);
}

#[test]
fn two_moving_edges_stop_before_the_analytical_crossing() {
    let start = [-Vec3::X, Vec3::X, Vec3::Y - Vec3::Z, Vec3::Y + Vec3::Z];
    let end = [
        -Vec3::X + Vec3::Y * 0.25,
        Vec3::X + Vec3::Y * 0.25,
        -Vec3::Y - Vec3::Z,
        -Vec3::Y + Vec3::Z,
    ];
    let (result, _) = query(CcdFeature::EdgeEdge, start, end);
    let expected = (1.0 - 0.001) / 2.25;
    assert!(matches!(result, CcdResult::Limited { .. }));
    assert!(
        result.fraction() > expected * 0.8 && result.fraction() < expected,
        "{result:?}"
    );
    let swap = |p: [Vec3; 4]| [p[2], p[3], p[0], p[1]];
    let (symmetric, _) = query(CcdFeature::EdgeEdge, swap(start), swap(end));
    assert!((result.fraction() - symmetric.fraction()).abs() < 1.0e-6);
}

#[test]
fn tangential_and_common_translation_have_constant_cost() {
    let mut start = vertex_face();
    start[0] = Vec3::new(-0.25, 0.0011, 0.0);
    let mut end = start;
    end[0].x = 0.25;
    let (result, work) = query(CcdFeature::VertexFace, start, end);
    assert_eq!(result, CcdResult::Clear);
    assert_eq!(work.ccd_checks, 1);
    // A large common translation must not inflate the relative-motion bound.
    let (translated, work) = query(
        CcdFeature::VertexFace,
        start,
        start.map(|p| p + Vec3::X * 10000.0),
    );
    assert_eq!(translated, CcdResult::Clear);
    assert_eq!(work.ccd_checks, 1);
    let edges = [
        -Vec3::X,
        Vec3::X,
        -Vec3::X + Vec3::Y * 0.0011,
        Vec3::X + Vec3::Y * 0.0011,
    ];
    let mut sliding = edges;
    sliding[2] += Vec3::X * 100.0;
    sliding[3] += Vec3::X * 100.0;
    let (result, work) = query(CcdFeature::EdgeEdge, edges, sliding);
    assert_eq!(result, CcdResult::Clear);
    assert_eq!(work.ccd_checks, 1);
}

#[test]
fn endpoint_separation_does_not_hide_a_triangle_sweeping_over_a_vertex() {
    let start = [
        Vec3::ZERO,
        Vec3::new(-1.0, 1.0, -1.0),
        Vec3::new(1.0, 1.0, -1.0),
        Vec3::new(0.0, 1.0, 1.0),
    ];
    let mut end = start;
    for p in &mut end[1..] {
        *p -= Vec3::Y * 2.0;
    }
    let (result, _) = query(CcdFeature::VertexFace, start, end);
    assert!(matches!(result, CcdResult::Limited { .. }));
    assert!(result.fraction() < 0.4995);
}

#[test]
fn ccd_work_limit_and_unresolved_initial_gap_are_explicit_failures() {
    let start = vertex_face();
    let mut end = start;
    end[0] = -Vec3::Y;
    let limits = CollisionLimits {
        ccd_checks: 1,
        ..Default::default()
    };
    let mut work = CollisionWork::default();
    assert!(matches!(
        conservative_advance(CcdFeature::VertexFace, start, end, 0.001, &mut work, limits),
        Err(ClothError::CollisionBudgetExceeded {
            kind: CollisionBudgetKind::CcdChecks,
            limit: 1
        })
    ));
    assert_eq!(work.ccd_checks, 1);
    let mut coincident = start;
    coincident[0] = Vec3::ZERO;
    assert!(matches!(
        conservative_advance(
            CcdFeature::VertexFace,
            coincident,
            end,
            0.001,
            &mut CollisionWork::default(),
            CollisionLimits::default()
        ),
        Err(ClothError::UnresolvedContinuousCollision(_))
    ));
    let mut invalid = start;
    invalid[0].x = Real::NAN;
    assert!(matches!(
        conservative_advance(
            CcdFeature::VertexFace,
            invalid,
            end,
            0.001,
            &mut CollisionWork::default(),
            CollisionLimits::default()
        ),
        Err(ClothError::InvalidParameter(_))
    ));
    let degenerate = [Vec3::Y, Vec3::ZERO, Vec3::X, Vec3::X * 2.0];
    assert!(matches!(
        conservative_advance(
            CcdFeature::VertexFace,
            degenerate,
            degenerate,
            0.001,
            &mut CollisionWork::default(),
            CollisionLimits::default()
        ),
        Err(ClothError::DegenerateConstraint)
    ));
}

fn stacked_triangles(gap: Real) -> Cloth {
    let p = vec![
        Vec3::new(-1.0, 0.0, -1.0),
        Vec3::new(1.0, 0.0, -1.0),
        Vec3::Z,
        Vec3::new(-0.05, gap, -0.05),
        Vec3::new(0.05, gap, -0.05),
        Vec3::new(0.0, gap, 0.05),
    ];
    let mut cloth = Cloth::new(
        ClothMesh::new(p, vec![[0, 1, 2], [3, 4, 5]]).unwrap(),
        ClothMaterial {
            damping: 0.0,
            ..Default::default()
        },
    )
    .unwrap();
    cloth
        .set_contact_settings(Some(ClothContactSettings {
            continuous_self_collision: true,
            static_friction: 0.0,
            kinetic_friction: 0.0,
            ..Default::default()
        }))
        .unwrap();
    for i in 0..3 {
        cloth.pin(i, cloth.positions()[i as usize]).unwrap();
    }
    cloth
}

#[test]
fn solver_prediction_cannot_move_a_patch_through_a_pinned_layer() {
    let mut cloth = stacked_triangles(0.01);
    for i in 3..6 {
        cloth.set_velocity(i, -Vec3::Y * 4.8).unwrap();
    }
    let report = Solver::new()
        .step(
            &mut cloth,
            1.0 / 240.0,
            Vec3::ZERO,
            &SolverSettings::default(),
        )
        .unwrap();
    assert!(report.surface_collision.ccd_checks > 0);
    assert!(report.surface_collision.limited_advances > 0);
    for p in &cloth.positions()[3..] {
        assert!(p.y >= 0.0009, "{p:?}");
    }
}

#[test]
fn hard_target_crossing_fails_atomically_instead_of_clamping_the_command() {
    let mut cloth = stacked_triangles(0.01);
    let before = cloth.clone();
    let targets: Vec<_> = (3..6)
        .map(|i| Target {
            particle: i,
            position: cloth.positions()[i as usize] - Vec3::Y * 0.02,
            compliance: 0.0,
        })
        .collect();
    let result = Solver::new().step_with_contacts(
        &mut cloth,
        1.0 / 240.0,
        Vec3::ZERO,
        &SolverSettings::default(),
        &targets,
        &mut NoContacts,
    );
    assert!(
        matches!(
            result,
            Err(ClothError::InfeasibleSurfaceContact | ClothError::ConflictingTarget(_))
        ),
        "{result:?}"
    );
    assert_eq!(cloth.positions(), before.positions());
    assert_eq!(cloth.velocities(), before.velocities());
    assert_eq!(cloth.contact_history_len(), before.contact_history_len());
}

#[test]
fn zero_friction_prediction_preserves_tangential_motion() {
    let mut cloth = stacked_triangles(0.001);
    for i in 3..6 {
        cloth.set_velocity(i, Vec3::X * 0.1).unwrap();
    }
    let before = cloth.positions().to_vec();
    Solver::new()
        .step(
            &mut cloth,
            1.0 / 240.0,
            -Vec3::Y * 9.81,
            &SolverSettings::default(),
        )
        .unwrap();
    for (i, initial) in before.iter().enumerate().skip(3) {
        assert!(
            (cloth.velocities()[i].x - 0.1).abs() < 1.0e-5,
            "velocity {:?}",
            cloth.velocities()[i]
        );
        assert!((cloth.positions()[i].x - initial.x - 0.1 / 240.0).abs() < 1.0e-6);
    }
}

#[test]
fn exhausted_ccd_budget_does_not_commit_a_partial_prediction() {
    let mut cloth = stacked_triangles(0.01);
    let mut config = cloth.contact_settings().unwrap();
    config.limits.ccd_checks = 1;
    cloth.set_contact_settings(Some(config)).unwrap();
    for i in 3..6 {
        cloth.set_velocity(i, -Vec3::Y * 4.8).unwrap();
    }
    let before = cloth.clone();
    let result = Solver::new().step(
        &mut cloth,
        1.0 / 240.0,
        Vec3::ZERO,
        &SolverSettings::default(),
    );
    assert!(
        matches!(
            result,
            Err(ClothError::CollisionBudgetExceeded {
                kind: CollisionBudgetKind::CcdChecks,
                limit: 1
            })
        ),
        "{result:?}"
    );
    assert_eq!(cloth.positions(), before.positions());
    assert_eq!(cloth.previous_positions(), before.previous_positions());
    assert_eq!(cloth.velocities(), before.velocities());
    assert_eq!(cloth.contact_history_len(), before.contact_history_len());
}

#[test]
fn ccd_prediction_retains_the_full_normal_support_for_kinetic_friction() {
    let mut cloth = stacked_triangles(0.001);
    let mut config = cloth.contact_settings().unwrap();
    config.static_friction = 0.5;
    config.kinetic_friction = 0.5;
    cloth.set_contact_settings(Some(config)).unwrap();
    for i in 3..6 {
        cloth.set_velocity(i, Vec3::X * 0.1).unwrap();
    }
    Solver::new()
        .step(
            &mut cloth,
            1.0 / 240.0,
            -Vec3::Y * 9.81,
            &SolverSettings::default(),
        )
        .unwrap();
    let expected = 0.1 - 0.5 * 9.81 / 240.0;
    for v in &cloth.velocities()[3..] {
        assert!(
            (v.x - expected).abs() < 1.0e-5,
            "{v:?}, expected vx={expected}"
        );
    }
}

#[test]
fn elastic_stretch_correction_cannot_cross_a_nonlocal_patch() {
    let rest = vec![
        Vec3::new(0.3, 0.0, -0.3),
        Vec3::new(0.9, 0.0, -0.3),
        Vec3::new(0.6, 0.0, 0.3),
        Vec3::new(-1.0, 0.1, 0.0),
        Vec3::new(-1.0, -0.2, 0.0),
        Vec3::new(0.6, -0.1, 0.0),
    ];
    let mut deformed = rest.clone();
    deformed[5].y = 1.0;
    let mut cloth = Cloth::new(
        ClothMesh::new(rest, vec![[0, 1, 2], [3, 4, 5]]).unwrap(),
        ClothMaterial {
            damping: 0.0,
            ..Default::default()
        },
    )
    .unwrap();
    cloth.set_positions(&deformed).unwrap();
    for i in 0..5 {
        cloth.pin(i, cloth.positions()[i as usize]).unwrap();
    }
    let settings = SolverSettings {
        iterations: 512,
        ..Default::default()
    };
    let mut legacy = cloth.clone();
    Solver::new()
        .step(&mut legacy, 1.0 / 240.0, Vec3::ZERO, &settings)
        .unwrap();
    assert!(
        legacy.positions()[5].y < -0.02,
        "legacy stretch control: {:?}",
        legacy.positions()[5]
    );
    cloth
        .set_contact_settings(Some(ClothContactSettings {
            continuous_self_collision: true,
            ..Default::default()
        }))
        .unwrap();
    let before = cloth.clone();
    match Solver::new().step(&mut cloth, 1.0 / 240.0, Vec3::ZERO, &settings) {
        Ok(report) => {
            assert!(cloth.positions()[5].y >= 0.0009);
            assert!(report.surface_collision.limited_advances > 0);
        }
        Err(ClothError::UnresolvedContinuousCollision(_)) => {
            assert_eq!(cloth.positions(), before.positions())
        }
        Err(e) => panic!("unexpected stretch failure: {e}"),
    }
}

#[test]
fn dihedral_correction_cannot_fold_through_an_unrelated_patch() {
    let rest = vec![
        Vec3::new(-0.5, 0.0, 0.3),
        Vec3::new(0.5, 0.0, 0.3),
        Vec3::new(0.0, 0.0, 1.4),
        Vec3::new(-0.1, 0.1, -1.0),
        Vec3::new(0.1, 0.1, -1.0),
        Vec3::new(0.0, 0.1, -2.0),
        Vec3::new(0.0, -0.1, 0.6),
    ];
    let mut deformed = rest.clone();
    deformed[6].y = 1.0;
    let mut cloth = Cloth::new(
        ClothMesh::new(rest, vec![[0, 1, 2], [3, 4, 5], [4, 3, 6]]).unwrap(),
        ClothMaterial {
            stretch_compliance: 1.0e12,
            bend_compliance: 0.0,
            damping: 0.0,
            ..Default::default()
        },
    )
    .unwrap();
    cloth.set_positions(&deformed).unwrap();
    for i in 0..6 {
        cloth.pin(i, cloth.positions()[i as usize]).unwrap();
    }
    let mut legacy = cloth.clone();
    Solver::new()
        .step(
            &mut legacy,
            1.0 / 240.0,
            Vec3::ZERO,
            &SolverSettings::default(),
        )
        .unwrap();
    assert!(
        legacy.positions()[6].y < -0.02,
        "legacy bend control: {:?}",
        legacy.positions()[6]
    );
    cloth
        .set_contact_settings(Some(ClothContactSettings {
            continuous_self_collision: true,
            ..Default::default()
        }))
        .unwrap();
    let report = Solver::new()
        .step(
            &mut cloth,
            1.0 / 240.0,
            Vec3::ZERO,
            &SolverSettings::default(),
        )
        .unwrap();
    assert!(
        cloth.positions()[6].y >= 0.0009,
        "{:?}",
        cloth.positions()[6]
    );
    assert!(report.surface_collision.limited_advances > 0);
}

struct PushOnce {
    legacy: bool,
    stage: ContactStage,
    fired: bool,
}
impl ContactSource for PushOnce {
    fn contacts(
        &mut self,
        _: &[Vec3],
        _: &[Vec3],
        _: Real,
        stage: ContactStage,
        out: &mut Vec<Contact>,
    ) -> Result<(), ClothError> {
        if self.legacy && !self.fired && stage == self.stage {
            self.fired = true;
            out.push(Contact {
                key: ContactKey {
                    particle: 3,
                    external: 42,
                    feature: 0,
                },
                point: -Vec3::Y * 0.02,
                normal: -Vec3::Y,
                surface_velocity: Vec3::ZERO,
                friction: 0.0,
            });
        }
        Ok(())
    }
    fn surface_contacts(
        &mut self,
        _: &[Vec3],
        _: &[Vec3],
        stage: ContactStage,
        out: &mut Vec<SurfaceContact>,
    ) -> Result<(), ClothError> {
        if !self.legacy && !self.fired && stage == self.stage {
            self.fired = true;
            out.push(SurfaceContact {
                key: SurfaceContactKey {
                    other_cloth: None,
                    features: [
                        SurfaceFeature::Vertex(3),
                        SurfaceFeature::External {
                            object: 42,
                            feature: 0,
                        },
                    ],
                },
                particles: [3, 0, 0, 0],
                weights: [1.0, 0.0, 0.0, 0.0],
                normal: -Vec3::Y,
                offset: -Vec3::Y * 0.02,
                surface_velocity: Vec3::ZERO,
                separation: 0.001,
                static_friction: 0.0,
                kinetic_friction: 0.0,
            });
        }
        Ok(())
    }
}
#[test]
fn stabilization_and_both_contact_projection_paths_are_bounded() {
    for legacy in [true, false] {
        for stage in [ContactStage::Stabilization, ContactStage::Iteration] {
            let mut cloth = stacked_triangles(0.01);
            let mut push = PushOnce {
                legacy,
                stage,
                fired: false,
            };
            let report = Solver::new()
                .step_with_contacts(
                    &mut cloth,
                    1.0 / 240.0,
                    Vec3::ZERO,
                    &SolverSettings::default(),
                    &[],
                    &mut push,
                )
                .unwrap();
            assert!(push.fired);
            assert!(
                report.surface_collision.limited_advances > 0,
                "legacy={legacy}, stage={stage:?}"
            );
            for p in &cloth.positions()[3..] {
                assert!(p.y >= 0.0009, "legacy={legacy}, stage={stage:?}: {p:?}");
            }
        }
    }
}
