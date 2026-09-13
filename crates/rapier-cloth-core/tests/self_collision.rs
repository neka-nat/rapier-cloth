use rapier_cloth_core::*;

const H: Real = 1.0 / 240.0;

fn layers(separation: Real) -> Cloth {
    let patch = GridBuilder::new(3, 3).size(0.1, 0.1).build().unwrap();
    let mut positions = patch.rest_positions().to_vec();
    positions.extend(
        patch
            .rest_positions()
            .iter()
            .map(|p| *p + Vec3::Y * separation),
    );
    let mut triangles = patch.triangles().to_vec();
    triangles.extend(patch.triangles().iter().map(|t| t.map(|i| i + 9)));
    let mut cloth = Cloth::new(
        ClothMesh::new(positions, triangles).unwrap(),
        ClothMaterial {
            damping: 0.0,
            ..Default::default()
        },
    )
    .unwrap();
    cloth
        .set_contact_settings(Some(ClothContactSettings::default()))
        .unwrap();
    cloth
}
fn pin_bottom(cloth: &mut Cloth) {
    for i in 0..9 {
        cloth.pin(i, cloth.positions()[i as usize]).unwrap();
    }
}
fn assert_state(a: &Cloth, b: &Cloth) {
    assert_eq!(a.positions(), b.positions());
    assert_eq!(a.previous_positions(), b.previous_positions());
    assert_eq!(a.velocities(), b.velocities());
    assert_eq!(a.pins(), b.pins());
    assert_eq!(a.contact_settings(), b.contact_settings());
    assert_eq!(a.contact_history_len(), b.contact_history_len());
}

#[test]
fn flat_sheet_has_no_false_self_support_and_collision_is_opt_in() {
    let mut cloth = Cloth::new(
        GridBuilder::new(32, 32).size(0.5, 0.5).build().unwrap(),
        ClothMaterial::default(),
    )
    .unwrap();
    assert_eq!(cloth.contact_settings(), None);
    let before = cloth.positions().to_vec();
    cloth
        .set_contact_settings(Some(ClothContactSettings::default()))
        .unwrap();
    let report = Solver::new()
        .step(&mut cloth, H, Vec3::ZERO, &SolverSettings::default())
        .unwrap();
    assert_eq!(report.contacts, 0);
    assert_eq!(cloth.contact_history_len(), 0);
    assert_eq!(cloth.positions(), before);
    assert!(report.surface_collision.candidate_pairs > 0);
    assert_eq!(report.surface_collision.retained_contacts, 0);
    assert_eq!(report.surface_collision.ccd_checks, 0); // Discrete phase only.
}

#[test]
fn small_motion_stack_is_supported_on_both_sides() {
    for side in [-1.0, 1.0] {
        let mut cloth = layers(side * 0.00105);
        pin_bottom(&mut cloth);
        let mut solver = Solver::new();
        for step in 0..240 {
            let report = solver
                .step(
                    &mut cloth,
                    H,
                    -Vec3::Y * (9.81 * side),
                    &SolverSettings::default(),
                )
                .unwrap_or_else(|e| panic!("side {side}, step {step}: {e}"));
            assert!(report.contacts > 0);
            assert!(report.max_penetration < 0.0001, "{report:?}");
            assert!(report.max_stretch < 0.01, "{report:?}");
            for p in &cloth.positions()[9..] {
                assert!((p.y * side - 0.001).abs() < 0.0001, "step {step}: {p:?}");
            }
        }
        assert!(cloth.contact_history_len() > 0);
    }
}

#[test]
fn all_fixed_thickness_conflict_and_work_overflow_are_atomic() {
    let base = layers(0.0008);
    for kind in 0..3 {
        let mut cloth = base.clone();
        if kind == 0 {
            for i in 0..cloth.positions().len() {
                cloth.pin(i as u32, cloth.positions()[i]).unwrap();
            }
        } else {
            let mut settings = cloth.contact_settings().unwrap();
            if kind == 1 {
                settings.limits.candidate_pairs = 1;
            } else {
                settings.limits.retained_contacts = 1;
            }
            cloth.set_contact_settings(Some(settings)).unwrap();
        }
        let before = cloth.clone();
        let result = Solver::new().step(&mut cloth, H, -Vec3::Y * 9.81, &SolverSettings::default());
        match kind {
            0 => assert!(matches!(result, Err(ClothError::InfeasibleSurfaceContact))),
            1 => assert!(matches!(
                result,
                Err(ClothError::CollisionBudgetExceeded {
                    kind: CollisionBudgetKind::CandidatePairs,
                    limit: 1
                })
            )),
            _ => assert!(matches!(
                result,
                Err(ClothError::CollisionBudgetExceeded {
                    kind: CollisionBudgetKind::RetainedContacts,
                    limit: 1
                })
            )),
        }
        assert_state(&cloth, &before);
    }
}

#[test]
fn initial_edge_face_piercing_is_rejected_without_committing() {
    let positions = vec![
        Vec3::new(-1.0, 0.0, -1.0),
        Vec3::new(1.0, 0.0, -1.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(-0.2, -1.0, 0.0),
        Vec3::new(-0.2, 1.0, 0.0),
        Vec3::new(0.2, 1.0, 0.0),
    ];
    let mut cloth = Cloth::new(
        ClothMesh::new(positions, vec![[0, 1, 2], [3, 4, 5]]).unwrap(),
        ClothMaterial::default(),
    )
    .unwrap();
    cloth
        .set_contact_settings(Some(ClothContactSettings::default()))
        .unwrap();
    let before = cloth.clone();
    assert!(matches!(
        Solver::new().step(&mut cloth, H, Vec3::ZERO, &SolverSettings::default()),
        Err(ClothError::InitialSelfIntersection { triangles: [0, 1] })
    ));
    assert_state(&cloth, &before);
}

#[test]
fn unequal_mass_internal_contact_preserves_center_of_mass() {
    let positions = vec![
        Vec3::new(-0.2, 0.0, -0.2),
        Vec3::new(0.2, 0.0, -0.2),
        Vec3::new(0.0, 0.0, 0.2),
        Vec3::new(-0.05, 0.0008, -0.05),
        Vec3::new(0.05, 0.0008, -0.05),
        Vec3::new(0.0, 0.0008, 0.05),
    ];
    let mut cloth = Cloth::new(
        ClothMesh::new(positions, vec![[0, 1, 2], [3, 4, 5]]).unwrap(),
        ClothMaterial {
            damping: 0.0,
            ..Default::default()
        },
    )
    .unwrap();
    cloth
        .set_contact_settings(Some(ClothContactSettings::default()))
        .unwrap();
    assert!(cloth.masses()[0] > cloth.masses()[3] * 10.0);
    let moment = |cloth: &Cloth| {
        cloth
            .positions()
            .iter()
            .zip(cloth.masses())
            .map(|(p, m)| *p * *m)
            .sum::<Vec3>()
    };
    let before = moment(&cloth);
    let report = Solver::new()
        .step(&mut cloth, H, Vec3::ZERO, &SolverSettings::default())
        .unwrap();
    assert!(report.stabilized_contacts > 0);
    assert!(moment(&cloth).distance(before) < 1.0e-9);
    assert!(cloth.positions()[0].y < 0.0 && cloth.positions()[3].y > 0.0008);
}

#[test]
fn finite_radius_connected_fold_is_valid_and_same_component_contact_is_not_excluded() {
    // A U-shaped rest strip with 5 mm bend radius and 10 mm layer separation.
    // Its two ends belong to one connected mesh, so component-wide exclusion
    // would incorrectly allow the second configuration to violate thickness.
    let rows = [
        (-0.05, 0.0),
        (-0.025, 0.0),
        (0.0, 0.0),
        (0.003535534, 0.001464466),
        (0.005, 0.005),
        (0.003535534, 0.008535534),
        (0.0, 0.01),
        (-0.025, 0.01),
        (-0.05, 0.01),
    ];
    let topology = GridBuilder::new(3, rows.len()).build().unwrap();
    let positions: Vec<_> = rows
        .iter()
        .flat_map(|&(x, y)| (0..3).map(move |z| Vec3::new(x, y, z as Real * 0.02)))
        .collect();
    let mesh = ClothMesh::new(positions.clone(), topology.triangles().to_vec()).unwrap();
    let mut cloth = Cloth::new(mesh, ClothMaterial::default()).unwrap();
    cloth
        .set_contact_settings(Some(ClothContactSettings::default()))
        .unwrap();
    for (i, &p) in positions.iter().enumerate() {
        cloth.pin(i as u32, p).unwrap();
    }
    let mut solver = Solver::new();
    let report = solver
        .step(&mut cloth, H, Vec3::ZERO, &SolverSettings::default())
        .unwrap();
    assert_eq!(report.contacts, 0);
    let mut lowered = positions;
    for p in &mut lowered[7 * 3..] {
        p.y = 0.0008;
    }
    cloth.set_positions(&lowered).unwrap();
    for (i, &p) in lowered.iter().enumerate() {
        cloth.pin(i as u32, p).unwrap();
    }
    let before = cloth.clone();
    assert!(matches!(
        solver.step(&mut cloth, H, Vec3::ZERO, &SolverSettings::default()),
        Err(ClothError::InfeasibleSurfaceContact)
    ));
    assert_state(&cloth, &before);
}

struct LateFailure;
impl ContactSource for LateFailure {
    fn contacts(
        &mut self,
        _: &[Vec3],
        _: &[Vec3],
        _: Real,
        stage: ContactStage,
        _: &mut Vec<Contact>,
    ) -> Result<(), ClothError> {
        if stage == ContactStage::Final {
            Err(ClothError::External("late failure".into()))
        } else {
            Ok(())
        }
    }
}
#[test]
fn settings_and_failed_steps_preserve_or_invalidate_history_deliberately() {
    let mut cloth = layers(0.001);
    pin_bottom(&mut cloth);
    let mut solver = Solver::new();
    solver
        .step(&mut cloth, H, -Vec3::Y * 9.81, &SolverSettings::default())
        .unwrap();
    let before = cloth.clone();
    assert!(before.contact_history_len() > 0);
    let settings = cloth.contact_settings().unwrap();
    cloth.set_contact_settings(Some(settings)).unwrap();
    assert_state(&cloth, &before);
    let mut invalid = settings;
    invalid.thickness = Real::NAN;
    assert!(cloth.set_contact_settings(Some(invalid)).is_err());
    assert_state(&cloth, &before);
    assert!(
        solver
            .step_with_contacts(
                &mut cloth,
                H,
                -Vec3::Y * 9.81,
                &SolverSettings::default(),
                &[],
                &mut LateFailure
            )
            .is_err()
    );
    assert_state(&cloth, &before);
    let mut replay = before;
    Solver::new()
        .step(&mut replay, H, -Vec3::Y * 9.81, &SolverSettings::default())
        .unwrap();
    solver
        .step(&mut cloth, H, -Vec3::Y * 9.81, &SolverSettings::default())
        .unwrap();
    assert_state(&cloth, &replay);
    let mut changed = settings;
    changed.activation_margin *= 2.0;
    cloth.set_contact_settings(Some(changed)).unwrap();
    assert_eq!(cloth.contact_history_len(), 0);
    solver
        .step(&mut cloth, H, -Vec3::Y * 9.81, &SolverSettings::default())
        .unwrap();
    assert!(cloth.contact_history_len() > 0);
    cloth.set_contact_settings(None).unwrap();
    assert_eq!(cloth.contact_history_len(), 0);
    let report = solver
        .step(&mut cloth, H, -Vec3::Y * 9.81, &SolverSettings::default())
        .unwrap();
    assert_eq!(report.surface_collision, CollisionWork::default());
}
