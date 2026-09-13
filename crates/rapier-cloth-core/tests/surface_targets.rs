use rapier_cloth_core::*;

fn cloth() -> Cloth {
    Cloth::new(
        ClothMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Z], vec![[0, 2, 1]]).unwrap(),
        ClothMaterial {
            damping: 0.0,
            ..Default::default()
        },
    )
    .unwrap()
}

#[test]
fn hard_surface_target_holds_the_point_but_preserves_unconstrained_rotation() {
    let mut cloth = cloth();
    let point = SurfacePoint::new(0, [0.2, 0.3, 0.5]).unwrap();
    let center = cloth.surface().point_position(point).unwrap();
    let start = cloth.positions().to_vec();
    for (i, p) in start.iter().enumerate() {
        cloth
            .set_velocity(i as u32, Vec3::Y.cross(*p - center))
            .unwrap();
    }
    let masses = cloth.masses().to_vec();
    let target = SurfaceTarget {
        point,
        position: center,
        compliance: 0.0,
    };
    let report = Solver::new()
        .step_with_surface_targets(
            &mut cloth,
            1.0 / 240.0,
            Vec3::ZERO,
            &SolverSettings::default(),
            &[],
            &[target],
            &mut NoContacts,
        )
        .unwrap();
    assert!(report.max_target_error < 1.0e-6);
    assert!(
        cloth
            .surface()
            .point_position(point)
            .unwrap()
            .distance(center)
            < 1.0e-6
    );
    assert!(
        cloth
            .positions()
            .iter()
            .zip(start)
            .all(|(p, q)| p.distance(q) > 0.0005)
    );
    assert_eq!(cloth.masses(), masses);
    assert!(cloth.pins().is_empty());
}

#[test]
fn compliant_weighted_target_matches_effective_mass_and_alpha() {
    let mut cloth = cloth();
    let point = SurfacePoint::new(0, [0.2, 0.3, 0.5]).unwrap();
    let start = cloth.surface().point_position(point).unwrap();
    let tri = cloth.mesh().triangles()[0];
    let inverse_mass: Real = tri
        .iter()
        .zip(point.barycentric())
        .map(|(&i, b)| b * b / cloth.masses()[i as usize])
        .sum();
    let h = 0.01;
    let compliance = 0.0001;
    let target = SurfaceTarget {
        point,
        position: start + Vec3::Y * 0.1,
        compliance,
    };
    Solver::new()
        .step_with_surface_targets(
            &mut cloth,
            h,
            Vec3::ZERO,
            &SolverSettings {
                iterations: 1,
                ..Default::default()
            },
            &[],
            &[target],
            &mut NoContacts,
        )
        .unwrap();
    let expected = 0.1 * inverse_mass / (inverse_mass + compliance / (h * h));
    assert!((cloth.surface().point_position(point).unwrap().y - expected).abs() < 1.0e-6);
}

#[test]
fn overlapping_targets_are_rejected_atomically_but_zero_weights_do_not_conflict() {
    let mut cloth = cloth();
    let point = SurfacePoint::new(0, [0.2, 0.3, 0.5]).unwrap();
    let target = SurfaceTarget {
        point,
        position: Vec3::Y,
        compliance: 0.0,
    };
    let before = cloth.clone();
    for duplicate in [false, true] {
        let surfaces = if duplicate {
            vec![target, target]
        } else {
            vec![target]
        };
        let vertices = if duplicate {
            vec![]
        } else {
            vec![Target {
                particle: 0,
                position: Vec3::ZERO,
                compliance: 0.0,
            }]
        };
        assert!(matches!(
            Solver::new().step_with_surface_targets(
                &mut cloth,
                0.01,
                Vec3::ZERO,
                &SolverSettings::default(),
                &vertices,
                &surfaces,
                &mut NoContacts
            ),
            Err(ClothError::ConflictingTarget(_))
        ));
        assert_eq!(cloth.positions(), before.positions());
        assert_eq!(cloth.velocities(), before.velocities());
    }
    cloth.pin(1, Vec3::X).unwrap();
    let vertex = SurfaceTarget {
        point: SurfacePoint::new(0, [1.0, 0.0, 0.0]).unwrap(),
        position: Vec3::Y * 0.01,
        compliance: 0.0,
    };
    Solver::new()
        .step_with_surface_targets(
            &mut cloth,
            0.01,
            Vec3::ZERO,
            &SolverSettings::default(),
            &[],
            &[vertex],
            &mut NoContacts,
        )
        .unwrap();
}

#[test]
fn checkpoint_restore_cannot_alias_a_cloth_handle_allocated_in_discarded_time() {
    let mut set = ClothSet::new();
    let original = set.insert(cloth());
    let checkpoint = set.clone();
    set.remove(original).unwrap();
    let discarded = set.insert(cloth());
    set = checkpoint;
    assert!(set.get(original).is_ok());
    set.remove(original).unwrap();
    let replacement = set.insert(cloth());
    assert_eq!(discarded.index(), replacement.index());
    assert_ne!(discarded.generation(), replacement.generation());
    assert!(set.get(discarded).is_err());
}

#[test]
fn weighted_prediction_cannot_tunnel_through_a_nonlocal_pinned_layer() {
    let base = GridBuilder::new(2, 2)
        .size(0.4, 0.4)
        .origin(Vec3::new(-0.2, 0.0, -0.2))
        .build()
        .unwrap();
    let mut positions = base.rest_positions().to_vec();
    positions.extend([
        Vec3::new(-0.01, 0.02, -0.01),
        Vec3::new(0.01, 0.02, -0.01),
        Vec3::new(0.0, 0.02, 0.01),
    ]);
    let mut triangles = base.triangles().to_vec();
    triangles.push([4, 6, 5]);
    let mut cloth = Cloth::new(
        ClothMesh::new(positions.clone(), triangles).unwrap(),
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
    for i in 0..4 {
        cloth.pin(i, positions[i as usize]).unwrap();
    }
    let point = SurfacePoint::new(2, [1.0 / 3.0; 3]).unwrap();
    let target = SurfaceTarget {
        point,
        position: cloth.surface().point_position(point).unwrap() - Vec3::Y * 0.03,
        compliance: 0.0,
    };
    let before = cloth.clone();
    let result = Solver::new().step_with_surface_targets(
        &mut cloth,
        0.01,
        Vec3::ZERO,
        &SolverSettings::default(),
        &[],
        &[target],
        &mut NoContacts,
    );
    assert!(
        matches!(
            result,
            Err(ClothError::UnresolvedContinuousCollision(_)
                | ClothError::ConflictingSurfaceTarget { .. })
        ),
        "{result:?}"
    );
    assert_eq!(cloth.positions(), before.positions());
    assert_eq!(cloth.velocities(), before.velocities());
    assert_eq!(cloth.contact_history_len(), before.contact_history_len());
}
