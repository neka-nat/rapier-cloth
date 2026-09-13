use rapier_cloth_core::*;

fn rotating_stack() -> Cloth {
    let lower = GridBuilder::new(2, 2).size(0.2, 0.2).build().unwrap();
    let upper = GridBuilder::new(2, 2)
        .size(0.02, 0.02)
        .origin(Vec3::new(0.05, 0.001, 0.05))
        .build()
        .unwrap();
    let mut positions = lower.rest_positions().to_vec();
    positions.extend_from_slice(upper.rest_positions());
    let mut triangles = lower.triangles().to_vec();
    triangles.extend(upper.triangles().iter().map(|t| t.map(|i| i + 4)));
    let mut cloth = Cloth::new(
        ClothMesh::new(positions, triangles).unwrap(),
        ClothMaterial {
            damping: 0.0,
            ..Default::default()
        },
    )
    .unwrap();
    cloth
        .set_contact_settings(Some(ClothContactSettings {
            continuous_self_collision: true,
            static_friction: 0.5,
            kinetic_friction: 0.2,
            ..Default::default()
        }))
        .unwrap();
    cloth
}

fn rotate(p: Vec3, angle: Real) -> Vec3 {
    let (sin, cos) = angle.sin_cos();
    Vec3::new(cos * p.x + sin * p.z, p.y, -sin * p.x + cos * p.z)
}

#[test]
fn rotating_self_support_history_replays_and_separation_releases_the_patch() {
    let mut cloth = rotating_stack();
    let initial = cloth.positions().to_vec();
    let masses = cloth.masses().to_vec();
    let h = 1.0 / 240.0;
    let mut solver = Solver::new();
    let advance = |cloth: &mut Cloth, solver: &mut Solver, step: usize| {
        let targets: Vec<_> = (0..4)
            .map(|i| Target {
                particle: i as u32,
                position: rotate(initial[i], step as Real * 0.001),
                compliance: 0.0,
            })
            .collect();
        solver
            .step_with_contacts(
                cloth,
                h,
                -Vec3::Y * 10.0,
                &SolverSettings::default(),
                &targets,
                &mut NoContacts,
            )
            .unwrap();
    };
    for step in 0..24 {
        advance(&mut cloth, &mut solver, step);
    }
    assert!(cloth.contact_history_len() > 0);
    let checkpoint = cloth.clone();
    for step in 24..48 {
        advance(&mut cloth, &mut solver, step);
    }
    let expected = cloth.clone();
    cloth = checkpoint;
    for step in 24..48 {
        advance(&mut cloth, &mut solver, step);
    }
    assert_eq!(cloth.positions(), expected.positions());
    assert_eq!(cloth.velocities(), expected.velocities());
    assert_eq!(cloth.contact_history_len(), expected.contact_history_len());
    assert_eq!(cloth.masses(), masses);
    assert!(cloth.pins().is_empty());
    for (&p, &start) in cloth.positions()[4..].iter().zip(&initial[4..]) {
        assert!(
            p.distance(rotate(start, 0.047)) < 0.0001,
            "rotating support lost the patch: {p:?}, initial={start:?}"
        );
    }

    let targets: Vec<_> = (0..4)
        .map(|i| Target {
            particle: i as u32,
            position: cloth.positions()[i],
            compliance: 0.0,
        })
        .collect();
    for i in 4..8 {
        cloth.set_velocity(i, Vec3::new(0.2, 0.5, 0.0)).unwrap();
    }
    // Elastic relaxation can change individual velocities after release. Compare
    // against the same physical state with no history, then check total momentum.
    let mut cleared = cloth.clone();
    cleared.clear_contact_history();
    solver
        .step_with_contacts(
            &mut cloth,
            h,
            Vec3::ZERO,
            &SolverSettings::default(),
            &targets,
            &mut NoContacts,
        )
        .unwrap();
    Solver::new()
        .step_with_contacts(
            &mut cleared,
            h,
            Vec3::ZERO,
            &SolverSettings::default(),
            &targets,
            &mut NoContacts,
        )
        .unwrap();
    assert_eq!(cloth.contact_history_len(), 0);
    assert_eq!(cloth.positions(), cleared.positions());
    assert_eq!(cloth.velocities(), cleared.velocities());
    let total_mass: Real = masses[4..].iter().sum();
    let center_velocity = cloth.velocities()[4..]
        .iter()
        .zip(&masses[4..])
        .map(|(&v, &m)| v * m)
        .sum::<Vec3>()
        / total_mass;
    assert!(
        center_velocity.distance(Vec3::new(0.2, 0.5, 0.0)) < 1e-5,
        "separated material lost momentum: {center_velocity:?}"
    );
}

struct VertexPair;
impl ContactSource for VertexPair {
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
        out.push(SurfaceContact {
            key: SurfaceContactKey {
                other_cloth: None,
                features: [SurfaceFeature::Vertex(3), SurfaceFeature::Vertex(0)],
            },
            particles: [3, 0, 0, 0],
            weights: [1.0, -1.0, 0.0, 0.0],
            normal: Vec3::Y,
            offset: Vec3::ZERO,
            surface_velocity: Vec3::ZERO,
            separation: 0.01,
            static_friction: 0.5,
            kinetic_friction: 0.2,
        });
        Ok(())
    }
}

#[test]
fn degenerate_material_frame_returns_a_typed_atomic_failure() {
    let mesh = ClothMesh::new(
        vec![
            Vec3::ZERO,
            Vec3::X,
            Vec3::Z,
            Vec3::Y * 0.01,
            Vec3::Y * 0.01 + Vec3::X,
            Vec3::Y * 0.01 + Vec3::Z,
        ],
        vec![[0, 2, 1], [3, 5, 4]],
    )
    .unwrap();
    let mut cloth = Cloth::new(mesh, ClothMaterial::default()).unwrap();
    let mut collapsed = cloth.positions().to_vec();
    collapsed[2] = Vec3::X * 2.0;
    cloth.set_positions(&collapsed).unwrap();
    let before = cloth.clone();
    let result = Solver::new().step_with_contacts(
        &mut cloth,
        1.0 / 240.0,
        Vec3::ZERO,
        &SolverSettings::default(),
        &[],
        &mut VertexPair,
    );
    assert!(
        matches!(
            result,
            Err(ClothError::InvalidSurfaceContact(
                "degenerate friction material triangle"
            ))
        ),
        "{result:?}"
    );
    assert_eq!(cloth.positions(), before.positions());
    assert_eq!(cloth.previous_positions(), before.previous_positions());
    assert_eq!(cloth.velocities(), before.velocities());
    assert_eq!(cloth.masses(), before.masses());
    assert_eq!(cloth.contact_history_len(), before.contact_history_len());
}
