use rapier_cloth_core::constraints::distance;
use rapier_cloth_core::*;

fn triangle(origin: Vec3) -> Cloth {
    Cloth::new(
        ClothMesh::new(
            vec![origin, origin + Vec3::X, origin + Vec3::Z],
            vec![[0, 2, 1]],
        )
        .unwrap(),
        ClothMaterial {
            damping: 0.0,
            ..ClothMaterial::default()
        },
    )
    .unwrap()
}

#[test]
fn distance_constraint_has_expected_compliant_solution() {
    let mut p = vec![Vec3::ZERO, Vec3::X * 2.0];
    let mut lambda = 0.0;
    assert!(distance::project(
        &mut p,
        &[0.0, 1.0],
        [0, 1],
        1.0,
        0.5,
        &mut lambda
    ));
    assert!((p[1].x - 4.0 / 3.0).abs() < 1.0e-6);
    assert!((lambda + 2.0 / 3.0).abs() < 1.0e-6);
    let first = p.clone();
    distance::project(&mut p, &[0.0, 1.0], [0, 1], 1.0, 0.5, &mut lambda);
    assert!(p[1].distance(first[1]) < 1.0e-6);
    assert_eq!(p[0], Vec3::ZERO);
}

#[test]
fn freefall_matches_discrete_solution_and_converges() {
    let g = Vec3::new(0.0, -9.81, 0.0);
    let mut errors = vec![];
    for n in [120, 240, 480] {
        let mut cloth = triangle(Vec3::new(0.0, 10.0, 0.0));
        let mut solver = Solver::new();
        let h = 1.0 / n as Real;
        for _ in 0..n {
            solver
                .step(&mut cloth, h, g, &SolverSettings::default())
                .unwrap();
        }
        let discrete = 10.0 + g.y * h * h * (n * (n + 1)) as Real / 2.0;
        let continuous = 10.0 + g.y / 2.0;
        assert!(
            (cloth.positions()[0].y - discrete).abs() < 0.002,
            "{n}: {} != {discrete}",
            cloth.positions()[0].y
        );
        errors.push((cloth.positions()[0].y - continuous).abs());
    }
    assert!(
        errors[1] < errors[0] * 0.7 && errors[2] < errors[1] * 0.7,
        "{errors:?}"
    );
}

#[test]
fn pins_preserve_mass_and_release_velocity() {
    let mut cloth = triangle(Vec3::Y);
    let masses = cloth.masses().to_vec();
    cloth.pin(0, Vec3::Y).unwrap();
    let mut solver = Solver::new();
    solver
        .step(&mut cloth, 0.01, -Vec3::Y, &SolverSettings::default())
        .unwrap();
    assert_eq!(cloth.positions()[0], Vec3::Y);
    assert_eq!(cloth.masses(), masses);
    cloth.pin(0, Vec3::Y + Vec3::X * 0.001).unwrap();
    solver
        .step(&mut cloth, 0.01, Vec3::ZERO, &SolverSettings::default())
        .unwrap();
    assert!((cloth.velocities()[0].x - 0.1).abs() < 1.0e-5);
    let before = cloth.velocities()[0];
    cloth.unpin(0).unwrap();
    assert_eq!(cloth.velocities()[0], before);
}

#[test]
fn handles_are_scoped_and_generational() {
    let mut set = ClothSet::new();
    let first = set.insert(triangle(Vec3::ZERO));
    let mut other = ClothSet::new();
    other.insert(triangle(Vec3::ZERO));
    assert!(other.get(first).is_err());
    set.remove(first).unwrap();
    let second = set.insert(triangle(Vec3::ZERO));
    assert_eq!(first.index(), second.index());
    assert_ne!(first.generation(), second.generation());
    assert!(set.get(first).is_err());
    assert!(set.get(second).is_ok());
}

#[test]
fn failed_substep_is_atomic() {
    struct Failure;
    impl ContactSource for Failure {
        fn contacts(
            &mut self,
            _: &[Vec3],
            _: &[Vec3],
            _: Real,
            stage: ContactStage,
            _: &mut Vec<Contact>,
        ) -> Result<(), ClothError> {
            if stage == ContactStage::Final {
                Err(ClothError::External("deliberate late query failure".into()))
            } else {
                Ok(())
            }
        }
    }
    let mut cloth = triangle(Vec3::Y);
    let original = cloth.clone();
    let mut solver = Solver::new();
    assert!(
        solver
            .step_with_contacts(
                &mut cloth,
                0.01,
                -Vec3::Y,
                &SolverSettings::default(),
                &[],
                &mut Failure
            )
            .is_err()
    );
    assert_eq!(cloth.positions(), original.positions());
    assert_eq!(cloth.velocities(), original.velocities());
    assert!(
        solver
            .step(
                &mut cloth,
                Real::NAN,
                Vec3::ZERO,
                &SolverSettings::default()
            )
            .is_err()
    );
    assert!(
        solver
            .step(&mut cloth, 1.0, Vec3::ZERO, &SolverSettings::default())
            .is_err()
    );
    solver
        .step(&mut cloth, 0.01, -Vec3::Y, &SolverSettings::default())
        .unwrap();
    assert!(cloth.positions()[0].y < 1.0);
}

#[test]
fn rigid_transform_equivariance_and_surface_normals() {
    let rotate = |p: Vec3| Vec3::new(-p.z, p.y, p.x);
    let mesh = GridBuilder::new(5, 5).build().unwrap();
    let rotated = ClothMesh::new(
        mesh.rest_positions().iter().map(|p| rotate(*p)).collect(),
        mesh.triangles().to_vec(),
    )
    .unwrap();
    let mut a = Cloth::new(mesh, ClothMaterial::default()).unwrap();
    let mut b = Cloth::new(rotated, ClothMaterial::default()).unwrap();
    a.pin(0, Vec3::ZERO).unwrap();
    b.pin(0, Vec3::ZERO).unwrap();
    let mut sa = Solver::new();
    let mut sb = Solver::new();
    let gravity = Vec3::new(0.2, -1.0, 0.4);
    for _ in 0..30 {
        sa.step(&mut a, 1.0 / 240.0, gravity, &SolverSettings::default())
            .unwrap();
        sb.step(
            &mut b,
            1.0 / 240.0,
            rotate(gravity),
            &SolverSettings::default(),
        )
        .unwrap();
    }
    for (a, b) in a.positions().iter().zip(b.positions()) {
        assert!(rotate(*a).distance(*b) < 5.0e-4);
    }
    let mut normals = vec![];
    assert_eq!(a.surface().write_normals(&mut normals), 0);
    assert_eq!(normals.len(), 25);
    assert!(normals.iter().all(|n| (n.length() - 1.0).abs() < 1.0e-5));
}

#[test]
fn material_and_state_setters_reject_invalid_input() {
    for bad in [
        ClothMaterial {
            surface_density: 0.0,
            ..Default::default()
        },
        ClothMaterial {
            friction: -0.1,
            ..Default::default()
        },
        ClothMaterial {
            contact_radius: Real::INFINITY,
            ..Default::default()
        },
        ClothMaterial {
            bend_compliance: Real::NAN,
            ..Default::default()
        },
    ] {
        assert!(bad.validate().is_err());
    }
    let mut c = triangle(Vec3::ZERO);
    assert!(c.set_velocity(99, Vec3::ZERO).is_err());
    assert!(c.set_force(0, Vec3::splat(Real::NAN)).is_err());
    assert!(c.set_positions(&[Vec3::ZERO]).is_err());
}
