#![cfg(all(feature = "implicit", feature = "f64"))]
use rapier_cloth_core::*;

fn cloth() -> Cloth {
    let mesh = GridBuilder::new(3, 3)
        .size(0.25, 0.25)
        .origin(Vec3::new(-0.125, 1.0, -0.125))
        .build()
        .unwrap();
    let mut cloth = Cloth::new(
        mesh,
        ClothMaterial {
            damping: 0.0,
            ..Default::default()
        },
    )
    .unwrap();
    cloth
        .set_contact_settings(Some(ClothContactSettings {
            self_collision: false,
            activation_margin: 0.001,
            ..Default::default()
        }))
        .unwrap();
    cloth
        .set_implicit_solver(Some(ImplicitSettings::default()))
        .unwrap();
    cloth
}
fn settings() -> SolverSettings {
    SolverSettings {
        max_substep: 0.1,
        ..Default::default()
    }
}
fn unchanged(a: &Cloth, b: &Cloth) {
    assert_eq!(a.positions(), b.positions());
    assert_eq!(a.previous_positions(), b.previous_positions());
    assert_eq!(a.velocities(), b.velocities());
    assert_eq!(a.pins(), b.pins());
    assert_eq!(a.implicit_solver_settings(), b.implicit_solver_settings());
    assert_eq!(a.contact_history_len(), b.contact_history_len());
}

#[test]
fn free_translation_and_gravity_preserve_shape_and_momentum() {
    let mut c = cloth();
    let initial = c.positions().to_vec();
    let v = Vec3::new(0.2, 0.3, -0.1);
    let gravity = Vec3::new(0.0, -9.81, 0.0);
    let h = 0.1;
    for i in 0..initial.len() {
        c.set_velocity(i as u32, v).unwrap();
    }
    Solver::new().step(&mut c, h, gravity, &settings()).unwrap();
    for (i, &p) in c.positions().iter().enumerate() {
        assert!((p - initial[i] - (v + gravity * h) * h).length() < 1e-9);
        assert!((c.velocities()[i] - v - gravity * h).length() < 1e-8);
    }
}

#[test]
fn zero_load_rest_is_a_valid_equilibrium() {
    let mut c = cloth();
    let before = c.positions().to_vec();
    Solver::new()
        .step(&mut c, 0.1, Vec3::ZERO, &settings())
        .unwrap();
    for (&a, &b) in c.positions().iter().zip(&before) {
        assert!((a - b).length() < 1e-10);
    }
}

#[test]
fn self_contact_friction_does_not_request_a_rigid_anchor() {
    struct NoRigidAnchors;
    impl ContactSource for NoRigidAnchors {
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
        fn transport_surface_anchor(
            &mut self,
            _: &SurfaceContact,
            _: Vec3,
            _: Real,
        ) -> Result<Vec3, ClothError> {
            Err(ClothError::InvalidSurfaceContact("no rigid surfaces"))
        }
    }
    let patch = GridBuilder::new(2, 2).size(0.1, 0.1).build().unwrap();
    let mut positions = patch.rest_positions().to_vec();
    positions.extend(patch.rest_positions().iter().map(|p| *p + Vec3::Y * 0.0012));
    let mut triangles = patch.triangles().to_vec();
    triangles.extend(patch.triangles().iter().map(|t| t.map(|i| i + 4)));
    let mut c = Cloth::new(
        ClothMesh::new(positions, triangles).unwrap(),
        ClothMaterial::default(),
    )
    .unwrap();
    c.set_contact_settings(Some(ClothContactSettings {
        thickness: 0.001,
        activation_margin: 0.001,
        continuous_self_collision: true,
        ..Default::default()
    }))
    .unwrap();
    c.set_implicit_solver(Some(ImplicitSettings::default()))
        .unwrap();
    for i in 0..c.positions().len() {
        c.pin(i as u32, c.positions()[i]).unwrap();
    }
    let report = Solver::new()
        .step_with_contacts(
            &mut c,
            0.1,
            Vec3::ZERO,
            &settings(),
            &[],
            &mut NoRigidAnchors,
        )
        .unwrap();
    assert!(report.contacts > 0);
}

#[test]
fn releasing_a_hard_grasp_keeps_physical_mass_and_velocity() {
    let mut c = cloth();
    let before = c.positions().to_vec();
    let masses = c.masses().to_vec();
    let movement = Vec3::new(0.01, 0.02, 0.0);
    for (i, &p) in before.iter().enumerate() {
        c.pin(i as u32, p + movement).unwrap();
    }
    let mut solver = Solver::new();
    solver.step(&mut c, 0.1, Vec3::ZERO, &settings()).unwrap();
    for i in 0..before.len() {
        c.unpin(i as u32).unwrap();
    }
    let released = c.positions().to_vec();
    solver.step(&mut c, 0.1, Vec3::ZERO, &settings()).unwrap();
    assert_eq!(c.masses(), masses);
    for (i, &p) in c.positions().iter().enumerate() {
        assert!((p - released[i] - movement).length() < 1e-8);
    }
}

#[test]
fn invalid_settings_and_unsupported_targets_leave_state_unchanged() {
    let mut c = cloth();
    let before = c.clone();
    assert!(
        c.set_implicit_solver(Some(ImplicitSettings {
            velocity_tolerance: Real::NAN,
            ..Default::default()
        }))
        .is_err()
    );
    unchanged(&c, &before);
    let p = c.positions()[0];
    c.pin(0, p).unwrap();
    let before = c.clone();
    let result = Solver::new().step_with_contacts(
        &mut c,
        0.1,
        Vec3::ZERO,
        &settings(),
        &[Target {
            particle: 0,
            position: p,
            compliance: 0.0,
        }],
        &mut NoContacts,
    );
    assert_eq!(result.unwrap_err(), ClothError::ConflictingTarget(0));
    unchanged(&c, &before);
    let result = Solver::new().step_with_contacts(
        &mut c,
        0.1,
        Vec3::ZERO,
        &settings(),
        &[Target {
            particle: 1,
            position: p,
            compliance: 0.001,
        }],
        &mut NoContacts,
    );
    assert!(matches!(result, Err(ClothError::InvalidParameter(_))));
    unchanged(&c, &before);
}

#[test]
fn external_friction_transports_a_world_space_point_and_rejects_nonfinite_results() {
    struct Surface {
        contact: SurfaceContact,
        point: Vec3,
        invalid: bool,
    }
    impl ContactSource for Surface {
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
            out.push(self.contact);
            Ok(())
        }
        fn transport_surface_anchor(
            &mut self,
            _: &SurfaceContact,
            point: Vec3,
            _: Real,
        ) -> Result<Vec3, ClothError> {
            assert!((point - self.point).length() < 1e-12);
            Ok(if self.invalid {
                Vec3::splat(Real::NAN)
            } else {
                point
            })
        }
    }
    let mut c = cloth();
    for i in 0..c.positions().len() {
        c.pin(i as u32, c.positions()[i]).unwrap();
    }
    let point = c.positions()[0];
    let mut source = Surface {
        point,
        invalid: false,
        contact: SurfaceContact {
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
            particles: [0; 4],
            weights: [1.0, 0.0, 0.0, 0.0],
            normal: Vec3::Y,
            offset: point - Vec3::Y * 0.0015,
            separation: 0.001,
            static_friction: 0.5,
            kinetic_friction: 0.5,
            surface_velocity: Vec3::ZERO,
        },
    };
    Solver::new()
        .step_with_contacts(&mut c, 0.1, Vec3::ZERO, &settings(), &[], &mut source)
        .unwrap();
    source.invalid = true;
    let before = c.clone();
    assert!(matches!(
        Solver::new().step_with_contacts(&mut c, 0.1, Vec3::ZERO, &settings(), &[], &mut source),
        Err(ClothError::InvalidSurfaceContact(_))
    ));
    unchanged(&c, &before);
}

struct InvalidMotion;
impl ContactSource for InvalidMotion {
    fn continuous_motion(&self) -> bool {
        true
    }
    fn motion_fraction(
        &mut self,
        _: ContactMotion<'_>,
        _: &mut CollisionWork,
    ) -> Result<Real, ClothError> {
        Ok(Real::NAN)
    }
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
}
#[test]
fn an_invalid_motion_certificate_is_atomic() {
    let mut c = cloth();
    let before = c.clone();
    let result = Solver::new().step_with_contacts(
        &mut c,
        0.1,
        -Vec3::Y,
        &settings(),
        &[],
        &mut InvalidMotion,
    );
    assert!(matches!(result, Err(ClothError::InvalidSurfaceContact(_))));
    unchanged(&c, &before);
}

#[test]
fn collision_budget_failure_keeps_the_cloth_and_can_be_retried() {
    let mut c = cloth();
    let mut contact = c.contact_settings().unwrap();
    contact.self_collision = true;
    contact.continuous_self_collision = true;
    contact.limits.candidate_pairs = 1;
    c.set_contact_settings(Some(contact)).unwrap();
    let before = c.clone();
    let mut solver = Solver::new();
    assert!(matches!(
        solver.step(&mut c, 0.1, -Vec3::Y, &settings()),
        Err(ClothError::CollisionBudgetExceeded { .. })
    ));
    unchanged(&c, &before);
    contact.limits = CollisionLimits::default();
    c.set_contact_settings(Some(contact)).unwrap();
    solver.step(&mut c, 0.1, -Vec3::Y, &settings()).unwrap();
}
