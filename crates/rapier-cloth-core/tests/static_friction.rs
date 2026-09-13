use rapier_cloth_core::*;

struct Plane {
    normal: Vec3,
    mu_s: Real,
    mu_k: Real,
    fail: bool,
}
impl ContactSource for Plane {
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
        positions: &[Vec3],
        stage: ContactStage,
        out: &mut Vec<SurfaceContact>,
    ) -> Result<(), ClothError> {
        if self.fail && stage == ContactStage::Final {
            return Err(ClothError::External("late friction failure".into()));
        }
        for (i, &p) in positions.iter().enumerate() {
            let distance = p.dot(self.normal);
            if distance > 0.0101 {
                continue;
            }
            out.push(SurfaceContact {
                key: SurfaceContactKey {
                    other_cloth: None,
                    features: [
                        SurfaceFeature::Vertex(i as u32),
                        SurfaceFeature::External {
                            object: 1,
                            feature: 0,
                        },
                    ],
                },
                particles: [i as u32, 0, 0, 0],
                weights: [1.0, 0.0, 0.0, 0.0],
                normal: self.normal,
                // Closest rigid witness follows the cloth: it must not replace
                // the material anchor and erase the measured tangential slip.
                offset: p - self.normal * distance,
                surface_velocity: Vec3::ZERO,
                separation: 0.01,
                static_friction: self.mu_s,
                kinetic_friction: self.mu_k,
            });
        }
        Ok(())
    }
}
fn patch(normal: Vec3) -> Cloth {
    let tangent = Vec3::Z.cross(normal).normalize();
    let points = [(-0.05, -0.05), (0.05, -0.05), (-0.05, 0.05), (0.05, 0.05)]
        .map(|(x, z)| tangent * x + Vec3::Z * z + normal * 0.01);
    Cloth::new(
        ClothMesh::new(points.to_vec(), vec![[0, 2, 1], [1, 2, 3]]).unwrap(),
        ClothMaterial {
            damping: 0.0,
            ..Default::default()
        },
    )
    .unwrap()
}
fn step(cloth: &mut Cloth, solver: &mut Solver, plane: &mut Plane, h: Real, acceleration: Vec3) {
    solver
        .step_with_contacts(
            cloth,
            h,
            acceleration,
            &SolverSettings::default(),
            &[],
            plane,
        )
        .unwrap();
}

#[test]
fn horizontal_pull_brackets_static_threshold_and_obeys_kinetic_acceleration() {
    for h in [1.0 / 120.0, 1.0 / 240.0, 1.0 / 480.0] {
        for pull in [4.9, 5.1] {
            let mut cloth = patch(Vec3::Y);
            let start = cloth.positions().to_vec();
            let mut source = Plane {
                normal: Vec3::Y,
                mu_s: 0.5,
                mu_k: 0.2,
                fail: false,
            };
            let mut solver = Solver::new();
            let count = (0.5 / h) as usize;
            for _ in 0..count {
                step(
                    &mut cloth,
                    &mut solver,
                    &mut source,
                    h,
                    Vec3::new(pull, -10.0, 0.0),
                );
            }
            for (p, original) in cloth.positions().iter().zip(start) {
                if pull < 5.0 {
                    assert!(
                        p.distance(original) < 1.0e-5,
                        "static drift {p:?} != {original:?}, h={h}"
                    );
                } else {
                    assert!(p.x - original.x > 0.3, "no positive slip: {p:?}");
                }
            }
            let expected = if pull < 5.0 {
                0.0
            } else {
                (pull - 2.0) * h * count as Real
            };
            for velocity in cloth.velocities() {
                assert!(
                    (velocity.x - expected).abs() < 0.002,
                    "h={h}, pull={pull}, {velocity:?} != {expected}"
                );
            }
        }
    }
}

#[test]
fn incline_threshold_depends_on_normal_load_and_zero_friction_slides() {
    for ratio in [0.49, 0.51] {
        let normal = Vec3::new(ratio, 1.0, 0.0).normalize();
        let downhill = Vec3::new(1.0, -ratio, 0.0).normalize();
        for friction in [true, false] {
            let mut cloth = patch(normal);
            let start = cloth.positions().to_vec();
            let mut source = Plane {
                normal,
                mu_s: if friction { 0.5 } else { 0.0 },
                mu_k: if friction { 0.2 } else { 0.0 },
                fail: false,
            };
            let mut solver = Solver::new();
            for _ in 0..120 {
                step(
                    &mut cloth,
                    &mut solver,
                    &mut source,
                    1.0 / 240.0,
                    -Vec3::Y * 10.0,
                );
            }
            for (p, original) in cloth.positions().iter().zip(start) {
                if friction && ratio < 0.5 {
                    assert!(
                        p.distance(original) < 1.0e-5,
                        "incline creep: {p:?} != {original:?}"
                    );
                } else {
                    assert!(
                        (*p - original).dot(downhill) > 0.2,
                        "no downhill slip: {p:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn kinetic_slowdown_is_not_applied_twice_and_initial_recovery_supplies_no_load() {
    for recovery in [false, true] {
        let mut cloth = patch(Vec3::Y);
        if recovery {
            let positions: Vec<_> = cloth
                .positions()
                .iter()
                .map(|p| *p - Vec3::Y * 0.1)
                .collect();
            cloth.set_positions(&positions).unwrap();
        }
        for i in 0..4 {
            cloth.set_velocity(i, Vec3::X).unwrap();
        }
        let mut source = Plane {
            normal: Vec3::Y,
            mu_s: 0.6,
            mu_k: 0.2,
            fail: false,
        };
        let mut solver = Solver::new();
        let g = if recovery {
            Vec3::ZERO
        } else {
            -Vec3::Y * 10.0
        };
        for _ in 0..60 {
            step(&mut cloth, &mut solver, &mut source, 1.0 / 240.0, g);
        }
        let expected = if recovery { 1.0 } else { 0.5 };
        for velocity in cloth.velocities() {
            assert!(
                (velocity.x - expected).abs() < 0.001,
                "{velocity:?} != {expected}"
            );
        }
    }
}

#[test]
fn separation_late_failure_and_checkpoint_do_not_leave_hidden_holds() {
    let mut cloth = patch(Vec3::Y);
    let mut source = Plane {
        normal: Vec3::Y,
        mu_s: 0.5,
        mu_k: 0.2,
        fail: false,
    };
    let mut solver = Solver::new();
    let acceleration = Vec3::new(4.9, -10.0, 0.0);
    for _ in 0..24 {
        step(
            &mut cloth,
            &mut solver,
            &mut source,
            1.0 / 240.0,
            acceleration,
        );
    }
    let checkpoint = cloth.clone();
    source.fail = true;
    assert!(
        solver
            .step_with_contacts(
                &mut cloth,
                1.0 / 240.0,
                acceleration,
                &SolverSettings::default(),
                &[],
                &mut source
            )
            .is_err()
    );
    assert_eq!(cloth.positions(), checkpoint.positions());
    assert_eq!(cloth.velocities(), checkpoint.velocities());
    assert_eq!(
        cloth.contact_history_len(),
        checkpoint.contact_history_len()
    );
    source.fail = false;
    let mut replay = checkpoint.clone();
    step(
        &mut cloth,
        &mut solver,
        &mut source,
        1.0 / 480.0,
        acceleration,
    );
    step(
        &mut replay,
        &mut Solver::new(),
        &mut source,
        1.0 / 480.0,
        acceleration,
    );
    assert_eq!(cloth.positions(), replay.positions());
    assert_eq!(cloth.velocities(), replay.velocities());
    for i in 0..4 {
        cloth.set_velocity(i, Vec3::new(1.0, 1.0, 0.0)).unwrap();
    }
    step(
        &mut cloth,
        &mut solver,
        &mut source,
        1.0 / 240.0,
        Vec3::ZERO,
    );
    assert_eq!(cloth.contact_history_len(), 0);
    for velocity in cloth.velocities() {
        assert!((velocity.x - 1.0).abs() < 1.0e-5);
    }
}

#[test]
fn self_contact_holds_a_loaded_patch_without_pinning_its_free_particles() {
    for continuous in [false, true] {
        for friction in [false, true] {
            let bottom = GridBuilder::new(2, 2).size(0.2, 0.2).build().unwrap();
            let top = GridBuilder::new(2, 2)
                .size(0.02, 0.02)
                .origin(Vec3::new(0.05, 0.001, 0.05))
                .build()
                .unwrap();
            let mut positions = bottom.rest_positions().to_vec();
            positions.extend_from_slice(top.rest_positions());
            let mut triangles = bottom.triangles().to_vec();
            triangles.extend(top.triangles().iter().map(|t| t.map(|i| i + 4)));
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
                    continuous_self_collision: continuous,
                    static_friction: if friction { 0.5 } else { 0.0 },
                    kinetic_friction: if friction { 0.2 } else { 0.0 },
                    ..Default::default()
                }))
                .unwrap();
            for i in 0..4 {
                cloth.pin(i, positions[i as usize]).unwrap();
            }
            let mut solver = Solver::new();
            // The zero-friction control travels 9.3 mm and stays over the base.
            for _ in 0..24 {
                solver
                    .step(
                        &mut cloth,
                        1.0 / 240.0,
                        Vec3::new(1.8, -10.0, 0.0),
                        &SolverSettings::default(),
                    )
                    .unwrap();
            }
            assert_eq!(cloth.pins().len(), 4);
            for (&p, &start) in cloth.positions()[4..].iter().zip(&positions[4..]) {
                if friction {
                    assert!(
                        p.distance(start) < 1.0e-5,
                        "continuous={continuous}: {p:?} != {start:?}"
                    );
                } else {
                    assert!(p.x - start.x > 0.008, "{p:?} != {start:?}");
                }
            }
        }
    }
}
