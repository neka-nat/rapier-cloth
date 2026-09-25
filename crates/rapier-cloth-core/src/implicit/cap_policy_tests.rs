use super::*;
use crate::{
    ClothContactSettings, ClothMaterial, CollisionBudgetKind, CollisionLimits, CollisionWork,
    Contact, GridBuilder,
};

#[derive(Clone, Copy, PartialEq)]
enum Fault {
    None,
    FinalSweep,
    FinalBudget,
    FinalContact,
    LineSearch,
}
struct Source {
    fault: Fault,
    iteration_queries: usize,
    final_calls: usize,
    /// Positions of the first iteration-stage query (the accepted state).
    initial: Option<Vec<Vec3>>,
}
impl ContactSource for Source {
    fn continuous_motion(&self) -> bool {
        true
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
    fn motion_fraction(
        &mut self,
        motion: ContactMotion<'_>,
        work: &mut CollisionWork,
    ) -> Result<Real, ClothError> {
        if motion.stage == ContactStage::Final {
            self.final_calls += 1;
            if self.fault == Fault::FinalSweep {
                return Ok(0.5);
            }
            if self.fault == Fault::FinalBudget {
                return work
                    .charge(
                        CollisionBudgetKind::CcdChecks,
                        2,
                        CollisionLimits {
                            ccd_checks: 1,
                            ..Default::default()
                        },
                    )
                    .map(|_| 1.0);
            }
        }
        Ok(1.0)
    }
    fn surface_contacts(
        &mut self,
        _: &[Vec3],
        x: &[Vec3],
        stage: ContactStage,
        out: &mut Vec<SurfaceContact>,
    ) -> Result<(), ClothError> {
        if stage == ContactStage::Final && self.fault == Fault::FinalContact {
            return Err(ClothError::InvalidSurfaceContact(
                "injected final contact failure",
            ));
        }
        if stage == ContactStage::Iteration {
            self.iteration_queries += 1;
            let initial = self.initial.get_or_insert_with(|| x.to_vec());
            if self.fault == Fault::LineSearch && initial.as_slice() != x {
                // Every trial away from the accepted state has invalid
                // clearance; queries at that state (seed and start) are clear.
                out.push(SurfaceContact {
                    key: crate::SurfaceContactKey {
                        other_cloth: None,
                        features: [
                            crate::SurfaceFeature::Vertex(0),
                            crate::SurfaceFeature::External {
                                object: 1,
                                feature: 0,
                            },
                        ],
                    },
                    particles: [0, 0, 0, 0],
                    weights: [1.0, 0.0, 0.0, 0.0],
                    normal: Vec3::Y,
                    offset: Vec3::Y,
                    surface_velocity: Vec3::ZERO,
                    separation: 0.001,
                    static_friction: 0.0,
                    kinetic_friction: 0.0,
                });
            }
        }
        Ok(())
    }
}
fn cloth() -> Cloth {
    let mesh = GridBuilder::new(3, 3).size(0.25, 0.25).build().unwrap();
    let mut c = Cloth::new(mesh, ClothMaterial::default()).unwrap();
    c.set_contact_settings(Some(ClothContactSettings {
        self_collision: false,
        continuous_self_collision: false,
        rigid_surface_collision: true,
        continuous_rigid_collision: true,
        activation_margin: 0.001,
        ..Default::default()
    }))
    .unwrap();
    c
}

#[test]
fn strict_default_and_reference_profile_restriction_are_explicit() {
    assert_eq!(
        ImplicitSettings::default().cap_policy,
        ImplicitCapPolicy::Strict
    );
    let approximate = ImplicitSettings {
        cap_policy: ImplicitCapPolicy::ApproximateWithFinalValidation,
        ..Default::default()
    };
    assert!(approximate.validate().is_ok());
    for (max_iterations, velocity_tolerance, convergence_window) in [
        (512, 1e-7, 3),
        (64, 1e-7, 3),
        (80, 1e-7, 3),
        (80, 0.001, 3),
        (128, 0.001, 1),
    ] {
        let s = ImplicitSettings {
            max_iterations,
            velocity_tolerance,
            convergence_window,
            ..approximate
        };
        assert!(s.validate().is_err());
        let strict = ImplicitSettings {
            cap_policy: ImplicitCapPolicy::Strict,
            ..s
        };
        assert!(strict.validate().is_ok());
    }
}

#[test]
fn ordinary_success_keeps_converged_status_in_both_policies() {
    for cap_policy in [
        ImplicitCapPolicy::Strict,
        ImplicitCapPolicy::ApproximateWithFinalValidation,
    ] {
        let mut c = cloth();
        let x = c.positions.clone();
        let mut source = Source {
            fault: Fault::None,
            iteration_queries: 0,
            final_calls: 0,
            initial: None,
        };
        let r = step(
            &mut c,
            0.1,
            Vec3::ZERO,
            &SolverSettings {
                max_substep: 0.1,
                ..Default::default()
            },
            &[],
            &mut source,
            ImplicitSettings {
                cap_policy,
                ..Default::default()
            },
        )
        .unwrap();
        let status = r.implicit.unwrap();
        assert!(status.converged);
        assert_eq!(status.termination, ImplicitTermination::Converged);
        assert!(status.force_rms.is_finite());
        assert_eq!(c.positions, x);
        assert_eq!(source.final_calls, 1);
    }
}

#[test]
fn opt_in_does_not_swallow_final_sweep_budget_contact_or_line_search_failures() {
    for fault in [
        Fault::FinalSweep,
        Fault::FinalBudget,
        Fault::FinalContact,
        Fault::LineSearch,
    ] {
        for cap_policy in [
            ImplicitCapPolicy::Strict,
            ImplicitCapPolicy::ApproximateWithFinalValidation,
        ] {
            let mut c = cloth();
            let x = c.positions.clone();
            let v = c.velocities.clone();
            let mut source = Source {
                fault,
                iteration_queries: 0,
                final_calls: 0,
                initial: None,
            };
            let error = step(
                &mut c,
                0.1,
                Vec3::new(0.0, -9.81, 0.0),
                &SolverSettings {
                    max_substep: 0.1,
                    ..Default::default()
                },
                &[],
                &mut source,
                ImplicitSettings {
                    cap_policy,
                    ..Default::default()
                },
            )
            .unwrap_err();
            match fault {
                Fault::FinalSweep => assert!(matches!(
                    error,
                    ClothError::ImplicitSolverFailed {
                        phase: "final rigid sweep",
                        ..
                    }
                )),
                Fault::FinalBudget => {
                    assert!(matches!(error, ClothError::CollisionBudgetExceeded { .. }))
                }
                Fault::FinalContact => {
                    assert!(matches!(error, ClothError::InvalidSurfaceContact(_)))
                }
                Fault::LineSearch => assert!(matches!(
                    error,
                    ClothError::ImplicitSolverFailed {
                        phase: "line search",
                        ..
                    }
                )),
                Fault::None => unreachable!(),
            }
            assert_eq!(c.positions, x);
            assert_eq!(c.velocities, v);
            if fault != Fault::LineSearch {
                assert_eq!(source.final_calls, 1);
            }
        }
    }
}

#[test]
fn approximate_state_guard_rejects_nonfinite_broken_grasps_and_three_percent_extension() {
    let mut c = cloth();
    let p = c.positions[0];
    c.pin(0, p).unwrap();
    let mut x = c.positions.clone();
    assert!(validate_approximate_targets(&c, &x, &[], 128).is_ok());
    x[1].x = Real::NAN;
    assert!(validate_approximate_targets(&c, &x, &[], 128).is_err());
    x = c.positions.clone();
    x[0].y += 0.01;
    assert!(validate_approximate_targets(&c, &x, &[], 128).is_err());
    x = c.positions.clone();
    let target = Target {
        particle: 1,
        position: x[1] + Vec3::Y,
        compliance: 0.0,
    };
    assert!(validate_approximate_targets(&c, &x, &[target], 128).is_err());
    assert!(validate_approximate_extension(1.029, 128).is_ok());
    for stretch in [1.03, 1.04, Real::NAN, Real::INFINITY] {
        assert!(validate_approximate_extension(stretch, 128).is_err());
    }
}

#[test]
fn a_state_at_rest_stops_on_its_first_direction_while_motion_uses_the_window() {
    for seed in [ImplicitSeed::Previous, ImplicitSeed::Velocity] {
        let mut c = cloth();
        for i in [0, 2] {
            let p = c.positions[i];
            c.pin(i as u32, p).unwrap();
        }
        let mut source = Source {
            fault: Fault::None,
            iteration_queries: 0,
            final_calls: 0,
            initial: None,
        };
        let settings = SolverSettings {
            max_substep: 0.1,
            ..Default::default()
        };
        let mut iterations = Vec::new();
        for _ in 0..200 {
            let before = c.positions.clone();
            let velocity = c.velocities.clone();
            let r = step(
                &mut c,
                0.1,
                Vec3::new(0.0, -9.81, 0.0),
                &settings,
                &[],
                &mut source,
                ImplicitSettings {
                    seed,
                    ..Default::default()
                },
            )
            .unwrap();
            assert!(r.implicit.unwrap().converged);
            iterations.push(r.iterations);
            if r.iterations == 1 {
                // The first direction was within tolerance and was not applied:
                // the accepted state is the seed itself.
                let expected: Vec<Vec3> = before
                    .iter()
                    .zip(&velocity)
                    .enumerate()
                    .map(|(i, (&p, &v))| match seed {
                        ImplicitSeed::Previous => p,
                        ImplicitSeed::Velocity if c.pins.contains_key(&(i as u32)) => p,
                        ImplicitSeed::Velocity => p + v * 0.1,
                    })
                    .collect();
                assert_eq!(c.positions, expected);
                if seed == ImplicitSeed::Previous {
                    assert!(c.velocities.iter().all(|v| *v == Vec3::ZERO));
                } else {
                    // The extrapolation is retained as the accepted velocity
                    // (up to the roundoff of the position difference).
                    for (a, b) in c.velocities.iter().zip(&velocity) {
                        assert!(a.distance(*b) <= 1e-9, "{a} {b}");
                    }
                }
                break;
            }
        }
        // Falling motion needs the full three-direction window before stopping.
        assert!(iterations[0] >= 3, "{seed:?}: {iterations:?}");
        assert_eq!(iterations.last(), Some(&1), "{seed:?}: {iterations:?}");
    }
}
