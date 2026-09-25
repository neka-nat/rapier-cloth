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
        _: &[Vec3],
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
            if self.fault == Fault::LineSearch && self.iteration_queries > 1 {
                // Every trial has invalid clearance; the initial iterate query is clear.
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
    for (max_iterations, velocity_tolerance) in [(512, 1e-7), (64, 1e-7), (80, 1e-7)] {
        let s = ImplicitSettings {
            cap_policy: ImplicitCapPolicy::ApproximateWithFinalValidation,
            max_iterations,
            velocity_tolerance,
            ..Default::default()
        };
        assert!(s.validate().is_err());
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
    assert!(validate_approximate_targets(&c, &x, &[]).is_ok());
    x[1].x = Real::NAN;
    assert!(validate_approximate_targets(&c, &x, &[]).is_err());
    x = c.positions.clone();
    x[0].y += 0.01;
    assert!(validate_approximate_targets(&c, &x, &[]).is_err());
    x = c.positions.clone();
    let target = Target {
        particle: 1,
        position: x[1] + Vec3::Y,
        compliance: 0.0,
    };
    assert!(validate_approximate_targets(&c, &x, &[target]).is_err());
    assert!(validate_approximate_extension(1.029, 80).is_ok());
    for stretch in [1.03, 1.04, Real::NAN, Real::INFINITY] {
        assert!(validate_approximate_extension(stretch, 80).is_err());
    }
}
