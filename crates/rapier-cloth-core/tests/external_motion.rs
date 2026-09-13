use rapier_cloth_core::*;

#[derive(Default)]
struct AuditSource {
    stages: Vec<ContactStage>,
    fail: Option<(ContactStage, Real)>,
    limits: CollisionLimits,
}
impl ContactSource for AuditSource {
    fn continuous_motion(&self) -> bool {
        true
    }
    fn motion_fraction(
        &mut self,
        motion: ContactMotion<'_>,
        work: &mut CollisionWork,
    ) -> Result<Real, ClothError> {
        assert_eq!(motion.start.len(), motion.end.len());
        assert!(motion.start.iter().chain(motion.end).all(|p| p.is_finite()));
        self.stages.push(motion.stage);
        work.charge(CollisionBudgetKind::CcdChecks, 1, self.limits)?;
        Ok(self
            .fail
            .filter(|(stage, _)| *stage == motion.stage)
            .map_or(1.0, |(_, f)| f))
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

fn cloth() -> Cloth {
    let mut cloth = Cloth::new(
        GridBuilder::new(2, 2).build().unwrap(),
        ClothMaterial {
            damping: 0.0,
            ..Default::default()
        },
    )
    .unwrap();
    cloth.set_velocity(0, Vec3::Y * 0.1).unwrap();
    cloth
}

#[test]
fn source_checks_all_stages_without_builtin_self_collision() {
    for explicit_config in [false, true] {
        let mut cloth = cloth();
        if explicit_config {
            cloth
                .set_contact_settings(Some(ClothContactSettings {
                    self_collision: false,
                    ..Default::default()
                }))
                .unwrap();
        }
        let mut source = AuditSource::default();
        let settings = SolverSettings::default();
        let report = Solver::new()
            .step_with_contacts(
                &mut cloth,
                1.0 / 240.0,
                Vec3::ZERO,
                &settings,
                &[],
                &mut source,
            )
            .unwrap();
        assert_eq!(source.stages.first(), Some(&ContactStage::Stabilization));
        assert_eq!(source.stages.last(), Some(&ContactStage::Final));
        assert_eq!(
            source
                .stages
                .iter()
                .filter(|s| **s == ContactStage::Prediction)
                .count(),
            2
        );
        // Every iteration certifies the completed elastic/target/contact trial
        // from the previous accepted pose, including an empty contact set.
        assert_eq!(
            source
                .stages
                .iter()
                .filter(|s| **s == ContactStage::Iteration)
                .count(),
            settings.iterations
        );
        assert_eq!(report.surface_collision.ccd_checks, source.stages.len());
    }
}

#[test]
fn invalid_motion_results_at_each_stage_preserve_the_entire_cloth() {
    for stage in [
        ContactStage::Stabilization,
        ContactStage::Prediction,
        ContactStage::Iteration,
        ContactStage::Final,
    ] {
        for fraction in [Real::NAN, Real::INFINITY, -0.1, 1.1] {
            let mut cloth = cloth();
            let before = cloth.clone();
            let mut source = AuditSource {
                fail: Some((stage, fraction)),
                ..Default::default()
            };
            let result = Solver::new().step_with_contacts(
                &mut cloth,
                1.0 / 240.0,
                Vec3::ZERO,
                &SolverSettings::default(),
                &[],
                &mut source,
            );
            assert!(matches!(result, Err(ClothError::External(_))), "{result:?}");
            assert_eq!(source.stages.last(), Some(&stage));
            assert_eq!(cloth.positions(), before.positions());
            assert_eq!(cloth.previous_positions(), before.previous_positions());
            assert_eq!(cloth.velocities(), before.velocities());
            assert_eq!(cloth.contact_history_len(), before.contact_history_len());
        }
    }
}

#[test]
fn external_ccd_budget_is_cumulative_and_resets_on_solver_reuse() {
    let mut solver = Solver::new();
    let mut source = AuditSource {
        limits: CollisionLimits {
            ccd_checks: 2,
            ..Default::default()
        },
        ..Default::default()
    };
    let mut cloth = cloth();
    let before = cloth.clone();
    let result = solver.step_with_contacts(
        &mut cloth,
        1.0 / 240.0,
        Vec3::ZERO,
        &SolverSettings::default(),
        &[],
        &mut source,
    );
    assert!(matches!(
        result,
        Err(ClothError::CollisionBudgetExceeded {
            kind: CollisionBudgetKind::CcdChecks,
            limit: 2
        })
    ));
    assert_eq!(cloth.positions(), before.positions());
    source.limits = CollisionLimits::default();
    source.stages.clear();
    let report = solver
        .step_with_contacts(
            &mut cloth,
            1.0 / 240.0,
            Vec3::ZERO,
            &SolverSettings::default(),
            &[],
            &mut source,
        )
        .unwrap();
    assert_eq!(report.surface_collision.ccd_checks, source.stages.len());
}

#[derive(Default)]
struct RejectProjection {
    contact_stage: Option<ContactStage>,
    rejected_stage: Option<ContactStage>,
}
impl ContactSource for RejectProjection {
    fn continuous_motion(&self) -> bool {
        true
    }
    fn motion_fraction(
        &mut self,
        motion: ContactMotion<'_>,
        _: &mut CollisionWork,
    ) -> Result<Real, ClothError> {
        if motion
            .start
            .iter()
            .zip(motion.end)
            .any(|(a, b)| a.distance(*b) > 1.0e-6)
        {
            self.rejected_stage = Some(motion.stage);
            return Err(ClothError::External(
                "analytical source rejects the proposed displacement".into(),
            ));
        }
        Ok(1.0)
    }
    fn contacts(
        &mut self,
        _: &[Vec3],
        _: &[Vec3],
        _: Real,
        stage: ContactStage,
        out: &mut Vec<Contact>,
    ) -> Result<(), ClothError> {
        if self.contact_stage == Some(stage) {
            out.push(Contact {
                key: ContactKey {
                    particle: 0,
                    external: 1,
                    feature: 0,
                },
                normal: Vec3::Y,
                point: Vec3::Y * 0.1,
                surface_velocity: Vec3::ZERO,
                friction: 0.0,
            });
        }
        Ok(())
    }
}

#[test]
fn each_constraint_kind_passes_its_actual_correction_to_the_external_source() {
    for kind in [
        "stretch",
        "bend",
        "compliant target",
        "hard target",
        "contact",
        "stabilization",
    ] {
        let mut material = ClothMaterial {
            damping: 0.0,
            ..Default::default()
        };
        if kind == "stretch" {
            material.bend_compliance = 1.0e12;
        }
        if kind == "bend" {
            material.stretch_compliance = 1.0e12;
            material.bend_compliance = 0.0;
        }
        let mut cloth = Cloth::new(GridBuilder::new(2, 2).build().unwrap(), material).unwrap();
        if kind == "stretch" || kind == "bend" {
            let mut positions = cloth.positions().to_vec();
            positions[0].y += 0.1;
            cloth.set_positions(&positions).unwrap();
        }
        let targets = if kind.ends_with("target") {
            vec![Target {
                particle: 0,
                position: cloth.positions()[0] + Vec3::Y * 0.1,
                compliance: if kind == "hard target" { 0.0 } else { 1.0e-5 },
            }]
        } else {
            vec![]
        };
        let mut source = RejectProjection {
            contact_stage: match kind {
                "contact" => Some(ContactStage::Iteration),
                "stabilization" => Some(ContactStage::Stabilization),
                _ => None,
            },
            ..Default::default()
        };
        let expected_stage = match kind {
            "hard target" => ContactStage::Prediction,
            "stabilization" => ContactStage::Stabilization,
            _ => ContactStage::Iteration,
        };
        let before = cloth.clone();
        let result = Solver::new().step_with_contacts(
            &mut cloth,
            1.0 / 240.0,
            Vec3::ZERO,
            &SolverSettings::default(),
            &targets,
            &mut source,
        );
        assert!(
            matches!(result, Err(ClothError::External(_))),
            "{kind}: {result:?}"
        );
        assert_eq!(source.rejected_stage, Some(expected_stage), "{kind}");
        assert_eq!(cloth.positions(), before.positions(), "{kind}");
        assert_eq!(cloth.velocities(), before.velocities(), "{kind}");
        assert_eq!(
            cloth.previous_positions(),
            before.previous_positions(),
            "{kind}"
        );
    }
}
