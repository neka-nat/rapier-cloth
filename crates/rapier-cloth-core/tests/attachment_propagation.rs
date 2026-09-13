use rapier_cloth_core::*;

#[test]
fn a_pulled_strip_recovers_near_grasp_strain_with_eight_iterations() {
    for continuous in [false, true] {
        let mut cloth = Cloth::new(
            GridBuilder::new(8, 4).size(0.5, 0.05).build().unwrap(),
            ClothMaterial {
                damping: 0.0,
                bend_compliance: 10000.0,
                ..Default::default()
            },
        )
        .unwrap();
        if continuous {
            cloth
                .set_contact_settings(Some(ClothContactSettings {
                    continuous_self_collision: true,
                    ..Default::default()
                }))
                .unwrap();
        }
        for i in [0, 8, 16, 24] {
            cloth
                .pin(
                    i,
                    cloth.positions()[i as usize] + Vec3::new(-0.03, 0.03, 0.0),
                )
                .unwrap();
        }
        let report = Solver::new()
            .step(
                &mut cloth,
                1.0 / 240.0,
                Vec3::ZERO,
                &SolverSettings::default(),
            )
            .unwrap();
        assert_eq!(report.iterations, 8);
        assert!(
            report.max_stretch < 0.01,
            "CCD={continuous}, max strain={}",
            report.max_stretch
        );
        assert!(report.max_target_error < 1.0e-6);
        // This is actual movement, not a rejected or stalled grasp command.
        assert!(cloth.positions()[1].x < cloth.mesh().rest_positions()[1].x - 0.005);
    }
}

#[test]
fn two_pulled_regions_do_not_open_a_gap_at_the_nearest_anchor_boundary() {
    let mut cloth = Cloth::new(
        GridBuilder::new(16, 2).size(0.5, 0.05).build().unwrap(),
        ClothMaterial {
            damping: 0.0,
            bend_compliance: 10000.0,
            ..Default::default()
        },
    )
    .unwrap();
    for i in [0, 15, 16, 31] {
        cloth
            .pin(i, cloth.positions()[i as usize] + Vec3::Y * 0.05)
            .unwrap();
    }
    let report = Solver::new()
        .step(
            &mut cloth,
            1.0 / 240.0,
            Vec3::ZERO,
            &SolverSettings::default(),
        )
        .unwrap();
    assert!(
        report.max_stretch < 0.1,
        "max strain={}",
        report.max_stretch
    );
    assert!(cloth.positions()[7].distance(cloth.positions()[8]) < 0.5 / 15.0 * 1.05);
}

#[test]
fn releasing_all_pins_leaves_no_distance_cap_on_inertial_motion() {
    let mut cloth = Cloth::new(
        GridBuilder::new(4, 4).build().unwrap(),
        ClothMaterial {
            damping: 0.0,
            ..Default::default()
        },
    )
    .unwrap();
    cloth.pin(0, cloth.positions()[0]).unwrap();
    let mut solver = Solver::new();
    solver
        .step(
            &mut cloth,
            1.0 / 240.0,
            Vec3::ZERO,
            &SolverSettings::default(),
        )
        .unwrap();
    cloth.unpin(0).unwrap();
    let velocity = Vec3::new(0.2, 0.1, -0.1);
    for i in 0..16 {
        cloth.set_velocity(i, velocity).unwrap();
    }
    let before = cloth.positions().to_vec();
    solver
        .step(
            &mut cloth,
            1.0 / 240.0,
            Vec3::ZERO,
            &SolverSettings::default(),
        )
        .unwrap();
    for ((p, old), v) in cloth.positions().iter().zip(before).zip(cloth.velocities()) {
        assert!((*p - old - velocity / 240.0).length() < 1.0e-6);
        assert!((*v - velocity).length() < 1.0e-4);
    }
}
