use rapier_cloth_core::*;

#[test]
fn horizontal_cloth_drapes_for_ten_thousand_substeps() {
    let mesh = GridBuilder::new(32, 32).origin(Vec3::Y).build().unwrap();
    let mut cloth = Cloth::new(mesh, ClothMaterial::default()).unwrap();
    for i in 0..32 {
        cloth.pin(i, cloth.positions()[i as usize]).unwrap();
    }
    let mut solver = Solver::new();
    let mut worst: Real = 0.0;
    for step in 0..10000 {
        let report = solver
            .step(
                &mut cloth,
                1.0 / 240.0,
                Vec3::new(0.0, -9.81, 0.0),
                &SolverSettings::default(),
            )
            .unwrap_or_else(|e| panic!("step {step}: {e}"));
        assert!(report.max_target_error <= rapier_cloth_core::math::LENGTH_EPSILON);
        if step >= 9000 {
            worst = worst.max(report.p95_stretch);
        }
    }
    eprintln!("hanging cloth: worst final-1000 p95 stretch={worst}");
    assert!(worst <= 0.05, "p95 stretch {worst}");
    assert!(
        cloth.positions()[1023].y < 0.2,
        "cloth must actually drape below the pinned edge"
    );
}

#[test]
fn resolution_and_timestep_sweep_records_error() {
    for n in [16, 32, 64] {
        for divisor in [240, 480] {
            let mesh = GridBuilder::new(n, n).origin(Vec3::Y).build().unwrap();
            let mut c = Cloth::new(mesh, ClothMaterial::default()).unwrap();
            for i in 0..n {
                c.pin(i as u32, c.positions()[i]).unwrap();
            }
            let mut s = Solver::new();
            let mut last = StepReport::default();
            for _ in 0..divisor {
                last = s
                    .step(
                        &mut c,
                        1.0 / divisor as Real,
                        -Vec3::Y * 9.81,
                        &SolverSettings::default(),
                    )
                    .unwrap();
            }
            eprintln!(
                "grid={n}, h=1/{divisor}, iterations=8, p95={}, max={}",
                last.p95_stretch, last.max_stretch
            );
            assert!(last.max_stretch.is_finite());
        }
    }
}

#[test]
fn iteration_sweep_records_error_for_same_initial_condition() {
    for iterations in [4, 8, 16] {
        let mesh = GridBuilder::new(16, 16).origin(Vec3::Y).build().unwrap();
        let mut cloth = Cloth::new(mesh, ClothMaterial::default()).unwrap();
        for i in 0..16 {
            cloth.pin(i, cloth.positions()[i as usize]).unwrap();
        }
        let settings = SolverSettings {
            iterations,
            ..Default::default()
        };
        let mut solver = Solver::new();
        let mut last = StepReport::default();
        for _ in 0..240 {
            last = solver
                .step(&mut cloth, 1.0 / 240.0, -Vec3::Y * 9.81, &settings)
                .unwrap();
        }
        eprintln!(
            "iteration sweep: grid=16, h=1/240, iterations={iterations}, p95={}, max={}",
            last.p95_stretch, last.max_stretch
        );
        assert!(last.max_stretch.is_finite());
    }
}
