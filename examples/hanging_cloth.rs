use rapier_cloth::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mesh = GridBuilder::new(32, 32).origin(Vec3::Y).build()?;
    let mut cloth = Cloth::new(mesh, ClothMaterial::default())?;
    for i in 0..32 {
        cloth.pin(i, cloth.positions()[i as usize])?;
    }
    let mut solver = Solver::new();
    let mut report = StepReport::default();
    for _ in 0..2400 {
        report = solver.step(
            &mut cloth,
            1.0 / 240.0,
            -Vec3::Y * 9.81,
            &SolverSettings::default(),
        )?;
    }
    println!(
        "particles={}, p95_stretch={}, max_stretch={}, pin_error={}",
        cloth.positions().len(),
        report.p95_stretch,
        report.max_stretch,
        report.max_target_error
    );
    Ok(())
}
