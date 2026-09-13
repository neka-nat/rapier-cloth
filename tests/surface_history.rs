mod support;
use rapier_cloth::*;
use support::*;

struct Plane;
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
        _: &[Vec3],
        _: ContactStage,
        out: &mut Vec<SurfaceContact>,
    ) -> Result<(), ClothError> {
        out.push(SurfaceContact {
            key: SurfaceContactKey {
                other_cloth: None,
                features: [
                    SurfaceFeature::Vertex(0),
                    SurfaceFeature::External {
                        object: 42,
                        feature: 0,
                    },
                ],
            },
            particles: [0; 4],
            weights: [1.0, 0.0, 0.0, 0.0],
            normal: Vec3::Y,
            offset: Vec3::ZERO,
            surface_velocity: Vec3::ZERO,
            separation: 0.01,
            static_friction: 0.6,
            kinetic_friction: 0.5,
        });
        Ok(())
    }
}
fn cached_cloth() -> Cloth {
    let mut cloth = Cloth::new(
        GridBuilder::new(2, 2)
            .origin(Vec3::Y * 0.01)
            .build()
            .unwrap(),
        ClothMaterial::default(),
    )
    .unwrap();
    Solver::new()
        .step_with_contacts(
            &mut cloth,
            1.0 / 240.0,
            -Vec3::Y * 9.81,
            &SolverSettings::default(),
            &[],
            &mut Plane,
        )
        .unwrap();
    assert_eq!(cloth.contact_history_len(), 1);
    cloth
}
#[test]
fn rapier_checkpoint_restores_surface_history_and_failed_second_cloth_does_not_commit_first() {
    let mut scene = TestWorld::new();
    let first = scene.cloth.add_cloth(cached_cloth());
    let second = scene.grid(2, 1.0, Vec3::Y * 2.0);
    let checkpoint = scene.cloth.checkpoint().unwrap();
    scene.tick().unwrap();
    assert_eq!(scene.cloth.cloth(first).unwrap().contact_history_len(), 0);
    scene.cloth.restore(&checkpoint).unwrap();
    assert_eq!(scene.cloth.cloth(first).unwrap().contact_history_len(), 1);
    let before = scene.cloth.cloth(first).unwrap().positions().to_vec();
    // This failure occurs after the first cloth has been staged successfully.
    scene
        .cloth
        .cloth_mut(second)
        .unwrap()
        .set_force(0, Vec3::splat(Real::MAX))
        .unwrap();
    // Restore the external time contract for this test's rigid world, which has
    // no rigid bodies, colliders, joints or caches to restore.
    scene.step = 0;
    assert!(scene.tick().is_err());
    assert_eq!(scene.cloth.cloth(first).unwrap().positions(), before);
    assert_eq!(scene.cloth.cloth(first).unwrap().contact_history_len(), 1);
    assert_eq!(scene.cloth.next_step_index(), 0);
}

fn stacked_cloth(gap: Real) -> Cloth {
    let grid = GridBuilder::new(2, 2).size(0.1, 0.1).build().unwrap();
    let mut p = grid.rest_positions().to_vec();
    p.extend(grid.rest_positions().iter().map(|p| *p + Vec3::Y * gap));
    let mut t = grid.triangles().to_vec();
    t.extend(grid.triangles().iter().map(|t| t.map(|i| i + 4)));
    let mut cloth = Cloth::new(ClothMesh::new(p, t).unwrap(), ClothMaterial::default()).unwrap();
    cloth
        .set_contact_settings(Some(ClothContactSettings::default()))
        .unwrap();
    for i in 0..4 {
        cloth.pin(i, cloth.positions()[i as usize]).unwrap();
    }
    cloth
}

#[test]
fn built_in_self_contact_settings_and_history_restore_after_another_cloth_fails() {
    let mut scene = TestWorld::new();
    let first = scene.cloth.add_cloth(stacked_cloth(0.001));
    scene.tick().unwrap();
    let checkpoint = scene.cloth.checkpoint().unwrap();
    let before = scene.cloth.cloth(first).unwrap().clone();
    assert!(before.contact_history_len() > 0);
    scene.tick().unwrap();
    let expected = scene.cloth.cloth(first).unwrap().clone();
    scene.cloth.restore(&checkpoint).unwrap();
    // There are no rigid bodies/colliders/joints in this test; only the public
    // external step index needs restoring alongside the cloth checkpoint.
    scene.step = 1;
    let mut bad = stacked_cloth(0.0008);
    for i in 4..8 {
        bad.pin(i, bad.positions()[i as usize]).unwrap();
    }
    scene.cloth.add_cloth(bad);
    assert!(scene.tick().is_err());
    let after = scene.cloth.cloth(first).unwrap();
    assert_eq!(after.positions(), before.positions());
    assert_eq!(after.velocities(), before.velocities());
    assert_eq!(after.contact_history_len(), before.contact_history_len());
    scene
        .cloth
        .cloth_mut(first)
        .unwrap()
        .set_contact_settings(None)
        .unwrap();
    scene.cloth.restore(&checkpoint).unwrap();
    assert_eq!(
        scene.cloth.cloth(first).unwrap().contact_settings(),
        before.contact_settings()
    );
    scene.step = 1;
    scene.tick().unwrap();
    let actual = scene.cloth.cloth(first).unwrap();
    assert_eq!(actual.positions(), expected.positions());
    assert_eq!(actual.velocities(), expected.velocities());
    assert_eq!(actual.contact_history_len(), expected.contact_history_len());
}
