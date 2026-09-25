#[path = "support/surface_oracle.rs"]
mod oracle;
mod support;
use rapier_cloth::{Real, rapier::prelude::*, *};
use support::TestWorld;

// Two cropped 2x2 patches from the nominal pre-step-1629 state. Rest geometry
// remains unchanged; density masses are recomputed on this smaller mesh.
fn folded_patch(speed_scale: Real, friction: bool) -> Cloth {
    let grid = GridBuilder::new(32, 32)
        .size(0.5, 0.5)
        .origin(Vec3::new(-0.25, 0.005, -0.25))
        .build()
        .unwrap();
    let ids = [243, 244, 275, 276, 979, 980, 1011, 1012];
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/prediction_support.json")).unwrap();
    let positions: Vec<[Real; 3]> = serde_json::from_value(fixture["positions"].clone()).unwrap();
    let velocities: Vec<[Real; 3]> = serde_json::from_value(fixture["velocities"].clone()).unwrap();
    let mesh = ClothMesh::new(
        ids.map(|i| grid.rest_positions()[i]).to_vec(),
        vec![[0, 2, 1], [1, 2, 3], [4, 6, 5], [5, 6, 7]],
    )
    .unwrap();
    let mut cloth = Cloth::new(
        mesh,
        ClothMaterial {
            surface_density: 0.2,
            bend_compliance: 10000.0,
            damping: 0.1,
            ..Default::default()
        },
    )
    .unwrap();
    cloth
        .set_positions(
            &positions
                .into_iter()
                .map(Vec3::from_array)
                .collect::<Vec<_>>(),
        )
        .unwrap();
    for (i, velocity) in velocities.into_iter().enumerate() {
        cloth
            .set_velocity(i as u32, Vec3::from_array(velocity) * speed_scale)
            .unwrap();
    }
    cloth
        .set_contact_settings(Some(ClothContactSettings {
            thickness: 0.001,
            activation_margin: 0.0001,
            static_friction: if friction { 0.6 } else { 0.0 },
            kinetic_friction: if friction { 0.5 } else { 0.0 },
            self_collision: true,
            continuous_self_collision: true,
            rigid_surface_collision: true,
            continuous_rigid_collision: true,
            ..Default::default()
        }))
        .unwrap();
    cloth
}

#[test]
fn supported_fold_prediction_converges_without_pinning_the_lower_layer() {
    for friction in [false, true] {
        for speed in [1.0, 2.0, 4.0] {
            let mut test = TestWorld::new();
            test.rigid.colliders.insert(
                ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)).friction(if friction {
                    0.5
                } else {
                    0.0
                }),
            );
            let handle = test.cloth.add_cloth(folded_patch(speed, friction));
            let masses = test.cloth.cloth(handle).unwrap().masses().to_vec();
            let report = test
                .tick()
                .unwrap_or_else(|e| panic!("speed={speed}, friction={friction}: {e}"));
            let cloth = test.cloth.cloth(handle).unwrap();
            assert_eq!(report.cloths[0].1.iterations, 8);
            assert_eq!(cloth.masses(), masses);
            assert!(cloth.pins().is_empty());
            assert!(
                cloth
                    .positions()
                    .iter()
                    .all(|p| p.is_finite() && p.y >= 0.0004)
            );
            assert!(report.cloths[0].1.max_stretch < 0.01);
            let positions: Vec<_> = cloth
                .positions()
                .iter()
                .map(|p| {
                    #[cfg(feature = "f32")]
                    {
                        p.to_array().map(f64::from)
                    }
                    #[cfg(feature = "f64")]
                    {
                        p.to_array()
                    }
                })
                .collect();
            let triangles = cloth.mesh().triangles();
            for a in 0..2 {
                for b in 2..4 {
                    let p = triangles[a].map(|i| positions[i as usize]);
                    let q = triangles[b].map(|i| positions[i as usize]);
                    assert!(oracle::triangle_distance_squared(p, q).sqrt() >= 0.0009);
                }
            }
        }
    }
}

#[test]
fn restoring_a_supported_surface_gap_does_not_add_velocity() {
    let patch = GridBuilder::new(2, 2)
        .size(0.02, 0.02)
        .origin(Vec3::Y * 0.00141)
        .build()
        .unwrap();
    let mut positions = patch.rest_positions().to_vec();
    positions.extend(
        patch
            .rest_positions()
            .iter()
            .map(|p| Vec3::new(p.x, 0.0005, p.z)),
    );
    let mut triangles = patch.triangles().to_vec();
    triangles.extend(patch.triangles().iter().map(|t| t.map(|i| i + 4)));
    let mut cloth = Cloth::new(
        ClothMesh::new(positions, triangles).unwrap(),
        ClothMaterial::default(),
    )
    .unwrap();
    cloth
        .set_contact_settings(Some(ClothContactSettings {
            thickness: 0.001,
            activation_margin: 0.0001,
            static_friction: 0.0,
            kinetic_friction: 0.0,
            self_collision: true,
            continuous_self_collision: true,
            rigid_surface_collision: true,
            continuous_rigid_collision: true,
            ..Default::default()
        }))
        .unwrap();
    let mut test = TestWorld::new();
    test.rigid.gravity = Vec3::ZERO;
    test.rigid
        .colliders
        .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)).friction(0.0));
    let handle = test.cloth.add_cloth(cloth);
    let report = test.tick().unwrap();
    assert!(report.cloths[0].1.stabilized_contacts > 0);
    let cloth = test.cloth.cloth(handle).unwrap();
    assert!(cloth.pins().is_empty());
    for (i, (&p, &v)) in cloth.positions().iter().zip(cloth.velocities()).enumerate() {
        let height = if i < 4 { 0.0015 } else { 0.0005 };
        assert!((p.y - height).abs() < 1e-7, "particle {i}: {p:?}");
        assert!(
            v.length() < 1e-6,
            "gap restoration added velocity at {i}: {v:?}"
        );
    }
}
