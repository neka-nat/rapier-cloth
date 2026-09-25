use rapier_cloth_core::*;

#[test]
fn ray_hits_material_points_with_transforms_winding_and_shared_edges() {
    for tilted in [false, true] {
        let u = if tilted {
            Vec3::new(0.6, 0.8, 0.0)
        } else {
            Vec3::X
        };
        let v = Vec3::Z;
        let n = v.cross(u);
        let mesh = GridBuilder::new(2, 2)
            .origin(Vec3::new(3.0, -2.0, 1.0))
            .axes(u, v)
            .build()
            .unwrap();
        for reverse in [false, true] {
            let triangles: Vec<_> = mesh
                .triangles()
                .iter()
                .map(|t| if reverse { [t[2], t[1], t[0]] } else { *t })
                .collect();
            let surface = SurfaceView {
                positions: mesh.rest_positions(),
                triangles: &triangles,
            };
            for k in 1..20 {
                let f = k as Real / 20.0;
                // The entire shared diagonal, not only face interiors.
                let target = mesh.rest_positions()[1] * (1.0 - f) + mesh.rest_positions()[2] * f;
                let hit = surface
                    .raycast(
                        SurfaceRay {
                            origin: target + n * 2.0,
                            direction: -n * 3.0,
                            max_distance: 3.0,
                        },
                        SurfaceQueryLimits::default(),
                    )
                    .unwrap()
                    .unwrap();
                assert!((hit.distance - 2.0).abs() < 2.0e-6, "{hit:?}");
                assert!(hit.position.distance(target) < 2.0e-6);
                assert!(surface.point_position(hit.point).unwrap().distance(target) < 2.0e-6);
                assert!(hit.normal.dot(if reverse { -n } else { n }) > 0.9999);
            }
        }
    }
}

#[test]
fn closest_and_swept_queries_cover_face_edge_vertex_tangent_and_initial_overlap() {
    let p = [Vec3::ZERO, Vec3::X, Vec3::Y];
    let surface = SurfaceView {
        positions: &p,
        triangles: &[[0, 1, 2]],
    };
    let limits = SurfaceQueryLimits::default();
    let nearest = surface
        .closest_point(Vec3::new(0.2, 0.3, 0.5), 1.0, limits)
        .unwrap()
        .unwrap();
    assert!(nearest.position.distance(Vec3::new(0.2, 0.3, 0.0)) < 1.0e-6);
    assert!((nearest.distance - 0.5).abs() < 1.0e-6);
    let cases = [
        (
            Vec3::new(0.2, 0.3, 1.0),
            -Vec3::Z,
            0.9,
            Vec3::new(0.2, 0.3, 0.0),
        ),
        (
            Vec3::new(-1.0, 0.3, 0.0),
            Vec3::X,
            0.9,
            Vec3::new(0.0, 0.3, 0.0),
        ),
        (
            Vec3::new(-1.0, -1.0, 0.0),
            Vec3::new(1.0, 1.0, 0.0),
            2.0_f64.sqrt() as Real - 0.1,
            Vec3::ZERO,
        ),
        (Vec3::new(-1.0, -0.1, 0.0), Vec3::X, 1.0, Vec3::ZERO),
        (
            Vec3::new(0.2, 0.3, 0.05),
            Vec3::Z,
            0.0,
            Vec3::new(0.2, 0.3, 0.0),
        ),
    ];
    for (origin, direction, distance, point) in cases {
        let hit = surface
            .sweep_sphere(
                SurfaceRay {
                    origin,
                    direction,
                    max_distance: 2.0,
                },
                0.1,
                limits,
            )
            .unwrap()
            .unwrap();
        assert!(
            (hit.distance - distance).abs() < 2.0e-6,
            "{hit:?}, expected {distance}"
        );
        assert!(hit.position.distance(point) < 2.0e-6, "{hit:?}");
    }
    assert!(
        surface
            .sweep_sphere(
                SurfaceRay {
                    origin: Vec3::new(10.0, 10.0, 1.0),
                    direction: -Vec3::Z,
                    max_distance: 2.0
                },
                0.1,
                limits
            )
            .unwrap()
            .is_none()
    );
}

#[test]
fn invalid_geometry_and_exhausted_work_cannot_expose_an_occluded_layer() {
    let p = [
        Vec3::ZERO,
        Vec3::X,
        Vec3::Z,
        Vec3::Y,
        Vec3::Y + Vec3::X,
        Vec3::Y + Vec3::Z,
    ];
    let ray = SurfaceRay {
        origin: Vec3::new(0.2, 2.0, 0.2),
        direction: -Vec3::Y,
        max_distance: 3.0,
    };
    let view = SurfaceView {
        positions: &p,
        triangles: &[[0, 2, 1], [3, 5, 4]],
    };
    let hit = view
        .raycast(ray, SurfaceQueryLimits::default())
        .unwrap()
        .unwrap();
    assert_eq!(hit.point.triangle(), 1);
    assert!((hit.distance - 1.0).abs() < 1.0e-6);
    assert!(matches!(
        view.raycast(
            ray,
            SurfaceQueryLimits {
                triangles: 1,
                ..Default::default()
            }
        ),
        Err(ClothError::SurfaceQueryBudgetExceeded { limit: 1 })
    ));
    let invalid = SurfaceView {
        positions: &p,
        triangles: &[[0, 2, 1], [3, 3, 3]],
    };
    assert!(invalid.raycast(ray, SurfaceQueryLimits::default()).is_err());
    let invalid = SurfaceView {
        positions: &p,
        triangles: &[[0, 2, 99]],
    };
    assert!(
        invalid
            .closest_point(Vec3::ZERO, 1.0, SurfaceQueryLimits::default())
            .is_err()
    );
    for direction in [Vec3::ZERO, Vec3::splat(Real::NAN)] {
        assert!(
            view.raycast(
                SurfaceRay { direction, ..ray },
                SurfaceQueryLimits::default()
            )
            .is_err()
        );
    }
    assert!(SurfacePoint::new(0, [-0.1, 0.6, 0.5]).is_err());
    assert!(SurfacePoint::new(0, [0.1, 0.2, 0.3]).is_err());
}

#[test]
fn material_patch_uses_edge_connectivity_and_never_silently_truncates() {
    let base = GridBuilder::new(4, 4).size(0.3, 0.3).build().unwrap();
    let mut positions = base.rest_positions().to_vec();
    positions.extend(base.rest_positions().iter().map(|p| *p + Vec3::Y * 0.001));
    let mut triangles = base.triangles().to_vec();
    triangles.extend(base.triangles().iter().map(|t| t.map(|i| i + 16)));
    let mesh = ClothMesh::new(positions, triangles).unwrap();
    let point = SurfacePoint::new(0, [1.0, 0.0, 0.0]).unwrap();
    let patch = mesh
        .surface_patch(point, 0.10001, SurfaceQueryLimits::default())
        .unwrap();
    assert_eq!(patch, vec![0, 1, 4]);
    assert!(matches!(
        mesh.surface_patch(
            point,
            0.10001,
            SurfaceQueryLimits {
                patch_vertices: 2,
                ..Default::default()
            }
        ),
        Err(ClothError::SurfaceQueryBudgetExceeded { limit: 2 })
    ));
    let interior = SurfacePoint::new(0, [1.0 / 3.0; 3]).unwrap();
    assert!(
        mesh.surface_patch(interior, 0.001, SurfaceQueryLimits::default())
            .is_err()
    );
}
