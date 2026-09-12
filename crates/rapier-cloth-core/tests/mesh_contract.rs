use rapier_cloth_core::{ClothMesh, GridBuilder, Real, Vec3};

#[test]
fn grid_counts_area_and_order() {
    for n in [2, 16, 32, 64] {
        let m = GridBuilder::new(n, n).build().unwrap();
        assert_eq!(m.rest_positions().len(), n * n);
        assert_eq!(m.triangles().len(), 2 * (n - 1) * (n - 1));
        assert_eq!(m.edges().len(), 2 * n * (n - 1) + (n - 1) * (n - 1));
        assert_eq!(m.hinges().len(), m.edges().len() - 4 * (n - 1));
        let mass: Real = m.vertex_areas().iter().map(|a| a * 0.2).sum();
        assert!((mass - 0.2).abs() < 1.0e-5, "{n}: {mass}");
        let m2 = GridBuilder::new(n, n).build().unwrap();
        assert_eq!(m.triangles(), m2.triangles());
        assert_eq!(
            m.edges().iter().map(|e| e.vertices).collect::<Vec<_>>(),
            m2.edges().iter().map(|e| e.vertices).collect::<Vec<_>>()
        );
    }
}

#[test]
fn invalid_topology_is_rejected_without_repair() {
    let p = vec![Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::Z, -Vec3::Y];
    for (positions, faces) in [
        (p[..3].to_vec(), vec![[0, 1, 9]]),
        (p[..3].to_vec(), vec![[0, 1, 1]]),
        (p[..3].to_vec(), vec![[0, 1, 2], [2, 1, 0]]),
        (p[..4].to_vec(), vec![[0, 1, 2], [0, 1, 3]]),
        (p.clone(), vec![[0, 1, 2], [1, 0, 3], [0, 1, 4]]),
        (p.clone(), vec![[0, 1, 2]]),
        (vec![Vec3::ZERO, Vec3::X, Vec3::X * 2.0], vec![[0, 1, 2]]),
        (
            vec![Vec3::splat(Real::NAN), Vec3::X, Vec3::Y],
            vec![[0, 1, 2]],
        ),
    ] {
        assert!(ClothMesh::new(positions, faces).is_err());
    }
    assert!(GridBuilder::new(1, 4).build().is_err());
    assert!(GridBuilder::new(4, 4).size(-1.0, 1.0).build().is_err());
    assert!(
        GridBuilder::new(4, 4)
            .axes(Vec3::X, Vec3::X)
            .build()
            .is_err()
    );
}
