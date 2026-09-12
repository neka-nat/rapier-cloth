use rapier_cloth_core::{
    Real, Vec3,
    constraints::bend::{angle_and_gradients, angle_difference},
};

#[test]
fn analytic_gradient_matches_finite_difference_and_rigid_motion() {
    #[cfg(feature = "f32")]
    let (h, tol) = (1.0e-3, 2.0e-3);
    #[cfg(feature = "f64")]
    let (h, tol) = (1.0e-6, 2.0e-8);
    for height in [0.0, 0.7, -0.8, 10.0] {
        let p = [
            Vec3::ZERO,
            Vec3::X,
            Vec3::new(0.2, 1.0, 0.1),
            Vec3::new(0.3, -0.6, height),
        ];
        let (theta, g) = angle_and_gradients(p).unwrap();
        assert!(g.iter().copied().sum::<Vec3>().length() < tol);
        assert!(
            p.iter()
                .zip(g)
                .map(|(p, g)| p.cross(g))
                .sum::<Vec3>()
                .length()
                < tol
        );
        for i in 0..4 {
            for j in 0..3 {
                let mut plus = p;
                let mut minus = p;
                plus[i][j] += h;
                minus[i][j] -= h;
                let d = angle_difference(
                    angle_and_gradients(plus).unwrap().0,
                    angle_and_gradients(minus).unwrap().0,
                ) / (2.0 * h);
                assert!(
                    (d - g[i][j]).abs() < tol,
                    "height {height}, {i},{j}: {d} != {}",
                    g[i][j]
                );
            }
        }
        let translated = p.map(|p| p + Vec3::new(2.0, 3.0, 4.0));
        assert!(angle_difference(angle_and_gradients(translated).unwrap().0, theta).abs() < tol);
    }
    assert!(angle_and_gradients([Vec3::ZERO; 4]).is_none());
    let pi = std::f64::consts::PI as Real;
    assert!(angle_difference(-pi + 0.001, pi - 0.001).abs() < 0.003);
}
