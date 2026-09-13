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

#[test]
fn closed_form_matches_original_chain_across_scales_and_fold_angles() {
    use rapier_cloth_core::constraints::bend::angle;
    #[cfg(feature = "f32")]
    let tolerance: Real = 3.0e-5;
    #[cfg(feature = "f64")]
    let tolerance: Real = 5.0e-13;
    // Deterministic, dependency-free sample sequence, including skewed hinges,
    // reversed shared edges, near-flat and nearly fully folded pairs.
    let mut seed = 12345_u64;
    let mut random = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        ((seed >> 32) as u32 as Real / u32::MAX as Real) * 2.0 - 1.0
    };
    for scale in [1.0e-4, 0.01, 1.0, 100.0, 1.0e4] {
        for sample in 0..400 {
            let mut p = [Vec3::ZERO; 4];
            for vertex in &mut p {
                *vertex = Vec3::new(random(), random(), random()) * scale;
            }
            let (old_angle, old_g) = reference_angle_and_gradients(p).unwrap();
            let (new_angle, new_g) = angle_and_gradients(p).unwrap();
            assert!(angle_difference(old_angle, new_angle).abs() < tolerance);
            assert_eq!(new_angle, angle(p).unwrap());
            for (a, b) in old_g.into_iter().zip(new_g) {
                assert!(
                    (a - b).length() <= tolerance * a.length().max(1.0 / scale),
                    "scale={scale}, sample={sample}, old={a:?}, new={b:?}"
                );
            }
        }
    }
    for p in [
        [Vec3::ZERO; 4],
        [Vec3::splat(Real::INFINITY); 4],
        [Vec3::splat(Real::NAN); 4],
    ] {
        assert!(angle_and_gradients(p).is_none());
        assert!(angle(p).is_none());
    }
}

#[test]
fn angle_difference_preserves_wrapping_and_boundary_behavior() {
    let pi = std::f64::consts::PI as Real;
    let tolerance = Real::EPSILON * 8.0;
    for a in [
        -100.0,
        -2.0 * pi,
        -pi,
        -pi + 1.0e-4,
        -0.0,
        0.0,
        0.1,
        pi - 1.0e-4,
        pi,
        2.0 * pi,
        100.0,
    ] {
        for rest in [-pi, -0.1, 0.0, 0.1, pi] {
            let delta = a - rest;
            let expected = delta.sin().atan2(delta.cos());
            assert!((angle_difference(a, rest) - expected).abs() <= tolerance);
        }
    }
    assert!(angle_difference(Real::INFINITY, 0.0).is_nan());
    assert!(angle_difference(Real::NAN, 0.0).is_nan());
}

// Independent oracle: the original differentiated atan2/normalization chain.
fn reference_angle_and_gradients(p: [Vec3; 4]) -> Option<(Real, [Vec3; 4])> {
    let e = p[1] - p[0];
    let u = p[2] - p[0];
    let v = p[3] - p[0];
    let raw_a = e.cross(u);
    let raw_b = v.cross(e);
    let le = e.length();
    let la = raw_a.length();
    let lb = raw_b.length();
    if [le, la, lb]
        .iter()
        .any(|n| !n.is_finite() || *n <= Real::MIN_POSITIVE)
    {
        return None;
    }
    let t = e / le;
    let a = raw_a / la;
    let b = raw_b / lb;
    let s = t.dot(a.cross(b));
    let c = a.dot(b);
    let denom = s * s + c * c;
    if denom <= Real::MIN_POSITIVE {
        return None;
    }
    let gs = c / denom;
    let gc = -s / denom;
    let ga = b.cross(t) * gs + b * gc;
    let gb = t.cross(a) * gs + a * gc;
    let gt = a.cross(b) * gs;
    let gna = (ga - a * a.dot(ga)) / la;
    let gnb = (gb - b * b.dot(gb)) / lb;
    let ge = (gt - t * t.dot(gt)) / le + u.cross(gna) + gnb.cross(v);
    let gu = gna.cross(e);
    let gv = e.cross(gnb);
    let gradients = [-ge - gu - gv, ge, gu, gv];
    if gradients.iter().any(|g| !g.is_finite()) {
        return None;
    }
    Some((s.atan2(c), gradients))
}
