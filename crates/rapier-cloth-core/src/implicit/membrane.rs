//! Analytic eigensystem of the isotropic membrane Hessian for a 3×2 deformation
//! gradient, after Smith et al. 2019 and Kim & Eberle 2020. It replaces the
//! numerical 6×6 eigen-solve of the positive-semidefinite projection.
use crate::{Real, Vec3};
use nalgebra::SMatrix;

/// Eigen-decomposition of the symmetric matrix `[[a, b], [b, c]]`, largest
/// eigenvalue first, with unit eigenvectors chosen to avoid cancellation.
fn symmetric2(a: Real, b: Real, c: Real) -> ([Real; 2], [[Real; 2]; 2]) {
    let mean = 0.5 * (a + c);
    let radius = (0.25 * (a - c) * (a - c) + b * b).sqrt();
    let (hi, lo) = (mean + radius, mean - radius);
    let first = if b == 0.0 {
        if a >= c { [1.0, 0.0] } else { [0.0, 1.0] }
    } else {
        // Both forms are eigenvectors of `hi`; the longer one is accurate.
        let p = [hi - c, b];
        let q = [b, hi - a];
        let v = if p[0] * p[0] + p[1] * p[1] >= q[0] * q[0] + q[1] * q[1] {
            p
        } else {
            q
        };
        let norm = (v[0] * v[0] + v[1] * v[1]).sqrt();
        [v[0] / norm, v[1] / norm]
    };
    ([hi, lo], [first, [-first[1], first[0]]])
}

/// Positive-semidefinite projection of the Hessian of
/// `0.5 mu (|f0|² + |f1|² - 2) - mu ln J + 0.5 lambda ln² J`, in the vec(F)
/// order `(f0, f1)`. Returns `None` for a degenerate deformation gradient, like
/// the energy evaluation.
pub(super) fn projected_hessian(
    f: [Vec3; 2],
    mu: Real,
    lambda: Real,
) -> Option<SMatrix<Real, 6, 6>> {
    let a = f[0].length_squared();
    let b = f[0].dot(f[1]);
    let c = f[1].length_squared();
    let det = a * c - b * b;
    if det <= 1e-16 || !det.is_finite() {
        return None;
    }
    // Singular values and right singular vectors from the Gram matrix; the
    // smaller eigenvalue comes from the determinant to avoid cancellation.
    let (gram, v) = symmetric2(a, b, c);
    let s1 = gram[0].sqrt();
    let s2 = (det / gram[0]).sqrt();
    let u1 = (f[0] * v[0][0] + f[1] * v[0][1]) / s1;
    let u2 = (f[0] * v[1][0] + f[1] * v[1][1]) / s2;
    let u3 = u1.cross(u2);
    // Derivatives of the energy in the singular values.
    let log_j = 0.5 * det.ln();
    let q = lambda * log_j - mu;
    let p1 = mu * s1 + q / s1;
    let p2 = mu * s2 + q / s2;
    let r = lambda * (1.0 - log_j) + mu;
    let p11 = mu + r / (s1 * s1);
    let p22 = mu + r / (s2 * s2);
    let p12 = lambda / (s1 * s2);
    let (stretch, e) = symmetric2(p11, p12, p22);
    let flip = if (s1 - s2).abs() <= 1e-6 * (s1 + s2) {
        // Limit of the quotient at equal singular values.
        0.5 * (p11 + p22) - p12
    } else {
        (p1 - p2) / (s1 - s2)
    };
    let twist = (p1 + p2) / (s1 + s2);
    let half = 0.5_f64.sqrt() as Real;
    // Eigenvectors as 3×2 matrices Q = Σ m_ij u_i v_jᵀ, vectorized by columns.
    let frame = |m: [[Real; 2]; 3]| -> [Real; 6] {
        let mut out = [0.0; 6];
        for (i, u) in [u1, u2, u3].iter().enumerate() {
            for j in 0..2 {
                if m[i][j] == 0.0 {
                    continue;
                }
                for col in 0..2 {
                    let w = m[i][j] * v[j][col];
                    out[3 * col] += w * u.x;
                    out[3 * col + 1] += w * u.y;
                    out[3 * col + 2] += w * u.z;
                }
            }
        }
        out
    };
    let modes = [
        (
            stretch[0],
            frame([[e[0][0], 0.0], [0.0, e[0][1]], [0.0, 0.0]]),
        ),
        (
            stretch[1],
            frame([[e[1][0], 0.0], [0.0, e[1][1]], [0.0, 0.0]]),
        ),
        (twist, frame([[0.0, -half], [half, 0.0], [0.0, 0.0]])),
        (flip, frame([[0.0, half], [half, 0.0], [0.0, 0.0]])),
        (p1 / s1, frame([[0.0, 0.0], [0.0, 0.0], [1.0, 0.0]])),
        (p2 / s2, frame([[0.0, 0.0], [0.0, 0.0], [0.0, 1.0]])),
    ];
    let mut h = SMatrix::<Real, 6, 6>::zeros();
    for (value, q) in modes {
        if value.is_nan() {
            return None;
        }
        if value <= 0.0 {
            continue;
        }
        for i in 0..6 {
            for j in 0..=i {
                h[(i, j)] += value * q[i] * q[j];
            }
        }
    }
    for i in 0..6 {
        for j in 0..i {
            h[(j, i)] = h[(i, j)];
        }
    }
    if h.iter().all(|v| v.is_finite()) {
        Some(h)
    } else {
        None
    }
}

#[cfg(all(test, feature = "f64"))]
mod tests {
    use super::*;

    fn numeric(f: [Vec3; 2], mu: Real, lambda: Real) -> SMatrix<Real, 6, 6> {
        let (_, _, hf) = super::super::membrane(f, mu, lambda, true).unwrap();
        let eig = ((hf + hf.transpose()) * 0.5).symmetric_eigen();
        let d = SMatrix::<Real, 6, 6>::from_diagonal(&eig.eigenvalues.map(|v| v.max(0.0)));
        eig.eigenvectors * d * eig.eigenvectors.transpose()
    }

    fn rotate(v: Vec3, axis: Vec3, angle: Real) -> Vec3 {
        let axis = axis.normalize();
        v * angle.cos() + axis.cross(v) * angle.sin() + axis * (axis.dot(v) * (1.0 - angle.cos()))
    }

    #[test]
    fn analytic_projection_matches_the_numerical_eigen_projection() {
        let mut seed = 91_u64;
        let mut random = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            (seed >> 33) as Real / (1u64 << 31) as Real - 0.5
        };
        let mut cases: Vec<[Vec3; 2]> = vec![
            [Vec3::X, Vec3::Y],                                    // rest
            [Vec3::X * 1.5, Vec3::Y * 1.5],                        // isotropic stretch
            [Vec3::X * 0.4, Vec3::Y * 0.4],                        // isotropic compression
            [Vec3::X * 1.3, Vec3::Y * 0.7],                        // anisotropic
            [Vec3::X, Vec3::new(0.6, 0.8, 0.0)],                   // shear
            [Vec3::X, Vec3::new(0.0, 1.0, 1e-7)],                  // near rest
            [Vec3::X * (1.0 + 1e-9), Vec3::Y * (1.0 - 1e-9)],      // nearly equal singular values
            [Vec3::new(1.0, 0.0, 0.3), Vec3::new(0.0, 1.0, -0.2)], // tilted
            [Vec3::X * 0.05, Vec3::Y * 0.05],                      // strong compression
        ];
        for _ in 0..200 {
            let base = [Vec3::X, Vec3::Y];
            let stretch = [1.0 + 0.8 * random(), 1.0 + 0.8 * random()];
            let shear = random();
            let axis = Vec3::new(random(), random(), random()) + Vec3::splat(1e-3);
            let angle = 3.0 * random();
            let f0 = rotate(base[0] * stretch[0], axis, angle);
            let f1 = rotate(base[0] * shear + base[1] * stretch[1], axis, angle);
            cases.push([f0, f1]);
        }
        for (mu, lambda) in [(2.5, 1.7), (3.0e5, 1.2e5), (1.0, 0.0)] {
            for &f in &cases {
                let analytic = projected_hessian(f, mu, lambda).unwrap();
                let reference = numeric(f, mu, lambda);
                let scale = reference.norm().max(mu);
                assert!(
                    (analytic - reference).norm() <= 1e-9 * scale,
                    "F={f:?} mu={mu} lambda={lambda}: {:e} vs {:e}",
                    (analytic - reference).norm(),
                    scale
                );
                // Every eigenvalue of the result is nonnegative.
                let eig = analytic.symmetric_eigen();
                assert!(eig.eigenvalues.min() >= -1e-9 * scale);
            }
        }
        assert!(projected_hessian([Vec3::X, Vec3::X * 2.0], 1.0, 1.0).is_none());
    }

    #[test]
    fn positive_definite_elements_are_reproduced_exactly_enough_to_skip_clamping() {
        let f = [Vec3::X * 1.2, Vec3::new(0.1, 1.1, 0.0)];
        let (_, _, hf) = super::super::membrane(f, 2.0, 1.0, true).unwrap();
        let eig = ((hf + hf.transpose()) * 0.5).symmetric_eigen();
        assert!(eig.eigenvalues.min() > 0.0);
        let analytic = projected_hessian(f, 2.0, 1.0).unwrap();
        assert!((analytic - hf).norm() <= 1e-10 * hf.norm());
    }
}
