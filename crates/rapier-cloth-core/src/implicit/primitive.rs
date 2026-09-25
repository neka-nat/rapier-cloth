//! One distance-based potential per parent VF/EE pair. Exact local derivatives
//! are computed before the local Hessian is projected for the Newton direction.
use super::*;
use crate::SurfaceFeature;
use crate::collision::geometry::{closest_segments, closest_triangle};
use nalgebra::SVector;
use std::ops::{Add, Div, Mul, Neg, Sub};

type Gradient = SVector<Real, 12>;
type Hessian = SMatrix<Real, 12, 12>;
// First-order AD carries no Hessian. Keep the scalar/gradient arithmetic shared
// with second order, including operation order at contact-feature boundaries.
pub(super) struct Order<const H: bool>;
pub(super) trait Curvature {
    type Storage: Copy + std::fmt::Debug;
    fn zero() -> Self::Storage;
    fn compose(_h: Self::Storage, _g: Gradient, _d: Real, _dd: Real) -> Self::Storage {
        Self::zero()
    }
    fn add(_a: Self::Storage, _b: Self::Storage) -> Self::Storage {
        Self::zero()
    }
    fn scale(_h: Self::Storage, _b: Real) -> Self::Storage {
        Self::zero()
    }
    fn product(
        _a: Self::Storage,
        _b: Self::Storage,
        _ae: Real,
        _be: Real,
        _ag: Gradient,
        _bg: Gradient,
    ) -> Self::Storage {
        Self::zero()
    }
    fn finite(_h: Self::Storage) -> bool {
        true
    }
}
impl Curvature for Order<false> {
    type Storage = ();
    fn zero() {}
}
impl Curvature for Order<true> {
    type Storage = Hessian;
    fn zero() -> Hessian {
        Hessian::zeros()
    }
    fn compose(h: Hessian, g: Gradient, d: Real, dd: Real) -> Hessian {
        h * d + g * g.transpose() * dd
    }
    fn add(a: Hessian, b: Hessian) -> Hessian {
        a + b
    }
    fn scale(h: Hessian, b: Real) -> Hessian {
        h * b
    }
    fn product(a: Hessian, b: Hessian, ae: Real, be: Real, ag: Gradient, bg: Gradient) -> Hessian {
        a * be + b * ae + ag * bg.transpose() + bg * ag.transpose()
    }
    fn finite(h: Hessian) -> bool {
        h.iter().all(|v| v.is_finite())
    }
}

#[derive(Clone, Copy, Debug)]
pub(super) struct Jet<const H: bool>
where
    Order<H>: Curvature,
{
    pub e: Real,
    pub g: Gradient,
    pub h: <Order<H> as Curvature>::Storage,
}
impl<const H: bool> Jet<H>
where
    Order<H>: Curvature,
{
    fn scalar(e: Real) -> Self {
        Self {
            e,
            g: Gradient::zeros(),
            h: Order::<H>::zero(),
        }
    }
    fn variable(e: Real, i: usize) -> Self {
        let mut a = Self::scalar(e);
        a.g[i] = 1.;
        a
    }
    fn compose(self, e: Real, d: Real, dd: Real) -> Self {
        Self {
            e,
            g: self.g * d,
            h: Order::<H>::compose(self.h, self.g, d, dd),
        }
    }
    fn sqrt(self) -> Self {
        let e = self.e.sqrt();
        self.compose(e, 0.5 / e, -0.25 / (self.e * e))
    }
    fn inverse(self) -> Self {
        self.compose(1. / self.e, -1. / self.e.powi(2), 2. / self.e.powi(3))
    }
}
impl<const H: bool> Add for Jet<H>
where
    Order<H>: Curvature,
{
    type Output = Self;
    fn add(self, b: Self) -> Self {
        Self {
            e: self.e + b.e,
            g: self.g + b.g,
            h: Order::<H>::add(self.h, b.h),
        }
    }
}
impl<const H: bool> Neg for Jet<H>
where
    Order<H>: Curvature,
{
    type Output = Self;
    fn neg(self) -> Self {
        self * -1.
    }
}
impl<const H: bool> Sub for Jet<H>
where
    Order<H>: Curvature,
{
    type Output = Self;
    fn sub(self, b: Self) -> Self {
        self + -b
    }
}
impl<const H: bool> Mul<Real> for Jet<H>
where
    Order<H>: Curvature,
{
    type Output = Self;
    fn mul(self, b: Real) -> Self {
        Self {
            e: self.e * b,
            g: self.g * b,
            h: Order::<H>::scale(self.h, b),
        }
    }
}
impl<const H: bool> Mul for Jet<H>
where
    Order<H>: Curvature,
{
    type Output = Self;
    fn mul(self, b: Self) -> Self {
        Self {
            e: self.e * b.e,
            g: self.g * b.e + b.g * self.e,
            h: Order::<H>::product(self.h, b.h, self.e, b.e, self.g, b.g),
        }
    }
}
// Quotient differentiation is multiplication by the differentiated reciprocal.
#[allow(clippy::suspicious_arithmetic_impl)]
impl<const H: bool> Div for Jet<H>
where
    Order<H>: Curvature,
{
    type Output = Self;
    fn div(self, b: Self) -> Self {
        self * b.inverse()
    }
}
#[allow(type_alias_bounds)]
type V<const H: bool>
where
    Order<H>: Curvature,
= [Jet<H>; 3];
fn sub<const H: bool>(a: V<H>, b: V<H>) -> V<H>
where
    Order<H>: Curvature,
{
    std::array::from_fn(|i| a[i] - b[i])
}
fn dot<const H: bool>(a: V<H>, b: V<H>) -> Jet<H>
where
    Order<H>: Curvature,
{
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross<const H: bool>(a: V<H>, b: V<H>) -> V<H>
where
    Order<H>: Curvature,
{
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn pp<const H: bool>(p: V<H>, a: V<H>) -> Jet<H>
where
    Order<H>: Curvature,
{
    let d = sub(p, a);
    dot(d, d)
}
fn pe<const H: bool>(p: V<H>, a: V<H>, b: V<H>) -> Jet<H>
where
    Order<H>: Curvature,
{
    let u = sub(b, a);
    let w = sub(p, a);
    let t = dot(w, u) / dot(u, u);
    let d = std::array::from_fn(|i| w[i] - u[i] * t);
    dot(d, d)
}

pub(super) fn is_parent(c: &SurfaceContact) -> bool {
    c.key.other_cloth.is_none()
        && matches!(
            c.key.features,
            [SurfaceFeature::Vertex(_), SurfaceFeature::Face(_)]
                | [SurfaceFeature::Edge(_), SurfaceFeature::Edge(_)]
        )
}
fn distance_squared<const H: bool>(
    c: &SurfaceContact,
    p: [Vec3; 4],
    q: [V<H>; 4],
) -> Result<Jet<H>, ClothError>
where
    Order<H>: Curvature,
{
    if matches!(c.key.features[0], SurfaceFeature::Vertex(_)) {
        let w = closest_triangle(p[0], [p[1], p[2], p[3]])
            .ok_or(ClothError::DegenerateConstraint)?
            .barycentric;
        let used: Vec<_> = (0..3).filter(|&i| w[i] != 0.0).collect();
        Ok(match *used.as_slice() {
            [a] => pp(q[0], q[a + 1]),
            [a, b] => pe(q[0], q[a + 1], q[b + 1]),
            _ => {
                let n = cross(sub(q[2], q[1]), sub(q[3], q[1]));
                let d = dot(sub(q[0], q[1]), n);
                d * d / dot(n, n)
            }
        })
    } else {
        let [s, t] = closest_segments([p[0], p[1]], [p[2], p[3]])
            .ok_or(ClothError::DegenerateConstraint)?
            .parameters;
        let a = if s == 0. {
            Some(0)
        } else if s == 1. {
            Some(1)
        } else {
            None
        };
        let b = if t == 0. {
            Some(2)
        } else if t == 1. {
            Some(3)
        } else {
            None
        };
        Ok(match (a, b) {
            (Some(a), Some(b)) => pp(q[a], q[b]),
            (Some(a), None) => pe(q[a], q[2], q[3]),
            (None, Some(b)) => pe(q[b], q[0], q[1]),
            (None, None) => {
                let n = cross(sub(q[1], q[0]), sub(q[3], q[2]));
                let d = dot(sub(q[0], q[2]), n);
                d * d / dot(n, n)
            }
        })
    }
}
fn mollifier<const H: bool>(c: &SurfaceContact, q: [V<H>; 4], rest: &[Vec3]) -> Jet<H>
where
    Order<H>: Curvature,
{
    if !matches!(c.key.features[0], SurfaceFeature::Edge(_)) {
        return Jet::scalar(1.);
    }
    let p = c.particles.map(|i| rest[i as usize]);
    let eps = 1e-3 * (p[1] - p[0]).length_squared() * (p[3] - p[2]).length_squared();
    let n = cross(sub(q[1], q[0]), sub(q[3], q[2]));
    let r = dot(n, n) * (1. / eps);
    if r.e < 1. {
        r * 2. - r * r
    } else {
        Jet::scalar(1.)
    }
}
pub(super) fn normal_weight(c: &SurfaceContact, x: &[Vec3], rest: &[Vec3]) -> Real {
    let p = c.particles.map(|i| x[i as usize]);
    let q = std::array::from_fn(|i| std::array::from_fn(|j| Jet::<false>::scalar(p[i][j])));
    mollifier(c, q, rest).e
}
pub(super) fn evaluate<const H: bool>(
    c: &SurfaceContact,
    x: &[Vec3],
    rest: &[Vec3],
    band: Real,
    k: Real,
) -> Result<Jet<H>, ClothError>
where
    Order<H>: Curvature,
{
    let p = c.particles.map(|i| x[i as usize]);
    let q =
        std::array::from_fn(|i| std::array::from_fn(|j| Jet::<H>::variable(p[i][j], i * 3 + j)));
    let d2 = distance_squared(c, p, q)?;
    if d2.e <= 0. || !d2.e.is_finite() {
        return Err(ClothError::UnresolvedSurfaceContact);
    }
    let gap = d2.sqrt() - Jet::scalar(c.separation);
    let b = barrier(gap.e, band, k).ok_or(ClothError::UnresolvedSurfaceContact)?;
    // Check feasibility before weighting even at exact parallelism, where m=0.
    if b[0] == 0. {
        return Ok(Jet::scalar(0.));
    }
    let value = gap.compose(b[0], b[1], b[2]) * mollifier(c, q, rest);
    if !value.e.is_finite()
        || value.g.iter().any(|v| !v.is_finite())
        || (H && !Order::<H>::finite(value.h))
    {
        return Err(ClothError::NonFiniteState);
    }
    Ok(value)
}
pub(super) fn project(h: Hessian) -> Hessian {
    let eig = ((h + h.transpose()) * 0.5).symmetric_eigen();
    let d = Hessian::from_diagonal(&eig.eigenvalues.map(|v| v.max(0.)));
    eig.eigenvectors * d * eig.eigenvectors.transpose()
}

#[cfg(all(test, feature = "f64"))]
mod tests {
    use super::*;
    fn contact(p: [Vec3; 4], edge: bool) -> SurfaceContact {
        let (features, weights) = if edge {
            let [s, t] = closest_segments([p[0], p[1]], [p[2], p[3]])
                .unwrap()
                .parameters;
            (
                [SurfaceFeature::Edge([0, 1]), SurfaceFeature::Edge([2, 3])],
                [1. - s, s, t - 1., -t],
            )
        } else {
            let w = closest_triangle(p[0], [p[1], p[2], p[3]])
                .unwrap()
                .barycentric;
            (
                [SurfaceFeature::Vertex(0), SurfaceFeature::Face(0)],
                [1., -w[0], -w[1], -w[2]],
            )
        };
        let delta = (0..4).map(|i| p[i] * weights[i]).sum::<Vec3>();
        SurfaceContact {
            key: crate::SurfaceContactKey {
                other_cloth: None,
                features,
            },
            particles: [0, 1, 2, 3],
            weights,
            normal: delta.normalize(),
            offset: Vec3::ZERO,
            surface_velocity: Vec3::ZERO,
            separation: 0.000318,
            static_friction: 0.5,
            kinetic_friction: 0.5,
        }
    }
    fn examples() -> Vec<([Vec3; 4], bool)> {
        let vf = |p| ([p, Vec3::ZERO, Vec3::X, Vec3::Z], false);
        vec![
            vf(Vec3::new(0.2, 0.0005, 0.2)),
            vf(Vec3::new(-0.0001, 0.00045, 0.2)),
            vf(Vec3::new(-0.0001, 0.00045, -0.0001)),
            (
                [
                    Vec3::new(-0.5, 0., 0.),
                    Vec3::new(0.5, 0., 0.),
                    Vec3::new(0., 0.0005, -0.5),
                    Vec3::new(0., 0.0005, 0.5),
                ],
                true,
            ),
            (
                [
                    Vec3::new(-0.5, 0., 0.),
                    Vec3::new(0.5, 0., 0.),
                    Vec3::new(0.5001, 0.00045, -0.5),
                    Vec3::new(0.5001, 0.00045, 0.5),
                ],
                true,
            ),
            (
                [
                    Vec3::new(-0.5, 0., 0.),
                    Vec3::new(0.5, 0., 0.),
                    Vec3::new(0.5001, 0.0004, 0.0001),
                    Vec3::new(1.5, 0.0004, 0.3),
                ],
                true,
            ),
            (
                [
                    Vec3::ZERO,
                    Vec3::X,
                    Vec3::new(0., 0.0005, -0.002),
                    Vec3::new(1., 0.0005, 0.002),
                ],
                true,
            ),
        ]
    }
    #[test]
    fn all_closest_feature_branches_have_consistent_energy_gradient_and_hessian() {
        let eps = 1e-8;
        for (case, (p, edge)) in examples().into_iter().enumerate() {
            let c = contact(p, edge);
            let a = evaluate::<true>(&c, &p, &p, 0.000318, 30.).unwrap();
            assert!(a.e > 0.);
            for j in 0..12 {
                let mut lo = p;
                let mut hi = p;
                lo[j / 3][j % 3] -= eps;
                hi[j / 3][j % 3] += eps;
                let l = evaluate::<false>(&c, &lo, &p, 0.000318, 30.).unwrap();
                let r = evaluate::<false>(&c, &hi, &p, 0.000318, 30.).unwrap();
                let fd = (r.e - l.e) / (2. * eps);
                assert!(
                    (fd - a.g[j]).abs() < 1e-8 + 1e-4 * a.g[j].abs(),
                    "case {case}, gradient {j}, {fd} {}",
                    a.g[j]
                );
                for i in 0..12 {
                    let fd = (r.g[i] - l.g[i]) / (2. * eps);
                    assert!(
                        (fd - a.h[(i, j)]).abs() < 1e-3 + 1e-4 * a.h[(i, j)].abs(),
                        "case {case}, Hessian {i},{j}: {fd} {}",
                        a.h[(i, j)]
                    );
                }
            }
            let projected = project(a.h);
            let eig = projected.symmetric_eigen();
            assert!(eig.eigenvalues.min() >= -1e-8 * projected.norm().max(1.));
        }
    }
    #[test]
    fn face_edge_vertex_transitions_are_continuous_without_snapping() {
        for boundary in [0., 1e-6] {
            let p = [
                Vec3::new(boundary, 0.0005, 0.2),
                Vec3::ZERO,
                Vec3::X,
                Vec3::Z,
            ];
            let c = contact(p, false);
            let mut lo = p;
            let mut hi = p;
            lo[0].x -= 1e-9;
            hi[0].x += 1e-9;
            let a = evaluate::<true>(&c, &lo, &p, 0.000318, 30.).unwrap();
            let b = evaluate::<true>(&c, &hi, &p, 0.000318, 30.).unwrap();
            assert!((a.e - b.e).abs() < 1e-12);
            assert!((a.g - b.g).norm() < 1e-6);
        }
        // Closest point on the first segment moves through its endpoint.
        let p = [
            Vec3::new(-0.5, 0., 0.),
            Vec3::new(0.5, 0., 0.),
            Vec3::new(0.5, 0.0005, -0.5),
            Vec3::new(0.5, 0.0005, 0.5),
        ];
        let c = contact(p, true);
        let mut lo = p;
        let mut hi = p;
        for i in [2, 3] {
            lo[i].x -= 1e-9;
            hi[i].x += 1e-9;
        }
        let a = evaluate::<true>(&c, &lo, &p, 0.000318, 30.).unwrap();
        let b = evaluate::<true>(&c, &hi, &p, 0.000318, 30.).unwrap();
        assert!((a.e - b.e).abs() < 1e-12);
        assert!((a.g - b.g).norm() < 1e-6);
    }
    #[test]
    fn mollifier_and_distance_are_rigid_invariant_and_balance_internal_forces() {
        for (p, edge) in examples() {
            let c = contact(p, edge);
            let a = evaluate::<true>(&c, &p, &p, 0.000318, 30.).unwrap();
            let axis = Vec3::new(1., 2., 3.).normalize();
            let angle: Real = 0.7;
            let rot = |v: Vec3| {
                v * angle.cos()
                    + axis.cross(v) * angle.sin()
                    + axis * (axis.dot(v) * (1. - angle.cos()))
            };
            let q = p.map(|v| rot(v) + Vec3::new(0.2, -0.1, 0.4));
            let b = evaluate::<true>(&c, &q, &p, 0.000318, 30.).unwrap();
            assert!((a.e - b.e).abs() < 1e-12);
            let mut force = Vec3::ZERO;
            let mut torque = Vec3::ZERO;
            for i in 0..4 {
                let g = Vec3::new(a.g[3 * i], a.g[3 * i + 1], a.g[3 * i + 2]);
                let h = Vec3::new(b.g[3 * i], b.g[3 * i + 1], b.g[3 * i + 2]);
                assert!(rot(g).distance(h) < 1e-8);
                force += g;
                torque += p[i].cross(g);
            }
            assert!(force.length() < 1e-10 && torque.length() < 1e-10);
        }
    }
    #[test]
    fn parallel_limit_and_activation_cutoff_do_not_drop_feasibility() {
        let p = [
            Vec3::ZERO,
            Vec3::X,
            Vec3::new(0., 0.0005, 0.),
            Vec3::new(1., 0.0005, 0.),
        ];
        let c = contact(p, true);
        let zero = evaluate::<true>(&c, &p, &p, 0.000318, 30.).unwrap();
        assert_eq!(zero.e, 0.);
        assert_eq!(zero.g, Gradient::zeros());
        for z in [-1e-9, 1e-9] {
            let mut q = p;
            q[3].z = z;
            let a = evaluate::<true>(&c, &q, &p, 0.000318, 30.).unwrap();
            assert!(a.e.abs() < 1e-15);
            assert!(a.g.norm() < 1e-8);
        }
        let mut penetrated = p;
        penetrated[2].y = 0.0001;
        penetrated[3].y = 0.0001;
        assert!(matches!(
            evaluate::<true>(&c, &penetrated, &p, 0.000318, 30.),
            Err(ClothError::UnresolvedSurfaceContact)
        ));
        let mut distant = p;
        distant[2].y = 0.001;
        distant[3].y = 0.001;
        let a = evaluate::<true>(&c, &distant, &p, 0.000318, 30.).unwrap();
        assert_eq!(a.e, 0.);
        assert_eq!(a.g, Gradient::zeros());
        // Exercise a nonzero-weight primitive at the activation boundary.
        let vf = [Vec3::new(0.2, 0.000636, 0.2), Vec3::ZERO, Vec3::X, Vec3::Z];
        let vc = contact(vf, false);
        for dy in [-1e-8, 0., 1e-8] {
            let mut q = vf;
            q[0].y += dy;
            let a = evaluate::<true>(&vc, &q, &vf, 0.000318, 30.).unwrap();
            assert!(a.e.abs() < 1e-16 && a.g.norm() < 1e-8);
            if dy >= 0. {
                assert_eq!(a.e, 0.);
                assert_eq!(a.g, Gradient::zeros());
            }
        }
        // The polynomial joins its constant branch with zero first derivative.
        let slope: Real = 0.001_f64.sqrt();
        let at = |dz: Real| {
            [
                Vec3::ZERO,
                Vec3::X,
                Vec3::new(0., 0.0005, -(slope + dz) * 0.5),
                Vec3::new(1., 0.0005, (slope + dz) * 0.5),
            ]
        };
        let lo = at(-1e-9);
        let hi = at(1e-9);
        let ec = contact(at(0.), true);
        let a = evaluate::<true>(&ec, &lo, &p, 0.000318, 30.).unwrap();
        let b = evaluate::<true>(&ec, &hi, &p, 0.000318, 30.).unwrap();
        assert!((a.e - b.e).abs() < 1e-12 && (a.g - b.g).norm() < 1e-7);
    }
    #[test]
    fn first_order_omits_curvature_and_retains_energy_gradient_and_errors() {
        assert_eq!(
            std::mem::size_of::<Jet<false>>(),
            13 * std::mem::size_of::<Real>()
        );
        assert_eq!(
            std::mem::size_of::<Jet<true>>(),
            157 * std::mem::size_of::<Real>()
        );
        for (p, edge) in examples() {
            let original = contact(p, edge);
            for separation in [original.separation, 1.0, 0.000001] {
                let c = SurfaceContact {
                    separation,
                    ..original
                };
                let first = evaluate::<false>(&c, &p, &p, 0.000318, 30.);
                let second = evaluate::<true>(&c, &p, &p, 0.000318, 30.);
                match (first, second) {
                    (Ok(a), Ok(b)) => {
                        assert_eq!(a.e.to_bits(), b.e.to_bits());
                        for i in 0..12 {
                            assert_eq!(a.g[i].to_bits(), b.g[i].to_bits());
                        }
                    }
                    (Err(a), Err(b)) => assert_eq!(a, b),
                    _ => panic!("first-order evaluation changed the contact result"),
                }
            }
        }
    }
}
