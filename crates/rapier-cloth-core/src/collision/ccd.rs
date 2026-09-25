//! Conservative advancement for linearly moving surface features.
//!
//! These geometric queries bound a proposed displacement; they do not advance
//! simulation time or change a cloth. Solver integration is a separate concern.
//! The bound follows conservative advancement for convex feature distances,
//! as used by C-IPC: <https://ipc-sim.github.io/C-IPC/>. Each accepted interval
//! retains a positive gap above `minimum_separation`, with a rounding allowance.

use super::{
    geometry::{closest_segments, closest_triangle},
    settings::{CollisionBudgetKind, CollisionLimits, CollisionWork},
};
use crate::{ClothError, Real, Vec3};

/// Vertex-face uses vertex 0 and triangle `[1,2,3]`. Edge-edge uses `[0,1]` and `[2,3]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CcdFeature {
    VertexFace,
    EdgeEdge,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CcdResult {
    /// The entire linear displacement maintains the requested separation.
    Clear,
    /// Only the prefix [0, fraction] has been accepted. This is a conservative
    /// displacement bound, not an exact time of impact or an elapsed time step.
    Limited { fraction: Real },
}
impl CcdResult {
    pub fn fraction(self) -> Real {
        match self {
            Self::Clear => 1.0,
            Self::Limited { fraction } => fraction,
        }
    }
}

impl CcdFeature {
    fn split(self) -> usize {
        match self {
            Self::VertexFace => 1,
            Self::EdgeEdge => 2,
        }
    }
    fn witnesses(self, positions: [Vec3; 4]) -> Option<[Vec3; 2]> {
        match self {
            Self::VertexFace => Some([
                positions[0],
                closest_triangle(positions[0], [positions[1], positions[2], positions[3]])?.point,
            ]),
            Self::EdgeEdge => {
                let w =
                    closest_segments([positions[0], positions[1]], [positions[2], positions[3]])?;
                Some([w.a, w.b])
            }
        }
    }
    fn projected_separation(self, positions: &[Vec3; 4], normal: Vec3) -> Real {
        let mut minimum = Real::INFINITY;
        for i in 0..self.split() {
            for j in self.split()..4 {
                // Bound dot-product rounding componentwise, so large motion
                // along a perpendicular axis does not consume normal clearance.
                let error = (positions[i].abs() + positions[j].abs()).dot(normal.abs())
                    * Real::EPSILON
                    * 64.0;
                minimum = minimum.min(normal.dot(positions[i] - positions[j]) - error);
            }
        }
        minimum
    }
}

/// Try to certify an entire linear motion after an early conservative prefix.
///
/// `Limited` is a safe prefix, not proof of a collision in the remaining motion.
/// In a rotating approach the distance can fall below the early-stop threshold
/// while the whole motion still retains clearance. Probe up to three additional
/// suffixes along the same motion, charging the original cumulative work budget.
/// If these cannot certify the whole segment, retain the original safe prefix.
/// This does not advance physical time or accept an unresolved suffix.
pub(crate) fn certify_linear_motion(
    feature: CcdFeature,
    start: [Vec3; 4],
    end: [Vec3; 4],
    minimum_separation: Real,
    work: &mut CollisionWork,
    limits: CollisionLimits,
) -> Result<CcdResult, ClothError> {
    let mut fraction = 0.0;
    let mut first = None;
    for _ in 0..4 {
        let from = std::array::from_fn(|i| start[i] + (end[i] - start[i]) * fraction);
        let result =
            match conservative_advance(feature, from, end, minimum_separation, work, limits) {
                Ok(result) => result,
                // An unresolved later probe does not invalidate the already
                // certified original prefix. Budget/invalid-geometry errors still
                // propagate, and an unresolved first query still fails.
                Err(ClothError::UnresolvedContinuousCollision(_)) if first.is_some() => break,
                Err(error) => return Err(error),
            };
        match result {
            CcdResult::Clear => return Ok(CcdResult::Clear),
            CcdResult::Limited { fraction: local } => {
                let next = fraction + (1.0 - fraction) * local;
                first.get_or_insert(next);
                if next <= fraction || next >= 1.0 {
                    break;
                }
                fraction = next;
            }
        }
    }
    Ok(CcdResult::Limited {
        fraction: first.unwrap_or(fraction),
    })
}

/// Bound linear interpolation from `start` to `end` without crossing the
/// minimum separation. Uses no allocations and never changes the input poses.
///
/// The distance Lipschitz bound is the largest relative displacement between
/// vertices on opposite features. A fixed separating direction at both ends
/// certifies an interval directly, including fast tangential sliding. Otherwise
/// advancement reserves 10% of the current positive clearance. It terminates
/// once the clearance reaches 10% of its initial value, or proves the whole
/// remaining interval clear. At most `MAX_EVALUATIONS` distance evaluations are
/// permitted for one query, also charged to the caller's cumulative `CcdChecks`
/// work budget; when they are exhausted, the prefix certified so far is returned
/// as `Limited`, which stays conservative (a feature pivoting about a point
/// near contact keeps its clearance while a far vertex moves a lot, so the
/// Lipschitz bound admits only short advances).
///
/// Degenerate distance queries, insufficient numerical clearance and
/// non-progress return typed failures. The rounding
/// distance allowance is 64 * Real::EPSILON times the interpolated local feature
/// extent (including cancellation); projection bounds use componentwise dot
/// product allowances. Retain a resolvable initial margin above the minimum.
/// Distance evaluations permitted for one conservative-advancement query.
const MAX_EVALUATIONS: usize = 2048;

pub fn conservative_advance(
    feature: CcdFeature,
    start: [Vec3; 4],
    end: [Vec3; 4],
    minimum_separation: Real,
    work: &mut CollisionWork,
    limits: CollisionLimits,
) -> Result<CcdResult, ClothError> {
    if start.iter().chain(&end).any(|p| !p.is_finite())
        || !minimum_separation.is_finite()
        || minimum_separation < 0.0
    {
        return Err(ClothError::InvalidParameter("continuous collision query"));
    }
    // A linearly translating local origin removes common rigid translation
    // without subtracting large, nearly equal world-space velocities.
    let a = start.map(|p| p - start[0]);
    let b = end.map(|p| p - end[0]);
    let motion = std::array::from_fn::<_, 4, _>(|i| b[i] - a[i]);
    let scale = a
        .iter()
        .chain(&b)
        .map(|p| p.length())
        .fold(minimum_separation, Real::max);
    let mut speed: Real = 0.0;
    for i in 0..feature.split() {
        for j in feature.split()..4 {
            speed = speed.max((motion[i] - motion[j]).length());
        }
    }
    speed *= 1.0 + Real::EPSILON * 64.0;
    if !speed.is_finite() || !scale.is_finite() {
        return Err(ClothError::UnresolvedContinuousCollision(
            "non-finite motion bound",
        ));
    }
    let mut fraction = 0.0;
    let mut target_clearance = 0.0;
    for evaluation in 0..MAX_EVALUATIONS {
        work.charge(CollisionBudgetKind::CcdChecks, 1, limits)?;
        let positions = std::array::from_fn(|i| a[i] + motion[i] * fraction);
        let witnesses = feature
            .witnesses(positions)
            .ok_or(ClothError::DegenerateConstraint)?;
        let delta = witnesses[0] - witnesses[1];
        let distance = delta.length();
        if !distance.is_finite() || distance <= 0.0 {
            return Err(ClothError::UnresolvedContinuousCollision(
                "insufficient separation or numerical clearance",
            ));
        }
        let normal = delta / distance;
        // Every point on a convex feature is a convex combination of its
        // vertices. Projection at both endpoints therefore bounds all feature
        // points over the entire remaining linear interval, including between
        // endpoints where an unsigned distance-only test would miss a crossing.
        let projected_minimum = minimum_separation * normal.length() * (1.0 + Real::EPSILON * 64.0);
        if feature.projected_separation(&positions, normal) > projected_minimum
            && feature.projected_separation(&b, normal) > projected_minimum
        {
            return Ok(CcdResult::Clear);
        }
        let rounding = a
            .iter()
            .zip(motion)
            .map(|(p, m)| (p.abs() + m.abs() * fraction).length())
            .fold(minimum_separation, Real::max)
            * Real::EPSILON
            * 64.0;
        let clearance = distance - minimum_separation - rounding;
        if !clearance.is_finite() || clearance <= 0.0 {
            return Err(ClothError::UnresolvedContinuousCollision(
                "insufficient separation or numerical clearance",
            ));
        }
        if evaluation == 0 {
            target_clearance = clearance * 0.1;
        }
        if speed == 0.0 {
            return Ok(CcdResult::Clear);
        }
        if fraction > 0.0 && clearance <= target_clearance {
            return Ok(CcdResult::Limited { fraction });
        }
        let remaining = 1.0 - fraction;
        if clearance > speed * remaining {
            return Ok(CcdResult::Clear);
        }
        let increment = 0.9 * clearance / speed;
        let next = fraction + increment;
        if !next.is_finite() || next <= fraction {
            return Err(ClothError::UnresolvedContinuousCollision(
                "advancement made no progress",
            ));
        }
        // Rounding upward to 1 cannot bypass the strict interval test above.
        fraction = next.min(1.0);
    }
    // Every advance kept the separation, so the prefix is a safe answer.
    Ok(CcdResult::Limited { fraction })
}

#[cfg(test)]
mod prefix_continuation_tests {
    use super::*;
    #[test]
    // Preserve the recorded f64 inputs; the f32 suite exercises their rounding.
    #[allow(clippy::excessive_precision)]
    fn saved_safe_approach_continues_past_first_conservative_prefix() {
        let start = [
            Vec3::new(
                -0.07684178301072304,
                0.02795432166043326,
                0.07290825811395008,
            ),
            Vec3::new(
                -0.055379011240968744,
                0.01345112529717496,
                0.07174130637554785,
            ),
            Vec3::new(-0.06901839538316046, 0.01600539918729742, 0.07282066952667),
            Vec3::new(
                -0.04801602865774236,
                0.003353019333387006,
                0.06646094998916459,
            ),
        ];
        let end = [
            Vec3::new(
                -0.07883581572015677,
                0.016647290626520112,
                0.06751404564198973,
            ),
            Vec3::new(
                -0.05744326632245079,
                0.0010016794642521724,
                0.06833899116623009,
            ),
            Vec3::new(
                -0.07036419493556886,
                0.004474044679588639,
                0.06935474904763744,
            ),
            Vec3::new(
                -0.04801602860334894,
                0.00015899999999999977,
                0.06646094909815944,
            ),
        ];

        let mut first_work = CollisionWork::default();
        let limits = CollisionLimits::default();
        let first = conservative_advance(
            CcdFeature::EdgeEdge,
            start,
            end,
            0.0002862,
            &mut first_work,
            limits,
        )
        .unwrap();
        assert!(matches!(first, CcdResult::Limited { .. }));
        let mut work = CollisionWork::default();
        assert_eq!(
            certify_linear_motion(
                CcdFeature::EdgeEdge,
                start,
                end,
                0.0002862,
                &mut work,
                limits
            )
            .unwrap(),
            CcdResult::Clear
        );
        // An independent interval-distance audit bounds the full segment above
        // 0.000304569 m; this fixture must not be rejected as an actual collision.
        let limited = CollisionLimits {
            ccd_checks: first_work.ccd_checks,
            ..limits
        };
        assert!(matches!(
            certify_linear_motion(
                CcdFeature::EdgeEdge,
                start,
                end,
                0.0002862,
                &mut CollisionWork::default(),
                limited
            ),
            Err(ClothError::CollisionBudgetExceeded {
                kind: CollisionBudgetKind::CcdChecks,
                ..
            })
        ));
    }
    /// Smallest witness distance sampled along the prefix `[0, fraction]`.
    #[cfg(feature = "f64")]
    fn sampled_minimum(
        feature: CcdFeature,
        start: [Vec3; 4],
        end: [Vec3; 4],
        fraction: Real,
    ) -> Real {
        (0..=400)
            .map(|k| {
                let t = fraction * k as Real / 400.0;
                let p: [Vec3; 4] = std::array::from_fn(|i| start[i] + (end[i] - start[i]) * t);
                let w = feature.witnesses(p).unwrap();
                (w[0] - w[1]).length()
            })
            .fold(Real::INFINITY, Real::min)
    }
    #[test]
    #[cfg(feature = "f64")]
    // Recorded at the towel release: one edge pivots about a vertex resting
    // 0.06 mm above the swept minimum while its other vertex falls 15 mm, so
    // the Lipschitz bound admits about 450 short advances. This exceeded the
    // former 256-evaluation limit and failed the whole step.
    fn pivoting_edge_near_contact_certifies_a_safe_prefix() {
        let start = [
            Vec3::new(
                0.13716286669625807,
                0.005251971094043289,
                -0.18653653977257792,
            ),
            Vec3::new(
                0.1532433122561256,
                0.0009546711255515203,
                -0.17095172096297542,
            ),
            Vec3::new(
                0.15336480237546643,
                0.000621823286506107,
                -0.15663752126516475,
            ),
            Vec3::new(
                0.15337966329171174,
                0.000638884387832846,
                -0.1727682013288491,
            ),
        ];
        let end = [
            Vec3::new(
                0.13709548932958118,
                -0.009646800459254316,
                -0.19035679901421476,
            ),
            Vec3::new(
                0.15322070580072014,
                0.0005079551347088778,
                -0.1709197677384579,
            ),
            Vec3::new(
                0.15335747136388173,
                0.0007497249288094587,
                -0.15661504549399607,
            ),
            Vec3::new(
                0.15336034066405152,
                0.00012094446448972587,
                -0.1727356092538063,
            ),
        ];
        let separation = 0.00028619999999999996;
        let mut work = CollisionWork::default();
        let result = conservative_advance(
            CcdFeature::EdgeEdge,
            start,
            end,
            separation,
            &mut work,
            CollisionLimits::default(),
        )
        .unwrap();
        let fraction = result.fraction();
        assert!(fraction > 0.8, "{result:?}");
        assert!(work.ccd_checks > 256 && work.ccd_checks <= MAX_EVALUATIONS);
        assert!(sampled_minimum(CcdFeature::EdgeEdge, start, end, fraction) >= separation);
        assert!(
            certify_linear_motion(
                CcdFeature::EdgeEdge,
                start,
                end,
                separation,
                &mut CollisionWork::default(),
                CollisionLimits::default(),
            )
            .unwrap()
            .fraction()
                >= fraction
        );
    }
    #[test]
    #[cfg(feature = "f64")]
    fn exhausted_advancement_returns_the_certified_prefix() {
        // A ten-metre edge pivots about a vertex 0.1 mm above the minimum while
        // its far vertex sweeps half a metre: every advance is tiny and the
        // clearance never drops to the early-stop threshold.
        let separation = 0.01;
        let start = [
            Vec3::new(10.0, separation + 1e-4, 0.0),
            Vec3::new(0.0, separation + 1e-4, 0.0),
            Vec3::new(0.0, 0.0, -0.001),
            Vec3::new(0.0, 0.0, 0.001),
        ];
        let end = [Vec3::new(10.0, -0.5, 0.0), start[1], start[2], start[3]];
        let mut work = CollisionWork::default();
        let result = conservative_advance(
            CcdFeature::EdgeEdge,
            start,
            end,
            separation,
            &mut work,
            CollisionLimits::default(),
        )
        .unwrap();
        let CcdResult::Limited { fraction } = result else {
            panic!("{result:?}");
        };
        assert_eq!(work.ccd_checks, MAX_EVALUATIONS, "fraction {fraction}");
        assert!(fraction > 0.0 && fraction < 0.5, "{fraction}");
        assert!(sampled_minimum(CcdFeature::EdgeEdge, start, end, fraction) >= separation);
        let continued = certify_linear_motion(
            CcdFeature::EdgeEdge,
            start,
            end,
            separation,
            &mut CollisionWork::default(),
            CollisionLimits::default(),
        )
        .unwrap();
        // The suffix probes certify the rest of this collision-free motion.
        assert!(
            continued.fraction() >= fraction,
            "{continued:?} after {fraction}"
        );
    }
    #[test]
    fn true_crossing_retains_the_original_safe_prefix() {
        let start = [
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, -1.0, 1.0),
            Vec3::new(0.0, 1.0, 1.0),
        ];
        let end = [
            start[0],
            start[1],
            Vec3::new(0.0, -1.0, -1.0),
            Vec3::new(0.0, 1.0, -1.0),
        ];
        let limits = CollisionLimits::default();
        let first = conservative_advance(
            CcdFeature::EdgeEdge,
            start,
            end,
            0.01,
            &mut CollisionWork::default(),
            limits,
        )
        .unwrap();
        let continued = certify_linear_motion(
            CcdFeature::EdgeEdge,
            start,
            end,
            0.01,
            &mut CollisionWork::default(),
            limits,
        )
        .unwrap();
        assert!(matches!(first, CcdResult::Limited { .. }));
        assert_eq!(continued, first);
    }
}
