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

/// Bound linear interpolation from `start` to `end` without crossing the
/// minimum separation. Uses no allocations and never changes the input poses.
///
/// The distance Lipschitz bound is the largest relative displacement between
/// vertices on opposite features. A fixed separating direction at both ends
/// certifies an interval directly, including fast tangential sliding. Otherwise
/// advancement reserves 10% of the current positive clearance. It terminates
/// once the clearance reaches 10% of its initial value, or proves the whole
/// remaining interval clear. At most 256 distance evaluations are permitted for
/// one query, also charged to the caller's cumulative `CcdChecks` work budget.
///
/// Degenerate distance queries, insufficient numerical clearance, non-progress
/// and the per-query convergence limit return typed failures. The rounding
/// distance allowance is 64 * Real::EPSILON times the interpolated local feature
/// extent (including cancellation); projection bounds use componentwise dot
/// product allowances. Retain a resolvable initial margin above the minimum.
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
    for evaluation in 0..256 {
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
    Err(ClothError::UnresolvedContinuousCollision(
        "per-query convergence limit",
    ))
}
