use super::settings::{CollisionBudgetKind, CollisionLimits, CollisionWork};
use crate::{ClothError, Real, Vec3};
use std::ops::Range;

#[derive(Debug, Clone, Copy)]
/// Axis-aligned bounds plus six diagonal projection intervals (an 18-DOP).
/// All intervals enclose the primitive's convex hull, including swept vertices.
pub(crate) struct Bounds {
    pub lo: Vec3,
    pub hi: Vec3,
    plus_lo: Vec3,
    plus_hi: Vec3,
    minus_lo: Vec3,
    minus_hi: Vec3,
}
impl Bounds {
    pub fn point(p: Vec3) -> Self {
        // In addition to XYZ, bound x+y/y+z/z+x and x-y/y-z/z-x.
        // These fixed directions reject diagonal features whose axis-aligned
        // boxes overlap despite a large true separation. Round outwards before
        // any unions, including when large coordinates nearly cancel.
        let cyclic = Vec3::new(p.y, p.z, p.x);
        let error = (p.abs() + cyclic.abs()) * (Real::EPSILON * 4.0);
        Self {
            lo: p,
            hi: p,
            plus_lo: p + cyclic - error,
            plus_hi: p + cyclic + error,
            minus_lo: p - cyclic - error,
            minus_hi: p - cyclic + error,
        }
    }
    pub fn points(points: impl IntoIterator<Item = Vec3>) -> Self {
        Self::union_all(points.into_iter().map(Self::point))
    }
    fn union_all(bounds: impl IntoIterator<Item = Self>) -> Self {
        bounds.into_iter().fold(
            Self {
                lo: Vec3::splat(Real::INFINITY),
                hi: Vec3::splat(Real::NEG_INFINITY),
                plus_lo: Vec3::splat(Real::INFINITY),
                plus_hi: Vec3::splat(Real::NEG_INFINITY),
                minus_lo: Vec3::splat(Real::INFINITY),
                minus_hi: Vec3::splat(Real::NEG_INFINITY),
            },
            Self::union,
        )
    }
    pub fn union(self, b: Self) -> Self {
        Self {
            lo: self.lo.min(b.lo),
            hi: self.hi.max(b.hi),
            plus_lo: self.plus_lo.min(b.plus_lo),
            plus_hi: self.plus_hi.max(b.plus_hi),
            minus_lo: self.minus_lo.min(b.minus_lo),
            minus_hi: self.minus_hi.max(b.minus_hi),
        }
    }
    pub fn expanded(self, margin: Real) -> Self {
        // Diagonal direction vectors have length sqrt(2). Expanding every
        // interval by that support radius encloses the Euclidean margin.
        let diagonal_margin =
            Vec3::splat(margin * 2.0_f64.sqrt() as Real * (1.0 + Real::EPSILON * 4.0));
        Self {
            lo: self.lo - Vec3::splat(margin),
            hi: self.hi + Vec3::splat(margin),
            plus_lo: self.plus_lo - diagonal_margin,
            plus_hi: self.plus_hi + diagonal_margin,
            minus_lo: self.minus_lo - diagonal_margin,
            minus_hi: self.minus_hi + diagonal_margin,
        }
    }
    pub fn overlaps(self, b: Self) -> bool {
        self.lo.cmple(b.hi).all()
            && b.lo.cmple(self.hi).all()
            && self.plus_lo.cmple(b.plus_hi).all()
            && b.plus_lo.cmple(self.plus_hi).all()
            && self.minus_lo.cmple(b.minus_hi).all()
            && b.minus_lo.cmple(self.minus_hi).all()
    }
    pub fn valid(self) -> bool {
        [
            (self.lo, self.hi),
            (self.plus_lo, self.plus_hi),
            (self.minus_lo, self.minus_hi),
        ]
        .into_iter()
        .all(|(lo, hi)| lo.is_finite() && hi.is_finite() && lo.cmple(hi).all())
    }
}
#[derive(Debug, Clone)]
struct Node {
    children: Option<[usize; 2]>,
    range: Range<usize>,
}
/// Immutable partition built from rest geometry. Bounds are separate scratch.
#[derive(Debug, Clone)]
pub(crate) struct Hierarchy {
    nodes: Vec<Node>,
    order: Vec<usize>,
}
impl Hierarchy {
    pub fn new(bounds: &[Bounds]) -> Self {
        let mut tree = Self {
            nodes: vec![],
            order: (0..bounds.len()).collect(),
        };
        if !bounds.is_empty() {
            tree.build(bounds, 0..bounds.len());
        }
        tree
    }
    fn build(&mut self, bounds: &[Bounds], range: Range<usize>) -> usize {
        let node = self.nodes.len();
        self.nodes.push(Node {
            children: None,
            range: range.clone(),
        });
        if range.len() > 4 {
            let bound = Bounds::union_all(self.order[range.clone()].iter().map(|&i| bounds[i]));
            let extent = bound.hi - bound.lo;
            let axis = if extent.x >= extent.y && extent.x >= extent.z {
                0
            } else if extent.y >= extent.z {
                1
            } else {
                2
            };
            self.order[range.clone()].sort_unstable_by(|&a, &b| {
                let ca = bounds[a].lo[axis] * 0.5 + bounds[a].hi[axis] * 0.5;
                let cb = bounds[b].lo[axis] * 0.5 + bounds[b].hi[axis] * 0.5;
                ca.total_cmp(&cb).then(a.cmp(&b))
            });
            let middle = range.start + range.len() / 2;
            let a = self.build(bounds, range.start..middle);
            let b = self.build(bounds, middle..range.end);
            self.nodes[node].children = Some([a, b]);
        }
        node
    }
    pub fn refit(&self, primitives: &[Bounds], bounds: &mut Vec<Bounds>) -> Result<(), ClothError> {
        if primitives.len() != self.order.len() || primitives.iter().any(|b| !b.valid()) {
            return Err(ClothError::InvalidSurfaceContact(
                "invalid hierarchy bounds",
            ));
        }
        bounds.resize(self.nodes.len(), Bounds::point(Vec3::ZERO));
        for i in (0..self.nodes.len()).rev() {
            bounds[i] = if let Some([a, b]) = self.nodes[i].children {
                bounds[a].union(bounds[b])
            } else {
                Bounds::union_all(
                    self.order[self.nodes[i].range.clone()]
                        .iter()
                        .map(|&j| primitives[j]),
                )
            };
        }
        Ok(())
    }
    #[allow(clippy::too_many_arguments)]
    pub fn query(
        &self,
        query: Bounds,
        minimum: usize,
        primitives: &[Bounds],
        bounds: &[Bounds],
        stack: &mut Vec<usize>,
        out: &mut Vec<usize>,
        work: &mut CollisionWork,
        limits: CollisionLimits,
    ) -> Result<(), ClothError> {
        out.clear();
        stack.clear();
        if self.nodes.is_empty() {
            return Ok(());
        }
        stack.push(0);
        while let Some(i) = stack.pop() {
            if !query.overlaps(bounds[i]) {
                continue;
            }
            if let Some([a, b]) = self.nodes[i].children {
                stack.push(b);
                stack.push(a);
            } else {
                for &j in &self.order[self.nodes[i].range.clone()] {
                    if j >= minimum && query.overlaps(primitives[j]) {
                        work.charge(CollisionBudgetKind::CandidatePairs, 1, limits)?;
                        out.push(j);
                    }
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diagonal_intervals_reject_separated_edges_with_overlapping_axis_boxes() {
        let a = Bounds::points([-Vec3::X - Vec3::Z, Vec3::X + Vec3::Z]);
        let delta = Vec3::new(0.1, 0.0, -0.1);
        let b = Bounds::points([-Vec3::X - Vec3::Z + delta, Vec3::X + Vec3::Z + delta]);
        assert!(a.lo.cmple(b.hi).all() && b.lo.cmple(a.hi).all());
        // Parallel lines are sqrt(0.02) apart; a 5 cm activation radius
        // cannot make them collide despite overlapping XYZ boxes.
        assert!(!a.expanded(0.05).overlaps(b));
        assert!(!b.expanded(0.05).overlaps(a));
        assert!(a.expanded(0.15).overlaps(b));
        assert!(b.expanded(0.15).overlaps(a));
    }

    #[test]
    fn unions_enclose_swept_convex_combinations_and_the_euclidean_margin() {
        let mut seed = 829_u64;
        let mut random = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            (seed >> 32) as Real / u32::MAX as Real - 0.5
        };
        for origin in [Vec3::ZERO, Vec3::new(100.0, -100.0, 100.0)] {
            for _ in 0..100 {
                let start: [Vec3; 3] =
                    std::array::from_fn(|_| origin + Vec3::new(random(), random(), random()));
                let end: [Vec3; 3] =
                    std::array::from_fn(|_| origin + Vec3::new(random(), random(), random()));
                let bounds =
                    Bounds::union_all([Bounds::points(start), Bounds::points(end)]).expanded(0.01);
                // An affine triangle trajectory and every barycentric witness
                // lie in the convex hull of the six endpoint vertices. Test
                // interior points, feature boundaries and almost-full margins.
                for sample in 0..40 {
                    let t = sample as Real / 39.0;
                    let u = random() + 0.5;
                    let v = (random() + 0.5) * (1.0 - u);
                    let barycentric = match sample % 4 {
                        0 => [1.0, 0.0, 0.0],
                        1 => [u, 1.0 - u, 0.0],
                        _ => [u, v, 1.0 - u - v],
                    };
                    let witness: Vec3 = (0..3)
                        .map(|i| (start[i] + (end[i] - start[i]) * t) * barycentric[i])
                        .sum();
                    let direction = Vec3::new(random(), random(), random()).normalize();
                    let point = witness + direction * 0.0099;
                    assert!(
                        bounds.overlaps(Bounds::point(point)),
                        "{start:?}, {end:?}, {point:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn hierarchy_refits_match_exhaustive_queries() {
        let mut seed = 123_u64;
        let mut random = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            ((seed >> 32) as Real / u32::MAX as Real) - 0.5
        };
        let mut primitives: Vec<_> = (0..127)
            .map(|_| Bounds::point(Vec3::new(random(), random(), random())).expanded(0.04))
            .collect();
        let tree = Hierarchy::new(&primitives);
        let mut bounds = vec![];
        let mut stack = vec![];
        let mut found = vec![];
        for _ in 0..5 {
            tree.refit(&primitives, &mut bounds).unwrap();
            for _ in 0..100 {
                let q = Bounds::point(Vec3::new(random(), random(), random())).expanded(0.1);
                tree.query(
                    q,
                    0,
                    &primitives,
                    &bounds,
                    &mut stack,
                    &mut found,
                    &mut CollisionWork::default(),
                    CollisionLimits::default(),
                )
                .unwrap();
                found.sort_unstable();
                let expected: Vec<_> = primitives
                    .iter()
                    .enumerate()
                    .filter(|(_, p)| q.overlaps(**p))
                    .map(|(i, _)| i)
                    .collect();
                assert_eq!(found, expected);
            }
            for b in &mut primitives {
                let center = (b.lo + b.hi) * 0.5;
                *b = Bounds::point(center + Vec3::new(random(), random(), random()) * 0.1)
                    .expanded(0.04);
            }
        }
        let limits = CollisionLimits {
            candidate_pairs: 1,
            ..Default::default()
        };
        assert!(
            tree.query(
                Bounds::point(Vec3::ZERO).expanded(10.0),
                0,
                &primitives,
                &bounds,
                &mut stack,
                &mut found,
                &mut CollisionWork::default(),
                limits
            )
            .is_err()
        );
    }
}
