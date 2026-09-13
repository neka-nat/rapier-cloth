use super::settings::{CollisionBudgetKind, CollisionLimits, CollisionWork};
use crate::{ClothError, Real, Vec3};
use std::ops::Range;

#[derive(Debug, Clone, Copy)]
pub(crate) struct Aabb {
    pub lo: Vec3,
    pub hi: Vec3,
}
impl Aabb {
    pub fn point(p: Vec3) -> Self {
        Self { lo: p, hi: p }
    }
    pub fn points(points: impl IntoIterator<Item = Vec3>) -> Self {
        points.into_iter().fold(
            Self {
                lo: Vec3::splat(Real::INFINITY),
                hi: Vec3::splat(Real::NEG_INFINITY),
            },
            |b, p| b.union(Self::point(p)),
        )
    }
    pub fn union(self, b: Self) -> Self {
        Self {
            lo: self.lo.min(b.lo),
            hi: self.hi.max(b.hi),
        }
    }
    pub fn expanded(self, margin: Real) -> Self {
        Self {
            lo: self.lo - Vec3::splat(margin),
            hi: self.hi + Vec3::splat(margin),
        }
    }
    pub fn overlaps(self, b: Self) -> bool {
        self.lo.cmple(b.hi).all() && b.lo.cmple(self.hi).all()
    }
    pub fn valid(self) -> bool {
        self.lo.is_finite() && self.hi.is_finite() && self.lo.cmple(self.hi).all()
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
    pub fn new(bounds: &[Aabb]) -> Self {
        let mut tree = Self {
            nodes: vec![],
            order: (0..bounds.len()).collect(),
        };
        if !bounds.is_empty() {
            tree.build(bounds, 0..bounds.len());
        }
        tree
    }
    fn build(&mut self, bounds: &[Aabb], range: Range<usize>) -> usize {
        let node = self.nodes.len();
        self.nodes.push(Node {
            children: None,
            range: range.clone(),
        });
        if range.len() > 4 {
            let bound = Aabb::points(
                self.order[range.clone()]
                    .iter()
                    .flat_map(|&i| [bounds[i].lo, bounds[i].hi]),
            );
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
    pub fn refit(&self, primitives: &[Aabb], bounds: &mut Vec<Aabb>) -> Result<(), ClothError> {
        if primitives.len() != self.order.len() || primitives.iter().any(|b| !b.valid()) {
            return Err(ClothError::InvalidSurfaceContact(
                "invalid hierarchy bounds",
            ));
        }
        bounds.resize(self.nodes.len(), Aabb::point(Vec3::ZERO));
        for i in (0..self.nodes.len()).rev() {
            bounds[i] = if let Some([a, b]) = self.nodes[i].children {
                bounds[a].union(bounds[b])
            } else {
                Aabb::points(
                    self.order[self.nodes[i].range.clone()]
                        .iter()
                        .flat_map(|&j| [primitives[j].lo, primitives[j].hi]),
                )
            };
        }
        Ok(())
    }
    #[allow(clippy::too_many_arguments)]
    pub fn query(
        &self,
        query: Aabb,
        minimum: usize,
        primitives: &[Aabb],
        bounds: &[Aabb],
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
    fn hierarchy_refits_match_exhaustive_queries() {
        let mut seed = 123_u64;
        let mut random = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            ((seed >> 32) as Real / u32::MAX as Real) - 0.5
        };
        let mut primitives: Vec<_> = (0..127)
            .map(|_| Aabb::point(Vec3::new(random(), random(), random())).expanded(0.04))
            .collect();
        let tree = Hierarchy::new(&primitives);
        let mut bounds = vec![];
        let mut stack = vec![];
        let mut found = vec![];
        for _ in 0..5 {
            tree.refit(&primitives, &mut bounds).unwrap();
            for _ in 0..100 {
                let q = Aabb::point(Vec3::new(random(), random(), random())).expanded(0.1);
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
                let delta = Vec3::new(random(), random(), random()) * 0.1;
                b.lo += delta;
                b.hi += delta;
            }
        }
        let limits = CollisionLimits {
            candidate_pairs: 1,
            ..Default::default()
        };
        assert!(
            tree.query(
                Aabb::point(Vec3::ZERO).expanded(10.0),
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
