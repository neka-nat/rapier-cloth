//! Redundant distance bounds for exactly inextensible, hard-attached meshes.
//!
//! The triangle inequality bounds distance to an anchor by any rest-edge path
//! length when every edge preserves its length. A shortest edge path is a cheap
//! upper bound, including on a folded rest mesh; it is not a Euclidean rest-pose
//! distance or an extra calibrated spring. See the long-range attachment idea:
//! <https://matthias-research.github.io/pages/publications/sca2012cloth.pdf>.

use crate::{ClothError, ClothMesh, Real, Target, Vec3};
use std::{cmp::Ordering, collections::BinaryHeap, sync::Arc};

#[derive(Debug, Clone, Copy)]
struct Visit {
    vertex: u32,
    anchor: u32,
    distance: Real,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::GridBuilder;

    #[test]
    fn folded_rest_paths_allow_unfolding_and_disconnected_components_remain_free() {
        let flat = GridBuilder::new(5, 2).size(0.4, 0.05).build().unwrap();
        let mut folded = flat.rest_positions().to_vec();
        for p in &mut folded {
            p.y = (p.x - 0.2).max(0.0);
            p.x = p.x.min(0.2);
        }
        let mut rest = folded.clone();
        rest.extend(folded.iter().map(|p| *p + Vec3::X * 2.0));
        let mut triangles = flat.triangles().to_vec();
        triangles.extend(flat.triangles().iter().map(|t| t.map(|i| i + 10)));
        let mesh = Arc::new(ClothMesh::new(rest, triangles).unwrap());
        let mut tethers = Tethers::default();
        let target = Target {
            particle: 0,
            position: Vec3::ZERO,
            compliance: 0.0,
        };
        tethers.prepare(&mesh, &[target], true).unwrap();
        assert!(tethers.distances[4] >= 0.4);
        assert!(folded[4].length() < 0.3);
        assert_eq!(tethers.nearest[10], u32::MAX);
        let mut positions = flat.rest_positions().to_vec();
        positions.extend(flat.rest_positions().iter().map(|p| *p + Vec3::X * 10.0));
        let expected = positions.clone();
        let mut weights = vec![1.0; 20];
        weights[0] = 0.0;
        tethers.project(&mut positions, &weights).unwrap();
        assert_eq!(positions, expected);
    }

    #[test]
    fn changed_anchors_meshes_compliance_and_release_invalidate_the_derived_map() {
        let mesh = Arc::new(GridBuilder::new(3, 2).build().unwrap());
        let target = |particle, compliance| Target {
            particle,
            position: Vec3::ZERO,
            compliance,
        };
        let mut tethers = Tethers::default();
        tethers.prepare(&mesh, &[target(0, 0.0)], true).unwrap();
        assert!(tethers.nearest.iter().all(|&i| i == 0));
        tethers.prepare(&mesh, &[target(5, 0.0)], true).unwrap();
        assert!(tethers.nearest.iter().all(|&i| i == 5));
        // Same vertex count, different rest path lengths: no stale geometry.
        let other = Arc::new(GridBuilder::new(3, 2).size(10.0, 10.0).build().unwrap());
        tethers.prepare(&other, &[target(0, 0.0)], true).unwrap();
        assert!(tethers.distances[2] >= 10.0);
        for (enabled, targets) in [
            (false, vec![target(0, 0.0)]),
            (true, vec![target(0, 0.001)]),
            (true, vec![]),
        ] {
            tethers.prepare(&mesh, &[target(0, 0.0)], true).unwrap();
            tethers.prepare(&mesh, &targets, enabled).unwrap();
            let mut moved = mesh.rest_positions().to_vec();
            moved[2].x += 10.0;
            let expected = moved.clone();
            tethers
                .project(&mut moved, &[0.0, 1.0, 1.0, 1.0, 1.0, 1.0])
                .unwrap();
            assert_eq!(moved, expected);
        }
    }
}
impl PartialEq for Visit {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}
impl Eq for Visit {}
impl PartialOrd for Visit {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Visit {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .distance
            .total_cmp(&self.distance)
            .then_with(|| other.anchor.cmp(&self.anchor))
            .then_with(|| other.vertex.cmp(&self.vertex))
    }
}

#[derive(Debug, Default)]
pub(crate) struct Tethers {
    mesh: Option<Arc<ClothMesh>>,
    anchors: Vec<u32>,
    cluster_offsets: Vec<usize>,
    cluster_anchors: Vec<u32>,
    offsets: Vec<usize>,
    neighbours: Vec<(u32, Real)>,
    nearest: Vec<u32>,
    distances: Vec<Real>,
    queue: BinaryHeap<Visit>,
    ready: bool,
}

impl Tethers {
    pub fn prepare(
        &mut self,
        mesh: &Arc<ClothMesh>,
        targets: &[Target],
        enabled: bool,
    ) -> Result<(), ClothError> {
        if !enabled || !targets.iter().any(|t| t.compliance == 0.0) {
            self.anchors.clear();
            self.ready = false;
            return Ok(());
        }
        let same_mesh = self.mesh.as_ref().is_some_and(|old| Arc::ptr_eq(old, mesh));
        let hard = || {
            targets
                .iter()
                .filter(|t| t.compliance == 0.0)
                .map(|t| t.particle)
        };
        if self.ready && same_mesh && hard().eq(self.anchors.iter().copied()) {
            return Ok(());
        }
        self.ready = false;
        self.anchors.clear();
        self.anchors.extend(hard());
        let n = mesh.rest_positions().len();
        if !same_mesh {
            self.offsets.clear();
            self.offsets.resize(n + 1, 0);
            for edge in mesh.edges() {
                for vertex in edge.vertices {
                    self.offsets[vertex as usize + 1] += 1;
                }
            }
            for i in 1..=n {
                self.offsets[i] += self.offsets[i - 1];
            }
            self.neighbours.resize(self.offsets[n], (0, 0.0));
            let mut cursor = self.offsets[..n].to_vec();
            for edge in mesh.edges() {
                let [a, b] = edge.vertices;
                self.neighbours[cursor[a as usize]] = (b, edge.rest_length);
                self.neighbours[cursor[b as usize]] = (a, edge.rest_length);
                cursor[a as usize] += 1;
                cursor[b as usize] += 1;
            }
            self.mesh = Some(mesh.clone());
        }
        self.nearest.clear();
        self.nearest.resize(n, u32::MAX);
        self.cluster_offsets.clear();
        self.cluster_offsets.push(0);
        self.cluster_anchors.clear();
        self.queue.clear();
        for &anchor in &self.anchors {
            self.nearest[anchor as usize] = 0;
        }
        // Adjacent hard vertices form an attachment region. Keep one nearest
        // anchor from each region; a single nearest anchor overall introduces
        // opposite corrections and artificial strain where two grasps meet.
        for &anchor in &self.anchors {
            if self.nearest[anchor as usize] != 0 {
                continue;
            }
            self.nearest[anchor as usize] = 1;
            self.queue.push(Visit {
                vertex: anchor,
                anchor,
                distance: 0.0,
            });
            while let Some(visit) = self.queue.pop() {
                self.cluster_anchors.push(visit.vertex);
                let i = visit.vertex as usize;
                for &(vertex, _) in &self.neighbours[self.offsets[i]..self.offsets[i + 1]] {
                    if self.nearest[vertex as usize] == 0 {
                        self.nearest[vertex as usize] = 1;
                        self.queue.push(Visit {
                            vertex,
                            anchor,
                            distance: 0.0,
                        });
                    }
                }
            }
            self.cluster_offsets.push(self.cluster_anchors.len());
        }
        let entries = n
            .checked_mul(self.cluster_offsets.len() - 1)
            .ok_or(ClothError::InvalidParameter("attachment bound count"))?;
        self.nearest.clear();
        self.nearest
            .try_reserve(entries)
            .map_err(|_| ClothError::InvalidParameter("attachment bound allocation"))?;
        self.nearest.resize(entries, u32::MAX);
        self.distances.clear();
        self.distances
            .try_reserve(entries)
            .map_err(|_| ClothError::InvalidParameter("attachment bound allocation"))?;
        self.distances.resize(entries, Real::INFINITY);
        for cluster in 0..self.cluster_offsets.len() - 1 {
            let base = cluster * n;
            for &anchor in &self.cluster_anchors
                [self.cluster_offsets[cluster]..self.cluster_offsets[cluster + 1]]
            {
                self.nearest[base + anchor as usize] = anchor;
                self.distances[base + anchor as usize] = 0.0;
                self.queue.push(Visit {
                    vertex: anchor,
                    anchor,
                    distance: 0.0,
                });
            }
            while let Some(visit) = self.queue.pop() {
                let i = visit.vertex as usize;
                if visit.distance != self.distances[base + i]
                    || visit.anchor != self.nearest[base + i]
                {
                    continue;
                }
                for &(vertex, length) in &self.neighbours[self.offsets[i]..self.offsets[i + 1]] {
                    // Round the path length upwards: its role is an upper bound.
                    let sum = visit.distance + length;
                    let distance = sum + sum.abs() * (Real::EPSILON * 2.0);
                    if !distance.is_finite() {
                        return Err(ClothError::InvalidParameter("attachment path length"));
                    }
                    let j = base + vertex as usize;
                    if distance < self.distances[j]
                        || (distance == self.distances[j] && visit.anchor < self.nearest[j])
                    {
                        self.distances[j] = distance;
                        self.nearest[j] = visit.anchor;
                        self.queue.push(Visit {
                            vertex,
                            anchor: visit.anchor,
                            distance,
                        });
                    }
                }
            }
        }
        self.ready = true;
        Ok(())
    }

    /// One-sided feasibility projection inside the solver's unaccepted trial.
    /// This has no compliant spring parameter and makes no force-report claim.
    pub fn project(&self, positions: &mut [Vec3], weights: &[Real]) -> Result<(), ClothError> {
        if !self.ready {
            return Ok(());
        }
        for i in 0..positions.len() {
            if weights[i] == 0.0 {
                continue;
            }
            let mut correction = Vec3::ZERO;
            let mut count = 0;
            for cluster in 0..self.cluster_offsets.len() - 1 {
                let j = cluster * positions.len() + i;
                if self.nearest[j] == u32::MAX {
                    continue;
                }
                count += 1;
                let offset = positions[i] - positions[self.nearest[j] as usize];
                let length = offset.length();
                if !length.is_finite() {
                    return Err(ClothError::NonFiniteState);
                }
                if length > self.distances[j] {
                    correction -= offset * (1.0 - self.distances[j] / length);
                }
            }
            // Jacobi averaging avoids choosing one gripper's projection over
            // the other's. This remains a bounded convergence aid, not a claim
            // that every individual distance bound is exact after one pass.
            if count > 0 {
                positions[i] += correction / count as Real;
            }
        }
        Ok(())
    }

    pub fn scratch_bytes(&self) -> usize {
        (self.anchors.capacity() + self.nearest.capacity() + self.cluster_anchors.capacity())
            * std::mem::size_of::<u32>()
            + (self.offsets.capacity() + self.cluster_offsets.capacity())
                * std::mem::size_of::<usize>()
            + self.neighbours.capacity() * std::mem::size_of::<(u32, Real)>()
            + self.distances.capacity() * std::mem::size_of::<Real>()
            + self.queue.capacity() * std::mem::size_of::<Visit>()
    }
}
