use crate::{ClothError, ClothMesh, Real, Vec3};
mod query;
pub use query::{SurfaceHit, SurfaceQueryLimits, SurfaceRay};

/// A material point on a fixed-topology mesh. It is local to one cloth; the
/// integration boundary additionally associates it with a generational handle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfacePoint {
    triangle: u32,
    barycentric: [Real; 3],
}
impl SurfacePoint {
    pub fn new(triangle: u32, barycentric: [Real; 3]) -> Result<Self, ClothError> {
        let sum: Real = barycentric.iter().sum();
        if barycentric
            .iter()
            .any(|b| !b.is_finite() || !(0.0..=1.0).contains(b))
            || !sum.is_finite()
            || (sum - 1.0).abs() > 64.0 * Real::EPSILON
        {
            return Err(ClothError::InvalidParameter(
                "surface barycentric coordinates",
            ));
        }
        Ok(Self {
            triangle,
            barycentric: barycentric.map(|b| b / sum),
        })
    }
    pub fn triangle(self) -> u32 {
        self.triangle
    }
    pub fn barycentric(self) -> [Real; 3] {
        self.barycentric
    }
}

impl ClothMesh {
    /// Vertices within a shortest rest-edge path from the material point.
    /// Paths start at the three supporting vertices; this is a mesh-edge metric,
    /// not an exact continuous geodesic. No world-space shortcut crosses a fold.
    /// Empty selections and exceeded limits are errors, never nearest-vertex
    /// fallbacks or silently truncated patches.
    pub fn surface_patch(
        &self,
        point: SurfacePoint,
        radius: Real,
        limits: SurfaceQueryLimits,
    ) -> Result<Vec<u32>, ClothError> {
        limits.check(self.triangles().len())?;
        if !radius.is_finite() || radius <= 0.0 || limits.patch_vertices == 0 {
            return Err(ClothError::InvalidParameter(
                "surface patch radius or limit",
            ));
        }
        let view = SurfaceView {
            positions: self.rest_positions(),
            triangles: self.triangles(),
        };
        let center = view.point_position(point)?;
        let triangle = self.triangles()[point.triangle as usize];
        let n = self.rest_positions().len();
        let mut adjacency = vec![Vec::new(); n];
        for edge in self.edges() {
            let [a, b] = edge.vertices;
            adjacency[a as usize].push((b, edge.rest_length));
            adjacency[b as usize].push((a, edge.rest_length));
        }
        let mut distance = vec![Real::INFINITY; n];
        let mut visited = vec![false; n];
        for vertex in triangle {
            distance[vertex as usize] = center.distance(self.rest_positions()[vertex as usize]);
        }
        let mut selected = Vec::new();
        loop {
            let next = (0..n)
                .filter(|&i| !visited[i] && distance[i] <= radius)
                .min_by(|&a, &b| distance[a].total_cmp(&distance[b]).then(a.cmp(&b)));
            let Some(i) = next else {
                break;
            };
            if selected.len() == limits.patch_vertices {
                return Err(ClothError::SurfaceQueryBudgetExceeded {
                    limit: limits.patch_vertices,
                });
            }
            visited[i] = true;
            selected.push(i as u32);
            for &(j, length) in &adjacency[i] {
                let next = distance[i] + length;
                if !next.is_finite() {
                    return Err(ClothError::NonFiniteState);
                }
                distance[j as usize] = distance[j as usize].min(next);
            }
        }
        if selected.is_empty() {
            return Err(ClothError::InvalidParameter(
                "empty surface patch; increase radius or use a surface-point target",
            ));
        }
        selected.sort_unstable();
        Ok(selected)
    }
}

/// One-to-one simulation/render vertex mapping in 0.1. Indices are stable for
/// the cloth lifetime. The borrow prevents stepping while these slices are live.
#[derive(Clone, Copy, Debug)]
pub struct SurfaceView<'a> {
    pub positions: &'a [Vec3],
    pub triangles: &'a [[u32; 3]],
}
impl SurfaceView<'_> {
    pub fn point_position(&self, point: SurfacePoint) -> Result<Vec3, ClothError> {
        let triangle = self.triangle(point.triangle)?;
        let position = triangle[0] * point.barycentric[0]
            + triangle[1] * point.barycentric[1]
            + triangle[2] * point.barycentric[2];
        if !position.is_finite() {
            return Err(ClothError::NonFiniteState);
        }
        Ok(position)
    }
    fn triangle(&self, face: u32) -> Result<[Vec3; 3], ClothError> {
        let ids = self
            .triangles
            .get(face as usize)
            .ok_or(ClothError::InvalidParameter("surface triangle index"))?;
        let mut points = [Vec3::ZERO; 3];
        for (p, &i) in points.iter_mut().zip(ids) {
            *p = *self
                .positions
                .get(i as usize)
                .ok_or(ClothError::InvalidParticle(i))?;
            if !p.is_finite() {
                return Err(ClothError::NonFiniteState);
            }
        }
        Ok(points)
    }
    /// Area-weighted normals. Returns the number of degenerate faces; zero
    /// normals on isolated/fully degenerate vertices are retained as diagnostics.
    pub fn write_normals(&self, out: &mut Vec<Vec3>) -> usize {
        out.resize(self.positions.len(), Vec3::ZERO);
        out.fill(Vec3::ZERO);
        let mut degenerate = 0;
        for &[a, b, c] in self.triangles {
            let n = (self.positions[b as usize] - self.positions[a as usize])
                .cross(self.positions[c as usize] - self.positions[a as usize]);
            if !n.is_finite() || n.length_squared() <= 0.0 {
                degenerate += 1;
                continue;
            }
            for i in [a, b, c] {
                out[i as usize] += n;
            }
        }
        for n in out {
            *n = n.normalize_or_zero();
        }
        degenerate
    }
}
