use crate::collision::self_collision::CollisionTopology;
use crate::{ClothError, Real, Vec3, constraints::bend::angle_and_gradients};
use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Arc, OnceLock};

#[derive(Debug, Clone)]
pub struct Edge {
    pub vertices: [u32; 2],
    pub rest_length: Real,
    /// Multiplier on this edge's XPBD stretch compliance (1 by default); set
    /// with [`ClothMesh::scale_edge_compliance`]. Ignored with zero
    /// compliance, where every edge is inextensible.
    pub compliance_scale: Real,
}

#[derive(Debug, Clone)]
pub struct Hinge {
    pub vertices: [u32; 4],
    pub rest_angle: Real,
    /// Multiplier on the bending stiffness of this hinge (1 by default), for
    /// example a stitched seam; set with [`ClothMesh::scale_hinge_stiffness`].
    pub stiffness_scale: Real,
}

/// A thread between two vertices of one cloth, for example along a seam
/// between separately meshed panels. The solvers keep the vertices
/// `rest_length` apart: XPBD as a distance constraint with
/// `ClothMaterial::stitch_compliance`, the implicit solver as a spring of
/// `ShellMaterial::stitch_stiffness`. Stitched vertices stay distinct for
/// collision, so a rest length of at least the contact thickness keeps the two
/// panels' seam edges apart the way a seam allowance does.
#[derive(Debug, Clone)]
pub struct Stitch {
    pub vertices: [u32; 2],
    pub rest_length: Real,
}

/// Validated, immutable simulation topology. No implicit vertex welding.
#[derive(Debug, Clone)]
pub struct ClothMesh {
    positions: Vec<Vec3>,
    triangles: Vec<[u32; 3]>,
    edges: Vec<Edge>,
    hinges: Vec<Hinge>,
    vertex_areas: Vec<Real>,
    vertex_faces: Vec<u32>,
    edge_opposites: Vec<u32>,
    area: Real,
    material_axes: Option<Vec<Vec3>>,
    triangle_stiffness_scales: Option<Vec<Real>>,
    stitches: Vec<Stitch>,
    collision_topology: OnceLock<Arc<CollisionTopology>>,
}

impl ClothMesh {
    pub fn new(positions: Vec<Vec3>, triangles: Vec<[u32; 3]>) -> Result<Self, ClothError> {
        let invalid = |s: &str| ClothError::InvalidMesh(s.to_owned());
        if positions.len() < 3 || positions.len() > u32::MAX as usize || triangles.is_empty() {
            return Err(invalid(
                "at least three vertices and one triangle are required",
            ));
        }
        if triangles.len() > u32::MAX as usize {
            return Err(invalid(
                "triangle count exceeds material feature index range",
            ));
        }
        if positions.iter().any(|p| !p.is_finite()) {
            return Err(invalid("non-finite vertex"));
        }
        let mut face_keys = BTreeSet::new();
        let mut adjacency: BTreeMap<[u32; 2], Vec<[u32; 3]>> = BTreeMap::new();
        let mut vertex_areas = vec![0.0; positions.len()];
        let mut vertex_faces = vec![u32::MAX; positions.len()];
        let mut area = 0.0;
        for (face, &tri) in triangles.iter().enumerate() {
            if tri.iter().any(|&i| i as usize >= positions.len()) {
                return Err(invalid("triangle index out of bounds"));
            }
            let mut key = tri;
            key.sort_unstable();
            if key[0] == key[1] || key[1] == key[2] {
                return Err(invalid("repeated triangle vertex"));
            }
            if !face_keys.insert(key) {
                return Err(invalid("duplicate triangle"));
            }
            let a = positions[tri[0] as usize];
            let b = positions[tri[1] as usize];
            let c = positions[tri[2] as usize];
            let face_area = (b - a).cross(c - a).length() * 0.5;
            if !face_area.is_finite() || face_area <= Real::MIN_POSITIVE {
                return Err(invalid("zero or non-finite triangle area"));
            }
            area += face_area;
            for i in tri {
                vertex_areas[i as usize] += face_area / 3.0;
                if vertex_faces[i as usize] == u32::MAX {
                    vertex_faces[i as usize] = face as u32;
                }
            }
            for [i, j, k] in [
                [tri[0], tri[1], tri[2]],
                [tri[1], tri[2], tri[0]],
                [tri[2], tri[0], tri[1]],
            ] {
                let pair = [i.min(j), i.max(j)];
                let faces = adjacency.entry(pair).or_default();
                if faces.len() == 2 {
                    return Err(invalid("non-manifold edge"));
                }
                if faces.first().is_some_and(|f| f[0] == i) {
                    return Err(invalid("inconsistent triangle winding"));
                }
                faces.push([i, j, k]);
            }
        }
        if !area.is_finite()
            || vertex_areas
                .iter()
                .any(|a| !a.is_finite() || *a <= Real::MIN_POSITIVE)
        {
            return Err(invalid("isolated vertex or invalid accumulated area"));
        }
        let mut edges = Vec::with_capacity(adjacency.len());
        let mut edge_opposites = Vec::with_capacity(adjacency.len());
        let mut hinges = Vec::new();
        for (vertices, faces) in adjacency {
            let rest_length =
                positions[vertices[0] as usize].distance(positions[vertices[1] as usize]);
            if !rest_length.is_finite() || rest_length <= Real::MIN_POSITIVE {
                return Err(invalid("invalid edge length"));
            }
            edges.push(Edge {
                vertices,
                rest_length,
                compliance_scale: 1.0,
            });
            edge_opposites.push(faces[0][2]);
            if faces.len() == 2 {
                let ids = [faces[0][0], faces[0][1], faces[0][2], faces[1][2]];
                let (rest_angle, _) = angle_and_gradients(ids.map(|i| positions[i as usize]))
                    .ok_or_else(|| invalid("degenerate rest hinge"))?;
                hinges.push(Hinge {
                    vertices: ids,
                    rest_angle,
                    stiffness_scale: 1.0,
                });
            }
        }
        Ok(Self {
            positions,
            triangles,
            edges,
            hinges,
            vertex_areas,
            vertex_faces,
            edge_opposites,
            area,
            material_axes: None,
            triangle_stiffness_scales: None,
            stitches: Vec::new(),
            collision_topology: OnceLock::new(),
        })
    }

    /// Adds stitches; each joins two distinct vertices with a finite, positive
    /// rest length, and no pair is stitched twice.
    pub fn add_stitches(&mut self, stitches: Vec<Stitch>) -> Result<(), ClothError> {
        let count = self.positions.len() as u32;
        let mut seen: BTreeSet<[u32; 2]> = self
            .stitches
            .iter()
            .map(|s| {
                [
                    s.vertices[0].min(s.vertices[1]),
                    s.vertices[0].max(s.vertices[1]),
                ]
            })
            .collect();
        for stitch in &stitches {
            let [a, b] = stitch.vertices;
            if a >= count || b >= count || a == b {
                return Err(ClothError::InvalidParameter(
                    "stitch must join two distinct vertices of the mesh",
                ));
            }
            if !stitch.rest_length.is_finite() || stitch.rest_length <= 0.0 {
                return Err(ClothError::InvalidParameter(
                    "stitch rest length must be finite and positive",
                ));
            }
            if !seen.insert([a.min(b), a.max(b)]) {
                return Err(ClothError::InvalidParameter("duplicate stitch"));
            }
        }
        self.stitches.extend(stitches);
        Ok(())
    }
    pub fn stitches(&self) -> &[Stitch] {
        &self.stitches
    }

    /// Multiplies the XPBD stretch compliance of each edge by `scale(edge)`,
    /// which must be finite and positive: the discrete counterpart of a
    /// direction- or region-dependent material. The implicit solver's membrane
    /// uses triangles instead; see [`ClothMesh::scale_triangle_stiffness`].
    pub fn scale_edge_compliance(
        &mut self,
        scale: impl Fn(&Edge) -> Real,
    ) -> Result<(), ClothError> {
        let scales: Vec<Real> = self.edges.iter().map(&scale).collect();
        if scales.iter().any(|s| !s.is_finite() || *s <= 0.0) {
            return Err(ClothError::InvalidParameter(
                "edge compliance scale must be finite and positive",
            ));
        }
        for (edge, s) in self.edges.iter_mut().zip(scales) {
            edge.compliance_scale *= s;
        }
        Ok(())
    }

    /// Scales each edge's XPBD compliance by its direction relative to the
    /// material axes: `along` for an edge parallel to the axis of an incident
    /// triangle, `across` for one perpendicular to it, blended by the squared
    /// cosine in between (so bias edges get the mean). Requires material axes.
    pub fn scale_edge_compliance_by_material_axes(
        &mut self,
        along: Real,
        across: Real,
    ) -> Result<(), ClothError> {
        let axes = self
            .material_axes
            .clone()
            .ok_or(ClothError::InvalidParameter("mesh has no material axes"))?;
        if [along, across].iter().any(|v| !v.is_finite() || *v <= 0.0) {
            return Err(ClothError::InvalidParameter(
                "edge compliance scale must be finite and positive",
            ));
        }
        // Axis of the first face of each edge, in edge order.
        let mut edge_axis: BTreeMap<[u32; 2], Vec3> = BTreeMap::new();
        for (k, t) in self.triangles.iter().enumerate() {
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                let key = [a.min(b), a.max(b)];
                edge_axis.entry(key).or_insert(axes[k]);
            }
        }
        let scales: Vec<Real> = self
            .edges
            .iter()
            .map(|edge| {
                let d = (self.positions[edge.vertices[1] as usize]
                    - self.positions[edge.vertices[0] as usize])
                    .normalize_or_zero();
                let mut key = edge.vertices;
                key.sort_unstable();
                let c2 = edge_axis[&key].dot(d).powi(2);
                along * c2 + across * (1.0 - c2)
            })
            .collect();
        for (edge, s) in self.edges.iter_mut().zip(scales) {
            edge.compliance_scale *= s;
        }
        Ok(())
    }

    /// Multiplies the membrane stiffness of each triangle (the implicit
    /// solver's Young's modulus and directional stiffness) by
    /// `scale(index, triangle)`, which must be finite and positive.
    pub fn scale_triangle_stiffness(
        &mut self,
        scale: impl Fn(usize, &[u32; 3]) -> Real,
    ) -> Result<(), ClothError> {
        let scales: Vec<Real> = self
            .triangles
            .iter()
            .enumerate()
            .map(|(k, t)| scale(k, t))
            .collect();
        if scales.iter().any(|s| !s.is_finite() || *s <= 0.0) {
            return Err(ClothError::InvalidParameter(
                "triangle stiffness scale must be finite and positive",
            ));
        }
        let current = self
            .triangle_stiffness_scales
            .get_or_insert_with(|| vec![1.0; scales.len()]);
        for (c, s) in current.iter_mut().zip(scales) {
            *c *= s;
        }
        Ok(())
    }
    /// Per-triangle membrane stiffness multipliers, if any were set.
    pub fn triangle_stiffness_scales(&self) -> Option<&[Real]> {
        self.triangle_stiffness_scales.as_deref()
    }

    /// Sets one material (warp) direction per triangle, in rest space; each
    /// must be finite and non-zero, and is stored normalized. The implicit
    /// solver projects it into the triangle's rest plane and applies
    /// `ShellMaterial::warp_stiffness` along it and `weft_stiffness` across
    /// it. `None` removes the axes.
    pub fn set_material_axes(&mut self, axes: Option<Vec<Vec3>>) -> Result<(), ClothError> {
        let Some(axes) = axes else {
            self.material_axes = None;
            return Ok(());
        };
        if axes.len() != self.triangles.len() {
            return Err(ClothError::InvalidParameter(
                "one material axis per triangle",
            ));
        }
        let mut normalized = Vec::with_capacity(axes.len());
        for axis in axes {
            let length = axis.length();
            if !axis.is_finite() || length <= Real::MIN_POSITIVE {
                return Err(ClothError::InvalidParameter(
                    "material axes must be finite and non-zero",
                ));
            }
            normalized.push(axis / length);
        }
        self.material_axes = Some(normalized);
        Ok(())
    }
    /// The material axes set with [`ClothMesh::set_material_axes`].
    pub fn material_axes(&self) -> Option<&[Vec3]> {
        self.material_axes.as_deref()
    }

    /// Multiplies the bending stiffness of each hinge by `scale(hinge)`, which
    /// must be finite and positive. The implicit solver scales the hinge's
    /// bending rigidity; XPBD divides the hinge's bend compliance.
    pub fn scale_hinge_stiffness(
        &mut self,
        scale: impl Fn(&Hinge) -> Real,
    ) -> Result<(), ClothError> {
        let scales: Vec<Real> = self.hinges.iter().map(&scale).collect();
        if scales.iter().any(|s| !s.is_finite() || *s <= 0.0) {
            return Err(ClothError::InvalidParameter(
                "hinge stiffness scale must be finite and positive",
            ));
        }
        for (hinge, s) in self.hinges.iter_mut().zip(scales) {
            hinge.stiffness_scale *= s;
        }
        Ok(())
    }

    pub fn rest_positions(&self) -> &[Vec3] {
        &self.positions
    }
    /// A fixed material triangle supplies orientation at vertex/edge contacts.
    /// The immutable topology avoids scanning incident faces during each solve.
    pub(crate) fn material_triangle(&self, particles: &[u32]) -> Option<[u32; 3]> {
        if particles.len() >= 3 {
            return Some([particles[0], particles[1], particles[2]]);
        }
        if let [a, b] = *particles {
            let vertices = [a.min(b), a.max(b)];
            let index = self
                .edges
                .binary_search_by_key(&vertices, |e| e.vertices)
                .ok()?;
            return Some([vertices[0], vertices[1], self.edge_opposites[index]]);
        }
        let face = *self.vertex_faces.get(*particles.first()? as usize)?;
        self.triangles.get(face as usize).copied()
    }
    pub(crate) fn collision_topology(&self) -> &Arc<CollisionTopology> {
        self.collision_topology
            .get_or_init(|| Arc::new(CollisionTopology::new(self)))
    }
    pub fn triangles(&self) -> &[[u32; 3]] {
        &self.triangles
    }
    pub fn edges(&self) -> &[Edge] {
        &self.edges
    }
    pub fn hinges(&self) -> &[Hinge] {
        &self.hinges
    }
    pub fn vertex_areas(&self) -> &[Real] {
        &self.vertex_areas
    }
    pub fn area(&self) -> Real {
        self.area
    }
}

/// Rectangular grid in an explicitly chosen plane; defaults to X/Z.
#[derive(Debug, Clone)]
pub struct GridBuilder {
    nx: usize,
    ny: usize,
    width: Real,
    height: Real,
    origin: Vec3,
    u: Vec3,
    v: Vec3,
}

impl GridBuilder {
    pub fn new(nx: usize, ny: usize) -> Self {
        Self {
            nx,
            ny,
            width: 1.0,
            height: 1.0,
            origin: Vec3::ZERO,
            u: Vec3::X,
            v: Vec3::Z,
        }
    }
    pub fn size(mut self, width: Real, height: Real) -> Self {
        self.width = width;
        self.height = height;
        self
    }
    pub fn origin(mut self, origin: Vec3) -> Self {
        self.origin = origin;
        self
    }
    /// Axes must be finite orthonormal unit vectors.
    pub fn axes(mut self, u: Vec3, v: Vec3) -> Self {
        self.u = u;
        self.v = v;
        self
    }
    pub fn build(self) -> Result<ClothMesh, ClothError> {
        if self.nx < 2
            || self.ny < 2
            || self
                .nx
                .checked_mul(self.ny)
                .is_none_or(|n| n > u32::MAX as usize)
        {
            return Err(ClothError::InvalidParameter("grid dimensions"));
        }
        if !self.width.is_finite()
            || self.width <= 0.0
            || !self.height.is_finite()
            || self.height <= 0.0
        {
            return Err(ClothError::InvalidParameter("grid size"));
        }
        if !self.u.is_finite()
            || !self.v.is_finite()
            || (self.u.length_squared() - 1.0).abs() > 1.0e-5
            || (self.v.length_squared() - 1.0).abs() > 1.0e-5
            || self.u.dot(self.v).abs() > 1.0e-5
        {
            return Err(ClothError::InvalidParameter(
                "grid axes must be orthonormal",
            ));
        }
        let mut p = Vec::with_capacity(self.nx * self.ny);
        for y in 0..self.ny {
            for x in 0..self.nx {
                p.push(
                    self.origin
                        + self.u * (self.width * x as Real / (self.nx - 1) as Real)
                        + self.v * (self.height * y as Real / (self.ny - 1) as Real),
                );
            }
        }
        let mut t = Vec::with_capacity((self.nx - 1) * (self.ny - 1) * 2);
        for y in 0..self.ny - 1 {
            for x in 0..self.nx - 1 {
                let a = (y * self.nx + x) as u32;
                let b = a + 1;
                let c = a + self.nx as u32;
                let d = c + 1;
                t.push([a, c, b]);
                t.push([b, c, d]);
            }
        }
        ClothMesh::new(p, t)
    }
}
