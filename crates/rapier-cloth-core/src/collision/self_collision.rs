use super::{
    broad_phase::{Aabb, Hierarchy},
    geometry::{closest_segments, closest_triangle, triangles_intersect},
    settings::*,
};
use crate::{ClothError, ClothMesh, Real, SurfaceContact, SurfaceContactKey, SurfaceFeature, Vec3};
use std::sync::Arc;

#[derive(Debug)]
pub(crate) struct CollisionTopology {
    triangles: Hierarchy,
    edges: Hierarchy,
}
impl CollisionTopology {
    pub fn new(mesh: &ClothMesh) -> Self {
        let positions = mesh.rest_positions();
        Self {
            triangles: Hierarchy::new(
                &mesh
                    .triangles()
                    .iter()
                    .map(|t| Aabb::points(t.map(|i| positions[i as usize])))
                    .collect::<Vec<_>>(),
            ),
            edges: Hierarchy::new(
                &mesh
                    .edges()
                    .iter()
                    .map(|e| Aabb::points(e.vertices.map(|i| positions[i as usize])))
                    .collect::<Vec<_>>(),
            ),
        }
    }
}
#[derive(Clone, Copy)]
struct Witness {
    feature: SurfaceFeature,
    particles: [u32; 3],
    weights: [Real; 3],
}
impl Witness {
    fn vertex(i: u32) -> Self {
        Self {
            feature: SurfaceFeature::Vertex(i),
            particles: [i, 0, 0],
            weights: [1.0, 0.0, 0.0],
        }
    }
    fn edge(edge: [u32; 2], t: Real) -> Self {
        if t <= 1.0e-6 {
            return Self::vertex(edge[0]);
        }
        if t >= 1.0 - 1.0e-6 {
            return Self::vertex(edge[1]);
        }
        let mut indices = edge;
        indices.sort_unstable();
        let weights = if indices == edge {
            [1.0 - t, t, 0.0]
        } else {
            [t, 1.0 - t, 0.0]
        };
        Self {
            feature: SurfaceFeature::Edge(indices),
            particles: [indices[0], indices[1], 0],
            weights,
        }
    }
    fn triangle(triangle: [u32; 3], face: u32, weights: [Real; 3]) -> Self {
        let mask = weights
            .iter()
            .enumerate()
            .fold(0u8, |mask, (i, &w)| mask | ((w > 1.0e-6) as u8) << i);
        match mask {
            1 => Self::vertex(triangle[0]),
            2 => Self::vertex(triangle[1]),
            4 => Self::vertex(triangle[2]),
            3 => Self::edge(
                [triangle[0], triangle[1]],
                weights[1] / (weights[0] + weights[1]),
            ),
            5 => Self::edge(
                [triangle[0], triangle[2]],
                weights[2] / (weights[0] + weights[2]),
            ),
            6 => Self::edge(
                [triangle[1], triangle[2]],
                weights[2] / (weights[1] + weights[2]),
            ),
            _ => Self {
                feature: SurfaceFeature::Face(face),
                particles: triangle,
                weights,
            },
        }
    }
}
fn contact(
    a: Witness,
    b: Witness,
    previous: &[Vec3],
    positions: &[Vec3],
    settings: ClothContactSettings,
) -> Result<Option<SurfaceContact>, ClothError> {
    let mut entries = [(0u32, 0.0); 4];
    let mut count = 0;
    for (w, sign) in [(a, 1.0), (b, -1.0)] {
        for i in 0..3 {
            if w.weights[i] != 0.0 {
                if entries[..count]
                    .iter()
                    .any(|(index, _)| *index == w.particles[i])
                {
                    return Ok(None);
                }
                entries[count] = (w.particles[i], w.weights[i] * sign);
                count += 1;
            }
        }
    }
    entries[..count].sort_unstable_by_key(|e| e.0);
    let mut features = [a.feature, b.feature];
    if features[0] > features[1] {
        features.swap(0, 1);
        for (_, w) in &mut entries {
            *w = -*w;
        }
    }
    let mut c = SurfaceContact {
        key: SurfaceContactKey {
            other_cloth: None,
            features,
        },
        particles: entries.map(|e| e.0),
        weights: entries.map(|e| e.1),
        normal: Vec3::ZERO,
        offset: Vec3::ZERO,
        surface_velocity: Vec3::ZERO,
        separation: settings.thickness,
        static_friction: settings.static_friction,
        kinetic_friction: settings.kinetic_friction,
    };
    let delta = c.relative(positions);
    let distance = delta.length();
    if distance > settings.thickness + settings.activation_margin {
        return Ok(None);
    }
    let old_delta = c.relative(previous);
    c.normal = if distance > settings.thickness * 1.0e-6 {
        let n = delta / distance;
        if old_delta.dot(n) < -settings.thickness * 1.0e-6 {
            -n
        } else {
            n
        }
    } else {
        old_delta
            .try_normalize()
            .ok_or(ClothError::InvalidSurfaceContact(
                "ambiguous self-contact orientation",
            ))?
    };
    Ok(Some(c))
}

fn retain_contact(
    out: &mut Vec<SurfaceContact>,
    c: SurfaceContact,
    work: &mut CollisionWork,
    limits: CollisionLimits,
) -> Result<(), ClothError> {
    // Canonical duplicates must not exhaust the physical contact limit. Bound
    // the raw buffer as well, so many incident primitives cannot allocate up
    // to the (much larger) candidate-work budget before the final deduplication.
    if out.len() >= limits.retained_contacts.saturating_mul(2) {
        out.sort_by_key(|c| c.key);
        out.dedup_by_key(|c| c.key);
        work.charge(CollisionBudgetKind::RetainedContacts, out.len(), limits)?;
    }
    out.push(c);
    Ok(())
}

/// Reusable bounds and pair buffers. The immutable hierarchy belongs to the
/// mesh; binding a different cloth refits geometry and never transfers history.
#[derive(Debug)]
pub(crate) struct SelfCollision {
    mesh: Arc<ClothMesh>,
    topology: Arc<CollisionTopology>,
    pub settings: ClothContactSettings,
    pub work: CollisionWork,
    triangle_bounds: Vec<Aabb>,
    edge_bounds: Vec<Aabb>,
    triangle_nodes: Vec<Aabb>,
    edge_nodes: Vec<Aabb>,
    stack: Vec<usize>,
    candidates: Vec<usize>,
    generated: Vec<SurfaceContact>,
    initial_checked: bool,
}
impl SelfCollision {
    pub fn new(mesh: Arc<ClothMesh>, settings: ClothContactSettings) -> Self {
        let topology = mesh.collision_topology().clone();
        Self {
            mesh,
            topology,
            settings,
            work: CollisionWork::default(),
            triangle_bounds: vec![],
            edge_bounds: vec![],
            triangle_nodes: vec![],
            edge_nodes: vec![],
            stack: vec![],
            candidates: vec![],
            generated: vec![],
            initial_checked: false,
        }
    }
    pub fn begin(&mut self, mesh: Arc<ClothMesh>, settings: ClothContactSettings) {
        if !Arc::ptr_eq(&mesh, &self.mesh) {
            self.topology = mesh.collision_topology().clone();
            self.mesh = mesh;
        }
        self.settings = settings;
        self.work = CollisionWork::default();
        self.initial_checked = false;
    }
    fn refit(&mut self, positions: &[Vec3]) -> Result<(), ClothError> {
        self.triangle_bounds.clear();
        self.edge_bounds.clear();
        self.triangle_bounds.extend(
            self.mesh
                .triangles()
                .iter()
                .map(|t| Aabb::points(t.map(|i| positions[i as usize]))),
        );
        self.edge_bounds.extend(
            self.mesh
                .edges()
                .iter()
                .map(|e| Aabb::points(e.vertices.map(|i| positions[i as usize]))),
        );
        self.topology
            .triangles
            .refit(&self.triangle_bounds, &mut self.triangle_nodes)?;
        self.topology
            .edges
            .refit(&self.edge_bounds, &mut self.edge_nodes)?;
        Ok(())
    }
    fn validate_initial(&mut self, positions: &[Vec3]) -> Result<(), ClothError> {
        for a in 0..self.mesh.triangles().len() {
            self.topology.triangles.query(
                self.triangle_bounds[a],
                a + 1,
                &self.triangle_bounds,
                &self.triangle_nodes,
                &mut self.stack,
                &mut self.candidates,
                &mut self.work,
                self.settings.limits,
            )?;
            let ta = self.mesh.triangles()[a];
            for &b in &self.candidates {
                let tb = self.mesh.triangles()[b];
                if ta.iter().any(|v| tb.contains(v)) {
                    continue;
                }
                if triangles_intersect(
                    ta.map(|i| positions[i as usize]),
                    tb.map(|i| positions[i as usize]),
                )
                .ok_or(ClothError::DegenerateConstraint)?
                {
                    return Err(ClothError::InitialSelfIntersection {
                        triangles: [a as u32, b as u32],
                    });
                }
            }
        }
        self.initial_checked = true;
        Ok(())
    }
    pub fn generate(
        &mut self,
        previous: &[Vec3],
        positions: &[Vec3],
        out: &mut Vec<SurfaceContact>,
    ) -> Result<(), ClothError> {
        self.refit(positions)?;
        if !self.initial_checked {
            self.validate_initial(positions)?;
        }
        self.generated.clear();
        let activation = self.settings.thickness + self.settings.activation_margin;
        for (vertex, &p) in positions.iter().enumerate() {
            self.topology.triangles.query(
                Aabb::point(p).expanded(activation),
                0,
                &self.triangle_bounds,
                &self.triangle_nodes,
                &mut self.stack,
                &mut self.candidates,
                &mut self.work,
                self.settings.limits,
            )?;
            for &face in &self.candidates {
                let triangle = self.mesh.triangles()[face];
                if triangle.contains(&(vertex as u32)) {
                    continue;
                }
                let witness = closest_triangle(p, triangle.map(|i| positions[i as usize]))
                    .ok_or(ClothError::DegenerateConstraint)?;
                if let Some(c) = contact(
                    Witness::vertex(vertex as u32),
                    Witness::triangle(triangle, face as u32, witness.barycentric),
                    previous,
                    positions,
                    self.settings,
                )? {
                    retain_contact(&mut self.generated, c, &mut self.work, self.settings.limits)?;
                }
            }
        }
        for a in 0..self.mesh.edges().len() {
            self.topology.edges.query(
                self.edge_bounds[a].expanded(activation),
                a + 1,
                &self.edge_bounds,
                &self.edge_nodes,
                &mut self.stack,
                &mut self.candidates,
                &mut self.work,
                self.settings.limits,
            )?;
            let ea = self.mesh.edges()[a].vertices;
            for &b in &self.candidates {
                let eb = self.mesh.edges()[b].vertices;
                if ea.iter().any(|v| eb.contains(v)) {
                    continue;
                }
                let witness = closest_segments(
                    ea.map(|i| positions[i as usize]),
                    eb.map(|i| positions[i as usize]),
                )
                .ok_or(ClothError::DegenerateConstraint)?;
                if let Some(c) = contact(
                    Witness::edge(ea, witness.parameters[0]),
                    Witness::edge(eb, witness.parameters[1]),
                    previous,
                    positions,
                    self.settings,
                )? {
                    retain_contact(&mut self.generated, c, &mut self.work, self.settings.limits)?;
                }
            }
        }
        // Boundary contacts reached through several incident primitives share
        // canonical features/support ordering and must contribute only once.
        self.generated.sort_by_key(|c| c.key);
        self.generated.dedup_by_key(|c| c.key);
        self.work.charge(
            CollisionBudgetKind::RetainedContacts,
            self.generated.len(),
            self.settings.limits,
        )?;
        out.extend_from_slice(&self.generated);
        Ok(())
    }
    pub fn scratch_bytes(&self) -> usize {
        (self.triangle_bounds.capacity()
            + self.edge_bounds.capacity()
            + self.triangle_nodes.capacity()
            + self.edge_nodes.capacity())
            * std::mem::size_of::<Aabb>()
            + (self.stack.capacity() + self.candidates.capacity()) * std::mem::size_of::<usize>()
            + self.generated.capacity() * std::mem::size_of::<SurfaceContact>()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::GridBuilder;

    #[test]
    fn boundary_features_deduplicate_and_topology_is_shared() {
        let grid = GridBuilder::new(2, 2).size(0.1, 0.1).build().unwrap();
        let mut positions = grid.rest_positions().to_vec();
        positions.extend(grid.rest_positions().iter().map(|p| *p + Vec3::Y * 0.001));
        let mut triangles = grid.triangles().to_vec();
        // Opposite diagonals give an interior edge-edge witness in addition
        // to the four duplicated corner witnesses.
        triangles.extend([[4, 6, 7], [4, 7, 5]]);
        let mesh = Arc::new(ClothMesh::new(positions.clone(), triangles).unwrap());
        let mut engine = SelfCollision::new(mesh.clone(), ClothContactSettings::default());
        assert!(Arc::ptr_eq(&engine.topology, mesh.collision_topology()));
        let mut contacts = vec![];
        engine
            .generate(&positions, &positions, &mut contacts)
            .unwrap();
        assert!(contacts.windows(2).all(|pair| pair[0].key < pair[1].key));
        for i in 0..4 {
            assert_eq!(
                contacts
                    .iter()
                    .filter(|c| c.key.features
                        == [SurfaceFeature::Vertex(i), SurfaceFeature::Vertex(i + 4)])
                    .count(),
                1
            );
        }
        assert!(contacts.iter().any(|c| matches!(
            c.key.features,
            [SurfaceFeature::Edge(_), SurfaceFeature::Edge(_)]
        )));
        for c in &contacts {
            c.validate(8).unwrap();
            assert!((c.gap(&positions)).abs() < 1.0e-7);
            assert!((c.weights.iter().sum::<Real>()).abs() < 1.0e-7);
        }
        let limits = CollisionLimits {
            retained_contacts: 1,
            ..Default::default()
        };
        let mut repeated = vec![];
        let mut work = CollisionWork::default();
        for _ in 0..100 {
            retain_contact(&mut repeated, contacts[0], &mut work, limits).unwrap();
            assert!(repeated.len() <= 2);
        }
        // Unique features, unlike repetitions, must fail instead of being dropped.
        assert!(retain_contact(&mut repeated, contacts[1], &mut work, limits).is_ok());
        assert!(matches!(
            retain_contact(&mut repeated, contacts[2], &mut work, limits),
            Err(ClothError::CollisionBudgetExceeded {
                kind: CollisionBudgetKind::RetainedContacts,
                limit: 1
            })
        ));
    }
}
