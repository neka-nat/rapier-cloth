use super::{
    broad_phase::{Bounds, Hierarchy},
    ccd::{CcdFeature, conservative_advance},
    geometry::{
        SurfaceWitness as Witness, closest_segments, closest_triangle, triangles_intersect,
    },
    settings::*,
};
#[cfg(test)]
use crate::SurfaceFeature;
use crate::{ClothError, ClothMesh, Real, SurfaceContact, SurfaceContactKey, Vec3};
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
                    .map(|t| Bounds::points(t.map(|i| positions[i as usize])))
                    .collect::<Vec<_>>(),
            ),
            edges: Hierarchy::new(
                &mesh
                    .edges()
                    .iter()
                    .map(|e| Bounds::points(e.vertices.map(|i| positions[i as usize])))
                    .collect::<Vec<_>>(),
            ),
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
    feature_contact(
        a,
        b,
        |i| previous[i as usize],
        |i| positions[i as usize],
        settings,
        true,
    )
}
fn feature_contact(
    a: Witness,
    b: Witness,
    previous: impl Fn(u32) -> Vec3,
    position: impl Fn(u32) -> Vec3,
    settings: ClothContactSettings,
    activation_only: bool,
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
    let delta = entries[..count]
        .iter()
        .map(|&(i, w)| position(i) * w)
        .sum::<Vec3>();
    let distance = delta.length();
    if activation_only && distance > settings.thickness + settings.activation_margin {
        return Ok(None);
    }
    let old_delta = entries[..count]
        .iter()
        .map(|&(i, w)| previous(i) * w)
        .sum::<Vec3>();
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

#[allow(clippy::too_many_arguments)]
fn swept_contact(
    feature: CcdFeature,
    ids: [u32; 4],
    face: u32,
    start: &[Vec3],
    end: &[Vec3],
    fraction: Real,
    settings: ClothContactSettings,
) -> Result<SurfaceContact, ClothError> {
    let at = |i: u32| start[i as usize] + (end[i as usize] - start[i as usize]) * fraction;
    let p = ids.map(at);
    let (a, b) = match feature {
        CcdFeature::VertexFace => {
            let witness = closest_triangle(p[0], [p[1], p[2], p[3]])
                .ok_or(ClothError::DegenerateConstraint)?;
            (
                Witness::vertex(ids[0]),
                Witness::triangle([ids[1], ids[2], ids[3]], face, witness.barycentric),
            )
        }
        CcdFeature::EdgeEdge => {
            let witness = closest_segments([p[0], p[1]], [p[2], p[3]])
                .ok_or(ClothError::DegenerateConstraint)?;
            (
                Witness::edge([ids[0], ids[1]], witness.parameters[0]),
                Witness::edge([ids[2], ids[3]], witness.parameters[1]),
            )
        }
    };
    feature_contact(a, b, |i| start[i as usize], at, settings, false)?
        .ok_or(ClothError::InvalidSurfaceContact("incident swept feature"))
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
    triangle_bounds: Vec<Bounds>,
    edge_bounds: Vec<Bounds>,
    triangle_nodes: Vec<Bounds>,
    edge_nodes: Vec<Bounds>,
    stack: Vec<usize>,
    candidates: Vec<usize>,
    generated: Vec<SurfaceContact>,
    motion_contacts: Vec<SurfaceContact>,
    initial_checked: bool,
    checked_initial_positions: Vec<Vec3>,
    cached_positions: Vec<Vec3>,
    cached_previous: Vec<Vec3>,
    contact_cache_valid: bool,
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
            motion_contacts: vec![],
            initial_checked: false,
            checked_initial_positions: vec![],
            cached_positions: vec![],
            cached_previous: vec![],
            contact_cache_valid: false,
        }
    }
    pub fn begin(&mut self, mesh: Arc<ClothMesh>, settings: ClothContactSettings) {
        if !Arc::ptr_eq(&mesh, &self.mesh) || settings != self.settings {
            self.contact_cache_valid = false;
            self.checked_initial_positions.clear();
        }
        if !Arc::ptr_eq(&mesh, &self.mesh) {
            self.topology = mesh.collision_topology().clone();
            self.mesh = mesh;
        }
        self.settings = settings;
        self.work = CollisionWork::default();
        self.initial_checked = false;
        self.motion_contacts.clear();
    }
    fn refit(&mut self, positions: &[Vec3]) -> Result<(), ClothError> {
        self.triangle_bounds.clear();
        self.edge_bounds.clear();
        self.triangle_bounds.extend(
            self.mesh
                .triangles()
                .iter()
                .map(|t| Bounds::points(t.map(|i| positions[i as usize]))),
        );
        self.edge_bounds.extend(
            self.mesh
                .edges()
                .iter()
                .map(|e| Bounds::points(e.vertices.map(|i| positions[i as usize]))),
        );
        self.topology
            .triangles
            .refit(&self.triangle_bounds, &mut self.triangle_nodes)?;
        self.topology
            .edges
            .refit(&self.edge_bounds, &mut self.edge_nodes)?;
        Ok(())
    }

    /// All vertices follow the same linear fraction during this motion batch.
    /// Swept boxes enclose both endpoints of every primitive; no static query
    /// from before the projection is reused as a swept candidate envelope.
    pub fn motion_fraction(&mut self, start: &[Vec3], end: &[Vec3]) -> Result<Real, ClothError> {
        self.motion_contacts.clear();
        if start.len() != self.mesh.rest_positions().len()
            || end.len() != start.len()
            || start.iter().chain(end).any(|p| !p.is_finite())
        {
            return Err(ClothError::NonFiniteState);
        }
        // Common translation preserves relative geometry. Compare local
        // endpoint positions, avoiding cancellation of large displacements.
        if start
            .iter()
            .zip(end)
            .all(|(a, b)| *a - start[0] == *b - end[0])
        {
            return Ok(1.0);
        }
        let separation = self.settings.thickness * 0.9;
        self.triangle_bounds.clear();
        self.edge_bounds.clear();
        self.triangle_bounds.extend(
            self.mesh.triangles().iter().map(|t| {
                Bounds::points(t.iter().flat_map(|&i| [start[i as usize], end[i as usize]]))
            }),
        );
        self.edge_bounds.extend(self.mesh.edges().iter().map(|e| {
            Bounds::points(
                e.vertices
                    .iter()
                    .flat_map(|&i| [start[i as usize], end[i as usize]]),
            )
        }));
        self.topology
            .triangles
            .refit(&self.triangle_bounds, &mut self.triangle_nodes)?;
        self.topology
            .edges
            .refit(&self.edge_bounds, &mut self.edge_nodes)?;
        let mut fraction: Real = 1.0;
        for vertex in 0..start.len() {
            self.topology.triangles.query(
                Bounds::points([start[vertex], end[vertex]]).expanded(separation),
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
                let ids = [
                    vertex,
                    triangle[0] as usize,
                    triangle[1] as usize,
                    triangle[2] as usize,
                ];
                let next = conservative_advance(
                    CcdFeature::VertexFace,
                    ids.map(|i| start[i]),
                    ids.map(|i| end[i]),
                    separation,
                    &mut self.work,
                    self.settings.limits,
                )?;
                fraction = fraction.min(next.fraction());
                if next.fraction() < 1.0 {
                    let c = swept_contact(
                        CcdFeature::VertexFace,
                        ids.map(|i| i as u32),
                        face as u32,
                        start,
                        end,
                        next.fraction(),
                        self.settings,
                    )?;
                    retain_contact(
                        &mut self.motion_contacts,
                        c,
                        &mut self.work,
                        self.settings.limits,
                    )?;
                }
            }
        }
        for a in 0..self.mesh.edges().len() {
            self.topology.edges.query(
                self.edge_bounds[a].expanded(separation),
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
                let ids = [ea[0], ea[1], eb[0], eb[1]].map(|i| i as usize);
                let next = conservative_advance(
                    CcdFeature::EdgeEdge,
                    ids.map(|i| start[i]),
                    ids.map(|i| end[i]),
                    separation,
                    &mut self.work,
                    self.settings.limits,
                )?;
                fraction = fraction.min(next.fraction());
                if next.fraction() < 1.0 {
                    let c = swept_contact(
                        CcdFeature::EdgeEdge,
                        ids.map(|i| i as u32),
                        0,
                        start,
                        end,
                        next.fraction(),
                        self.settings,
                    )?;
                    retain_contact(
                        &mut self.motion_contacts,
                        c,
                        &mut self.work,
                        self.settings.limits,
                    )?;
                }
            }
        }
        if fraction < 1.0 {
            self.work.limited_advances += 1;
        }
        self.motion_contacts.sort_by_key(|c| c.key);
        self.motion_contacts.dedup_by_key(|c| c.key);
        self.work.charge(
            CollisionBudgetKind::RetainedContacts,
            self.motion_contacts.len(),
            self.settings.limits,
        )?;
        Ok(fraction)
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
        self.checked_initial_positions.clear();
        self.checked_initial_positions.extend_from_slice(positions);
        Ok(())
    }
    pub fn generate(
        &mut self,
        previous: &[Vec3],
        positions: &[Vec3],
        out: &mut Vec<SurfaceContact>,
        include_motion_contacts: bool,
    ) -> Result<(), ClothError> {
        let mut refitted = false;
        if !self.initial_checked {
            if positions == self.checked_initial_positions {
                self.initial_checked = true;
            } else {
                self.refit(positions)?;
                refitted = true;
                self.validate_initial(positions)?;
            }
        }
        // Exact pose reuse only: there is no movement tolerance or stale BVH
        // envelope. An empty discrete result does not depend on old orientation;
        // nonempty contacts also require the same previous witness positions.
        // Motion seeds are query-specific and never become a cached result.
        if !include_motion_contacts
            && self.contact_cache_valid
            && positions == self.cached_positions
            && (self.generated.is_empty() || previous == self.cached_previous)
        {
            self.work.charge(
                CollisionBudgetKind::RetainedContacts,
                self.generated.len(),
                self.settings.limits,
            )?;
            out.extend_from_slice(&self.generated);
            return Ok(());
        }
        // Invalidate before mutating a result, including on any failing query.
        self.contact_cache_valid = false;
        if !refitted {
            self.refit(positions)?;
        }
        self.generated.clear();
        if include_motion_contacts {
            self.generated.extend_from_slice(&self.motion_contacts);
        }
        let activation = self.settings.thickness + self.settings.activation_margin;
        for (vertex, &p) in positions.iter().enumerate() {
            self.topology.triangles.query(
                Bounds::point(p).expanded(activation),
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
        if !include_motion_contacts {
            self.cached_positions.clear();
            self.cached_positions.extend_from_slice(positions);
            self.cached_previous.clear();
            self.cached_previous.extend_from_slice(previous);
            self.contact_cache_valid = true;
        }
        out.extend_from_slice(&self.generated);
        Ok(())
    }
    pub fn scratch_bytes(&self) -> usize {
        (self.triangle_bounds.capacity()
            + self.edge_bounds.capacity()
            + self.triangle_nodes.capacity()
            + self.edge_nodes.capacity())
            * std::mem::size_of::<Bounds>()
            + (self.stack.capacity() + self.candidates.capacity()) * std::mem::size_of::<usize>()
            + (self.generated.capacity() + self.motion_contacts.capacity())
                * std::mem::size_of::<SurfaceContact>()
            + (self.checked_initial_positions.capacity()
                + self.cached_positions.capacity()
                + self.cached_previous.capacity())
                * std::mem::size_of::<Vec3>()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::GridBuilder;

    fn compare_contacts(a: &[SurfaceContact], b: &[SurfaceContact]) {
        assert_eq!(a.len(), b.len());
        for (a, b) in a.iter().zip(b) {
            assert_eq!(
                (
                    a.key,
                    a.particles,
                    a.weights,
                    a.normal,
                    a.offset,
                    a.surface_velocity,
                    a.separation,
                    a.static_friction,
                    a.kinetic_friction
                ),
                (
                    b.key,
                    b.particles,
                    b.weights,
                    b.normal,
                    b.offset,
                    b.surface_velocity,
                    b.separation,
                    b.static_friction,
                    b.kinetic_friction
                )
            );
        }
    }

    #[test]
    fn exact_contact_reuse_matches_fresh_geometry_and_invalidates_changed_inputs() {
        let grid = GridBuilder::new(2, 2).size(0.1, 0.1).build().unwrap();
        let mut positions = grid.rest_positions().to_vec();
        positions.extend(grid.rest_positions().iter().map(|p| *p + Vec3::Y * 0.001));
        let mut triangles = grid.triangles().to_vec();
        triangles.extend(grid.triangles().iter().map(|t| t.map(|i| i + 4)));
        let mesh = Arc::new(ClothMesh::new(positions.clone(), triangles).unwrap());
        let config = ClothContactSettings::default();
        let mut engine = SelfCollision::new(mesh.clone(), config);
        let mut original = vec![];
        engine
            .generate(&positions, &positions, &mut original, false)
            .unwrap();
        assert!(!original.is_empty());
        let work = engine.work.candidate_pairs;
        let mut cached = vec![];
        engine
            .generate(&positions, &positions, &mut cached, false)
            .unwrap();
        compare_contacts(&original, &cached);
        assert_eq!(engine.work.candidate_pairs, work);
        engine.begin(mesh.clone(), config);
        cached.clear();
        engine
            .generate(&positions, &positions, &mut cached, false)
            .unwrap();
        assert_eq!(engine.work.candidate_pairs, 0);
        assert_eq!(engine.work.retained_contacts, original.len());
        compare_contacts(&original, &cached);
        for change in 0..4 {
            let mut previous = positions.clone();
            let mut current = positions.clone();
            let mut settings = config;
            let mut query_mesh = mesh.clone();
            match change {
                0 => previous[4..].iter_mut().for_each(|p| p.y -= 0.01),
                1 => current[4..].iter_mut().for_each(|p| p.y += 0.00001),
                2 => {
                    settings.kinetic_friction = 0.25;
                    settings.static_friction = 0.5;
                }
                _ => {
                    // Identical vertex coordinates do not imply identical
                    // topology or feature identities on another cloth.
                    let mut triangles = mesh.triangles().to_vec();
                    triangles.reverse();
                    query_mesh = Arc::new(ClothMesh::new(current.clone(), triangles).unwrap());
                }
            }
            engine.begin(query_mesh.clone(), settings);
            cached.clear();
            engine
                .generate(&previous, &current, &mut cached, false)
                .unwrap();
            let mut fresh = vec![];
            SelfCollision::new(query_mesh, settings)
                .generate(&previous, &current, &mut fresh, false)
                .unwrap();
            compare_contacts(&cached, &fresh);
            assert!(engine.work.candidate_pairs > 0);
        }
    }

    #[test]
    fn failed_refresh_and_motion_queries_cannot_reuse_a_partial_contact_result() {
        let mesh = Arc::new(GridBuilder::new(3, 3).build().unwrap());
        let positions = mesh.rest_positions();
        let settings = ClothContactSettings::default();
        let mut engine = SelfCollision::new(mesh.clone(), settings);
        engine
            .generate(positions, positions, &mut vec![], false)
            .unwrap();
        let mut changed = positions.to_vec();
        changed[0].y += 0.01;
        engine.work.candidate_pairs = settings.limits.candidate_pairs;
        assert!(matches!(
            engine.generate(positions, &changed, &mut vec![], false),
            Err(ClothError::CollisionBudgetExceeded { .. })
        ));
        assert!(!engine.contact_cache_valid);
        engine.begin(mesh.clone(), settings);
        let mut actual = vec![];
        engine
            .generate(positions, positions, &mut actual, false)
            .unwrap();
        assert!(engine.work.candidate_pairs > 0);
        let mut expected = vec![];
        SelfCollision::new(mesh.clone(), settings)
            .generate(positions, positions, &mut expected, false)
            .unwrap();
        compare_contacts(&actual, &expected);

        // Even at exactly the same query pose, a prediction with motion seeds
        // must perform its own query and must not populate the discrete cache.
        engine.begin(mesh.clone(), settings);
        engine.motion_fraction(positions, &changed).unwrap();
        let before = engine.work.candidate_pairs;
        actual.clear();
        engine
            .generate(positions, positions, &mut actual, true)
            .unwrap();
        assert!(engine.work.candidate_pairs > before);
        assert!(!engine.contact_cache_valid);
        compare_contacts(&actual, &expected);
    }

    #[test]
    fn an_uncertified_trial_cannot_bypass_next_substeps_initial_intersection_check() {
        let intersecting = vec![
            Vec3::new(-1.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, -1.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, -1.0, -0.3),
            Vec3::new(0.0, 1.0, -0.3),
            Vec3::new(0.0, 1.0, 0.3),
        ];
        let mut initial = intersecting.clone();
        initial[3..].iter_mut().for_each(|p| p.x += 3.0);
        let mesh = Arc::new(ClothMesh::new(initial.clone(), vec![[0, 1, 2], [3, 4, 5]]).unwrap());
        let config = ClothContactSettings::default();
        let mut engine = SelfCollision::new(mesh.clone(), config);
        engine
            .generate(&initial, &initial, &mut vec![], false)
            .unwrap();
        let mut trial = vec![];
        engine
            .generate(&initial, &intersecting, &mut trial, false)
            .unwrap();
        // A discrete VF/EE query can miss edges piercing a triangle interior;
        // neither that empty result nor a failed caller certifies an initial pose.
        assert!(trial.is_empty());
        engine.begin(mesh, config);
        assert!(matches!(
            engine.generate(&intersecting, &intersecting, &mut vec![], false),
            Err(ClothError::InitialSelfIntersection { .. })
        ));
    }

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
            .generate(&positions, &positions, &mut contacts, false)
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
