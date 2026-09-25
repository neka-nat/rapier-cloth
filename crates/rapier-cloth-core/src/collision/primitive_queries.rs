//! Internal implicit-only parent primitive queries. Shared projection queries
//! keep their canonical feature snapping and history orientation unchanged.
use super::*;
use crate::implicit::workers::{Workers, handoff, take};
use std::sync::Arc;

fn chunk(n: usize, lanes: usize, k: usize) -> std::ops::Range<usize> {
    k * n / lanes..(k + 1) * n / lanes
}

/// Writes the bounds of a triangle range and an edge range, given their first indices.
pub(super) type BoundsFill<'a> = dyn Fn(usize, &mut [Bounds], usize, &mut [Bounds]) + Sync + 'a;

/// Fills contiguous lane ranges of the triangle and edge bounds in parallel.
pub(super) fn fill_bounds_parallel(
    workers: &Workers,
    fill: &BoundsFill<'_>,
    triangles: &mut [Bounds],
    edges: &mut [Bounds],
) -> Result<(), ClothError> {
    let (nt, ne, lanes) = (triangles.len(), edges.len(), workers.lanes());
    let (mut tri_rest, mut edge_rest) = (triangles, edges);
    let mut parts = Vec::with_capacity(lanes);
    for k in 0..lanes {
        let (t, e) = (chunk(nt, lanes, k), chunk(ne, lanes, k));
        let (tri, rest) = tri_rest.split_at_mut(t.len());
        tri_rest = rest;
        let (ed, rest) = edge_rest.split_at_mut(e.len());
        edge_rest = rest;
        parts.push((t.start, tri, e.start, ed));
    }
    let parts = handoff(parts);
    workers.run(lanes, |k| {
        let (t, tri, e, ed) = take(&parts, k);
        fill(t, tri, e, ed);
    })?;
    Ok(())
}

/// Adds the parent contact of a vertex and a non-incident face if in range.
fn face_pair(
    settings: ClothContactSettings,
    mesh: &ClothMesh,
    x: &[Vec3],
    vertex: usize,
    face: usize,
    out: &mut Vec<SurfaceContact>,
) -> Result<(), ClothError> {
    let triangle = mesh.triangles()[face];
    let witness = closest_triangle(x[vertex], triangle.map(|i| x[i as usize]))
        .ok_or(ClothError::DegenerateConstraint)?;
    let ids = [vertex as u32, triangle[0], triangle[1], triangle[2]];
    let weights = [
        1.0,
        -witness.barycentric[0],
        -witness.barycentric[1],
        -witness.barycentric[2],
    ];
    let features = [
        SurfaceFeature::Vertex(vertex as u32),
        SurfaceFeature::Face(face as u32),
    ];
    append_implicit(settings, x, ids, weights, features, out)
}

/// Adds the parent contact of two edges without a shared vertex if in range.
fn edge_pair(
    settings: ClothContactSettings,
    mesh: &ClothMesh,
    x: &[Vec3],
    a: usize,
    b: usize,
    out: &mut Vec<SurfaceContact>,
) -> Result<(), ClothError> {
    let mut ea = mesh.edges()[a].vertices;
    let mut eb = mesh.edges()[b].vertices;
    ea.sort_unstable();
    eb.sort_unstable();
    if ea > eb {
        std::mem::swap(&mut ea, &mut eb);
    }
    let witness = closest_segments(ea.map(|i| x[i as usize]), eb.map(|i| x[i as usize]))
        .ok_or(ClothError::DegenerateConstraint)?;
    let [s, t] = witness.parameters;
    append_implicit(
        settings,
        x,
        [ea[0], ea[1], eb[0], eb[1]],
        [1.0 - s, s, t - 1.0, -t],
        [SurfaceFeature::Edge(ea), SurfaceFeature::Edge(eb)],
        out,
    )
}

fn append_implicit(
    settings: ClothContactSettings,
    x: &[Vec3],
    ids: [u32; 4],
    weights: [Real; 4],
    features: [SurfaceFeature; 2],
    out: &mut Vec<SurfaceContact>,
) -> Result<(), ClothError> {
    let delta = (0..4)
        .map(|i| x[ids[i] as usize] * weights[i])
        .sum::<Vec3>();
    let distance = delta.length();
    if distance > settings.thickness + settings.activation_margin {
        return Ok(());
    }
    if distance <= 0.0 || !distance.is_finite() {
        return Err(ClothError::UnresolvedSurfaceContact);
    }
    if out.len() >= settings.limits.retained_contacts {
        return Err(ClothError::ContactBudgetExceeded {
            limit: settings.limits.retained_contacts,
        });
    }
    out.push(SurfaceContact {
        key: SurfaceContactKey {
            other_cloth: None,
            features,
        },
        particles: ids,
        weights,
        normal: delta / distance,
        offset: Vec3::ZERO,
        surface_velocity: Vec3::ZERO,
        separation: settings.thickness,
        static_friction: settings.static_friction,
        kinetic_friction: settings.kinetic_friction,
    });
    Ok(())
}

/// Depth of the subtree frontier whose node pairs are shared out to the lanes.
const ROOT_DEPTH: usize = 4;
/// Meshes with fewer vertices are queried on the caller; a lane split would
/// cost more than it saves.
pub(super) const PARALLEL_VERTICES: usize = 1024;

/// Overlapping vertex-triangle and edge-edge subtree pairs at the frontier,
/// in a fixed order; lane `k` takes every `lanes`-th pair.
struct Roots {
    faces: Vec<(usize, usize)>,
    edges: Vec<(usize, usize)>,
}
impl Roots {
    fn new(scene: &Scene<'_>) -> Self {
        let vertices = scene.topology.vertices.frontier(ROOT_DEPTH);
        let triangles = scene.topology.triangles.frontier(ROOT_DEPTH);
        let mut faces = Vec::new();
        for &i in &vertices {
            for &j in &triangles {
                if scene.vertex_nodes[i].overlaps(scene.triangle_nodes[j]) {
                    faces.push((i, j));
                }
            }
        }
        let edges_frontier = scene.topology.edges.frontier(ROOT_DEPTH);
        let mut edges = Vec::new();
        for (k, &i) in edges_frontier.iter().enumerate() {
            for &j in &edges_frontier[k..] {
                if scene.edge_nodes_half[i].overlaps(scene.edge_nodes_half[j]) {
                    edges.push((i, j));
                }
            }
        }
        Self { faces, edges }
    }
    /// The roots lane `k` of `lanes` processes: every `lanes`-th, faces first.
    fn lane(&self, lanes: usize, k: usize) -> Roots {
        let pick = |pairs: &[(usize, usize)]| {
            pairs
                .iter()
                .skip(k)
                .step_by(lanes)
                .copied()
                .collect::<Vec<_>>()
        };
        Roots {
            faces: pick(&self.faces),
            edges: pick(&self.edges),
        }
    }
}

/// Visits every non-incident vertex-face pair with overlapping query bounds
/// below the given subtree pairs. Like `Hierarchy::query`, every overlapping
/// pair is charged to the candidate budget, incident ones included.
fn visit_faces(
    scene: &Scene<'_>,
    roots: &[(usize, usize)],
    stack: &mut Vec<(usize, usize)>,
    work: &mut CollisionWork,
    mut visit: impl FnMut(usize, usize, &mut CollisionWork) -> Result<(), ClothError>,
) -> Result<(), ClothError> {
    let limits = scene.settings.limits;
    for &(a, b) in roots {
        scene.topology.vertices.visit_pairs(
            a,
            scene.vertex_nodes,
            scene.vertex_bounds,
            &scene.topology.triangles,
            b,
            scene.triangle_nodes,
            scene.triangle_bounds,
            stack,
            |vertex, face| {
                work.charge(CollisionBudgetKind::CandidatePairs, 1, limits)?;
                if scene.mesh.triangles()[face].contains(&(vertex as u32)) {
                    return Ok(());
                }
                visit(vertex, face, work)
            },
        )?;
    }
    Ok(())
}

/// Visits every edge pair `a < b` without a shared vertex whose half-expanded
/// bounds overlap, charging every overlapping pair to the candidate budget.
fn visit_edges(
    scene: &Scene<'_>,
    roots: &[(usize, usize)],
    stack: &mut Vec<(usize, usize)>,
    work: &mut CollisionWork,
    mut visit: impl FnMut(usize, usize, &mut CollisionWork) -> Result<(), ClothError>,
) -> Result<(), ClothError> {
    let limits = scene.settings.limits;
    for &(i, j) in roots {
        scene.topology.edges.visit_self_pairs(
            i,
            j,
            scene.edge_nodes_half,
            scene.edge_half,
            stack,
            |a, b| {
                work.charge(CollisionBudgetKind::CandidatePairs, 1, limits)?;
                let ea = scene.mesh.edges()[a].vertices;
                let eb = scene.mesh.edges()[b].vertices;
                if ea.iter().any(|v| eb.contains(v)) {
                    return Ok(());
                }
                visit(a, b, work)
            },
        )?;
    }
    Ok(())
}

/// Per root: the number of contacts and of recorded pairs it produced.
type RootCounts = Vec<[usize; 2]>;

/// Parent contacts within the activation distance at `x` for the lane's roots.
fn static_pairs(
    scene: &Scene<'_>,
    x: &[Vec3],
    roots: &Roots,
    stack: &mut Vec<(usize, usize)>,
    work: &mut CollisionWork,
    out: &mut Vec<SurfaceContact>,
    counts: &mut RootCounts,
) -> Result<(), ClothError> {
    let (settings, mesh) = (scene.settings, scene.mesh);
    for root in &roots.faces {
        let before = out.len();
        visit_faces(
            scene,
            std::slice::from_ref(root),
            stack,
            work,
            |vertex, face, _| face_pair(settings, mesh, x, vertex, face, out),
        )?;
        counts.push([out.len() - before, 0]);
    }
    for root in &roots.edges {
        let before = out.len();
        visit_edges(scene, std::slice::from_ref(root), stack, work, |a, b, _| {
            edge_pair(settings, mesh, x, a, b, out)
        })?;
        counts.push([out.len() - before, 0]);
    }
    Ok(())
}

/// Certifies the linear motion of the lane's candidate pairs and records the
/// pairs within contact reach; returns the safe fraction.
#[allow(clippy::too_many_arguments)]
fn swept_pairs(
    scene: &Scene<'_>,
    start: &[Vec3],
    end: &[Vec3],
    roots: &Roots,
    stack: &mut Vec<(usize, usize)>,
    work: &mut CollisionWork,
    out: &mut Vec<SurfaceContact>,
    mut pairs: Option<&mut SweptPairs>,
    counts: &mut RootCounts,
) -> Result<Real, ClothError> {
    let settings = scene.settings;
    let separation = settings.thickness * 0.9;
    let mut fraction: Real = 1.0;
    for root in &roots.faces {
        let before = (out.len(), pairs.as_deref().map_or(0, |p| p.faces.len()));
        visit_faces(
            scene,
            std::slice::from_ref(root),
            stack,
            work,
            |vertex, face, work| {
                if let Some(pairs) = pairs.as_deref_mut() {
                    pairs.faces.push([vertex as u32, face as u32]);
                    // Certify exactly the pairs of a separation-only query.
                    let certify = Bounds::points([start[vertex], end[vertex]]).expanded(separation);
                    if !certify.overlaps(scene.triangle_bounds[face]) {
                        return Ok(());
                    }
                }
                let triangle = scene.mesh.triangles()[face];
                let ids = [
                    vertex,
                    triangle[0] as usize,
                    triangle[1] as usize,
                    triangle[2] as usize,
                ];
                let next = certify_linear_motion(
                    CcdFeature::VertexFace,
                    ids.map(|i| start[i]),
                    ids.map(|i| end[i]),
                    separation,
                    work,
                    settings.limits,
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
                        settings,
                    )?;
                    retain_contact(out, c, work, settings.limits)?;
                }
                Ok(())
            },
        )?;
        counts.push([
            out.len() - before.0,
            pairs.as_deref().map_or(0, |p| p.faces.len()) - before.1,
        ]);
    }
    for root in &roots.edges {
        let before = (out.len(), pairs.as_deref().map_or(0, |p| p.edges.len()));
        visit_edges(
            scene,
            std::slice::from_ref(root),
            stack,
            work,
            |a, b, work| {
                if let Some(pairs) = pairs.as_deref_mut() {
                    pairs.edges.push([a as u32, b as u32]);
                    let certify = scene.edge_bounds[a].expanded(separation);
                    if !certify.overlaps(scene.edge_bounds[b]) {
                        return Ok(());
                    }
                }
                let ea = scene.mesh.edges()[a].vertices;
                let eb = scene.mesh.edges()[b].vertices;
                let ids = [ea[0], ea[1], eb[0], eb[1]].map(|i| i as usize);
                let next = certify_linear_motion(
                    CcdFeature::EdgeEdge,
                    ids.map(|i| start[i]),
                    ids.map(|i| end[i]),
                    separation,
                    work,
                    settings.limits,
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
                        settings,
                    )?;
                    retain_contact(out, c, work, settings.limits)?;
                }
                Ok(())
            },
        )?;
        counts.push([
            out.len() - before.0,
            pairs.as_deref().map_or(0, |p| p.edges.len()) - before.1,
        ]);
    }
    Ok(fraction)
}

/// Output of one query lane. `counts` holds, per processed root in lane
/// order, the number of contacts and recorded pairs.
struct Lane<T> {
    result: Result<T, ClothError>,
    work: CollisionWork,
    contacts: Vec<SurfaceContact>,
    pairs: SweptPairs,
    counts: RootCounts,
}

/// Interleaves per-root slices of the lanes' vectors back into global root
/// order: root `r` was processed by lane `r % lanes` as its `r / lanes`-th.
fn interleave<T: Clone>(parts: Vec<(Vec<T>, Vec<usize>)>, roots: usize, out: &mut Vec<T>) {
    let lanes = parts.len();
    let mut offsets: Vec<usize> = vec![0; lanes];
    for r in 0..roots {
        let (k, local) = (r % lanes, r / lanes);
        let (items, counts) = &parts[k];
        let count = counts[local];
        out.extend_from_slice(&items[offsets[k]..offsets[k] + count]);
        offsets[k] += count;
    }
}

impl SelfCollision {
    /// Split later implicit-solver queries across the workers' lanes. Results
    /// are merged in a fixed order, so contacts, fractions and work counts do
    /// not depend on the lane count.
    pub(crate) fn set_workers(&mut self, workers: Arc<Workers>) {
        self.workers = Some(workers);
    }
    fn lanes(&self) -> usize {
        if self.mesh.rest_positions().len() < PARALLEL_VERTICES {
            return 1;
        }
        self.workers.as_ref().map_or(1, |w| w.lanes())
    }
    /// Runs a lane job on the workers; a single lane runs on the caller.
    fn run<T: Send>(
        &self,
        lanes: usize,
        job: impl Fn(usize) -> T + Sync,
    ) -> Result<Vec<T>, ClothError> {
        match &self.workers {
            Some(workers) => workers.run(lanes, job),
            None => Ok(vec![job(0)]),
        }
    }

    /// Query bounds of every vertex and the edge bounds expanded by half the
    /// query reach, with their refitted nodes. Vertex and triangle bounds must
    /// come from the same positions (static) or motion (swept).
    fn fit_query(
        &mut self,
        vertex: impl Fn(usize) -> Bounds,
        reach: Real,
    ) -> Result<(), ClothError> {
        self.vertex_bounds.clear();
        self.vertex_bounds
            .extend((0..self.mesh.rest_positions().len()).map(vertex));
        self.topology
            .vertices
            .refit(&self.vertex_bounds, &mut self.vertex_nodes)?;
        let half = 0.5 * reach;
        self.edge_half.clear();
        self.edge_half
            .extend(self.edge_bounds.iter().map(|b| b.expanded(half)));
        self.edge_nodes_half.clear();
        self.edge_nodes_half
            .extend(self.edge_nodes.iter().map(|b| b.expanded(half)));
        Ok(())
    }

    /// Adds the lanes' work to the caller's counters and reports the first
    /// failure in lane order. Contacts and recorded pairs return to global root
    /// order (`roots` = face roots, edge roots), so the output does not depend
    /// on the lane count.
    fn merge_lanes<T>(
        &mut self,
        base: CollisionWork,
        lanes: Vec<Lane<T>>,
        roots: (usize, usize),
        out: &mut Vec<SurfaceContact>,
        pairs: &mut SweptPairs,
    ) -> Result<Vec<T>, ClothError> {
        let limits = self.settings.limits;
        self.work = base;
        for lane in &lanes {
            self.work.charge(
                CollisionBudgetKind::CandidatePairs,
                lane.work.candidate_pairs - base.candidate_pairs,
                limits,
            )?;
            self.work.charge(
                CollisionBudgetKind::CcdChecks,
                lane.work.ccd_checks - base.ccd_checks,
                limits,
            )?;
            self.work.charge(
                CollisionBudgetKind::RetainedContacts,
                lane.work.retained_contacts,
                limits,
            )?;
        }
        let n = lanes.len();
        let mut values = Vec::with_capacity(n);
        let (mut face_contacts, mut edge_contacts) = (Vec::with_capacity(n), Vec::with_capacity(n));
        let (mut face_pairs, mut edge_pairs) = (Vec::with_capacity(n), Vec::with_capacity(n));
        for (k, lane) in lanes.into_iter().enumerate() {
            values.push(lane.result?);
            // Lane `k` processed face roots k, k + n, ... before its edge roots.
            let face_roots = if k < roots.0 {
                (roots.0 - k).div_ceil(n)
            } else {
                0
            };
            let (fc, ec) = lane.counts.split_at(face_roots.min(lane.counts.len()));
            let contacts = |counts: &[[usize; 2]]| counts.iter().map(|c| c[0]).collect::<Vec<_>>();
            let recorded = |counts: &[[usize; 2]]| counts.iter().map(|c| c[1]).collect::<Vec<_>>();
            let face_items: usize = fc.iter().map(|c| c[0]).sum();
            let (fi, ei) = lane.contacts.split_at(face_items);
            face_contacts.push((fi.to_vec(), contacts(fc)));
            edge_contacts.push((ei.to_vec(), contacts(ec)));
            face_pairs.push((lane.pairs.faces, recorded(fc)));
            edge_pairs.push((lane.pairs.edges, recorded(ec)));
        }
        interleave(face_contacts, roots.0, out);
        interleave(edge_contacts, roots.1, out);
        interleave(face_pairs, roots.0, &mut pairs.faces);
        interleave(edge_pairs, roots.1, &mut pairs.edges);
        Ok(values)
    }

    /// `validate_initial` across the query lanes with a pairwise traversal.
    pub(super) fn validate_initial_lanes(
        &mut self,
        positions: &[Vec3],
    ) -> Result<bool, ClothError> {
        let lanes = self.lanes();
        let frontier = self.topology.triangles.frontier(ROOT_DEPTH);
        let mut roots = Vec::new();
        for (k, &i) in frontier.iter().enumerate() {
            for &j in &frontier[k..] {
                if self.triangle_nodes[i].overlaps(self.triangle_nodes[j]) {
                    roots.push((i, j));
                }
            }
        }
        let base = self.work;
        let scene = self.scene();
        let results = self.run(lanes, |k| {
            let (mut stack, mut work) = (vec![], base);
            let limits = scene.settings.limits;
            let mut result = Ok(());
            for &(i, j) in roots.iter().skip(k).step_by(lanes) {
                result = scene.topology.triangles.visit_self_pairs(
                    i,
                    j,
                    scene.triangle_nodes,
                    scene.triangle_bounds,
                    &mut stack,
                    |a, b| {
                        work.charge(CollisionBudgetKind::CandidatePairs, 1, limits)?;
                        let ta = scene.mesh.triangles()[a];
                        let tb = scene.mesh.triangles()[b];
                        if ta.iter().any(|v| tb.contains(v)) {
                            return Ok(());
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
                        Ok(())
                    },
                );
                if result.is_err() {
                    break;
                }
            }
            (result, work)
        })?;
        let limits = self.settings.limits;
        self.work = base;
        for (_, work) in &results {
            self.work.charge(
                CollisionBudgetKind::CandidatePairs,
                work.candidate_pairs - base.candidate_pairs,
                limits,
            )?;
        }
        for (result, _) in results {
            result?;
        }
        Ok(true)
    }

    pub(crate) fn implicit_primitives(
        &mut self,
        x: &[Vec3],
        out: &mut Vec<SurfaceContact>,
    ) -> Result<(), ClothError> {
        // This result never populates the projection contact cache.
        self.contact_cache_valid = false;
        let limit = self.settings.limits.retained_contacts;
        if let Some((positions, contacts, candidates)) = &self.primitive_cache
            && positions.as_slice() == x
        {
            // Identical positions give identical contacts; charge the same work.
            self.work.charge(
                CollisionBudgetKind::CandidatePairs,
                *candidates,
                self.settings.limits,
            )?;
            if out.len() + contacts.len() > limit {
                return Err(ClothError::ContactBudgetExceeded { limit });
            }
            out.extend_from_slice(contacts);
            return Ok(());
        }
        self.primitive_cache = None;
        let initial = out.len();
        let candidates_before = self.work.candidate_pairs;
        self.refit(x)?;
        if !self.initial_checked {
            self.validate_initial(x)?;
        }
        let activation = self.settings.thickness + self.settings.activation_margin;
        self.fit_query(|v| Bounds::point(x[v]).expanded(activation), activation)?;
        let lanes = self.lanes();
        let base = self.work;
        let scene = self.scene();
        let roots = Roots::new(&scene);
        let results = self.run(lanes, |k| {
            let (mut stack, mut work, mut contacts, mut counts) = (vec![], base, vec![], vec![]);
            let result = static_pairs(
                &scene,
                x,
                &roots.lane(lanes, k),
                &mut stack,
                &mut work,
                &mut contacts,
                &mut counts,
            );
            Lane {
                result,
                work,
                contacts,
                pairs: SweptPairs::default(),
                counts,
            }
        })?;
        let root_counts = (roots.faces.len(), roots.edges.len());
        self.merge_lanes(base, results, root_counts, out, &mut SweptPairs::default())?;
        if out.len() - initial > 0 && out.len() > limit {
            return Err(ClothError::ContactBudgetExceeded { limit });
        }
        self.primitive_cache = Some((
            x.to_vec(),
            out[initial..].to_vec(),
            self.work.candidate_pairs - candidates_before,
        ));
        Ok(())
    }

    /// `motion_fraction` for the implicit solver, split across the query lanes.
    /// With `collect`, it also records every pair within contact range of the
    /// swept motion, so trial points on it can skip the hierarchy; otherwise it
    /// discards previously recorded pairs.
    pub(crate) fn implicit_motion_fraction(
        &mut self,
        start: &[Vec3],
        end: &[Vec3],
        collect: bool,
    ) -> Result<Real, ClothError> {
        self.swept_ready = false;
        self.swept.faces.clear();
        self.swept.edges.clear();
        if !self.prepare_sweep(start, end)? {
            return Ok(1.0);
        }
        let separation = self.settings.thickness * 0.9;
        let reach = if collect {
            separation.max(self.settings.thickness + self.settings.activation_margin)
        } else {
            separation
        };
        self.fit_query(
            |v| Bounds::points([start[v], end[v]]).expanded(reach),
            reach,
        )?;
        let lanes = self.lanes();
        let mut pairs = std::mem::take(&mut self.swept);
        let base = self.work;
        let scene = self.scene();
        let roots = Roots::new(&scene);
        let results = self.run(lanes, |k| {
            let (mut stack, mut work, mut contacts, mut counts) = (vec![], base, vec![], vec![]);
            let mut lane_pairs = SweptPairs::default();
            let result = swept_pairs(
                &scene,
                start,
                end,
                &roots.lane(lanes, k),
                &mut stack,
                &mut work,
                &mut contacts,
                collect.then_some(&mut lane_pairs),
                &mut counts,
            );
            Lane {
                result,
                work,
                contacts,
                pairs: lane_pairs,
                counts,
            }
        });
        let root_counts = (roots.faces.len(), roots.edges.len());
        let fraction = match results {
            Ok(results) => {
                let mut contacts = std::mem::take(&mut self.motion_contacts);
                let fractions =
                    self.merge_lanes(base, results, root_counts, &mut contacts, &mut pairs);
                self.motion_contacts = contacts;
                fractions.map(|f| f.into_iter().fold(1.0, Real::min))
            }
            Err(error) => Err(error),
        };
        let fraction = self.finish_sweep(fraction?)?;
        self.swept = pairs;
        self.swept_ready = collect;
        Ok(fraction)
    }

    /// Parent contacts at a point of the latest collecting sweep, from its
    /// recorded pairs. Returns false, without output, if no sweep is recorded.
    pub(crate) fn implicit_swept_primitives(
        &mut self,
        x: &[Vec3],
        out: &mut Vec<SurfaceContact>,
    ) -> Result<bool, ClothError> {
        if !self.swept_ready {
            return Ok(false);
        }
        let (settings, mesh, pairs, lanes) =
            (self.settings, &*self.mesh, &self.swept, self.lanes());
        let evaluate = |faces: &[[u32; 2]], edges: &[[u32; 2]], out: &mut Vec<SurfaceContact>| {
            for &[vertex, face] in faces {
                face_pair(settings, mesh, x, vertex as usize, face as usize, out)?;
            }
            for &[a, b] in edges {
                edge_pair(settings, mesh, x, a as usize, b as usize, out)?;
            }
            Ok::<_, ClothError>(())
        };
        let initial = out.len();
        if lanes == 1 || pairs.faces.len() + pairs.edges.len() < 4096 {
            evaluate(&pairs.faces, &pairs.edges, out)?;
        } else {
            let (nf, ne) = (pairs.faces.len(), pairs.edges.len());
            let results = self.run(lanes, |k| {
                let mut faces = vec![];
                let mut edges = vec![];
                let a = evaluate(&pairs.faces[chunk(nf, lanes, k)], &[], &mut faces);
                let b = evaluate(&[], &pairs.edges[chunk(ne, lanes, k)], &mut edges);
                (a, faces, b, edges)
            })?;
            let mut edge_parts = Vec::with_capacity(lanes);
            for (a, faces, b, edges) in results {
                a?;
                out.extend(faces);
                edge_parts.push((b, edges));
            }
            for (b, edges) in edge_parts {
                b?;
                out.extend(edges);
            }
        }
        let limit = settings.limits.retained_contacts;
        if out.len() > initial && out.len() > limit {
            return Err(ClothError::ContactBudgetExceeded { limit });
        }
        Ok(true)
    }
}

#[cfg(all(test, feature = "f64"))]
mod tests {
    use super::*;
    fn two_layers() -> (Arc<ClothMesh>, Vec<Vec3>, ClothContactSettings) {
        let x = vec![
            Vec3::new(-0.1, 0., -0.1),
            Vec3::new(0.1, 0., -0.1),
            Vec3::new(0., 0., 0.1),
            Vec3::new(-0.1, 0.0015, -0.1),
            Vec3::new(0.1, 0.0015, -0.1),
            Vec3::new(0., 0.0015, 0.1),
        ];
        let mesh = Arc::new(ClothMesh::new(x.clone(), vec![[0, 1, 2], [3, 4, 5]]).unwrap());
        let config = ClothContactSettings {
            thickness: 0.001,
            activation_margin: 0.001,
            continuous_self_collision: true,
            ..Default::default()
        };
        (mesh, x, config)
    }
    #[test]
    fn parent_contacts_do_not_certify_crossing_or_replace_projection_queries() {
        let (mesh, old, config) = two_layers();
        let mut x = old.clone();
        for p in &mut x[3..] {
            p.y = -0.0015;
        }
        let mut engine = SelfCollision::new(mesh.clone(), config);
        let mut cs = vec![];
        engine.implicit_primitives(&x, &mut cs).unwrap();
        assert!(!cs.is_empty());
        assert!(cs.iter().all(|c| c.gap(&x) > 0.));
        assert!(engine.motion_fraction(&old, &x).unwrap() < 1.);
        let mut shared = vec![];
        engine.generate(&old, &x, &mut shared, false).unwrap();
        assert!(shared.iter().any(|c| c.gap(&x) < 0.));
        let mut fresh = SelfCollision::new(mesh, config);
        let mut expected = vec![];
        fresh.generate(&old, &x, &mut expected, false).unwrap();
        assert_eq!(shared.len(), expected.len());
        for (a, b) in shared.iter().zip(&expected) {
            assert_eq!(a.key, b.key);
            assert_eq!(a.weights, b.weights);
            assert_eq!(a.normal, b.normal);
        }
    }
    #[test]
    fn initial_intersection_and_parent_contact_limits_remain_errors() {
        let (mesh, x, mut config) = two_layers();
        config.limits.retained_contacts = 1;
        let mut engine = SelfCollision::new(mesh.clone(), config);
        assert!(matches!(
            engine.implicit_primitives(&x, &mut vec![]),
            Err(ClothError::ContactBudgetExceeded { .. })
        ));
        let mut crossed = x;
        crossed[3].y = -0.0015;
        engine.begin(mesh, ClothContactSettings::default());
        assert!(matches!(
            engine.implicit_primitives(&crossed, &mut vec![]),
            Err(ClothError::InitialSelfIntersection { .. })
        ));
    }
    #[test]
    fn parent_faces_keep_multiplicity_at_a_shared_edge_and_snap_threshold() {
        let x = vec![
            Vec3::ZERO,
            Vec3::X,
            Vec3::Z,
            Vec3::new(0.5, 0., 0.5),
            Vec3::new(0.25, 0.0005, 0.25),
            Vec3::new(0.25, 1., 0.25),
            Vec3::new(0.35, 1., 0.25),
        ];
        let mesh =
            Arc::new(ClothMesh::new(x.clone(), vec![[0, 1, 3], [0, 3, 2], [4, 5, 6]]).unwrap());
        let config = ClothContactSettings {
            thickness: 0.000318,
            activation_margin: 0.000318,
            ..Default::default()
        };
        let mut engine = SelfCollision::new(mesh, config);
        for delta in [-1e-8, 0., 1e-8] {
            let mut q = x.clone();
            q[4].x += delta;
            let mut cs = vec![];
            engine.implicit_primitives(&q, &mut cs).unwrap();
            let selected: Vec<_> = cs
                .iter()
                .filter(|c| {
                    c.key.features[0] == SurfaceFeature::Vertex(4)
                        && matches!(c.key.features[1], SurfaceFeature::Face(0 | 1))
                })
                .collect();
            assert_eq!(selected.len(), 2);
            assert!(
                selected
                    .iter()
                    .all(|c| (c.gap(&q) - (0.0005 - 0.000318)).abs() < 1e-11)
            );
        }
    }
    #[test]
    fn query_lanes_reproduce_serial_contacts_fractions_and_work() {
        // Two offset 23x23 layers exceed the parallel threshold (1,058 vertices).
        let patch = crate::GridBuilder::new(23, 23)
            .size(0.22, 0.22)
            .build()
            .unwrap();
        let n = patch.rest_positions().len() as u32;
        let mut x = patch.rest_positions().to_vec();
        x.extend(
            patch
                .rest_positions()
                .iter()
                .map(|p| *p + Vec3::new(0.0031, 0.0009, -0.0017)),
        );
        let mut faces = patch.triangles().to_vec();
        faces.extend(patch.triangles().iter().map(|f| f.map(|i| i + n)));
        let mesh = Arc::new(ClothMesh::new(x.clone(), faces).unwrap());
        let config = ClothContactSettings {
            thickness: 0.000318,
            activation_margin: 0.000636,
            continuous_self_collision: true,
            ..Default::default()
        };
        // The upper layer moves down past the lower layer's clearance.
        let end: Vec<_> = x
            .iter()
            .enumerate()
            .map(|(i, p)| {
                if i >= n as usize {
                    *p - Vec3::Y * 0.0008 + Vec3::X * 0.0002 * (i % 7) as Real
                } else {
                    *p
                }
            })
            .collect();
        let run = |lanes: usize| {
            let mut engine = SelfCollision::new(mesh.clone(), config);
            engine.set_workers(Arc::new(Workers::new(lanes)));
            let mut contacts = vec![];
            engine.implicit_primitives(&x, &mut contacts).unwrap();
            let fraction = engine.implicit_motion_fraction(&x, &end, true).unwrap();
            let swept = engine.motion_contacts.clone();
            // Points on the recorded motion reproduce the hierarchy query.
            for alpha in [0.0, 0.3, 0.7, 1.0] {
                let trial: Vec<_> = x
                    .iter()
                    .zip(&end)
                    .map(|(&a, &b)| a + (b - a) * alpha)
                    .collect();
                let mut recorded = vec![];
                assert!(
                    engine
                        .implicit_swept_primitives(&trial, &mut recorded)
                        .unwrap()
                );
                let mut queried = vec![];
                let before = engine.work;
                engine.implicit_primitives(&trial, &mut queried).unwrap();
                engine.work = before;
                recorded.sort_by_key(|c| c.key);
                queried.sort_by_key(|c| c.key);
                assert_eq!(format!("{recorded:?}"), format!("{queried:?}"));
            }
            let still = engine.implicit_motion_fraction(&x, &x, false).unwrap();
            assert!(!engine.implicit_swept_primitives(&x, &mut vec![]).unwrap());
            (contacts, fraction, still, swept, engine.work)
        };
        let serial = run(1);
        assert!(serial.0.len() > 1000, "{}", serial.0.len());
        assert!(serial.1 < 1.0 && serial.2 == 1.0);
        assert!(!serial.3.is_empty());
        let same = |label: &str, a: &[SurfaceContact], b: &[SurfaceContact]| {
            assert_eq!(a.len(), b.len(), "{label}: lengths");
            for (i, (x, y)) in a.iter().zip(b).enumerate() {
                assert_eq!(format!("{x:?}"), format!("{y:?}"), "{label}: contact {i}");
            }
        };
        for lanes in [2, 3, 4] {
            let parallel = run(lanes);
            same("static", &parallel.0, &serial.0);
            assert_eq!(parallel.1.to_bits(), serial.1.to_bits());
            assert_eq!(parallel.2, serial.2);
            same("swept", &parallel.3, &serial.3);
            assert_eq!(parallel.4, serial.4);
        }
        // A small candidate budget fails in both modes without partial output.
        let mut bounded = config;
        bounded.limits.candidate_pairs = 100;
        for lanes in [1, 4] {
            let mut engine = SelfCollision::new(mesh.clone(), bounded);
            engine.set_workers(Arc::new(Workers::new(lanes)));
            assert!(matches!(
                engine.implicit_primitives(&x, &mut vec![]),
                Err(ClothError::CollisionBudgetExceeded { .. })
            ));
        }
    }
}
