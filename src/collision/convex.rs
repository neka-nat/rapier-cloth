//! Exact feature geometry of convex parts.
//!
//! The distance between a cloth triangle and a convex part is attained at a
//! cloth vertex against the part, a part vertex against the triangle, or a part
//! edge against a triangle edge (with ties for parallel features). Those three
//! families, each a continuous closed-form distance, give the whole-triangle
//! contacts and the sweep distance: no GJK direction to lose precision, no
//! clipped manifold whose points appear and vanish while the cloth moves.
use super::surface::{bounds, check_distance, pack_feature, push_contact};
use super::*;
use crate::collision::geometry::{SurfaceWitness, closest_segments, closest_triangle};
use crate::{
    ClothContactSettings, CollisionWork, SurfaceContact, SurfaceFeature,
    rapier::parry::shape::Shape,
};

/// Part-local vertices, edges and faces of a convex part, plus the radius of a
/// sphere-swept part (ball, capsule).
pub(super) struct ConvexFeatures {
    points: Vec<Vec3>,
    segments: Vec<[u32; 2]>,
    faces: Vec<ConvexFace>,
    face_vertices: Vec<u32>,
    radius: Real,
    /// A box's half extents: point distances and containment are then a
    /// clamp instead of face and edge tests.
    half_extents: Option<Vec3>,
}

struct ConvexFace {
    normal: Vec3,
    first: usize,
    count: usize,
}

/// Which part feature a pair involves; the external feature code follows
/// Parry's packing (header bits 01 vertex, 10 edge) so nothing aliases.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum PairFeature {
    /// A cloth vertex against the whole part.
    Part,
    Point(u32),
    Segment(u32),
}

/// A closest pair between a triangle and a part, in the part's local frame.
#[derive(Clone, Copy, Debug)]
pub(super) struct Pair {
    /// Point on the part's core (before the sphere-swept radius).
    pub on_part: Vec3,
    /// Unit direction from the part towards the cloth.
    pub normal: Vec3,
    /// Signed distance between the part's surface and the cloth point.
    pub distance: Real,
    /// Barycentric weights of the cloth point in the triangle's vertex order.
    pub weights: [Real; 3],
    pub feature: PairFeature,
}

impl ConvexFeatures {
    /// `None` for shapes the bridge does not support.
    pub(super) fn new(shape: &dyn Shape) -> Option<Self> {
        if let Some(ball) = shape.as_ball() {
            return Some(Self {
                points: vec![Vec3::ZERO],
                segments: Vec::new(),
                faces: Vec::new(),
                face_vertices: Vec::new(),
                radius: ball.radius,
                half_extents: None,
            });
        }
        if let Some(capsule) = shape.as_capsule() {
            return Some(Self {
                points: vec![capsule.segment.a, capsule.segment.b],
                segments: vec![[0, 1]],
                faces: Vec::new(),
                face_vertices: Vec::new(),
                radius: capsule.radius,
                half_extents: None,
            });
        }
        if let Some(cuboid) = shape.as_cuboid() {
            let he = cuboid.half_extents;
            let points: Vec<Vec3> = (0..8u32)
                .map(|i| {
                    Vec3::new(
                        if i & 1 == 0 { -he.x } else { he.x },
                        if i & 2 == 0 { -he.y } else { he.y },
                        if i & 4 == 0 { -he.z } else { he.z },
                    )
                })
                .collect();
            let mut segments = Vec::new();
            for i in 0..8u32 {
                for bit in [1u32, 2, 4] {
                    if i & bit == 0 {
                        segments.push([i, i | bit]);
                    }
                }
            }
            let mut faces = Vec::new();
            let mut face_vertices = Vec::new();
            for axis in 0..3usize {
                let bit = 1u32 << axis;
                let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
                for sign in [0u32, bit] {
                    let mut normal = Vec3::ZERO;
                    normal[axis] = if sign == 0 { -1.0 } else { 1.0 };
                    let first = face_vertices.len();
                    // Around the face: (0,0), (1,0), (1,1), (0,1) in the other axes.
                    for (a, b) in [(0u32, 0u32), (1, 0), (1, 1), (0, 1)] {
                        face_vertices.push(sign | (a << u) | (b << v));
                    }
                    faces.push(ConvexFace {
                        normal,
                        first,
                        count: 4,
                    });
                }
            }
            return Some(Self {
                points,
                segments,
                faces,
                face_vertices,
                radius: 0.0,
                half_extents: Some(he),
            });
        }
        if let Some(polyhedron) = shape.as_convex_polyhedron() {
            let points = polyhedron.points().to_vec();
            // Edges merged into a coplanar face keep both face ids equal.
            let segments = polyhedron
                .edges()
                .iter()
                .filter(|edge| edge.faces[0] != edge.faces[1])
                .map(|edge| edge.vertices)
                .collect();
            let adjacency = polyhedron.vertices_adj_to_face();
            let mut faces = Vec::new();
            let mut face_vertices = Vec::new();
            for face in polyhedron.faces() {
                let first = face_vertices.len();
                let start = face.first_vertex_or_edge as usize;
                let count = face.num_vertices_or_edges as usize;
                face_vertices.extend_from_slice(&adjacency[start..start + count]);
                faces.push(ConvexFace {
                    normal: face.normal,
                    first,
                    count,
                });
            }
            return Some(Self {
                points,
                segments,
                faces,
                face_vertices,
                radius: 0.0,
                half_extents: None,
            });
        }
        None
    }

    /// Farthest extent of the part along the local direction `normal`.
    pub(super) fn support(&self, normal: Vec3) -> Real {
        if let Some(he) = self.half_extents {
            return normal.abs().dot(he) + self.radius;
        }
        self.points
            .iter()
            .map(|p| normal.dot(*p))
            .fold(-Real::INFINITY, Real::max)
            + self.radius
    }

    /// Closest surface point to `p`: the point on the part's core, the outward
    /// direction, and the signed distance to the sphere-swept surface. Inside a
    /// polytope the direction is the normal of the least-deep face.
    pub(super) fn closest_to_point(&self, p: Vec3) -> Option<(Vec3, Vec3, Real)> {
        if let Some(he) = self.half_extents {
            let q = p.clamp(-he, he);
            let delta = p - q;
            let distance = delta.length();
            if distance > 0.0 {
                return Some((q, delta / distance, distance - self.radius));
            }
            // Inside: leave through the nearest face.
            let depths = he - p.abs();
            let axis = if depths.x <= depths.y && depths.x <= depths.z {
                0
            } else if depths.y <= depths.z {
                1
            } else {
                2
            };
            let mut normal = Vec3::ZERO;
            normal[axis] = if p[axis] < 0.0 { -1.0 } else { 1.0 };
            let mut foot = p;
            foot[axis] = normal[axis] * he[axis];
            return Some((foot, normal, -depths[axis] - self.radius));
        }
        let mut best: Option<(Vec3, Vec3, Real)> = None;
        let mut deepest: Option<(Vec3, Vec3, Real)> = None;
        let mut outside = self.faces.is_empty();
        for face in &self.faces {
            let vertices = &self.face_vertices[face.first..face.first + face.count];
            let origin = self.points[vertices[0] as usize];
            let height = face.normal.dot(p - origin);
            if height < 0.0 {
                if deepest.is_none_or(|d| height > d.2) {
                    deepest = Some((p - face.normal * height, face.normal, height));
                }
                continue;
            }
            outside = true;
            let foot = p - face.normal * height;
            let mut positive = false;
            let mut negative = false;
            for i in 0..face.count {
                let a = self.points[vertices[i] as usize];
                let b = self.points[vertices[(i + 1) % face.count] as usize];
                let edge = b - a;
                let side = face.normal.dot(edge.cross(foot - a));
                let tolerance =
                    Real::EPSILON * 16.0 * (edge.length_squared() + (foot - a).length_squared());
                if side > tolerance {
                    positive = true;
                } else if side < -tolerance {
                    negative = true;
                }
            }
            if positive && negative {
                continue;
            }
            if best.is_none_or(|b| height < b.2) {
                best = Some((foot, face.normal, height));
            }
        }
        if !outside {
            return deepest.map(|(q, n, depth)| (q, n, depth - self.radius));
        }
        if best.is_none() {
            for segment in &self.segments {
                let a = self.points[segment[0] as usize];
                let b = self.points[segment[1] as usize];
                let witness = closest_segments([a, b], [p, p])?;
                let delta = p - witness.a;
                let distance = delta.length();
                if best.is_none_or(|b| distance < b.2) && distance > 0.0 {
                    best = Some((witness.a, delta / distance, distance));
                }
            }
            if self.segments.is_empty() {
                for &q in &self.points {
                    let delta = p - q;
                    let distance = delta.length();
                    if best.is_none_or(|b| distance < b.2) && distance > 0.0 {
                        best = Some((q, delta / distance, distance));
                    }
                }
            }
        }
        best.map(|(q, n, distance)| (q, n, distance - self.radius))
    }

    /// Whether the part-local point `p` lies inside the part's surface.
    fn contains(&self, p: Vec3) -> bool {
        if let Some(he) = self.half_extents {
            return p.abs().cmple(he).all();
        }
        if self.faces.is_empty() {
            return self
                .closest_to_point(p)
                .is_some_and(|(_, _, distance)| distance < 0.0);
        }
        self.faces.iter().all(|face| {
            let origin = self.points[self.face_vertices[face.first] as usize];
            face.normal.dot(p - origin) <= 0.0
        })
    }

    /// Minimum translation separating a penetrating triangle from a polytope:
    /// the separating-axis search over face normals, the triangle normal and
    /// edge cross products. `None` when the triangle is separated (or the part
    /// has no faces). The axis points from the part towards the triangle.
    fn penetration(&self, t: [Vec3; 3]) -> Option<(Vec3, Real)> {
        if self.faces.is_empty() {
            return None;
        }
        let mut best: Option<(Vec3, Real)> = None;
        let mut separated = false;
        let mut consider = |axis: Vec3| {
            if separated {
                return;
            }
            let n = axis.normalize_or_zero();
            if !n.is_finite() || n.length_squared() < 0.5 {
                return;
            }
            let (mut tmin, mut tmax) = (Real::INFINITY, -Real::INFINITY);
            for p in t {
                let d = n.dot(p);
                tmin = tmin.min(d);
                tmax = tmax.max(d);
            }
            let (mut pmin, mut pmax) = (Real::INFINITY, -Real::INFINITY);
            for q in &self.points {
                let d = n.dot(*q);
                pmin = pmin.min(d);
                pmax = pmax.max(d);
            }
            pmin -= self.radius;
            pmax += self.radius;
            if tmin > pmax || pmin > tmax {
                separated = true;
                return;
            }
            let (axis, depth) = if pmax - tmin <= tmax - pmin {
                (n, pmax - tmin)
            } else {
                (-n, tmax - pmin)
            };
            if best.is_none_or(|b| depth < b.1) {
                best = Some((axis, depth));
            }
        };
        for face in &self.faces {
            consider(face.normal);
        }
        let edges = [t[1] - t[0], t[2] - t[1], t[0] - t[2]];
        consider(edges[0].cross(edges[1]));
        for segment in &self.segments {
            let direction = self.points[segment[1] as usize] - self.points[segment[0] as usize];
            for edge in edges {
                consider(direction.cross(edge));
            }
        }
        if separated { None } else { best }
    }

    /// The contact of the cloth witness `a`, selected by the part feature at
    /// `selector` (a vertex or edge point) or by the whole part. Only a
    /// penetrating triangle can have a witness inside a polytope; such a
    /// witness reports the signed depth along the triangle's minimum
    /// translation axis, so every inside witness of the triangle is pushed out
    /// the same way. For sphere-swept parts the selected distance is already
    /// signed; a witness exactly on the core uses the triangle normal `face`.
    fn witness_pair(
        &self,
        a: Vec3,
        selector: Option<Vec3>,
        weights: [Real; 3],
        feature: PairFeature,
        face: Vec3,
        penetration: &impl Fn() -> Option<(Vec3, Real)>,
    ) -> Option<Pair> {
        if !self.faces.is_empty()
            && self.contains(a)
            && let Some((normal, _)) = penetration()
        {
            let distance = normal.dot(a) - self.support(normal);
            return Some(Pair {
                on_part: a - normal * (distance + self.radius),
                normal,
                distance,
                weights,
                feature,
            });
        }
        let (on_part, normal, distance) = match selector {
            Some(q) => {
                let delta = a - q;
                let length = delta.length();
                if length > 0.0 {
                    (q, delta / length, length - self.radius)
                } else if self.radius > 0.0 && face.is_finite() {
                    (q, face, -self.radius)
                } else {
                    return None;
                }
            }
            None => self.closest_to_point(a)?,
        };
        Some(Pair {
            on_part,
            normal,
            distance,
            weights,
            feature,
        })
    }

    /// Every pair of the triangle `t` (part-local) and the part within
    /// `range`, in a fixed order.
    /// Every pair of the triangle `t` (part-local) and the part within
    /// `range`, in a fixed order; `skip_vertex(k)` omits cloth vertex `k`'s
    /// pair with the whole part (a caller emits it once per query).
    pub(super) fn pairs<E>(
        &self,
        t: [Vec3; 3],
        range: Real,
        mut skip_vertex: impl FnMut(usize) -> bool,
        mut emit: impl FnMut(Pair) -> Result<(), E>,
    ) -> Result<(), E> {
        let overlap = std::cell::OnceCell::new();
        let penetration = || *overlap.get_or_init(|| self.penetration(t));
        let face = (t[1] - t[0]).cross(t[2] - t[0]).normalize_or_zero();
        self.vertex_pairs(t, range, face, &penetration, &mut skip_vertex, &mut emit)?;
        self.feature_pairs(t, range, face, &penetration, &mut emit)
    }

    /// The closest pair of the triangle `t` (part-local) and the part. The
    /// cloth vertices' distances bound it, so only part vertices and edges
    /// within that bound of the triangle need to be tested.
    pub(super) fn closest_pair(&self, t: [Vec3; 3]) -> Option<Pair> {
        let overlap = std::cell::OnceCell::new();
        let penetration = || *overlap.get_or_init(|| self.penetration(t));
        let face = (t[1] - t[0]).cross(t[2] - t[0]).normalize_or_zero();
        fn keep(best: &mut Option<Pair>, pair: Pair) {
            if best.is_none_or(|b| pair.distance < b.distance) {
                *best = Some(pair);
            }
        }
        let mut best: Option<Pair> = None;
        let _ = self.vertex_pairs(
            t,
            Real::INFINITY,
            face,
            &penetration,
            &mut |_| false,
            &mut |pair| {
                keep(&mut best, pair);
                Ok::<(), ()>(())
            },
        );
        let bound = best.map_or(Real::INFINITY, |b| b.distance.max(0.0));
        let _ = self.feature_pairs(t, bound, face, &penetration, &mut |pair| {
            keep(&mut best, pair);
            Ok::<(), ()>(())
        });
        best
    }

    /// Each cloth vertex against the whole part.
    fn vertex_pairs<E>(
        &self,
        t: [Vec3; 3],
        range: Real,
        face: Vec3,
        penetration: &impl Fn() -> Option<(Vec3, Real)>,
        skip_vertex: &mut impl FnMut(usize) -> bool,
        emit: &mut impl FnMut(Pair) -> Result<(), E>,
    ) -> Result<(), E> {
        for (k, &p) in t.iter().enumerate() {
            if skip_vertex(k) {
                continue;
            }
            let mut weights = [0.0; 3];
            weights[k] = 1.0;
            if let Some(pair) =
                self.witness_pair(p, None, weights, PairFeature::Part, face, penetration)
                && pair.distance <= range
            {
                emit(pair)?;
            }
        }
        Ok(())
    }

    /// Part vertices against the triangle and part edges against the cloth
    /// edges, for part features within `range` of the triangle's bounds.
    fn feature_pairs<E>(
        &self,
        t: [Vec3; 3],
        range: Real,
        face: Vec3,
        penetration: &impl Fn() -> Option<(Vec3, Real)>,
        emit: &mut impl FnMut(Pair) -> Result<(), E>,
    ) -> Result<(), E> {
        let reach = bounds(t, range + self.radius);
        for (i, &q) in self.points.iter().enumerate() {
            if q.cmplt(reach.mins).any() || q.cmpgt(reach.maxs).any() {
                continue;
            }
            let Some(witness) = closest_triangle(q, t) else {
                continue;
            };
            if let Some(pair) = self.witness_pair(
                witness.point,
                Some(q),
                witness.barycentric,
                PairFeature::Point(i as u32),
                face,
                penetration,
            ) && pair.distance <= range
            {
                emit(pair)?;
            }
        }
        for (j, segment) in self.segments.iter().enumerate() {
            let a = self.points[segment[0] as usize];
            let b = self.points[segment[1] as usize];
            if a.min(b).cmpgt(reach.maxs).any() || a.max(b).cmplt(reach.mins).any() {
                continue;
            }
            for e in 0..3 {
                let (u, v) = (e, (e + 1) % 3);
                let Some(witness) = closest_segments([t[u], t[v]], [a, b]) else {
                    continue;
                };
                let mut weights = [0.0; 3];
                weights[u] = 1.0 - witness.parameters[0];
                weights[v] = witness.parameters[0];
                if let Some(pair) = self.witness_pair(
                    witness.a,
                    Some(witness.b),
                    weights,
                    PairFeature::Segment(j as u32),
                    face,
                    penetration,
                ) && pair.distance <= range
                {
                    emit(pair)?;
                }
            }
            // A part edge piercing the triangle's interior leaves every other
            // witness outside; the piercing point itself carries the push.
            if let Some(witness) = pierce(t, [a, b])
                && penetration().is_some()
                && let Some(pair) = self.witness_pair(
                    witness.point,
                    None,
                    witness.barycentric,
                    PairFeature::Segment(j as u32),
                    face,
                    penetration,
                )
                && pair.distance <= range
            {
                emit(pair)?;
            }
        }
        Ok(())
    }
}

/// Emits the contacts of cloth triangles against one convex part.
pub(super) struct PartContacts<'a> {
    pub features: &'a ConvexFeatures,
    pub collider: ColliderHandle,
    pub body: Option<&'a RigidBody>,
    /// Pose of the part the pairs are evaluated at.
    pub pose: Pose,
    /// Pose the contacts are expressed at (the ending pose for swept witnesses).
    pub output_pose: Pose,
    pub particle_count: usize,
    pub template: SurfaceContact,
    pub part: Option<u32>,
}

impl PartContacts<'_> {
    /// Contacts of one triangle (`indices`, world `points`) with the part within
    /// `range` of its surface. `skip_vertex` lets a caller emit each cloth
    /// vertex's contact with the whole part once per query.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn generate(
        &self,
        indices: [u32; 3],
        face: u32,
        points: [Vec3; 3],
        range: Real,
        stage: Option<ContactStage>,
        config: ClothContactSettings,
        limit: usize,
        excluded: &BTreeSet<(u32, (u32, u32))>,
        out: &mut Vec<SurfaceContact>,
        work: &mut CollisionWork,
        mut skip_vertex: impl FnMut(u32) -> bool,
    ) -> Result<(), IntegrationError> {
        let local = points.map(|p| self.pose.inverse_transform_point(p));
        if closest_triangle(Vec3::ZERO, local).is_none() {
            return Err(ClothError::DegenerateConstraint.into());
        }
        self.features.pairs(
            local,
            range,
            |k| skip_vertex(indices[k]),
            |pair| {
                let (witness, code) = match pair.feature {
                    PairFeature::Part => (
                        SurfaceWitness::from_triangle(indices, face, pair.weights)?,
                        0,
                    ),
                    PairFeature::Point(i) => (
                        SurfaceWitness::from_triangle(indices, face, pair.weights)?,
                        0x4000_0000 | i,
                    ),
                    PairFeature::Segment(j) => (
                        SurfaceWitness::from_triangle(indices, face, pair.weights)?,
                        0x8000_0000 | j,
                    ),
                };
                if witness
                    .particles
                    .iter()
                    .zip(witness.weights)
                    .all(|(&i, w)| {
                        w == 0.0 || excluded.contains(&(i, self.collider.into_raw_parts()))
                    })
                {
                    return Ok(());
                }
                if !pair.normal.is_finite() || !pair.distance.is_finite() {
                    return Err(ClothError::NonFiniteState.into());
                }
                let mut contact = self.template;
                contact.key.features[0] = witness.feature;
                if let SurfaceFeature::External { feature, .. } = &mut contact.key.features[1] {
                    *feature = pack_feature(self.part, code).map_err(|reason| {
                        IntegrationError::UnsupportedCollision {
                            collider: self.collider,
                            reason,
                        }
                    })?;
                }
                contact.particles = [
                    witness.particles[0],
                    witness.particles[1],
                    witness.particles[2],
                    0,
                ];
                contact.weights = [
                    witness.weights[0],
                    witness.weights[1],
                    witness.weights[2],
                    0.0,
                ];
                contact.normal = self.output_pose.rotation * pair.normal;
                contact.offset = self
                    .output_pose
                    .transform_point(pair.on_part + pair.normal * self.features.radius);
                contact.surface_velocity = self
                    .body
                    .filter(|b| b.is_kinematic())
                    .map_or(Vec3::ZERO, |b| b.velocity_at_point(contact.offset));
                if let Some(stage) = stage {
                    check_distance(
                        pair.distance,
                        contact.separation,
                        stage,
                        self.collider,
                        witness.feature,
                    )?;
                }
                push_contact(contact, self.particle_count, config, limit, out, work)
                    .map_err(IntegrationError::from)
            },
        )
    }
}

/// Where the segment crosses the triangle's interior, as a triangle witness.
fn pierce(t: [Vec3; 3], segment: [Vec3; 2]) -> Option<crate::collision::geometry::TriangleWitness> {
    let normal = (t[1] - t[0]).cross(t[2] - t[0]);
    let da = normal.dot(segment[0] - t[0]);
    let db = normal.dot(segment[1] - t[0]);
    if !(da.is_finite() && db.is_finite()) || da * db > 0.0 || da == db {
        return None;
    }
    let point = segment[0] + (segment[1] - segment[0]) * (da / (da - db));
    let witness = closest_triangle(point, t)?;
    (witness.point.distance_squared(point) <= Real::EPSILON * 64.0 * normal.length())
        .then_some(witness)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::parry::shape::SharedShape;

    fn features(shape: &SharedShape) -> ConvexFeatures {
        ConvexFeatures::new(shape.as_ref()).unwrap()
    }

    #[test]
    fn point_distances_match_parry_projection_for_every_supported_shape() {
        let hull = SharedShape::convex_hull(
            &(0..8)
                .map(|i| {
                    Vec3::new(
                        if i & 1 == 0 { -0.02 } else { 0.03 },
                        if i & 2 == 0 { -0.01 } else { 0.02 },
                        if i & 4 == 0 { -0.015 } else { 0.025 },
                    )
                })
                .chain([Vec3::new(0.0, 0.05, 0.0)])
                .collect::<Vec<_>>(),
        )
        .unwrap();
        let shapes = [
            SharedShape::ball(0.03),
            SharedShape::capsule_x(0.02, 0.01),
            SharedShape::cuboid(0.02, 0.03, 0.04),
            hull,
        ];
        let mut seed = 12345u64;
        let mut random = || {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((seed >> 33) as Real / (1u64 << 31) as Real) * 0.2 - 0.1
        };
        for shape in &shapes {
            let features = features(shape);
            for _ in 0..200 {
                let p = Vec3::new(random(), random(), random());
                let (_, normal, distance) = features.closest_to_point(p).unwrap();
                // Project onto the boundary so inside points report their depth.
                let projection = shape.project_point(&Pose::IDENTITY, p, false);
                let expected = if projection.is_inside {
                    -projection.point.distance(p)
                } else {
                    projection.point.distance(p)
                };
                if shape.as_convex_polyhedron().is_some() && Real::EPSILON > 1e-10 && expected > 0.0
                {
                    // Parry's f32 GJK projection onto a hull can overestimate by
                    // millimetres (13% was observed); the exact features never
                    // exceed it and agree with it at f64 below.
                    assert!(
                        distance <= expected + 1e-6 && distance >= expected * 0.75,
                        "{shape:?}: p={p:?} distance {distance} expected {expected}"
                    );
                } else {
                    let tolerance = 2e-5 * (1.0 + p.length());
                    assert!(
                        (distance - expected).abs() < tolerance,
                        "{shape:?}: p={p:?} distance {distance} expected {expected}"
                    );
                }
                assert!((normal.length() - 1.0).abs() < 1e-4);
                if distance > 0.0
                    && (shape.as_convex_polyhedron().is_none() || Real::EPSILON < 1e-10)
                {
                    let towards = (p - projection.point).normalize();
                    assert!(
                        normal.dot(towards) > 0.999,
                        "{shape:?}: {normal:?} vs {towards:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn triangle_pairs_reach_every_feature_family() {
        let cube = features(&SharedShape::cuboid(0.02, 0.02, 0.02));
        // A triangle hovering over the top face: its interior is cut by the
        // cube's top corners (points), its edges cross the top edges (segments)
        // and one vertex hangs beside the cube (part).
        let triangle = [
            Vec3::new(-0.05, 0.0205, 0.0),
            Vec3::new(0.05, 0.0205, -0.05),
            Vec3::new(0.05, 0.0205, 0.05),
        ];
        let mut kinds = std::collections::BTreeSet::new();
        let closest = cube.closest_pair(triangle).unwrap();
        assert!((closest.distance - 0.0005).abs() < 1e-9, "{closest:?}");
        cube.pairs(
            triangle,
            0.001,
            |_| false,
            |pair| -> Result<(), ()> {
                kinds.insert(match pair.feature {
                    PairFeature::Part => 0,
                    PairFeature::Point(_) => 1,
                    PairFeature::Segment(_) => 2,
                });
                assert!(pair.distance >= 0.0005 - 1e-9);
                assert!(pair.normal.y > 0.0);
                Ok(())
            },
        )
        .unwrap();
        assert_eq!(
            kinds.len(),
            2,
            "corner points and top edges are within range"
        );
        // A vertex beside the cube at the same height reaches the part family.
        let beside = [
            Vec3::new(0.0205, 0.0, 0.0),
            Vec3::new(0.1, 0.0, -0.05),
            Vec3::new(0.1, 0.0, 0.05),
        ];
        let closest = cube.closest_pair(beside).unwrap();
        assert_eq!(closest.feature, PairFeature::Part);
        assert!((closest.distance - 0.0005).abs() < 1e-9);
        assert!(closest.normal.x > 0.999);
    }

    #[test]
    fn support_bounds_the_projection_of_every_pair() {
        let capsule = features(&SharedShape::capsule_y(0.03, 0.01));
        let triangle = [
            Vec3::new(0.02, 0.01, -0.03),
            Vec3::new(0.05, 0.05, 0.02),
            Vec3::new(0.01, -0.06, 0.03),
        ];
        let closest = capsule.closest_pair(triangle).unwrap();
        let projected = triangle
            .iter()
            .map(|p| closest.normal.dot(*p))
            .fold(Real::INFINITY, Real::min)
            - capsule.support(closest.normal);
        assert!(
            (projected - closest.distance).abs() < 1e-6,
            "{projected} vs {closest:?}"
        );
    }
}
