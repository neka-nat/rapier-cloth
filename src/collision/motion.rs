use super::surface::{TriangleContactQuery, bounds, deduplicate, overlaps, push_contact};
use super::*;
use crate::rapier::parry::shape::{Shape, Triangle};
use crate::{ContactMotion, SurfaceContact, SurfaceContactKey, SurfaceFeature};

impl RapierContacts<'_, '_> {
    pub(super) fn generate_motion(
        &mut self,
        proposed: ContactMotion<'_>,
        work: &mut CollisionWork,
    ) -> Result<Real, IntegrationError> {
        self.motion_contacts.clear();
        let Some(config) = self
            .surface_settings
            .filter(|s| s.continuous_rigid_collision)
        else {
            return Ok(1.0);
        };
        let mesh = self.surface_mesh.as_ref().unwrap();
        if proposed.start.len() != mesh.rest_positions().len()
            || proposed.end.len() != proposed.start.len()
            || proposed
                .start
                .iter()
                .chain(proposed.end)
                .any(|p| !p.is_finite())
        {
            return Err(
                ClothError::InvalidSurfaceContact("invalid external motion positions").into(),
            );
        }
        let separation = config.thickness * 0.5;
        let minimum = separation * 0.9;
        let extent = proposed
            .start
            .iter()
            .chain(proposed.end)
            .map(|p| p.length())
            .fold(0.0, Real::max);
        let cloth_bounds = bounds(
            proposed.start.iter().chain(proposed.end).copied(),
            minimum + extent * Real::EPSILON * 128.0,
        );
        let mut fraction: Real = 1.0;
        self.candidate_queries += 1;
        for (handle, collider) in self.scene.query.colliders.iter() {
            if !collider.is_enabled()
                || collider.is_sensor()
                || self.settings.excluded_colliders.contains(&handle)
                || !self
                    .scene
                    .query
                    .filter
                    .test(self.scene.query.bodies, handle, collider)
            {
                continue;
            }
            let body = collider
                .parent()
                .and_then(|h| self.scene.query.bodies.get(h));
            let shape = collider.shape();
            let motion = RigidMotion::new(self.scene, handle, proposed.stage)?;
            if shape.as_halfspace().is_none() && !overlaps(&cloth_bounds, &motion.swept_bounds()) {
                continue;
            }
            if body.is_some_and(|b| b.is_dynamic())
                || (shape.as_ball().is_none()
                    && shape.as_cuboid().is_none()
                    && shape.as_capsule().is_none()
                    && shape.as_halfspace().is_none())
            {
                return Err(IntegrationError::UnsupportedCollision {
                    collider: handle,
                    reason: "continuous surface primitive is unsupported",
                });
            }
            let raw = handle.into_raw_parts();
            let external = (u64::from(raw.1) << 32) | u64::from(raw.0);
            let friction = |mu: Real| {
                self.settings
                    .friction_override
                    .unwrap_or((mu + collider.friction()) * 0.5)
            };
            let mut template = SurfaceContact {
                key: SurfaceContactKey {
                    other_cloth: None,
                    features: [
                        SurfaceFeature::Vertex(0),
                        SurfaceFeature::External {
                            object: external,
                            feature: 0,
                        },
                    ],
                },
                particles: [0; 4],
                weights: [0.0; 4],
                normal: Vec3::Y,
                offset: Vec3::ZERO,
                surface_velocity: Vec3::ZERO,
                separation,
                static_friction: friction(config.static_friction),
                kinetic_friction: friction(config.kinetic_friction),
            };
            if let Some(plane) = shape.as_halfspace() {
                let pose = motion.at(0.0);
                let normal = pose.rotation * plane.normal;
                for (i, (&a, &b)) in proposed.start.iter().zip(proposed.end).enumerate() {
                    if self.excluded_pairs.contains(&(i as u32, raw)) {
                        continue;
                    }
                    work.charge(CollisionBudgetKind::CandidatePairs, 1, config.limits)?;
                    work.charge(CollisionBudgetKind::CcdChecks, 1, config.limits)?;
                    self.pair_queries += 1;
                    let rounding = (a.abs() + b.abs() + pose.translation.abs() * 2.0)
                        .dot(normal.abs())
                        * Real::EPSILON
                        * 64.0;
                    let da = normal.dot(a - pose.translation) - rounding;
                    let db = normal.dot(b - pose.translation) - rounding;
                    if !da.is_finite() || !db.is_finite() || da <= minimum {
                        return Err(ClothError::UnresolvedContinuousCollision(
                            "insufficient plane separation or numerical clearance",
                        )
                        .into());
                    }
                    if db > minimum {
                        continue;
                    }
                    let allowed = 0.8 * (da - minimum) / (da - db);
                    fraction = fraction.min(allowed);
                    template.key.features[0] = SurfaceFeature::Vertex(i as u32);
                    template.particles = [i as u32, 0, 0, 0];
                    template.weights = [1.0, 0.0, 0.0, 0.0];
                    template.normal = normal;
                    template.offset = a - normal * normal.dot(a - pose.translation);
                    push_contact(
                        template,
                        proposed.start.len(),
                        config,
                        self.limit,
                        &mut self.motion_contacts,
                        work,
                    )?;
                }
                continue;
            }
            let obstacle_bounds = motion.swept_bounds();
            for (face, &indices) in mesh.triangles().iter().enumerate() {
                if indices
                    .iter()
                    .all(|i| self.excluded_pairs.contains(&(*i, raw)))
                {
                    continue;
                }
                let start = indices.map(|i| proposed.start[i as usize]);
                let end = indices.map(|i| proposed.end[i as usize]);
                if !overlaps(
                    &bounds(
                        start.into_iter().chain(end),
                        minimum + extent * Real::EPSILON * 128.0,
                    ),
                    &obstacle_bounds,
                ) {
                    continue;
                }
                work.charge(CollisionBudgetKind::CandidatePairs, 1, config.limits)?;
                self.pair_queries += 1;
                let sweep = PrimitiveSweep {
                    shape,
                    motion,
                    start,
                    end,
                    minimum,
                };
                let result = sweep.advance(work, config.limits)?;
                if let CcdResult::Limited { fraction: allowed } = result {
                    fraction = fraction.min(allowed);
                    let pose = motion.at(allowed);
                    let points = sweep.positions(allowed);
                    // Conservative advancement can stop outside the ordinary
                    // activation band; request the swept manifold explicitly.
                    let activation = points
                        .iter()
                        .map(|p| p.distance(pose.translation))
                        .fold(0.0, Real::max)
                        + motion.radius * 2.0
                        + separation;
                    TriangleContactQuery {
                        shape,
                        collider: handle,
                        body,
                        pose,
                        output_pose: motion.at(1.0),
                        indices,
                        particle_count: proposed.start.len(),
                        face: face as u32,
                        points,
                        stage: None,
                        activation,
                        template,
                    }
                    .generate(
                        &mut self.surface_manifold,
                        config,
                        self.limit,
                        self.excluded_pairs,
                        &mut self.motion_contacts,
                        work,
                    )?;
                }
            }
        }
        deduplicate(&mut self.motion_contacts, config, self.limit, work)?;
        if fraction < 1.0 {
            work.limited_advances = work.limited_advances.saturating_add(1);
        }
        Ok(fraction)
    }
}
use crate::{
    CollisionBudgetKind, CollisionLimits, CollisionWork,
    collision::{ccd::CcdResult, geometry::closest_triangle},
};

/// COM-linear motion with the actual endpoint rotation. Rapier 0.34's serial
/// solver advances velocity-based rotations with normalized linear quaternion
/// increments. Their total angle is smaller than |omega|*h. Below a half-turn
/// the endpoint angle is unambiguous; constant angular interpolation also meets
/// its equal-size internal solver endpoints. Larger velocity commands require
/// a smaller application substep, even if the ending quaternion looks unchanged.
#[derive(Debug, Clone, Copy)]
struct RigidMotion {
    start: Pose,
    end: Pose,
    body_start: Pose,
    local: Pose,
    local_com: Vec3,
    com_start: Vec3,
    com_delta: Vec3,
    angular: Vec3,
    angular_speed_bound: Real,
    radius: Real,
    physical: bool,
}
impl RigidMotion {
    fn new(
        scene: &RapierScene<'_>,
        handle: ColliderHandle,
        stage: ContactStage,
    ) -> Result<Self, IntegrationError> {
        let collider = &scene.query.colliders[handle];
        let raw = handle.into_raw_parts();
        let previous = *scene
            .previous
            .colliders
            .get(&raw)
            .ok_or(IntegrationError::MissingPreviousPose(handle))?;
        let old_shape = scene
            .previous
            .shapes
            .get(&raw)
            .ok_or(IntegrationError::MissingPreviousPose(handle))?;
        if !std::sync::Arc::ptr_eq(&old_shape.0, &collider.shared_shape().0) {
            return Err(IntegrationError::InvalidScene(
                "collider shape changed during a continuous substep",
            ));
        }
        let current = *collider.position();
        let bound = collider.shape().compute_local_aabb();
        let radius = bound.mins.abs().max(bound.maxs.abs()).length();
        let mut result = Self {
            start: previous,
            end: current,
            body_start: previous,
            local: Pose::IDENTITY,
            local_com: Vec3::ZERO,
            com_start: previous.translation,
            com_delta: Vec3::ZERO,
            angular: Vec3::ZERO,
            angular_speed_bound: 0.0,
            radius,
            physical: false,
        };
        let body = collider.parent().and_then(|h| scene.query.bodies.get(h));
        if body.is_some_and(|b| b.is_dynamic()) {
            return Err(IntegrationError::UnsupportedCollision {
                collider: handle,
                reason: "continuous dynamic surface collision requires two-way coupling",
            });
        }
        if let Some(body) = body.filter(|b| b.is_kinematic()) {
            if collider.shape().as_halfspace().is_some() {
                return Err(IntegrationError::UnsupportedCollision {
                    collider: handle,
                    reason: "continuous kinematic half-space is unsupported",
                });
            }
            let old = *scene
                .previous
                .body_pose(collider.parent().unwrap())
                .ok_or(IntegrationError::MissingPreviousPose(handle))?;
            let now = *body.position();
            let local = *collider.position_wrt_parent().unwrap();
            let local_com = body.local_center_of_mass();
            let com_start = old.transform_point(local_com);
            let com_end = now.transform_point(local_com);
            // Rapier writes damping back after integrating poses. Undo that
            // final velocity scaling when validating the motion just taken.
            let commanded = body.angvel() * (scene.h * (1.0 + scene.h * body.angular_damping()));
            let commanded_angle = commanded.length();
            let angular = rotation_vector(now.rotation, old.rotation);
            let actual_angle = angular.length();
            let velocity_based = body.body_type() == RigidBodyType::KinematicVelocityBased;
            if velocity_based && commanded_angle >= Real::acos(-1.0) {
                return Err(ClothError::UnresolvedContinuousCollision(
                    "velocity-based angular command reaches a half-turn; reduce the external substep",
                ).into());
            }
            let scale = old
                .translation
                .length()
                .max(now.translation.length())
                .max(local_com.length())
                .max(radius)
                .max(0.001);
            let length_tolerance = scale * Real::EPSILON * 128.0;
            let angle_tolerance = (1.0 + commanded_angle) * Real::EPSILON * 128.0;
            let minimum_angle = if velocity_based {
                // One normalized Euler increment is the smallest total angle;
                // additional Rapier iterations approach the exponential angle.
                2.0 * (commanded_angle * 0.5).atan()
            } else {
                commanded_angle
            };
            let axis_error = if commanded_angle > 0.0 {
                (angular - commanded * (actual_angle / commanded_angle)).length()
            } else {
                actual_angle
            };
            let linear_displacement =
                body.linvel() * (scene.h * (1.0 + scene.h * body.linear_damping()));
            let old_collider = old * local;
            if !angular.is_finite()
                || !commanded.is_finite()
                || !com_start.is_finite()
                || !com_end.is_finite()
                || actual_angle + angle_tolerance < minimum_angle
                || actual_angle > commanded_angle + angle_tolerance
                || axis_error > angle_tolerance
                || (com_end - com_start - linear_displacement).length() > length_tolerance
                || old_collider.translation.distance(previous.translation) > length_tolerance
                || rotation_error(old_collider.rotation, previous.rotation) > angle_tolerance
            {
                return Err(IntegrationError::InvalidScene(
                    "kinematic poses/velocity/local offset do not match the continuous motion model",
                ));
            }
            result.body_start = old;
            result.local = local;
            result.local_com = local_com;
            result.com_start = com_start;
            result.com_delta = com_end - com_start;
            result.angular = angular;
            result.angular_speed_bound =
                commanded_angle.max(actual_angle) * (1.0 + 128.0 * Real::EPSILON);
            result.radius = radius + (local.translation - local_com).length();
            result.physical = matches!(stage, ContactStage::Prediction | ContactStage::Final);
        } else if current != previous {
            return Err(IntegrationError::InvalidScene(
                "fixed collider moved during a continuous substep",
            ));
        }
        if !result.physical {
            let pose = if stage == ContactStage::Stabilization {
                previous
            } else {
                current
            };
            result.start = pose;
            result.end = pose;
            result.com_start = pose.translation;
            result.com_delta = Vec3::ZERO;
            result.angular = Vec3::ZERO;
            result.angular_speed_bound = 0.0;
            result.radius = radius;
        }
        Ok(result)
    }
    fn at(&self, fraction: Real) -> Pose {
        if fraction == 0.0 || !self.physical {
            return self.start;
        }
        if fraction == 1.0 {
            return self.end;
        }
        let rotation =
            Rotation::from_scaled_axis(self.angular * fraction) * self.body_start.rotation;
        let com = self.com_start + self.com_delta * fraction;
        Pose::from_parts(com - rotation * self.local_com, rotation) * self.local
    }
    fn angular_bound(&self) -> Real {
        self.angular_speed_bound
    }
    fn speed_bound(&self, start: [Vec3; 3], end: [Vec3; 3]) -> Real {
        let translation = start
            .iter()
            .zip(end)
            .map(|(&a, b)| (b - a - self.com_delta).length())
            .fold(0.0, Real::max);
        (translation + self.angular_bound() * self.radius) * (1.0 + 128.0 * Real::EPSILON)
    }
    fn swept_bounds(&self) -> Aabb {
        let a = self.com_start;
        let b = self.com_start + self.com_delta;
        let pad = Vec3::splat(
            self.radius + (self.radius + a.length() + b.length()) * Real::EPSILON * 128.0,
        );
        Aabb::new(a.min(b) - pad, a.max(b) + pad)
    }
}

fn rotation_error(a: Rotation, b: Rotation) -> Real {
    rotation_vector(a, b).length()
}
fn rotation_vector(a: Rotation, b: Rotation) -> Vec3 {
    let mut delta = a * b.inverse();
    if delta.w < 0.0 {
        delta = -delta;
    }
    delta.to_scaled_axis()
}

struct PrimitiveSweep<'a> {
    shape: &'a dyn Shape,
    motion: RigidMotion,
    start: [Vec3; 3],
    end: [Vec3; 3],
    minimum: Real,
}
impl PrimitiveSweep<'_> {
    fn positions(&self, fraction: Real) -> [Vec3; 3] {
        std::array::from_fn(|i| self.start[i] + (self.end[i] - self.start[i]) * fraction)
    }
    fn rounding(&self) -> Real {
        let world_extent = self
            .start
            .iter()
            .chain(&self.end)
            .map(|p| p.length())
            .chain([
                self.motion.start.translation.length(),
                self.motion.end.translation.length(),
            ])
            .fold(0.0, Real::max);
        let local_extent = self
            .start
            .iter()
            .map(|p| p.distance(self.motion.start.translation))
            .chain(
                self.end
                    .iter()
                    .map(|p| p.distance(self.motion.end.translation)),
            )
            .fold(0.0, Real::max);
        (world_extent + local_extent + self.motion.radius * (1.0 + self.motion.angular_bound()))
            * Real::EPSILON
            * 64.0
    }
    fn direction(&self, positions: [Vec3; 3], pose: Pose) -> Result<Vec3, ClothError> {
        let points = positions.map(|p| pose.inverse_transform_point(p));
        let triangle = Triangle::new(points[0], points[1], points[2]);
        let closest =
            closest_triangle(Vec3::ZERO, points).ok_or(ClothError::DegenerateConstraint)?;
        let local = if self.shape.as_ball().is_some() {
            closest.point.normalize_or_zero()
        } else {
            let prediction = points.iter().map(|p| p.length()).fold(0.0, Real::max)
                + self.motion.radius * 2.0
                + self.minimum;
            contact(
                &Pose::IDENTITY,
                self.shape,
                &Pose::IDENTITY,
                &triangle,
                prediction,
            )
            .map_err(|_| {
                ClothError::UnresolvedContinuousCollision("unsupported primitive distance query")
            })?
            .ok_or(ClothError::UnresolvedContinuousCollision(
                "missing separating direction",
            ))?
            .normal1
        };
        let mut normal = (pose.rotation * local).normalize_or_zero();
        // A GJK witness difference loses angular accuracy when a large face is
        // only micrometres from the primitive. The triangle plane and box SAT
        // axes remain useful separating directions in that regime. Every
        // candidate is certified by a support projection, never by its origin.
        let face = (points[1] - points[0]).cross(points[2] - points[0]);
        let mut best = self.projected(positions, pose, normal);
        let mut consider = |axis: Vec3| {
            let candidate = (pose.rotation * axis).normalize_or_zero();
            if candidate.is_finite() && candidate.length_squared() > 0.5 {
                for candidate in [candidate, -candidate] {
                    let gap = self.projected(positions, pose, candidate);
                    if gap > best {
                        best = gap;
                        normal = candidate;
                    }
                }
            }
        };
        consider(face);
        if self.shape.as_cuboid().is_some() {
            for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
                consider(axis);
                for edge in [
                    points[1] - points[0],
                    points[2] - points[1],
                    points[0] - points[2],
                ] {
                    consider(edge.cross(axis));
                }
            }
        }
        if !normal.is_finite() || normal.length_squared() < 0.5 {
            return Err(ClothError::UnresolvedContinuousCollision(
                "unresolved primitive direction",
            ));
        }
        Ok(normal)
    }
    /// A separating projection is a lower distance bound even if GJK's
    /// approximate closest normal is imperfect. Never advance using a possibly
    /// overestimated unsigned GJK distance alone.
    fn projected(&self, positions: [Vec3; 3], pose: Pose, normal: Vec3) -> Real {
        let local_normal = pose.rotation.inverse() * normal;
        let support = self
            .shape
            .as_support_map()
            .unwrap()
            .local_support_point(local_normal);
        positions
            .map(|p| local_normal.dot(pose.inverse_transform_point(p) - support))
            .into_iter()
            .fold(Real::INFINITY, Real::min)
    }
    fn advance(
        &self,
        work: &mut CollisionWork,
        limits: CollisionLimits,
    ) -> Result<CcdResult, ClothError> {
        let speed = self.motion.speed_bound(self.start, self.end);
        let rounding = self.rounding();
        if !speed.is_finite()
            || !rounding.is_finite()
            || !self.minimum.is_finite()
            || self.minimum <= 0.0
        {
            return Err(ClothError::UnresolvedContinuousCollision(
                "non-finite rigid motion bound",
            ));
        }
        let mut fraction = 0.0;
        let mut reserve = 0.0;
        // A direction separating the ending triangle may certify the whole
        // remaining interval even when the current closest direction cannot.
        // Without this test the initial-clearance reserve can repeatedly seed
        // an already satisfied contact and stall prediction forever.
        work.charge(CollisionBudgetKind::CcdChecks, 1, limits)?;
        let ending_pose = self.motion.at(1.0);
        let ending_direction = self.direction(self.end, ending_pose).ok();
        for iteration in 0..256 {
            work.charge(CollisionBudgetKind::CcdChecks, 1, limits)?;
            let positions = self.positions(fraction);
            let pose = self.motion.at(fraction);
            let normal = self.direction(positions, pose)?;
            let distance = self.projected(positions, pose, normal) - rounding;
            let clearance = distance - self.minimum;
            if !clearance.is_finite() || clearance <= 0.0 {
                return Err(ClothError::UnresolvedContinuousCollision(
                    "insufficient rigid separation or numerical clearance",
                ));
            }
            let remaining = 1.0 - fraction;
            // A rigid material point's second derivative is bounded by r*w^2
            // under the declared constant-angular interpolation.
            // Its deviation from the endpoint chord is therefore <= r*w^2/8.
            // This also covers collider offsets around a moving body COM.
            let curvature =
                self.motion.radius * (self.motion.angular_bound() * remaining).powi(2) * 0.125;
            let end_distance = self.projected(self.end, self.motion.at(1.0), normal) - rounding;
            if distance.min(end_distance) - curvature > self.minimum {
                return Ok(CcdResult::Clear);
            }
            if let Some(normal) = ending_direction {
                let here = self.projected(positions, pose, normal) - rounding;
                let end = self.projected(self.end, ending_pose, normal) - rounding;
                if here.min(end) - curvature > self.minimum {
                    return Ok(CcdResult::Clear);
                }
            }
            if iteration == 0 {
                reserve = clearance * 0.1;
            }
            if fraction > 0.0 && clearance <= reserve {
                return Ok(CcdResult::Limited { fraction });
            }
            if speed == 0.0 || clearance > speed * remaining {
                return Ok(CcdResult::Clear);
            }
            let next = fraction + 0.8 * clearance / speed;
            if !next.is_finite() || next <= fraction {
                return Err(ClothError::UnresolvedContinuousCollision(
                    "rigid advancement made no progress",
                ));
            }
            fraction = next.min(1.0);
        }
        Err(ClothError::UnresolvedContinuousCollision(
            "rigid advancement convergence limit",
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{SceneSnapshot, WorldId};

    #[test]
    fn interpolation_matches_rapier_solver_endpoints_and_offset_bounds() {
        let h = 1.0 / 240.0;
        for (iterations, angle) in [1, 4, 12]
            .into_iter()
            .flat_map(|n| [0.0, 0.2, 1.0, 2.9].map(|a| (n, a)))
        {
            let mut rigid = PhysicsWorld::new();
            rigid.integration_parameters.dt = h;
            rigid.integration_parameters.num_solver_iterations = iterations;
            let commanded_angular = Vec3::new(1.0, 2.0, -1.0).normalize() * (angle / h);
            let commanded_linear = Vec3::new(0.13, -0.05, 0.11);
            let handle = rigid.bodies.insert(
                RigidBodyBuilder::kinematic_velocity_based()
                    .translation(Vec3::new(0.21, -0.13, 0.18))
                    .rotation(Vec3::new(0.1, -0.2, 0.15))
                    .linvel(commanded_linear)
                    .angvel(commanded_angular)
                    .linear_damping(0.5)
                    .angular_damping(0.3)
                    .additional_mass_properties(MassProperties::new(Vec3::ZERO, 1.0, Vec3::ONE)),
            );
            let collider = rigid.colliders.insert_with_parent(
                ColliderBuilder::ball(0.01)
                    .translation(Vec3::new(0.12, 0.07, 0.1))
                    .density(0.0),
                handle,
                &mut rigid.bodies,
            );
            let before = SceneSnapshot::capture(WorldId::new(), 0, &rigid.bodies, &rigid.colliders);
            rigid.step();
            let query = rigid.broad_phase.as_query_pipeline(
                rigid.narrow_phase.query_dispatcher(),
                &rigid.bodies,
                &rigid.colliders,
                QueryFilter::default(),
            );
            let scene = RapierScene::new(query, &before, h, Vec3::ZERO);
            let motion = RigidMotion::new(&scene, collider, ContactStage::Prediction).unwrap();
            let body = &rigid.bodies[handle];
            let local = *rigid.colliders[collider].position_wrt_parent().unwrap();
            let bounds = motion.swept_bounds();
            let tolerance = 256.0 * Real::EPSILON;
            assert_eq!(body.local_center_of_mass(), Vec3::ZERO);
            // Independently replay the normalized quaternion Euler increments
            // used by Rapier's serial velocity solver, including its iteration
            // count. Exact exponential integration is a different algorithm.
            let mut expected_body = *before.body_pose(handle).unwrap();
            let half_increment = commanded_angular * (h / iterations as Real * 0.5);
            let increment =
                Rotation::from_xyzw(half_increment.x, half_increment.y, half_increment.z, 1.0);
            for i in 0..=iterations {
                let t = i as Real / iterations as Real;
                let expected = expected_body * local;
                let actual = motion.at(t);
                assert!(
                    actual.translation.distance(expected.translation) <= tolerance,
                    "iterations={iterations}, angle={angle}, t={t}: {actual:?} vs {expected:?}"
                );
                for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
                    assert!(
                        (actual.rotation * axis).distance(expected.rotation * axis) <= tolerance
                    );
                }
                expected_body.rotation = (increment * expected_body.rotation).normalize();
                expected_body.translation += commanded_linear * (h / iterations as Real);
            }
            for i in 0..=256 {
                let t = i as Real / 256.0;
                let actual = motion.at(t);
                for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
                    for p in [
                        actual.translation + axis * 0.01,
                        actual.translation - axis * 0.01,
                    ] {
                        assert!(p.cmpge(bounds.mins).all() && p.cmple(bounds.maxs).all());
                    }
                }
            }
            assert_eq!(motion.at(0.0), before.colliders[&collider.into_raw_parts()]);
            assert_eq!(motion.at(1.0), *rigid.colliders[collider].position());
            for stage in [ContactStage::Stabilization, ContactStage::Iteration] {
                let frozen = RigidMotion::new(&scene, collider, stage).unwrap();
                let expected = if stage == ContactStage::Stabilization {
                    motion.start
                } else {
                    motion.end
                };
                for t in [0.0, 0.25, 0.5, 1.0] {
                    assert_eq!(frozen.at(t), expected);
                }
            }
        }
    }

    #[test]
    fn offset_arc_stops_before_analytical_plane_impact() {
        let h = 1.0 / 240.0;
        let commanded_angle = 0.8;
        let mut rigid = PhysicsWorld::new();
        rigid.integration_parameters.dt = h;
        let handle = rigid.bodies.insert(
            RigidBodyBuilder::kinematic_velocity_based()
                .angvel(Vec3::Z * (commanded_angle / h))
                .additional_mass_properties(MassProperties::new(Vec3::ZERO, 1.0, Vec3::ONE)),
        );
        let collider = rigid.colliders.insert_with_parent(
            ColliderBuilder::ball(0.01)
                .translation(Vec3::X * 0.15)
                .density(0.0),
            handle,
            &mut rigid.bodies,
        );
        let before = SceneSnapshot::capture(WorldId::new(), 0, &rigid.bodies, &rigid.colliders);
        rigid.step();
        let query = rigid.broad_phase.as_query_pipeline(
            rigid.narrow_phase.query_dispatcher(),
            &rigid.bodies,
            &rigid.colliders,
            QueryFilter::default(),
        );
        let scene = RapierScene::new(query, &before, h, Vec3::ZERO);
        let motion = RigidMotion::new(&scene, collider, ContactStage::Prediction).unwrap();
        let n = rigid.integration_parameters.num_solver_iterations as Real;
        let actual_angle = 2.0 * n * (commanded_angle / (2.0 * n)).atan();
        let points = [
            Vec3::new(0.08, 0.05, -0.1),
            Vec3::new(0.22, 0.05, -0.1),
            Vec3::new(0.15, 0.05, 0.1),
        ];
        let sweep = PrimitiveSweep {
            shape: rigid.colliders[collider].shape(),
            motion,
            start: points,
            end: points,
            minimum: 0.00045,
        };
        let result = sweep
            .advance(&mut CollisionWork::default(), CollisionLimits::default())
            .unwrap();
        let earliest_plane_impact = ((0.05 - 0.01 - 0.00045) as Real / 0.15).asin() / actual_angle;
        assert!(matches!(result, CcdResult::Limited { .. }), "{result:?}");
        assert!(
            result.fraction() > 0.0 && result.fraction() < earliest_plane_impact,
            "{result:?}"
        );
        // The sphere is below the whole cloth plane throughout this prefix.
        // The offset center follows an arc; a center-endpoint chord is different.
        for i in 0..=256 {
            let t = result.fraction() * i as Real / 256.0;
            let independent_gap = 0.05 - 0.15 * (actual_angle * t).sin() - 0.01;
            assert!(independent_gap > 0.00045);
        }
        let center_crossing = (0.05 as Real / 0.15).asin() / actual_angle;
        assert!((motion.at(center_crossing).translation.y - 0.05).abs() < 256.0 * Real::EPSILON);
    }

    #[test]
    fn primitive_sweeps_match_transformed_analytical_crossings_and_sliding() {
        for shape in [
            SharedShape::ball(0.02),
            SharedShape::cuboid(0.02, 0.02, 0.02),
            SharedShape::capsule_x(0.015, 0.02),
        ] {
            for seed in 0..32 {
                let s = seed as Real;
                let pose = Pose::from_parts(
                    Vec3::new((s * 0.71).sin(), (s * 0.39).cos(), (s * 1.13).sin()) * 0.2,
                    Rotation::from_scaled_axis(Vec3::new(
                        (s * 0.31).sin(),
                        (s * 0.57).cos(),
                        (s * 0.89).sin(),
                    )),
                );
                let mut rigid = PhysicsWorld::new();
                let collider = rigid
                    .colliders
                    .insert(ColliderBuilder::new(shape.clone()).position(pose));
                let before =
                    SceneSnapshot::capture(WorldId::new(), 0, &rigid.bodies, &rigid.colliders);
                rigid.step();
                let query = rigid.broad_phase.as_query_pipeline(
                    rigid.narrow_phase.query_dispatcher(),
                    &rigid.bodies,
                    &rigid.colliders,
                    QueryFilter::default(),
                );
                let scene = RapierScene::new(query, &before, 1.0 / 240.0, Vec3::ZERO);
                let motion = RigidMotion::new(&scene, collider, ContactStage::Prediction).unwrap();
                for crossing in [false, true] {
                    let height = if crossing { 0.04 } else { 0.0205 };
                    let start = [
                        Vec3::new(-0.4, height, -0.3),
                        Vec3::new(0.4, height, -0.3),
                        Vec3::new(0.0, height, 0.4),
                    ]
                    .map(|p| pose.transform_point(p));
                    let displacement = pose.rotation
                        * if crossing {
                            -Vec3::Y * 0.08
                        } else {
                            Vec3::X * 0.05
                        };
                    let sweep = PrimitiveSweep {
                        shape: rigid.colliders[collider].shape(),
                        motion,
                        start,
                        end: start.map(|p| p + displacement),
                        minimum: 0.00045,
                    };
                    let result = sweep
                        .advance(&mut CollisionWork::default(), CollisionLimits::default())
                        .unwrap();
                    if crossing {
                        assert!(
                            matches!(result, CcdResult::Limited { .. }),
                            "seed {seed}: {result:?}"
                        );
                        assert!(
                            result.fraction() > 0.0
                                && result.fraction() < (0.04 - 0.02 - 0.00045) / 0.08
                        );
                        for i in 0..=32 {
                            let t = result.fraction() * i as Real / 32.0;
                            // A common rigid transform preserves this exact
                            // plane-to-primitive distance throughout the sweep.
                            assert!(0.04 - 0.08 * t - 0.02 > 0.00045);
                        }
                    } else {
                        assert_eq!(result, CcdResult::Clear, "seed {seed}");
                    }
                }
            }
        }
    }
}
