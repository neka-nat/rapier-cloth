use super::*;
use crate::{
    ClothContactSettings, CollisionBudgetKind, CollisionWork, SurfaceContact, SurfaceContactKey,
    SurfaceFeature,
    collision::geometry::{SurfaceWitness, closest_triangle},
    rapier::parry::{
        query::{DefaultQueryDispatcher, PersistentQueryDispatcher},
        shape::Triangle,
    },
};

impl RapierContacts<'_, '_> {
    pub(super) fn generate_surface(
        &mut self,
        _previous: &[Vec3],
        positions: &[Vec3],
        stage: ContactStage,
        out: &mut Vec<SurfaceContact>,
        work: &mut CollisionWork,
    ) -> Result<(), IntegrationError> {
        let Some(config) = self.surface_settings else {
            return Ok(());
        };
        let mesh = self.surface_mesh.as_ref().unwrap();
        if positions.len() != mesh.rest_positions().len() {
            return Err(ClothError::InvalidSurfaceContact("surface vertex count").into());
        }
        let separation = config.thickness * 0.5;
        let activation = separation + config.activation_margin;
        let cloth_bounds = bounds(positions.iter().copied(), activation);
        self.candidate_queries += 1;
        // Previous-pose stabilization must find an obstacle even if its current
        // broad-phase bounds have moved away. Test the declared pose explicitly.
        for (handle, collider) in self.scene.query.colliders.iter() {
            if !self
                .scene
                .query
                .filter
                .test(self.scene.query.bodies, handle, collider)
            {
                continue;
            }
            if collider.is_sensor()
                || !collider.is_enabled()
                || self.settings.excluded_colliders.contains(&handle)
            {
                self.ignored.insert(handle.into_raw_parts());
                continue;
            }
            let body = collider
                .parent()
                .and_then(|h| self.scene.query.bodies.get(h));
            let pose =
                if stage == ContactStage::Stabilization && body.is_some_and(|b| b.is_kinematic()) {
                    self.scene
                        .previous
                        .colliders
                        .get(&handle.into_raw_parts())
                        .ok_or(IntegrationError::MissingPreviousPose(handle))?
                } else {
                    collider.position()
                };
            let shape = collider.shape();
            let obstacle_bounds = shape.compute_aabb(pose);
            if !overlaps(&cloth_bounds, &obstacle_bounds) {
                continue;
            }
            if body.is_some_and(|b| b.is_dynamic()) {
                return Err(IntegrationError::UnsupportedCollision {
                    collider: handle,
                    reason: "dynamic surface collision requires two-way coupling",
                });
            }
            if (shape.as_halfspace().is_some() && body.is_some_and(|b| b.is_kinematic()))
                || (shape.as_ball().is_none()
                    && shape.as_cuboid().is_none()
                    && shape.as_capsule().is_none()
                    && shape.as_halfspace().is_none())
            {
                return Err(IntegrationError::UnsupportedCollision {
                    collider: handle,
                    reason: "surface collision supports sphere/box/capsule/fixed-halfspace",
                });
            }
            let raw = handle.into_raw_parts();
            let external = (u64::from(raw.1) << 32) | u64::from(raw.0);
            let friction = |mu: Real| {
                self.settings
                    .friction_override
                    .unwrap_or((mu + collider.friction()) * 0.5)
            };
            let mut contact = SurfaceContact {
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
                // Signed distance to a plane is affine over a triangle, so its
                // three vertex inequalities cover the complete triangle.
                let normal = pose.rotation * plane.normal;
                for (i, &p) in positions.iter().enumerate() {
                    if self.excluded_pairs.contains(&(i as u32, raw)) {
                        continue;
                    }
                    work.charge(CollisionBudgetKind::CandidatePairs, 1, config.limits)?;
                    self.pair_queries += 1;
                    let distance = normal.dot(p - pose.translation);
                    if distance > activation {
                        continue;
                    }
                    contact.key.features[0] = SurfaceFeature::Vertex(i as u32);
                    contact.particles = [i as u32, 0, 0, 0];
                    contact.weights = [1.0, 0.0, 0.0, 0.0];
                    contact.normal = normal;
                    contact.offset = p - normal * distance;
                    check_distance(distance, separation, stage, handle, contact.key.features[0])?;
                    push_contact(contact, positions.len(), config, self.limit, out, work)?;
                }
                continue;
            }
            for (face, &indices) in mesh.triangles().iter().enumerate() {
                // Excluding a selected patch must not hide its unselected
                // neighboring vertices or an entire cloth component.
                if indices
                    .iter()
                    .all(|i| self.excluded_pairs.contains(&(*i, raw)))
                {
                    continue;
                }
                let points = indices.map(|i| positions[i as usize]);
                if !overlaps(&bounds(points, activation), &obstacle_bounds) {
                    continue;
                }
                work.charge(CollisionBudgetKind::CandidatePairs, 1, config.limits)?;
                self.pair_queries += 1;
                // Work in a translated frame for small contact gaps. The
                // triangle changes on each query; never reuse another triangle's
                // manifold or its contact-feature cache.
                let origin = points[0];
                let triangle = Triangle::new(Vec3::ZERO, points[1] - origin, points[2] - origin);
                if closest_triangle(Vec3::ZERO, [triangle.a, triangle.b, triangle.c]).is_none() {
                    return Err(ClothError::DegenerateConstraint.into());
                }
                let triangle_pose = Pose::from_translation(origin);
                self.surface_manifold.clear();
                DefaultQueryDispatcher
                    .contact_manifold_convex_convex(
                        &pose.inv_mul(&triangle_pose),
                        shape,
                        &triangle,
                        None,
                        None,
                        activation,
                        &mut self.surface_manifold,
                    )
                    .map_err(|_| IntegrationError::UnsupportedCollision {
                        collider: handle,
                        reason: "Parry surface manifold query unsupported",
                    })?;
                let normal = pose.rotation * self.surface_manifold.local_n1;
                for point in &self.surface_manifold.points {
                    let witness =
                        closest_triangle(point.local_p2, [triangle.a, triangle.b, triangle.c])
                            .ok_or(ClothError::DegenerateConstraint)?;
                    let support =
                        SurfaceWitness::from_triangle(indices, face as u32, witness.barycentric)?;
                    if support
                        .particles
                        .iter()
                        .zip(support.weights)
                        .all(|(&i, w)| w == 0.0 || self.excluded_pairs.contains(&(i, raw)))
                    {
                        continue;
                    }
                    contact.key.features = [
                        support.feature,
                        SurfaceFeature::External {
                            object: external,
                            feature: point.fid1.0,
                        },
                    ];
                    contact.particles = [
                        support.particles[0],
                        support.particles[1],
                        support.particles[2],
                        0,
                    ];
                    contact.weights = [
                        support.weights[0],
                        support.weights[1],
                        support.weights[2],
                        0.0,
                    ];
                    contact.normal = normal;
                    contact.offset = pose.transform_point(point.local_p1);
                    contact.surface_velocity =
                        body.map_or(Vec3::ZERO, |b| b.velocity_at_point(contact.offset));
                    check_distance(point.dist, separation, stage, handle, support.feature)?;
                    push_contact(contact, positions.len(), config, self.limit, out, work)?;
                }
            }
        }
        deduplicate(out, config, self.limit, work)?;
        Ok(())
    }
}

fn bounds(points: impl IntoIterator<Item = Vec3>, margin: Real) -> Aabb {
    let mut lo = Vec3::splat(Real::MAX);
    let mut hi = Vec3::splat(-Real::MAX);
    for p in points {
        lo = lo.min(p);
        hi = hi.max(p);
    }
    Aabb::new(lo - Vec3::splat(margin), hi + Vec3::splat(margin))
}
fn overlaps(a: &Aabb, b: &Aabb) -> bool {
    a.mins.cmple(b.maxs).all() && b.mins.cmple(a.maxs).all()
}
fn check_distance(
    distance: Real,
    separation: Real,
    stage: ContactStage,
    collider: ColliderHandle,
    feature: SurfaceFeature,
) -> Result<(), IntegrationError> {
    if !distance.is_finite() {
        return Err(ClothError::NonFiniteState.into());
    }
    if stage == ContactStage::Stabilization && distance < -crate::core::math::LENGTH_EPSILON {
        return Err(IntegrationError::InitialRigidIntersection { collider, feature });
    }
    if stage == ContactStage::Final && distance < separation * 0.9 {
        return Err(ClothError::UnresolvedSurfaceContact.into());
    }
    Ok(())
}
fn push_contact(
    contact: SurfaceContact,
    particle_count: usize,
    config: ClothContactSettings,
    limit: usize,
    out: &mut Vec<SurfaceContact>,
    work: &mut CollisionWork,
) -> Result<(), ClothError> {
    contact.validate(particle_count)?;
    if out.len() >= config.limits.retained_contacts.min(limit).saturating_mul(2) {
        deduplicate(out, config, limit, work)?;
    }
    out.push(contact);
    Ok(())
}
fn deduplicate(
    out: &mut Vec<SurfaceContact>,
    config: ClothContactSettings,
    limit: usize,
    work: &mut CollisionWork,
) -> Result<(), ClothError> {
    out.sort_by_key(|c| c.key);
    out.dedup_by_key(|c| c.key);
    work.charge(
        CollisionBudgetKind::RetainedContacts,
        out.len(),
        config.limits,
    )?;
    if out.len() > limit {
        return Err(ClothError::ContactBudgetExceeded { limit });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Cloth, ClothMaterial, ClothMesh, GridBuilder, SceneSnapshot, WorldId};

    fn query(
        cloth: &Cloth,
        shape: SharedShape,
        pose: Pose,
        excluded_vertices: &[u32],
    ) -> Result<Vec<SurfaceContact>, IntegrationError> {
        let mut rigid = PhysicsWorld::new();
        let collider = rigid
            .colliders
            .insert(ColliderBuilder::new(shape).position(pose));
        let before = SceneSnapshot::capture(WorldId::new(), 0, &rigid.bodies, &rigid.colliders);
        rigid.step();
        let pipeline = rigid.broad_phase.as_query_pipeline(
            rigid.narrow_phase.query_dispatcher(),
            &rigid.bodies,
            &rigid.colliders,
            QueryFilter::default(),
        );
        let scene = RapierScene::new(pipeline, &before, 1.0 / 240.0, Vec3::ZERO);
        let settings = CollisionSettings::default();
        let excluded = excluded_vertices
            .iter()
            .map(|&v| (v, collider.into_raw_parts()))
            .collect();
        let mut source = RapierContacts::new(&scene, &settings, cloth.material(), 1024, &excluded)
            .with_surface(cloth);
        let mut out = vec![];
        source.generate_surface(
            cloth.positions(),
            cloth.positions(),
            ContactStage::Prediction,
            &mut out,
            &mut CollisionWork::default(),
        )?;
        Ok(out)
    }
    fn enabled(mesh: ClothMesh) -> Cloth {
        let mut cloth = Cloth::new(mesh, ClothMaterial::default()).unwrap();
        cloth
            .set_contact_settings(Some(ClothContactSettings {
                self_collision: false,
                rigid_surface_collision: true,
                ..Default::default()
            }))
            .unwrap();
        cloth
    }

    #[test]
    fn shared_edge_witness_deduplicates_and_excludes_only_its_selected_support() {
        let mesh = GridBuilder::new(2, 2)
            .size(0.1, 0.1)
            .origin(Vec3::new(-0.05, 0.0205, -0.05))
            .build()
            .unwrap();
        let mut cloth = enabled(mesh);
        let mut config = cloth.contact_settings().unwrap();
        config.limits.retained_contacts = 1;
        cloth.set_contact_settings(Some(config)).unwrap();
        let contacts = query(&cloth, SharedShape::ball(0.02), Pose::IDENTITY, &[]).unwrap();
        assert_eq!(contacts.len(), 1);
        let SurfaceFeature::Edge(edge) = contacts[0].key.features[0] else {
            panic!("expected shared edge, got {:?}", contacts[0]);
        };
        assert_eq!(
            query(&cloth, SharedShape::ball(0.02), Pose::IDENTITY, &[edge[0]])
                .unwrap()
                .len(),
            1
        );
        assert!(
            query(&cloth, SharedShape::ball(0.02), Pose::IDENTITY, &edge)
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn primitive_manifold_witnesses_follow_rotated_and_translated_triangle_interiors() {
        for kind in 0..3 {
            for trial in 0..64 {
                let rotation =
                    Rotation::from_scaled_axis(Vec3::new(0.3, 0.7, -0.2) * (trial as Real * 0.13));
                let mut pose =
                    Pose::from_translation(Vec3::new(0.17, -0.1, 0.09) * (trial as Real * 0.07));
                pose.rotation = rotation;
                let points = [
                    Vec3::new(-0.4, 0.0205, -0.3),
                    Vec3::new(0.4, 0.0205, -0.3),
                    Vec3::new(0.0, 0.0205, 0.4),
                ]
                .map(|p| pose.transform_point(p));
                let cloth = enabled(ClothMesh::new(points.to_vec(), vec![[0, 2, 1]]).unwrap());
                let shape = match kind {
                    0 => SharedShape::ball(0.02),
                    1 => SharedShape::cuboid(0.02, 0.02, 0.02),
                    _ => SharedShape::capsule_x(0.015, 0.02),
                };
                let contacts = query(&cloth, shape, pose, &[]).unwrap();
                assert!(!contacts.is_empty(), "kind={kind}, trial={trial}");
                let minimum = [1, 3, 2][kind];
                assert!(
                    contacts.len() >= minimum,
                    "kind={kind}, trial={trial}: {} manifold supports, expected at least {minimum}",
                    contacts.len()
                );
                for contact in contacts {
                    assert!(
                        contact.normal.dot(rotation * Vec3::Y) > 0.999,
                        "{contact:?}"
                    );
                    let local = pose.inverse_transform_point(contact.offset);
                    let distance = match kind {
                        0 => local.length() - 0.02,
                        1 => {
                            let q = local.abs() - Vec3::splat(0.02);
                            q.max(Vec3::ZERO).length() + q.max_element().min(0.0)
                        }
                        _ => {
                            Vec3::new((local.x.abs() - 0.015).max(0.0), local.y, local.z).length()
                                - 0.02
                        }
                    };
                    assert!(distance.abs() < 2.0e-5, "kind={kind}, distance={distance}");
                    assert_eq!(contact.key.features[0], SurfaceFeature::Face(0));
                    let mut on_cloth = Vec3::ZERO;
                    for (&i, &w) in contact.particles.iter().zip(&contact.weights) {
                        on_cloth += cloth.positions()[i as usize] * w;
                    }
                    let gap = contact.normal.dot(on_cloth - contact.offset);
                    assert!((gap - 0.0005).abs() < 2.0e-5, "kind={kind}, gap={gap}");
                }
            }
        }
    }
}
