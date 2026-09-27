use super::*;
use crate::{
    ClothContactSettings, CollisionBudgetKind, CollisionWork, SurfaceContact, SurfaceContactKey,
    SurfaceFeature,
};

/// External feature identity of a contact. A compound part keeps the feature
/// code's two header bits, then ten bits of part index and twenty of feature
/// code, so parts never alias each other; single shapes keep the code unchanged.
pub(super) fn pack_feature(part: Option<u32>, packed: u32) -> Result<u32, &'static str> {
    let Some(part) = part else {
        return Ok(packed);
    };
    let code = packed & 0x3fff_ffff;
    if part >= 1 << 10 || code >= 1 << 20 {
        return Err("compound part or feature index exceeds the contact identity range");
    }
    Ok((packed & 0xc000_0000) | (part << 20) | code)
}

impl RapierContacts<'_, '_> {
    pub(super) fn transport_anchor(
        &self,
        contact: &SurfaceContact,
        previous_point: Vec3,
    ) -> Result<Vec3, IntegrationError> {
        let object = contact
            .key
            .features
            .iter()
            .find_map(|f| match f {
                SurfaceFeature::External { object, .. } => Some(*object),
                _ => None,
            })
            .ok_or(ClothError::InvalidSurfaceContact(
                "missing external anchor identity",
            ))?;
        let raw = (object as u32, (object >> 32) as u32);
        let handle = ColliderHandle::from_raw_parts(raw.0, raw.1);
        let collider = self
            .scene
            .query
            .colliders
            .get(handle)
            .ok_or(IntegrationError::MissingPreviousPose(handle))?;
        if !collider
            .parent()
            .and_then(|h| self.scene.query.bodies.get(h))
            .is_some_and(|body| body.is_kinematic())
        {
            // Fixed geometry has no prescribed physical trajectory. Discrete
            // stabilization uses its current pose; a teleport is not frictional
            // surface velocity. Continuous mode rejects that motion separately.
            return Ok(previous_point);
        }
        let previous = self
            .scene
            .previous
            .colliders
            .get(&raw)
            .ok_or(IntegrationError::MissingPreviousPose(handle))?;
        let total: Real = contact.weights.iter().sum();
        let local = previous.rotation.inverse() * (previous_point - previous.translation * total);
        Ok(collider.position().rotation * local + collider.position().translation * total)
    }

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
        let mesh = self.surface_mesh.clone().unwrap();
        if positions.len() != mesh.rest_positions().len() {
            return Err(ClothError::InvalidSurfaceContact("surface vertex count").into());
        }
        let separation = config.thickness * 0.5;
        let activation = separation + config.activation_margin;
        // Swept witnesses describe each limited triangle at its own certified
        // stop pose. The prediction solve projects the complete inertial
        // prediction against them, so there they are the intended contacts.
        // Every other query evaluates the geometry at the queried positions;
        // a witness taken further along another triangle's motion is a stale
        // half-space there and can report penetration around a convex edge.
        if stage == ContactStage::Prediction {
            out.extend_from_slice(&self.motion_contacts);
        }
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
            if shape.as_halfspace().is_some() && body.is_some_and(|b| b.is_kinematic()) {
                return Err(IntegrationError::UnsupportedCollision {
                    collider: handle,
                    reason: "kinematic halfspaces are not supported",
                });
            }
            if let Err(reason) = super::supported_shape(shape) {
                return Err(IntegrationError::UnsupportedCollision {
                    collider: handle,
                    reason,
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
            // A compound is queried part by part at the part's world pose.
            let parts: Vec<(Pose, &dyn crate::rapier::parry::shape::Shape, Option<u32>)> =
                match shape.as_compound() {
                    Some(compound) => compound
                        .shapes()
                        .iter()
                        .enumerate()
                        .map(|(i, (local, part))| (*pose * *local, part.as_ref(), Some(i as u32)))
                        .collect(),
                    None => vec![(*pose, shape, None)],
                };
            for (part_pose, part_shape, part) in parts {
                let part_bounds = part_shape.compute_aabb(&part_pose);
                if !overlaps(&cloth_bounds, &part_bounds) {
                    continue;
                }
                let features = self.features(handle, part_shape)?;
                let emitter = super::convex::PartContacts {
                    features: &features,
                    collider: handle,
                    body,
                    pose: part_pose,
                    output_pose: part_pose,
                    particle_count: positions.len(),
                    template: contact,
                    part,
                };
                // Each cloth vertex meets the whole part once per query, not
                // once per incident triangle.
                self.mark_serial = self.mark_serial.wrapping_add(1);
                let serial = self.mark_serial;
                let mut marks = std::mem::take(&mut self.vertex_marks);
                marks.resize(positions.len(), serial.wrapping_sub(1));
                let excluded = self.excluded_pairs;
                let limit = self.limit;
                let mut result = Ok(());
                for (face, &indices) in mesh.triangles().iter().enumerate() {
                    // Excluding a selected patch must not hide its unselected
                    // neighboring vertices or an entire cloth component.
                    if indices.iter().all(|i| excluded.contains(&(*i, raw))) {
                        continue;
                    }
                    let points = indices.map(|i| positions[i as usize]);
                    if !overlaps(&bounds(points, activation), &part_bounds) {
                        continue;
                    }
                    if let Err(e) =
                        work.charge(CollisionBudgetKind::CandidatePairs, 1, config.limits)
                    {
                        result = Err(e.into());
                        break;
                    }
                    self.pair_queries += 1;
                    if let Err(e) = emitter.generate(
                        indices,
                        face as u32,
                        points,
                        activation,
                        Some(stage),
                        config,
                        limit,
                        excluded,
                        out,
                        work,
                        |vertex| {
                            let mark = &mut marks[vertex as usize];
                            if *mark == serial {
                                true
                            } else {
                                *mark = serial;
                                false
                            }
                        },
                    ) {
                        result = Err(e);
                        break;
                    }
                }
                self.vertex_marks = marks;
                result?;
            }
        }
        deduplicate(out, config, self.limit, work)?;
        Ok(())
    }
}

pub(super) fn bounds(points: impl IntoIterator<Item = Vec3>, margin: Real) -> Aabb {
    let mut lo = Vec3::splat(Real::MAX);
    let mut hi = Vec3::splat(-Real::MAX);
    for p in points {
        lo = lo.min(p);
        hi = hi.max(p);
    }
    Aabb::new(lo - Vec3::splat(margin), hi + Vec3::splat(margin))
}
pub(super) fn overlaps(a: &Aabb, b: &Aabb) -> bool {
    a.mins.cmple(b.maxs).all() && b.mins.cmple(a.maxs).all()
}
pub(super) fn check_distance(
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
pub(super) fn push_contact(
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
pub(super) fn deduplicate(
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

    fn cube_hull(half: Real) -> SharedShape {
        let corners: Vec<Vec3> = (0..8)
            .map(|i| {
                Vec3::new(
                    if i & 1 == 0 { -half } else { half },
                    if i & 2 == 0 { -half } else { half },
                    if i & 4 == 0 { -half } else { half },
                )
            })
            .collect();
        SharedShape::convex_hull(&corners).unwrap()
    }
    fn box_distance(local: Vec3, half: Real) -> Real {
        let q = local.abs() - Vec3::splat(half);
        q.max(Vec3::ZERO).length() + q.max_element().min(0.0)
    }

    #[test]
    fn primitive_manifold_witnesses_follow_rotated_and_translated_triangle_interiors() {
        for kind in 0..5 {
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
                    2 => SharedShape::capsule_x(0.015, 0.02),
                    3 => cube_hull(0.02),
                    _ => SharedShape::compound(vec![
                        (Pose::from_translation(Vec3::X * 0.05), cube_hull(0.02)),
                        (Pose::from_translation(-Vec3::X * 0.05), cube_hull(0.02)),
                    ]),
                };
                let contacts = query(&cloth, shape, pose, &[]).unwrap();
                assert!(!contacts.is_empty(), "kind={kind}, trial={trial}");
                if kind == 4 {
                    // Both parts touch the triangle and keep distinct identities.
                    let parts: std::collections::BTreeSet<u32> = contacts
                        .iter()
                        .map(|c| match c.key.features[1] {
                            SurfaceFeature::External { feature, .. } => (feature >> 20) & 0x3ff,
                            _ => unreachable!(),
                        })
                        .collect();
                    assert_eq!(parts, [0, 1].into_iter().collect(), "trial={trial}");
                }
                let minimum = [1, 3, 2, 1, 2][kind];
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
                        1 | 3 => box_distance(local, 0.02),
                        2 => {
                            Vec3::new((local.x.abs() - 0.015).max(0.0), local.y, local.z).length()
                                - 0.02
                        }
                        _ => box_distance(local - Vec3::X * 0.05, 0.02)
                            .min(box_distance(local + Vec3::X * 0.05, 0.02)),
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
