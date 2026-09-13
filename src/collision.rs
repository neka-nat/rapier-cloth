use crate::rapier::{
    parry::{
        query::{ShapeCastOptions, cast_shapes, contact},
        shape::Ball,
    },
    prelude::*,
};
use crate::{
    ClothError, ClothMaterial, CollisionSettings, Contact, ContactKey, ContactSource, ContactStage,
    IntegrationError, RapierScene, Real, Vec3,
};
use std::{
    collections::{BTreeMap, BTreeSet},
    time::Instant,
};

pub(crate) struct RapierContacts<'a, 'b> {
    scene: &'a RapierScene<'b>,
    settings: &'a CollisionSettings,
    material: ClothMaterial,
    limit: usize,
    excluded_pairs: &'a BTreeSet<(u32, (u32, u32))>,
    sweeps: BTreeMap<ContactKey, Contact>,
    pub error: Option<IntegrationError>,
    pub ignored: BTreeSet<(u32, u32)>,
    pub candidate_queries: usize,
    pub pair_queries: usize,
    pub query_time_seconds: f64,
}
impl<'a, 'b> RapierContacts<'a, 'b> {
    pub fn new(
        scene: &'a RapierScene<'b>,
        settings: &'a CollisionSettings,
        material: ClothMaterial,
        limit: usize,
        excluded_pairs: &'a BTreeSet<(u32, (u32, u32))>,
    ) -> Self {
        Self {
            scene,
            settings,
            material,
            limit,
            excluded_pairs,
            sweeps: BTreeMap::new(),
            error: None,
            ignored: BTreeSet::new(),
            candidate_queries: 0,
            pair_queries: 0,
            query_time_seconds: 0.0,
        }
    }
    fn generate(
        &mut self,
        previous: &[Vec3],
        positions: &[Vec3],
        radius: Real,
        stage: ContactStage,
        out: &mut Vec<Contact>,
    ) -> Result<(), IntegrationError> {
        let mut mins = Vec3::splat(Real::MAX);
        let mut maxs = Vec3::splat(-Real::MAX);
        for p in previous.iter().chain(positions) {
            mins = mins.min(*p);
            maxs = maxs.max(*p);
        }
        let margin = radius * 0.05;
        let pad = Vec3::splat(radius + margin);
        let aabb = Aabb::new(mins - pad, maxs + pad);
        self.candidate_queries += 1;
        let mut candidates: Vec<_> = self
            .scene
            .query
            .intersect_aabb_conservative(aabb)
            .map(|(h, _)| h)
            .collect();
        candidates.sort_by_key(|h| h.into_raw_parts());
        let ball = Ball::new(radius);
        for handle in candidates {
            let c = &self.scene.query.colliders[handle];
            if c.is_sensor()
                || !c.is_enabled()
                || self.settings.excluded_colliders.contains(&handle)
            {
                self.ignored.insert(handle.into_raw_parts());
                continue;
            }
            let body = c.parent().and_then(|h| self.scene.query.bodies.get(h));
            if body.is_some_and(|b| b.is_dynamic()) {
                return Err(IntegrationError::UnsupportedCollision {
                    collider: handle,
                    reason: "dynamic body: one-way MVP accepts fixed or kinematic bodies",
                });
            }
            let shape = c.shape();
            if shape.as_halfspace().is_some() && body.is_some_and(|b| b.is_kinematic()) {
                return Err(IntegrationError::UnsupportedCollision {
                    collider: handle,
                    reason: "kinematic halfspaces are not supported",
                });
            }
            if shape.as_ball().is_none()
                && shape.as_cuboid().is_none()
                && shape.as_capsule().is_none()
                && shape.as_halfspace().is_none()
            {
                return Err(IntegrationError::UnsupportedCollision {
                    collider: handle,
                    reason: "shape outside sphere/box/capsule/fixed-halfspace support",
                });
            }
            let (index, generation) = handle.into_raw_parts();
            let external = ((generation as u64) << 32) | index as u64;
            for (i, &p) in positions.iter().enumerate() {
                if self
                    .excluded_pairs
                    .contains(&(i as u32, handle.into_raw_parts()))
                {
                    continue;
                }
                let key = ContactKey {
                    particle: i as u32,
                    external,
                    feature: 0,
                };
                let friction = self
                    .settings
                    .friction_override
                    .unwrap_or((self.material.friction + c.friction()) * 0.5);
                if !friction.is_finite() || friction < 0.0 {
                    return Err(ClothError::InvalidParameter("collider friction").into());
                }
                self.pair_queries += 1;
                // Only pre-existing overlap is stabilization. A kinematic body's
                // displacement this step must contribute physical contact impulse.
                let contact_pose = if stage == ContactStage::Stabilization
                    && body.is_some_and(|b| b.is_kinematic())
                {
                    self.scene
                        .previous
                        .colliders
                        .get(&handle.into_raw_parts())
                        .ok_or(IntegrationError::MissingPreviousPose(handle))?
                } else {
                    c.position()
                };
                let current = contact(
                    contact_pose,
                    shape,
                    &Pose::from_translation(p),
                    &ball,
                    margin,
                )
                .map_err(|_| IntegrationError::UnsupportedCollision {
                    collider: handle,
                    reason: "Parry contact query unsupported",
                })?;
                let mut geometry = current.map(|ct| Contact {
                    key,
                    normal: ct.normal1,
                    point: ct.point1,
                    surface_velocity: body.map_or(Vec3::ZERO, |b| b.velocity_at_point(ct.point1)),
                    friction,
                });
                if self.settings.static_sweep
                    && stage == ContactStage::Prediction
                    && !body.is_some_and(|b| b.is_kinematic())
                {
                    let movement = p - previous[i];
                    if movement.length_squared() > Real::MIN_POSITIVE {
                        self.pair_queries += 1;
                        // Normalized time: velocity is displacement and max TOI=1.
                        let hit = cast_shapes(
                            c.position(),
                            Vec3::ZERO,
                            shape,
                            &Pose::from_translation(previous[i]),
                            movement,
                            &ball,
                            ShapeCastOptions {
                                max_time_of_impact: 1.0,
                                stop_at_penetration: false,
                                ..Default::default()
                            },
                        )
                        .map_err(|_| {
                            IntegrationError::UnsupportedCollision {
                                collider: handle,
                                reason: "Parry sweep unsupported",
                            }
                        })?;
                        if let Some(hit) = hit {
                            let normal = c.position().rotation * hit.normal1;
                            let point = c.position().transform_point(hit.witness1);
                            if movement.dot(normal) < 0.0 {
                                let swept = Contact {
                                    key,
                                    normal,
                                    point,
                                    surface_velocity: Vec3::ZERO,
                                    friction,
                                };
                                self.sweeps.insert(key, swept);
                                geometry = Some(swept);
                            }
                        }
                    }
                }
                if let Some(swept) = self.sweeps.get(&key) {
                    geometry = Some(*swept);
                }
                if let Some(contact) = geometry {
                    if out.len() >= self.limit {
                        return Err(ClothError::ContactBudgetExceeded { limit: self.limit }.into());
                    }
                    out.push(contact);
                }
            }
        }
        // Keep sweep planes even if constraint projection moves the particle
        // outside the source collider's candidate AABB during this substep.
        let present: BTreeSet<_> = out.iter().map(|c| c.key).collect();
        for (key, c) in &self.sweeps {
            if !present.contains(key) {
                if out.len() >= self.limit {
                    return Err(ClothError::ContactBudgetExceeded { limit: self.limit }.into());
                }
                out.push(*c);
            }
        }
        Ok(())
    }
}
impl ContactSource for RapierContacts<'_, '_> {
    fn contacts(
        &mut self,
        previous: &[Vec3],
        positions: &[Vec3],
        radius: Real,
        stage: ContactStage,
        out: &mut Vec<Contact>,
    ) -> Result<(), ClothError> {
        let start = Instant::now();
        let result = self.generate(previous, positions, radius, stage, out);
        self.query_time_seconds += start.elapsed().as_secs_f64();
        result.map_err(|e| {
            let message = e.to_string();
            self.error = Some(e);
            ClothError::External(message)
        })
    }
}
