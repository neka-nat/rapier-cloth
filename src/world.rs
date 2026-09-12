use crate::attachment::Attachments;
use crate::collision::RapierContacts;
use crate::rapier::prelude::*;
use crate::{AttachmentDesc, AttachmentEvent, AttachmentEventKind, AttachmentHandle, Target};
use crate::{
    Cloth, ClothError, ClothHandle, ClothSet, IntegrationError, RapierScene, Real, Solver,
    SolverSettings, WorldId, WorldStepReport,
};
use std::{collections::BTreeSet, time::Instant};

#[derive(Debug, Clone)]
pub struct CollisionSettings {
    pub static_sweep: bool,
    pub friction_override: Option<Real>,
    pub excluded_colliders: Vec<ColliderHandle>,
    /// Max kinematic surface movement / smallest particle radius per substep.
    pub motion_limit_ratio: Real,
}
impl Default for CollisionSettings {
    fn default() -> Self {
        Self {
            static_sweep: true,
            friction_override: None,
            excluded_colliders: vec![],
            motion_limit_ratio: 0.5,
        }
    }
}
impl CollisionSettings {
    pub fn validate(&self) -> Result<(), IntegrationError> {
        if self
            .friction_override
            .is_some_and(|m| !m.is_finite() || m < 0.0)
            || !self.motion_limit_ratio.is_finite()
            || self.motion_limit_ratio <= 0.0
        {
            return Err(ClothError::InvalidParameter("collision settings").into());
        }
        Ok(())
    }
}

/// Owns cloth state only. Rapier is always stepped by the application.
pub struct RapierClothWorld {
    world: WorldId,
    next_step: u64,
    desynchronized: bool,
    cloths: ClothSet,
    solver: Solver,
    attachments: Attachments,
    events: Vec<AttachmentEvent>,
    pub solver_settings: SolverSettings,
    pub collision_settings: CollisionSettings,
}
impl RapierClothWorld {
    pub fn new(world: WorldId) -> Self {
        Self {
            world,
            next_step: 0,
            desynchronized: false,
            cloths: ClothSet::new(),
            solver: Solver::new(),
            attachments: Attachments::new(),
            events: vec![],
            solver_settings: SolverSettings::default(),
            collision_settings: CollisionSettings::default(),
        }
    }
    pub fn world_id(&self) -> WorldId {
        self.world
    }
    pub fn next_step_index(&self) -> u64 {
        self.next_step
    }
    pub fn is_desynchronized(&self) -> bool {
        self.desynchronized
    }
    pub fn cloths(&self) -> &ClothSet {
        &self.cloths
    }
    pub fn add_cloth(&mut self, cloth: Cloth) -> ClothHandle {
        self.cloths.insert(cloth)
    }
    pub fn cloth(&self, h: ClothHandle) -> Result<&Cloth, ClothError> {
        self.cloths.get(h)
    }
    pub fn cloth_mut(&mut self, h: ClothHandle) -> Result<&mut Cloth, ClothError> {
        self.cloths.get_mut(h)
    }
    pub fn remove_cloth(&mut self, h: ClothHandle) -> Result<Cloth, ClothError> {
        let cloth = self.cloths.remove(h)?;
        let handles: Vec<_> = self
            .attachments
            .iter()
            .filter(|(_, a)| a.cloth == h)
            .map(|(h, _)| h)
            .collect();
        for handle in handles {
            self.release_with_reason(handle, AttachmentEventKind::ClothRemoved)
                .expect("live attachment");
        }
        Ok(cloth)
    }

    /// A checkpoint contains cloth state only. Restore Rapier to the same time
    /// separately, and discard handles created after the checkpoint.
    pub fn checkpoint(&self) -> Result<ClothCheckpoint, IntegrationError> {
        if self.desynchronized {
            return Err(IntegrationError::InvalidScene(
                "cannot checkpoint a failed step",
            ));
        }
        Ok(ClothCheckpoint {
            world: self.world,
            step: self.next_step,
            cloths: self.cloths.clone(),
            attachments: self.attachments.clone(),
            events: self.events.clone(),
            solver_settings: self.solver_settings,
            collision_settings: self.collision_settings.clone(),
        })
    }
    pub fn restore(&mut self, checkpoint: &ClothCheckpoint) -> Result<(), IntegrationError> {
        if checkpoint.world != self.world
            || checkpoint.attachments.identity != self.attachments.identity
        {
            return Err(IntegrationError::InvalidScene(
                "checkpoint belongs to another cloth world",
            ));
        }
        self.cloths = checkpoint.cloths.clone();
        self.attachments = checkpoint.attachments.clone();
        self.events = checkpoint.events.clone();
        self.next_step = checkpoint.step;
        self.solver_settings = checkpoint.solver_settings;
        self.collision_settings = checkpoint.collision_settings.clone();
        self.desynchronized = false;
        Ok(())
    }
    pub fn attach(
        &mut self,
        desc: AttachmentDesc,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
    ) -> Result<AttachmentHandle, IntegrationError> {
        let cloth = self.cloths.get(desc.cloth)?;
        let body = bodies
            .get(desc.body)
            .ok_or(IntegrationError::InvalidAttachment("body handle"))?;
        if body.is_dynamic() || !body.is_enabled() {
            return Err(IntegrationError::InvalidAttachment(
                "body must be enabled and fixed or kinematic",
            ));
        }
        if desc.points.is_empty() || !desc.compliance.is_finite() || desc.compliance < 0.0 {
            return Err(IntegrationError::InvalidAttachment("points or compliance"));
        }
        let mut particles = BTreeSet::new();
        for p in &desc.points {
            if p.particle as usize >= cloth.positions().len() {
                return Err(ClothError::InvalidParticle(p.particle).into());
            }
            if !p.local_anchor.is_finite() {
                return Err(IntegrationError::InvalidAttachment("non-finite anchor"));
            }
            if !particles.insert(p.particle)
                || cloth.pins().contains_key(&p.particle)
                || self.attachments.iter().any(|(_, a)| {
                    a.cloth == desc.cloth && a.points.iter().any(|q| q.particle == p.particle)
                })
            {
                return Err(ClothError::ConflictingTarget(p.particle).into());
            }
        }
        for handle in &desc.excluded_colliders {
            if colliders
                .get(*handle)
                .is_none_or(|c| c.parent() != Some(desc.body))
            {
                return Err(IntegrationError::InvalidAttachment(
                    "excluded collider must belong to attached body",
                ));
            }
        }
        Ok(self.attachments.insert(desc))
    }
    pub fn attachment(
        &self,
        handle: AttachmentHandle,
    ) -> Result<&AttachmentDesc, IntegrationError> {
        self.attachments
            .get(handle)
            .ok_or(IntegrationError::InvalidAttachment(
                "stale or foreign handle",
            ))
    }
    pub fn attachments(&self) -> impl Iterator<Item = (AttachmentHandle, &AttachmentDesc)> {
        self.attachments.iter()
    }
    pub fn release(&mut self, handle: AttachmentHandle) -> Result<(), IntegrationError> {
        self.release_with_reason(handle, AttachmentEventKind::Released)
    }
    fn release_with_reason(
        &mut self,
        handle: AttachmentHandle,
        kind: AttachmentEventKind,
    ) -> Result<(), IntegrationError> {
        let a = self
            .attachments
            .remove(handle)
            .ok_or(IntegrationError::InvalidAttachment(
                "stale or foreign handle",
            ))?;
        self.events.push(AttachmentEvent {
            handle,
            cloth: a.cloth,
            body: a.body,
            kind,
        });
        Ok(())
    }
    pub fn drain_attachment_events(&mut self) -> impl Iterator<Item = AttachmentEvent> + '_ {
        self.events.drain(..)
    }

    pub fn step_substep(
        &mut self,
        h: Real,
        scene: &RapierScene<'_>,
    ) -> Result<WorldStepReport, IntegrationError> {
        if self.desynchronized {
            return Err(IntegrationError::InvalidScene(
                "previous substep failed; restore both worlds from a checkpoint",
            ));
        }
        if scene.previous.world != self.world {
            return Err(IntegrationError::InvalidScene("world identity mismatch"));
        }
        if scene.previous.step != self.next_step {
            return Err(IntegrationError::InvalidScene(
                "substep must be consumed exactly once, in order",
            ));
        }
        if h != scene.h || !scene.gravity.is_finite() {
            return Err(IntegrationError::InvalidScene(
                "duration or gravity mismatch",
            ));
        }
        self.solver_settings.validate(h)?;
        self.collision_settings.validate()?;
        let next_step = self
            .next_step
            .checked_add(1)
            .ok_or(IntegrationError::InvalidScene("step counter exhausted"))?;
        let start = Instant::now();
        let result = self.perform_step(h, scene);
        match result {
            Ok(mut report) => {
                self.next_step = next_step;
                report.total_time_seconds = start.elapsed().as_secs_f64();
                Ok(report)
            }
            Err(e) => {
                self.desynchronized = true;
                Err(e)
            }
        }
    }
    fn perform_step(
        &mut self,
        h: Real,
        scene: &RapierScene<'_>,
    ) -> Result<WorldStepReport, IntegrationError> {
        let min_radius = self
            .cloths
            .iter()
            .map(|(_, c)| c.material().contact_radius)
            .reduce(Real::min);
        if let Some(radius) = min_radius {
            self.validate_motion(scene, radius)?;
        }
        // World-level atomicity: a later cloth failure cannot partially commit
        // earlier cloths. Topology is Arc-shared by Cloth checkpoints.
        let mut staged_attachments = self.attachments.clone();
        let mut events = vec![];
        for (handle, a) in self.attachments.iter() {
            let reason = match scene.query.bodies.get(a.body) {
                None => Some(AttachmentEventKind::BodyRemoved),
                Some(b) if !b.is_enabled() => Some(AttachmentEventKind::BodyDisabled),
                Some(b) if b.is_dynamic() => {
                    return Err(IntegrationError::InvalidAttachment(
                        "attached body changed to dynamic",
                    ));
                }
                Some(_) => None,
            };
            if let Some(kind) = reason {
                staged_attachments.remove(handle);
                events.push(AttachmentEvent {
                    handle,
                    cloth: a.cloth,
                    body: a.body,
                    kind,
                });
            }
        }
        let mut staged = self.cloths.clone();
        let handles: Vec<_> = staged.iter().map(|(h, _)| h).collect();
        let mut report = WorldStepReport {
            step: self.next_step,
            ..Default::default()
        };
        let mut ignored = BTreeSet::new();
        for handle in handles {
            let cloth = staged.get_mut(handle)?;
            let mut targets = vec![];
            let mut excluded_pairs = BTreeSet::new();
            for (_, a) in staged_attachments.iter().filter(|(_, a)| a.cloth == handle) {
                let body = &scene.query.bodies[a.body];
                if scene.previous.body_pose(a.body).is_none() {
                    return Err(IntegrationError::InvalidAttachment(
                        "missing previous body pose",
                    ));
                }
                for p in &a.points {
                    targets.push(Target {
                        particle: p.particle,
                        position: body.position().transform_point(p.local_anchor),
                        compliance: a.compliance,
                    });
                    for c in &a.excluded_colliders {
                        excluded_pairs.insert((p.particle, c.into_raw_parts()));
                    }
                }
            }
            let mut source = RapierContacts::new(
                scene,
                &self.collision_settings,
                cloth.material(),
                self.solver_settings.max_contacts,
                &excluded_pairs,
            );
            let solver_start = Instant::now();
            let result = self.solver.step_with_contacts(
                cloth,
                h,
                scene.gravity,
                &self.solver_settings,
                &targets,
                &mut source,
            );
            report.core_time_seconds +=
                (solver_start.elapsed().as_secs_f64() - source.query_time_seconds).max(0.0);
            match result {
                Ok(r) => report.cloths.push((handle, r)),
                Err(e) => return Err(source.error.take().unwrap_or_else(|| e.into())),
            }
            ignored.extend(source.ignored);
            report.candidate_queries += source.candidate_queries;
            report.pair_queries += source.pair_queries;
            report.query_time_seconds += source.query_time_seconds;
        }
        report.ignored_colliders = ignored
            .into_iter()
            .map(|(i, g)| ColliderHandle::from_raw_parts(i, g))
            .collect();
        self.cloths = staged;
        self.attachments = staged_attachments;
        self.events.extend(events);
        Ok(report)
    }
    fn validate_motion(
        &self,
        scene: &RapierScene<'_>,
        radius: Real,
    ) -> Result<(), IntegrationError> {
        for (handle, c) in scene.query.colliders.iter() {
            if !c.is_enabled()
                || c.is_sensor()
                || self.collision_settings.excluded_colliders.contains(&handle)
                || !scene.query.filter.test(scene.query.bodies, handle, c)
            {
                continue;
            }
            let Some(body) = c.parent().and_then(|h| scene.query.bodies.get(h)) else {
                continue;
            };
            if !body.is_kinematic() {
                continue;
            }
            let old = scene
                .previous
                .colliders
                .get(&handle.into_raw_parts())
                .ok_or(IntegrationError::MissingPreviousPose(handle))?;
            let now = c.position();
            let delta = now.rotation * old.rotation.inverse();
            let s = (delta.x * delta.x + delta.y * delta.y + delta.z * delta.z).sqrt();
            let angle = 2.0 * s.atan2(delta.w.abs());
            let translation = now.translation.distance(old.translation);
            if angle == 0.0 && translation == 0.0 {
                continue;
            }
            if c.shape().as_halfspace().is_some() {
                return Err(IntegrationError::UnsupportedCollision {
                    collider: handle,
                    reason: "kinematic halfspaces are not supported",
                });
            }
            let aabb = c.shape().compute_local_aabb();
            let bound = aabb.mins.abs().max(aabb.maxs.abs()).length();
            let movement = translation + bound * angle;
            let allowed = radius * self.collision_settings.motion_limit_ratio;
            if !movement.is_finite() || movement > allowed {
                return Err(IntegrationError::MotionBudget {
                    collider: handle,
                    movement,
                    allowed,
                    required_substeps: (movement / allowed).ceil() as usize,
                });
            }
        }
        Ok(())
    }
}

/// Opaque in-memory checkpoint; not a public serialization format.
#[derive(Clone, Debug)]
pub struct ClothCheckpoint {
    world: WorldId,
    step: u64,
    cloths: ClothSet,
    attachments: Attachments,
    events: Vec<AttachmentEvent>,
    solver_settings: SolverSettings,
    collision_settings: CollisionSettings,
}
