use crate::attachment::{Attachment, Attachments};
use crate::rapier::prelude::*;
use crate::rapier_collision::RapierContacts;
use crate::{
    AttachmentDesc, AttachmentEvent, AttachmentEventKind, AttachmentHandle, AttachmentPoint, Target,
};
use crate::{
    Cloth, ClothError, ClothHandle, ClothSet, IntegrationError, RapierScene, Real, Solver,
    SolverSettings, WorldId, WorldStepReport,
};
use std::{collections::BTreeSet, time::Instant};

#[derive(Debug, Clone, PartialEq)]
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
    pub(crate) cloths: ClothSet,
    solver: Solver,
    pub(crate) attachments: Attachments,
    events: Vec<AttachmentEvent>,
    surface_history_scene: Option<crate::SceneSnapshot>,
    surface_history_settings: Option<CollisionSettings>,
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
            surface_history_scene: None,
            surface_history_settings: None,
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
            .filter(|(_, a)| a.cloth() == h)
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
            surface_history_scene: self.surface_history_scene.clone(),
            surface_history_settings: self.surface_history_settings.clone(),
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
        self.surface_history_scene = checkpoint.surface_history_scene.clone();
        self.surface_history_settings = checkpoint.surface_history_settings.clone();
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
                    a.cloth() == desc.cloth && a.uses_particle(p.particle, cloth.mesh())
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
        self.cloths.get_mut(desc.cloth)?.clear_contact_history();
        Ok(self.attachments.insert(Attachment::Vertices(desc)))
    }
    /// Hands a vertex attachment over to another body without releasing it:
    /// every point keeps its particle and its current world position, its
    /// anchor is re-expressed in the new body's frame, and the compliance is
    /// kept. `excluded_colliders` replaces the exclusions and must belong to
    /// the new body. The next step follows the new body.
    pub fn transfer_attachment(
        &mut self,
        handle: AttachmentHandle,
        body: RigidBodyHandle,
        excluded_colliders: Vec<ColliderHandle>,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
    ) -> Result<(), IntegrationError> {
        let desc = self.attachment(handle)?.clone();
        let cloth = self.cloths.get(desc.cloth)?;
        let target = bodies
            .get(body)
            .ok_or(IntegrationError::InvalidAttachment("body handle"))?;
        if target.is_dynamic() || !target.is_enabled() {
            return Err(IntegrationError::InvalidAttachment(
                "body must be enabled and fixed or kinematic",
            ));
        }
        for handle in &excluded_colliders {
            if colliders
                .get(*handle)
                .is_none_or(|c| c.parent() != Some(body))
            {
                return Err(IntegrationError::InvalidAttachment(
                    "excluded collider must belong to attached body",
                ));
            }
        }
        let pose = *target.position();
        let points = desc
            .points
            .iter()
            .map(|p| AttachmentPoint {
                particle: p.particle,
                local_anchor: pose.inverse_transform_point(cloth.positions()[p.particle as usize]),
            })
            .collect();
        if let Some(Attachment::Vertices(a)) = self.attachments.get_mut(handle) {
            a.body = body;
            a.points = points;
            a.excluded_colliders = excluded_colliders;
        }
        self.cloths.get_mut(desc.cloth)?.clear_contact_history();
        Ok(())
    }
    /// Changes the compliance of a vertex attachment in place: the body and
    /// the anchors stay, so the held vertices keep their targets and only the
    /// stiffness of the hold (`1 / compliance`) changes from the next step.
    /// Raising it over a few steps before `release` is a soft release.
    pub fn set_attachment_compliance(
        &mut self,
        handle: AttachmentHandle,
        compliance: Real,
    ) -> Result<(), IntegrationError> {
        if !compliance.is_finite() || compliance < 0.0 {
            return Err(IntegrationError::InvalidAttachment("compliance"));
        }
        match self.attachments.get_mut(handle) {
            Some(Attachment::Vertices(a)) => {
                a.compliance = compliance;
                Ok(())
            }
            _ => Err(IntegrationError::InvalidAttachment(
                "stale, foreign or surface attachment handle",
            )),
        }
    }
    pub fn attachment(
        &self,
        handle: AttachmentHandle,
    ) -> Result<&AttachmentDesc, IntegrationError> {
        self.attachments
            .get(handle)
            .and_then(|a| match a {
                Attachment::Vertices(a) => Some(a),
                Attachment::Surface(_) => None,
            })
            .ok_or(IntegrationError::InvalidAttachment(
                "stale, foreign or surface attachment handle",
            ))
    }
    pub fn attachments(&self) -> impl Iterator<Item = (AttachmentHandle, &AttachmentDesc)> {
        self.attachments.iter().filter_map(|(h, a)| match a {
            Attachment::Vertices(a) => Some((h, a)),
            Attachment::Surface(_) => None,
        })
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
        if let Ok(cloth) = self.cloths.get_mut(a.cloth()) {
            cloth.clear_contact_history();
        }
        self.events.push(AttachmentEvent {
            handle,
            cloth: a.cloth(),
            body: a.body(),
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
        // The budget protects discrete queries from missing a fast crossing.
        // A cloth with continuous rigid collision certifies every kinematic
        // motion by sweeping it instead, so the budget applies only while some
        // cloth in the world still relies on discrete or particle contacts.
        let discrete = self.cloths.iter().any(|(_, c)| {
            !c.contact_settings()
                .is_some_and(|s| s.rigid_surface_collision && s.continuous_rigid_collision)
        });
        let min_radius = self
            .cloths
            .iter()
            .map(|(_, c)| c.material().contact_radius)
            .reduce(Real::min);
        if let Some(radius) = min_radius
            && discrete
        {
            self.validate_motion(scene, radius)?;
        }
        // World-level atomicity: a later cloth failure cannot partially commit
        // earlier cloths. Topology is Arc-shared by Cloth checkpoints.
        let mut staged_attachments = self.attachments.clone();
        let mut events = vec![];
        for (handle, a) in self.attachments.iter() {
            let reason = match scene.query.bodies.get(a.body()) {
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
                    cloth: a.cloth(),
                    body: a.body(),
                    kind,
                });
            }
        }
        let mut staged = self.cloths.clone();
        // Between-step teleports or shape/filter changes invalidate material
        // anchors even if a regenerated manifold happens to reuse its key.
        // Ordinary kinematic motion begins at the last committed rigid pose.
        let context_changed = self
            .surface_history_settings
            .as_ref()
            .is_some_and(|settings| settings != &self.collision_settings)
            || self.surface_history_scene.as_ref().is_some_and(|last| {
                last.colliders.iter().any(|(key, pose)| {
                    scene.previous.colliders.get(key) != Some(pose)
                        || last
                            .shapes
                            .get(key)
                            .zip(scene.previous.shapes.get(key))
                            .is_none_or(|(a, b)| !std::sync::Arc::ptr_eq(&a.0, &b.0))
                })
            });
        if context_changed {
            let handles: Vec<_> = staged.iter().map(|(h, _)| h).collect();
            for handle in handles {
                staged.get_mut(handle)?.clear_contact_history();
            }
        }
        for event in &events {
            staged.get_mut(event.cloth)?.clear_contact_history();
        }
        let handles: Vec<_> = staged.iter().map(|(h, _)| h).collect();
        let mut report = WorldStepReport {
            step: self.next_step,
            ..Default::default()
        };
        let mut ignored = BTreeSet::new();
        for handle in handles {
            let cloth = staged.get_mut(handle)?;
            let mut targets = vec![];
            let mut surface_targets = vec![];
            let mut excluded_pairs = BTreeSet::new();
            for (_, a) in staged_attachments
                .iter()
                .filter(|(_, a)| a.cloth() == handle)
            {
                let body = &scene.query.bodies[a.body()];
                if scene.previous.body_pose(a.body()).is_none() {
                    return Err(IntegrationError::InvalidAttachment(
                        "missing previous body pose",
                    ));
                }
                match a {
                    Attachment::Vertices(a) => {
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
                    Attachment::Surface(a) => {
                        for p in &a.points {
                            surface_targets.push(crate::SurfaceTarget {
                                point: p.point,
                                position: body.position().transform_point(p.local_anchor),
                                compliance: a.compliance,
                            });
                            let triangle = cloth.mesh().triangles()[p.point.triangle() as usize];
                            for (i, b) in triangle.into_iter().zip(p.point.barycentric()) {
                                if b != 0.0 {
                                    for c in &a.excluded_colliders {
                                        excluded_pairs.insert((i, c.into_raw_parts()));
                                    }
                                }
                            }
                        }
                    }
                }
            }
            let mut source = RapierContacts::new(
                scene,
                &self.collision_settings,
                cloth.material(),
                self.solver_settings.max_contacts,
                &excluded_pairs,
            )
            .with_surface(cloth);
            let solver_start = Instant::now();
            let result = self.solver.step_with_surface_targets(
                cloth,
                h,
                scene.gravity,
                &self.solver_settings,
                &targets,
                &surface_targets,
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
        if self.cloths.iter().any(|(_, cloth)| {
            cloth
                .contact_settings()
                .is_some_and(|s| s.rigid_surface_collision)
        }) {
            self.surface_history_scene = Some(crate::SceneSnapshot::capture(
                self.world,
                self.next_step,
                scene.query.bodies,
                scene.query.colliders,
            ));
            self.surface_history_settings = Some(self.collision_settings.clone());
        } else {
            self.surface_history_scene = None;
            self.surface_history_settings = None;
        }
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
            // End poses alone alias complete revolutions for velocity-based
            // kinematics. Include Rapier's angular speed over the external step.
            let angle = (2.0 * s.atan2(delta.w.abs())).max(body.angvel().length() * scene.h);
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
            // An offset collider travels an arc around the body origin, even
            // when its start/end translations almost coincide.
            let lever_arm = c
                .position_wrt_parent()
                .map_or(0.0, |p| p.translation.length());
            let movement = translation + (bound + lever_arm) * angle;
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
    surface_history_scene: Option<crate::SceneSnapshot>,
    surface_history_settings: Option<CollisionSettings>,
    solver_settings: SolverSettings,
    collision_settings: CollisionSettings,
}
