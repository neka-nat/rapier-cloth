use crate::collision::{
    ClothContactSettings, CollisionBudgetKind, CollisionWork, friction::FrictionState,
    normal_block, self_collision::SelfCollision,
};
use crate::{
    Cloth, ClothError, Contact, ContactMotion, ContactSource, ContactStage, NoContacts, Real,
    StepReport, Vec3,
    constraints::{
        bend::{angle, angle_and_gradients, angle_difference},
        distance,
        target::{ResolvedSurfaceTarget, SurfaceTarget},
        tether::Tethers,
    },
    contact::{SurfaceContact, SurfaceContactState, friction_velocity},
};

#[derive(Debug, Clone, Copy)]
pub struct SolverSettings {
    pub iterations: usize,
    pub max_substep: Real,
    pub max_contacts: usize,
}

impl Default for SolverSettings {
    fn default() -> Self {
        Self {
            iterations: 8,
            max_substep: 1.0 / 30.0,
            max_contacts: 65536,
        }
    }
}
impl SolverSettings {
    pub fn validate(&self, h: Real) -> Result<(), ClothError> {
        if self.iterations == 0 || self.iterations > 1024 {
            return Err(ClothError::InvalidParameter(
                "iterations must be in 1..=1024",
            ));
        }
        if !self.max_substep.is_finite()
            || self.max_substep <= 0.0
            || !h.is_finite()
            || h <= 0.0
            || h > self.max_substep
            || !(1.0 / (h * h)).is_finite()
        {
            return Err(ClothError::InvalidParameter("substep duration"));
        }
        if self.max_contacts == 0 {
            return Err(ClothError::InvalidParameter("max_contacts"));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Target {
    pub particle: u32,
    pub position: Vec3,
    pub compliance: Real,
}

#[derive(Clone, Copy, Debug)]
struct ContactState {
    contact: Contact,
    lambda: Real,
}

/// Reusable staging buffers. Cloth state is committed only after all queries,
/// projections and finite-value checks succeed.
#[derive(Default, Debug)]
pub struct Solver {
    reference: Vec<Vec3>,
    positions: Vec<Vec3>,
    prediction: Vec<Vec3>,
    velocities: Vec<Vec3>,
    weights: Vec<Real>,
    distance_lambda: Vec<Real>,
    bend_lambda: Vec<Real>,
    target_lambda: Vec<Vec3>,
    surface_target_lambda: Vec<Vec3>,
    tethers: Tethers,
    contacts: Vec<Contact>,
    // Sorted by key. Merge each query with the cumulative substep states, so
    // inactive contacts retain their lambda without per-contact tree lookups.
    contact_states: Vec<ContactState>,
    next_contact_states: Vec<ContactState>,
    surface_contacts: Vec<SurfaceContact>,
    surface_states: Vec<SurfaceContactState>,
    previous_surface_states: Vec<SurfaceContactState>,
    next_surface_states: Vec<SurfaceContactState>,
    surface_solve_order: Vec<usize>,
    rigid_support: Vec<usize>,
    stabilization_lambda: Vec<Real>,
    stretches: Vec<Real>,
    self_collision: Option<SelfCollision>,
    #[cfg(feature = "implicit")]
    implicit_cache: crate::implicit::Cache,
    contact_settings: Option<ClothContactSettings>,
    surface_high_water: usize,
    external_collision_work: CollisionWork,
    external_continuous_motion: bool,
    motion_start: Vec<Vec3>,
    motion_trial: Vec<Vec3>,
    motion_distance_lambda: Vec<Real>,
    motion_bend_lambda: Vec<Real>,
    motion_target_lambda: Vec<Vec3>,
    motion_surface_target_lambda: Vec<Vec3>,
}

impl Solver {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn step(
        &mut self,
        cloth: &mut Cloth,
        h: Real,
        gravity: Vec3,
        settings: &SolverSettings,
    ) -> Result<StepReport, ClothError> {
        self.step_with_contacts(cloth, h, gravity, settings, &[], &mut NoContacts)
    }
    pub fn step_with_contacts(
        &mut self,
        cloth: &mut Cloth,
        h: Real,
        gravity: Vec3,
        settings: &SolverSettings,
        external_targets: &[Target],
        source: &mut impl ContactSource,
    ) -> Result<StepReport, ClothError> {
        self.step_with_surface_targets(cloth, h, gravity, settings, external_targets, &[], source)
    }

    /// Add material-point targets to the existing particle-target/contact path.
    /// Nonzero supports may not overlap another target or pin. Hard surface
    /// targets constrain only their weighted point; support inverse masses stay
    /// physical. Every correction and multiplier participates in motion checks.
    #[allow(clippy::too_many_arguments)]
    pub fn step_with_surface_targets(
        &mut self,
        cloth: &mut Cloth,
        h: Real,
        gravity: Vec3,
        settings: &SolverSettings,
        external_targets: &[Target],
        surface_targets: &[SurfaceTarget],
        source: &mut impl ContactSource,
    ) -> Result<StepReport, ClothError> {
        settings.validate(h)?;
        #[cfg(feature = "implicit")]
        if let Some(implicit) = cloth.implicit_settings {
            if !surface_targets.is_empty() {
                return Err(ClothError::InvalidParameter(
                    "implicit solver currently supports particle targets only",
                ));
            }
            return crate::implicit::step(
                cloth,
                h,
                gravity,
                settings,
                external_targets,
                source,
                implicit,
                &mut self.implicit_cache,
            );
        }
        if !gravity.is_finite() {
            return Err(ClothError::InvalidParameter("gravity"));
        }
        self.contact_settings = cloth.contact_settings;
        self.surface_high_water = 0;
        self.external_collision_work = CollisionWork::default();
        self.external_continuous_motion = source.continuous_motion();
        if let Some(config) = cloth.contact_settings {
            config.validate()?;
            if config.self_collision {
                self.self_collision
                    .get_or_insert_with(|| SelfCollision::new(cloth.mesh.clone(), config))
                    .begin(cloth.mesh.clone(), config);
            }
        }
        let mut targets: Vec<_> = cloth
            .pins
            .iter()
            .map(|(&particle, &position)| Target {
                particle,
                position,
                compliance: 0.0,
            })
            .collect();
        targets.extend_from_slice(external_targets);
        targets.sort_by_key(|t| t.particle);
        for (i, t) in targets.iter().enumerate() {
            if t.particle as usize >= cloth.positions.len() {
                return Err(ClothError::InvalidParticle(t.particle));
            }
            if !t.position.is_finite() || !t.compliance.is_finite() || t.compliance < 0.0 {
                return Err(ClothError::InvalidParameter("target"));
            }
            if i > 0 && targets[i - 1].particle == t.particle {
                return Err(ClothError::ConflictingTarget(t.particle));
            }
        }
        let mut resolved_surface_targets = Vec::with_capacity(surface_targets.len());
        if !surface_targets.is_empty() {
            let mut occupied: std::collections::BTreeSet<_> =
                targets.iter().map(|t| t.particle).collect();
            for &target in surface_targets {
                let resolved = ResolvedSurfaceTarget::new(target, &cloth.mesh)?;
                for (&i, b) in resolved.vertices.iter().zip(target.point.barycentric()) {
                    if b != 0.0 && !occupied.insert(i) {
                        return Err(ClothError::ConflictingTarget(i));
                    }
                }
                resolved_surface_targets.push(resolved);
            }
        }
        self.reference.clone_from(&cloth.positions);
        self.positions.clone_from(&cloth.positions);
        self.velocities.clone_from(&cloth.velocities);
        self.weights.clone_from(&cloth.inverse_masses);
        self.surface_states.clear();
        self.previous_surface_states
            .clone_from(&cloth.contact_history);
        for state in &mut self.previous_surface_states {
            state.normal_lambda = 0.0;
            state.friction.reset_step();
            if !state.friction.anchor.is_finite() {
                return Err(ClothError::NonFiniteState);
            }
        }
        for t in &targets {
            if t.compliance == 0.0 {
                self.weights[t.particle as usize] = 0.0;
            }
        }
        self.tethers.prepare(
            &cloth.mesh,
            &targets,
            cloth.material.stretch_compliance == 0.0,
        )?;
        let radius = cloth.material.contact_radius;
        let mut report = StepReport::default();
        self.contact_states.clear();
        // Restore initial overlaps separately from physical integration. This
        // displacement is excluded from reconstructed velocities and friction.
        for _ in 0..settings.iterations {
            self.query(
                source,
                Some(&cloth.positions),
                radius,
                ContactStage::Stabilization,
                settings,
            )?;
            self.capture_motion();
            let mut corrected = false;
            for c in &self.contacts {
                let i = c.key.particle as usize;
                let depth = radius - (self.positions[i] - c.point).dot(c.normal);
                if depth > radius * 1.0e-4 && self.weights[i] > 0.0 {
                    let relative = self.positions[i] - c.point;
                    let tangent = relative - c.normal * relative.dot(c.normal);
                    self.positions[i] = c.point + tangent + c.normal * radius;
                    report.stabilized_contacts += 1;
                    corrected = true;
                }
            }
            corrected |= self.stabilize_surface_contacts(&mut report)?;
            self.accept_motion(source, ContactStage::Stabilization)?;
            if !corrected {
                break;
            }
        }
        self.reference.copy_from_slice(&self.positions);
        for state in &mut self.previous_surface_states {
            // Initial-overlap recovery is not physical tangential motion.
            state
                .friction
                .restore_reference(&cloth.positions, &self.reference)?;
        }
        self.capture_motion();
        let damp = (-cloth.material.damping * h).exp();
        for i in 0..self.positions.len() {
            self.velocities[i] = (cloth.velocities[i]
                + (gravity + cloth.forces[i] * cloth.inverse_masses[i]) * h)
                * damp;
            self.positions[i] += self.velocities[i] * h;
        }
        self.prediction.clone_from(&self.positions);
        for t in &targets {
            if t.compliance == 0.0 {
                self.positions[t.particle as usize] = t.position;
            }
        }
        self.distance_lambda.resize(cloth.mesh.edges().len(), 0.0);
        self.distance_lambda.fill(0.0);
        self.bend_lambda.resize(cloth.mesh.hinges().len(), 0.0);
        self.bend_lambda.fill(0.0);
        self.target_lambda.resize(targets.len(), Vec3::ZERO);
        self.target_lambda.fill(Vec3::ZERO);
        self.surface_target_lambda
            .resize(surface_targets.len(), Vec3::ZERO);
        self.surface_target_lambda.fill(Vec3::ZERO);
        for (target, lambda) in resolved_surface_targets
            .iter()
            .zip(&mut self.surface_target_lambda)
        {
            if target.target.compliance == 0.0 {
                target.project(&mut self.positions, &self.weights, h, lambda)?;
            }
        }
        self.solve_prediction_contacts(source, radius, h, settings, &cloth.mesh)?;
        let stretch_alpha = cloth.material.stretch_compliance / (h * h);
        let bend_alpha = cloth.material.bend_compliance / (h * h);
        for _ in 0..settings.iterations {
            self.capture_motion();
            if self.continuous_motion() {
                self.motion_distance_lambda
                    .clone_from(&self.distance_lambda);
                self.motion_bend_lambda.clone_from(&self.bend_lambda);
                self.motion_target_lambda.clone_from(&self.target_lambda);
                self.motion_surface_target_lambda
                    .clone_from(&self.surface_target_lambda);
            }
            for (i, e) in cloth.mesh.edges().iter().enumerate() {
                if !distance::project(
                    &mut self.positions,
                    &self.weights,
                    e.vertices,
                    e.rest_length,
                    stretch_alpha,
                    &mut self.distance_lambda[i],
                ) {
                    return Err(ClothError::DegenerateConstraint);
                }
            }
            for (j, hinge) in cloth.mesh.hinges().iter().enumerate() {
                let ids = hinge.vertices.map(|i| i as usize);
                let (angle, g) = angle_and_gradients(ids.map(|i| self.positions[i]))
                    .ok_or(ClothError::DegenerateConstraint)?;
                let denom: Real = g
                    .iter()
                    .zip(ids)
                    .map(|(g, i)| self.weights[i] * g.length_squared())
                    .sum::<Real>()
                    + bend_alpha;
                if denom > 0.0 {
                    let dl = (-angle_difference(angle, hinge.rest_angle)
                        - bend_alpha * self.bend_lambda[j])
                        / denom;
                    self.bend_lambda[j] += dl;
                    for k in 0..4 {
                        self.positions[ids[k]] += g[k] * (self.weights[ids[k]] * dl);
                    }
                }
            }
            for (j, t) in targets.iter().enumerate() {
                let i = t.particle as usize;
                if t.compliance == 0.0 {
                    self.positions[i] = t.position;
                    continue;
                }
                let alpha = t.compliance / (h * h);
                let dl = (-(self.positions[i] - t.position) - self.target_lambda[j] * alpha)
                    / (self.weights[i] + alpha);
                self.target_lambda[j] += dl;
                self.positions[i] += dl * self.weights[i];
            }
            self.tethers.project(&mut self.positions, &self.weights)?;
            for (target, lambda) in resolved_surface_targets
                .iter()
                .zip(&mut self.surface_target_lambda)
            {
                target.project(&mut self.positions, &self.weights, h, lambda)?;
            }
            // Elastic and contact projections together form one trial. A
            // penetrated intermediate elastic pose is not an accepted motion:
            // normal contact may restore separation while retaining tangential
            // stretch recovery. Certify the completed trial from the previous
            // accepted pose, and scale every contributing multiplier together.
            self.query(source, None, radius, ContactStage::Iteration, settings)?;
            self.project_contacts(radius, settings.max_contacts, false)?;
            self.project_surface_contacts(source, h, settings.max_contacts, &cloth.mesh, false)?;
            let fraction = self.accept_motion(source, ContactStage::Iteration)?;
            if fraction < 1.0 {
                for (value, &before) in self
                    .distance_lambda
                    .iter_mut()
                    .zip(&self.motion_distance_lambda)
                {
                    *value = before + (*value - before) * fraction;
                }
                for (value, &before) in self
                    .surface_target_lambda
                    .iter_mut()
                    .zip(&self.motion_surface_target_lambda)
                {
                    *value = before + (*value - before) * fraction;
                }
                for (value, &before) in self.bend_lambda.iter_mut().zip(&self.motion_bend_lambda) {
                    *value = before + (*value - before) * fraction;
                }
                for (value, &before) in self
                    .target_lambda
                    .iter_mut()
                    .zip(&self.motion_target_lambda)
                {
                    *value = before + (*value - before) * fraction;
                }
            }
            self.scale_contact_motion(fraction)?;
            if fraction < 1.0 {
                // A distant trial can pass beyond a feature's discrete contact
                // margin before CCD shortens it. Refresh at the accepted prefix
                // to restore full thickness; otherwise repeated elastic trials
                // consume its small numerical clearance without ever activating
                // normal contact. This correction has its own certified sweep.
                // It only pushes features outward: a contact whose accumulated
                // multiplier would pull its point back to the target gap can
                // pivot a neighbouring feature towards the obstacle, and the
                // shortened batch would then never regain clearance.
                self.query(source, None, radius, ContactStage::Iteration, settings)?;
                self.capture_motion();
                self.project_contacts(radius, settings.max_contacts, true)?;
                self.project_surface_contacts(source, h, settings.max_contacts, &cloth.mesh, true)?;
                let fraction = self.accept_motion(source, ContactStage::Iteration)?;
                self.scale_contact_motion(fraction)?;
            }
        }
        // The certified path is the sequence of accepted batches above. The
        // substep's straight chord is not certified separately: a vertex that
        // rounds a convex obstacle edge within one substep has a clear
        // piecewise path whose chord cuts the corner.
        for i in 0..self.positions.len() {
            if self.weights[i] == 0.0 {
                self.velocities[i] = (self.positions[i] - cloth.positions[i]) / h;
            } else {
                // Equivalent to (x - reference) / h in exact arithmetic, but
                // preserves the integrated velocity when f32 world positions
                // lose low bits in the small per-substep displacement.
                self.velocities[i] += (self.positions[i] - self.prediction[i]) / h;
            }
        }
        // Position solve impulses plus any residual inward normal velocity.
        // Stabilization never contributed to these lambdas.
        for state in &self.contact_states {
            let c = state.contact;
            let i = c.key.particle as usize;
            if self.weights[i] == 0.0 {
                continue;
            }
            if (self.positions[i] - c.point).dot(c.normal) > radius * 1.01 {
                continue;
            }
            let vn = (self.velocities[i] - c.surface_velocity).dot(c.normal);
            let dv = (-vn).max(0.0);
            self.velocities[i] += c.normal * dv;
            let support = state.lambda * self.weights[i] / h + dv;
            self.velocities[i] = friction_velocity(
                self.velocities[i],
                c.surface_velocity,
                c.normal,
                support,
                c.friction,
            );
        }
        for state in &self.surface_states {
            let c = state.contact;
            if self
                .surface_contacts
                .binary_search_by_key(&c.key, |contact| contact.key)
                .is_err()
            {
                continue;
            }
            if c.gap(&self.positions) > c.separation * 0.01 {
                continue;
            }
            let inverse_mass = c.inverse_mass(&self.weights);
            if inverse_mass == 0.0 {
                continue;
            }
            let relative = c.relative(&self.velocities) - c.surface_velocity;
            let normal_speed = relative.dot(c.normal);
            let normal_impulse = (-normal_speed).max(0.0) / inverse_mass;
            c.apply(
                &mut self.velocities,
                &self.weights,
                c.normal * normal_impulse,
            );
            let tangent = relative - c.normal * normal_speed;
            let speed = tangent.length();
            if speed > Real::MIN_POSITIVE {
                // Position friction already spent the positional normal load.
                // Only a residual velocity-level impact supplies an extra load.
                let impulse = (c.kinetic_friction * normal_impulse).min(speed / inverse_mass);
                c.apply(
                    &mut self.velocities,
                    &self.weights,
                    -tangent * (impulse / speed),
                );
            }
        }
        self.query(source, None, radius, ContactStage::Final, settings)?;
        for c in &self.contacts {
            let i = c.key.particle as usize;
            let depth = (radius - (self.positions[i] - c.point).dot(c.normal)).max(0.0);
            report.max_penetration = report.max_penetration.max(depth);
            if self.weights[i] == 0.0 && depth > radius * 0.2 {
                return Err(ClothError::ConflictingTarget(i as u32));
            }
        }
        for c in &self.surface_contacts {
            let depth = (-c.gap(&self.positions)).max(0.0);
            report.max_penetration = report.max_penetration.max(depth);
            if c.inverse_mass(&self.weights) == 0.0 && depth > c.separation * 1.0e-4 {
                return Err(ClothError::InfeasibleSurfaceContact);
            }
        }
        if self
            .positions
            .iter()
            .chain(&self.velocities)
            .any(|v| !v.is_finite())
        {
            return Err(ClothError::NonFiniteState);
        }
        self.stretches.clear();
        for e in cloth.mesh.edges() {
            let strain = (self.positions[e.vertices[0] as usize]
                .distance(self.positions[e.vertices[1] as usize])
                / e.rest_length
                - 1.0)
                .max(0.0);
            self.stretches.push(strain);
            report.max_stretch = report.max_stretch.max(strain);
        }
        let rank = ((self.stretches.len() as Real * 0.95).ceil() as usize).saturating_sub(1);
        report.p95_stretch = *self
            .stretches
            .select_nth_unstable_by(rank, Real::total_cmp)
            .1;
        for hinge in cloth.mesh.hinges() {
            let angle = angle(hinge.vertices.map(|i| self.positions[i as usize]))
                .ok_or(ClothError::DegenerateConstraint)?;
            report.max_bend_error = report
                .max_bend_error
                .max(angle_difference(angle, hinge.rest_angle).abs());
        }
        for t in &targets {
            let error = self.positions[t.particle as usize].distance(t.position);
            if self.continuous_motion()
                && t.compliance == 0.0
                && error > crate::math::LENGTH_EPSILON
            {
                return Err(ClothError::ConflictingTarget(t.particle));
            }
            report.max_target_error = report.max_target_error.max(error);
        }
        for target in &resolved_surface_targets {
            let error = target
                .position(&self.positions)
                .distance(target.target.position);
            if !error.is_finite() {
                return Err(ClothError::NonFiniteState);
            }
            if target.target.compliance == 0.0 && error > crate::math::LENGTH_EPSILON {
                return Err(ClothError::ConflictingSurfaceTarget {
                    triangle: target.target.point.triangle(),
                });
            }
            report.max_target_error = report.max_target_error.max(error);
        }
        for &[a, b, c] in cloth.mesh.triangles() {
            if (self.positions[b as usize] - self.positions[a as usize])
                .cross(self.positions[c as usize] - self.positions[a as usize])
                .length_squared()
                <= Real::MIN_POSITIVE
            {
                report.degenerate_faces += 1;
            }
        }
        report.contacts = self.contact_states.len() + self.surface_states.len();
        report.surface_collision = if self.contact_settings.is_some_and(|s| s.self_collision) {
            self.self_collision.as_ref().unwrap().work
        } else {
            self.external_collision_work
        };
        report.surface_collision.retained_contacts = report
            .surface_collision
            .retained_contacts
            .max(self.surface_high_water);
        report.iterations = settings.iterations;
        report.scratch_bytes = (self.positions.capacity()
            + self.prediction.capacity()
            + self.reference.capacity()
            + self.velocities.capacity()
            + self.motion_start.capacity()
            + self.motion_trial.capacity()
            + self.motion_target_lambda.capacity()
            + self.motion_surface_target_lambda.capacity()
            + self.surface_target_lambda.capacity()
            + self.target_lambda.capacity())
            * std::mem::size_of::<Vec3>()
            + (self.weights.capacity()
                + self.distance_lambda.capacity()
                + self.bend_lambda.capacity()
                + self.motion_distance_lambda.capacity()
                + self.motion_bend_lambda.capacity()
                + self.stretches.capacity())
                * std::mem::size_of::<Real>()
            + self.contacts.capacity() * std::mem::size_of::<Contact>()
            + (self.contact_states.capacity() + self.next_contact_states.capacity())
                * std::mem::size_of::<ContactState>()
            + self.surface_contacts.capacity() * std::mem::size_of::<SurfaceContact>()
            + (self.surface_states.capacity()
                + self.previous_surface_states.capacity()
                + self.next_surface_states.capacity())
                * std::mem::size_of::<SurfaceContactState>();
        report.scratch_bytes += self
            .self_collision
            .as_ref()
            .map_or(0, SelfCollision::scratch_bytes);
        report.scratch_bytes += self.tethers.scratch_bytes();
        report.scratch_bytes += (self.surface_solve_order.capacity()
            + self.rigid_support.capacity())
            * std::mem::size_of::<usize>();
        report.scratch_bytes += self.stabilization_lambda.capacity() * std::mem::size_of::<Real>();
        // Persist only current, still touching features. A disappeared contact
        // must not act as adhesion or consume history indefinitely.
        self.surface_states.retain(|state| {
            self.surface_contacts
                .binary_search_by_key(&state.contact.key, |c| c.key)
                .is_ok()
                && state.contact.gap(&self.positions) <= state.contact.separation * 0.01
        });
        for state in &mut self.surface_states {
            state
                .friction
                .finish(state.contact, &self.positions, &cloth.mesh)?;
        }
        cloth.previous.copy_from_slice(&cloth.positions);
        std::mem::swap(&mut cloth.positions, &mut self.positions);
        std::mem::swap(&mut cloth.velocities, &mut self.velocities);
        std::mem::swap(&mut cloth.contact_history, &mut self.surface_states);
        Ok(report)
    }

    fn continuous_self_collision(&self) -> bool {
        self.contact_settings
            .is_some_and(|s| s.continuous_self_collision)
    }
    fn continuous_motion(&self) -> bool {
        self.continuous_self_collision() || self.external_continuous_motion
    }
    fn motion_fraction(
        &mut self,
        source: &mut impl ContactSource,
        stage: ContactStage,
        initial: Option<&[Vec3]>,
    ) -> Result<Real, ClothError> {
        let start = initial.unwrap_or(&self.motion_start);
        let mut fraction = if self.continuous_self_collision() {
            self.self_collision
                .as_mut()
                .unwrap()
                .motion_fraction(start, &self.positions)?
        } else {
            1.0
        };
        if self.external_continuous_motion {
            let work = if self.contact_settings.is_some_and(|s| s.self_collision) {
                &mut self.self_collision.as_mut().unwrap().work
            } else {
                &mut self.external_collision_work
            };
            let external = source.motion_fraction(
                ContactMotion {
                    start,
                    end: &self.positions,
                    stage,
                },
                work,
            )?;
            if !external.is_finite() || !(0.0..=1.0).contains(&external) {
                return Err(ClothError::External("invalid motion fraction".into()));
            }
            fraction = fraction.min(external);
        }
        Ok(fraction)
    }
    fn solve_prediction_contacts(
        &mut self,
        source: &mut impl ContactSource,
        radius: Real,
        h: Real,
        settings: &SolverSettings,
        mesh: &crate::ClothMesh,
    ) -> Result<(), ClothError> {
        if !self.continuous_motion() {
            self.query(source, None, radius, ContactStage::Prediction, settings)?;
            self.project_contacts(radius, settings.max_contacts, false)?;
            return self.project_surface_contacts(source, h, settings.max_contacts, mesh, false);
        }
        // CCD supplies contact witnesses at an intermediate safe query pose.
        // Solve those contacts against the full inertial prediction. Scaling the
        // inertial displacement itself would damp tangential motion even with
        // zero friction, and lose part of the normal support impulse.
        let mut certified: Real = 0.0;
        for _ in 0..settings.iterations {
            self.motion_trial.clone_from(&self.positions);
            let fraction = self.motion_fraction(source, ContactStage::Prediction, None)?;
            if fraction < 1.0 {
                for (p, &start) in self.positions.iter_mut().zip(&self.motion_start) {
                    *p = start + (*p - start) * fraction;
                }
            }
            self.query(source, None, radius, ContactStage::Prediction, settings)?;
            self.positions.copy_from_slice(&self.motion_trial);
            self.project_contacts(radius, settings.max_contacts, false)?;
            self.project_surface_contacts(source, h, settings.max_contacts, mesh, false)?;
            let after = self.motion_fraction(source, ContactStage::Prediction, None)?;
            if after == 1.0 {
                return Ok(());
            }
            // The witnesses describe the prediction's endpoint. A feature that
            // only grazes an obstacle along the way (a protruding hull vertex
            // under a sliding sheet) can satisfy every witness at the endpoint
            // while the path still dips below the swept minimum, so the
            // projection changes nothing and another attempt would repeat it.
            if after <= certified.max(fraction) {
                break;
            }
            certified = after;
        }
        // Keep the certified prefix of the projected prediction instead of
        // failing: the path stays certified and the elastic batches continue
        // from there; only the grazing feature loses part of its inertial
        // motion in this substep.
        self.accept_motion(source, ContactStage::Prediction)?;
        Ok(())
    }
    fn capture_motion(&mut self) {
        if self.continuous_motion() {
            self.motion_start.clone_from(&self.positions);
        }
    }
    /// A projection batch is a trial pose until its linear motion from the
    /// previous accepted pose is bounded. This includes stabilization,
    /// prediction/targets, elastic constraints and contact corrections.
    fn accept_motion(
        &mut self,
        source: &mut impl ContactSource,
        stage: ContactStage,
    ) -> Result<Real, ClothError> {
        if !self.continuous_motion() {
            return Ok(1.0);
        }
        let mut total: Real = 1.0;
        for _ in 0..4 {
            let fraction = self.motion_fraction(source, stage, None)?;
            if fraction == 1.0 {
                return Ok(total);
            }
            if fraction <= 0.0 {
                return Err(ClothError::UnresolvedContinuousCollision(
                    "motion batch made no progress",
                ));
            }
            for (position, &start) in self.positions.iter_mut().zip(&self.motion_start) {
                *position = start + (*position - start) * fraction;
            }
            total *= fraction;
            // Recheck the actual rounded endpoint before accepting it. World
            // coordinate interpolation can round differently from local CCD.
        }
        Err(ClothError::UnresolvedContinuousCollision(
            "rounded endpoint could not be certified",
        ))
    }
    fn scale_contact_motion(&mut self, fraction: Real) -> Result<(), ClothError> {
        if fraction == 1.0 {
            return Ok(());
        }
        // After the sorted-state swaps, the next_* arrays still contain the
        // old states. Scale only this batch's multipliers, matching its accepted
        // displacement, so clipped trials cannot inflate frictional support.
        let mut old = self.next_contact_states.iter().peekable();
        for state in &mut self.contact_states {
            while old
                .peek()
                .is_some_and(|s| s.contact.key < state.contact.key)
            {
                old.next();
            }
            let base = old
                .peek()
                .filter(|s| {
                    s.contact.key == state.contact.key
                        && s.contact.normal.dot(state.contact.normal) >= 0.9
                })
                .map_or(0.0, |s| s.lambda);
            state.lambda = base + (state.lambda - base) * fraction;
        }
        let mut old = self.next_surface_states.iter().peekable();
        for state in &mut self.surface_states {
            while old
                .peek()
                .is_some_and(|s| s.contact.key < state.contact.key)
            {
                old.next();
            }
            let base = old
                .peek()
                .filter(|s| {
                    s.contact.key == state.contact.key
                        && s.contact.normal.dot(state.contact.normal) >= 0.9
                })
                .map_or(0.0, |s| s.normal_lambda);
            state.normal_lambda = base + (state.normal_lambda - base) * fraction;
            let previous_friction = old
                .peek()
                .filter(|s| s.contact.key == state.contact.key)
                .map(|s| &s.friction);
            state.friction.scale_motion(previous_friction, fraction)?;
        }
        Ok(())
    }

    fn query(
        &mut self,
        source: &mut impl ContactSource,
        previous: Option<&[Vec3]>,
        radius: Real,
        stage: ContactStage,
        settings: &SolverSettings,
    ) -> Result<(), ClothError> {
        if self.positions.iter().any(|v| !v.is_finite()) {
            return Err(ClothError::NonFiniteState);
        }
        self.contacts.clear();
        self.surface_contacts.clear();
        source.contacts(
            previous.unwrap_or(&self.reference),
            &self.positions,
            radius,
            stage,
            &mut self.contacts,
        )?;
        let work = if self.contact_settings.is_some_and(|s| s.self_collision) {
            &mut self.self_collision.as_mut().unwrap().work
        } else {
            &mut self.external_collision_work
        };
        source.surface_contacts_with_work(
            previous.unwrap_or(&self.reference),
            &self.positions,
            stage,
            &mut self.surface_contacts,
            work,
        )?;
        if self.contact_settings.is_some_and(|s| s.self_collision) {
            self.self_collision.as_mut().unwrap().generate(
                previous.unwrap_or(&self.reference),
                &self.positions,
                &mut self.surface_contacts,
                stage == ContactStage::Prediction
                    && self
                        .contact_settings
                        .is_some_and(|s| s.continuous_self_collision),
            )?;
        }
        Self::check_surface_capacity(
            self.contact_settings,
            &mut self.surface_high_water,
            self.surface_contacts.len(),
        )?;
        if self.contacts.len() + self.surface_contacts.len() > settings.max_contacts {
            return Err(ClothError::ContactBudgetExceeded {
                limit: settings.max_contacts,
            });
        }
        self.contacts.sort_by_key(|c| c.key);
        for (j, c) in self.contacts.iter().enumerate() {
            if c.key.particle as usize >= self.positions.len()
                || !c.point.is_finite()
                || !c.normal.is_finite()
                || (c.normal.length_squared() - 1.0).abs() > 1.0e-3
                || !c.surface_velocity.is_finite()
                || !c.friction.is_finite()
                || c.friction < 0.0
            {
                return Err(ClothError::External("invalid contact geometry".into()));
            }
            if j > 0 && self.contacts[j - 1].key == c.key {
                return Err(ClothError::External("duplicate contact key".into()));
            }
        }
        self.surface_contacts.sort_by_key(|c| c.key);
        for (i, c) in self.surface_contacts.iter().enumerate() {
            c.validate(self.positions.len())?;
            if i > 0 && self.surface_contacts[i - 1].key == c.key {
                return Err(ClothError::InvalidSurfaceContact("duplicate contact key"));
            }
        }
        Ok(())
    }
    /// Initial gap restoration uses the same normal support coupling as physical
    /// motion, with fresh multipliers and no friction/history updates. Residual
    /// penetration must not become a physical impulse in the prediction stage.
    fn stabilize_surface_contacts(&mut self, report: &mut StepReport) -> Result<bool, ClothError> {
        self.stabilization_lambda
            .resize(self.surface_contacts.len(), 0.0);
        self.stabilization_lambda.fill(0.0);
        self.rigid_support.resize(self.positions.len(), usize::MAX);
        self.rigid_support.fill(usize::MAX);
        for (index, c) in self.surface_contacts.iter().enumerate() {
            if let Some((particle, _)) = normal_block::single_particle(c)
                && self.weights[particle as usize] > 0.0
                && self.rigid_support[particle as usize] == usize::MAX
            {
                self.rigid_support[particle as usize] = index;
            }
        }
        let mut corrected = false;
        for (index, &c) in self.surface_contacts.iter().enumerate() {
            if c.gap(&self.positions) >= -c.separation * 1.0e-4
                && self.stabilization_lambda[index] == 0.0
            {
                continue;
            }
            let external = c
                .key
                .features
                .iter()
                .any(|f| matches!(f, crate::SurfaceFeature::External { .. }));
            let supports = std::array::from_fn(|i| {
                if c.weights[i] == 0.0 || external {
                    return None;
                }
                let support = self.rigid_support[c.particles[i] as usize];
                (support != usize::MAX).then(|| normal_block::Support {
                    contact: self.surface_contacts[support],
                    lambda: self.stabilization_lambda[support],
                })
            });
            if supports.iter().any(Option::is_some) {
                let (lambda, loads) = normal_block::project(
                    c,
                    &mut self.positions,
                    &self.weights,
                    self.stabilization_lambda[index],
                    supports,
                )?;
                self.stabilization_lambda[index] = lambda;
                for i in 0..4 {
                    if supports[i].is_some() {
                        self.stabilization_lambda[self.rigid_support[c.particles[i] as usize]] =
                            loads[i];
                    }
                }
            } else {
                c.project_validated(
                    &mut self.positions,
                    &self.weights,
                    &mut self.stabilization_lambda[index],
                )?;
            }
            report.stabilized_contacts += 1;
            corrected = true;
        }
        Ok(corrected)
    }

    /// Projects the queried surface contacts. With `restore`, only contacts
    /// short of their target gap are pushed outward, without friction or
    /// coupled support blocks: the pass that follows a shortened batch must
    /// not move any feature towards an obstacle.
    fn project_surface_contacts(
        &mut self,
        source: &mut impl ContactSource,
        h: Real,
        limit: usize,
        mesh: &crate::ClothMesh,
        restore: bool,
    ) -> Result<(), ClothError> {
        self.next_surface_states.clear();
        self.surface_solve_order.clear();
        self.rigid_support.resize(self.positions.len(), usize::MAX);
        self.rigid_support.fill(usize::MAX);
        let mut old = self.surface_states.iter().peekable();
        for &contact in &self.surface_contacts {
            while old.peek().is_some_and(|s| s.contact.key < contact.key) {
                self.next_surface_states.push(*old.next().unwrap());
            }
            let existing = old.peek().is_some_and(|s| s.contact.key == contact.key);
            let mut state = if existing {
                *old.next().unwrap()
            } else if let Ok(index) = self
                .previous_surface_states
                .binary_search_by_key(&contact.key, |state| state.contact.key)
            {
                self.previous_surface_states[index]
            } else {
                SurfaceContactState {
                    contact,
                    normal_lambda: 0.0,
                    friction: FrictionState::new(contact, &self.reference),
                }
            };
            if !existing {
                if !state.friction.compatible(&contact, &self.reference) {
                    state.friction = FrictionState::new(contact, &self.reference);
                }
                state
                    .friction
                    .ensure_material_frame(mesh, &self.reference)?;
                if state.friction.external() && contact.static_friction > 0.0 {
                    let material_contact = SurfaceContact {
                        particles: state.friction.support.particles,
                        weights: state.friction.support.weights,
                        ..contact
                    };
                    state.friction.anchor = source.transport_surface_anchor(
                        &material_contact,
                        state.friction.anchor,
                        h,
                    )?;
                }
                if !state.friction.anchor.is_finite() {
                    return Err(ClothError::NonFiniteState);
                }
            }
            if state.contact.normal.dot(contact.normal) < 0.9 {
                state.normal_lambda = 0.0;
            }
            state.contact = contact;
            let index = self.next_surface_states.len();
            self.surface_solve_order.push(index);
            if let Some((particle, _)) = normal_block::single_particle(&contact)
                && self.weights[particle as usize] > 0.0
                && self.rigid_support[particle as usize] == usize::MAX
            {
                self.rigid_support[particle as usize] = index;
            }
            self.next_surface_states.push(state);
            Self::check_surface_capacity(
                self.contact_settings,
                &mut self.surface_high_water,
                self.next_surface_states.len(),
            )?;
            if self.next_surface_states.len() + self.contact_states.len() > limit {
                return Err(ClothError::ContactBudgetExceeded { limit });
            }
        }
        self.next_surface_states.extend(old);
        // Check once after the merge; no state has been committed to the cloth.
        Self::check_surface_capacity(
            self.contact_settings,
            &mut self.surface_high_water,
            self.next_surface_states.len(),
        )?;
        if self.next_surface_states.len() + self.contact_states.len() > limit {
            return Err(ClothError::ContactBudgetExceeded { limit });
        }
        std::mem::swap(&mut self.surface_states, &mut self.next_surface_states);
        // Keep the existing contact order, but solve a deforming contact and
        // its selected independent rigid supports as one normal block. Support
        // forces can increase or release; no support inverse mass is zeroed.
        for &index in &self.surface_solve_order {
            let contact = self.surface_states[index].contact;
            if restore {
                if contact.gap(&self.positions) < 0.0 {
                    contact.project_validated(
                        &mut self.positions,
                        &self.weights,
                        &mut self.surface_states[index].normal_lambda,
                    )?;
                }
                continue;
            }
            let supports = std::array::from_fn(|i| {
                if contact.weights[i] == 0.0 || self.surface_states[index].friction.external() {
                    return None;
                }
                let support_index = self.rigid_support[contact.particles[i] as usize];
                (support_index != usize::MAX).then(|| {
                    let state = self.surface_states[support_index];
                    normal_block::Support {
                        contact: state.contact,
                        lambda: state.normal_lambda,
                    }
                })
            });
            if supports.iter().any(Option::is_some) {
                let (lambda, loads) = normal_block::project(
                    contact,
                    &mut self.positions,
                    &self.weights,
                    self.surface_states[index].normal_lambda,
                    supports,
                )?;
                self.surface_states[index].normal_lambda = lambda;
                for i in 0..4 {
                    if supports[i].is_some() {
                        let support_index = self.rigid_support[contact.particles[i] as usize];
                        let state = &mut self.surface_states[support_index];
                        state.normal_lambda = loads[i];
                        state
                            .friction
                            .limit_load(&mut self.positions, &self.weights, loads[i])?;
                    }
                }
            } else {
                contact.project_validated(
                    &mut self.positions,
                    &self.weights,
                    &mut self.surface_states[index].normal_lambda,
                )?;
            }
            let state = &mut self.surface_states[index];
            state.friction.project(
                &mut self.positions,
                &self.weights,
                contact.normal,
                state.normal_lambda,
            )?;
        }
        Ok(())
    }
    fn check_surface_capacity(
        settings: Option<ClothContactSettings>,
        high_water: &mut usize,
        count: usize,
    ) -> Result<(), ClothError> {
        if let Some(settings) = settings
            && count > settings.limits.retained_contacts
        {
            return Err(ClothError::CollisionBudgetExceeded {
                kind: CollisionBudgetKind::RetainedContacts,
                limit: settings.limits.retained_contacts,
            });
        }
        *high_water = (*high_water).max(count);
        Ok(())
    }
    /// Projects the queried particle contacts; `restore` as in
    /// `project_surface_contacts`.
    fn project_contacts(
        &mut self,
        radius: Real,
        limit: usize,
        restore: bool,
    ) -> Result<(), ClothError> {
        self.next_contact_states.clear();
        let mut old = self.contact_states.iter().peekable();
        for &c in &self.contacts {
            let i = c.key.particle as usize;
            if self.weights[i] == 0.0 {
                continue;
            }
            while old.peek().is_some_and(|s| s.contact.key < c.key) {
                self.next_contact_states.push(*old.next().unwrap());
            }
            if self.next_contact_states.len() >= limit {
                return Err(ClothError::ContactBudgetExceeded { limit });
            }
            let mut state = if old.peek().is_some_and(|s| s.contact.key == c.key) {
                *old.next().unwrap()
            } else {
                ContactState {
                    contact: c,
                    lambda: 0.0,
                }
            };
            if state.contact.normal.dot(c.normal) < 0.9 {
                state.lambda = 0.0;
            }
            state.contact = c;
            let constraint = (self.positions[i] - c.point).dot(c.normal) - radius;
            if !(restore && constraint >= 0.0) {
                let next = (state.lambda - constraint / self.weights[i]).max(0.0);
                self.positions[i] += c.normal * ((next - state.lambda) * self.weights[i]);
                state.lambda = next;
            }
            self.next_contact_states.push(state);
            if !self.positions[i].is_finite() {
                return Err(ClothError::NonFiniteState);
            }
        }
        self.next_contact_states.extend(old);
        if self.next_contact_states.len() > limit {
            return Err(ClothError::ContactBudgetExceeded { limit });
        }
        std::mem::swap(&mut self.contact_states, &mut self.next_contact_states);
        Ok(())
    }
}

#[cfg(test)]
mod contact_state_tests {
    use super::*;
    use crate::ContactKey;
    use std::collections::BTreeMap;

    #[test]
    fn clipped_material_friction_retains_only_the_applied_force_increment() {
        // A synthetic motion callback isolates the solver's shortening contract;
        // geometric CCD coverage is tested by the continuous-collision fixtures.
        struct ShortenOnce(bool);
        impl ContactSource for ShortenOnce {
            fn contacts(
                &mut self,
                _: &[Vec3],
                _: &[Vec3],
                _: Real,
                _: ContactStage,
                _: &mut Vec<Contact>,
            ) -> Result<(), ClothError> {
                Ok(())
            }
            fn motion_fraction(
                &mut self,
                _: crate::ContactMotion<'_>,
                _: &mut CollisionWork,
            ) -> Result<Real, ClothError> {
                Ok(if std::mem::replace(&mut self.0, false) {
                    0.25
                } else {
                    1.0
                })
            }
        }
        let reference = vec![
            Vec3::new(0.004, 0.01, 0.003),
            Vec3::new(-0.02, 0.0, -0.02),
            Vec3::new(0.02, 0.0, -0.02),
            Vec3::new(0.0, 0.0, 0.02),
        ];
        let mesh = crate::ClothMesh::new(reference.clone(), vec![[1, 2, 3], [0, 3, 2]]).unwrap();
        let contact = SurfaceContact {
            key: crate::SurfaceContactKey {
                other_cloth: None,
                features: [
                    crate::SurfaceFeature::Vertex(0),
                    crate::SurfaceFeature::Face(0),
                ],
            },
            particles: [0, 1, 2, 3],
            weights: [1.0, -0.2, -0.3, -0.5],
            normal: Vec3::Y,
            offset: Vec3::ZERO,
            surface_velocity: Vec3::ZERO,
            separation: 0.01,
            static_friction: 0.5,
            kinetic_friction: 0.2,
        };
        for slip in [0.0001, 0.02] {
            let mut solver = Solver::new();
            solver.reference = reference.clone();
            solver.positions = reference.clone();
            solver.weights = vec![0.5, 1.0 / 3.0, 0.2, 1.0 / 7.0];
            solver.external_continuous_motion = true;
            solver.positions[0].x += slip;
            let mut friction = FrictionState::new(contact, &reference);
            friction
                .project(&mut solver.positions, &solver.weights, Vec3::Y, 0.01)
                .unwrap();
            friction.finish(contact, &solver.positions, &mesh).unwrap();
            let before = SurfaceContactState {
                contact,
                normal_lambda: 0.01,
                friction,
            };
            solver.surface_states.push(before);
            solver.surface_contacts.push(contact);
            solver.capture_motion();

            // Represent a preceding elastic/support correction in the same batch.
            solver.positions[0].x += slip;
            let (sin, cos) = (0.15 as Real).sin_cos();
            for p in &mut solver.positions[1..] {
                *p = Vec3::new(cos * p.x + sin * p.z, p.y, -sin * p.x + cos * p.z);
            }
            let unconstrained = solver.positions.clone();
            solver
                .project_surface_contacts(&mut NoContacts, 1.0 / 240.0, 16, &mesh, false)
                .unwrap();
            let fraction = solver
                .accept_motion(&mut ShortenOnce(true), ContactStage::Iteration)
                .unwrap();
            assert_eq!(fraction, 0.25);
            solver.scale_contact_motion(fraction).unwrap();
            let after = solver.surface_states[0];
            let force_increment = contact.normal * (after.normal_lambda - before.normal_lambda)
                + after.friction.lambda
                - before.friction.lambda;
            assert!(force_increment.length() > 1e-6);
            for (i, unconstrained) in unconstrained.iter().enumerate() {
                let free =
                    solver.motion_start[i] + (*unconstrained - solver.motion_start[i]) * fraction;
                let applied_force = (solver.positions[i] - free) / solver.weights[i];
                let expected = contact.normal
                    * (after.normal_lambda - before.normal_lambda)
                    * contact.weights[i]
                    + after.friction.applied_force(i as u32)
                    - before.friction.applied_force(i as u32);
                assert!(applied_force.distance(expected) < 1e-7);
            }

            // Transport through the rejected trial must compose to the same
            // accepted material frame as direct transport from the old pose.
            let mut direct = before.friction;
            direct.sliding = false;
            direct.finish(contact, &solver.positions, &mesh).unwrap();
            let mut through_trial = after.friction;
            through_trial.sliding = false;
            through_trial
                .finish(contact, &solver.positions, &mesh)
                .unwrap();
            assert!(through_trial.anchor.distance(direct.anchor) < 1e-7);
        }
    }

    #[test]
    fn sorted_states_match_tree_oracle_with_changing_contacts() {
        let mut solver = Solver::new();
        solver.positions = vec![Vec3::ZERO; 3];
        solver.weights = vec![1.0, 0.0, 2.0];
        let mut expected_positions = solver.positions.clone();
        let mut expected = BTreeMap::<ContactKey, ContactState>::new();
        for iteration in 0..12 {
            solver.contacts.clear();
            // Alternate keys to cover insertions before/between/after old
            // states, disappearances, returns and changed contact normals.
            for particle in 0..3 {
                for external in 0..5 {
                    if (iteration + external) % 3 == 0 {
                        continue;
                    }
                    let normal = if iteration % 2 == 0 { Vec3::Y } else { Vec3::X };
                    solver.contacts.push(Contact {
                        key: ContactKey {
                            particle,
                            external,
                            feature: 0,
                        },
                        normal,
                        point: normal * (external as Real * 0.001),
                        surface_velocity: Vec3::ZERO,
                        friction: 0.3,
                    });
                }
            }
            // Independent pre-optimization map algorithm, preserving exact
            // operation order for projections and normal-change resets.
            for &c in &solver.contacts {
                let i = c.key.particle as usize;
                if solver.weights[i] == 0.0 {
                    continue;
                }
                let state = expected.entry(c.key).or_insert(ContactState {
                    contact: c,
                    lambda: 0.0,
                });
                if state.contact.normal.dot(c.normal) < 0.9 {
                    state.lambda = 0.0;
                }
                state.contact = c;
                let constraint = (expected_positions[i] - c.point).dot(c.normal) - 0.005;
                let next = (state.lambda - constraint / solver.weights[i]).max(0.0);
                expected_positions[i] += c.normal * ((next - state.lambda) * solver.weights[i]);
                state.lambda = next;
            }
            solver.project_contacts(0.005, 10, false).unwrap();
            assert_eq!(solver.positions, expected_positions);
            assert_eq!(solver.contact_states.len(), expected.len());
            for (actual, (key, oracle)) in solver.contact_states.iter().zip(&expected) {
                assert_eq!(actual.contact.key, *key);
                assert_eq!(actual.contact.normal, oracle.contact.normal);
                assert_eq!(actual.lambda, oracle.lambda);
            }
        }
        // A new key exceeds the cumulative budget even though this individual
        // query only has one contact. Disappeared states still count.
        solver.contacts.truncate(1);
        solver.contacts[0].key.external = 99;
        assert!(matches!(
            solver.project_contacts(0.005, 10, false),
            Err(ClothError::ContactBudgetExceeded { limit: 10 })
        ));
    }
}
