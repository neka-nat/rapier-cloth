use crate::{
    Cloth, ClothError, Contact, ContactKey, ContactSource, ContactStage, NoContacts, Real,
    StepReport, Vec3,
    constraints::{
        bend::{angle_and_gradients, angle_difference},
        distance,
    },
    contact::friction_velocity,
};
use std::collections::BTreeMap;

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

#[derive(Clone, Debug)]
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
    contacts: Vec<Contact>,
    contact_states: BTreeMap<ContactKey, ContactState>,
    stretches: Vec<Real>,
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
        settings.validate(h)?;
        if !gravity.is_finite() {
            return Err(ClothError::InvalidParameter("gravity"));
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
        self.reference.clone_from(&cloth.positions);
        self.positions.clone_from(&cloth.positions);
        self.velocities.clone_from(&cloth.velocities);
        self.weights.clone_from(&cloth.inverse_masses);
        for t in &targets {
            if t.compliance == 0.0 {
                self.weights[t.particle as usize] = 0.0;
            }
        }
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
            if !corrected {
                break;
            }
        }
        self.reference.copy_from_slice(&self.positions);
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
        self.query(source, None, radius, ContactStage::Prediction, settings)?;
        self.project_contacts(radius, settings.max_contacts)?;
        let stretch_alpha = cloth.material.stretch_compliance / (h * h);
        let bend_alpha = cloth.material.bend_compliance / (h * h);
        for _ in 0..settings.iterations {
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
            self.query(source, None, radius, ContactStage::Iteration, settings)?;
            self.project_contacts(radius, settings.max_contacts)?;
        }
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
        for state in self.contact_states.values() {
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
        self.query(source, None, radius, ContactStage::Final, settings)?;
        for c in &self.contacts {
            let i = c.key.particle as usize;
            let depth = (radius - (self.positions[i] - c.point).dot(c.normal)).max(0.0);
            report.max_penetration = report.max_penetration.max(depth);
            if self.weights[i] == 0.0 && depth > radius * 0.2 {
                return Err(ClothError::ConflictingTarget(i as u32));
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
        }
        self.stretches.sort_unstable_by(Real::total_cmp);
        report.max_stretch = self.stretches.last().copied().unwrap_or(0.0);
        report.p95_stretch = self.stretches
            [((self.stretches.len() as Real * 0.95).ceil() as usize).saturating_sub(1)];
        for hinge in cloth.mesh.hinges() {
            let (angle, _) =
                angle_and_gradients(hinge.vertices.map(|i| self.positions[i as usize]))
                    .ok_or(ClothError::DegenerateConstraint)?;
            report.max_bend_error = report
                .max_bend_error
                .max(angle_difference(angle, hinge.rest_angle).abs());
        }
        for t in &targets {
            report.max_target_error = report
                .max_target_error
                .max(self.positions[t.particle as usize].distance(t.position));
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
        report.contacts = self.contact_states.len();
        report.iterations = settings.iterations;
        report.scratch_bytes = (self.positions.capacity()
            + self.prediction.capacity()
            + self.reference.capacity()
            + self.velocities.capacity()
            + self.target_lambda.capacity())
            * std::mem::size_of::<Vec3>()
            + (self.weights.capacity()
                + self.distance_lambda.capacity()
                + self.bend_lambda.capacity()
                + self.stretches.capacity())
                * std::mem::size_of::<Real>()
            + self.contacts.capacity() * std::mem::size_of::<Contact>();
        cloth.previous.copy_from_slice(&cloth.positions);
        std::mem::swap(&mut cloth.positions, &mut self.positions);
        std::mem::swap(&mut cloth.velocities, &mut self.velocities);
        Ok(report)
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
        source.contacts(
            previous.unwrap_or(&self.reference),
            &self.positions,
            radius,
            stage,
            &mut self.contacts,
        )?;
        if self.contacts.len() > settings.max_contacts {
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
        Ok(())
    }
    fn project_contacts(&mut self, radius: Real, limit: usize) -> Result<(), ClothError> {
        for &c in &self.contacts {
            let i = c.key.particle as usize;
            if self.weights[i] == 0.0 {
                continue;
            }
            if self.contact_states.len() >= limit && !self.contact_states.contains_key(&c.key) {
                return Err(ClothError::ContactBudgetExceeded { limit });
            }
            let state = self.contact_states.entry(c.key).or_insert(ContactState {
                contact: c,
                lambda: 0.0,
            });
            if state.contact.normal.dot(c.normal) < 0.9 {
                state.lambda = 0.0;
            }
            state.contact = c;
            let constraint = (self.positions[i] - c.point).dot(c.normal) - radius;
            let next = (state.lambda - constraint / self.weights[i]).max(0.0);
            self.positions[i] += c.normal * ((next - state.lambda) * self.weights[i]);
            state.lambda = next;
            if !self.positions[i].is_finite() {
                return Err(ClothError::NonFiniteState);
            }
        }
        Ok(())
    }
}
