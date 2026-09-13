use crate::{ClothError, ClothHandle, CollisionWork, Real, Vec3};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct ContactKey {
    pub particle: u32,
    pub external: u64,
    pub feature: u32,
}

/// A unilateral plane constraint n dot (x - point) >= particle radius.
/// normal is a unit vector from the obstacle towards the cloth. All vectors
/// are in the application's world frame; friction is already combined.
#[derive(Debug, Clone, Copy)]
pub struct Contact {
    pub key: ContactKey,
    pub normal: Vec3,
    pub point: Vec3,
    pub surface_velocity: Vec3,
    pub friction: Real,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactStage {
    Stabilization,
    Prediction,
    Iteration,
    Final,
}

/// A narrow boundary for refreshing external constraints. Output is cleared
/// by the solver before every call. Failure aborts the entire cloth substep.
/// Every stage, including prediction, may be queried repeatedly for trial poses
/// in the same substep. A source must retain unique keys when caching witnesses.
pub trait ContactSource {
    fn contacts(
        &mut self,
        previous: &[Vec3],
        positions: &[Vec3],
        radius: Real,
        stage: ContactStage,
        out: &mut Vec<Contact>,
    ) -> Result<(), ClothError>;

    /// Optional contacts involving several cloth particles. The owner cloth is
    /// implicit; other cloth identities in keys must use generational handles.
    /// Output is cleared by the solver. The default preserves particle contacts.
    fn surface_contacts(
        &mut self,
        _previous: &[Vec3],
        _positions: &[Vec3],
        _stage: ContactStage,
        _out: &mut Vec<SurfaceContact>,
    ) -> Result<(), ClothError> {
        Ok(())
    }

    /// Surface callback with the same work counters used by built-in collision.
    /// Charge work against the owning cloth's configured limits before querying.
    /// The default forwards to the existing callback for source compatibility.
    fn surface_contacts_with_work(
        &mut self,
        previous: &[Vec3],
        positions: &[Vec3],
        stage: ContactStage,
        out: &mut Vec<SurfaceContact>,
        _work: &mut CollisionWork,
    ) -> Result<(), ClothError> {
        self.surface_contacts(previous, positions, stage, out)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SurfaceFeature {
    Vertex(u32),
    /// Canonical ascending vertex order.
    Edge([u32; 2]),
    Face(u32),
    External {
        object: u64,
        feature: u32,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct SurfaceContactKey {
    /// None for a self-contact or a rigid external feature. State ownership
    /// supplies the current cloth identity; a different cloth retains generation.
    pub other_cloth: Option<ClothHandle>,
    pub features: [SurfaceFeature; 2],
}

/// A surface constraint with relative position `sum(weights[i]*x[i])-offset`.
/// Vertex-face weights are `[1, -b0, -b1, -b2]`; edge-edge weights are
/// `[1-s, s, -(1-t), -t]`. Unused slots have zero weights. Each used particle
/// appears once. Normal and offset are in the application world frame.
#[derive(Debug, Clone, Copy)]
pub struct SurfaceContact {
    pub key: SurfaceContactKey,
    pub particles: [u32; 4],
    pub weights: [Real; 4],
    pub normal: Vec3,
    pub offset: Vec3,
    pub surface_velocity: Vec3,
    pub separation: Real,
    pub static_friction: Real,
    pub kinetic_friction: Real,
}
impl SurfaceContact {
    pub fn validate(&self, particle_count: usize) -> Result<(), ClothError> {
        if !self.normal.is_finite()
            || (self.normal.length_squared() - 1.0).abs() > 1.0e-3
            || !self.offset.is_finite()
            || !self.surface_velocity.is_finite()
            || !self.separation.is_finite()
            || self.separation <= 0.0
            || !self.static_friction.is_finite()
            || !self.kinetic_friction.is_finite()
            || self.kinetic_friction < 0.0
            || self.static_friction < self.kinetic_friction
            || self.weights.iter().any(|w| !w.is_finite())
            || self.weights.iter().all(|w| *w == 0.0)
            || self.key.other_cloth.is_some()
        {
            return Err(ClothError::InvalidSurfaceContact(
                "invalid geometry, material, or unsupported other cloth",
            ));
        }
        for i in 0..4 {
            if self.weights[i] == 0.0 {
                continue;
            }
            if self.particles[i] as usize >= particle_count {
                return Err(ClothError::InvalidParticle(self.particles[i]));
            }
            if (0..i).any(|j| self.weights[j] != 0.0 && self.particles[j] == self.particles[i]) {
                return Err(ClothError::InvalidSurfaceContact(
                    "duplicate support particle",
                ));
            }
        }
        for feature in self.key.features {
            if let SurfaceFeature::Edge([a, b]) = feature
                && a >= b
            {
                return Err(ClothError::InvalidSurfaceContact("noncanonical edge key"));
            }
        }
        Ok(())
    }
    pub(crate) fn relative(&self, values: &[Vec3]) -> Vec3 {
        let mut sum = Vec3::ZERO;
        for i in 0..4 {
            if self.weights[i] != 0.0 {
                sum += values[self.particles[i] as usize] * self.weights[i];
            }
        }
        sum
    }
    pub(crate) fn gap(&self, positions: &[Vec3]) -> Real {
        self.normal.dot(self.relative(positions) - self.offset) - self.separation
    }
    pub(crate) fn inverse_mass(&self, masses: &[Real]) -> Real {
        let mut sum = 0.0;
        for i in 0..4 {
            if self.weights[i] != 0.0 {
                sum += masses[self.particles[i] as usize] * self.weights[i].powi(2);
            }
        }
        sum
    }
    pub(crate) fn apply(&self, values: &mut [Vec3], masses: &[Real], impulse: Vec3) {
        for i in 0..4 {
            if self.weights[i] != 0.0 {
                let p = self.particles[i] as usize;
                values[p] += impulse * (masses[p] * self.weights[i]);
            }
        }
    }
    /// Apply a unilateral mass-weighted projection. Invalid/infeasible input
    /// leaves both positions and the caller-owned multiplier unchanged.
    pub fn project(
        &self,
        positions: &mut [Vec3],
        masses: &[Real],
        lambda: &mut Real,
    ) -> Result<Real, ClothError> {
        self.validate(positions.len())?;
        if masses.len() != positions.len()
            || !lambda.is_finite()
            || *lambda < 0.0
            || (0..4).any(|i| {
                self.weights[i] != 0.0
                    && (!positions[self.particles[i] as usize].is_finite()
                        || !masses[self.particles[i] as usize].is_finite()
                        || masses[self.particles[i] as usize] < 0.0)
            })
        {
            return Err(ClothError::InvalidSurfaceContact(
                "invalid state or inverse masses",
            ));
        }
        self.project_validated(positions, masses, lambda)
    }
    pub(crate) fn project_validated(
        &self,
        positions: &mut [Vec3],
        masses: &[Real],
        lambda: &mut Real,
    ) -> Result<Real, ClothError> {
        let gap = self.gap(positions);
        let inverse_mass = self.inverse_mass(masses);
        if !gap.is_finite() || !inverse_mass.is_finite() || inverse_mass < 0.0 {
            return Err(ClothError::NonFiniteState);
        }
        if inverse_mass == 0.0 {
            if gap < -self.separation * 1.0e-4 {
                return Err(ClothError::InfeasibleSurfaceContact);
            }
            return Ok(0.0);
        }
        let next = (*lambda - gap / inverse_mass).max(0.0);
        let delta = next - *lambda;
        let mut updated = [Vec3::ZERO; 4];
        for (i, p) in updated.iter_mut().enumerate() {
            if self.weights[i] != 0.0 {
                let index = self.particles[i] as usize;
                *p = positions[index] + self.normal * (delta * masses[index] * self.weights[i]);
                if !p.is_finite() {
                    return Err(ClothError::NonFiniteState);
                }
            }
        }
        if !next.is_finite() {
            return Err(ClothError::NonFiniteState);
        }
        for (i, p) in updated.iter().enumerate() {
            if self.weights[i] != 0.0 {
                positions[self.particles[i] as usize] = *p;
            }
        }
        *lambda = next;
        Ok(delta)
    }
}

/// Persistent physical state. Scratch solvers stage a copy; successful steps
/// commit it with the cloth. Keys are local to the owning cloth's lifetime.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SurfaceContactState {
    pub contact: SurfaceContact,
    pub normal_lambda: Real,
    pub tangent_reference: Vec3,
    pub tangent_lambda: Vec3,
    pub h: Real,
}
#[derive(Default)]
pub struct NoContacts;
impl ContactSource for NoContacts {
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
}

/// Coulomb-limited kinetic friction in velocity units. Cannot reverse slip.
pub fn friction_velocity(
    velocity: Vec3,
    surface_velocity: Vec3,
    normal: Vec3,
    normal_delta: Real,
    mu: Real,
) -> Vec3 {
    let relative = velocity - surface_velocity;
    let tangent = relative - normal * relative.dot(normal);
    let speed = tangent.length();
    if speed <= Real::MIN_POSITIVE {
        return velocity;
    }
    velocity - tangent * ((mu * normal_delta.max(0.0)).min(speed) / speed)
}
