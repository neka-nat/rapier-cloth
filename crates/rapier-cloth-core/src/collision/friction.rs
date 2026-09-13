use crate::{ClothError, Real, SurfaceContact, SurfaceFeature, Vec3};

/// Material witnesses remain fixed during a sticking contact. The normal
/// manifold may refresh its closest-point weights independently.
#[derive(Debug, Clone, Copy)]
pub(crate) struct FrictionState {
    pub support: SurfaceContact,
    pub anchor: Vec3,
    pub lambda: Vec3,
    pub sliding: bool,
}

impl FrictionState {
    pub fn new(contact: SurfaceContact, reference: &[Vec3]) -> Self {
        Self {
            support: contact,
            anchor: contact.relative(reference),
            lambda: Vec3::ZERO,
            sliding: false,
        }
    }

    pub fn external(&self) -> bool {
        self.support
            .key
            .features
            .iter()
            .any(|f| matches!(f, SurfaceFeature::External { .. }))
    }

    /// A coupled normal block can unload this support after its tangent update.
    /// Retract any now-unsupported tangent multiplier and its actual displacement.
    pub fn limit_load(
        &mut self,
        positions: &mut [Vec3],
        inverse_masses: &[Real],
        normal_lambda: Real,
    ) -> Result<(), ClothError> {
        let length = self.lambda.length();
        let coefficient = if self.sliding {
            self.support.kinetic_friction
        } else {
            self.support.static_friction
        };
        let limit = coefficient * normal_lambda;
        if !length.is_finite() || !limit.is_finite() || limit < 0.0 {
            return Err(ClothError::NonFiniteState);
        }
        if length > limit {
            let next = self.lambda * (self.support.kinetic_friction * normal_lambda / length);
            self.support
                .apply(positions, inverse_masses, next - self.lambda);
            self.lambda = next;
            self.sliding = true;
        }
        Ok(())
    }

    /// Preserve nearby material witnesses, not an arbitrarily distant point on
    /// the same triangle/edge. Compare each deforming side independently: their
    /// relative vector alone would miss a simultaneous shift of both witnesses.
    pub fn compatible(&self, contact: &SurfaceContact, positions: &[Vec3]) -> bool {
        if self.support.key != contact.key
            || self.support.normal.dot(contact.normal) < 0.9
            || self.support.static_friction != contact.static_friction
            || self.support.kinetic_friction != contact.kinetic_friction
            || self.support.separation != contact.separation
        {
            return false;
        }
        for positive in [false, true] {
            let point = |c: &SurfaceContact| {
                let mut sum = Vec3::ZERO;
                for i in 0..4 {
                    let w = c.weights[i];
                    if w != 0.0 && (w > 0.0) == positive {
                        sum += positions[c.particles[i] as usize] * w;
                    }
                }
                sum
            };
            if point(&self.support).distance(point(contact)) > contact.separation * 2.0 {
                return false;
            }
        }
        true
    }

    /// Normal-load multipliers are in position units (force times h squared).
    /// The accumulated tangent multiplier has the same units. Replacing, rather
    /// than adding another full Coulomb impulse, makes repeated passes bounded.
    pub fn project(
        &mut self,
        positions: &mut [Vec3],
        inverse_masses: &[Real],
        normal: Vec3,
        normal_lambda: Real,
    ) -> Result<(), ClothError> {
        if !self.external() {
            let a = self.support.normal;
            let axis = a.cross(normal);
            let v = self.anchor;
            if a.dot(normal) > -0.9 {
                self.anchor = v + axis.cross(v) + axis.cross(axis.cross(v)) / (1.0 + a.dot(normal));
            } else {
                // A reversed manifold is a new contact, not a sticking bond.
                self.anchor = self.support.relative(positions);
            }
        }
        self.support.normal = normal;
        let inverse_mass = self.support.inverse_mass(inverse_masses);
        if inverse_mass == 0.0 {
            self.lambda = Vec3::ZERO;
            return Ok(());
        }
        let relative = self.support.relative(positions) - self.anchor;
        let tangent = relative - normal * relative.dot(normal);
        let trial = self.lambda - tangent / inverse_mass;
        let trial = trial - normal * trial.dot(normal);
        let length = trial.length();
        let static_limit = self.support.static_friction * normal_lambda;
        let kinetic_limit = self.support.kinetic_friction * normal_lambda;
        if !relative.is_finite()
            || !trial.is_finite()
            || !length.is_finite()
            || !static_limit.is_finite()
            || !kinetic_limit.is_finite()
            || !inverse_mass.is_finite()
        {
            return Err(ClothError::NonFiniteState);
        }
        let sliding = length > static_limit;
        let next = if sliding && length > 0.0 {
            trial * (kinetic_limit / length)
        } else {
            trial
        };
        self.support
            .apply(positions, inverse_masses, next - self.lambda);
        self.lambda = next;
        self.sliding = sliding;
        Ok(())
    }

    /// Sliding starts the next step at its current material witnesses. Sticking
    /// keeps the anchor. Multipliers are reset, not unapplied warm-start forces.
    pub fn finish(&mut self, contact: SurfaceContact, positions: &[Vec3]) {
        if self.sliding {
            *self = Self::new(contact, positions);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{SurfaceContactKey, SurfaceFeature};

    fn pair() -> SurfaceContact {
        SurfaceContact {
            key: SurfaceContactKey {
                other_cloth: None,
                features: [SurfaceFeature::Vertex(0), SurfaceFeature::Face(0)],
            },
            particles: [0, 1, 2, 3],
            weights: [1.0, -0.2, -0.3, -0.5],
            normal: Vec3::Y,
            offset: Vec3::ZERO,
            surface_velocity: Vec3::ZERO,
            separation: 0.01,
            static_friction: 0.5,
            kinetic_friction: 0.2,
        }
    }

    #[test]
    fn two_deforming_sides_conserve_momentum_and_dissipate_slip_energy() {
        let c = pair();
        let reference = [Vec3::Y * 0.01, -Vec3::X, Vec3::X, Vec3::ZERO];
        let masses: [Real; 4] = [2.0, 3.0, 5.0, 7.0];
        let inverse = masses.map(|m| 1.0 / m);
        let mut positions = reference;
        positions[0].x += 0.02;
        let momentum = |p: &[Vec3; 4]| {
            (0..4)
                .map(|i| (p[i] - reference[i]) * masses[i])
                .sum::<Vec3>()
        };
        let energy = |p: &[Vec3; 4]| {
            (0..4)
                .map(|i| (p[i] - reference[i]).length_squared() * masses[i] * 0.5)
                .sum::<Real>()
        };
        let before_momentum = momentum(&positions);
        let before_energy = energy(&positions);
        let mut state = FrictionState::new(c, &reference);
        state
            .project(&mut positions, &inverse, Vec3::Y, 0.01)
            .unwrap();
        assert!(state.sliding);
        assert!((state.lambda.length() - 0.002).abs() < 1.0e-7);
        assert!(momentum(&positions).distance(before_momentum) < 1.0e-6);
        assert!(energy(&positions) < before_energy);
        let after = positions;
        for _ in 0..7 {
            state
                .project(&mut positions, &inverse, Vec3::Y, 0.01)
                .unwrap();
        }
        for (a, b) in positions.iter().zip(after) {
            assert!(a.distance(b) < 1.0e-7);
        }
    }

    #[test]
    fn closest_witness_refresh_does_not_replace_sticking_material_weights() {
        let c = pair();
        let reference = [Vec3::Y * 0.01, -Vec3::X * 0.01, Vec3::X * 0.01, Vec3::ZERO];
        let mut state = FrictionState::new(c, &reference);
        let mut refreshed = c;
        refreshed.weights = [1.0, -0.21, -0.29, -0.5];
        assert!(state.compatible(&refreshed, &reference));
        let mut positions = reference;
        positions[0].x += 0.0001;
        state
            .project(&mut positions, &[1.0; 4], c.normal, 0.01)
            .unwrap();
        assert!(!state.sliding);
        assert!((c.relative(&positions) - state.anchor).x.abs() < 1.0e-7);
        assert_eq!(state.support.weights, c.weights);
        let mut far = reference;
        far[1] = -Vec3::X * 100.0;
        assert!(!state.compatible(&refreshed, &far));
        refreshed.static_friction = 0.7;
        assert!(!state.compatible(&refreshed, &reference));
    }

    #[test]
    fn unloading_retracts_friction_and_restores_free_slip_on_release() {
        let c = SurfaceContact {
            key: SurfaceContactKey {
                other_cloth: None,
                features: [
                    SurfaceFeature::Vertex(0),
                    SurfaceFeature::External {
                        object: 0,
                        feature: 0,
                    },
                ],
            },
            particles: [0; 4],
            weights: [1.0, 0.0, 0.0, 0.0],
            ..pair()
        };
        let mut positions = [Vec3::Y * 0.01];
        let mut state = FrictionState::new(c, &positions);
        positions[0].x = 0.004;
        state
            .project(&mut positions, &[1.0], Vec3::Y, 0.02)
            .unwrap();
        assert!(positions[0].x.abs() < 1e-7);
        state.limit_load(&mut positions, &[1.0], 0.002).unwrap();
        assert!((positions[0].x - 0.0036).abs() < 1e-7);
        assert!((state.lambda.length() - 0.0004).abs() < 1e-7);
        state.limit_load(&mut positions, &[1.0], 0.0).unwrap();
        assert!(positions[0].distance(Vec3::new(0.004, 0.01, 0.0)) < 1e-7);
        assert_eq!(state.lambda, Vec3::ZERO);
    }
}
