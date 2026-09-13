use super::friction_frame::{MaterialFrame, double, side_vertices, wide};
use super::friction_stencil::{Applied, Stencil, traction};
use crate::{ClothError, ClothMesh, Real, SurfaceContact, SurfaceFeature, Vec3};

/// Material witnesses remain fixed during a sticking contact. The normal
/// manifold may refresh its closest-point weights independently.
#[derive(Debug, Clone, Copy)]
pub(crate) struct FrictionState {
    pub support: SurfaceContact,
    pub anchor: Vec3,
    pub lambda: Vec3,
    pub sliding: bool,
    material_frame: Option<MaterialFrame>,
    applied: Applied,
}

impl FrictionState {
    pub fn new(contact: SurfaceContact, reference: &[Vec3]) -> Self {
        Self {
            support: contact,
            anchor: contact.relative(reference),
            lambda: Vec3::ZERO,
            sliding: false,
            material_frame: MaterialFrame::from_contact(contact, reference),
            applied: Applied::default(),
        }
    }

    pub fn reset_step(&mut self) {
        self.lambda = Vec3::ZERO;
        self.applied = Applied::default();
    }

    #[cfg(test)]
    pub fn applied_force(&self, particle: u32) -> Vec3 {
        Vec3::from_array(self.applied.force(particle).to_array().map(|x| x as Real))
    }

    pub fn scale_motion(
        &mut self,
        before: Option<&Self>,
        fraction: Real,
    ) -> Result<(), ClothError> {
        let base = before.map_or(Vec3::ZERO, |s| s.lambda);
        let fraction_wide = wide(fraction);
        let lambda = double(base) * (1.0 - fraction_wide) + double(self.lambda) * fraction_wide;
        let lambda = Vec3::from_array(lambda.to_array().map(|x| x as Real));
        if !lambda.is_finite() {
            return Err(ClothError::NonFiniteState);
        }
        let applied = self
            .applied
            .interpolate(before.map(|s| &s.applied), fraction_wide)?;
        self.applied = applied;
        self.lambda = lambda;
        Ok(())
    }

    pub fn external(&self) -> bool {
        self.support
            .key
            .features
            .iter()
            .any(|f| matches!(f, SurfaceFeature::External { .. }))
    }

    pub fn ensure_material_frame(
        &mut self,
        mesh: &ClothMesh,
        reference: &[Vec3],
    ) -> Result<(), ClothError> {
        if self.material_frame.is_none() && !self.external() && self.support.static_friction > 0.0 {
            let (vertices, count) = side_vertices(self.support);
            let vertices = mesh.material_triangle(&vertices[..count]).ok_or(
                ClothError::InvalidSurfaceContact("missing friction material triangle"),
            )?;
            self.material_frame = Some(MaterialFrame::new(vertices, reference).ok_or(
                ClothError::InvalidSurfaceContact("degenerate friction material triangle"),
            )?);
        }
        Ok(())
    }

    fn transport_material_frame(&mut self, positions: &[Vec3]) -> Result<(), ClothError> {
        if let Some(frame) = &mut self.material_frame {
            self.anchor = frame.transport(self.anchor, positions).ok_or(
                ClothError::InvalidSurfaceContact("degenerate friction material triangle"),
            )?;
        }
        if !self.anchor.is_finite() {
            return Err(ClothError::NonFiniteState);
        }
        Ok(())
    }

    /// Overlap recovery is not physical slip or support rotation. First align
    /// history with the old committed pose, then shift its reference explicitly.
    pub fn restore_reference(&mut self, before: &[Vec3], after: &[Vec3]) -> Result<(), ClothError> {
        self.transport_material_frame(before)?;
        self.anchor += self.support.relative(after) - self.support.relative(before);
        if let Some(frame) = &mut self.material_frame {
            frame
                .reset_basis(after)
                .ok_or(ClothError::InvalidSurfaceContact(
                    "degenerate friction material triangle",
                ))?;
        }
        if !self.anchor.is_finite() {
            return Err(ClothError::NonFiniteState);
        }
        Ok(())
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
            let ratio = wide(self.support.kinetic_friction * normal_lambda / length);
            self.applied
                .replace(self.applied.scaled(ratio), positions, inverse_masses)?;
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
        self.transport_material_frame(positions)?;
        self.support.normal = normal;
        let relative = self.support.relative(positions) - self.anchor;
        if !relative.is_finite() || !normal_lambda.is_finite() || normal_lambda < 0.0 {
            return Err(ClothError::NonFiniteState);
        }
        let stencil = Stencil::new(self.support, self.material_frame.as_ref(), positions)?;
        let (mass, old) = stencil.metric(inverse_masses, &self.applied);
        let (next, sliding) = traction(
            mass,
            |direction| stencil.directional_mass(inverse_masses, direction),
            old - double(relative),
            double(normal),
            wide(self.support.static_friction) * wide(normal_lambda),
            wide(self.support.kinetic_friction) * wide(normal_lambda),
        )?;
        let lambda = Vec3::from_array(next.to_array().map(|x| x as Real));
        if !lambda.is_finite() {
            return Err(ClothError::NonFiniteState);
        }
        self.applied
            .replace(stencil.forces(next), positions, inverse_masses)?;
        self.lambda = lambda;
        self.sliding = sliding;
        Ok(())
    }

    /// Sliding starts the next step at its current material witnesses. Sticking
    /// keeps the anchor. Multipliers are reset, not unapplied warm-start forces.
    pub fn finish(
        &mut self,
        contact: SurfaceContact,
        positions: &[Vec3],
        mesh: &ClothMesh,
    ) -> Result<(), ClothError> {
        if self.sliding {
            *self = Self::new(contact, positions);
            self.ensure_material_frame(mesh, positions)?;
        } else {
            self.transport_material_frame(positions)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{SurfaceContactKey, SurfaceFeature};

    #[test]
    fn unrepresentable_tangent_multiplier_fails_before_applying_positions() {
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
            static_friction: 4.0,
            kinetic_friction: 1.0,
            ..pair()
        };
        let reference = [Vec3::Y * 0.01];
        let mut state = FrictionState::new(c, &reference);
        let original = [reference[0] + Vec3::X * 10.0];
        let mut positions = original;
        let result = state.project(&mut positions, &[Real::MIN_POSITIVE], Vec3::Y, Real::MAX);
        assert!(
            matches!(result, Err(ClothError::NonFiniteState)),
            "{result:?}, lambda={:?}",
            state.lambda
        );
        assert_eq!(positions, original);
        assert_eq!(state.applied.force(0), glam::DVec3::ZERO);
        assert_eq!(state.lambda, Vec3::ZERO);
    }

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
    #[test]
    fn sticking_self_contact_is_objective_under_common_tangential_spin() {
        let c = pair();
        let reference = [
            Vec3::new(0.004, 0.01, 0.003),
            Vec3::new(-0.02, 0.0, -0.02),
            Vec3::new(0.02, 0.0, -0.02),
            Vec3::new(0.0, 0.0, 0.02),
        ];
        let mut state = FrictionState::new(c, &reference);
        let (sin, cos) = (0.7 as Real).sin_cos();
        let rotate = |p: Vec3| Vec3::new(cos * p.x + sin * p.z, p.y, -sin * p.x + cos * p.z);
        let expected = reference.map(rotate);
        let mut positions = expected;
        state
            .project(&mut positions, &[1.0; 4], Vec3::Y, 0.1)
            .unwrap();
        let error = positions
            .iter()
            .zip(expected)
            .map(|(a, b)| a.distance(b))
            .fold(0.0 as Real, Real::max);
        assert!(
            error < 1e-7,
            "common rotation added phantom slip correction: {error}"
        );
        assert!(state.lambda.length() < 1e-7);
    }

    fn surface_reference() -> [Vec3; 4] {
        [
            Vec3::new(0.004, 0.01, 0.003),
            Vec3::new(-0.02, 0.0, -0.02),
            Vec3::new(0.02, 0.0, -0.02),
            Vec3::new(0.0, 0.0, 0.02),
        ]
    }

    fn rotate(p: Vec3, rotation: glam::DQuat) -> Vec3 {
        #[cfg(not(feature = "f64"))]
        let p = p.as_dvec3();
        Vec3::from_array((rotation * p).to_array().map(|x| x as Real))
    }

    #[test]
    fn common_rigid_motion_preserves_material_points_for_general_rotations() {
        for axis in [
            glam::DVec3::X,
            glam::DVec3::Y,
            glam::DVec3::new(1.0, 2.0, 3.0).normalize(),
        ] {
            for angle in [-0.7, 0.3, 1.2] {
                let rotation = glam::DQuat::from_axis_angle(axis, angle);
                let rotate = |p| rotate(p, rotation);
                let c = pair();
                let reference = surface_reference();
                let mut state = FrictionState::new(c, &reference);
                let expected = reference.map(|p| rotate(p) + Vec3::new(2.0, -3.0, 4.0));
                let mut positions = expected;
                state
                    .project(&mut positions, &[1.0, 2.0, 3.0, 4.0], rotate(c.normal), 0.1)
                    .unwrap();
                for (actual, expected) in positions.iter().zip(expected) {
                    assert!(
                        actual.distance(expected) < 1e-6,
                        "rotation {axis:?}/{angle}: {actual:?} != {expected:?}"
                    );
                }
                assert!(state.lambda.length() < 1e-6);
            }
        }
    }

    #[test]
    fn changing_the_contact_normal_alone_does_not_move_material_witnesses() {
        let reference = surface_reference();
        let mut state = FrictionState::new(pair(), &reference);
        let mut positions = reference;
        state
            .project(
                &mut positions,
                &[1.0; 4],
                Vec3::new(0.15, 0.98, 0.05).normalize(),
                0.1,
            )
            .unwrap();
        assert_eq!(positions, reference);
        assert_eq!(state.lambda, Vec3::ZERO);
    }

    #[test]
    fn vertex_and_edge_contacts_keep_their_incident_material_orientation() {
        let reference = [
            Vec3::new(0.004, 0.01, 0.003),
            Vec3::new(0.024, 0.01, 0.003),
            Vec3::new(0.004, 0.01, 0.023),
            Vec3::ZERO,
            Vec3::X * 0.02,
            Vec3::Z * 0.02,
        ];
        let mesh = ClothMesh::new(reference.to_vec(), vec![[0, 1, 2], [3, 4, 5]]).unwrap();
        for edge_contact in [false, true] {
            let c = if edge_contact {
                SurfaceContact {
                    key: SurfaceContactKey {
                        other_cloth: None,
                        features: [SurfaceFeature::Edge([0, 1]), SurfaceFeature::Edge([3, 4])],
                    },
                    particles: [0, 1, 3, 4],
                    weights: [0.5, 0.5, -0.5, -0.5],
                    ..pair()
                }
            } else {
                SurfaceContact {
                    key: SurfaceContactKey {
                        other_cloth: None,
                        features: [SurfaceFeature::Vertex(0), SurfaceFeature::Vertex(3)],
                    },
                    particles: [0, 3, 0, 0],
                    weights: [1.0, -1.0, 0.0, 0.0],
                    ..pair()
                }
            };
            let mut state = FrictionState::new(c, &reference);
            state.ensure_material_frame(&mesh, &reference).unwrap();
            let (sin, cos) = (0.7 as Real).sin_cos();
            let rotate = |p: Vec3| Vec3::new(cos * p.x + sin * p.z, p.y, -sin * p.x + cos * p.z);
            let expected = reference.map(rotate);
            let mut positions = expected;
            state
                .project(&mut positions, &[1.0; 6], Vec3::Y, 0.1)
                .unwrap();
            for (actual, expected) in positions.iter().zip(expected) {
                assert!(
                    actual.distance(expected) < 1e-7,
                    "edge={edge_contact}: {actual:?} != {expected:?}"
                );
            }
            assert!(state.lambda.length() < 1e-7);
        }
    }

    #[test]
    fn recovery_changes_the_reference_without_becoming_physical_slip() {
        let c = pair();
        let before = surface_reference();
        let mut after = before;
        after[0] += Vec3::Y * 0.0001;
        after[2].z += 0.0002;
        let mut state = FrictionState::new(c, &before);
        state.restore_reference(&before, &after).unwrap();
        let expected = after;
        state.project(&mut after, &[1.0; 4], Vec3::Y, 0.1).unwrap();
        assert_eq!(after, expected);
        assert_eq!(state.lambda, Vec3::ZERO);
    }

    #[test]
    fn edge_material_frame_applies_and_releases_its_additional_support_vertex() {
        let reference = [
            Vec3::Y * 0.01,
            Vec3::new(0.02, 0.01, 0.0),
            Vec3::new(0.0, 0.01, 0.02),
            Vec3::ZERO,
            Vec3::X * 0.02,
            Vec3::Z * 0.02,
        ];
        let mesh = ClothMesh::new(reference.to_vec(), vec![[0, 1, 2], [3, 4, 5]]).unwrap();
        let contact = SurfaceContact {
            key: SurfaceContactKey {
                other_cloth: None,
                features: [SurfaceFeature::Edge([0, 1]), SurfaceFeature::Edge([3, 4])],
            },
            particles: [0, 1, 3, 4],
            weights: [0.5, 0.5, -0.5, -0.5],
            ..pair()
        };
        let mut state = FrictionState::new(contact, &reference);
        state.ensure_material_frame(&mesh, &reference).unwrap();
        let inverse: [Real; 6] = [1.0, 0.5, 0.25, 0.2, 0.3, 0.4];
        let mut positions = reference;
        positions[0].z += 0.005;
        positions[1].z += 0.005;
        let before = positions;
        state
            .project(&mut positions, &inverse, Vec3::Y, 0.001)
            .unwrap();
        assert!(state.sliding);
        assert!(
            positions[5].distance(before[5]) > 1e-6,
            "incident vertex received no moment correction"
        );
        let force: Vec3 = (0..6)
            .map(|i| (positions[i] - before[i]) / inverse[i])
            .sum();
        assert!(force.length() < 1e-7);
        // A shortened motion must retract only the accepted fraction, including
        // the incident vertex absent from the four-particle normal contact.
        for (p, &start) in positions.iter_mut().zip(&before) {
            *p = start + (*p - start) * 0.25;
        }
        state.scale_motion(None, 0.25).unwrap();
        state.limit_load(&mut positions, &inverse, 0.0).unwrap();
        for (&p, &expected) in positions.iter().zip(&before) {
            assert!(p.distance(expected) < 1e-7);
        }
        assert_eq!(state.lambda, Vec3::ZERO);
    }

    #[test]
    fn material_frame_preserves_slip_covariance_momentum_and_dissipation() {
        let reference = surface_reference();
        let masses: [Real; 4] = [2.0, 3.0, 5.0, 7.0];
        let inverse = masses.map(|m| 1.0 / m);
        let rotation =
            glam::DQuat::from_axis_angle(glam::DVec3::new(1.0, 2.0, 3.0).normalize(), 0.7);
        let rotate = |p| rotate(p, rotation);
        for slip in [0.0001, 0.02] {
            let mut start = reference;
            start[0] += Vec3::X * slip;
            let mut positions = start;
            let mut rotated_positions = start.map(rotate);
            let mut state = FrictionState::new(pair(), &reference);
            let mut rotated_state = FrictionState::new(pair(), &reference);
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
            for _ in 0..8 {
                state
                    .project(&mut positions, &inverse, Vec3::Y, 0.01)
                    .unwrap();
                rotated_state
                    .project(&mut rotated_positions, &inverse, rotate(Vec3::Y), 0.01)
                    .unwrap();
                for (p, rotated) in positions.iter().zip(rotated_positions) {
                    assert!(rotate(*p).distance(rotated) < 1e-7);
                }
                assert!(rotate(state.lambda).distance(rotated_state.lambda) < 1e-7);
                assert!(momentum(&positions).distance(momentum(&start)) < 1e-7);
                assert!(energy(&positions) < energy(&start));
            }
            assert_eq!(state.sliding, slip > 0.001);
            if state.sliding {
                assert!((state.lambda.length() - 0.002).abs() < 1e-7);
            }
        }
    }

    #[test]
    fn frame_refresh_with_existing_load_reconciles_actual_tangent_displacement() {
        let reference = surface_reference();
        let inverse = [0.5, 1.0 / 3.0, 0.2, 1.0 / 7.0];
        let mut positions = reference;
        positions[0].x += 0.02;
        let mut state = FrictionState::new(pair(), &reference);
        state
            .project(&mut positions, &inverse, Vec3::Y, 0.01)
            .unwrap();
        let previous_lambda = state.lambda;
        let previous_forces: [Vec3; 4] = std::array::from_fn(|i| state.applied_force(i as u32));
        assert!(previous_lambda.length() > 0.001);

        // Another constraint rotates the support after a friction impulse has
        // already been applied. The next friction solve must account for the
        // old WORLD impulse rather than rotating it without moving particles.
        let rotation = glam::DQuat::from_rotation_y(0.4);
        for p in &mut positions[1..] {
            *p = rotate(*p, rotation);
        }
        let before = positions;
        state
            .project(&mut positions, &inverse, Vec3::Y, 0.01)
            .unwrap();
        for i in 0..4 {
            let force = (positions[i] - before[i]) / inverse[i];
            let expected = state.applied_force(i as u32) - previous_forces[i];
            assert!(force.distance(expected) < 1e-7);
        }
        let before_release = positions;
        let loaded_forces: [Vec3; 4] = std::array::from_fn(|i| state.applied_force(i as u32));
        state.limit_load(&mut positions, &inverse, 0.0).unwrap();
        for i in 0..4 {
            let force = (positions[i] - before_release[i]) / inverse[i];
            assert!(force.distance(-loaded_forces[i]) < 1e-7);
        }
        assert_eq!(state.lambda, Vec3::ZERO);
    }
}

#[cfg(test)]
mod traction_torque_probe {
    use super::*;

    #[test]
    fn material_frame_traction_balances_the_surface_offset_torque() {
        let reference = [
            Vec3::new(0.002, 0.01, 0.0),
            Vec3::new(-0.02, 0.0, -0.02),
            Vec3::new(0.02, 0.0, -0.02),
            Vec3::new(0.0, 0.0, 0.02),
        ];
        let contact = SurfaceContact {
            key: crate::SurfaceContactKey {
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
        };
        let mut state = FrictionState::new(contact, &reference);
        let inverse: [Real; 4] = [0.5, 1.0 / 3.0, 0.2, 1.0 / 7.0];
        let mut positions = reference;
        positions[0].x += 0.005;
        let before = positions;
        state
            .project(&mut positions, &inverse, Vec3::Y, 0.001)
            .unwrap();
        assert!(state.sliding);
        let forces: [Vec3; 4] = std::array::from_fn(|i| (positions[i] - before[i]) / inverse[i]);
        let force: Vec3 = forces.into_iter().sum();
        let torque: Vec3 = before
            .into_iter()
            .zip(forces)
            .map(|(p, f)| p.cross(f))
            .sum();
        assert!(force.length() < 1e-7, "force {force:?}");
        assert!(
            torque.length() < 1e-9,
            "material anchor rotation needs its force gradient; torque={torque:?}, tangent={:?}",
            state.lambda
        );
    }
}
