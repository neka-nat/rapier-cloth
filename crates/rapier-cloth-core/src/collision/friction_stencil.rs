//! Bounded material-traction stencil and its actual applied particle forces.
use super::friction_frame::{MaterialFrame, double, wide};
use crate::{ClothError, Real, SurfaceContact, Vec3};
use glam::{DMat3, DVec3};

const CAPACITY: usize = 7; // Four contact vertices plus three frame vertices.

#[cfg(feature = "f64")]
fn narrow(v: DVec3) -> Vec3 {
    v
}
#[cfg(not(feature = "f64"))]
fn narrow(v: DVec3) -> Vec3 {
    v.as_vec3()
}

#[derive(Debug, Clone, Copy, Default)]
pub(super) struct Applied {
    ids: [u32; CAPACITY],
    forces: [DVec3; CAPACITY],
    count: usize,
}
impl Applied {
    pub fn force(&self, particle: u32) -> DVec3 {
        (0..self.count)
            .find(|&i| self.ids[i] == particle)
            .map_or(DVec3::ZERO, |i| self.forces[i])
    }
    fn insert(&mut self, particle: u32, force: DVec3) -> Result<(), ClothError> {
        if let Some(i) = (0..self.count).find(|&i| self.ids[i] == particle) {
            self.forces[i] += force;
        } else {
            if self.count == CAPACITY {
                return Err(ClothError::InvalidSurfaceContact(
                    "friction stencil capacity",
                ));
            }
            self.ids[self.count] = particle;
            self.forces[self.count] = force;
            self.count += 1;
        }
        Ok(())
    }
    pub fn scaled(mut self, scale: f64) -> Self {
        for f in &mut self.forces[..self.count] {
            *f *= scale;
        }
        self
    }
    pub fn interpolate(&self, before: Option<&Self>, fraction: f64) -> Result<Self, ClothError> {
        let mut next = self.scaled(fraction);
        if let Some(before) = before {
            for i in 0..before.count {
                next.insert(before.ids[i], before.forces[i] * (1.0 - fraction))?;
            }
        }
        if next.forces[..next.count].iter().any(|f| !f.is_finite()) {
            return Err(ClothError::NonFiniteState);
        }
        Ok(next)
    }
    pub fn replace(
        &mut self,
        next: Self,
        positions: &mut [Vec3],
        inverse: &[Real],
    ) -> Result<(), ClothError> {
        // Stage the union so a bad new force never leaves a partially applied update.
        let mut ids = [0; 2 * CAPACITY];
        let mut updated = [Vec3::ZERO; 2 * CAPACITY];
        let mut count = 0;
        for &particle in next.ids[..next.count].iter().chain(&self.ids[..self.count]) {
            if ids[..count].contains(&particle) {
                continue;
            }
            let i = particle as usize;
            let value = double(positions[i])
                + (next.force(particle) - self.force(particle)) * wide(inverse[i]);
            let value = narrow(value);
            if !value.is_finite() {
                return Err(ClothError::NonFiniteState);
            }
            ids[count] = particle;
            updated[count] = value;
            count += 1;
        }
        for i in 0..count {
            positions[ids[i] as usize] = updated[i];
        }
        *self = next;
        Ok(())
    }
}

pub(super) struct Stencil {
    ids: [u32; CAPACITY],
    jacobians: [DMat3; CAPACITY],
    count: usize,
}
impl Stencil {
    pub fn new(
        contact: SurfaceContact,
        frame: Option<&MaterialFrame>,
        positions: &[Vec3],
    ) -> Result<Self, ClothError> {
        let mut result = Self {
            ids: [0; CAPACITY],
            jacobians: [DMat3::ZERO; CAPACITY],
            count: 0,
        };
        for i in 0..4 {
            if contact.weights[i] != 0.0 {
                result.add(
                    contact.particles[i],
                    DMat3::IDENTITY * wide(contact.weights[i]),
                )?;
            }
        }
        if let Some(frame) = frame {
            let relative = (0..4)
                .filter(|&i| contact.weights[i] != 0.0)
                .map(|i| {
                    double(positions[contact.particles[i] as usize]) * wide(contact.weights[i])
                })
                .sum();
            for (particle, derivative) in frame.coordinate_gradients(relative, positions).ok_or(
                ClothError::InvalidSurfaceContact("degenerate friction material triangle"),
            )? {
                result.add(particle, -derivative)?;
            }
        }
        Ok(result)
    }
    fn add(&mut self, particle: u32, matrix: DMat3) -> Result<(), ClothError> {
        if let Some(i) = (0..self.count).find(|&i| self.ids[i] == particle) {
            self.jacobians[i] += matrix;
        } else {
            if self.count == CAPACITY {
                return Err(ClothError::InvalidSurfaceContact(
                    "friction stencil capacity",
                ));
            }
            self.ids[self.count] = particle;
            self.jacobians[self.count] = matrix;
            self.count += 1;
        }
        Ok(())
    }
    pub fn metric(&self, inverse: &[Real], old: &Applied) -> (DMat3, DVec3) {
        let mut mass = DMat3::ZERO;
        let mut previous = DVec3::ZERO;
        for i in 0..self.count {
            let w = wide(inverse[self.ids[i] as usize]);
            let j = self.jacobians[i];
            mass += j * j.transpose() * w;
            previous += j * old.force(self.ids[i]) * w;
        }
        (mass, previous)
    }
    pub fn directional_mass(&self, inverse: &[Real], direction: DVec3) -> f64 {
        // Retain a weak tangent direction even when its squared gradient is
        // lost while adding large entries to the Gram matrix.
        (0..self.count)
            .map(|i| {
                (self.jacobians[i].transpose() * direction).length_squared()
                    * wide(inverse[self.ids[i] as usize])
            })
            .sum()
    }
    pub fn forces(&self, traction: DVec3) -> Applied {
        let mut result = Applied {
            ids: self.ids,
            count: self.count,
            ..Default::default()
        };
        for i in 0..self.count {
            result.forces[i] = self.jacobians[i].transpose() * traction;
        }
        result
    }
}

/// Solve a 2D positive-semidefinite quadratic on the Coulomb disk. The static
/// trial uses the unconstrained solution; sliding uses a bounded secular solve.
pub(super) fn traction(
    mass: DMat3,
    directional_mass: impl Fn(DVec3) -> f64,
    rhs: DVec3,
    normal: DVec3,
    static_limit: f64,
    kinetic_limit: f64,
) -> Result<(DVec3, bool), ClothError> {
    let normal = normal.try_normalize().ok_or(ClothError::NonFiniteState)?;
    let axis = if normal.x.abs() <= normal.y.abs() && normal.x.abs() <= normal.z.abs() {
        DVec3::X
    } else if normal.y.abs() <= normal.z.abs() {
        DVec3::Y
    } else {
        DVec3::Z
    };
    let u = (axis - normal * normal.dot(axis)).normalize();
    let v = normal.cross(u);
    let a = u.dot(mass * u);
    let b = u.dot(mass * v);
    let c = v.dot(mass * v);
    if !mass.is_finite()
        || !rhs.is_finite()
        || !static_limit.is_finite()
        || !kinetic_limit.is_finite()
        || kinetic_limit < 0.0
        || static_limit < kinetic_limit
    {
        return Err(ClothError::NonFiniteState);
    }
    if a.max(c) <= 0.0 {
        return Ok((DVec3::ZERO, false));
    }
    let scale = a.abs().max(b.abs()).max(c.abs());
    let difference = a / scale - c / scale;
    let off_diagonal = 2.0 * (b / scale);
    let denominator = difference.abs() + difference.hypot(off_diagonal);
    let tangent = if denominator == 0.0 {
        0.0
    } else {
        off_diagonal / denominator
    };
    let cosine = 1.0 / (1.0 + tangent * tangent).sqrt();
    let sine = tangent * cosine;
    let (major, minor) = if a >= c {
        (u * cosine + v * sine, -u * sine + v * cosine)
    } else {
        (u * sine + v * cosine, -u * cosine + v * sine)
    };
    let values = [directional_mass(major), directional_mass(minor)];
    if values.iter().any(|v| !v.is_finite() || *v < 0.0) {
        return Err(ClothError::NonFiniteState);
    }
    let q = [major.dot(rhs), minor.dot(rhs)];
    let free = std::array::from_fn::<_, 2, _>(|i| {
        if values[i] > 0.0 {
            q[i] / values[i]
        } else if q[i] == 0.0 {
            0.0
        } else {
            f64::INFINITY
        }
    });
    if free[0].hypot(free[1]) <= static_limit {
        return Ok((major * free[0] + minor * free[1], false));
    }
    if kinetic_limit == 0.0 {
        return Ok((DVec3::ZERO, true));
    }
    let mut low = 0.0;
    let mut high = q[0].hypot(q[1]) / kinetic_limit;
    if !high.is_finite() || high <= 0.0 {
        return Err(ClothError::NonFiniteState);
    }
    for _ in 0..64 {
        let middle = (low + high) * 0.5;
        let x = [q[0] / (values[0] + middle), q[1] / (values[1] + middle)];
        if x[0].hypot(x[1]) > kinetic_limit {
            low = middle;
        } else {
            high = middle;
        }
    }
    Ok((
        major * (q[0] / (values[0] + high)) + minor * (q[1] / (values[1] + high)),
        true,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn material_traction_matches_the_independent_virtual_work_oracle() {
        let reference = [
            Vec3::new(0.002, 0.01, 0.0),
            Vec3::new(-0.02, 0.0, -0.02),
            Vec3::new(0.02, 0.0, -0.02),
            Vec3::new(0.0, 0.0, 0.02),
        ];
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
        let frame = MaterialFrame::from_contact(contact, &reference).unwrap();
        let mut positions = reference;
        positions[0].x += 0.005;
        let stencil = Stencil::new(contact, Some(&frame), &positions).unwrap();
        let applied = stencil.forces(DVec3::new(-0.0002, 0.0, 0.0));
        // Expected values from an independent analytic + finite-difference
        // virtual-work audit; the extra normal force pair balances the offset.
        let expected = [
            DVec3::new(-0.0002, 0.0, 0.0),
            DVec3::new(0.00004, 0.00005, 0.0),
            DVec3::new(0.00006, -0.00005, 0.0),
            DVec3::new(0.0001, 0.0, 0.0),
        ];
        for (i, expected) in expected.into_iter().enumerate() {
            assert!(applied.force(i as u32).distance(expected) < 1e-10);
        }
        let force: DVec3 = (0..4).map(|i| applied.force(i)).sum();
        let moment: DVec3 = (0..4)
            .map(|i| double(positions[i]).cross(applied.force(i as u32)))
            .sum();
        assert!(force.length() < 1e-10);
        assert!(moment.length() < 1e-10);
    }

    #[test]
    fn anisotropic_tangent_disk_matches_static_solution_and_kinetic_kkt() {
        let mass = DMat3::from_cols(
            DVec3::new(4.0, 0.0, 1.0),
            DVec3::Y * 3.0,
            DVec3::new(1.0, 0.0, 2.0),
        );
        let rhs = DVec3::new(3.0, 5.0, -2.0);
        let directional = |d: DVec3| d.dot(mass * d);
        let (sticking, slide) = traction(mass, directional, rhs, DVec3::Y, 5.0, 0.1).unwrap();
        assert!(!slide);
        assert!(sticking.distance(DVec3::new(8.0 / 7.0, 0.0, -11.0 / 7.0)) < 1e-12);
        let (kinetic, slide) = traction(mass, directional, rhs, DVec3::Y, 0.2, 0.1).unwrap();
        assert!(slide);
        assert!((kinetic.length() - 0.1).abs() < 1e-12);
        assert!(kinetic.y.abs() < 1e-14);
        let mut gradient = mass * kinetic - rhs;
        gradient.y = 0.0;
        assert!(gradient.cross(kinetic).length() < 1e-12);
        assert!(gradient.dot(kinetic) < 0.0);
        let objective = |x: DVec3| 0.5 * x.dot(mass * x) - rhs.dot(x);
        for i in 0..360 {
            let (sin, cos) = (i as f64 * std::f64::consts::TAU / 360.0).sin_cos();
            assert!(objective(kinetic) <= objective(DVec3::new(cos, 0.0, sin) * 0.1) + 1e-12);
        }
    }

    #[test]
    fn weak_material_direction_survives_gram_cancellation() {
        let mut stencil = Stencil {
            ids: [0; CAPACITY],
            jacobians: [DMat3::ZERO; CAPACITY],
            count: 0,
        };
        stencil
            .add(
                0,
                DMat3::from_cols(DVec3::new(1e10, 0.0, 1e10), DVec3::Y, DVec3::Z),
            )
            .unwrap();
        let (mass, _) = stencil.metric(&[1.0], &Applied::default());
        let weak = DVec3::new(-1.0, 0.0, 1.0).normalize();
        // The unit contribution cannot be represented next to 1e20 in Gram.
        assert_eq!(weak.dot(mass * weak), 0.0);
        assert!((stencil.directional_mass(&[1.0], weak) - 0.5).abs() < 1e-14);
        let (lambda, sliding) = traction(
            mass,
            |d| stencil.directional_mass(&[1.0], d),
            weak * 0.1,
            DVec3::Y,
            0.3,
            0.1,
        )
        .unwrap();
        assert!(!sliding, "a representable weak constraint incorrectly slid");
        assert!(lambda.distance(weak * 0.2) < 1e-12);
    }

    #[test]
    fn singular_tangent_direction_uses_the_bounded_kinetic_disk() {
        let mass = DMat3::from_diagonal(DVec3::new(2.0, 0.0, 0.0));
        let (lambda, sliding) =
            traction(mass, |d| d.dot(mass * d), DVec3::Z, DVec3::Y, 0.5, 0.2).unwrap();
        assert!(sliding);
        assert!(lambda.distance(DVec3::Z * 0.2) < 1e-12);
        let (lambda, sliding) = traction(
            mass,
            |d| d.dot(mass * d),
            DVec3::X * 0.2,
            DVec3::Y,
            0.5,
            0.2,
        )
        .unwrap();
        assert!(!sliding);
        assert!(lambda.distance(DVec3::X * 0.1) < 1e-12);
    }

    #[test]
    fn invalid_force_replacement_preserves_every_particle_and_the_ledger() {
        let mut applied = Applied::default();
        applied.insert(0, DVec3::X * 0.1).unwrap();
        let mut next = Applied::default();
        next.insert(0, DVec3::X * 0.2).unwrap();
        next.insert(1, DVec3::splat(f64::INFINITY)).unwrap();
        let original = [Vec3::X, Vec3::Y];
        let mut positions = original;
        assert!(matches!(
            applied.replace(next, &mut positions, &[1.0, 0.5]),
            Err(ClothError::NonFiniteState)
        ));
        assert_eq!(positions, original);
        assert_eq!(applied.force(0), DVec3::X * 0.1);
        assert_eq!(applied.force(1), DVec3::ZERO);
    }
}
