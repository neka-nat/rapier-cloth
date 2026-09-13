use super::friction_frame::{MaterialFrame, double, wide};
use super::friction_stencil::Stencil;
use crate::{Real, SurfaceContact, SurfaceContactKey, SurfaceFeature, Vec3};
use glam::{DMat3, DQuat, DVec3};

// Independent Gram-Schmidt coordinates, evaluated entirely in f64. In
// particular, the finite differences do not call the analytic frame Jacobian
// or quantize the perturbed positions to the selected simulation precision.
fn basis(p: &[DVec3; 4]) -> DMat3 {
    let t = (p[2] - p[1]).normalize();
    let e = p[3] - p[1];
    let v = (e - t * t.dot(e)).normalize();
    DMat3::from_cols(t, v, t.cross(v))
}

fn relative(c: SurfaceContact, p: &[DVec3; 4]) -> DVec3 {
    (0..4)
        .map(|i| p[c.particles[i] as usize] * wide(c.weights[i]))
        .sum()
}

fn fixture(sample: usize) -> (SurfaceContact, [Vec3; 4], [Vec3; 4]) {
    let reference = [
        DVec3::new(0.003, 0.01, -0.002),
        DVec3::new(-0.02, 0.0, -0.02),
        DVec3::new(0.02, 0.0, -0.02),
        DVec3::new(0.0, 0.0, 0.02),
    ];
    let a = sample as f64 * 0.17;
    let r = DQuat::from_axis_angle(DVec3::new(1.0, 2.0, -3.0).normalize(), a);
    let shear = DMat3::from_cols(
        DVec3::new(0.7 + 0.02 * sample as f64, 0.13, 0.1),
        DVec3::new(0.09, 1.1, 0.11),
        DVec3::new(0.2, -0.15, 1.2),
    );
    let mut current = reference.map(|p| r * (shear * p) + DVec3::new(0.12, -0.08, 0.07));
    current[0] += r * DVec3::new(0.006, 0.003, -0.004);
    let normal = basis(&current).z_axis;
    let narrow = |p: DVec3| Vec3::from_array(p.to_array().map(|x| x as Real));
    let c = SurfaceContact {
        key: SurfaceContactKey {
            other_cloth: None,
            features: [SurfaceFeature::Vertex(0), SurfaceFeature::Face(0)],
        },
        particles: [0, 1, 2, 3],
        // Exactly representable weights preserve translation invariance in
        // both precisions, independently of contact-weight normalization.
        weights: [1.0, -0.25, -0.25, -0.5],
        normal: narrow(normal),
        offset: Vec3::ZERO,
        surface_velocity: Vec3::ZERO,
        separation: 0.01,
        static_friction: 0.5,
        kinetic_friction: 0.2,
    };
    (c, reference.map(narrow), current.map(narrow))
}

#[test]
fn advected_anchor_gradient_matches_deformed_support_finite_differences() {
    for sample in 0..24 {
        let (c, reference, positions) = fixture(sample);
        let frame = MaterialFrame::from_contact(c, &reference).unwrap();
        let anchor = c.relative(&reference);
        let local = basis(&reference.map(double)).transpose() * double(anchor);
        let p = positions.map(double);
        let gradient = frame.anchor_gradients(anchor, &positions).unwrap();
        let h = 1e-7;
        for (particle, matrix) in gradient {
            for axis in 0..3 {
                let mut plus = p;
                let mut minus = p;
                plus[particle as usize][axis] += h;
                minus[particle as usize][axis] -= h;
                let fd = (basis(&plus) * local - basis(&minus) * local) / (2.0 * h);
                let error = matrix.col(axis).distance(fd);
                assert!(
                    error < 2e-8,
                    "sample {sample}, particle {particle}, axis {axis}: {error}"
                );
            }
        }
    }
}

#[test]
fn material_coordinate_force_matches_virtual_work_and_rigid_motion_nullspace() {
    for sample in 0..24 {
        let (c, reference, positions) = fixture(sample);
        let mut frame = MaterialFrame::from_contact(c, &reference).unwrap();
        frame.transport(c.relative(&reference), &positions).unwrap();
        let stencil = Stencil::new(c, Some(&frame), &positions).unwrap();
        let p = positions.map(double);
        let r = basis(&p);
        let local_traction = DVec3::new(-0.00017, 0.00009, 0.0);
        let applied = stencil.forces(r * local_traction);
        // The material coordinates are R(x)^T r(x). The fixed reference
        // anchor has zero derivative in these coordinates.
        let work = |x: &[DVec3; 4]| local_traction.dot(basis(x).transpose() * relative(c, x));
        let h = 1e-7;
        let mut maximum_error: f64 = 0.0;
        for particle in 0..4 {
            for axis in 0..3 {
                let mut plus = p;
                let mut minus = p;
                plus[particle][axis] += h;
                minus[particle][axis] -= h;
                let fd = (work(&plus) - work(&minus)) / (2.0 * h);
                maximum_error =
                    maximum_error.max((applied.force(particle as u32)[axis] - fd).abs());
            }
        }
        let force: DVec3 = (0..4).map(|i| applied.force(i)).sum();
        let torque: DVec3 = (0..4).map(|i| p[i].cross(applied.force(i as u32))).sum();
        assert!(
            maximum_error < 2e-10,
            "sample {sample}: virtual-work error {maximum_error}, torque {torque:?}"
        );
        assert!(force.length() < 2e-10);
        assert!(torque.length() < 2e-10, "sample {sample}: {torque:?}");
    }
}
