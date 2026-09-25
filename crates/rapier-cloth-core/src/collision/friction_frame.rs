use crate::{Real, SurfaceContact, SurfaceFeature, Vec3};
use glam::DVec3;

#[cfg(not(feature = "f64"))]
pub(super) fn wide(x: Real) -> f64 {
    f64::from(x)
}
#[cfg(feature = "f64")]
pub(super) fn wide(x: Real) -> f64 {
    x
}
pub(super) fn double(v: Vec3) -> DVec3 {
    DVec3::from_array(v.to_array().map(wide))
}

pub(super) fn side_vertices(contact: SurfaceContact) -> ([u32; 3], usize) {
    // The negative side supplies the support frame. Generalized one-sided
    // deforming constraints can use their positive side instead.
    for negative in [true, false] {
        let mut vertices = [0; 3];
        let mut count = 0;
        for i in 0..4 {
            let weight = contact.weights[i];
            if weight != 0.0 && (weight < 0.0) == negative && count < 3 {
                vertices[count] = contact.particles[i];
                count += 1;
            }
        }
        if count > 0 {
            return (vertices, count);
        }
    }
    ([0; 3], 0)
}

#[derive(Debug, Clone, Copy)]
pub(super) struct MaterialFrame {
    vertices: [u32; 3],
    basis: [DVec3; 3],
}
impl MaterialFrame {
    /// Rotation derivative applied to the current material-witness vector.
    /// For local coordinates c = R^T r - a, R dc = dr - dR R^T r.
    /// Use r here, not the rest anchor: the slipping residual rotates too.
    pub fn coordinate_gradients(
        &self,
        relative: DVec3,
        positions: &[Vec3],
    ) -> Option<[(u32, glam::DMat3); 3]> {
        let current = Self::basis(self.vertices, positions)?;
        self.gradients(current.map(|axis| axis.dot(relative)), positions)
    }

    #[cfg(test)]
    pub fn anchor_gradients(
        &self,
        anchor: Vec3,
        positions: &[Vec3],
    ) -> Option<[(u32, glam::DMat3); 3]> {
        let anchor = double(anchor);
        self.gradients(self.basis.map(|axis| axis.dot(anchor)), positions)
    }

    fn gradients(&self, local: [f64; 3], positions: &[Vec3]) -> Option<[(u32, glam::DMat3); 3]> {
        let p = self
            .vertices
            .map(|i| positions.get(i as usize).copied().map(double));
        let [a, b, c] = [p[0]?, p[1]?, p[2]?];
        let e = b - a;
        let f = c - a;
        let edge_length = e.length();
        let area = e.cross(f).length();
        let [t, v, n] = Self::basis(self.vertices, positions)?;
        let bitangent_length = n.cross(t).length();
        let mut result = [(0, glam::DMat3::ZERO); 3];
        for i in 0..3 {
            let columns: [DVec3; 3] = std::array::from_fn(|axis| {
                let direction = [DVec3::X, DVec3::Y, DVec3::Z][axis];
                let de = direction * [-1.0, 1.0, 0.0][i];
                let df = direction * [-1.0, 0.0, 1.0][i];
                let dt = (de - t * t.dot(de)) / edge_length;
                let dc = de.cross(f) + e.cross(df);
                let dn = (dc - n * n.dot(dc)) / area;
                let dv = dn.cross(t) + n.cross(dt);
                let dv = (dv - v * v.dot(dv)) / bitangent_length;
                dt * local[0] + dv * local[1] + dn * local[2]
            });
            let matrix = glam::DMat3::from_cols(columns[0], columns[1], columns[2]);
            if !matrix.is_finite() {
                return None;
            }
            result[i] = (self.vertices[i], matrix);
        }
        Some(result)
    }
    fn basis(vertices: [u32; 3], positions: &[Vec3]) -> Option<[DVec3; 3]> {
        let a = double(*positions.get(vertices[0] as usize)?);
        let b = double(*positions.get(vertices[1] as usize)?);
        let c = double(*positions.get(vertices[2] as usize)?);
        let tangent = (b - a).try_normalize()?;
        let normal = (b - a).cross(c - a).try_normalize()?;
        let bitangent = normal.cross(tangent).try_normalize()?;
        Some([tangent, bitangent, normal])
    }
    pub fn new(vertices: [u32; 3], positions: &[Vec3]) -> Option<Self> {
        Some(Self {
            vertices,
            basis: Self::basis(vertices, positions)?,
        })
    }
    pub fn from_contact(contact: SurfaceContact, positions: &[Vec3]) -> Option<Self> {
        if contact.static_friction == 0.0
            || contact
                .key
                .features
                .iter()
                .any(|f| matches!(f, SurfaceFeature::External { .. }))
        {
            return None;
        }
        let (vertices, count) = side_vertices(contact);
        (count == 3)
            .then(|| Self::new(vertices, positions))
            .flatten()
    }
    pub fn transport(&mut self, anchor: Vec3, positions: &[Vec3]) -> Option<Vec3> {
        let next = Self::basis(self.vertices, positions)?;
        if next == self.basis {
            return Some(anchor);
        }
        let v = double(anchor);
        let transformed: DVec3 = next
            .iter()
            .zip(self.basis)
            .map(|(axis, old)| *axis * old.dot(v))
            .sum();
        self.basis = next;
        Some(Vec3::from_array(transformed.to_array().map(|x| x as Real)))
    }
    pub fn reset_basis(&mut self, positions: &[Vec3]) -> Option<()> {
        self.basis = Self::basis(self.vertices, positions)?;
        Some(())
    }
}
