use crate::{ClothError, ClothMesh, Real, SurfacePoint, Vec3};

/// Constrain a triangle's material point without pinning its support vertices.
#[derive(Clone, Copy, Debug)]
pub struct SurfaceTarget {
    pub point: SurfacePoint,
    pub position: Vec3,
    pub compliance: Real,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct ResolvedSurfaceTarget {
    pub target: SurfaceTarget,
    pub vertices: [u32; 3],
}
impl ResolvedSurfaceTarget {
    pub fn new(target: SurfaceTarget, mesh: &ClothMesh) -> Result<Self, ClothError> {
        let vertices = *mesh
            .triangles()
            .get(target.point.triangle() as usize)
            .ok_or(ClothError::InvalidParameter("surface target triangle"))?;
        if !target.position.is_finite() || !target.compliance.is_finite() || target.compliance < 0.0
        {
            return Err(ClothError::InvalidParameter("surface target"));
        }
        Ok(Self { target, vertices })
    }
    pub fn position(&self, positions: &[Vec3]) -> Vec3 {
        let b = self.target.point.barycentric();
        positions[self.vertices[0] as usize] * b[0]
            + positions[self.vertices[1] as usize] * b[1]
            + positions[self.vertices[2] as usize] * b[2]
    }
    pub fn project(
        &self,
        positions: &mut [Vec3],
        masses: &[Real],
        h: Real,
        lambda: &mut Vec3,
    ) -> Result<(), ClothError> {
        let bary = self.target.point.barycentric();
        let alpha = self.target.compliance / (h * h);
        let inverse_mass: Real = self
            .vertices
            .iter()
            .zip(bary)
            .map(|(&i, b)| masses[i as usize] * b * b)
            .sum();
        let error = self.position(positions) - self.target.position;
        let denominator = inverse_mass + alpha;
        if !denominator.is_finite() || !error.is_finite() {
            return Err(ClothError::NonFiniteState);
        }
        if denominator <= 0.0 {
            return Err(ClothError::ConflictingSurfaceTarget {
                triangle: self.target.point.triangle(),
            });
        }
        let delta = (-error - *lambda * alpha) / denominator;
        if !delta.is_finite() {
            return Err(ClothError::NonFiniteState);
        }
        for (&i, b) in self.vertices.iter().zip(bary) {
            positions[i as usize] += delta * (masses[i as usize] * b);
        }
        *lambda += delta;
        Ok(())
    }
}
