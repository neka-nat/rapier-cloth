use super::{SurfacePoint, SurfaceView};
use crate::{ClothError, Real, Vec3, collision::geometry::closest_triangle};
use glam::DVec3;

#[derive(Clone, Copy, Debug)]
pub struct SurfaceQueryLimits {
    /// Total triangle tests in one query, including all cloths at world scope.
    pub triangles: usize,
    pub patch_vertices: usize,
}
impl Default for SurfaceQueryLimits {
    fn default() -> Self {
        Self {
            triangles: 65_536,
            patch_vertices: 256,
        }
    }
}
impl SurfaceQueryLimits {
    pub(crate) fn check(self, triangles: usize) -> Result<(), ClothError> {
        if self.triangles == 0 {
            return Err(ClothError::InvalidParameter("surface query triangle limit"));
        }
        if triangles > self.triangles {
            return Err(ClothError::SurfaceQueryBudgetExceeded {
                limit: self.triangles,
            });
        }
        if triangles > u32::MAX as usize {
            return Err(ClothError::InvalidParameter("surface triangle count"));
        }
        Ok(())
    }
}

/// A finite point approach in the current, static cloth pose. Direction is
/// normalized by the query; max_distance is in world length units.
#[derive(Clone, Copy, Debug)]
pub struct SurfaceRay {
    pub origin: Vec3,
    pub direction: Vec3,
    pub max_distance: Real,
}
impl SurfaceRay {
    pub fn validate(self) -> Result<(), ClothError> {
        self.prepare().map(|_| ())
    }
    fn prepare(self) -> Result<(DVec3, DVec3, f64), ClothError> {
        let origin = double(self.origin);
        let direction = double(self.direction);
        let length = direction.length();
        if !origin.is_finite()
            || !direction.is_finite()
            || !length.is_finite()
            || length <= 0.0
            || !self.max_distance.is_finite()
            || self.max_distance < 0.0
        {
            return Err(ClothError::InvalidParameter("surface ray"));
        }
        Ok((origin, direction / length, wide(self.max_distance)))
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SurfaceHit {
    pub point: SurfacePoint,
    pub position: Vec3,
    /// Oriented triangle normal; ray queries are double-sided.
    pub normal: Vec3,
    /// Ray/sweep travel or closest-point Euclidean distance, in world units.
    pub distance: Real,
    distance_wide: f64,
}
impl SurfaceHit {
    /// Keep nearby layers ordered even when long travel distances round to the
    /// same f32 value. The public distance field remains in selected precision.
    pub fn distance_f64(self) -> f64 {
        self.distance_wide
    }
}
#[cfg(not(feature = "f64"))]
fn wide(value: Real) -> f64 {
    f64::from(value)
}
#[cfg(feature = "f64")]
fn wide(value: Real) -> f64 {
    value
}
fn double(v: Vec3) -> DVec3 {
    DVec3::new(wide(v.x), wide(v.y), wide(v.z))
}
fn real(v: DVec3) -> Vec3 {
    Vec3::new(v.x as Real, v.y as Real, v.z as Real)
}

fn normal(p: [Vec3; 3]) -> Result<Vec3, ClothError> {
    let p = p.map(double);
    let n = (p[1] - p[0]).cross(p[2] - p[0]);
    let length = n.length();
    if !length.is_finite() || length <= 0.0 {
        return Err(ClothError::DegenerateConstraint);
    }
    Ok(real(n / length))
}

impl SurfaceView<'_> {
    /// First contact of a sphere moving along a finite approach against the
    /// current triangle surfaces. Tests triangle faces, edge cylinders and
    /// vertex spheres analytically; this does not predict future cloth motion.
    /// Initial overlap returns distance zero. Radius zero uses raycast semantics.
    pub fn sweep_sphere(
        &self,
        ray: SurfaceRay,
        radius: Real,
        limits: SurfaceQueryLimits,
    ) -> Result<Option<SurfaceHit>, ClothError> {
        limits.check(self.triangles.len())?;
        let (origin, direction, maximum) = ray.prepare()?;
        if !radius.is_finite() || radius < 0.0 {
            return Err(ClothError::InvalidParameter("surface sweep radius"));
        }
        if radius == 0.0 {
            return self.raycast(ray, limits);
        }
        let radius = wide(radius);
        let mut best = None;
        let mut best_distance = maximum;
        for face in 0..self.triangles.len() {
            let triangle = self.triangle(face as u32)?;
            let n = normal(triangle)?;
            let p = triangle.map(double);
            let initial =
                closest_triangle(ray.origin, triangle).ok_or(ClothError::DegenerateConstraint)?;
            let mut accept = |distance: f64, bary: [f64; 3]| -> Result<(), ClothError> {
                if !distance.is_finite() || bary.iter().any(|b| !b.is_finite()) {
                    return Err(ClothError::NonFiniteState);
                }
                if distance < 0.0
                    || distance > best_distance
                    || (distance == best_distance && best.is_some())
                {
                    return Ok(());
                }
                let point = SurfacePoint::new(face as u32, bary.map(|b| b as Real))?;
                best = Some(SurfaceHit {
                    point,
                    position: self.point_position(point)?,
                    normal: n,
                    distance: distance as Real,
                    distance_wide: distance,
                });
                best_distance = distance;
                Ok(())
            };
            if origin.distance(double(initial.point)) <= radius {
                accept(0.0, initial.barycentric.map(wide))?;
                continue;
            }
            let normal = (p[1] - p[0]).cross(p[2] - p[0]).normalize();
            let height = (origin - p[0]).dot(normal);
            let rate = direction.dot(normal);
            if rate != 0.0 {
                for side in [-1.0, 1.0] {
                    let distance = (side * radius - height) / rate;
                    if distance < 0.0 || distance > maximum {
                        continue;
                    }
                    let q = origin + direction * distance - normal * (side * radius);
                    let areas = [
                        (p[1] - q).cross(p[2] - q).dot(normal),
                        (p[2] - q).cross(p[0] - q).dot(normal),
                        (p[0] - q).cross(p[1] - q).dot(normal),
                    ];
                    if areas.iter().any(|a| !a.is_finite()) {
                        return Err(ClothError::NonFiniteState);
                    }
                    if areas.iter().all(|a| *a >= 0.0) {
                        let sum: f64 = areas.iter().sum();
                        accept(distance, areas.map(|a| a / sum))?;
                    }
                }
            }
            for i in 0..3 {
                let j = (i + 1) % 3;
                let offset = origin - p[i];
                for t in roots(
                    1.0,
                    offset.dot(direction),
                    offset.length_squared() - radius * radius,
                )?
                .into_iter()
                .flatten()
                {
                    let mut bary = [0.0; 3];
                    bary[i] = 1.0;
                    accept(t, bary)?;
                }
                let edge = p[j] - p[i];
                let length = edge.length();
                let axis = edge / length;
                let along = offset.dot(axis);
                let speed = direction.dot(axis);
                let radial = offset - axis * along;
                let radial_speed = direction - axis * speed;
                for t in roots(
                    radial_speed.length_squared(),
                    radial.dot(radial_speed),
                    radial.length_squared() - radius * radius,
                )?
                .into_iter()
                .flatten()
                {
                    if t < 0.0 || t > maximum {
                        continue;
                    }
                    let u = (along + t * speed) / length;
                    if (0.0..=1.0).contains(&u) {
                        let mut bary = [0.0; 3];
                        bary[i] = 1.0 - u;
                        bary[j] = u;
                        accept(t, bary)?;
                    }
                }
            }
        }
        Ok(best)
    }
    /// First triangle hit, including edges/vertices and either winding. Coplanar
    /// rays have no unique intersection and return no hit. Degenerate or invalid
    /// geometry fails the query instead of exposing an occluded lower layer.
    pub fn raycast(
        &self,
        ray: SurfaceRay,
        limits: SurfaceQueryLimits,
    ) -> Result<Option<SurfaceHit>, ClothError> {
        limits.check(self.triangles.len())?;
        let (origin, direction, maximum) = ray.prepare()?;
        self.raycast_prepared(origin, direction, maximum)
    }

    /// Whether another surface intersects a segment before its known endpoint.
    /// The endpoint is excluded by a floating-point margin of 256*f64::EPSILON
    /// of segment length. Start/end conversion precedes subtraction, preserving
    /// short end geometry for long approaches in f32.
    pub fn segment_occluded(
        &self,
        start: Vec3,
        end: Vec3,
        limits: SurfaceQueryLimits,
    ) -> Result<bool, ClothError> {
        limits.check(self.triangles.len())?;
        let origin = double(start);
        let delta = double(end) - origin;
        let length = delta.length();
        if !origin.is_finite() || !delta.is_finite() || !length.is_finite() || length <= 0.0 {
            return Err(ClothError::InvalidParameter("surface visibility segment"));
        }
        Ok(self
            .raycast_prepared(
                origin,
                delta / length,
                length * (1.0 - 256.0 * f64::EPSILON),
            )?
            .is_some())
    }

    fn raycast_prepared(
        &self,
        origin: DVec3,
        direction: DVec3,
        maximum: f64,
    ) -> Result<Option<SurfaceHit>, ClothError> {
        let kz = (0..3)
            .max_by(|&a, &b| direction[a].abs().total_cmp(&direction[b].abs()))
            .unwrap();
        let kx = (kz + 1) % 3;
        let ky = (kx + 1) % 3;
        let sx = direction[kx] / direction[kz];
        let sy = direction[ky] / direction[kz];
        let mut best: Option<SurfaceHit> = None;
        let mut best_distance = maximum;
        for face in 0..self.triangles.len() {
            let triangle = self.triangle(face as u32)?;
            let n = normal(triangle)?;
            // Shared-edge functions use the same sheared endpoint arithmetic
            // with opposite signs on adjacent triangles. All intermediates use
            // f64, also in the f32 build; no barycentric acceptance padding.
            let p = triangle.map(|p| {
                let v = double(p) - origin;
                DVec3::new(
                    v[kx] - sx * v[kz],
                    v[ky] - sy * v[kz],
                    v[kz] / direction[kz],
                )
            });
            if p.iter().any(|p| !p.is_finite()) {
                return Err(ClothError::NonFiniteState);
            }
            let cross = |a: DVec3, b: DVec3| a.x * b.y - a.y * b.x;
            let e = [cross(p[1], p[2]), cross(p[2], p[0]), cross(p[0], p[1])];
            if e.iter().any(|e| !e.is_finite()) {
                return Err(ClothError::NonFiniteState);
            }
            if e.iter().any(|e| *e < 0.0) && e.iter().any(|e| *e > 0.0) {
                continue;
            }
            let det: f64 = e.iter().sum();
            if !det.is_finite() {
                return Err(ClothError::NonFiniteState);
            }
            if det == 0.0 {
                continue;
            }
            let bary = e.map(|e| e / det);
            let distance = bary[0] * p[0].z + bary[1] * p[1].z + bary[2] * p[2].z;
            if !distance.is_finite() {
                return Err(ClothError::NonFiniteState);
            }
            if distance < 0.0
                || distance > best_distance
                || (distance == best_distance && best.is_some())
            {
                continue;
            }
            let point = SurfacePoint::new(face as u32, bary.map(|b| b as Real))?;
            best = Some(SurfaceHit {
                point,
                position: self.point_position(point)?,
                normal: n,
                distance: distance as Real,
                distance_wide: distance,
            });
            best_distance = distance;
        }
        Ok(best)
    }

    /// Nearest point regardless of visibility. Use raycast for selecting the
    /// first visible cloth layer along an approach direction.
    pub fn closest_point(
        &self,
        position: Vec3,
        max_distance: Real,
        limits: SurfaceQueryLimits,
    ) -> Result<Option<SurfaceHit>, ClothError> {
        limits.check(self.triangles.len())?;
        if !position.is_finite() || !max_distance.is_finite() || max_distance < 0.0 {
            return Err(ClothError::InvalidParameter("closest surface query"));
        }
        let mut best = None;
        let mut best_distance = wide(max_distance);
        for face in 0..self.triangles.len() {
            let triangle = self.triangle(face as u32)?;
            let n = normal(triangle)?;
            let witness =
                closest_triangle(position, triangle).ok_or(ClothError::DegenerateConstraint)?;
            let distance = double(position).distance(double(witness.point));
            if !distance.is_finite() {
                return Err(ClothError::NonFiniteState);
            }
            if distance > best_distance || (distance == best_distance && best.is_some()) {
                continue;
            }
            let point = SurfacePoint::new(face as u32, witness.barycentric)?;
            best = Some(SurfaceHit {
                point,
                position: self.point_position(point)?,
                normal: n,
                distance: distance as Real,
                distance_wide: distance,
            });
            best_distance = distance;
        }
        Ok(best)
    }
}

/// Roots of a*t*t + 2*b*t + c. The stable product form avoids cancellation
/// for the root nearest the origin. Zero direction components are supported.
fn roots(a: f64, b: f64, c: f64) -> Result<[Option<f64>; 2], ClothError> {
    if !a.is_finite() || !b.is_finite() || !c.is_finite() {
        return Err(ClothError::NonFiniteState);
    }
    if a == 0.0 {
        return Ok([None; 2]);
    }
    let discriminant = b.mul_add(b, -a * c);
    if !discriminant.is_finite() {
        return Err(ClothError::NonFiniteState);
    }
    if discriminant < 0.0 {
        return Ok([None; 2]);
    }
    let root = discriminant.sqrt();
    if root == 0.0 {
        return Ok([Some(-b / a), None]);
    }
    let q = -b - root.copysign(b);
    let mut values = [q / a, c / q];
    values.sort_by(f64::total_cmp);
    Ok(values.map(Some))
}
