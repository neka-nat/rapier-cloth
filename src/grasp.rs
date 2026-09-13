//! Surface selection and idealized commanded grasps. Robot kinematics and
//! fingertip force control remain responsibilities of the host simulator.
use crate::{
    AttachmentDesc, AttachmentHandle, AttachmentPoint, ClothError, ClothHandle, IntegrationError,
    RapierClothWorld, Real, SurfaceAttachmentDesc, SurfaceAttachmentPoint, SurfaceHit,
    SurfacePoint, SurfaceQueryLimits, SurfaceRay, Vec3, attachment::Attachment, rapier::prelude::*,
};
use std::collections::BTreeSet;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClothSurfacePoint {
    pub cloth: ClothHandle,
    pub point: SurfacePoint,
}

#[derive(Debug, Clone, Copy)]
pub struct ClothSurfaceHit {
    pub point: ClothSurfacePoint,
    pub position: Vec3,
    pub normal: Vec3,
    pub distance: Real,
}
impl ClothSurfaceHit {
    fn new(cloth: ClothHandle, hit: SurfaceHit) -> Self {
        Self {
            point: ClothSurfacePoint {
                cloth,
                point: hit.point,
            },
            position: hit.position,
            normal: hit.normal,
            distance: hit.distance,
        }
    }
}

/// A bounded material selection, independent of the later world pose. Re-query
/// visibility if the cloth moves between selection and grasp creation.
#[derive(Debug, Clone)]
pub struct GraspPatch {
    point: ClothSurfacePoint,
    vertices: Vec<u32>,
}
impl GraspPatch {
    pub fn point(&self) -> ClothSurfacePoint {
        self.point
    }
    pub fn vertices(&self) -> &[u32] {
        &self.vertices
    }
}

#[derive(Debug, Clone)]
pub struct GraspOptions {
    pub body: RigidBodyHandle,
    pub compliance: Real,
    pub excluded_colliders: Vec<ColliderHandle>,
}
impl GraspOptions {
    pub fn new(body: RigidBodyHandle) -> Self {
        Self {
            body,
            compliance: 0.0,
            excluded_colliders: vec![],
        }
    }
}

impl RapierClothWorld {
    /// First cloth surface along the approach, across every cloth. Ineligible
    /// pinned/already grasped surfaces still occlude deeper layers; attempting
    /// to grasp them returns a conflict instead of searching underneath.
    /// Rigid-object occlusion is a separate host/Rapier query.
    pub fn raycast_cloth(
        &self,
        ray: SurfaceRay,
        limits: SurfaceQueryLimits,
    ) -> Result<Option<ClothSurfaceHit>, ClothError> {
        ray.validate()?;
        self.query_cloths(limits, |cloth| cloth.surface().raycast(ray, limits))
    }

    /// Closest surface irrespective of visibility. Prefer raycast_cloth when
    /// selecting the exposed layer for an approaching gripper.
    pub fn closest_cloth_point(
        &self,
        position: Vec3,
        max_distance: Real,
        limits: SurfaceQueryLimits,
    ) -> Result<Option<ClothSurfaceHit>, ClothError> {
        if !position.is_finite() || !max_distance.is_finite() || max_distance < 0.0 {
            return Err(ClothError::InvalidParameter("closest surface query"));
        }
        self.query_cloths(limits, |cloth| {
            cloth
                .surface()
                .closest_point(position, max_distance, limits)
        })
    }

    pub fn sweep_cloth_sphere(
        &self,
        ray: SurfaceRay,
        radius: Real,
        limits: SurfaceQueryLimits,
    ) -> Result<Option<ClothSurfaceHit>, ClothError> {
        ray.validate()?;
        if !radius.is_finite() || radius < 0.0 {
            return Err(ClothError::InvalidParameter("surface sweep radius"));
        }
        self.query_cloths(limits, |cloth| {
            cloth.surface().sweep_sphere(ray, radius, limits)
        })
    }

    fn query_cloths(
        &self,
        limits: SurfaceQueryLimits,
        mut query: impl FnMut(&crate::Cloth) -> Result<Option<SurfaceHit>, ClothError>,
    ) -> Result<Option<ClothSurfaceHit>, ClothError> {
        if limits.triangles == 0 {
            return Err(ClothError::InvalidParameter("surface query triangle limit"));
        }
        let mut count: usize = 0;
        for (_, cloth) in self.cloths.iter() {
            count = count.checked_add(cloth.mesh().triangles().len()).ok_or(
                ClothError::SurfaceQueryBudgetExceeded {
                    limit: limits.triangles,
                },
            )?;
            if count > limits.triangles {
                return Err(ClothError::SurfaceQueryBudgetExceeded {
                    limit: limits.triangles,
                });
            }
        }
        let mut best: Option<ClothSurfaceHit> = None;
        let mut best_distance = f64::INFINITY;
        for (handle, cloth) in self.cloths.iter() {
            if let Some(hit) = query(cloth)?
                && hit.distance_f64() < best_distance
            {
                best_distance = hit.distance_f64();
                best = Some(ClothSurfaceHit::new(handle, hit));
            }
        }
        Ok(best)
    }

    pub fn surface_point_position(&self, point: ClothSurfacePoint) -> Result<Vec3, ClothError> {
        self.cloths
            .get(point.cloth)?
            .surface()
            .point_position(point.point)
    }

    pub fn select_grasp_patch(
        &self,
        point: ClothSurfacePoint,
        radius: Real,
        limits: SurfaceQueryLimits,
    ) -> Result<GraspPatch, ClothError> {
        let cloth = self.cloths.get(point.cloth)?;
        let vertices = cloth.mesh().surface_patch(point.point, radius, limits)?;
        for &i in &vertices {
            if cloth.pins().contains_key(&i)
                || self
                    .attachments
                    .iter()
                    .any(|(_, a)| a.cloth() == point.cloth && a.uses_particle(i, cloth.mesh()))
            {
                return Err(ClothError::ConflictingTarget(i));
            }
        }
        Ok(GraspPatch { point, vertices })
    }

    /// Select a material patch from the first ray hit and verify every vertex
    /// is exposed from that ray's origin. An occluded member rejects the whole
    /// patch; no hidden layer is attached and no partial patch is substituted.
    /// The triangle budget is cumulative over the hit and visibility queries.
    pub fn select_visible_grasp_patch(
        &self,
        ray: SurfaceRay,
        radius: Real,
        limits: SurfaceQueryLimits,
    ) -> Result<Option<GraspPatch>, ClothError> {
        let Some(hit) = self.raycast_cloth(ray, limits)? else {
            return Ok(None);
        };
        let patch = self.select_grasp_patch(hit.point, radius, limits)?;
        let triangles: usize = self
            .cloths
            .iter()
            .map(|(_, c)| c.mesh().triangles().len())
            .sum();
        if triangles
            .checked_mul(patch.vertices.len() + 1)
            .is_none_or(|tests| tests > limits.triangles)
        {
            return Err(ClothError::SurfaceQueryBudgetExceeded {
                limit: limits.triangles,
            });
        }
        let selected = self.cloths.get(hit.point.cloth)?;
        for &i in &patch.vertices {
            let end = selected.positions()[i as usize];
            for (_, cloth) in self.cloths.iter() {
                if cloth.surface().segment_occluded(ray.origin, end, limits)? {
                    return Err(ClothError::InvalidParameter(
                        "grasp patch includes an occluded vertex",
                    ));
                }
            }
        }
        Ok(Some(patch))
    }

    /// Capture each selected vertex's current body-local offset. No vertex is
    /// collapsed onto the patch center. Release with the shared release method.
    pub fn grasp_patch(
        &mut self,
        patch: &GraspPatch,
        options: GraspOptions,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
    ) -> Result<AttachmentHandle, IntegrationError> {
        let cloth = self.cloths.get(patch.point.cloth)?;
        let body = bodies
            .get(options.body)
            .ok_or(IntegrationError::InvalidAttachment("body handle"))?;
        let points = patch
            .vertices
            .iter()
            .map(|&particle| AttachmentPoint {
                particle,
                local_anchor: body
                    .position()
                    .inverse_transform_point(cloth.positions()[particle as usize]),
            })
            .collect();
        self.attach(
            AttachmentDesc {
                cloth: patch.point.cloth,
                body: options.body,
                points,
                compliance: options.compliance,
                excluded_colliders: options.excluded_colliders,
            },
            bodies,
            colliders,
        )
    }

    /// Capture one barycentric material point. Only its weighted position is
    /// constrained; the triangle's individual vertices retain physical masses.
    pub fn grasp_surface(
        &mut self,
        point: ClothSurfacePoint,
        options: GraspOptions,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
    ) -> Result<AttachmentHandle, IntegrationError> {
        let position = self.surface_point_position(point)?;
        let body = bodies
            .get(options.body)
            .ok_or(IntegrationError::InvalidAttachment("body handle"))?;
        self.attach_surface(
            SurfaceAttachmentDesc {
                cloth: point.cloth,
                body: options.body,
                points: vec![SurfaceAttachmentPoint {
                    point: point.point,
                    local_anchor: body.position().inverse_transform_point(position),
                }],
                compliance: options.compliance,
                excluded_colliders: options.excluded_colliders,
            },
            bodies,
            colliders,
        )
    }

    pub fn attach_surface(
        &mut self,
        desc: SurfaceAttachmentDesc,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
    ) -> Result<AttachmentHandle, IntegrationError> {
        let cloth = self.cloths.get(desc.cloth)?;
        let body = bodies
            .get(desc.body)
            .ok_or(IntegrationError::InvalidAttachment("body handle"))?;
        if body.is_dynamic() || !body.is_enabled() {
            return Err(IntegrationError::InvalidAttachment(
                "body must be enabled and fixed or kinematic",
            ));
        }
        if desc.points.is_empty() || !desc.compliance.is_finite() || desc.compliance < 0.0 {
            return Err(IntegrationError::InvalidAttachment(
                "surface points or compliance",
            ));
        }
        let mut support = BTreeSet::new();
        for p in &desc.points {
            cloth.surface().point_position(p.point)?;
            if !p.local_anchor.is_finite() {
                return Err(IntegrationError::InvalidAttachment("non-finite anchor"));
            }
            let triangle = cloth.mesh().triangles()[p.point.triangle() as usize];
            for (i, b) in triangle.into_iter().zip(p.point.barycentric()) {
                if b == 0.0 {
                    continue;
                }
                if !support.insert(i)
                    || cloth.pins().contains_key(&i)
                    || self
                        .attachments
                        .iter()
                        .any(|(_, a)| a.cloth() == desc.cloth && a.uses_particle(i, cloth.mesh()))
                {
                    return Err(ClothError::ConflictingTarget(i).into());
                }
            }
        }
        for handle in &desc.excluded_colliders {
            if colliders
                .get(*handle)
                .is_none_or(|c| c.parent() != Some(desc.body))
            {
                return Err(IntegrationError::InvalidAttachment(
                    "excluded collider must belong to attached body",
                ));
            }
        }
        self.cloths.get_mut(desc.cloth)?.clear_contact_history();
        Ok(self.attachments.insert(Attachment::Surface(desc)))
    }

    pub fn surface_attachment(
        &self,
        handle: AttachmentHandle,
    ) -> Result<&SurfaceAttachmentDesc, IntegrationError> {
        match self.attachments.get(handle) {
            Some(Attachment::Surface(a)) => Ok(a),
            _ => Err(IntegrationError::InvalidAttachment(
                "stale, foreign or vertex attachment handle",
            )),
        }
    }
    pub fn surface_attachments(
        &self,
    ) -> impl Iterator<Item = (AttachmentHandle, &SurfaceAttachmentDesc)> {
        self.attachments.iter().filter_map(|(h, a)| match a {
            Attachment::Surface(a) => Some((h, a)),
            _ => None,
        })
    }
}
