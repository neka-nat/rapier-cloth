use crate::Real;

#[derive(Debug, Clone, Default)]
pub struct StepReport {
    /// Accepted implicit-step diagnostics; `None` for XPBD or before any step.
    #[cfg(feature = "implicit")]
    pub implicit: Option<crate::implicit::ImplicitOutcome>,
    pub max_stretch: Real,
    pub p95_stretch: Real,
    pub max_bend_error: Real,
    pub max_target_error: Real,
    pub max_penetration: Real,
    pub contacts: usize,
    pub surface_collision: crate::collision::CollisionWork,
    pub stabilized_contacts: usize,
    pub degenerate_faces: usize,
    pub iterations: usize,
    /// Retained array capacities, including contact states; excludes bridge staging.
    pub scratch_bytes: usize,
}
