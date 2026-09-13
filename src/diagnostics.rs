use crate::{ClothError, ClothHandle, Real, StepReport, rapier::prelude::ColliderHandle};
use std::fmt;

#[derive(Debug, Clone, PartialEq)]
pub enum IntegrationError {
    Core(ClothError),
    InvalidScene(&'static str),
    InvalidAttachment(&'static str),
    UnsupportedCollision {
        collider: ColliderHandle,
        reason: &'static str,
    },
    MissingPreviousPose(ColliderHandle),
    MotionBudget {
        collider: ColliderHandle,
        movement: Real,
        allowed: Real,
        required_substeps: usize,
    },
}
impl fmt::Display for IntegrationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidAttachment(s) => write!(f, "invalid attachment: {s}"),
            Self::Core(e) => write!(f, "{e}"),
            Self::InvalidScene(s) => write!(f, "invalid Rapier scene: {s}"),
            Self::UnsupportedCollision { collider, reason } => {
                write!(f, "unsupported collision {collider:?}: {reason}")
            }
            Self::MissingPreviousPose(c) => {
                write!(f, "missing previous kinematic collider pose: {c:?}")
            }
            Self::MotionBudget {
                collider,
                movement,
                allowed,
                required_substeps,
            } => write!(
                f,
                "kinematic motion budget exceeded for {collider:?}: {movement} > {allowed}; at least {required_substeps} subdivisions required"
            ),
        }
    }
}
impl std::error::Error for IntegrationError {}
impl From<ClothError> for IntegrationError {
    fn from(e: ClothError) -> Self {
        Self::Core(e)
    }
}

#[derive(Debug, Clone, Default)]
pub struct WorldStepReport {
    pub step: u64,
    pub cloths: Vec<(ClothHandle, StepReport)>,
    /// Candidates explicitly skipped by adapter settings or sensor/disabled status.
    /// QueryFilter exclusions happen before candidate enumeration and are not counted.
    pub ignored_colliders: Vec<ColliderHandle>,
    pub candidate_queries: usize,
    pub pair_queries: usize,
    /// Time in the core solver, excluding callbacks into the Rapier adapter.
    pub core_time_seconds: f64,
    pub query_time_seconds: f64,
    pub total_time_seconds: f64,
}
