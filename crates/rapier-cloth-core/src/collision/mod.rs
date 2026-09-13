//! Surface contact geometry, independent of any rigid-body engine.
mod broad_phase;
pub mod ccd;
pub(crate) mod friction;
pub mod geometry;
pub(crate) mod normal_block;
pub(crate) mod self_collision;
mod settings;
pub use settings::{ClothContactSettings, CollisionBudgetKind, CollisionLimits, CollisionWork};
