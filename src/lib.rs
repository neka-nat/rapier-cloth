#![doc = include_str!("../README.md")]
#![forbid(unsafe_code)]

#[cfg(any(
    all(feature = "f32", feature = "f64"),
    not(any(feature = "f32", feature = "f64"))
))]
compile_error!("rapier-cloth: select exactly one precision feature: f32 or f64");

pub use rapier_cloth_core as core;
pub use rapier_cloth_core::*;

#[cfg(any(feature = "f32", feature = "f64"))]
pub mod attachment;
#[cfg(any(feature = "f32", feature = "f64"))]
mod collision;
#[cfg(any(feature = "f32", feature = "f64"))]
pub use attachment::{
    AttachmentDesc, AttachmentEvent, AttachmentEventKind, AttachmentHandle, AttachmentPoint,
};
#[cfg(any(feature = "f32", feature = "f64"))]
pub mod diagnostics;
#[cfg(any(feature = "f32", feature = "f64"))]
pub mod scene;
#[cfg(any(feature = "f32", feature = "f64"))]
pub mod world;
#[cfg(any(feature = "f32", feature = "f64"))]
pub use diagnostics::{IntegrationError, WorldStepReport};
#[cfg(all(feature = "f32", not(feature = "f64")))]
pub use rapier_f32 as rapier;
#[cfg(feature = "f64")]
pub use rapier_f64 as rapier;
#[cfg(any(feature = "f32", feature = "f64"))]
pub use scene::{RapierScene, SceneSnapshot, WorldId};
#[cfg(any(feature = "f32", feature = "f64"))]
pub use world::{ClothCheckpoint, CollisionSettings, RapierClothWorld};

pub mod prelude {
    pub use crate::{Cloth, ClothMaterial, ClothMesh, GridBuilder, Real, SolverSettings, Vec3};
    #[cfg(any(feature = "f32", feature = "f64"))]
    pub use crate::{RapierClothWorld, RapierScene, SceneSnapshot, WorldId};
}
