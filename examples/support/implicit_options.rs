//! Serializable configuration and diagnostics for the examples and live transport.
#![allow(dead_code)]
use serde::{Deserialize, Serialize};

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CapPolicy {
    #[default]
    Strict,
    Approximate,
}
impl CapPolicy {
    pub fn parse(value: Option<String>) -> Result<Self, &'static str> {
        match value.as_deref() {
            Some("strict") => Ok(Self::Strict),
            Some("approximate") => Ok(Self::Approximate),
            _ => Err("--cap-policy requires strict or approximate"),
        }
    }
}
#[cfg(all(feature = "f64", feature = "implicit"))]
impl From<CapPolicy> for rapier_cloth::ImplicitCapPolicy {
    fn from(value: CapPolicy) -> Self {
        match value {
            CapPolicy::Strict => Self::Strict,
            CapPolicy::Approximate => Self::ApproximateWithFinalValidation,
        }
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(default, deny_unknown_fields)]
pub struct ImplicitOptions {
    pub variant: String,
    pub cap_policy: CapPolicy,
}
impl Default for ImplicitOptions {
    fn default() -> Self {
        Self {
            variant: "nominal".into(),
            cap_policy: CapPolicy::Strict,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Termination {
    Converged,
    ApproximateIterationCap,
}

#[cfg(all(feature = "f64", feature = "implicit"))]
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct Outcome {
    pub termination: Termination,
    pub converged: bool,
    pub energy: rapier_cloth::Real,
    pub force_rms: rapier_cloth::Real,
    pub force_max: rapier_cloth::Real,
}
#[cfg(all(feature = "f64", feature = "implicit"))]
impl From<rapier_cloth::ImplicitOutcome> for Outcome {
    fn from(value: rapier_cloth::ImplicitOutcome) -> Self {
        Self {
            termination: match value.termination {
                rapier_cloth::ImplicitTermination::Converged => Termination::Converged,
                rapier_cloth::ImplicitTermination::ApproximateIterationCap => {
                    Termination::ApproximateIterationCap
                }
            },
            converged: value.converged,
            energy: value.energy,
            force_rms: value.force_rms,
            force_max: value.force_max,
        }
    }
}
