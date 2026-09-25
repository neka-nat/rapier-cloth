use crate::{ClothError, Real};

/// Action when the Newton iteration budget is exhausted. Other failures always
/// reject the step. Task qualification is limited to the documented towel fixture.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum ImplicitCapPolicy {
    /// Reject an unconverged step and preserve the accepted state.
    #[default]
    Strict,
    /// Return the last accepted iterate only after final CCD, contact, finite-state,
    /// hard-target and less-than-3% edge-extension checks. Requires the default
    /// iteration cap, tolerance and convergence window. This does not certify
    /// stationary accuracy.
    ApproximateWithFinalValidation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImplicitTermination {
    /// The configured Newton-displacement criterion was met.
    Converged,
    /// The iteration cap was reached; final physical validation passed.
    ApproximateIterationCap,
}

/// Diagnostics for an accepted implicit step, after all final checks succeed.
/// Residual forces are the negative free-particle objective gradient, in newtons.
/// Neither the residual nor `converged` bounds trajectory or shape error.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImplicitOutcome {
    pub termination: ImplicitTermination,
    /// False for `ApproximateIterationCap`, even when the physical checks pass.
    pub converged: bool,
    /// Final backward-Euler objective, in joules.
    pub energy: Real,
    /// RMS residual force magnitude over free particles (zero if all fixed).
    pub force_rms: Real,
    /// Maximum free-particle residual force magnitude.
    pub force_max: Real,
}

/// CPU execution budget for one implicit cloth solve. Numerical accumulation
/// order is the same in both modes. Host applications own scheduling across
/// cloths; no global pool or thread-affinity policy is installed.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum ImplicitExecution {
    /// Run on the calling thread, without creating workers.
    #[default]
    Serial,
    /// Use the calling thread and up to three scoped workers for material and
    /// parent-contact evaluation. Small inputs and other phases stay serial.
    /// Requires a target that supports spawning native threads.
    Parallel4,
}

/// Isotropic thin-shell material in metres and pascals. Areal mass and velocity
/// damping remain in `ClothMaterial`. Collision thickness is configured
/// separately through `ClothContactSettings`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShellMaterial {
    pub youngs_modulus: Real,
    pub poisson_ratio: Real,
    pub thickness: Real,
}

impl Default for ShellMaterial {
    fn default() -> Self {
        Self {
            youngs_modulus: 821000.0,
            poisson_ratio: 0.243,
            thickness: 0.000318,
        }
    }
}

/// Initial iterate of the Newton solve. Both converge to the same objective;
/// the seed changes the iteration count and roundoff-level differences only.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum ImplicitSeed {
    /// Start free vertices at their previous positions.
    Previous,
    /// Start free vertices at `previous + velocity * h`, shortened by the
    /// continuous checks and only if every contact keeps positive clearance;
    /// otherwise fall back to the previous positions. This typically saves
    /// 10–20% of the Newton iterations.
    #[default]
    Velocity,
}

/// Bounded global backward-Euler solve with a Neo-Hookean membrane, dihedral
/// bending, a positive-gap contact barrier and lagged smooth Coulomb friction.
/// The initial supported runtime uses f64 and hard particle grasps. Contact
/// activation margin sets the barrier width and must be positive. Existing
/// collision-work limits apply to the entire physical step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImplicitSettings {
    pub material: ShellMaterial,
    /// Explicit per-cloth CPU budget, also preserved by checkpoints.
    pub execution: ImplicitExecution,
    /// Explicit opt-in to approximate returns; checkpoints preserve this policy.
    pub cap_policy: ImplicitCapPolicy,
    pub max_iterations: usize,
    pub max_line_search_iterations: usize,
    /// RMS Newton displacement divided by h, in m/s. The solve converges once
    /// `convergence_window` consecutive Newton directions are within it, or when
    /// the first direction of the step already is (a state at rest stays put).
    pub velocity_tolerance: Real,
    /// Consecutive iterations that must meet `velocity_tolerance`, 1 to 8. The
    /// default 3 follows the author solver. Smaller windows stop sooner but can
    /// accept the low point of an oscillating iteration far from convergence.
    pub convergence_window: usize,
    /// Per-contact barrier stiffness, in N/m; this is a numerical contact
    /// parameter and is not a continuum material modulus.
    pub barrier_stiffness: Real,
    /// Smooth Coulomb transition velocity, in m/s. Nonzero values allow creep
    /// under static tangential load; this is not exact static friction.
    pub friction_velocity: Real,
    /// Initial Newton iterate; checkpoints preserve it.
    pub seed: ImplicitSeed,
}

impl Default for ImplicitSettings {
    fn default() -> Self {
        Self {
            material: ShellMaterial::default(),
            execution: ImplicitExecution::Serial,
            cap_policy: ImplicitCapPolicy::Strict,
            max_iterations: 128,
            max_line_search_iterations: 24,
            velocity_tolerance: 0.001,
            convergence_window: 3,
            barrier_stiffness: 30.0,
            friction_velocity: 0.001,
            seed: ImplicitSeed::Velocity,
        }
    }
}

impl ImplicitSettings {
    pub fn validate(&self) -> Result<(), ClothError> {
        // Keep approximate returns within the qualified stopping profile.
        let profile = Self::default();
        if self.cap_policy == ImplicitCapPolicy::ApproximateWithFinalValidation
            && (self.max_iterations != profile.max_iterations
                || self.velocity_tolerance != profile.velocity_tolerance
                || self.convergence_window != profile.convergence_window)
        {
            return Err(ClothError::InvalidParameter(
                "approximate cap policy requires baseline profile",
            ));
        }
        if cfg!(feature = "f32") {
            return Err(ClothError::InvalidParameter(
                "the experimental implicit solver requires f64",
            ));
        }
        for value in [
            self.material.youngs_modulus,
            self.material.thickness,
            self.velocity_tolerance,
            self.barrier_stiffness,
            self.friction_velocity,
        ] {
            if !value.is_finite() || value <= 0.0 {
                return Err(ClothError::InvalidParameter("implicit solver parameter"));
            }
        }
        if !self.material.poisson_ratio.is_finite()
            || self.material.poisson_ratio <= -1.0
            || self.material.poisson_ratio >= 0.5
            || !(3..=1024).contains(&self.max_iterations)
            || !(1..=128).contains(&self.max_line_search_iterations)
            || !(1..=8).contains(&self.convergence_window)
        {
            return Err(ClothError::InvalidParameter("implicit solver parameter"));
        }
        Ok(())
    }
}
