use crate::{ClothError, Real};

/// CPU execution budget for one implicit cloth solve. Numerical accumulation
/// order is the same in both modes. Host applications own scheduling across
/// cloths; no global pool or thread-affinity policy is installed.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum ImplicitExecution {
    /// Run on the calling thread, without creating workers.
    #[default]
    Serial,
    /// Use the calling thread and up to three scoped workers for material and
    /// sparse-column assembly. Small inputs and other phases stay serial.
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
    pub max_iterations: usize,
    pub max_line_search_iterations: usize,
    /// RMS Newton displacement divided by h, in m/s, for three iterations.
    pub velocity_tolerance: Real,
    /// Per-contact barrier stiffness, in N/m; this is a numerical contact
    /// parameter and is not a continuum material modulus.
    pub barrier_stiffness: Real,
    /// Smooth Coulomb transition velocity, in m/s. Nonzero values allow creep
    /// under static tangential load; this is not exact static friction.
    pub friction_velocity: Real,
}

impl Default for ImplicitSettings {
    fn default() -> Self {
        Self {
            material: ShellMaterial::default(),
            execution: ImplicitExecution::Serial,
            max_iterations: 80,
            max_line_search_iterations: 24,
            velocity_tolerance: 0.001,
            barrier_stiffness: 30.0,
            friction_velocity: 0.001,
        }
    }
}

impl ImplicitSettings {
    pub fn validate(&self) -> Result<(), ClothError> {
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
        {
            return Err(ClothError::InvalidParameter("implicit solver parameter"));
        }
        Ok(())
    }
}
