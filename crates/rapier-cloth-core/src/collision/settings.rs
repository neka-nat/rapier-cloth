use crate::{ClothError, Real};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CollisionBudgetKind {
    CandidatePairs,
    RetainedContacts,
    CcdChecks,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CollisionLimits {
    /// Cumulative primitive candidates over one substep, including refreshes.
    pub candidate_pairs: usize,
    /// Maximum simultaneous/cumulative retained contact keys, not iteration work.
    pub retained_contacts: usize,
    /// Cumulative narrow-phase distance evaluations for continuous advancement.
    pub ccd_checks: usize,
}
impl Default for CollisionLimits {
    fn default() -> Self {
        Self {
            candidate_pairs: 2_000_000,
            retained_contacts: 65_536,
            ccd_checks: 2_000_000,
        }
    }
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClothContactSettings {
    /// Full physical thickness. Self-contact separates two midsurfaces by this value.
    pub thickness: Real,
    /// Candidate activation distance in addition to physical separation.
    pub activation_margin: Real,
    pub self_collision: bool,
    /// Bound every accepted self-contact motion, including solver corrections.
    /// Experimental; rigid surface CCD is separate. Uses 90% of full thickness
    /// as the minimum swept separation while projections target full thickness.
    pub continuous_self_collision: bool,
    /// Use whole-triangle contacts in supporting external adapters. In the
    /// Rapier bridge this replaces the legacy particle-radius collision path.
    /// Discrete only; continuous rigid motion is a separate capability.
    pub rigid_surface_collision: bool,
    pub static_friction: Real,
    pub kinetic_friction: Real,
    pub limits: CollisionLimits,
}
impl Default for ClothContactSettings {
    fn default() -> Self {
        Self {
            thickness: 0.001,
            activation_margin: 0.0001,
            self_collision: true,
            continuous_self_collision: false,
            rigid_surface_collision: false,
            static_friction: 0.6,
            kinetic_friction: 0.5,
            limits: CollisionLimits::default(),
        }
    }
}
impl ClothContactSettings {
    pub fn validate(&self) -> Result<(), ClothError> {
        if self.continuous_self_collision && !self.self_collision {
            return Err(ClothError::InvalidParameter(
                "continuous self-collision requires self-collision",
            ));
        }
        if !self.thickness.is_finite()
            || self.thickness <= 0.0
            || !self.activation_margin.is_finite()
            || self.activation_margin < 0.0
            || !(self.thickness + self.activation_margin).is_finite()
            || !self.static_friction.is_finite()
            || !self.kinetic_friction.is_finite()
            || self.kinetic_friction < 0.0
            || self.static_friction < self.kinetic_friction
        {
            return Err(ClothError::InvalidParameter(
                "surface thickness, activation margin or friction",
            ));
        }
        if self.limits.candidate_pairs == 0
            || self.limits.retained_contacts == 0
            || self.limits.ccd_checks == 0
        {
            return Err(ClothError::InvalidParameter(
                "surface collision limits must be positive",
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct CollisionWork {
    pub candidate_pairs: usize,
    pub retained_contacts: usize,
    pub ccd_checks: usize,
    /// Number of motion checks that requested a displacement reduction.
    pub limited_advances: usize,
}
impl CollisionWork {
    /// Charge bounded external or built-in collision work before performing it.
    /// Retained contacts use a high-water mark; other counters are cumulative.
    /// On failure the counter is unchanged.
    pub fn charge(
        &mut self,
        kind: CollisionBudgetKind,
        amount: usize,
        limits: CollisionLimits,
    ) -> Result<(), ClothError> {
        let (counter, limit) = match kind {
            CollisionBudgetKind::CandidatePairs => {
                (&mut self.candidate_pairs, limits.candidate_pairs)
            }
            CollisionBudgetKind::RetainedContacts => {
                (&mut self.retained_contacts, limits.retained_contacts)
            }
            CollisionBudgetKind::CcdChecks => (&mut self.ccd_checks, limits.ccd_checks),
        };
        let value = if kind == CollisionBudgetKind::RetainedContacts {
            (*counter).max(amount)
        } else {
            counter
                .checked_add(amount)
                .ok_or(ClothError::CollisionBudgetExceeded { kind, limit })?
        };
        if value > limit {
            return Err(ClothError::CollisionBudgetExceeded { kind, limit });
        }
        *counter = value;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn work_and_capacity_budgets_have_distinct_semantics() {
        let limits = CollisionLimits {
            candidate_pairs: 3,
            retained_contacts: 2,
            ccd_checks: 1,
        };
        let mut work = CollisionWork::default();
        work.charge(CollisionBudgetKind::CandidatePairs, 2, limits)
            .unwrap();
        assert!(
            work.charge(CollisionBudgetKind::CandidatePairs, 2, limits)
                .is_err()
        );
        assert_eq!(work.candidate_pairs, 2);
        for _ in 0..20 {
            work.charge(CollisionBudgetKind::RetainedContacts, 2, limits)
                .unwrap();
        }
        assert!(
            work.charge(CollisionBudgetKind::RetainedContacts, 3, limits)
                .is_err()
        );
        work.charge(CollisionBudgetKind::CcdChecks, 1, limits)
            .unwrap();
        assert_eq!(
            work.charge(CollisionBudgetKind::CcdChecks, 1, limits),
            Err(ClothError::CollisionBudgetExceeded {
                kind: CollisionBudgetKind::CcdChecks,
                limit: 1
            })
        );
    }
}
