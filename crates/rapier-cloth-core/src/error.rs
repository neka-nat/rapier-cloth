use std::fmt;

/// Errors never silently repair or discard unsupported input.
#[derive(Debug, Clone, PartialEq)]
pub enum ClothError {
    InvalidMesh(String),
    InvalidParameter(&'static str),
    InvalidHandle,
    InvalidParticle(u32),
    ConflictingTarget(u32),
    DegenerateConstraint,
    NonFiniteState,
    ContactBudgetExceeded {
        limit: usize,
    },
    InvalidSurfaceContact(&'static str),
    InfeasibleSurfaceContact,
    UnresolvedSurfaceContact,
    UnresolvedContinuousCollision(&'static str),
    InitialSelfIntersection {
        triangles: [u32; 2],
    },
    CollisionBudgetExceeded {
        kind: crate::collision::CollisionBudgetKind,
        limit: usize,
    },
    External(String),
}

impl fmt::Display for ClothError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidMesh(s) => write!(f, "invalid cloth mesh: {s}"),
            Self::InvalidParameter(s) => write!(f, "invalid parameter: {s}"),
            Self::InvalidHandle => write!(f, "invalid or expired cloth handle"),
            Self::InvalidParticle(i) => write!(f, "invalid particle index {i}"),
            Self::ConflictingTarget(i) => write!(f, "conflicting targets on particle {i}"),
            Self::DegenerateConstraint => write!(f, "constraint geometry became degenerate"),
            Self::NonFiniteState => write!(f, "non-finite simulation state; substep not committed"),
            Self::ContactBudgetExceeded { limit } => write!(f, "contact budget exceeded ({limit})"),
            Self::InvalidSurfaceContact(s) => write!(f, "invalid surface contact: {s}"),
            Self::InfeasibleSurfaceContact => {
                write!(f, "surface contact conflicts with fixed targets")
            }
            Self::UnresolvedSurfaceContact => {
                write!(
                    f,
                    "surface contact did not converge within the separation tolerance"
                )
            }
            Self::UnresolvedContinuousCollision(reason) => write!(
                f,
                "continuous collision could not certify the proposed motion: {reason}"
            ),
            Self::InitialSelfIntersection { triangles } => write!(
                f,
                "initial self-intersection between triangles {triangles:?}"
            ),
            Self::CollisionBudgetExceeded { kind, limit } => {
                write!(f, "surface collision {kind:?} budget exceeded ({limit})")
            }
            Self::External(s) => write!(f, "external contact error: {s}"),
        }
    }
}
impl std::error::Error for ClothError {}
