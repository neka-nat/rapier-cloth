use rapier_cloth::Real;
use serde::{Deserialize, Serialize};
mod fold;
#[cfg(all(feature = "f64", feature = "implicit"))]
mod implicit;
mod legacy;
use fold::{FoldDemo, FoldingFrame};
use legacy::LegacyDemo;

pub const H: Real = 1.0 / 240.0;
pub const SUBSTEPS: usize = 4;
pub const PROTOCOL: u32 = 2;

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum SceneKind {
    Drape,
    Hanging,
    FoldTowel,
    ImplicitTowel,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Options {
    pub auto_motion: bool,
    pub sphere_x: Real,
    pub sphere_z: Real,
    pub wind: Real,
}
impl Options {
    fn validate(self) -> Result<Self, String> {
        if !self.sphere_x.is_finite()
            || self.sphere_x.abs() > 0.45
            || !self.sphere_z.is_finite()
            || self.sphere_z.abs() > 0.45
            || !self.wind.is_finite()
            || !(0.0..=1.0).contains(&self.wind)
        {
            return Err(
                "Invalid options. Sphere targets must be within ±0.45 m and wind between 0 and 1."
                    .into(),
            );
        }
        Ok(self)
    }
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum Command {
    Step,
    Reset {
        scene: SceneKind,
    },
    SetOptions {
        options: Options,
    },
    Release,
    ReleaseGripper {
        gripper: usize,
    },
    Inspect,
    // Parse the matching protocol even when this build cannot execute it.
    #[cfg_attr(not(all(feature = "f64", feature = "implicit")), allow(dead_code))]
    SetGripperPose {
        gripper: usize,
        translation: [Real; 3],
        rotation: [Real; 4],
        at_step: usize,
    },
    #[cfg_attr(not(all(feature = "f64", feature = "implicit")), allow(dead_code))]
    GraspGripper {
        gripper: usize,
    },
}

#[derive(Serialize)]
pub struct Frame {
    pub r#type: &'static str,
    pub protocol: u32,
    pub request_id: u32,
    pub scene: SceneKind,
    pub precision: &'static str,
    pub step: u64,
    pub time: f64,
    pub h: Real,
    pub substeps: usize,
    pub advanced_substeps: usize,
    pub iterations: usize,
    pub positions: Vec<Real>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub triangles: Option<Vec<u32>>,
    pub pins: Vec<u32>,
    pub sphere: [Real; 3],
    pub sphere_radius: Real,
    pub options: Options,
    pub physics_ms: f64,
    pub p95_stretch: Real,
    pub max_penetration: Real,
    pub contacts: usize,
    pub max_target_error: Real,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub folding: Option<FoldingFrame>,
    pub implicit_available: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub implicit: Option<serde_json::Value>,
}

pub enum Demo {
    Free(Box<LegacyDemo>),
    Folding(Box<FoldDemo>),
    #[cfg(all(feature = "f64", feature = "implicit"))]
    Implicit(Box<implicit::ImplicitDemo>),
}
impl Demo {
    pub fn new(scene: SceneKind) -> Result<Self, String> {
        match scene {
            SceneKind::ImplicitTowel => {
                #[cfg(all(feature = "f64", feature = "implicit"))]
                {
                    Ok(Self::Implicit(Box::new(implicit::ImplicitDemo::new()?)))
                }
                #[cfg(not(all(feature = "f64", feature = "implicit")))]
                {
                    Err("Rebuild the live server with --no-default-features --features f64,implicit".into())
                }
            }
            SceneKind::FoldTowel => Ok(Self::Folding(Box::new(FoldDemo::new()?))),
            _ => Ok(Self::Free(Box::new(LegacyDemo::new(scene)?))),
        }
    }
    pub fn command(&mut self, command: Command, request_id: u32) -> Result<Frame, String> {
        if let Command::Reset { scene } = command {
            *self = Self::new(scene)?;
            return Ok(self.frame(request_id, true));
        }
        match self {
            Self::Free(demo) => demo.command(command, request_id),
            Self::Folding(demo) => demo.command(command, request_id),
            #[cfg(all(feature = "f64", feature = "implicit"))]
            Self::Implicit(demo) => demo.command(command, request_id),
        }
    }
    pub fn frame(&self, request_id: u32, topology: bool) -> Frame {
        match self {
            Self::Free(demo) => demo.frame(request_id, topology),
            Self::Folding(demo) => demo.frame(request_id, topology),
            #[cfg(all(feature = "f64", feature = "implicit"))]
            Self::Implicit(demo) => demo.frame(request_id, topology),
        }
    }
}

#[cfg(all(test, not(all(feature = "f64", feature = "implicit"))))]
mod unavailable_tests {
    use super::*;
    #[test]
    fn unavailable_scene_does_not_replace_existing_world() {
        let mut demo = Demo::new(SceneKind::Hanging).unwrap();
        let before = demo.frame(0, false);
        assert!(!before.implicit_available);
        assert!(
            demo.command(
                Command::Reset {
                    scene: SceneKind::ImplicitTowel
                },
                1
            )
            .is_err()
        );
        let after = demo.frame(2, false);
        assert_eq!(after.scene, SceneKind::Hanging);
        assert_eq!(after.positions, before.positions);
        assert_eq!(after.step, before.step);
    }
}
