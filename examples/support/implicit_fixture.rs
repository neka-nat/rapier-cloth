//! Bounded variations of the retained towel commands, shared by headless and live examples.
#![allow(dead_code)]
use rapier_cloth::Real;
use serde::Deserialize;

pub const CASES: &[&str] = &[
    "nominal",
    "grasp_inset",
    "lift_5mm",
    "left_early",
    "right_late",
    "friction_low",
    "friction_high",
];

#[derive(Clone, Deserialize)]
pub struct Input {
    pub x: Vec<[Real; 3]>,
    pub faces: Vec<[u32; 3]>,
    pub grasp: Vec<u32>,
    pub targets: Vec<Option<Vec<[Real; 3]>>>,
    pub h: Real,
    pub thickness: Real,
}
impl Input {
    pub fn load(h: Real, variant: &str) -> Result<Self, String> {
        if !CASES.contains(&variant) {
            return Err(format!(
                "unknown case {variant}; choose {}",
                CASES.join(", ")
            ));
        }
        let text = if h == 0.1 {
            include_str!("../assets/towel-fold-10hz.json")
        } else if h == 0.04 {
            include_str!("../assets/towel-fold-25hz.json")
        } else {
            return Err("--dt must be 0.04 or 0.1".into());
        };
        let mut input: Self = serde_json::from_str(text).map_err(|e| e.to_string())?;
        if variant == "grasp_inset" {
            for (j, particle) in input.grasp.iter_mut().enumerate() {
                let old = *particle as usize;
                *particle = if input.x[old][0] < 0.0 {
                    *particle + 1
                } else {
                    *particle - 1
                };
                let dx = input.x[*particle as usize][0] - input.x[old][0];
                for points in input.targets.iter_mut().flatten() {
                    points[j][0] += dx;
                }
            }
        }
        if variant == "lift_5mm" {
            for (step, targets) in input.targets.iter_mut().enumerate() {
                if let Some(points) = targets {
                    let dy = 0.005
                        * (std::f64::consts::PI as Real * ((step + 1) as Real * h) / 4.0).sin();
                    for p in points {
                        p[1] += dy;
                    }
                }
            }
        }
        Ok(input)
    }
    pub fn side(&self, particle: u32) -> usize {
        usize::from(self.x[particle as usize][0] >= 0.0)
    }
    pub fn target(&self, step: usize, selected: usize, variant: &str) -> Option<[Real; 3]> {
        let side = self.side(self.grasp[selected]);
        let release = 4.0
            + match (variant, side) {
                ("left_early", 0) => -self.h,
                ("right_late", 1) => self.h,
                _ => 0.0,
            };
        if step as Real * self.h >= release - 1e-9 {
            return None;
        }
        let targets = self
            .targets
            .get(step)
            .and_then(Option::as_ref)
            .or_else(|| self.targets.iter().rev().find_map(Option::as_ref))?;
        Some(targets[selected])
    }
    pub fn friction(variant: &str) -> Real {
        match variant {
            "friction_low" => 0.4,
            "friction_high" => 0.6,
            _ => 0.5,
        }
    }
}
