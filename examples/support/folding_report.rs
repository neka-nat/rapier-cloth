//! Task observations outside the timed physics scope. Shared with report tests.
#![allow(dead_code)]
use crate::folding::FoldingWorld;
use rapier_cloth::IntegrationError;
use serde::Serialize;

#[derive(Debug, Default, Clone, Copy, Serialize, PartialEq, Eq)]
pub struct TargetCounts {
    pub vertex_attachments: usize,
    pub surface_attachments: usize,
    pub pins: usize,
    /// Target points supplied by this task's world attachments and cloth pins.
    pub target_points: usize,
    pub commanded_grasps: usize,
}
pub fn target_counts(task: &FoldingWorld) -> Result<TargetCounts, IntegrationError> {
    let pins = task.world.cloth(task.cloth)?.pins().len();
    Ok(TargetCounts {
        vertex_attachments: task.world.attachments().count(),
        surface_attachments: task.world.surface_attachments().count(),
        pins,
        target_points: pins
            + task
                .world
                .attachments()
                .map(|(_, a)| a.points.len())
                .sum::<usize>()
            + task
                .world
                .surface_attachments()
                .map(|(_, a)| a.points.len())
                .sum::<usize>(),
        commanded_grasps: task.attachments.iter().filter(|h| h.is_some()).count(),
    })
}

#[derive(Debug, Serialize)]
pub struct TaskAudit {
    pub observed_substeps: u64,
    pub all_substeps_finite: bool,
    pub max_table_penetration: f64,
    pub grasped_substeps: u64,
    pub released_substeps: u64,
    pub released_substeps_without_targets: u64,
    pub settle_samples: u64,
    pub settle_start_step: Option<u64>,
    pub settle_end_step: Option<u64>,
    pub settle_drift: f64,
    #[serde(skip)]
    settle_center: Option<[f64; 3]>,
}
impl Default for TaskAudit {
    fn default() -> Self {
        Self {
            observed_substeps: 0,
            all_substeps_finite: true,
            max_table_penetration: 0.0,
            grasped_substeps: 0,
            released_substeps: 0,
            released_substeps_without_targets: 0,
            settle_samples: 0,
            settle_start_step: None,
            settle_end_step: None,
            settle_drift: 0.0,
            settle_center: None,
        }
    }
}
impl TaskAudit {
    /// Observe exactly once after each accepted substep, including the boundary
    /// before the final five-second settling interval. Failure is not an observation.
    pub fn observe(&mut self, task: &FoldingWorld) -> Result<(), IntegrationError> {
        let cloth = task.world.cloth(task.cloth)?;
        self.observed_substeps += 1;
        self.all_substeps_finite &= cloth
            .positions()
            .iter()
            .chain(cloth.velocities())
            .all(|p| p.is_finite());
        for &p in cloth.positions() {
            let y = crate::folding::point(p)[1];
            self.max_table_penetration = self
                .max_table_penetration
                .max((task.config.thickness * 0.5 - y).max(0.0));
        }
        let targets = target_counts(task)?;
        if (task.config.attach_step..task.config.release_step).contains(&task.step)
            && targets
                == (TargetCounts {
                    vertex_attachments: 2,
                    target_points: 8,
                    commanded_grasps: 2,
                    ..Default::default()
                })
        {
            self.grasped_substeps += 1;
        }
        if task.step >= task.config.release_step {
            self.released_substeps += 1;
            if targets == TargetCounts::default() {
                self.released_substeps_without_targets += 1;
            }
        }
        if task.step >= task.config.retract_end {
            let center = task.center_of_mass();
            self.all_substeps_finite &= center.iter().all(|v| v.is_finite());
            let initial = self.settle_center.get_or_insert(center);
            self.settle_start_step.get_or_insert(task.step);
            self.settle_end_step = Some(task.step);
            self.settle_samples += 1;
            self.settle_drift = self
                .settle_drift
                .max((center[0] - initial[0]).hypot(center[2] - initial[2]));
        }
        Ok(())
    }
}
