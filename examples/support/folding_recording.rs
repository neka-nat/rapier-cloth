//! Recording v1 already supports multiple bodies and per-particle anchors.
use super::{folding, recording};
use rapier_cloth::{StepReport, Vec3, rapier::prelude::Pose};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
pub struct Outcome {
    pub stop_reason: String,
    pub steps: u64,
    pub end_step: u64,
    pub failure: Option<String>,
}

#[derive(Serialize)]
pub struct FinishedRecording {
    #[serde(flatten)]
    pub data: recording::Recording<folding::Config>,
    pub outcome: Outcome,
}

#[derive(Default)]
pub struct Recorder {
    pub recording: Option<recording::Recording<folding::Config>>,
    diagnostics: recording::Diagnostics,
}
impl Recorder {
    pub fn finish(
        self,
        report: &serde_json::Value,
    ) -> Result<Option<FinishedRecording>, Box<dyn std::error::Error>> {
        let Some(data) = self.recording else {
            return Ok(None);
        };
        let outcome: Outcome = serde_json::from_value(serde_json::json!({
            "stop_reason":report["stop_reason"], "steps":report["steps"],
            "end_step":data.config.end_step, "failure":report["failure"]
        }))?;
        if data.frames.last().map(|frame| frame.step) != Some(outcome.steps) {
            return Err("recording does not end at the reported accepted step".into());
        }
        Ok(Some(FinishedRecording { data, outcome }))
    }

    pub fn observe(
        &mut self,
        task: &folding::FoldingWorld,
        report: Option<&StepReport>,
        last: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        if task
            .world
            .surface_attachments()
            .any(|(_, attachment)| attachment.cloth == task.cloth)
        {
            return Err("recording v1 cannot encode weighted surface attachments".into());
        }
        if self.recording.is_none() {
            let mut shapes = vec![recording::BodyShape {
                id: 0,
                kind: "box".into(),
                half_extents: [0.35, 0.01, 0.35],
                local_translation: [0.0; 3],
                color: "#94a3b8".into(),
            }];
            for g in 0..2 {
                let collider = &task.rigid.colliders[task.gripper_colliders[g]];
                let cuboid = collider.shape().as_cuboid().ok_or("expected box gripper")?;
                shapes.push(recording::BodyShape {
                    id: g as u32 + 1,
                    kind: "box".into(),
                    half_extents: cuboid.half_extents.to_array(),
                    local_translation: collider
                        .position_wrt_parent()
                        .ok_or("gripper collider has no parent")?
                        .translation
                        .to_array(),
                    color: if g == 0 { "#f97316" } else { "#0ea5e9" }.into(),
                });
            }
            self.recording = Some(recording::Recording {
                schema_version: recording::SCHEMA_VERSION,
                precision: if cfg!(feature = "f64") { "f64" } else { "f32" }.into(),
                config: task.config.clone(),
                triangles: task.world.cloth(task.cloth)?.mesh().triangles().to_vec(),
                shapes,
                frames: vec![],
            });
        }
        if let Some(report) = report {
            self.diagnostics = recording::Diagnostics {
                max_stretch: report.max_stretch,
                p95_stretch: report.p95_stretch,
                max_penetration: report.max_penetration,
                max_target_error: report.max_target_error,
                contacts: report.contacts,
            };
        }
        let recording = self.recording.as_mut().unwrap();
        if recording
            .frames
            .last()
            .is_some_and(|frame| frame.step == task.step)
            || (!last && !task.step.is_multiple_of(4))
        {
            return Ok(());
        }
        let mut bodies = vec![recording::body_frame(
            0,
            &Pose::translation(0.0, -0.01, 0.0),
        )];
        for g in 0..2 {
            bodies.push(recording::body_frame(
                g as u32 + 1,
                task.rigid.bodies[task.grippers[g]].position(),
            ));
        }
        let mut attached_particles = vec![];
        let mut anchors = vec![];
        for (_, attachment) in task.world.attachments() {
            if attachment.cloth != task.cloth {
                continue;
            }
            for point in &attachment.points {
                attached_particles.push(point.particle);
                anchors.push(
                    task.rigid.bodies[attachment.body]
                        .position()
                        .transform_point(point.local_anchor)
                        .to_array(),
                );
            }
        }
        let cloth = task.world.cloth(task.cloth)?;
        recording.frames.push(recording::Frame {
            step: task.step,
            time: task.step as f64 * task.config.h,
            phase: format!("{:?}", folding::phase(&task.config, task.step)),
            positions: cloth.positions().iter().map(Vec3::to_array).collect(),
            bodies,
            attached_particles,
            anchors,
            pinned_particles: cloth.pins().keys().copied().collect(),
            diagnostics: self.diagnostics.clone(),
        });
        Ok(())
    }
}
