use super::{Command, Frame, Options, PROTOCOL, SceneKind};
use crate::implicit_robot::RobotTowel;
use rapier_cloth::ImplicitExecution;
use serde_json::json;

pub struct ImplicitDemo {
    pub task: RobotTowel,
    advanced: usize,
}
impl ImplicitDemo {
    pub fn new() -> Result<Self, String> {
        Ok(Self {
            task: RobotTowel::new("nominal", ImplicitExecution::Parallel4)?,
            advanced: 0,
        })
    }
    pub fn command(&mut self, command: Command, request_id: u32) -> Result<Frame, String> {
        self.advanced = 0;
        match command {
            Command::Step => {
                if !self.task.finished() && self.task.stopped.is_none() && self.task.tick().is_ok()
                {
                    self.advanced = 1;
                }
            }
            Command::SetGripperPose {
                gripper,
                translation,
                rotation,
                at_step,
            } => self
                .task
                .set_pose(gripper, translation, rotation, at_step)?,
            Command::GraspGripper { gripper } => self.task.grasp(gripper)?,
            Command::ReleaseGripper { gripper } => self.task.release(gripper)?,
            Command::Inspect => {}
            _ => return Err("This command does not apply to the implicit towel".into()),
        }
        Ok(self.frame(request_id, false))
    }
    pub fn frame(&self, request_id: u32, topology: bool) -> Frame {
        let t = &self.task;
        let cloth = t.world.cloth(t.cloth).expect("task cloth");
        let grippers:Vec<_>=(0..2).map(|side| {
            let pose=t.pose(side);
            json!({"id":side,"translation":pose.translation.to_array(),"rotation":pose.rotation.to_array(),
                "local_translation":[0.0,0.012,0.0],"local_rotation":[0.0,0.0,0.0,1.0],"half_extents":[0.012,0.01,0.012],
                "holding":t.attachments[side].is_some()})
        }).collect();
        let grasps: Vec<_> = t
            .world
            .attachments()
            .map(|(_, a)| {
                let side = usize::from(a.body == t.grippers[1]);
                let pose = t.pose(side);
                json!({"gripper":side,"points":a.points.iter().map(|p|json!({
                "material":{"kind":"vertex","particle":p.particle},
                "position":cloth.positions()[p.particle as usize].to_array(),
                "anchor":pose.transform_point(p.local_anchor).to_array()
            })).collect::<Vec<_>>()})
            })
            .collect();
        let phase = if t.finished() {
            "Finished"
        } else if !t.automatic {
            "Manual control"
        } else if t.attachments.iter().all(Option::is_none) {
            "Released / settling"
        } else {
            "Folding"
        };
        Frame {
            r#type: "frame",
            protocol: PROTOCOL,
            request_id,
            scene: SceneKind::ImplicitTowel,
            precision: "f64",
            step: t.step as u64,
            time: t.step as f64 * t.input.h,
            h: t.input.h,
            substeps: 1,
            advanced_substeps: self.advanced,
            iterations: t.report.iterations,
            positions: cloth
                .positions()
                .iter()
                .flat_map(|p| p.to_array())
                .collect(),
            triangles: topology.then(|| t.input.faces.iter().flatten().copied().collect()),
            pins: vec![],
            sphere: [0.0; 3],
            sphere_radius: 0.0,
            options: Options {
                auto_motion: false,
                sphere_x: 0.0,
                sphere_z: 0.0,
                wind: 0.0,
            },
            physics_ms: t.physics_ms,
            p95_stretch: t.report.p95_stretch,
            max_penetration: t.report.max_penetration,
            contacts: t.report.contacts,
            max_target_error: t.report.max_target_error,
            folding: None,
            implicit_available: true,
            implicit: Some(
                json!({"solver":"implicit","phase":phase,"automatic":t.automatic,
                    "stopped":t.stopped,"completed":t.finished(),"end_step":t.input.targets.len(),
                    "floor_height":t.input.thickness*0.5,"grippers":grippers,"grasps":grasps,"sample":t.sample(),
                    "desired":t.desired.map(|p|json!({"translation":p.translation.to_array(),"rotation":p.rotation.to_array()}))
                }),
            ),
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn live_and_headless_advance_identically_and_inspection_is_read_only() {
        let mut live = ImplicitDemo::new().unwrap();
        let mut headless = RobotTowel::new("nominal", ImplicitExecution::Parallel4).unwrap();
        for step in 1..=3 {
            headless.tick().unwrap();
            let frame = live.command(Command::Step, step).unwrap();
            assert_eq!(
                frame.positions,
                headless
                    .positions()
                    .iter()
                    .flat_map(|p| p.to_array())
                    .collect::<Vec<_>>()
            );
            assert_eq!(frame.advanced_substeps, 1);
        }
        let before = live.frame(0, false).positions;
        let frame = live.command(Command::Inspect, 4).unwrap();
        assert_eq!(frame.positions, before);
        assert_eq!(frame.step, 3);
        assert_eq!(frame.advanced_substeps, 0);
        live.command(Command::ReleaseGripper { gripper: 0 }, 5)
            .unwrap();
        assert_eq!(live.task.held_particles().len(), 26);
        assert_eq!(live.frame(0, false).positions, before);
    }
    #[test]
    fn stopped_response_retains_accepted_state_and_reset_recovers_both_worlds() {
        let mut demo = super::super::Demo::new(SceneKind::ImplicitTowel).unwrap();
        let initial = demo.frame(0, true);
        let super::super::Demo::Implicit(live) = &mut demo else {
            unreachable!()
        };
        let mut contact = live
            .task
            .world
            .cloth(live.task.cloth)
            .unwrap()
            .contact_settings()
            .unwrap();
        contact.limits.candidate_pairs = 1;
        live.task
            .world
            .cloth_mut(live.task.cloth)
            .unwrap()
            .set_contact_settings(Some(contact))
            .unwrap();
        let failed = demo.command(Command::Step, 1).unwrap();
        assert_eq!(failed.step, 0);
        assert_eq!(failed.advanced_substeps, 0);
        assert_eq!(failed.positions, initial.positions);
        let data = failed.implicit.as_ref().unwrap();
        assert!(data["stopped"].is_string());
        assert_eq!(
            data["grippers"],
            initial.implicit.as_ref().unwrap()["grippers"]
        );
        assert_eq!(data["grasps"], initial.implicit.as_ref().unwrap()["grasps"]);
        assert_eq!(
            demo.command(Command::Step, 2).unwrap().positions,
            initial.positions
        );
        let reset = demo
            .command(
                Command::Reset {
                    scene: SceneKind::ImplicitTowel,
                },
                3,
            )
            .unwrap();
        assert_eq!(reset.positions, initial.positions);
        assert!(reset.implicit.as_ref().unwrap()["stopped"].is_null());
        assert_eq!(demo.command(Command::Step, 4).unwrap().step, 1);
    }
}
