use super::{Command, Frame, Options, PROTOCOL, SUBSTEPS, SceneKind};
use crate::{folding::*, folding_oracle};
use rapier_cloth::{Real, rapier::prelude::Pose, *};
use serde::Serialize;
use std::time::Instant;

#[derive(Serialize)]
pub struct GripperFrame {
    pub id: usize,
    pub translation: [Real; 3],
    pub rotation: [Real; 4],
    pub half_extents: [Real; 3],
    pub local_translation: [Real; 3],
    pub local_rotation: [Real; 4],
    pub holding: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn small() -> FoldDemo {
        let mut config = Config::with_version(2).unwrap();
        config.grid = 4;
        FoldDemo::with_task(configured_world(config, 0, CollisionMode::default()).unwrap())
    }

    #[test]
    fn live_and_headless_share_the_exact_frozen_surface_task() {
        let mut live = FoldDemo::new().unwrap();
        let mut control = configured_world(
            Config::with_version(2).unwrap(),
            0,
            CollisionMode {
                self_collision: true,
                rigid_surface: true,
                continuous_self: true,
                continuous_rigid: true,
            },
        )
        .unwrap();
        for request in 1..=3 {
            let frame = live.command(Command::Step, request).unwrap();
            for _ in 0..4 {
                control.tick().unwrap();
            }
            assert_eq!(frame.advanced_substeps, 4);
            assert_eq!(frame.step, control.step);
            assert_eq!(
                frame.positions,
                control
                    .world
                    .cloth(control.cloth)
                    .unwrap()
                    .positions()
                    .iter()
                    .flat_map(|p| p.to_array())
                    .collect::<Vec<_>>()
            );
            let folding = frame.folding.unwrap();
            assert_eq!(
                serde_json::to_value(folding.config).unwrap(),
                serde_json::to_value(&control.config).unwrap()
            );
            assert!(
                folding.collision_mode.continuous_self && folding.collision_mode.continuous_rigid
            );
            for (g, expected) in folding.grippers.iter().zip(control.grippers) {
                assert_eq!(
                    g.translation,
                    control.rigid.bodies[expected].translation().to_array()
                );
            }
        }
        let before = live.task.positions();
        let frame = live.command(Command::Inspect, 4).unwrap();
        assert_eq!(frame.step, 12);
        assert_eq!(frame.advanced_substeps, 0);
        assert_eq!(frame.folding.unwrap().inspection.unwrap().step, 12);
        assert_eq!(live.task.positions(), before);
        assert!(
            live.command(Command::ReleaseGripper { gripper: 2 }, 5)
                .is_err()
        );
        assert_eq!(live.task.positions(), before);
        live.command(Command::Step, 6).unwrap();
        assert!(live.inspection.is_none());
    }

    #[test]
    fn one_gripper_releases_vertex_and_weighted_grasps_without_advancing_or_releasing_the_other() {
        let mut live = small();
        while live.task.step < live.task.config.attach_step {
            live.command(Command::Step, 1).unwrap();
        }
        let weighted = SurfacePoint::new(12, [0.2, 0.3, 0.5]).unwrap();
        live.task
            .world
            .grasp_surface(
                ClothSurfacePoint {
                    cloth: live.task.cloth,
                    point: weighted,
                },
                GraspOptions::new(live.task.grippers[0]),
                &live.task.rigid.bodies,
                &live.task.rigid.colliders,
            )
            .unwrap();
        let before = live.frame(1, false);
        let before_velocity = live
            .task
            .world
            .cloth(live.task.cloth)
            .unwrap()
            .velocities()
            .to_vec();
        let data = before.folding.as_ref().unwrap();
        assert_eq!(data.grasps.len(), 3);
        let surface = data
            .grasps
            .iter()
            .flat_map(|g| &g.points)
            .find(|p| matches!(p.material, MaterialPoint::Surface { .. }))
            .unwrap();
        assert_eq!(
            surface.position,
            live.task
                .world
                .surface_point_position(ClothSurfacePoint {
                    cloth: live.task.cloth,
                    point: weighted
                })
                .unwrap()
                .to_array()
        );
        let released = live
            .command(Command::ReleaseGripper { gripper: 0 }, 2)
            .unwrap();
        assert_eq!(released.positions, before.positions);
        assert_eq!(released.step, before.step);
        assert_eq!(released.advanced_substeps, 0);
        assert_eq!(
            live.task.world.cloth(live.task.cloth).unwrap().velocities(),
            before_velocity
        );
        let data = released.folding.unwrap();
        assert!(data.manual_release && !data.grippers[0].holding && data.grippers[1].holding);
        assert_eq!(data.grasps.len(), 1);
        assert_eq!(live.task.attachments[0], None);
        assert!(live.task.attachments[1].is_some());
        assert_eq!(live.task.world.surface_attachments().count(), 0);
        assert_eq!(live.task.world.drain_attachment_events().count(), 2);
        live.command(Command::ReleaseGripper { gripper: 1 }, 3)
            .unwrap();
        assert_eq!(live.task.world.attachments().count(), 0);
        // The small legacy fixture exercises task lifecycle, not surface quality.
        while live.task.step <= live.task.config.end_step {
            live.command(Command::Step, 4).unwrap();
        }
        assert!(live.stopped.is_none());
        assert_eq!(live.frame(5, false).folding.unwrap().phase, "Finished");
    }

    #[test]
    fn failure_after_partial_frame_returns_last_accepted_state_and_latches_until_reset() {
        let mut live = small();
        let mut control = small();
        for _ in 0..2 {
            control.task.tick().unwrap();
        }
        let mut calls = 0;
        live.advance_with(|task| {
            calls += 1;
            if calls == 3 {
                task.world.solver_settings.max_contacts = 1;
            }
            task.tick()
        });
        let failed = live.frame(1, false);
        assert_eq!(calls, 3);
        assert_eq!(failed.step, 2);
        assert_eq!(failed.advanced_substeps, 2);
        assert_eq!(failed.positions, control.frame(0, false).positions);
        assert!(failed.folding.unwrap().stopped.unwrap().contains("budget"));
        assert!(!live.task.world.is_desynchronized());
        for i in 0..2 {
            assert_eq!(
                live.task.rigid.bodies[live.task.grippers[i]].position(),
                control.task.rigid.bodies[control.task.grippers[i]].position()
            );
        }
        let repeated = live.command(Command::Step, 2).unwrap();
        assert_eq!(repeated.step, 2);
        assert_eq!(repeated.advanced_substeps, 0);
        assert_eq!(repeated.positions, control.frame(0, false).positions);
        live.command(Command::Inspect, 3).unwrap();
        let mut demo = super::super::Demo::Folding(Box::new(live));
        let reset = demo
            .command(
                Command::Reset {
                    scene: SceneKind::FoldTowel,
                },
                4,
            )
            .unwrap();
        assert_eq!(reset.step, 0);
        assert_eq!(reset.advanced_substeps, 0);
        let initial = FoldDemo::new().unwrap().frame(0, true);
        assert_eq!(reset.positions, initial.positions);
        let data = reset.folding.unwrap();
        assert!(!data.manual_release && data.stopped.is_none() && data.inspection.is_none());
        assert!(data.grasps.is_empty());
        let free = demo
            .command(
                Command::Reset {
                    scene: SceneKind::Drape,
                },
                5,
            )
            .unwrap();
        assert!(free.folding.is_none());
        assert_eq!(demo.command(Command::Step, 6).unwrap().step, 4);
    }
}
#[derive(Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum MaterialPoint {
    Vertex {
        particle: u32,
    },
    Surface {
        triangle: u32,
        barycentric: [Real; 3],
    },
}
#[derive(Serialize)]
pub struct GraspPoint {
    pub material: MaterialPoint,
    pub position: [Real; 3],
    pub anchor: [Real; 3],
}
#[derive(Serialize)]
pub struct GraspFrame {
    pub gripper: usize,
    pub points: Vec<GraspPoint>,
}
#[derive(Clone, Serialize)]
pub struct Inspection {
    pub step: u64,
    pub metrics: folding_oracle::FoldMetrics,
}
#[derive(Serialize)]
pub struct FoldingFrame {
    pub config: Config,
    pub variant: usize,
    pub phase: String,
    pub collision_mode: CollisionMode,
    pub grippers: Vec<GripperFrame>,
    pub grasps: Vec<GraspFrame>,
    pub manual_release: bool,
    pub stopped: Option<String>,
    pub inspection: Option<Inspection>,
}

pub struct FoldDemo {
    task: FoldingWorld,
    stopped: Option<String>,
    manual_release: bool,
    physics_ms: f64,
    advanced_substeps: usize,
    report: StepReport,
    inspection: Option<Inspection>,
}

fn rotation(pose: &Pose) -> [Real; 4] {
    let q = pose.rotation;
    [q.x, q.y, q.z, q.w]
}

impl FoldDemo {
    pub fn new() -> Result<Self, String> {
        let task = configured_world(
            Config::with_version(2).map_err(|e| e.to_string())?,
            0,
            CollisionMode {
                self_collision: true,
                rigid_surface: true,
                continuous_self: true,
                continuous_rigid: true,
            },
        )
        .map_err(|e| e.to_string())?;
        Ok(Self::with_task(task))
    }
    fn with_task(task: FoldingWorld) -> Self {
        Self {
            task,
            stopped: None,
            manual_release: false,
            physics_ms: 0.0,
            advanced_substeps: 0,
            report: StepReport::default(),
            inspection: None,
        }
    }
    fn advance_with(
        &mut self,
        mut step: impl FnMut(&mut FoldingWorld) -> Result<WorldStepReport, IntegrationError>,
    ) {
        // A failed physical step is already rolled back by the shared
        // task. Do not retry a failed command automatically.
        if self.stopped.is_none() {
            let begin = Instant::now();
            let mut diagnostics = StepReport::default();
            for _ in 0..SUBSTEPS {
                match step(&mut self.task) {
                    Ok(step) => {
                        self.advanced_substeps += 1;
                        let r = &step.cloths[0].1;
                        diagnostics.p95_stretch = diagnostics.p95_stretch.max(r.p95_stretch);
                        diagnostics.max_stretch = diagnostics.max_stretch.max(r.max_stretch);
                        diagnostics.max_penetration =
                            diagnostics.max_penetration.max(r.max_penetration);
                        diagnostics.max_target_error =
                            diagnostics.max_target_error.max(r.max_target_error);
                        diagnostics.contacts = diagnostics.contacts.max(r.contacts);
                    }
                    Err(error) => {
                        self.stopped = Some(error.to_string());
                        break;
                    }
                }
            }
            self.physics_ms = begin.elapsed().as_secs_f64() * 1000.0;
            if self.advanced_substeps > 0 {
                self.report = diagnostics;
                self.inspection = None;
            }
        }
    }
    pub fn command(&mut self, command: Command, request_id: u32) -> Result<Frame, String> {
        self.advanced_substeps = 0;
        match command {
            Command::Step => {
                self.advance_with(FoldingWorld::tick);
            }
            Command::ReleaseGripper { gripper } => {
                let &body = self
                    .task
                    .grippers
                    .get(gripper)
                    .ok_or("Gripper must be 0 or 1")?;
                let handles: Vec<_> = self
                    .task
                    .world
                    .attachments()
                    .filter(|(_, a)| a.cloth == self.task.cloth && a.body == body)
                    .map(|(h, _)| h)
                    .chain(
                        self.task
                            .world
                            .surface_attachments()
                            .filter(|(_, a)| a.cloth == self.task.cloth && a.body == body)
                            .map(|(h, _)| h),
                    )
                    .collect();
                if handles.is_empty() {
                    return Err("This gripper is not holding cloth".into());
                }
                let before = self.task.world.checkpoint().map_err(|e| e.to_string())?;
                for handle in handles {
                    if let Err(error) = self.task.world.release(handle) {
                        self.task
                            .world
                            .restore(&before)
                            .map_err(|e| e.to_string())?;
                        return Err(error.to_string());
                    }
                }
                self.task.attachments[gripper] = None;
                self.manual_release = true;
            }
            Command::Inspect => {
                let cloth = self
                    .task
                    .world
                    .cloth(self.task.cloth)
                    .map_err(|e| e.to_string())?;
                let rest = cloth
                    .mesh()
                    .rest_positions()
                    .iter()
                    .copied()
                    .map(point)
                    .collect::<Vec<_>>();
                self.inspection = Some(Inspection {
                    step: self.task.step,
                    metrics: folding_oracle::fold_metrics(
                        &rest,
                        &self.task.positions(),
                        cloth.mesh().triangles(),
                        self.task.config.grid,
                        self.task.config.size,
                    ),
                });
            }
            Command::Reset { .. } => return Err("Reset is handled by the scene coordinator".into()),
            Command::SetOptions { .. }
            | Command::Release
            | Command::SetGripperPose { .. }
            | Command::GraspGripper { .. } => {
                return Err(
                    "Wind, sphere controls and pin release do not apply to this task".into(),
                );
            }
        }
        Ok(self.frame(request_id, false))
    }
    pub fn frame(&self, request_id: u32, topology: bool) -> Frame {
        let cloth = self
            .task
            .world
            .cloth(self.task.cloth)
            .expect("task owns its cloth");
        let mut grasps = vec![];
        let grippers = self
            .task
            .grippers
            .iter()
            .enumerate()
            .map(|(id, &body)| {
                let pose = self.task.rigid.bodies[body].position();
                let collider = &self.task.rigid.colliders[self.task.gripper_colliders[id]];
                let local = collider
                    .position_wrt_parent()
                    .expect("owned gripper collider");
                for (_, a) in self
                    .task
                    .world
                    .attachments()
                    .filter(|(_, a)| a.cloth == self.task.cloth && a.body == body)
                {
                    grasps.push(GraspFrame {
                        gripper: id,
                        points: a
                            .points
                            .iter()
                            .map(|p| GraspPoint {
                                material: MaterialPoint::Vertex {
                                    particle: p.particle,
                                },
                                position: cloth.positions()[p.particle as usize].to_array(),
                                anchor: pose.transform_point(p.local_anchor).to_array(),
                            })
                            .collect(),
                    });
                }
                for (_, a) in self
                    .task
                    .world
                    .surface_attachments()
                    .filter(|(_, a)| a.cloth == self.task.cloth && a.body == body)
                {
                    grasps.push(GraspFrame {
                        gripper: id,
                        points: a
                            .points
                            .iter()
                            .map(|p| GraspPoint {
                                material: MaterialPoint::Surface {
                                    triangle: p.point.triangle(),
                                    barycentric: p.point.barycentric(),
                                },
                                position: self
                                    .task
                                    .world
                                    .surface_point_position(ClothSurfacePoint {
                                        cloth: self.task.cloth,
                                        point: p.point,
                                    })
                                    .expect("validated attachment material point")
                                    .to_array(),
                                anchor: pose.transform_point(p.local_anchor).to_array(),
                            })
                            .collect(),
                    });
                }
                GripperFrame {
                    id,
                    translation: pose.translation.to_array(),
                    rotation: rotation(pose),
                    half_extents: collider
                        .shape()
                        .as_cuboid()
                        .expect("box gripper")
                        .half_extents
                        .to_array(),
                    local_translation: local.translation.to_array(),
                    local_rotation: rotation(local),
                    holding: grasps.iter().any(|g| g.gripper == id),
                }
            })
            .collect();
        let mode = cloth
            .contact_settings()
            .map(|s| CollisionMode {
                self_collision: s.self_collision,
                rigid_surface: s.rigid_surface_collision,
                continuous_self: s.continuous_self_collision,
                continuous_rigid: s.continuous_rigid_collision,
            })
            .unwrap_or_default();
        Frame {
            r#type: "frame",
            protocol: PROTOCOL,
            request_id,
            scene: SceneKind::FoldTowel,
            precision: if cfg!(feature = "f64") { "f64" } else { "f32" },
            step: self.task.step,
            time: self.task.step as f64 * self.task.config.h,
            h: self.task.rigid.integration_parameters.dt,
            substeps: SUBSTEPS,
            advanced_substeps: self.advanced_substeps,
            iterations: self.task.world.solver_settings.iterations,
            positions: cloth
                .positions()
                .iter()
                .flat_map(|p| p.to_array())
                .collect(),
            triangles: topology
                .then(|| cloth.mesh().triangles().iter().flatten().copied().collect()),
            pins: cloth.pins().keys().copied().collect(),
            sphere: [0.0; 3],
            sphere_radius: 0.0,
            options: Options {
                auto_motion: false,
                sphere_x: 0.0,
                sphere_z: 0.0,
                wind: 0.0,
            },
            physics_ms: self.physics_ms,
            p95_stretch: self.report.p95_stretch,
            max_penetration: self.report.max_penetration,
            max_target_error: self.report.max_target_error,
            contacts: self.report.contacts,
            implicit_available: cfg!(all(feature = "f64", feature = "implicit")),
            implicit: None,
            folding: Some(FoldingFrame {
                config: self.task.config.clone(),
                variant: 0,
                phase: format!("{:?}", phase(&self.task.config, self.task.step)),
                collision_mode: mode,
                grippers,
                grasps,
                manual_release: self.manual_release,
                stopped: self.stopped.clone(),
                inspection: self.inspection.clone(),
            }),
        }
    }
}
