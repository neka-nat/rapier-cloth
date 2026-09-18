//! Example-level host adapter: two Rapier end-effector poses and ideal vertex grasps.
//! End effectors have no colliders. This is not physical finger pinching or force feedback.
#![allow(dead_code)]
use super::implicit_fixture::Input;
use rapier_cloth::{Real, rapier::prelude::*, *};
use serde::Serialize;
use std::time::Instant;

pub struct RobotTowel {
    pub input: Input,
    pub variant: String,
    pub world: RapierClothWorld,
    pub rigid: PhysicsWorld,
    pub cloth: ClothHandle,
    pub grippers: [RigidBodyHandle; 2],
    pub attachments: [Option<AttachmentHandle>; 2],
    pub desired: [Pose; 2],
    pub step: usize,
    pub automatic: bool,
    pub stopped: Option<String>,
    pub physics_ms: f64,
    pub report: StepReport,
    pub samples: Vec<Sample>,
}
#[derive(Clone, Serialize)]
pub struct Sample {
    pub step: usize,
    pub time: Real,
    pub physics_ms: f64,
    pub rms_speed: Real,
    pub max_speed: Real,
    pub max_edge_extension: Real,
    pub held_vertices: usize,
    pub iterations: usize,
}
impl RobotTowel {
    pub fn new(variant: &str, execution: ImplicitExecution) -> Result<Self, String> {
        let input = Input::load(0.1, variant)?;
        let id = WorldId::new();
        let mut rigid = PhysicsWorld::new();
        rigid.integration_parameters.dt = input.h;
        rigid.gravity = Vec3::new(0.0, -9.81, 0.0);
        let mu = Input::friction(variant);
        rigid.colliders.insert(
            ColliderBuilder::new(SharedShape::halfspace(Vec3::Y))
                .translation(Vec3::Y * (input.thickness * 0.5))
                .friction(mu),
        );
        let mut world = RapierClothWorld::new(id);
        world.solver_settings.max_substep = input.h;
        let mesh = ClothMesh::new(
            input.x.iter().copied().map(Vec3::from_array).collect(),
            input.faces.clone(),
        )
        .map_err(|e| e.to_string())?;
        let mut cloth = Cloth::new(
            mesh,
            ClothMaterial {
                surface_density: 0.1503,
                damping: 0.0,
                friction: mu,
                contact_radius: input.thickness * 0.5,
                ..Default::default()
            },
        )
        .map_err(|e| e.to_string())?;
        cloth
            .set_contact_settings(Some(ClothContactSettings {
                thickness: input.thickness,
                activation_margin: input.thickness,
                static_friction: mu,
                kinetic_friction: mu,
                self_collision: true,
                continuous_self_collision: true,
                rigid_surface_collision: true,
                continuous_rigid_collision: true,
                limits: CollisionLimits {
                    candidate_pairs: 10_000_000,
                    ccd_checks: 10_000_000,
                    ..Default::default()
                },
            }))
            .map_err(|e| e.to_string())?;
        cloth
            .set_implicit_solver(Some(ImplicitSettings {
                execution,
                ..Default::default()
            }))
            .map_err(|e| e.to_string())?;
        let cloth = world.add_cloth(cloth);
        let desired = [0, 1].map(|side| {
            let i = *input
                .grasp
                .iter()
                .find(|&&i| input.side(i) == side)
                .unwrap() as usize;
            let p = input.x[i];
            Pose::translation(p[0], p[1], p[2])
        });
        let grippers = desired.map(|pose| {
            rigid
                .bodies
                .insert(RigidBodyBuilder::kinematic_position_based().pose(pose))
        });
        let mut task = Self {
            input,
            variant: variant.into(),
            world,
            rigid,
            cloth,
            grippers,
            attachments: [None; 2],
            desired,
            step: 0,
            automatic: true,
            stopped: None,
            physics_ms: 0.0,
            report: StepReport::default(),
            samples: vec![],
        };
        for side in 0..2 {
            task.attach_current(side)?;
        }
        Ok(task)
    }
    pub fn finished(&self) -> bool {
        self.step >= self.input.targets.len()
    }
    pub fn positions(&self) -> Vec<Vec3> {
        self.world.cloth(self.cloth).unwrap().positions().to_vec()
    }
    pub fn held_particles(&self) -> Vec<u32> {
        self.world
            .attachments()
            .filter(|(_, a)| a.cloth == self.cloth)
            .flat_map(|(_, a)| a.points.iter().map(|p| p.particle))
            .collect()
    }
    pub fn pose(&self, side: usize) -> Pose {
        *self.rigid.bodies[self.grippers[side]].position()
    }
    fn check_side(&self, side: usize) -> Result<(), String> {
        if side >= 2 {
            return Err("Gripper must be 0 or 1".into());
        }
        if self.stopped.is_some() || self.finished() {
            return Err("Reset the stopped or completed task before commanding it".into());
        }
        Ok(())
    }
    fn manual(&mut self) {
        if self.automatic {
            self.desired = [self.pose(0), self.pose(1)];
        }
        self.automatic = false;
    }
    pub fn set_pose(
        &mut self,
        side: usize,
        translation: [Real; 3],
        rotation: [Real; 4],
        at_step: usize,
    ) -> Result<(), String> {
        self.check_side(side)?;
        if at_step != self.step + 1 {
            return Err("Pose command must target the next accepted step".into());
        }
        let p = Vec3::from_array(translation);
        let q = rapier::math::Rotation::from_array(rotation);
        if !p.is_finite()
            || p.x.abs() > 0.75
            || p.z.abs() > 0.75
            || !(0.0..=0.75).contains(&p.y)
            || !q.is_finite()
            || (q.length_squared() - 1.0).abs() > 1e-6
        {
            return Err(
                "Invalid world pose: metres, Y-up, unit quaternion [x,y,z,w], workspace ±0.75 m"
                    .into(),
            );
        }
        let current = self.pose(side);
        if p.distance(current.translation) > 0.05 + 1e-9
            || current.rotation.dot(q).abs() < (std::f64::consts::PI as Real / 16.0).cos()
        {
            return Err("One command may move at most 5 cm and rotate at most 22.5 degrees per accepted step".into());
        }
        self.manual();
        let mut pose = Pose::translation(p.x, p.y, p.z);
        pose.rotation = q.normalize();
        self.desired[side] = pose;
        Ok(())
    }
    fn attach_current(&mut self, side: usize) -> Result<(), String> {
        if self.attachments[side].is_some() {
            return Err("This gripper already holds cloth".into());
        }
        let inverse = self.pose(side).inverse();
        let cloth = self.world.cloth(self.cloth).map_err(|e| e.to_string())?;
        let particles: Vec<_> = self
            .input
            .grasp
            .iter()
            .copied()
            .filter(|&i| self.input.side(i) == side)
            .collect();
        if cloth.positions()[particles[0] as usize].distance(self.pose(side).translation) > 0.02 {
            return Err(
                "Move the end-effector origin within 2 cm of its material patch before grasping"
                    .into(),
            );
        }
        let points = particles
            .into_iter()
            .map(|particle| AttachmentPoint {
                particle,
                local_anchor: inverse.transform_point(cloth.positions()[particle as usize]),
            })
            .collect();
        let handle = self
            .world
            .attach(
                AttachmentDesc {
                    cloth: self.cloth,
                    body: self.grippers[side],
                    points,
                    compliance: 0.0,
                    excluded_colliders: vec![],
                },
                &self.rigid.bodies,
                &self.rigid.colliders,
            )
            .map_err(|e| e.to_string())?;
        self.attachments[side] = Some(handle);
        Ok(())
    }
    pub fn grasp(&mut self, side: usize) -> Result<(), String> {
        self.check_side(side)?;
        self.attach_current(side)?;
        self.manual();
        Ok(())
    }
    fn release_inner(&mut self, side: usize) -> Result<(), String> {
        let handle = self.attachments[side].ok_or("This gripper is not holding cloth")?;
        self.world.release(handle).map_err(|e| e.to_string())?;
        self.attachments[side] = None;
        Ok(())
    }
    pub fn release(&mut self, side: usize) -> Result<(), String> {
        self.check_side(side)?;
        self.release_inner(side)?;
        self.manual();
        Ok(())
    }
    /// Recover the fixture's rigid patch pose. The original commands rotate about X.
    pub fn scripted_pose(&self, step: usize, side: usize) -> Pose {
        let first = self
            .input
            .grasp
            .iter()
            .position(|&i| self.input.side(i) == side)
            .unwrap();
        let particle = self.input.grasp[first];
        let second = self
            .input
            .grasp
            .iter()
            .position(|&i| i == particle + 32)
            .unwrap();
        let targets = self.input.targets[step]
            .as_ref()
            .or_else(|| self.input.targets.iter().rev().find_map(Option::as_ref))
            .unwrap();
        let a = targets[first];
        let b = targets[second];
        let dz = self.input.x[(particle + 32) as usize][2] - self.input.x[particle as usize][2];
        let angle = (-(b[1] - a[1]) / dz).atan2((b[2] - a[2]) / dz);
        let mut pose = Pose::translation(a[0], a[1], a[2]);
        pose.rotation = rapier::math::Rotation::from_rotation_x(angle);
        pose
    }
    /// One physical step. Rejected transitions leave both worlds and accepted time unchanged.
    pub fn tick(&mut self) -> Result<(), String> {
        if let Some(error) = &self.stopped {
            return Err(error.clone());
        }
        if self.finished() {
            return Err("Task completed".into());
        }
        let start = Instant::now();
        let before = self.world.checkpoint().map_err(|e| e.to_string())?;
        let rigid_before = RigidCheckpoint::capture(&self.rigid);
        let attachments_before = self.attachments;
        let mut desired = self.desired;
        let result = (|| -> Result<WorldStepReport, String> {
            if self.automatic {
                for (side, pose) in desired.iter_mut().enumerate() {
                    *pose = self.scripted_pose(self.step, side);
                    let selected = self
                        .input
                        .grasp
                        .iter()
                        .position(|&i| self.input.side(i) == side)
                        .unwrap();
                    if self
                        .input
                        .target(self.step, selected, &self.variant)
                        .is_none()
                        && self.attachments[side].is_some()
                    {
                        self.release_inner(side)?;
                    }
                }
            }
            for (body, pose) in self.grippers.iter().zip(desired) {
                self.rigid.bodies[*body].set_next_kinematic_position(pose);
            }
            let previous = SceneSnapshot::capture(
                self.world.world_id(),
                self.step as u64,
                &self.rigid.bodies,
                &self.rigid.colliders,
            );
            self.rigid.step();
            let query = self.rigid.broad_phase.as_query_pipeline(
                self.rigid.narrow_phase.query_dispatcher(),
                &self.rigid.bodies,
                &self.rigid.colliders,
                QueryFilter::default(),
            );
            self.world
                .step_substep(
                    self.input.h,
                    &RapierScene::new(query, &previous, self.input.h, self.rigid.gravity),
                )
                .map_err(|e| e.to_string())
        })();
        self.physics_ms = start.elapsed().as_secs_f64() * 1000.0;
        match result {
            Err(error) => {
                self.world.restore(&before).map_err(|e| e.to_string())?;
                rigid_before.restore(&mut self.rigid);
                self.attachments = attachments_before;
                self.stopped = Some(error.clone());
                Err(error)
            }
            Ok(mut report) => {
                self.report = report.cloths.remove(0).1;
                self.desired = desired;
                self.step += 1;
                self.samples.push(self.sample());
                Ok(())
            }
        }
    }
    pub fn sample(&self) -> Sample {
        let cloth = self.world.cloth(self.cloth).unwrap();
        let rms = (cloth
            .velocities()
            .iter()
            .map(|v| v.length_squared())
            .sum::<Real>()
            / cloth.velocities().len() as Real)
            .sqrt();
        Sample {
            step: self.step,
            time: self.step as Real * self.input.h,
            physics_ms: self.physics_ms,
            rms_speed: rms,
            max_speed: cloth
                .velocities()
                .iter()
                .map(|v| v.length())
                .fold(0.0, Real::max),
            max_edge_extension: (self.report.max_stretch - 1.0).max(0.0),
            held_vertices: self.held_particles().len(),
            iterations: self.report.iterations,
        }
    }
}

struct RigidCheckpoint {
    gravity: Vec3,
    integration_parameters: IntegrationParameters,
    islands: IslandManager,
    broad_phase: BroadPhaseBvh,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
}
impl RigidCheckpoint {
    fn capture(world: &PhysicsWorld) -> Self {
        Self {
            gravity: world.gravity,
            integration_parameters: world.integration_parameters,
            islands: world.islands.clone(),
            broad_phase: world.broad_phase.clone(),
            narrow_phase: world.narrow_phase.clone(),
            bodies: world.bodies.clone(),
            colliders: world.colliders.clone(),
            impulse_joints: world.impulse_joints.clone(),
            multibody_joints: world.multibody_joints.clone(),
            ccd_solver: world.ccd_solver.clone(),
        }
    }
    fn restore(self, world: &mut PhysicsWorld) {
        *world = PhysicsWorld {
            gravity: self.gravity,
            integration_parameters: self.integration_parameters,
            physics_pipeline: PhysicsPipeline::new(),
            islands: self.islands,
            broad_phase: self.broad_phase,
            narrow_phase: self.narrow_phase,
            bodies: self.bodies,
            colliders: self.colliders,
            impulse_joints: self.impulse_joints,
            multibody_joints: self.multibody_joints,
            ccd_solver: self.ccd_solver,
        };
    }
}
