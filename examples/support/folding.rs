//! Frozen dual-gripper task shared by benchmarks, examples and tests.
#![allow(dead_code)]
use rapier::prelude::*;
use rapier_cloth::{Real, *};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Variant {
    pub name: String,
    pub approach_x: f64,
    pub approach_z: f64,
    pub speed: f64,
}
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Acceptance {
    pub max_separation_deficit: f64,
    pub max_table_penetration: f64,
    pub max_strain: f64,
    pub p95_strain: f64,
    pub max_target_error: f64,
    pub max_corner_error: f64,
    pub min_overlap: f64,
    pub relative_area_error: f64,
    pub max_settle_drift: f64,
    pub physics_p95_ms: f64,
    pub physics_max_ms: f64,
}
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub version: u32,
    pub grid: usize,
    pub size: f64,
    pub h: f64,
    pub iterations: usize,
    pub thickness: f64,
    pub activation_margin: f64,
    pub surface_density: f64,
    pub stretch_compliance: f64,
    pub bend_compliance: f64,
    pub damping: f64,
    pub static_friction: f64,
    pub kinetic_friction: f64,
    pub legacy_contact_radius: f64,
    pub settle_end: u64,
    pub attach_step: u64,
    pub lift_end: u64,
    pub fold_end: u64,
    pub lower_end: u64,
    pub release_step: u64,
    pub retract_end: u64,
    pub end_step: u64,
    pub variants: Vec<Variant>,
    pub acceptance: Acceptance,
}
impl Default for Config {
    fn default() -> Self {
        serde_json::from_str(include_str!("fold_fixture.json")).expect("checked folding fixture")
    }
}
impl Config {
    /// Version 1 preserves the original pre-motion attachment baseline. Version
    /// 2 captures anchors after the final approach movement, before cloth solve.
    /// Every physical parameter, trajectory sample and acceptance limit agrees.
    pub fn with_version(version: u32) -> Result<Self, Box<dyn std::error::Error>> {
        let text = match version {
            1 => include_str!("fold_fixture.json"),
            2 => include_str!("fold_fixture_v2.json"),
            _ => return Err("unknown folding fixture version".into()),
        };
        let config: Self = serde_json::from_str(text)?;
        config.validate()?;
        Ok(config)
    }
    pub fn validate(&self) -> Result<(), Box<dyn std::error::Error>> {
        if self.grid < 4
            || self.grid > 1024
            || !(1..=2).contains(&self.version)
            || self.variants.is_empty()
        {
            return Err("invalid fixture layout".into());
        }
        let times = [
            0,
            self.settle_end,
            self.attach_step,
            self.lift_end,
            self.fold_end,
            self.lower_end,
            self.release_step,
            self.retract_end,
            self.end_step,
        ];
        if times.windows(2).any(|w| w[0] >= w[1]) || times.iter().any(|t| t % 4 != 0) {
            return Err("fixture phases must increase on four-step boundaries".into());
        }
        if [
            self.size,
            self.h,
            self.thickness,
            self.legacy_contact_radius,
            self.surface_density,
        ]
        .iter()
        .any(|x| !x.is_finite() || *x <= 0.0)
            || [
                self.activation_margin,
                self.stretch_compliance,
                self.bend_compliance,
                self.damping,
                self.static_friction,
                self.kinetic_friction,
            ]
            .iter()
            .any(|x| !x.is_finite() || *x < 0.0)
            || self.static_friction < self.kinetic_friction
        {
            return Err("invalid fixture physical parameters".into());
        }
        if self.variants.iter().any(|v| {
            !v.speed.is_finite()
                || !(0.5..=2.0).contains(&v.speed)
                || !v.approach_x.is_finite()
                || !v.approach_z.is_finite()
                || v.approach_x.abs() > 0.005
                || v.approach_z.abs() > 0.005
        }) {
            return Err("invalid bounded trajectory variant".into());
        }
        Ok(())
    }
    pub fn for_variant(&self, index: usize) -> Result<Self, Box<dyn std::error::Error>> {
        self.validate()?;
        let variant = self.variants.get(index).ok_or("unknown fixture variant")?;
        let mut config = self.clone();
        // Stretch the commanded-motion phases; preserve at least five seconds of
        // final settling and keep every phase on an application-frame boundary.
        let scaled = |step: u64| {
            self.attach_step
                + (((step - self.attach_step) as f64 / variant.speed / 4.0).ceil() as u64) * 4
        };
        config.lift_end = scaled(self.lift_end);
        config.fold_end = scaled(self.fold_end);
        config.lower_end = scaled(self.lower_end);
        config.release_step = scaled(self.release_step);
        config.retract_end = scaled(self.retract_end);
        config.end_step = config.retract_end + (self.end_step - self.retract_end);
        config.validate()?;
        Ok(config)
    }
}
#[cfg(feature = "f64")]
pub fn real(x: f64) -> Real {
    x
}
#[cfg(not(feature = "f64"))]
pub fn real(x: f64) -> Real {
    x as Real
}
#[cfg(feature = "f64")]
pub fn point(x: Vec3) -> [f64; 3] {
    x.to_array()
}
#[cfg(not(feature = "f64"))]
pub fn point(x: Vec3) -> [f64; 3] {
    x.to_array().map(f64::from)
}

#[derive(Clone, Copy, Debug, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Phase {
    Settle,
    Approach,
    Lift,
    Fold,
    Lower,
    Hold,
    Retract,
    Released,
    Finished,
}
pub fn phase(config: &Config, step: u64) -> Phase {
    if step < config.settle_end {
        Phase::Settle
    } else if step < config.attach_step {
        Phase::Approach
    } else if step < config.lift_end {
        Phase::Lift
    } else if step < config.fold_end {
        Phase::Fold
    } else if step < config.lower_end {
        Phase::Lower
    } else if step < config.release_step {
        Phase::Hold
    } else if step < config.retract_end {
        Phase::Retract
    } else if step < config.end_step {
        Phase::Released
    } else {
        Phase::Finished
    }
}
fn smooth(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}
fn between(step: u64, start: u64, end: u64) -> f64 {
    smooth((step as f64 - start as f64) / (end - start) as f64)
}
pub fn gripper_target(
    config: &Config,
    variant: &Variant,
    gripper: usize,
    step: u64,
    base: f64,
) -> Vec3 {
    let half = config.size * 0.5;
    let x = if gripper == 0 { -half } else { half };
    let pi = std::f64::consts::PI;
    let (y, z) = if step < config.attach_step {
        (
            base + 0.05 * (1.0 - between(step, config.settle_end, config.attach_step)),
            -half,
        )
    } else if step < config.release_step {
        let angle = if step < config.lift_end {
            pi * 0.5 * between(step, config.attach_step, config.lift_end)
        } else if step < config.fold_end {
            pi * (0.5 + 0.42 * between(step, config.lift_end, config.fold_end))
        } else {
            pi * (0.92 + 0.08 * between(step, config.fold_end, config.lower_end))
        };
        (
            base + half * angle.sin() + config.thickness * (angle / pi),
            -half * angle.cos(),
        )
    } else {
        (
            base + config.thickness + 0.1 * between(step, config.release_step, config.retract_end),
            half,
        )
    };
    Vec3::new(
        real(x + variant.approach_x),
        real(y),
        real(z + variant.approach_z),
    )
}

pub struct FoldingWorld {
    pub config: Config,
    pub variant: Variant,
    pub id: WorldId,
    pub rigid: PhysicsWorld,
    pub world: RapierClothWorld,
    pub cloth: ClothHandle,
    pub grippers: [RigidBodyHandle; 2],
    pub gripper_colliders: [ColliderHandle; 2],
    pub attachments: [Option<AttachmentHandle>; 2],
    pub step: u64,
    pub base_height: f64,
}

/// Physical Rapier state for one task transaction. Pipeline buffers are scratch
/// and are recreated only on rollback; collider clones retain shared shapes.
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
impl FoldingWorld {
    /// F01 baseline intentionally retains the library's particle collision mode.
    pub fn new(config: Config, variant_index: usize) -> Result<Self, Box<dyn std::error::Error>> {
        let variant = config
            .variants
            .get(variant_index)
            .ok_or("unknown variant")?
            .clone();
        let config = config.for_variant(variant_index)?;
        let base_height = config.legacy_contact_radius;
        let id = WorldId::new();
        let mut rigid = PhysicsWorld::new();
        rigid.integration_parameters.dt = real(config.h);
        rigid.colliders.insert(
            ColliderBuilder::new(SharedShape::halfspace(Vec3::Y))
                .friction(real(config.kinetic_friction)),
        );
        let grippers = std::array::from_fn(|i| {
            rigid.bodies.insert(
                RigidBodyBuilder::kinematic_position_based().translation(gripper_target(
                    &config,
                    &variant,
                    i,
                    0,
                    base_height,
                )),
            )
        });
        let gripper_colliders = grippers.map(|body| {
            rigid.colliders.insert_with_parent(
                ColliderBuilder::cuboid(real(0.012), real(0.008), real(0.012))
                    .translation(Vec3::new(0.0, real(0.024), 0.0)),
                body,
                &mut rigid.bodies,
            )
        });
        let mesh = GridBuilder::new(config.grid, config.grid)
            .size(real(config.size), real(config.size))
            .origin(Vec3::new(
                real(-config.size * 0.5),
                real(base_height),
                real(-config.size * 0.5),
            ))
            .build()?;
        let mut world = RapierClothWorld::new(id);
        world.solver_settings.iterations = config.iterations;
        let cloth = world.add_cloth(Cloth::new(
            mesh,
            ClothMaterial {
                surface_density: real(config.surface_density),
                stretch_compliance: real(config.stretch_compliance),
                bend_compliance: real(config.bend_compliance),
                damping: real(config.damping),
                friction: real(config.kinetic_friction),
                contact_radius: real(config.legacy_contact_radius),
            },
        )?);
        Ok(Self {
            config,
            variant,
            id,
            rigid,
            world,
            cloth,
            grippers,
            gripper_colliders,
            attachments: [None; 2],
            step: 0,
            base_height,
        })
    }
    pub fn corner_patch(&self, gripper: usize) -> [u32; 4] {
        let n = self.config.grid as u32;
        if gripper == 0 {
            [0, 1, n, n + 1]
        } else {
            [n - 2, n - 1, 2 * n - 2, 2 * n - 1]
        }
    }
    /// Commit one physical substep and its task events together. On error, the
    /// caller may inspect the last accepted state, repair the cause, and retry.
    /// Checkpoint cost belongs to the task's timed physics scope.
    pub fn tick(&mut self) -> Result<WorldStepReport, IntegrationError> {
        let cloth = self.world.checkpoint()?;
        let rigid = RigidCheckpoint::capture(&self.rigid);
        let attachments = self.attachments;
        let step = self.step;
        match self.advance() {
            Ok(report) => Ok(report),
            Err(error) => {
                rigid.restore(&mut self.rigid);
                self.attachments = attachments;
                self.step = step;
                self.world.restore(&cloth)?;
                Err(error)
            }
        }
    }
    fn advance(&mut self) -> Result<WorldStepReport, IntegrationError> {
        let next = self.step + 1;
        if next == self.config.attach_step && self.config.version == 1 {
            self.attach_grippers()?;
        }
        if next == self.config.release_step {
            for attachment in &mut self.attachments {
                if let Some(handle) = attachment.take() {
                    self.world.release(handle)?;
                }
            }
        }
        let before = SceneSnapshot::capture(
            self.id,
            self.step,
            &self.rigid.bodies,
            &self.rigid.colliders,
        );
        for i in 0..2 {
            self.rigid.bodies[self.grippers[i]].set_next_kinematic_translation(gripper_target(
                &self.config,
                &self.variant,
                i,
                next,
                self.base_height,
            ));
        }
        self.rigid.step();
        // The grasp starts at the completed approach pose. Capturing the old
        // pose first would include the last downward approach increment in the
        // newly attached targets, commanding a settled patch into the table.
        if next == self.config.attach_step && self.config.version == 2 {
            self.attach_grippers()?;
        }
        let query = self.rigid.broad_phase.as_query_pipeline(
            self.rigid.narrow_phase.query_dispatcher(),
            &self.rigid.bodies,
            &self.rigid.colliders,
            QueryFilter::default(),
        );
        let report = self.world.step_substep(
            real(self.config.h),
            &RapierScene::new(query, &before, real(self.config.h), self.rigid.gravity),
        )?;
        self.step = next;
        Ok(report)
    }
    fn attach_grippers(&mut self) -> Result<(), IntegrationError> {
        for g in 0..2 {
            let inverse = self.rigid.bodies[self.grippers[g]].position().inverse();
            let points = self
                .corner_patch(g)
                .map(|particle| AttachmentPoint {
                    particle,
                    local_anchor: inverse.transform_point(
                        self.world.cloth(self.cloth).unwrap().positions()[particle as usize],
                    ),
                })
                .to_vec();
            self.attachments[g] = Some(self.world.attach(
                AttachmentDesc {
                    cloth: self.cloth,
                    body: self.grippers[g],
                    points,
                    compliance: 0.0,
                    excluded_colliders: vec![self.gripper_colliders[g]],
                },
                &self.rigid.bodies,
                &self.rigid.colliders,
            )?);
        }
        Ok(())
    }
    pub fn positions(&self) -> Vec<[f64; 3]> {
        self.world
            .cloth(self.cloth)
            .unwrap()
            .positions()
            .iter()
            .copied()
            .map(point)
            .collect()
    }
    pub fn center_of_mass(&self) -> [f64; 3] {
        let cloth = self.world.cloth(self.cloth).unwrap();
        let mass: Real = cloth.masses().iter().sum();
        point(
            cloth
                .positions()
                .iter()
                .zip(cloth.masses())
                .map(|(p, m)| *p * *m)
                .sum::<Vec3>()
                / mass,
        )
    }
}
