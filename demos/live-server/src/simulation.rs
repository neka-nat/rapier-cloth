use rapier::prelude::*;
use rapier_cloth::{Real, *};
use serde::{Deserialize, Serialize};
use std::time::Instant;

pub const H: Real = 1.0 / 240.0;
pub const SUBSTEPS: usize = 4;
const SPHERE_RADIUS: Real = 0.3;

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum SceneKind {
    Drape,
    Hanging,
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
            return Err("操作値が範囲外です。球は±0.45m、風は0〜1で指定してください。".into());
        }
        Ok(self)
    }
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum Command {
    Step,
    Reset { scene: SceneKind },
    SetOptions { options: Options },
    Release,
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
}

pub struct Demo {
    rigid: PhysicsWorld,
    world: RapierClothWorld,
    cloth: ClothHandle,
    sphere: RigidBodyHandle,
    scene: SceneKind,
    options: Options,
    physics_ms: f64,
    report: StepReport,
}
impl Demo {
    pub fn new(scene: SceneKind) -> Result<Self, String> {
        let id = WorldId::new();
        let mut rigid = PhysicsWorld::new();
        rigid.integration_parameters.dt = H;
        rigid
            .colliders
            .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)));
        // A hemisphere above the floor avoids trapping cloth in a closing,
        // sub-radius gap between two opposing one-way contact surfaces.
        let sphere = rigid
            .bodies
            .insert(RigidBodyBuilder::kinematic_position_based().translation(Vec3::ZERO));
        rigid.colliders.insert_with_parent(
            ColliderBuilder::ball(SPHERE_RADIUS),
            sphere,
            &mut rigid.bodies,
        );
        let height = if scene == SceneKind::Hanging {
            1.15
        } else {
            SPHERE_RADIUS + 0.01
        };
        let mesh = GridBuilder::new(32, 32)
            .origin(Vec3::new(-0.5, height, -0.5))
            .build()
            .map_err(|e| e.to_string())?;
        // The default material strongly resists folding on this 32x32 grid.
        // Increase only bending compliance: retain inextensible edges, mass,
        // damping and the same solver budget while allowing fabric-like folds.
        let material = ClothMaterial {
            bend_compliance: 1.0e3,
            ..ClothMaterial::default()
        };
        let mut cloth = Cloth::new(mesh, material).map_err(|e| e.to_string())?;
        if scene == SceneKind::Hanging {
            for i in 0..32 {
                cloth
                    .pin(i, cloth.positions()[i as usize])
                    .map_err(|e| e.to_string())?;
            }
        }
        let mut world = RapierClothWorld::new(id);
        let cloth = world.add_cloth(cloth);
        Ok(Self {
            rigid,
            world,
            cloth,
            sphere,
            scene,
            options: Options {
                auto_motion: true,
                sphere_x: 0.0,
                sphere_z: 0.0,
                wind: if scene == SceneKind::Hanging {
                    0.5
                } else {
                    0.0
                },
            },
            physics_ms: 0.0,
            report: StepReport::default(),
        })
    }
    pub fn command(&mut self, command: Command, request_id: u32) -> Result<Frame, String> {
        if self.world.is_desynchronized() && !matches!(command, Command::Reset { .. }) {
            return Err("計算が停止しています。リセットして再開してください。".into());
        }
        let topology = matches!(command, Command::Reset { .. });
        match command {
            Command::Reset { scene } => *self = Self::new(scene)?,
            Command::SetOptions { options } => self.options = options.validate()?,
            Command::Release => {
                let cloth = self
                    .world
                    .cloth_mut(self.cloth)
                    .map_err(|e| e.to_string())?;
                let pins: Vec<_> = cloth.pins().keys().copied().collect();
                for pin in pins {
                    cloth.unpin(pin).map_err(|e| e.to_string())?;
                }
            }
            Command::Step => {
                let start = Instant::now();
                let mut penetration: Real = 0.0;
                let mut stretch: Real = 0.0;
                let mut contacts = 0;
                for _ in 0..SUBSTEPS {
                    self.substep()?;
                    penetration = penetration.max(self.report.max_penetration);
                    stretch = stretch.max(self.report.p95_stretch);
                    contacts = contacts.max(self.report.contacts);
                }
                self.report.max_penetration = penetration;
                self.report.p95_stretch = stretch;
                self.report.contacts = contacts;
                self.physics_ms = start.elapsed().as_secs_f64() * 1000.0;
            }
        }
        Ok(self.frame(request_id, topology))
    }
    fn substep(&mut self) -> Result<(), String> {
        let step = self.world.next_step_index();
        let t = (step + 1) as Real * H;
        let before = SceneSnapshot::capture(
            self.world.world_id(),
            step,
            &self.rigid.bodies,
            &self.rigid.colliders,
        );
        let target = if self.options.auto_motion {
            Vec3::new(0.1 * (0.9 * t).sin(), 0.0, 0.07 * (0.6 * t).sin())
        } else {
            Vec3::new(self.options.sphere_x, 0.0, self.options.sphere_z)
        };
        let current = self.rigid.bodies[self.sphere].translation();
        // UI targets may jump. Limit actual surface motion to 0.25m/s, below
        // the cloth bridge's per-substep kinematic motion budget.
        let next = current + (target - current).clamp_length_max(0.25 * H);
        self.rigid.bodies[self.sphere].set_next_kinematic_translation(next);
        let acceleration =
            Vec3::new(2.0 * (1.8 * t).sin(), 0.0, 4.0 + 2.0 * (0.8 * t).sin()) * self.options.wind;
        let cloth = self
            .world
            .cloth_mut(self.cloth)
            .map_err(|e| e.to_string())?;
        for i in 0..cloth.positions().len() {
            // A smooth travelling gust varies over the sheet instead of
            // accelerating every vertex identically. Rest coordinates keep
            // the pattern continuous as the cloth folds; wind=0 clears it.
            let rest = cloth.mesh().rest_positions()[i];
            let gust = (3.0 * t - 7.0 * rest.x + 4.0 * rest.z).sin() * self.options.wind;
            let local_acceleration = acceleration + Vec3::new(0.4, 1.2, 2.0) * gust;
            cloth
                .set_force(i as u32, local_acceleration * cloth.masses()[i])
                .map_err(|e| e.to_string())?;
        }
        self.rigid.step();
        let query = self.rigid.broad_phase.as_query_pipeline(
            self.rigid.narrow_phase.query_dispatcher(),
            &self.rigid.bodies,
            &self.rigid.colliders,
            QueryFilter::default(),
        );
        let report = self
            .world
            .step_substep(H, &RapierScene::new(query, &before, H, self.rigid.gravity))
            .map_err(|e| e.to_string())?;
        self.report = report.cloths[0].1.clone();
        Ok(())
    }
    pub fn frame(&self, request_id: u32, topology: bool) -> Frame {
        let cloth = self
            .world
            .cloth(self.cloth)
            .expect("demo owns a live cloth");
        Frame {
            r#type: "frame",
            protocol: 1,
            request_id,
            scene: self.scene,
            precision: if cfg!(feature = "f64") { "f64" } else { "f32" },
            step: self.world.next_step_index(),
            time: self.world.next_step_index() as f64 / 240.0,
            h: H,
            substeps: SUBSTEPS,
            iterations: self.world.solver_settings.iterations,
            positions: cloth
                .positions()
                .iter()
                .flat_map(|p| p.to_array())
                .collect(),
            triangles: topology
                .then(|| cloth.mesh().triangles().iter().flatten().copied().collect()),
            pins: cloth.pins().keys().copied().collect(),
            sphere: self.rigid.bodies[self.sphere].translation().to_array(),
            sphere_radius: SPHERE_RADIUS,
            options: self.options,
            physics_ms: self.physics_ms,
            p95_stretch: self.report.p95_stretch,
            max_penetration: self.report.max_penetration,
            contacts: self.report.contacts,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn failed_physics_requires_reset_before_any_further_frame() {
        let mut demo = Demo::new(SceneKind::Drape).unwrap();
        demo.world
            .cloth_mut(demo.cloth)
            .unwrap()
            .pin(0, Vec3::new(0.0, -0.1, 0.0))
            .unwrap();
        assert!(demo.command(Command::Step, 1).is_err());
        assert!(demo.world.is_desynchronized());
        assert!(
            matches!(demo.command(Command::Release, 2), Err(message) if message.contains("リセット"))
        );
        assert!(
            demo.command(
                Command::SetOptions {
                    options: demo.options
                },
                3
            )
            .is_err()
        );
        let reset = demo
            .command(
                Command::Reset {
                    scene: SceneKind::Drape,
                },
                4,
            )
            .unwrap();
        assert_eq!(reset.step, 0);
        demo.command(Command::Step, 5).unwrap();
    }
    #[test]
    fn real_solver_advances_deforms_and_resets_exactly() {
        let mut demo = Demo::new(SceneKind::Drape).unwrap();
        let initial = demo.frame(0, true);
        assert_eq!(initial.positions.len(), 3072);
        assert_eq!(initial.triangles.as_ref().unwrap().len(), 31 * 31 * 6);
        let mut draped = false;
        for seq in 1..=1200 {
            let frame = demo.command(Command::Step, seq).unwrap();
            assert_eq!(frame.step, seq as u64 * 4);
            assert!(frame.positions.iter().all(|p| p.is_finite()));
            assert!(
                frame.max_penetration < 0.001,
                "frame {seq}: penetration {}",
                frame.max_penetration
            );
            draped |= frame.positions.chunks_exact(3).any(|p| p[1] < 0.1)
                && frame
                    .positions
                    .chunks_exact(3)
                    .any(|p| p[1] > SPHERE_RADIUS * 0.8);
        }
        assert!(draped, "cloth must actually drape over the sphere");
        let reset = demo
            .command(
                Command::Reset {
                    scene: SceneKind::Drape,
                },
                1201,
            )
            .unwrap();
        assert_eq!(reset.positions, initial.positions);
        assert_eq!(reset.step, 0);
    }
    #[test]
    fn targets_are_bounded_controls_do_not_step_and_release_keeps_state() {
        let mut demo = Demo::new(SceneKind::Hanging).unwrap();
        let before = demo.frame(0, true);
        assert_eq!(before.pins.len(), 32);
        let invalid = Options {
            auto_motion: false,
            sphere_x: Real::NAN,
            sphere_z: 0.0,
            wind: 0.0,
        };
        assert!(
            demo.command(Command::SetOptions { options: invalid }, 1)
                .is_err()
        );
        assert_eq!(demo.frame(1, false).positions, before.positions);
        let options = Options {
            auto_motion: false,
            sphere_x: 0.45,
            sphere_z: -0.45,
            wind: 1.0,
        };
        let set = demo.command(Command::SetOptions { options }, 2).unwrap();
        assert_eq!(set.step, 0);
        let moved = demo.command(Command::Step, 3).unwrap();
        assert!(
            Vec3::from_array(moved.sphere).distance(Vec3::from_array(before.sphere))
                <= 0.25 / 60.0 + 1.0e-6
        );
        let released = demo.command(Command::Release, 4).unwrap();
        assert!(released.pins.is_empty());
        assert_eq!(released.positions, moved.positions);
        assert_eq!(released.step, moved.step);
        for seq in 5..65 {
            demo.command(Command::Step, seq).unwrap();
        }
        assert!(demo.frame(65, false).positions[1] < before.positions[1]);
    }
}
