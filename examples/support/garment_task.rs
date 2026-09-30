//! T-shirt folding on the implicit solver with two ideal grippers.
//!
//! The garment lies flat on a table. Each gripper pinches a cuff, turns the
//! sleeve over the body about the body's side (a page turn: the patch travels a
//! semicircle and rotates with it), releases, travels to a hem corner, pinches
//! it, turns the lower half over the shoulder line and releases. Every step is
//! transactional: a rejected step restores both worlds and the attachments.
//! The run produces garment metrics and a schema-2 recording for the viewer.
use rapier_cloth::core::garment::{Landmark, Side, TShirtPattern};
use rapier_cloth::rapier::prelude::*;
use rapier_cloth::{Real, *};
use serde_json::{Value, json};
use std::time::Instant;

/// Scene and script parameters. Durations are in seconds and are rounded to
/// whole steps.
#[derive(Debug, Clone, Copy)]
pub struct ShirtTaskConfig {
    pub pattern: TShirtPattern,
    pub h: Real,
    pub friction: Real,
    pub thickness: Real,
    /// Barrier band (`activation_margin`). Layers start one thickness plus one
    /// band apart.
    pub band: Real,
    pub surface_density: Real,
    pub execution: ImplicitExecution,
    pub cap_policy: ImplicitCapPolicy,
    /// Newton iteration cap per step (`ImplicitSettings::max_iterations`).
    pub max_iterations: usize,
    /// Extra membrane stiffness along the garment's length (warp) and across
    /// it (weft), in pascals; zero keeps the isotropic shell.
    pub warp_stiffness: Real,
    pub weft_stiffness: Real,
    /// Spring stiffness of stitched seams (N/m).
    pub stitch_stiffness: Real,
    /// Radius of the pinched patch around a landmark; at least one and a half
    /// cells, so a coarse mesh still holds the landmark's neighbours.
    pub patch_radius: Real,
    /// Which layers a gripper closes on.
    pub pinch: Pinch,
    /// Compliance (m/N) of the hem grasps: 0 pins the patch rigidly, a
    /// positive value holds it with springs of stiffness 1/compliance, so the
    /// flap sagging between the two grippers extends the springs instead of
    /// stretching the hem.
    pub hem_compliance: Real,
    /// How far each hem gripper moves towards the other at mid-flight, so the
    /// sagging flap between them is not stretched across.
    pub hem_inset: Real,
    /// Height of the hem turn relative to a semicircle: 1 turns the flap like a
    /// page, less lets it drag over the body instead of hanging free.
    pub hem_arc_height: Real,
    /// Height above the plane at which a turned patch is released: the turn
    /// stops short of a half circle by that much, keeping the fabric length.
    pub release_height: Real,
    pub settle: Real,
    pub sleeve_fold: Real,
    /// Hold at the end of the sleeve turn before releasing, so the sleeve is
    /// at rest when it is let go.
    pub sleeve_dwell: Real,
    pub sleeve_settle: Real,
    pub hem_fold: Real,
    /// Hold at the end of the hem turn before releasing.
    pub hem_dwell: Real,
    /// Steps over which a compliant hem grasp softens tenfold per step before
    /// it is released, so the flap descends instead of dropping.
    pub hem_release_ramp: usize,
    pub final_settle: Real,
}

impl Default for ShirtTaskConfig {
    fn default() -> Self {
        Self {
            pattern: TShirtPattern::default(),
            h: 0.1,
            friction: 0.5,
            thickness: 0.000318,
            band: 0.001,
            surface_density: 0.1503,
            execution: ImplicitExecution::Serial,
            cap_policy: ImplicitCapPolicy::Strict,
            max_iterations: 128,
            warp_stiffness: 0.0,
            weft_stiffness: 0.0,
            stitch_stiffness: 500.0,
            patch_radius: 0.04,
            pinch: Pinch::AllLayers,
            hem_compliance: 0.02,
            hem_inset: 0.0,
            hem_arc_height: 1.0,
            release_height: 0.05,
            settle: 0.5,
            sleeve_fold: 3.0,
            sleeve_dwell: 0.0,
            sleeve_settle: 1.5,
            hem_fold: 12.0,
            hem_dwell: 0.0,
            hem_release_ramp: 0,
            final_settle: 2.0,
        }
    }
}

/// Which layers a gripper pinches.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pinch {
    /// The front panel and the seams only, as a gripper closing on the top
    /// layer of a garment lying front up.
    TopLayer,
    /// Every layer within the patch radius: a closed pinch through the cuff
    /// or hem.
    AllLayers,
}

/// Step boundaries of the script.
#[derive(Debug, Clone, Copy)]
pub struct Schedule {
    pub sleeve_grasp: usize,
    /// End of the sleeve turn; the gripper holds still until the release.
    pub sleeve_turn_end: usize,
    pub sleeve_release: usize,
    pub hem_grasp: usize,
    pub hem_turn_end: usize,
    pub hem_release: usize,
    pub end: usize,
}

impl Schedule {
    fn new(config: &ShirtTaskConfig) -> Self {
        let steps = |seconds: Real| (seconds / config.h).round().max(1.0) as usize;
        let dwell = |seconds: Real| (seconds / config.h).round().max(0.0) as usize;
        let sleeve_grasp = steps(config.settle);
        let sleeve_turn_end = sleeve_grasp + steps(config.sleeve_fold);
        let sleeve_release = sleeve_turn_end + dwell(config.sleeve_dwell);
        let hem_grasp = sleeve_release + steps(config.sleeve_settle);
        let hem_turn_end = hem_grasp + steps(config.hem_fold);
        let hem_release = hem_turn_end + dwell(config.hem_dwell);
        Self {
            sleeve_grasp,
            sleeve_turn_end,
            sleeve_release,
            hem_grasp,
            hem_turn_end,
            hem_release,
            end: hem_release + steps(config.final_settle),
        }
    }
}

/// One accepted step.
#[derive(Debug, Clone, Copy)]
pub struct StepSample {
    pub step: usize,
    pub time: Real,
    pub seconds: f64,
    pub iterations: usize,
    pub converged: bool,
    pub contacts: usize,
    pub max_edge_extension: Real,
    pub rms_speed: Real,
    pub max_speed: Real,
    pub held: usize,
}

/// Task metrics and gates.
#[derive(Debug, Clone)]
pub struct ShirtSummary {
    pub steps: usize,
    pub simulated_seconds: Real,
    pub simulation_wall_seconds: f64,
    pub max_edge_extension: Real,
    pub approximate_steps: usize,
    pub max_contacts: usize,
    pub settled: bool,
    pub final_window_drift: Real,
    /// Final over initial area of the bounding rectangle in the table plane.
    pub footprint_ratio: Real,
    /// Both cuff centres ended over the body's original footprint.
    pub cuffs_over_body: [bool; 2],
    /// Distance of each hem corner behind the shoulder line at the end
    /// (positive means short of it).
    pub hem_short_of_shoulders: [Real; 2],
    pub crossing_pairs: usize,
    pub max_separation_deficit: f64,
    pub min_height: Real,
    pub passed: bool,
}

impl ShirtSummary {
    pub fn to_json(&self) -> Value {
        json!({
            "steps": self.steps,
            "simulated_seconds": self.simulated_seconds,
            "simulation_wall_seconds": self.simulation_wall_seconds,
            "max_edge_extension": self.max_edge_extension,
            "approximate_steps": self.approximate_steps,
            "max_contacts": self.max_contacts,
            "settled": self.settled,
            "final_window_drift": self.final_window_drift,
            "footprint_ratio": self.footprint_ratio,
            "cuffs_over_body": self.cuffs_over_body,
            "hem_short_of_shoulders": self.hem_short_of_shoulders,
            "crossing_pairs": self.crossing_pairs,
            "max_separation_deficit": self.max_separation_deficit,
            "min_height": self.min_height,
            "passed": self.passed,
        })
    }
}

struct Frame {
    step: usize,
    positions: Vec<Vec3>,
    bodies: [Pose; 2],
    attached: Vec<u32>,
    anchors: Vec<Vec3>,
    diagnostics: Option<(Real, Real, Real)>,
    iterations: usize,
    seconds: f64,
}

pub struct ShirtTask {
    config: ShirtTaskConfig,
    schedule: Schedule,
    garment: Garment,
    id: WorldId,
    rigid: PhysicsWorld,
    world: RapierClothWorld,
    cloth: ClothHandle,
    grippers: [RigidBodyHandle; 2],
    attachments: [Option<AttachmentHandle>; 2],
    /// The pinched vertices of each gripper's current or next grasp.
    patches: [Vec<u32>; 2],
    step: usize,
    stopped: Option<String>,
    initial: Vec<Vec3>,
    samples: Vec<StepSample>,
    frames: Vec<Frame>,
    max_extension: Real,
    max_contacts: usize,
    approximate_steps: usize,
    /// Cloth midsurface height of the lowest layer on the table.
    rest_height: Real,
    /// Centre of each hem patch when it was pinched.
    hem_grasp_centre: [Option<Vec3>; 2],
}

fn smoothstep(s: Real) -> Real {
    let s = s.clamp(0.0, 1.0);
    s * s * (3.0 - 2.0 * s)
}

fn side_index(side: Side) -> usize {
    match side {
        Side::Left => 0,
        Side::Right => 1,
    }
}

impl ShirtTask {
    pub fn new(config: ShirtTaskConfig) -> Result<Self, String> {
        let garment = config.pattern.build().map_err(|e| e.to_string())?;
        let schedule = Schedule::new(&config);
        let id = WorldId::new();
        let mut rigid = PhysicsWorld::new();
        rigid.integration_parameters.dt = config.h;
        rigid.gravity = Vec3::new(0.0, -9.81, 0.0);
        let thickness = config.thickness;
        rigid.colliders.insert(
            ColliderBuilder::new(SharedShape::halfspace(Vec3::Y))
                .translation(Vec3::Y * (thickness * 0.5))
                .friction(config.friction),
        );
        let mut world = RapierClothWorld::new(id);
        world.solver_settings.max_substep = config.h;
        let mut cloth = Cloth::new(
            garment.mesh().clone(),
            ClothMaterial {
                surface_density: config.surface_density,
                damping: 0.0,
                friction: config.friction,
                contact_radius: thickness * 0.5,
                ..Default::default()
            },
        )
        .map_err(|e| e.to_string())?;
        // Lowest layer where the towel rests (half a thickness plus one band
        // above the table top); sewn layers one thickness plus one band apart.
        let rest_height = thickness + config.band;
        let gap = thickness + config.band;
        let origin = match config.pattern.layers {
            GarmentLayers::Single => Vec3::Y * rest_height,
            GarmentLayers::Sewn => Vec3::Y * (rest_height + 0.5 * gap),
        };
        let placed = garment
            .placed_positions(origin, gap)
            .map_err(|e| e.to_string())?;
        cloth.set_positions(&placed).map_err(|e| e.to_string())?;
        cloth
            .set_contact_settings(Some(ClothContactSettings {
                thickness,
                activation_margin: config.band,
                static_friction: config.friction,
                kinetic_friction: config.friction,
                self_collision: true,
                continuous_self_collision: true,
                rigid_surface_collision: true,
                continuous_rigid_collision: true,
                // Per-step budgets over every Newton iteration: a full-size
                // sewn shirt with four stacked layers exceeds the towel's
                // 10 M candidate pairs.
                limits: CollisionLimits {
                    candidate_pairs: 400_000_000,
                    ccd_checks: 400_000_000,
                    retained_contacts: 1 << 20,
                },
            }))
            .map_err(|e| e.to_string())?;
        world.solver_settings.max_contacts = 1 << 20;
        cloth
            .set_implicit_solver(Some(ImplicitSettings {
                execution: config.execution,
                cap_policy: config.cap_policy,
                max_iterations: config.max_iterations,
                material: ShellMaterial {
                    warp_stiffness: config.warp_stiffness,
                    weft_stiffness: config.weft_stiffness,
                    stitch_stiffness: config.stitch_stiffness,
                    ..Default::default()
                },
                ..Default::default()
            }))
            .map_err(|e| e.to_string())?;
        let cloth = world.add_cloth(cloth);
        let positions = world.cloth(cloth).map_err(|e| e.to_string())?.positions();
        let mut patches = [Vec::new(), Vec::new()];
        let mut grippers = [RigidBodyHandle::invalid(); 2];
        for side in [Side::Left, Side::Right] {
            let k = side_index(side);
            patches[k] = select_patch(&garment, &config, Landmark::CuffCenter(side));
            if patches[k].is_empty() {
                return Err("the pattern has no cuff patch".into());
            }
            let centre = patch_centre(positions, &patches[k]);
            grippers[k] = rigid.bodies.insert(
                RigidBodyBuilder::kinematic_position_based().pose(Pose::from_translation(centre)),
            );
        }
        let initial = positions.to_vec();
        let mut task = Self {
            config,
            schedule,
            garment,
            id,
            rigid,
            world,
            cloth,
            grippers,
            attachments: [None, None],
            patches,
            step: 0,
            stopped: None,
            initial,
            samples: Vec::new(),
            frames: Vec::new(),
            max_extension: 0.0,
            max_contacts: 0,
            approximate_steps: 0,
            rest_height,
            hem_grasp_centre: [None, None],
        };
        task.record_frame(None, 0, 0.0);
        Ok(task)
    }

    pub fn schedule(&self) -> Schedule {
        self.schedule
    }
    pub fn garment(&self) -> &Garment {
        &self.garment
    }
    pub fn step(&self) -> usize {
        self.step
    }
    pub fn finished(&self) -> bool {
        self.step >= self.schedule.end
    }
    pub fn samples(&self) -> &[StepSample] {
        &self.samples
    }
    pub fn cloth(&self) -> &Cloth {
        self.world.cloth(self.cloth).expect("task cloth")
    }

    /// Pattern-frame helpers: the body's side (the sleeve fold line) and the
    /// hem, shoulder and cuff coordinates of the placed garment.
    fn body_half_width(&self) -> Real {
        let cells = self.garment.cells();
        cells.body[0] as Real * self.config.pattern.spacing * 0.5
    }
    fn body_half_length(&self) -> Real {
        let cells = self.garment.cells();
        cells.body[1] as Real * self.config.pattern.spacing * 0.5
    }
    fn sleeve_length(&self) -> Real {
        self.garment.cells().sleeve[0] as Real * self.config.pattern.spacing
    }

    /// The gripper pose commanded for the end of step `step` (0-based): the
    /// grasped patch turns about the fold line like a page.
    pub fn scripted_pose(&self, step: usize, side: Side) -> Pose {
        let sign = match side {
            Side::Left => -1.0,
            Side::Right => 1.0,
        };
        let s = self.schedule;
        let next = step + 1;
        let cuff_rest = self.cuff_start(side);
        let hem_rest = self.hem_start(side);
        let ratio = |from: usize, to: usize| -> Real {
            if to == from {
                1.0
            } else {
                (next.saturating_sub(from) as Real / (to - from) as Real).min(1.0)
            }
        };
        if next <= s.sleeve_grasp {
            return Pose::from_translation(cuff_rest);
        }
        if next <= s.sleeve_release {
            // Turn about the body's side (axis z through sign * half width).
            // The turn stops short of the plane by the release height, so the
            // fabric between the fold line and the patch keeps its length;
            // after the turn the gripper holds still until the release.
            let pivot = Vec3::new(sign * self.body_half_width(), cuff_rest.y, cuff_rest.z);
            let theta = self.turn_angle(cuff_rest, pivot)
                * smoothstep(ratio(s.sleeve_grasp, s.sleeve_turn_end));
            // The patch rises first: a left sleeve (at -x) turns about -z, a
            // right sleeve about +z.
            let rotation = Rotation::from_axis_angle(Vec3::Z, sign * theta);
            let mut pose = Pose::from_translation(pivot + rotation * (cuff_rest - pivot));
            pose.rotation = rotation;
            return pose;
        }
        if next <= s.hem_grasp {
            // Travel from the sleeve release pose to the hem corner, lifted.
            let from = self.sleeve_release_pose(side).translation;
            let t = smoothstep(ratio(s.sleeve_release, s.hem_grasp));
            let arc = Vec3::Y * (0.08 * (std::f64::consts::PI as Real * t).sin());
            return Pose::from_translation(from.lerp(hem_rest, t) + arc);
        }
        if next <= s.hem_release {
            // Turn about the shoulder-hem midline (axis x through z = 0),
            // stopping short of the plane by the release height.
            let pivot = Vec3::new(hem_rest.x, hem_rest.y, 0.0);
            let theta =
                self.turn_angle(hem_rest, pivot) * smoothstep(ratio(s.hem_grasp, s.hem_turn_end));
            let rotation = Rotation::from_axis_angle(Vec3::X, theta);
            let inset = Vec3::X * (-sign * self.config.hem_inset * theta.sin());
            // A flattened arc keeps the same start and end but lowers the
            // flight, so the flap drags over the body instead of hanging.
            let arc = rotation * (hem_rest - pivot);
            let arc = Vec3::new(arc.x, arc.y * self.config.hem_arc_height, arc.z);
            let mut pose = Pose::from_translation(pivot + arc + inset);
            pose.rotation = rotation;
            return pose;
        }
        // Retract upwards after the final release.
        let from = self.hem_release_pose(side);
        let t = smoothstep(ratio(s.hem_release, s.end));
        let mut pose = Pose::from_translation(from.translation + Vec3::Y * (0.1 * t));
        pose.rotation = from.rotation;
        pose
    }
    /// The angle of a turn about `pivot` that ends `release_height` above
    /// the plane it started in, keeping the radius (the fabric length).
    fn turn_angle(&self, start: Vec3, pivot: Vec3) -> Real {
        let radius = start.distance(pivot).max(1e-9);
        let short = (self.config.release_height / radius).clamp(0.0, 1.0).asin();
        std::f64::consts::PI as Real - short
    }
    fn cuff_start(&self, side: Side) -> Vec3 {
        let k = side_index(side);
        patch_centre(&self.initial, &self.patches_at_start(k))
    }
    fn patches_at_start(&self, k: usize) -> Vec<u32> {
        let side = if k == 0 { Side::Left } else { Side::Right };
        select_patch(&self.garment, &self.config, Landmark::CuffCenter(side))
    }
    fn hem_patch(&self, side: Side) -> Vec<u32> {
        select_patch(&self.garment, &self.config, Landmark::HemCorner(side))
    }
    /// Where the hem corner was pinched, or where it started before that.
    fn hem_start(&self, side: Side) -> Vec3 {
        self.hem_grasp_centre[side_index(side)]
            .unwrap_or_else(|| patch_centre(&self.initial, &self.hem_patch(side)))
    }
    fn sleeve_release_pose(&self, side: Side) -> Pose {
        self.scripted_pose(self.schedule.sleeve_release - 1, side)
    }
    fn hem_release_pose(&self, side: Side) -> Pose {
        self.scripted_pose(self.schedule.hem_release - 1, side)
    }

    fn attach(&mut self, side: Side, vertices: Vec<u32>, compliance: Real) -> Result<(), String> {
        let k = side_index(side);
        let body = self.grippers[k];
        let pose = *self.rigid.bodies[body].position();
        let positions = self
            .world
            .cloth(self.cloth)
            .map_err(|e| e.to_string())?
            .positions();
        let points = vertices
            .iter()
            .map(|&particle| AttachmentPoint {
                particle,
                local_anchor: pose.inverse_transform_point(positions[particle as usize]),
            })
            .collect();
        let handle = self
            .world
            .attach(
                AttachmentDesc {
                    cloth: self.cloth,
                    body,
                    points,
                    compliance,
                    excluded_colliders: vec![],
                },
                &self.rigid.bodies,
                &self.rigid.colliders,
            )
            .map_err(|e| e.to_string())?;
        self.attachments[k] = Some(handle);
        self.patches[k] = vertices;
        Ok(())
    }
    fn release(&mut self, side: Side) -> Result<(), String> {
        let k = side_index(side);
        if let Some(handle) = self.attachments[k].take() {
            self.world.release(handle).map_err(|e| e.to_string())?;
        }
        Ok(())
    }

    /// One transactional step.
    pub fn tick(&mut self) -> Result<StepSample, String> {
        if let Some(error) = &self.stopped {
            return Err(error.clone());
        }
        if self.finished() {
            return Err("task completed".into());
        }
        let start = Instant::now();
        let checkpoint = self.world.checkpoint().map_err(|e| e.to_string())?;
        let rigid_before = RigidCheckpoint::capture(&self.rigid);
        let attachments_before = self.attachments;
        let patches_before = self.patches.clone();
        let step = self.step;
        let result = (|| -> Result<WorldStepReport, String> {
            let s = self.schedule;
            for side in [Side::Left, Side::Right] {
                let k = side_index(side);
                if step == s.sleeve_grasp {
                    let patch = self.patches_at_start(k);
                    self.attach(side, patch, 0.0)?;
                } else if step == s.sleeve_release {
                    self.release(side)?;
                } else if step == s.hem_grasp {
                    let patch = self.hem_patch(side);
                    // Pinch the hem corner where it lies now.
                    let positions = self
                        .world
                        .cloth(self.cloth)
                        .map_err(|e| e.to_string())?
                        .positions();
                    let centre = patch_centre(positions, &patch);
                    let body = self.grippers[k];
                    let mut pose = *self.rigid.bodies[body].position();
                    pose.translation = centre;
                    pose.rotation = Rotation::IDENTITY;
                    self.rigid.bodies[body].set_position(pose, true);
                    self.hem_grasp_centre[k] = Some(centre);
                    self.attach(side, patch, self.config.hem_compliance)?;
                } else if step == s.hem_release {
                    self.release(side)?;
                } else if step + self.config.hem_release_ramp >= s.hem_release
                    && step < s.hem_release
                    && step > s.hem_grasp
                    && self.config.hem_compliance > 0.0
                {
                    // Soften the grasp tenfold per remaining step: re-anchor at
                    // the current positions with a larger compliance.
                    let remaining = s.hem_release - step;
                    let factor =
                        (10.0 as Real).powi((self.config.hem_release_ramp - remaining + 1) as i32);
                    let patch = self.patches[k].clone();
                    self.release(side)?;
                    self.attach(side, patch, self.config.hem_compliance * factor)?;
                }
                let pose = self.scripted_pose(step, side);
                self.rigid.bodies[self.grippers[k]].set_next_kinematic_position(pose);
            }
            let before = SceneSnapshot::capture(
                self.id,
                step as u64,
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
            let scene = RapierScene::new(query, &before, self.config.h, self.rigid.gravity);
            self.world
                .step_substep(self.config.h, &scene)
                .map_err(|e| format!("{e:?}"))
        })();
        let seconds = start.elapsed().as_secs_f64();
        let report = match result {
            Ok(report) => report,
            Err(error) => {
                self.world.restore(&checkpoint).map_err(|e| e.to_string())?;
                rigid_before.restore(&mut self.rigid);
                self.attachments = attachments_before;
                self.patches = patches_before;
                let message = format!("step {}: {error}", step + 1);
                self.stopped = Some(message.clone());
                return Err(message);
            }
        };
        self.step += 1;
        let cloth_report = &report.cloths[0].1;
        let outcome = cloth_report.implicit.expect("implicit outcome");
        if !outcome.converged {
            self.approximate_steps += 1;
        }
        let extension = (cloth_report.max_stretch - 1.0).max(0.0);
        self.max_extension = self.max_extension.max(extension);
        self.max_contacts = self.max_contacts.max(cloth_report.contacts);
        let cloth = self.world.cloth(self.cloth).map_err(|e| e.to_string())?;
        let rms = (cloth
            .velocities()
            .iter()
            .map(|v| v.length_squared())
            .sum::<Real>()
            / cloth.velocities().len() as Real)
            .sqrt();
        let max_speed = cloth
            .velocities()
            .iter()
            .map(|v| v.length())
            .fold(0.0, Real::max);
        let sample = StepSample {
            step: self.step,
            time: self.step as Real * self.config.h,
            seconds,
            iterations: cloth_report.iterations,
            converged: outcome.converged,
            contacts: cloth_report.contacts,
            max_edge_extension: extension,
            rms_speed: rms,
            max_speed,
            held: self.held().len(),
        };
        self.samples.push(sample);
        self.record_frame(
            Some((extension, rms, max_speed)),
            cloth_report.iterations,
            seconds,
        );
        Ok(sample)
    }

    fn held(&self) -> Vec<u32> {
        let mut held: Vec<u32> = self
            .attachments
            .iter()
            .zip(&self.patches)
            .filter(|(a, _)| a.is_some())
            .flat_map(|(_, p)| p.iter().copied())
            .collect();
        held.sort_unstable();
        held
    }

    fn record_frame(
        &mut self,
        diagnostics: Option<(Real, Real, Real)>,
        iterations: usize,
        seconds: f64,
    ) {
        let cloth = self.world.cloth(self.cloth).expect("task cloth");
        let attached = self.held();
        let anchors = attached
            .iter()
            .map(|&i| cloth.positions()[i as usize])
            .collect();
        self.frames.push(Frame {
            step: self.step,
            positions: cloth.positions().to_vec(),
            bodies: self.grippers.map(|g| *self.rigid.bodies[g].position()),
            attached,
            anchors,
            diagnostics,
            iterations,
            seconds,
        });
    }

    /// Runs the whole script; stops at the first rejected step.
    pub fn run(&mut self) -> Result<ShirtSummary, String> {
        while !self.finished() {
            self.tick()?;
        }
        Ok(self.summary())
    }

    /// Metrics over the run so far; the gates apply to a completed run.
    pub fn summary(&self) -> ShirtSummary {
        let cloth = self.cloth();
        let positions = cloth.positions();
        let h = self.config.h;
        let simulated = self.step as Real * h;
        let window_start = simulated - 0.5 - 1e-9;
        let window: Vec<&StepSample> = self
            .samples
            .iter()
            .filter(|s| s.time >= window_start)
            .collect();
        let first_window_frame = self
            .frames
            .iter()
            .find(|f| f.step as Real * h >= window_start);
        let drift = first_window_frame
            .map(|f| {
                positions
                    .iter()
                    .zip(&f.positions)
                    .map(|(a, b)| a.distance(*b))
                    .fold(0.0, Real::max)
            })
            .unwrap_or(Real::INFINITY);
        let settled = !window.is_empty()
            && window
                .iter()
                .all(|s| s.rms_speed < 0.001 && s.max_speed < 0.005 && s.held == 0)
            && drift < 0.001;
        let footprint = |points: &[Vec3]| -> (Real, [Real; 2], [Real; 2]) {
            let (mut lo, mut hi) = ([Real::INFINITY; 2], [Real::NEG_INFINITY; 2]);
            for p in points {
                lo[0] = lo[0].min(p.x);
                lo[1] = lo[1].min(p.z);
                hi[0] = hi[0].max(p.x);
                hi[1] = hi[1].max(p.z);
            }
            ((hi[0] - lo[0]) * (hi[1] - lo[1]), lo, hi)
        };
        let (initial_area, _, _) = footprint(&self.initial);
        let (final_area, _, _) = footprint(positions);
        let half_width = self.body_half_width();
        let half_length = self.body_half_length();
        let cuffs_over_body = [Side::Left, Side::Right].map(|side| {
            let centre = patch_centre(positions, &self.patches_at_start(side_index(side)));
            centre.x.abs() <= half_width && centre.z.abs() <= half_length
        });
        let hem_short_of_shoulders = [Side::Left, Side::Right].map(|side| {
            let centre = patch_centre(positions, &self.hem_patch(side));
            half_length - centre.z
        });
        let audit = folding_oracle_audit(
            positions,
            self.garment.mesh().triangles(),
            self.config.thickness,
        );
        let min_height = positions
            .iter()
            .map(|p| p.y)
            .fold(Real::INFINITY, Real::min);
        let completed = self.finished() && self.stopped.is_none();
        let passed = completed
            && settled
            && self.max_extension <= 0.05
            && final_area <= 0.45 * initial_area
            && cuffs_over_body.iter().all(|&c| c)
            && hem_short_of_shoulders.iter().all(|&d| d.abs() <= 0.08)
            && audit.0 == 0
            && min_height >= self.config.thickness * 0.9;
        ShirtSummary {
            steps: self.step,
            simulated_seconds: simulated,
            simulation_wall_seconds: self.samples.iter().map(|s| s.seconds).sum(),
            max_edge_extension: self.max_extension,
            approximate_steps: self.approximate_steps,
            max_contacts: self.max_contacts,
            settled,
            final_window_drift: drift,
            footprint_ratio: final_area / initial_area,
            cuffs_over_body,
            hem_short_of_shoulders,
            crossing_pairs: audit.0,
            max_separation_deficit: audit.1,
            min_height,
            passed,
        }
    }

    /// A schema-2 recording for `demos/viewer`: the table and the grippers as
    /// boxes, one frame per accepted step.
    pub fn recording(&self, summary: &ShirtSummary) -> Value {
        let h = self.config.h;
        let table_top = self.config.thickness * 0.5;
        let half = 0.6 + self.sleeve_length();
        let shapes = json!([
            {"id": 0, "kind": "box", "half_extents": [half, 0.01, 0.5 + self.body_half_length()],
             "local_translation": [0.0, 0.0, 0.0], "color": "#34483e"},
            {"id": 1, "kind": "box", "half_extents": [0.012, 0.008, 0.012],
             "local_translation": [0.0, 0.0, 0.0], "color": "#c08a3e"},
            {"id": 2, "kind": "box", "half_extents": [0.012, 0.008, 0.012],
             "local_translation": [0.0, 0.0, 0.0], "color": "#3e8ac0"},
        ]);
        let s = self.schedule;
        let phase = |step: usize| -> &'static str {
            if step == 0 {
                "Initial state"
            } else if step <= s.sleeve_grasp {
                "Settling"
            } else if step <= s.sleeve_release {
                "Folding the sleeves"
            } else if step <= s.hem_grasp {
                "Sleeves released"
            } else if step <= s.hem_release {
                "Folding the hem to the shoulders"
            } else {
                "Released"
            }
        };
        let frames: Vec<Value> = self
            .frames
            .iter()
            .map(|f| {
                let mut bodies = vec![json!({"id": 0, "translation": [0.0, table_top - 0.01, 0.0],
                    "rotation": [0.0, 0.0, 0.0, 1.0]})];
                for (i, pose) in f.bodies.iter().enumerate() {
                    let q = pose.rotation.to_array();
                    bodies.push(
                        json!({"id": i + 1, "translation": pose.translation.to_array(),
                        "rotation": [q[0], q[1], q[2], q[3]]}),
                    );
                }
                json!({
                    "step": f.step,
                    "time": f.step as Real * h,
                    "phase": phase(f.step),
                    "positions": f.positions.iter().map(|p| p.to_array()).collect::<Vec<_>>(),
                    "bodies": bodies,
                    "attached_particles": f.attached,
                    "anchors": f.anchors.iter().map(|p| p.to_array()).collect::<Vec<_>>(),
                    "pinned_particles": Vec::<u32>::new(),
                    "diagnostics": f.diagnostics.map(|(extension, rms, max)| json!({
                        "max_edge_extension": extension, "rms_speed": rms, "max_speed": max,
                        "iterations": f.iterations, "seconds": f.seconds})),
                })
            })
            .collect();
        let completed = self.finished() && self.stopped.is_none();
        let outcome = json!({
            "stop_reason": if completed { "completed" } else { "solver_error" },
            "steps": self.step,
            "end_step": s.end,
            "failure": self.stopped,
        });
        let mut recording = json!({
            "schema_version": 2,
            "precision": "f64",
            "config": {
                "solver": "implicit",
                "h": h,
                "task": "fold_shirt",
                "layers": format!("{:?}", self.config.pattern.layers).to_lowercase(),
                "spacing": self.config.pattern.spacing,
                "neck": format!("{:?}", self.config.pattern.neck).to_lowercase(),
                "seam_stiffness": self.config.pattern.seam_stiffness,
                "warp_stiffness": self.config.warp_stiffness,
                "weft_stiffness": self.config.weft_stiffness,
                "thickness": self.config.thickness,
                "band": self.config.band,
                "friction": self.config.friction,
                "schedule": {"sleeve_grasp": s.sleeve_grasp, "sleeve_turn_end": s.sleeve_turn_end,
                    "sleeve_release": s.sleeve_release, "hem_grasp": s.hem_grasp,
                    "hem_turn_end": s.hem_turn_end, "hem_release": s.hem_release, "end": s.end},
                "landmarks": self.landmark_indices(),
            },
            "triangles": self.garment.mesh().triangles(),
            "shapes": shapes,
            "frames": frames,
            "outcome": outcome,
        });
        if completed {
            recording["summary"] = json!({
                "settled": summary.settled,
                "simulated_seconds": summary.simulated_seconds,
                "simulation_wall_seconds": summary.simulation_wall_seconds,
                "final_window_drift": summary.final_window_drift,
                "max_edge_extension": summary.max_edge_extension,
                "footprint_ratio": summary.footprint_ratio,
                "passed": summary.passed,
            });
        }
        recording
    }
}

impl ShirtTask {
    /// Landmark vertex indices by name, for recordings and hosts.
    pub fn landmark_indices(&self) -> Value {
        let named = [
            ("hem_left", Landmark::HemCorner(Side::Left)),
            ("hem_right", Landmark::HemCorner(Side::Right)),
            ("hem_center", Landmark::HemCenter),
            ("cuff_left", Landmark::CuffCenter(Side::Left)),
            ("cuff_right", Landmark::CuffCenter(Side::Right)),
            ("shoulder_left", Landmark::Shoulder(Side::Left)),
            ("shoulder_right", Landmark::Shoulder(Side::Right)),
            ("underarm_left", Landmark::Underarm(Side::Left)),
            ("underarm_right", Landmark::Underarm(Side::Right)),
            ("neck_front", Landmark::NeckFront),
            ("neck_back", Landmark::NeckBack),
            ("chest", Landmark::Chest),
        ];
        let mut map = serde_json::Map::new();
        for (name, landmark) in named {
            if let Some(index) = self.garment.landmark(landmark) {
                map.insert(name.to_owned(), json!(index));
            }
        }
        Value::Object(map)
    }
}

fn select_patch(garment: &Garment, config: &ShirtTaskConfig, landmark: Landmark) -> Vec<u32> {
    let radius = config.patch_radius.max(1.5 * config.pattern.spacing);
    match config.pinch {
        Pinch::TopLayer => garment.top_patch(landmark, radius),
        Pinch::AllLayers => garment.patch(landmark, radius),
    }
}

impl ShirtTask {
    /// Writes the run as a USD ASCII stage: the garment as a `Mesh` with
    /// time-sampled points, the table and the grippers as `Cube` prims under
    /// time-sampled `Xform`s, one time code per accepted step at
    /// `1 / h` codes per second, Y up, metres. Hosts that read USD time
    /// samples (botrail's scenes are USD) can play it back without the viewer.
    pub fn write_usda(&self, path: &std::path::Path) -> std::io::Result<()> {
        use std::io::Write;
        let mut out = std::io::BufWriter::new(std::fs::File::create(path)?);
        let h = self.config.h;
        let last = self.frames.last().map_or(0, |f| f.step);
        writeln!(out, "#usda 1.0")?;
        writeln!(
            out,
            "(\n    defaultPrim = \"World\"\n    startTimeCode = 0\n    endTimeCode = {last}\n    timeCodesPerSecond = {}\n    metersPerUnit = 1\n    upAxis = \"Y\"\n)",
            1.0 / h
        )?;
        writeln!(out, "def Xform \"World\"\n{{")?;
        let mesh = self.garment.mesh();
        writeln!(out, "    def Mesh \"garment\"\n    {{")?;
        write!(out, "        int[] faceVertexCounts = [")?;
        for (k, _) in mesh.triangles().iter().enumerate() {
            write!(out, "{}3", if k == 0 { "" } else { ", " })?;
        }
        writeln!(out, "]")?;
        write!(out, "        int[] faceVertexIndices = [")?;
        for (k, t) in mesh.triangles().iter().enumerate() {
            write!(
                out,
                "{}{}, {}, {}",
                if k == 0 { "" } else { ", " },
                t[0],
                t[1],
                t[2]
            )?;
        }
        writeln!(out, "]")?;
        writeln!(out, "        uniform token subdivisionScheme = \"none\"")?;
        // A default value as well as the samples: importers that read the
        // default time (botrail's does) get the initial shape.
        if let Some(first) = self.frames.first() {
            write!(out, "        point3f[] points = [")?;
            for (k, p) in first.positions.iter().enumerate() {
                write!(
                    out,
                    "{}({}, {}, {})",
                    if k == 0 { "" } else { ", " },
                    p.x,
                    p.y,
                    p.z
                )?;
            }
            writeln!(out, "]")?;
        }
        writeln!(out, "        point3f[] points.timeSamples = {{")?;
        for f in &self.frames {
            write!(out, "            {}: [", f.step)?;
            for (k, p) in f.positions.iter().enumerate() {
                write!(
                    out,
                    "{}({}, {}, {})",
                    if k == 0 { "" } else { ", " },
                    p.x,
                    p.y,
                    p.z
                )?;
            }
            writeln!(out, "],")?;
        }
        writeln!(out, "        }}\n    }}")?;
        let table_top = self.config.thickness * 0.5;
        let half = 0.6 + self.sleeve_length();
        writeln!(
            out,
            "    def Cube \"table\"\n    {{\n        double size = 1\n        float3 xformOp:scale = ({}, 0.02, {})\n        double3 xformOp:translate = (0, {}, 0)\n        uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:scale\"]\n    }}",
            2.0 * half,
            2.0 * (0.5 + self.body_half_length()),
            table_top - 0.01
        )?;
        for (g, name) in ["gripper_left", "gripper_right"].iter().enumerate() {
            writeln!(out, "    def Xform \"{name}\"\n    {{")?;
            writeln!(out, "        matrix4d xformOp:transform.timeSamples = {{")?;
            for f in &self.frames {
                let pose = f.bodies[g];
                let (cx, cy, cz) = (
                    pose.rotation * Vec3::X,
                    pose.rotation * Vec3::Y,
                    pose.rotation * Vec3::Z,
                );
                let t = pose.translation;
                writeln!(
                    out,
                    "            {}: ( ({}, {}, {}, 0), ({}, {}, {}, 0), ({}, {}, {}, 0), ({}, {}, {}, 1) ),",
                    f.step, cx.x, cx.y, cx.z, cy.x, cy.y, cy.z, cz.x, cz.y, cz.z, t.x, t.y, t.z
                )?;
            }
            writeln!(out, "        }}")?;
            writeln!(
                out,
                "        uniform token[] xformOpOrder = [\"xformOp:transform\"]"
            )?;
            writeln!(
                out,
                "        def Cube \"finger\"\n        {{\n            double size = 1\n            float3 xformOp:scale = (0.024, 0.016, 0.024)\n            uniform token[] xformOpOrder = [\"xformOp:scale\"]\n        }}"
            )?;
            writeln!(out, "    }}")?;
        }
        writeln!(out, "}}")?;
        out.flush()
    }
}

fn patch_centre(positions: &[Vec3], patch: &[u32]) -> Vec3 {
    let sum: Vec3 = patch.iter().map(|&i| positions[i as usize]).sum();
    sum / patch.len().max(1) as Real
}

/// Non-incident triangle pairs that cross, and the largest separation deficit,
/// from the independent folding oracle.
fn folding_oracle_audit(
    positions: &[Vec3],
    triangles: &[[u32; 3]],
    thickness: Real,
) -> (usize, f64) {
    let points: Vec<[f64; 3]> = positions.iter().map(|p| [p.x, p.y, p.z]).collect();
    let audit = super::folding_oracle::audit_surface(&points, triangles, thickness);
    (audit.crossing_pairs, audit.max_separation_deficit)
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
