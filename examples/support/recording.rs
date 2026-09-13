#![allow(dead_code)]
use rapier::prelude::*;
use rapier_cloth::{Real, *};
use serde::{Deserialize, Serialize};

pub const SCHEMA_VERSION: u32 = 1;
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    pub grid: usize,
    pub size: Real,
    pub h: Real,
    pub iterations: usize,
    pub attach_step: u64,
    pub lift_end: u64,
    pub release_step: u64,
    pub end_step: u64,
    pub lift_height: Real,
    pub transport_distance: Real,
    pub record_stride: u64,
    pub surface_density: Real,
    pub stretch_compliance: Real,
    pub bend_compliance: Real,
    pub damping: Real,
    pub friction: Real,
    pub contact_radius: Real,
}
impl Default for Config {
    fn default() -> Self {
        serde_json::from_str(include_str!("pick_fixture.json")).unwrap()
    }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BodyShape {
    pub id: u32,
    pub kind: String,
    pub half_extents: [Real; 3],
    pub local_translation: [Real; 3],
    pub color: String,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BodyFrame {
    pub id: u32,
    pub translation: [Real; 3],
    pub rotation: [Real; 4],
}
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Diagnostics {
    pub max_stretch: Real,
    pub p95_stretch: Real,
    pub max_penetration: Real,
    pub max_target_error: Real,
    pub contacts: usize,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Frame {
    pub step: u64,
    pub time: f64,
    pub phase: String,
    pub positions: Vec<[Real; 3]>,
    pub bodies: Vec<BodyFrame>,
    pub attached_particles: Vec<u32>,
    pub anchors: Vec<[Real; 3]>,
    pub pinned_particles: Vec<u32>,
    pub diagnostics: Diagnostics,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Recording {
    pub schema_version: u32,
    pub precision: String,
    pub config: Config,
    pub triangles: Vec<[u32; 3]>,
    pub shapes: Vec<BodyShape>,
    pub frames: Vec<Frame>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Summary {
    pub schema_version: u32,
    pub precision: String,
    pub steps: u64,
    pub finite: bool,
    pub release_time: f64,
    pub release_velocities: Vec<[Real; 3]>,
    pub lifted_y: Real,
    pub transported_anchor_x: Real,
    pub final_center: [Real; 3],
    pub final_max_y: Real,
    pub max_penetration: Real,
    pub max_stretch: Real,
    pub max_target_error: Real,
}
fn time(step: u64, h: Real) -> f64 {
    #[cfg(feature = "f64")]
    let seconds = h;
    #[cfg(not(feature = "f64"))]
    let seconds = f64::from(h);
    step as f64 * seconds
}
fn body_frame(id: u32, pose: &Pose) -> BodyFrame {
    BodyFrame {
        id,
        translation: pose.translation.to_array(),
        rotation: [
            pose.rotation.x,
            pose.rotation.y,
            pose.rotation.z,
            pose.rotation.w,
        ],
    }
}
// Immutable views are passed explicitly; this helper owns no simulation state.
#[allow(clippy::too_many_arguments)]
fn frame(
    step: u64,
    phase: &str,
    config: &Config,
    cloth: &RapierClothWorld,
    handle: ClothHandle,
    rigid: &PhysicsWorld,
    grip: RigidBodyHandle,
    diagnostics: Diagnostics,
) -> Frame {
    let state = cloth.cloth(handle).unwrap();
    let mut attached_particles = vec![];
    let mut anchors = vec![];
    for (_, a) in cloth.attachments() {
        for p in &a.points {
            attached_particles.push(p.particle);
            anchors.push(
                rigid.bodies[a.body]
                    .position()
                    .transform_point(p.local_anchor)
                    .to_array(),
            );
        }
    }
    Frame {
        step,
        time: time(step, config.h),
        phase: phase.into(),
        positions: state.positions().iter().map(|p| p.to_array()).collect(),
        bodies: vec![
            body_frame(0, &Pose::translation(0.2, -0.05, 0.0)),
            body_frame(1, rigid.bodies[grip].position()),
        ],
        attached_particles,
        anchors,
        pinned_particles: state.pins().keys().copied().collect(),
        diagnostics,
    }
}
/// Deterministic CPU simulation shared by the headless example and its tests.
pub fn simulate(config: Config) -> Result<(Recording, Summary), Box<dyn std::error::Error>> {
    if !(config.attach_step < config.lift_end
        && config.lift_end < config.release_step
        && config.release_step < config.end_step)
        || config.record_stride == 0
    {
        return Err("invalid scenario timing".into());
    }
    let id = WorldId::new();
    let mut rigid = PhysicsWorld::new();
    rigid.integration_parameters.dt = config.h;
    let mut world = RapierClothWorld::new(id);
    world.solver_settings.iterations = config.iterations;
    rigid
        .colliders
        .insert(ColliderBuilder::cuboid(2.0, 0.05, 2.0).translation(Vec3::new(0.2, -0.05, 0.0)));
    let origin = Vec3::new(
        -config.size * 0.5,
        config.contact_radius,
        -config.size * 0.5,
    );
    let grip = rigid
        .bodies
        .insert(RigidBodyBuilder::kinematic_position_based().translation(origin));
    let grip_collider = rigid.colliders.insert_with_parent(
        ColliderBuilder::cuboid(config.size * 0.55, 0.015, 0.02).translation(Vec3::new(
            config.size * 0.5,
            0.04,
            0.0,
        )),
        grip,
        &mut rigid.bodies,
    );
    let mesh = GridBuilder::new(config.grid, config.grid)
        .size(config.size, config.size)
        .origin(origin)
        .build()?;
    let triangles = mesh.triangles().to_vec();
    let cloth = world.add_cloth(Cloth::new(
        mesh,
        ClothMaterial {
            surface_density: config.surface_density,
            stretch_compliance: config.stretch_compliance,
            bend_compliance: config.bend_compliance,
            damping: config.damping,
            friction: config.friction,
            contact_radius: config.contact_radius,
        },
    )?);
    let precision = if cfg!(feature = "f64") { "f64" } else { "f32" }.to_string();
    let mut recording = Recording {
        schema_version: SCHEMA_VERSION,
        precision: precision.clone(),
        config: config.clone(),
        triangles,
        shapes: vec![
            BodyShape {
                id: 0,
                kind: "box".into(),
                half_extents: [2.0, 0.05, 2.0],
                local_translation: [0.0; 3],
                color: "#74808f".into(),
            },
            BodyShape {
                id: 1,
                kind: "box".into(),
                half_extents: [config.size * 0.55, 0.015, 0.02],
                local_translation: [config.size * 0.5, 0.04, 0.0],
                color: "#f5a94d".into(),
            },
        ],
        frames: vec![],
    };
    let mut summary = Summary {
        schema_version: SCHEMA_VERSION,
        precision,
        steps: config.end_step,
        finite: true,
        release_time: time(config.release_step, config.h),
        release_velocities: vec![],
        lifted_y: 0.0,
        transported_anchor_x: 0.0,
        final_center: [0.0; 3],
        final_max_y: 0.0,
        max_penetration: 0.0,
        max_stretch: 0.0,
        max_target_error: 0.0,
    };
    recording.frames.push(frame(
        0,
        "settle",
        &config,
        &world,
        cloth,
        &rigid,
        grip,
        Diagnostics::default(),
    ));
    let mut attachment = None;
    for step in 0..config.end_step {
        let before = SceneSnapshot::capture(id, step, &rigid.bodies, &rigid.colliders);
        if step == config.attach_step {
            let points = (0..config.grid as u32)
                .map(|particle| AttachmentPoint {
                    particle,
                    local_anchor: rigid.bodies[grip].position().inverse_transform_point(
                        world.cloth(cloth).unwrap().positions()[particle as usize],
                    ),
                })
                .collect();
            attachment = Some(world.attach(
                AttachmentDesc {
                    cloth,
                    body: grip,
                    points,
                    compliance: 0.0,
                    excluded_colliders: vec![grip_collider],
                },
                &rigid.bodies,
                &rigid.colliders,
            )?);
        }
        let end = step + 1;
        let lift = (end.saturating_sub(config.attach_step) as Real
            / (config.lift_end - config.attach_step) as Real)
            .min(1.0);
        let travel = end.saturating_sub(config.lift_end) as Real
            / (config.release_step - config.lift_end) as Real;
        let next = origin
            + Vec3::Y * (lift * config.lift_height)
            + Vec3::X * (travel * config.transport_distance);
        rigid.bodies[grip].set_next_kinematic_translation(next);
        rigid.step();
        let query = rigid.broad_phase.as_query_pipeline(
            rigid.narrow_phase.query_dispatcher(),
            &rigid.bodies,
            &rigid.colliders,
            QueryFilter::default(),
        );
        // On failure Rapier is already at t+h. Stop; a retry requires restoring
        // both worlds, never stepping only the old cloth against this new scene.
        let report = world.step_substep(
            config.h,
            &RapierScene::new(query, &before, config.h, rigid.gravity),
        )?;
        let r = &report.cloths[0].1;
        summary.max_penetration = summary.max_penetration.max(r.max_penetration);
        summary.max_stretch = summary.max_stretch.max(r.max_stretch);
        summary.max_target_error = summary.max_target_error.max(r.max_target_error);
        if end == config.lift_end {
            summary.lifted_y = world.cloth(cloth)?.positions()[0].y;
        }
        if end == config.release_step {
            summary.transported_anchor_x = world.cloth(cloth)?.positions()[0].x;
            summary.release_velocities = world.cloth(cloth)?.velocities()[..config.grid]
                .iter()
                .map(|v| v.to_array())
                .collect();
            world.release(attachment.take().ok_or("attachment absent at release")?)?;
        }
        let phase = if end < config.attach_step {
            "settle"
        } else if end < config.lift_end {
            "lift"
        } else if end < config.release_step {
            "transport"
        } else if end == config.release_step {
            "release"
        } else {
            "drop"
        };
        if end % config.record_stride == 0
            || [
                config.attach_step,
                config.lift_end,
                config.release_step,
                config.end_step,
            ]
            .contains(&end)
        {
            recording.frames.push(frame(
                end,
                phase,
                &config,
                &world,
                cloth,
                &rigid,
                grip,
                Diagnostics {
                    max_stretch: r.max_stretch,
                    p95_stretch: r.p95_stretch,
                    max_penetration: r.max_penetration,
                    max_target_error: r.max_target_error,
                    contacts: r.contacts,
                },
            ));
        }
    }
    let state = world.cloth(cloth)?;
    summary.final_center = (state.positions().iter().copied().sum::<Vec3>()
        / state.positions().len() as Real)
        .to_array();
    summary.final_max_y = state
        .positions()
        .iter()
        .map(|p| p.y)
        .reduce(Real::max)
        .unwrap();
    Ok((recording, summary))
}
