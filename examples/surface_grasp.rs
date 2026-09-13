//! Select an exposed triangle interior, lift its weighted material point and
//! release it. This is an ideal attachment API example, not fingertip physics.
//! Run: cargo run --release --example surface_grasp
use rapier::prelude::*;
use rapier_cloth::{Real, *};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let id = WorldId::new();
    let h: Real = 1.0 / 240.0;
    let mut rigid = PhysicsWorld::new();
    rigid.integration_parameters.dt = h;
    let body = rigid
        .bodies
        .insert(RigidBodyBuilder::kinematic_position_based());
    let mut world = RapierClothWorld::new(id);
    let handle = world.add_cloth(Cloth::new(
        GridBuilder::new(4, 4)
            .size(0.3, 0.3)
            .origin(Vec3::Y * 0.3)
            .build()?,
        ClothMaterial {
            bend_compliance: 10000.0,
            ..Default::default()
        },
    )?);
    let hit = world
        .raycast_cloth(
            SurfaceRay {
                origin: Vec3::new(0.04, 0.8, 0.03),
                direction: -Vec3::Y,
                max_distance: 1.0,
            },
            SurfaceQueryLimits::default(),
        )?
        .ok_or("no exposed cloth surface")?;
    assert_eq!(hit.point.cloth, handle);
    assert!(hit.point.point.barycentric().iter().all(|b| *b > 0.0));
    let grasp = world.grasp_surface(
        hit.point,
        GraspOptions::new(body),
        &rigid.bodies,
        &rigid.colliders,
    )?;
    let mut max_target_error: Real = 0.0;
    for step in 0..240 {
        if step == 120 {
            world.release(grasp)?;
        }
        let before = SceneSnapshot::capture(id, step, &rigid.bodies, &rigid.colliders);
        rigid.bodies[body]
            .set_next_kinematic_translation(Vec3::Y * ((step + 1).min(120) as Real * h * 0.1));
        rigid.step();
        let query = rigid.broad_phase.as_query_pipeline(
            rigid.narrow_phase.query_dispatcher(),
            &rigid.bodies,
            &rigid.colliders,
            QueryFilter::default(),
        );
        let report = world.step_substep(h, &RapierScene::new(query, &before, h, rigid.gravity))?;
        max_target_error = max_target_error.max(report.cloths[0].1.max_target_error);
    }
    let cloth = world.cloth(handle)?;
    assert!(max_target_error <= core::math::LENGTH_EPSILON);
    assert!(cloth.pins().is_empty());
    assert_eq!(world.surface_attachments().count(), 0);
    let finite = cloth
        .positions()
        .iter()
        .chain(cloth.velocities())
        .all(|p| p.is_finite());
    assert!(finite);
    println!(
        "{}",
        serde_json::json!({
            "precision": if cfg!(feature="f64") { "f64" } else { "f32" },
            "steps": 240, "surface_triangle": hit.point.point.triangle(),
            "barycentric": hit.point.point.barycentric(), "max_target_error": max_target_error,
            "released": true, "finite": finite,
        })
    );
    Ok(())
}
