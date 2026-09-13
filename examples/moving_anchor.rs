use rapier::prelude::*;
use rapier_cloth::{Real, *};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let id = WorldId::new();
    let h: Real = 1.0 / 240.0;
    let mut rigid = PhysicsWorld::new();
    rigid.integration_parameters.dt = h;
    let body = rigid
        .bodies
        .insert(RigidBodyBuilder::kinematic_position_based().translation(Vec3::Y));
    let mut world = RapierClothWorld::new(id);
    let cloth = world.add_cloth(Cloth::new(
        GridBuilder::new(16, 16).origin(Vec3::Y).build()?,
        ClothMaterial::default(),
    )?);
    let attachment = world.attach(
        AttachmentDesc {
            cloth,
            body,
            points: vec![AttachmentPoint {
                particle: 0,
                local_anchor: Vec3::ZERO,
            }],
            compliance: 0.0,
            excluded_colliders: vec![],
        },
        &rigid.bodies,
        &rigid.colliders,
    )?;
    for step in 0..240 {
        let before = SceneSnapshot::capture(id, step, &rigid.bodies, &rigid.colliders);
        rigid.bodies[body]
            .set_next_kinematic_translation(Vec3::Y + Vec3::X * ((step + 1) as Real * h * 0.1));
        rigid.step();
        let q = rigid.broad_phase.as_query_pipeline(
            rigid.narrow_phase.query_dispatcher(),
            &rigid.bodies,
            &rigid.colliders,
            QueryFilter::default(),
        );
        world.step_substep(h, &RapierScene::new(q, &before, h, rigid.gravity))?;
    }
    world.release(attachment)?;
    println!(
        "released position={:?}, velocity={:?}",
        world.cloth(cloth)?.positions()[0],
        world.cloth(cloth)?.velocities()[0]
    );
    Ok(())
}
