//! Cloth contacts with a fixed Rapier sphere and floor; runs for 5 seconds.
//! Run: cargo run --release --example drape_static
use rapier::prelude::*;
use rapier_cloth::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let id = WorldId::new();
    let mut rigid = PhysicsWorld::new();
    let mut cloth = RapierClothWorld::new(id);
    let h = 1.0 / 240.0;
    rigid.integration_parameters.dt = h;
    rigid.colliders.insert(ColliderBuilder::ball(0.35));
    rigid
        .colliders
        .insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)).translation(-Vec3::Y * 0.5));
    cloth.add_cloth(Cloth::new(
        GridBuilder::new(24, 24)
            .size(0.6, 0.6)
            .origin(Vec3::new(-0.3, 0.7, -0.3))
            .build()?,
        ClothMaterial::default(),
    )?);
    let mut report = WorldStepReport::default();
    for step in 0..1200 {
        let previous = SceneSnapshot::capture(id, step, &rigid.bodies, &rigid.colliders);
        rigid.step();
        let query = rigid.broad_phase.as_query_pipeline(
            rigid.narrow_phase.query_dispatcher(),
            &rigid.bodies,
            &rigid.colliders,
            QueryFilter::default(),
        );
        report = cloth.step_substep(h, &RapierScene::new(query, &previous, h, rigid.gravity))?;
    }
    println!(
        "p95_stretch={}, max_penetration={}, contacts={}",
        report.cloths[0].1.p95_stretch,
        report.cloths[0].1.max_penetration,
        report.cloths[0].1.contacts
    );
    Ok(())
}
