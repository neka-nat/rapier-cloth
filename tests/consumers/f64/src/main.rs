use rapier_cloth::{rapier, *};
use rapier3d_f64::prelude::{PhysicsWorld, RigidBodyBuilder, SharedShape, ColliderBuilder, QueryFilter};
fn main() {
    let id=WorldId::new();
    let mut rigid=PhysicsWorld::new();
    rigid.integration_parameters.dt=1.0/240.0;
    let body:rapier::prelude::RigidBodyHandle=rigid.bodies.insert(RigidBodyBuilder::fixed());
    let shape:rapier::parry::shape::SharedShape=SharedShape::ball(0.2);
    rigid.colliders.insert_with_parent(ColliderBuilder::new(shape),body,&mut rigid.bodies);
    let mut cloths=RapierClothWorld::new(id);
    let handle=cloths.add_cloth(Cloth::new(GridBuilder::new(2,2).origin(Vec3::Y).build().unwrap(),ClothMaterial::default()).unwrap());
    let before=SceneSnapshot::capture(id,0,&rigid.bodies,&rigid.colliders);
    rigid.step();
    let query=rigid.broad_phase.as_query_pipeline(rigid.narrow_phase.query_dispatcher(),&rigid.bodies,&rigid.colliders,QueryFilter::default());
    let report=cloths.step_substep(rigid.integration_parameters.dt,&RapierScene::new(query,&before,rigid.integration_parameters.dt,rigid.gravity)).unwrap();
    assert_eq!(report.step,0);
    { let surface=cloths.cloth(handle).unwrap().surface();assert_eq!(surface.positions.len(),4);assert!(surface.positions[0].y<1.0); }
    cloths.cloth_mut(handle).unwrap().pin(0,Vec3::Y).unwrap();
    continuous_surface_contact();
}
fn continuous_surface_contact() {
    let id = WorldId::new();
    let mut rigid = PhysicsWorld::new();
    let h = 1.0 / 240.0;
    rigid.integration_parameters.dt = h;
    rigid.colliders.insert(ColliderBuilder::new(SharedShape::halfspace(Vec3::Y)).friction(0.0));
    let mut cloth = Cloth::new(GridBuilder::new(2, 2).origin(Vec3::Y * 0.0005).build().unwrap(),
        ClothMaterial { damping: 0.0, ..Default::default() }).unwrap();
    cloth.set_contact_settings(Some(ClothContactSettings {
        self_collision: false, rigid_surface_collision: true, continuous_rigid_collision: true,
        static_friction: 0.0, kinetic_friction: 0.0, ..Default::default()
    })).unwrap();
    let mut cloths = RapierClothWorld::new(id);
    let handle = cloths.add_cloth(cloth);
    let before = SceneSnapshot::capture(id, 0, &rigid.bodies, &rigid.colliders);
    rigid.step();
    let query = rigid.broad_phase.as_query_pipeline(rigid.narrow_phase.query_dispatcher(),
        &rigid.bodies, &rigid.colliders, QueryFilter::default());
    let report = cloths.step_substep(h, &RapierScene::new(query, &before, h, rigid.gravity)).unwrap();
    assert!(report.cloths[0].1.surface_collision.ccd_checks > 0);
    assert!(report.cloths[0].1.surface_collision.limited_advances > 0);
    assert!(cloths.cloth(handle).unwrap().positions().iter().all(|p| (p.y - 0.0005).abs() < 1.0e-6));
}
#[test]
fn direct_dependency_can_exchange_types_and_step_cloth(){main();}
