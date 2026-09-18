#![cfg(all(feature = "implicit", feature = "f64"))]
use rapier_cloth::rapier::prelude::*;
use rapier_cloth::*;

#[test]
fn checkpoint_restores_solver_choice_and_replays_the_same_step() {
    replay(ImplicitExecution::Serial, 3);
}

#[test]
fn checkpoint_restores_parallel_execution_and_replays_the_same_step() {
    replay(ImplicitExecution::Parallel4, 32);
}

fn replay(execution: ImplicitExecution, grid: usize) {
    let id = WorldId::new();
    let mut rigid = PhysicsWorld::new();
    rigid.integration_parameters.dt = 0.1;
    let mut world = RapierClothWorld::new(id);
    world.solver_settings.max_substep = 0.1;
    let mesh = GridBuilder::new(grid, grid)
        .size(0.25, 0.25)
        .origin(Vec3::Y)
        .build()
        .unwrap();
    let mut cloth = Cloth::new(
        mesh,
        ClothMaterial {
            damping: 0.0,
            ..Default::default()
        },
    )
    .unwrap();
    cloth
        .set_contact_settings(Some(ClothContactSettings {
            self_collision: false,
            ..Default::default()
        }))
        .unwrap();
    let implicit = ImplicitSettings {
        execution,
        ..Default::default()
    };
    cloth.set_implicit_solver(Some(implicit)).unwrap();
    let handle = world.add_cloth(cloth);
    let checkpoint = world.checkpoint().unwrap();
    let run = |world: &mut RapierClothWorld, rigid: &mut PhysicsWorld| {
        let previous = SceneSnapshot::capture(id, 0, &rigid.bodies, &rigid.colliders);
        rigid.step();
        let query = rigid.broad_phase.as_query_pipeline(
            rigid.narrow_phase.query_dispatcher(),
            &rigid.bodies,
            &rigid.colliders,
            QueryFilter::default(),
        );
        world
            .step_substep(0.1, &RapierScene::new(query, &previous, 0.1, rigid.gravity))
            .unwrap();
    };
    run(&mut world, &mut rigid);
    let expected = world.cloth(handle).unwrap().positions().to_vec();
    let expected_velocities = world.cloth(handle).unwrap().velocities().to_vec();
    world
        .cloth_mut(handle)
        .unwrap()
        .set_implicit_solver(None)
        .unwrap();
    world.restore(&checkpoint).unwrap();
    // This scene contains no rigid bodies or colliders. Recreate that empty
    // rigid state separately; cloth checkpoints do not snapshot Rapier.
    rigid = PhysicsWorld::new();
    rigid.integration_parameters.dt = 0.1;
    assert_eq!(
        world.cloth(handle).unwrap().implicit_solver_settings(),
        Some(implicit)
    );
    run(&mut world, &mut rigid);
    assert_eq!(world.cloth(handle).unwrap().positions(), expected);
    assert_eq!(
        world.cloth(handle).unwrap().velocities(),
        expected_velocities
    );
}
