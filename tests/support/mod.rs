#![allow(dead_code)]
use rapier::prelude::*;
use rapier_cloth::Real;
use rapier_cloth::*;

pub struct TestWorld {
    pub rigid: PhysicsWorld,
    pub cloth: RapierClothWorld,
    pub id: WorldId,
    pub step: u64,
    pub h: Real,
}
impl TestWorld {
    pub fn new() -> Self {
        let id = WorldId::new();
        let mut rigid = PhysicsWorld::new();
        let h = 1.0 / 240.0;
        rigid.integration_parameters.dt = h;
        Self {
            rigid,
            cloth: RapierClothWorld::new(id),
            id,
            step: 0,
            h,
        }
    }
    pub fn tick(&mut self) -> Result<WorldStepReport, IntegrationError> {
        self.tick_filtered(QueryFilter::default())
    }
    pub fn tick_filtered(
        &mut self,
        filter: QueryFilter<'_>,
    ) -> Result<WorldStepReport, IntegrationError> {
        let before = SceneSnapshot::capture(
            self.id,
            self.step,
            &self.rigid.bodies,
            &self.rigid.colliders,
        );
        self.rigid.integration_parameters.dt = self.h;
        self.rigid.step();
        let query = self.rigid.broad_phase.as_query_pipeline(
            self.rigid.narrow_phase.query_dispatcher(),
            &self.rigid.bodies,
            &self.rigid.colliders,
            filter,
        );
        let scene = RapierScene::new(query, &before, self.h, self.rigid.gravity);
        let result = self.cloth.step_substep(self.h, &scene);
        self.step += 1;
        result
    }
    pub fn grid(&mut self, n: usize, size: Real, origin: Vec3) -> ClothHandle {
        self.cloth.add_cloth(
            Cloth::new(
                GridBuilder::new(n, n)
                    .size(size, size)
                    .origin(origin)
                    .build()
                    .unwrap(),
                ClothMaterial {
                    damping: 0.0,
                    ..Default::default()
                },
            )
            .unwrap(),
        )
    }
}
