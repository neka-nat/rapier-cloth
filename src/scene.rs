use crate::{
    Real, Vec3,
    rapier::{pipeline::QueryPipeline, prelude::*},
};
use std::{
    collections::BTreeMap,
    sync::atomic::{AtomicU64, Ordering},
};

/// Application world identity. A token detects accidental mismatches; callers
/// remain responsible for associating it with the same live Rapier world.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct WorldId(u64);
static NEXT_WORLD: AtomicU64 = AtomicU64::new(1);
impl Default for WorldId {
    fn default() -> Self {
        Self::new()
    }
}
impl WorldId {
    pub fn new() -> Self {
        Self(
            NEXT_WORLD
                .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |x| x.checked_add(1))
                .expect("world identity exhausted"),
        )
    }
}

/// Capture before setting the next kinematic target and stepping Rapier.
/// step is the external substep number, starting at zero.
#[derive(Debug, Clone)]
pub struct SceneSnapshot {
    pub(crate) world: WorldId,
    pub(crate) step: u64,
    pub(crate) bodies: BTreeMap<(u32, u32), Pose>,
    pub(crate) colliders: BTreeMap<(u32, u32), Pose>,
}
impl SceneSnapshot {
    pub fn capture(
        world: WorldId,
        step: u64,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
    ) -> Self {
        Self {
            world,
            step,
            bodies: bodies
                .iter()
                .map(|(h, b)| (h.into_raw_parts(), *b.position()))
                .collect(),
            colliders: colliders
                .iter()
                .map(|(h, c)| {
                    let pose = c
                        .parent()
                        .and_then(|b| bodies.get(b))
                        .zip(c.position_wrt_parent())
                        .map_or(*c.position(), |(b, local)| b.position() * local);
                    (h.into_raw_parts(), pose)
                })
                .collect(),
        }
    }
    pub fn world_id(&self) -> WorldId {
        self.world
    }
    pub fn step_index(&self) -> u64 {
        self.step
    }
    pub fn body_pose(&self, body: RigidBodyHandle) -> Option<&Pose> {
        self.bodies.get(&body.into_raw_parts())
    }
}

/// Read-only view after Rapier has advanced exactly h. Constructing a query
/// does not update the BVH; see the integration guide for the required order.
pub struct RapierScene<'a> {
    pub query: QueryPipeline<'a>,
    pub previous: &'a SceneSnapshot,
    pub h: Real,
    pub gravity: Vec3,
}
impl<'a> RapierScene<'a> {
    pub fn new(
        query: QueryPipeline<'a>,
        previous: &'a SceneSnapshot,
        h: Real,
        gravity: Vec3,
    ) -> Self {
        Self {
            query,
            previous,
            h,
            gravity,
        }
    }
}
