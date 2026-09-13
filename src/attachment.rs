use crate::{ClothHandle, Real, Vec3, rapier::prelude::*};
use std::sync::atomic::{AtomicU64, Ordering};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct AttachmentHandle {
    arena: u64,
    index: u32,
    generation: u32,
}
impl AttachmentHandle {
    pub fn index(self) -> u32 {
        self.index
    }
    pub fn generation(self) -> u32 {
        self.generation
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AttachmentPoint {
    pub particle: u32,
    /// Position in the parent rigid body's coordinate system.
    pub local_anchor: Vec3,
}

#[derive(Debug, Clone)]
pub struct AttachmentDesc {
    pub cloth: ClothHandle,
    pub body: RigidBodyHandle,
    pub points: Vec<AttachmentPoint>,
    pub compliance: Real,
    /// Only these collider/attached-particle pairs are excluded from contact.
    pub excluded_colliders: Vec<ColliderHandle>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AttachmentEventKind {
    Released,
    ClothRemoved,
    BodyRemoved,
    BodyDisabled,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AttachmentEvent {
    pub handle: AttachmentHandle,
    pub cloth: ClothHandle,
    pub body: RigidBodyHandle,
    pub kind: AttachmentEventKind,
}

#[derive(Debug, Clone)]
struct Slot {
    generation: u32,
    value: Option<AttachmentDesc>,
}

#[derive(Debug, Clone)]
pub(crate) struct Attachments {
    pub identity: u64,
    slots: Vec<Slot>,
}
static NEXT_ARENA: AtomicU64 = AtomicU64::new(1);
impl Attachments {
    pub fn new() -> Self {
        Self {
            identity: NEXT_ARENA
                .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |v| v.checked_add(1))
                .expect("attachment arena exhausted"),
            slots: vec![],
        }
    }
    pub fn insert(&mut self, desc: AttachmentDesc) -> AttachmentHandle {
        let index = self
            .slots
            .iter()
            .position(|s| s.value.is_none() && s.generation < u32::MAX)
            .unwrap_or(self.slots.len());
        if index == self.slots.len() {
            assert!(index < u32::MAX as usize, "attachment arena full");
            self.slots.push(Slot {
                generation: 0,
                value: None,
            });
        }
        self.slots[index].value = Some(desc);
        AttachmentHandle {
            arena: self.identity,
            index: index as u32,
            generation: self.slots[index].generation,
        }
    }
    pub fn get(&self, h: AttachmentHandle) -> Option<&AttachmentDesc> {
        if h.arena != self.identity {
            return None;
        }
        self.slots
            .get(h.index as usize)
            .filter(|s| s.generation == h.generation)
            .and_then(|s| s.value.as_ref())
    }
    pub fn remove(&mut self, h: AttachmentHandle) -> Option<AttachmentDesc> {
        self.get(h)?;
        let slot = &mut self.slots[h.index as usize];
        let value = slot.value.take();
        slot.generation = slot.generation.saturating_add(1);
        value
    }
    pub fn iter(&self) -> impl Iterator<Item = (AttachmentHandle, &AttachmentDesc)> {
        self.slots.iter().enumerate().filter_map(|(i, s)| {
            s.value.as_ref().map(|v| {
                (
                    AttachmentHandle {
                        arena: self.identity,
                        index: i as u32,
                        generation: s.generation,
                    },
                    v,
                )
            })
        })
    }
}
