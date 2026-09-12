mod support;
use rapier::prelude::*;
use rapier_cloth::*;
use support::TestWorld;

#[test]
fn particle_sweep_catches_thin_box_crossing() {
    let mut ends = vec![];
    for sweep in [false, true] {
        let mut w = TestWorld::new();
        w.rigid.gravity = Vec3::ZERO;
        w.cloth.collision_settings.static_sweep = sweep;
        w.rigid
            .colliders
            .insert(ColliderBuilder::cuboid(2.0, 0.002, 2.0));
        let h = w.grid(2, 0.1, Vec3::Y * 0.2);
        for i in 0..4 {
            w.cloth
                .cloth_mut(h)
                .unwrap()
                .set_velocity(i, -Vec3::Y * 100.0)
                .unwrap();
        }
        w.tick().unwrap();
        ends.push(w.cloth.cloth(h).unwrap().positions()[0].y);
        if sweep {
            assert!(w.cloth.cloth(h).unwrap().velocities()[0].y >= -1.0e-4);
        }
    }
    assert!(ends[0] < -0.1, "discrete comparison should cross: {ends:?}");
    assert!(ends[1] >= 0.0069, "sweep should stop above box: {ends:?}");
}
