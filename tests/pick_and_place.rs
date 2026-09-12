#[path = "../examples/support/recording.rs"]
mod recording;
use rapier_cloth::Real;
use recording::*;
use serde::Deserialize;
#[derive(Deserialize)]
struct Acceptance {
    release_time: f64,
    release_velocity: [Real; 3],
    lifted_y: Real,
    transported_distance: Real,
    final_max_y: Real,
    min_final_center_x: Real,
}
#[test]
fn canonical_pick_and_place_meets_motion_and_recording_contracts() {
    let acceptance: Acceptance =
        serde_json::from_str(include_str!("fixtures/pick_acceptance.json")).unwrap();
    let (record, summary) = simulate(Config::default()).unwrap();
    println!("{summary:?}");
    assert!(summary.finite);
    assert!((summary.release_time - acceptance.release_time).abs() < 1.0e-6);
    assert!((summary.lifted_y - acceptance.lifted_y).abs() < 1.0e-5);
    assert!(
        (summary.transported_anchor_x
            - record
                .frames
                .iter()
                .find(|f| f.step == record.config.attach_step)
                .unwrap()
                .positions[0][0]
            - acceptance.transported_distance)
            .abs()
            < 1.0e-5
    );
    for v in &summary.release_velocities {
        for (actual, expected) in v.iter().zip(acceptance.release_velocity) {
            assert!((*actual - expected).abs() < 1.0e-4, "{v:?}");
        }
    }
    assert!(
        (summary.final_max_y) < acceptance.final_max_y,
        "{summary:?}"
    );
    assert!(
        (summary.final_center[0]) > acceptance.min_final_center_x,
        "{summary:?}"
    );
    assert!(summary.max_penetration <= record.config.contact_radius * 0.2);
    let encoded = serde_json::to_vec(&record).unwrap();
    let decoded: Recording = serde_json::from_slice(&encoded).unwrap();
    assert_eq!(decoded.schema_version, 1);
    assert_eq!(decoded.triangles.len(), 2 * (record.config.grid - 1).pow(2));
    for (a, b) in record.frames.iter().zip(&decoded.frames) {
        assert_eq!(a.positions.len(), record.config.grid.pow(2));
        assert_eq!(a.positions, b.positions);
        assert_eq!(a.time, b.time);
        assert!(a.positions.iter().flatten().all(|x| x.is_finite()));
    }
    assert_eq!(record.frames.last().unwrap().step, record.config.end_step);
    let release = record
        .frames
        .iter()
        .find(|f| f.step == record.config.release_step)
        .unwrap();
    assert!(release.attached_particles.is_empty());
    println!("{}", serde_json::to_string(&summary).unwrap());
}
