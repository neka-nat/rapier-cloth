//! The T-shirt folding task on a coarse sewn shirt: sleeves in, hem to the
//! shoulders, released and settled, with the garment gates.
#![cfg(all(feature = "implicit", feature = "f64"))]
#[path = "../examples/support/folding_oracle.rs"]
mod folding_oracle;
#[path = "../examples/support/garment_task.rs"]
#[allow(dead_code)]
mod garment_task;

use garment_task::{ShirtTask, ShirtTaskConfig};
use rapier_cloth::core::garment::{GarmentLayers, NeckShape, SeamJoin, TShirtPattern};

fn coarse(layers: GarmentLayers) -> ShirtTaskConfig {
    ShirtTaskConfig {
        pattern: TShirtPattern {
            spacing: 0.05,
            layers,
            ..TShirtPattern::default()
        },
        ..ShirtTaskConfig::default()
    }
}

#[test]
fn a_coarse_sewn_shirt_folds_into_a_quarter_of_its_footprint() {
    let mut task = ShirtTask::new(coarse(GarmentLayers::Sewn)).unwrap();
    let schedule = task.schedule();
    let summary = task.run().unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(summary.steps, schedule.end);
    assert_eq!(summary.approximate_steps, 0);
    assert!(summary.max_edge_extension <= 0.03, "{summary:?}");
    assert!(summary.settled, "{summary:?}");
    assert!(summary.footprint_ratio <= 0.45, "{summary:?}");
    assert!(summary.cuffs_over_body.iter().all(|&c| c), "{summary:?}");
    assert!(
        summary
            .hem_short_of_shoulders
            .iter()
            .all(|d| d.abs() <= 0.08),
        "{summary:?}"
    );
    assert_eq!(summary.crossing_pairs, 0, "{summary:?}");
    assert!(summary.passed, "{summary:?}");
    // The recording is complete and consistent with the run.
    let recording = task.recording(&summary);
    assert_eq!(recording["schema_version"], 2);
    assert_eq!(recording["outcome"]["stop_reason"], "completed");
    assert_eq!(
        recording["frames"].as_array().unwrap().len(),
        schedule.end + 1
    );
    assert_eq!(recording["summary"]["passed"], true);
}

#[test]
fn a_round_neck_woven_shirt_with_stiff_seams_folds_too() {
    let base = coarse(GarmentLayers::Sewn);
    let config = ShirtTaskConfig {
        pattern: TShirtPattern {
            neck: NeckShape::Round,
            seam_stiffness: 3.0,
            ..base.pattern
        },
        warp_stiffness: 4.0e5,
        weft_stiffness: 2.0e5,
        ..base
    };
    let mut task = ShirtTask::new(config).unwrap();
    let summary = task.run().unwrap_or_else(|e| panic!("{e}"));
    assert!(summary.passed, "{summary:?}");
}

#[test]
fn a_coarse_stitched_shirt_folds_too() {
    let base = coarse(GarmentLayers::Sewn);
    let config = ShirtTaskConfig {
        pattern: TShirtPattern {
            seams: SeamJoin::Stitched,
            // The task places the layers thickness + band apart.
            stitch_length: base.thickness + base.band,
            ..base.pattern
        },
        ..base
    };
    let mut task = ShirtTask::new(config).unwrap();
    let summary = task.run().unwrap_or_else(|e| panic!("{e}"));
    assert!(summary.passed, "{summary:?}");
}

#[test]
fn a_coarse_single_panel_shirt_folds_too() {
    // A coarse single sheet lands unevenly and creeps with the default hem
    // compliance; a stiffer grasp settles it (see docs/garments.md).
    let config = ShirtTaskConfig {
        hem_compliance: 0.01,
        ..coarse(GarmentLayers::Single)
    };
    let mut task = ShirtTask::new(config).unwrap();
    let summary = task.run().unwrap_or_else(|e| panic!("{e}"));
    assert!(summary.passed, "{summary:?}");
}
