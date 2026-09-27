//! Garment meshes under the implicit solver: a single-panel and a sewn T-shirt
//! lie on a table without stretching, crossing themselves or drifting.
#![cfg(all(feature = "implicit", feature = "f64"))]
#[path = "../examples/support/folding_oracle.rs"]
mod oracle;
mod support;

use rapier_cloth::Real;
use rapier_cloth::core::garment::{GarmentLayers, Landmark, Layer, Side, TShirtPattern};
use rapier_cloth::{rapier::prelude::*, *};
use support::TestWorld;

/// The towel task's shell: 0.318 mm thick cloth, a barrier band of one
/// thickness, mu 0.5, on a table whose top is half a thickness above y = 0.
const THICKNESS: Real = 0.000318;
const MU: Real = 0.5;

fn table_world() -> TestWorld {
    let mut world = TestWorld::new();
    world.h = 0.1;
    world.cloth.solver_settings.max_substep = 0.1;
    world.rigid.gravity = Vec3::new(0.0, -9.81, 0.0);
    world.rigid.colliders.insert(
        ColliderBuilder::new(SharedShape::halfspace(Vec3::Y))
            .translation(Vec3::Y * (THICKNESS * 0.5))
            .friction(MU),
    );
    world
}

/// Places the garment so its lowest layer rests where the towel rests: the
/// midsurface half a thickness plus one band above the table top. Sewn
/// layers start one thickness plus one band apart, free of contact force.
fn add_shirt(world: &mut TestWorld, pattern: TShirtPattern) -> (ClothHandle, Garment) {
    let garment = pattern.build().unwrap();
    let mut cloth = Cloth::new(
        garment.mesh().clone(),
        ClothMaterial {
            surface_density: 0.1503,
            damping: 0.0,
            friction: MU,
            contact_radius: THICKNESS * 0.5,
            ..Default::default()
        },
    )
    .unwrap();
    let gap = 2.0 * THICKNESS;
    let lowest = 2.0 * THICKNESS;
    let origin = match pattern.layers {
        GarmentLayers::Single => Vec3::Y * lowest,
        GarmentLayers::Sewn => Vec3::Y * (lowest + gap * 0.5),
    };
    cloth
        .set_positions(&garment.placed_positions(origin, gap).unwrap())
        .unwrap();
    cloth
        .set_contact_settings(Some(ClothContactSettings {
            thickness: THICKNESS,
            activation_margin: THICKNESS,
            static_friction: MU,
            kinetic_friction: MU,
            self_collision: true,
            continuous_self_collision: true,
            rigid_surface_collision: true,
            continuous_rigid_collision: true,
            limits: CollisionLimits {
                candidate_pairs: 10_000_000,
                ccd_checks: 10_000_000,
                ..Default::default()
            },
        }))
        .unwrap();
    cloth
        .set_implicit_solver(Some(ImplicitSettings::default()))
        .unwrap();
    (world.cloth.add_cloth(cloth), garment)
}

fn points(positions: &[Vec3]) -> Vec<[f64; 3]> {
    positions.iter().map(|p| [p.x, p.y, p.z]).collect()
}

fn max_speed(cloth: &Cloth) -> Real {
    cloth
        .velocities()
        .iter()
        .map(|v| v.length())
        .fold(0.0, Real::max)
}

#[test]
fn a_single_panel_shirt_rests_on_the_table() {
    let mut world = table_world();
    let pattern = TShirtPattern {
        spacing: 0.04,
        layers: GarmentLayers::Single,
        ..TShirtPattern::default()
    };
    let (handle, garment) = add_shirt(&mut world, pattern);
    let start = world.cloth.cloth(handle).unwrap().positions().to_vec();
    for step in 0..10 {
        let report = world
            .tick()
            .unwrap_or_else(|e| panic!("step {step}: {e:?}"));
        let (_, cloth_report) = &report.cloths[0];
        assert!(
            cloth_report.implicit.is_some_and(|o| o.converged),
            "step {step}: {:?}",
            cloth_report.implicit
        );
        assert!(
            cloth_report.max_stretch < 1.01,
            "step {step}: stretch {}",
            cloth_report.max_stretch
        );
    }
    let cloth = world.cloth.cloth(handle).unwrap();
    // On the table, within the band above it, and not slid: the sheet only
    // drops its 0.3 mm of initial clearance.
    for (p, s) in cloth.positions().iter().zip(&start) {
        assert!(
            p.y >= THICKNESS * 0.9,
            "vertex below the table clearance: {p:?}"
        );
        assert!(p.y <= s.y + 1e-6, "vertex rose: {p:?}");
        assert!(
            (p.x - s.x).abs() < 1e-3 && (p.z - s.z).abs() < 1e-3,
            "{p:?} slid from {s:?}"
        );
    }
    assert!(max_speed(cloth) < 0.005, "max speed {}", max_speed(cloth));
    let audit = oracle::audit_surface(
        &points(cloth.positions()),
        garment.mesh().triangles(),
        THICKNESS,
    );
    assert_eq!(audit.crossing_pairs, 0);
    assert!(audit.tested_pairs > 0);
}

#[test]
fn a_sewn_shirt_keeps_its_layers_apart_on_the_table() {
    let mut world = table_world();
    let pattern = TShirtPattern {
        spacing: 0.05,
        ..TShirtPattern::default()
    };
    let (handle, garment) = add_shirt(&mut world, pattern);
    let start = world.cloth.cloth(handle).unwrap().positions().to_vec();
    let mut contacts = 0;
    for step in 0..10 {
        let report = world
            .tick()
            .unwrap_or_else(|e| panic!("step {step}: {e:?}"));
        let (_, cloth_report) = &report.cloths[0];
        assert!(
            cloth_report.implicit.is_some_and(|o| o.converged),
            "step {step}: {:?}",
            cloth_report.implicit
        );
        assert!(
            cloth_report.max_stretch < 1.01,
            "step {step}: stretch {}",
            cloth_report.max_stretch
        );
        contacts = contacts.max(cloth_report.contacts);
    }
    let cloth = world.cloth.cloth(handle).unwrap();
    let positions = cloth.positions();
    assert!(max_speed(cloth) < 0.005, "max speed {}", max_speed(cloth));
    // The front panel stays above the back panel by at least most of a
    // thickness at every cell both panels have; seams stay between them.
    let cells = garment.cells();
    let mut compared = 0;
    for info in garment.vertices() {
        if info.layer != Layer::Back {
            continue;
        }
        let Some(front) = garment.vertex(Layer::Front, info.cell[0], info.cell[1]) else {
            continue;
        };
        let back = garment
            .vertex(Layer::Back, info.cell[0], info.cell[1])
            .unwrap();
        let separation = positions[front as usize].y - positions[back as usize].y;
        assert!(
            separation > 0.8 * THICKNESS,
            "cell {:?}: layers {separation} apart",
            info.cell
        );
        compared += 1;
    }
    assert!(compared > cells.body[0] * cells.body[1] / 2);
    for (p, s) in positions.iter().zip(&start) {
        assert!(
            p.y >= THICKNESS * 0.9,
            "vertex below the table clearance: {p:?}"
        );
        assert!(
            (p.x - s.x).abs() < 2e-3 && (p.z - s.z).abs() < 2e-3,
            "{p:?} slid from {s:?}"
        );
    }
    // Landmarks are usable as grasp points: they exist on the expected layers.
    let seam = garment.landmark(Landmark::CuffTop(Side::Left)).unwrap();
    assert_eq!(garment.vertices()[seam as usize].layer, Layer::Seam);
    let hem = garment.landmark(Landmark::HemCenter).unwrap();
    assert_eq!(garment.vertices()[hem as usize].layer, Layer::Front);
    let audit = oracle::audit_surface(&points(positions), garment.mesh().triangles(), THICKNESS);
    assert_eq!(audit.crossing_pairs, 0);
    assert!(
        audit.max_separation_deficit < THICKNESS * 0.3,
        "layers closer than the thickness by {}",
        audit.max_separation_deficit
    );
    assert!(contacts > 0, "stacked layers report self-contacts");
}
