mod support;

use rapier_cloth::Real;
use rapier_cloth::{rapier::prelude::*, *};
use support::TestWorld;

/// Closed triangle soup of two overlapping boxes: an L-shaped solid, the kind
/// of non-convex part a robot cell decomposes into convex hulls.
fn l_block() -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let boxes = [
        (Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.06, 0.02, 0.02)),
        (Vec3::new(0.04, 0.02, 0.0), Vec3::new(0.02, 0.04, 0.02)),
    ];
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for (center, half) in boxes {
        let base = vertices.len() as u32;
        for i in 0..8u32 {
            vertices.push(
                center
                    + Vec3::new(
                        if i & 1 == 0 { -half.x } else { half.x },
                        if i & 2 == 0 { -half.y } else { half.y },
                        if i & 4 == 0 { -half.z } else { half.z },
                    ),
            );
        }
        for [a, b, c, d] in [
            [0, 2, 3, 1],
            [4, 5, 7, 6],
            [0, 1, 5, 4],
            [2, 6, 7, 3],
            [0, 4, 6, 2],
            [1, 3, 7, 5],
        ] {
            indices.push([base + a, base + b, base + c]);
            indices.push([base + a, base + c, base + d]);
        }
    }
    (vertices, indices)
}

fn vhacd_compound() -> SharedShape {
    let (vertices, indices) = l_block();
    let shape = SharedShape::convex_decomposition(&vertices, &indices);
    let compound = shape
        .as_compound()
        .expect("convex decomposition is a compound");
    assert!(
        compound.shapes().len() >= 2,
        "{} parts",
        compound.shapes().len()
    );
    assert!(
        compound
            .shapes()
            .iter()
            .all(|(_, s)| s.as_convex_polyhedron().is_some())
    );
    shape
}

/// The construction a robot-cell collision layer uses: convex hulls of point
/// sets composed with local poses.
fn hull_pad_compound() -> SharedShape {
    let pad = |half: Vec3| -> Vec<Vec3> {
        (0..8)
            .map(|i| {
                Vec3::new(
                    if i & 1 == 0 { -half.x } else { half.x },
                    if i & 2 == 0 { -half.y } else { half.y },
                    if i & 4 == 0 { -half.z } else { half.z },
                )
            })
            .chain([Vec3::new(0.0, half.y * 1.3, 0.0)])
            .collect()
    };
    SharedShape::compound(vec![
        (
            Pose::from_parts(Vec3::new(-0.03, 0.0, 0.0), Rotation::from_rotation_z(0.15)),
            SharedShape::convex_hull(&pad(Vec3::new(0.012, 0.02, 0.025))).unwrap(),
        ),
        (
            Pose::from_parts(Vec3::new(0.03, 0.0, 0.0), Rotation::from_rotation_z(-0.15)),
            SharedShape::convex_hull(&pad(Vec3::new(0.012, 0.02, 0.025))).unwrap(),
        ),
    ])
}

fn sheet(origin: Vec3, size: Real, surface: bool, continuous: bool) -> Cloth {
    let mut cloth = Cloth::new(
        GridBuilder::new(8, 8)
            .size(size, size)
            .origin(origin)
            .build()
            .unwrap(),
        ClothMaterial {
            damping: 0.0,
            friction: 0.0,
            ..Default::default()
        },
    )
    .unwrap();
    if surface {
        cloth
            .set_contact_settings(Some(ClothContactSettings {
                self_collision: false,
                rigid_surface_collision: true,
                continuous_rigid_collision: continuous,
                static_friction: 0.0,
                kinetic_friction: 0.0,
                ..Default::default()
            }))
            .unwrap();
    }
    cloth
}

/// Distance from the solver's point of view: Parry's point projection onto the
/// compound, negative inside.
fn shape_gap(shape: &SharedShape, pose: &Pose, p: Vec3) -> Real {
    let projection = shape.project_point(pose, p, true);
    let d = projection.point.distance(p);
    if projection.is_inside { -d } else { d }
}

fn triangle(height: Real, continuous: bool) -> Cloth {
    let mut cloth = Cloth::new(
        ClothMesh::new(
            vec![
                Vec3::new(-0.4, height, -0.3),
                Vec3::new(0.4, height, -0.3),
                Vec3::new(0.0, height, 0.4),
            ],
            vec![[0, 2, 1]],
        )
        .unwrap(),
        ClothMaterial {
            damping: 0.0,
            friction: 0.0,
            ..Default::default()
        },
    )
    .unwrap();
    cloth
        .set_contact_settings(Some(ClothContactSettings {
            self_collision: false,
            rigid_surface_collision: true,
            continuous_rigid_collision: continuous,
            static_friction: 0.0,
            kinetic_friction: 0.0,
            ..Default::default()
        }))
        .unwrap();
    cloth
}

fn two_cube_compound() -> SharedShape {
    let corners: Vec<Vec3> = (0..8)
        .map(|i| {
            Vec3::new(
                if i & 1 == 0 { -0.02 } else { 0.02 },
                if i & 2 == 0 { -0.02 } else { 0.02 },
                if i & 4 == 0 { -0.02 } else { 0.02 },
            )
        })
        .collect();
    let hull = SharedShape::convex_hull(&corners).unwrap();
    SharedShape::compound(vec![
        (Pose::from_translation(Vec3::X * 0.05), hull.clone()),
        (Pose::from_translation(-Vec3::X * 0.05), hull),
    ])
}

#[test]
fn a_convex_decomposition_settles_a_resting_sheet_in_both_contact_modes() {
    for continuous in [false, true] {
        let shape = vhacd_compound();
        let mut world = TestWorld::new();
        world
            .rigid
            .colliders
            .insert(ColliderBuilder::new(shape.clone()).friction(0.0));
        // A small sheet 1.5 mm above the tall arm settles onto it.
        let handle = world.cloth.add_cloth(sheet(
            Vec3::new(0.025, 0.0615, -0.015),
            0.03,
            true,
            continuous,
        ));
        let mut touched = false;
        for step in 0..120 {
            let report = world
                .tick()
                .unwrap_or_else(|e| panic!("continuous={continuous}, step={step}: {e:?}"));
            touched |= report.cloths[0].1.surface_collision.retained_contacts > 0;
            let cloth = world.cloth.cloth(handle).unwrap();
            for &p in cloth.positions() {
                let gap = shape_gap(&shape, &Pose::IDENTITY, p);
                assert!(
                    gap >= 0.00045,
                    "continuous={continuous}, step={step}: gap {gap} at {p:?}"
                );
            }
        }
        assert!(touched, "continuous={continuous}");
        let cloth = world.cloth.cloth(handle).unwrap();
        // Resting: every vertex sits within a millimetre of the arm top.
        assert!(cloth.positions().iter().all(|p| p.y > 0.06 && p.y < 0.062));
    }
}

#[test]
fn single_triangle_crossings_are_stopped_by_hulls_and_compounds() {
    let hull = two_cube_compound().as_compound().unwrap().shapes()[0]
        .1
        .clone();
    for (name, shape) in [
        ("hull", hull),
        ("two-cube compound", two_cube_compound()),
        ("decomposition", vhacd_compound()),
    ] {
        for continuous in [false, true] {
            let mut world = TestWorld::new();
            world.rigid.gravity = Vec3::ZERO;
            world
                .rigid
                .colliders
                .insert(ColliderBuilder::new(shape.clone()).friction(0.0));
            let aabb = shape.compute_aabb(&Pose::IDENTITY);
            let (top, bottom) = (aabb.maxs.y, aabb.mins.y);
            let mut cloth = triangle(top + 0.02, continuous);
            // One substep carries the triangle from 2 cm above the shape to
            // 2 cm below it, so a discrete end pose is outside the shape and
            // only the crossing itself goes unnoticed.
            let travel = top - bottom + 0.04;
            for i in 0..3 {
                cloth
                    .set_velocity(i, -Vec3::Y * (travel / world.h))
                    .unwrap();
            }
            let handle = world.cloth.add_cloth(cloth);
            let report = world
                .tick()
                .unwrap_or_else(|e| panic!("{name}, CCD={continuous}: {e:?}"));
            let cloth = world.cloth.cloth(handle).unwrap();
            if continuous {
                assert!(
                    report.cloths[0].1.surface_collision.ccd_checks > 0,
                    "{name}"
                );
                assert!(
                    report.cloths[0].1.surface_collision.limited_advances > 0,
                    "{name}"
                );
                for &p in cloth.positions() {
                    let gap = shape_gap(&shape, &Pose::IDENTITY, p);
                    assert!(gap >= 0.00045, "{name}: gap {gap} at {p:?}");
                }
                assert!(cloth.positions().iter().map(|p| p.y).sum::<Real>() > 3.0 * (top - 0.06));
            } else {
                // Discrete queries cannot stop a whole-thickness crossing in one substep.
                assert!(
                    cloth.positions().iter().all(|p| p.y < bottom - 0.01),
                    "{name}"
                );
            }
        }
    }
}

#[test]
fn a_moving_hull_compound_pushes_the_sheet_and_keeps_its_clearance() {
    for (name, shape) in [("vhacd", vhacd_compound()), ("pads", hull_pad_compound())] {
        for rotating in [false, true] {
            let mut world = TestWorld::new();
            world.rigid.gravity = Vec3::ZERO;
            let mut builder = RigidBodyBuilder::kinematic_velocity_based().linvel(Vec3::Y * 0.02);
            if rotating {
                builder = builder.angvel(Vec3::Z * (0.003 / world.h));
            }
            let body = world.rigid.bodies.insert(builder);
            world.rigid.colliders.insert_with_parent(
                ColliderBuilder::new(shape.clone()).friction(0.0),
                body,
                &mut world.rigid.bodies,
            );
            // Start 1.5 mm above the part; it reaches the sheet within a few steps.
            let top = shape.compute_aabb(&Pose::IDENTITY).maxs.y;
            let handle = world.cloth.add_cloth(sheet(
                Vec3::new(-0.05, top + 0.0015, -0.05),
                0.1,
                true,
                true,
            ));
            let mut touched = false;
            for step in 0..48 {
                let report = world
                    .tick()
                    .unwrap_or_else(|e| panic!("{name}, rotating={rotating}, step={step}: {e:?}"));
                touched |= report.cloths[0].1.surface_collision.retained_contacts > 0;
                let pose = *world.rigid.bodies[body].position();
                let cloth = world.cloth.cloth(handle).unwrap();
                for &p in cloth.positions() {
                    let gap = shape_gap(&shape, &pose, p);
                    assert!(
                        gap >= 0.00045,
                        "{name}, rotating={rotating}, step={step}: gap {gap}"
                    );
                }
            }
            assert!(touched, "{name}, rotating={rotating}");
            let cloth = world.cloth.cloth(handle).unwrap();
            assert!(
                cloth.velocities().iter().any(|v| v.y > 0.005),
                "{name}, rotating={rotating}"
            );
        }
    }
}

#[test]
fn particle_contacts_and_static_sweeps_accept_compounds() {
    let shape = vhacd_compound();
    let mut world = TestWorld::new();
    world
        .rigid
        .colliders
        .insert(ColliderBuilder::new(shape.clone()).friction(0.0));
    let handle = world
        .cloth
        .add_cloth(sheet(Vec3::new(-0.02, 0.075, -0.05), 0.1, false, false));
    for step in 0..120 {
        world
            .tick()
            .unwrap_or_else(|e| panic!("step={step}: {e:?}"));
    }
    let cloth = world.cloth.cloth(handle).unwrap();
    for &p in cloth.positions() {
        // Particle contacts keep the vertex radius outside the decomposition.
        assert!(shape_gap(&shape, &Pose::IDENTITY, p) > -0.001, "{p:?}");
    }
    assert!(cloth.positions().iter().any(|p| p.y > 0.05));
}

#[test]
fn triangle_meshes_are_still_reported_as_unsupported() {
    // Parry forbids nested composite shapes, so a mesh can only appear directly.
    let (vertices, indices) = l_block();
    let shape = SharedShape::trimesh(vertices, indices).unwrap();
    for surface in [false, true] {
        let mut world = TestWorld::new();
        world
            .rigid
            .colliders
            .insert(ColliderBuilder::new(shape.clone()));
        world
            .cloth
            .add_cloth(sheet(Vec3::new(-0.02, 0.03, -0.05), 0.1, surface, surface));
        let error = world.tick().unwrap_err();
        match error {
            IntegrationError::UnsupportedCollision { reason, .. } => {
                assert!(
                    reason.starts_with("shape outside"),
                    "surface={surface}: {reason}"
                )
            }
            other => panic!("surface={surface}: {other:?}"),
        }
    }
}

/// The L-block as a compound of its two exact boxes.
fn exact_l_compound() -> SharedShape {
    SharedShape::compound(vec![
        (Pose::IDENTITY, SharedShape::cuboid(0.06, 0.02, 0.02)),
        (
            Pose::from_translation(Vec3::new(0.04, 0.02, 0.0)),
            SharedShape::cuboid(0.02, 0.04, 0.02),
        ),
    ])
}

/// A 10 cm sheet released 1.5 cm above the 4 cm arm top lands at 0.5 m/s and
/// drapes over the arm's edges. Triangles pivoting about the top edge used to
/// lose their swept clearance batch by batch until the continuous path failed
/// with `UnresolvedContinuousCollision`; sliding past an edge also failed the
/// former straight-chord sweep of each substep.
#[test]
fn a_free_falling_sheet_lands_across_an_arm_edge_and_drapes() {
    for (name, shape, pose) in [
        (
            "box",
            SharedShape::cuboid(0.02, 0.04, 0.02),
            Pose::from_translation(Vec3::new(0.04, 0.02, 0.0)),
        ),
        ("two boxes", exact_l_compound(), Pose::IDENTITY),
        ("decomposition", vhacd_compound(), Pose::IDENTITY),
    ] {
        let mut world = TestWorld::new();
        world.rigid.colliders.insert(
            ColliderBuilder::new(shape.clone())
                .position(pose)
                .friction(0.0),
        );
        let handle = world
            .cloth
            .add_cloth(sheet(Vec3::new(-0.02, 0.075, -0.05), 0.1, true, true));
        let mut limited = 0;
        let mut draped = false;
        let mut stretch = 0.0;
        for step in 0..240 {
            let report = world
                .tick()
                .unwrap_or_else(|e| panic!("{name}, step {step}: {e:?}"));
            limited += report.cloths[0].1.surface_collision.limited_advances;
            let cloth = world.cloth.cloth(handle).unwrap();
            for &p in cloth.positions() {
                let gap = shape_gap(&shape, &pose, p);
                assert!(gap >= 0.00045, "{name}, step {step}: gap {gap} at {p:?}");
            }
            // The sheet stays whole: the impact on the edge stretches it by a
            // few percent for a few substeps, while a contact that holds a
            // vertex back as its neighbours fall tears the coarse mesh apart.
            stretch = cloth
                .mesh()
                .edges()
                .iter()
                .map(|e| {
                    cloth.positions()[e.vertices[0] as usize]
                        .distance(cloth.positions()[e.vertices[1] as usize])
                        / e.rest_length
                })
                .fold(0.0, Real::max);
            assert!(stretch < 1.15, "{name}, step {step}: stretch {stretch}");
            draped |= cloth.positions().iter().any(|p| p.y > 0.0595)
                && cloth.positions().iter().any(|p| p.y < 0.04);
        }
        assert!(limited > 0, "{name}: the impact never shortened a batch");
        assert!(stretch < 1.01, "{name}: final stretch {stretch}");
        // The frictionless sheet hung across the arm edge (top at 0.06) at
        // some point before sliding on.
        assert!(draped, "{name}: the sheet never hung across the edge");
    }
}

/// One implicit step drapes the overhanging part of a sheet over the arm's
/// edges (a 1.2 cm fall at 0.05 s, 4.9 cm at 0.1 s), and the following steps
/// lay it onto the lower block or let it slide off the bare arm. The straight
/// chord of a vertex rounding the edge cuts the corner, which the former final
/// rigid sweep rejected at the first step; the clipped contact manifolds of
/// faceted parts made the line search fail at 0.1 s.
#[cfg(feature = "implicit")]
#[test]
fn the_implicit_solver_drapes_a_sheet_over_an_edge_within_one_step() {
    for (name, shape, pose, h) in [
        ("decomposition", vhacd_compound(), Pose::IDENTITY, 0.05),
        ("decomposition", vhacd_compound(), Pose::IDENTITY, 0.1),
        (
            "box",
            SharedShape::cuboid(0.02, 0.04, 0.02),
            Pose::from_translation(Vec3::new(0.04, 0.02, 0.0)),
            0.1,
        ),
    ] {
        let mut world = TestWorld::new();
        world.h = h;
        world.cloth.solver_settings.max_substep = h;
        world.rigid.colliders.insert(
            ColliderBuilder::new(shape.clone())
                .position(pose)
                .friction(0.0),
        );
        let mut cloth = sheet(Vec3::new(-0.02, 0.063, -0.05), 0.1, true, true);
        // A 1 mm barrier band (three times the towel example's, which only
        // meets a flat floor) lets the barrier steer the coarse 14 mm triangles
        // around the edge before continuous collision has to cut the Newton
        // steps short.
        let mut contact = cloth.contact_settings().unwrap();
        contact.activation_margin = 0.001;
        cloth.set_contact_settings(Some(contact)).unwrap();
        cloth
            .set_implicit_solver(Some(ImplicitSettings::default()))
            .unwrap();
        let handle = world.cloth.add_cloth(cloth);
        for step in 0..4 {
            let report = world
                .tick()
                .unwrap_or_else(|e| panic!("{name}, h={h}, step {step}: {e:?}"));
            assert!(
                report.cloths[0].1.implicit.is_some_and(|o| o.converged),
                "{name}, h={h}, step {step}"
            );
            let cloth = world.cloth.cloth(handle).unwrap();
            for &p in cloth.positions() {
                let gap = shape_gap(&shape, &pose, p);
                assert!(gap >= 0.00045, "{name}, h={h}, step {step}: gap {gap}");
            }
            if step == 0 {
                // The overhang already hangs below the arm top after one step.
                assert!(
                    cloth.positions().iter().any(|p| p.y < 0.055),
                    "{name}, h={h}: no vertex rounded the edge in the first step"
                );
            }
        }
        if name == "decomposition" {
            // Part of the sheet lies on the lower block, part still covers the
            // arm top.
            let cloth = world.cloth.cloth(handle).unwrap();
            assert!(cloth.positions().iter().any(|p| p.y < 0.03), "h={h}");
            assert!(cloth.positions().iter().any(|p| p.y > 0.059), "h={h}");
        }
    }
}

/// A kinematic box reaches resting cloth within one implicit step and pushes
/// it: sideways along the table and down onto it. Cloth that does not move
/// cannot be certified against an obstacle arriving through it, so the step
/// seeds the cloth ahead of the obstacle from the physical sweep's witnesses.
#[cfg(feature = "implicit")]
#[test]
fn the_implicit_solver_lets_an_arriving_box_push_resting_cloth() {
    let shape = SharedShape::cuboid(0.02, 0.02, 0.02);
    // (name, start, velocity, steps, steps before the box stops)
    for (name, start, velocity, steps, moving) in [
        // The box face starts 1 mm from the sheet's edge and moves 1 cm per
        // step (0.1 m/s), far beyond the kinematic motion budget, which does
        // not apply while every cloth sweeps its rigid contacts.
        (
            "side",
            Vec3::new(0.071, 0.0205, 0.0),
            Vec3::new(-0.1, 0.0, 0.0),
            6,
            6,
        ),
        // The box bottom starts 0.8 mm above the sheet's midsurface, 0.3 mm
        // beyond the contact separation, descends 0.5 mm in the first step
        // and squeezes the sheet against the table, then stops.
        (
            "press",
            Vec3::new(0.0, 0.0223, 0.0),
            Vec3::new(0.0, -0.005, 0.0),
            3,
            1,
        ),
    ] {
        let mut world = TestWorld::new();
        world.h = 0.1;
        world.cloth.solver_settings.max_substep = 0.1;
        world.rigid.colliders.insert(
            ColliderBuilder::new(SharedShape::halfspace(Vec3::Y))
                .translation(Vec3::Y * 0.0005)
                .friction(0.3),
        );
        let body = world.rigid.bodies.insert(
            RigidBodyBuilder::kinematic_velocity_based()
                .translation(start)
                .linvel(velocity),
        );
        world.rigid.colliders.insert_with_parent(
            ColliderBuilder::new(shape.clone()).friction(0.3),
            body,
            &mut world.rigid.bodies,
        );
        // Lying on the table: the midsurface is half a thickness plus half the
        // barrier band above it.
        let mut cloth = sheet(Vec3::new(-0.05, 0.0015, -0.05), 0.1, true, true);
        let mut contact = cloth.contact_settings().unwrap();
        contact.activation_margin = 0.001;
        cloth.set_contact_settings(Some(contact)).unwrap();
        cloth
            .set_implicit_solver(Some(ImplicitSettings::default()))
            .unwrap();
        let handle = world.cloth.add_cloth(cloth);
        let edge_start = world
            .cloth
            .cloth(handle)
            .unwrap()
            .positions()
            .iter()
            .map(|p| p.x)
            .fold(-Real::INFINITY, Real::max);
        for step in 0..steps {
            if step == moving {
                world.rigid.bodies[body].set_linvel(Vec3::ZERO, true);
            }
            let report = world
                .tick()
                .unwrap_or_else(|e| panic!("{name}, step {step}: {e:?}"));
            assert!(
                report.cloths[0].1.implicit.is_some_and(|o| o.converged),
                "{name}, step {step}"
            );
            let pose = *world.rigid.bodies[body].position();
            let cloth = world.cloth.cloth(handle).unwrap();
            for &p in cloth.positions() {
                let gap = shape_gap(&shape, &pose, p);
                assert!(gap >= 0.00045, "{name}, step {step}: box gap {gap}");
                assert!(p.y >= 0.00095, "{name}, step {step}: table gap {}", p.y);
            }
        }
        let cloth = world.cloth.cloth(handle).unwrap();
        let pose = *world.rigid.bodies[body].position();
        if name == "side" {
            // The box advanced 6 cm, 5 cm of it into the sheet's footprint,
            // and moved the edge in front of it; the strip beside the box
            // stays in place.
            let pushed = cloth
                .positions()
                .iter()
                .filter(|p| p.z.abs() < 0.015)
                .map(|p| p.x)
                .fold(-Real::INFINITY, Real::max);
            assert!(pushed < edge_start - 0.045, "{name}: edge at {pushed}");
            assert!(pushed < pose.translation.x - 0.02, "{name}");
        } else {
            // Squeezed between the box bottom (at 0.0018) and the table.
            let under = cloth
                .positions()
                .iter()
                .filter(|p| p.x.abs() < 0.015 && p.z.abs() < 0.015)
                .map(|p| p.y)
                .fold(-Real::INFINITY, Real::max);
            assert!(under < 0.0013, "{name}: sheet under the box at {under}");
        }
    }
}

#[cfg(feature = "implicit")]
#[test]
fn the_implicit_solver_lands_on_a_decomposition_and_takes_a_hull_press() {
    let table = vhacd_compound();
    let mut world = TestWorld::new();
    world.h = 0.1;
    world.rigid.integration_parameters.dt = 0.1;
    world.cloth.solver_settings.max_substep = 0.1;
    world
        .rigid
        .colliders
        .insert(ColliderBuilder::new(table.clone()).friction(0.3));
    // A flat-bottomed convex-hull finger pad (a chamfered block) on a
    // kinematic body, parked above the arm.
    let pad_points: Vec<Vec3> = (0..8)
        .map(|i| {
            Vec3::new(
                if i & 1 == 0 { -0.015 } else { 0.015 },
                if i & 2 == 0 { -0.01 } else { 0.01 },
                if i & 4 == 0 { -0.015 } else { 0.015 },
            )
        })
        .chain((0..4).map(|i| {
            Vec3::new(
                if i & 1 == 0 { -0.02 } else { 0.02 },
                0.004,
                if i & 2 == 0 { -0.02 } else { 0.02 },
            )
        }))
        .collect();
    let pad = SharedShape::convex_hull(&pad_points).unwrap();
    // Parked 26 mm above the arm, clear of the sheet's start height; the
    // kinematic motion budget bounds each 0.1 s step to a few millimetres.
    let bottom_offset = 0.010;
    let parked = 0.06 + bottom_offset + 0.026;
    let body = world.rigid.bodies.insert(
        RigidBodyBuilder::kinematic_position_based().translation(Vec3::new(0.04, parked, 0.0)),
    );
    world.rigid.colliders.insert_with_parent(
        ColliderBuilder::new(pad.clone()).friction(0.3),
        body,
        &mut world.rigid.bodies,
    );
    let mut cloth = sheet(Vec3::new(0.025, 0.08, -0.015), 0.03, true, true);
    cloth
        .set_implicit_solver(Some(ImplicitSettings::default()))
        .unwrap();
    let handle = world.cloth.add_cloth(cloth);
    // Five 0.1 s steps: the sheet falls two centimetres onto the arm.
    for step in 0..5 {
        let report = world
            .tick()
            .unwrap_or_else(|e| panic!("landing step {step}: {e:?}"));
        assert!(report.cloths[0].1.implicit.is_some_and(|o| o.converged));
        let cloth = world.cloth.cloth(handle).unwrap();
        for &p in cloth.positions() {
            let gap = shape_gap(&table, &Pose::IDENTITY, p);
            assert!(gap >= 0.00045, "landing step {step}: gap {gap} at {p:?}");
        }
    }
    let landed = world.cloth.cloth(handle).unwrap();
    assert!(landed.positions().iter().all(|p| p.y > 0.06 && p.y < 0.063));
    // The pad descends two millimetres per step onto the sheet and stops
    // 1.5 mm above the arm (the sheet's mid-surface rests 0.5 mm above it and
    // keeps 0.5 mm to the pad): pressed, never crossed.
    let descents: Vec<Real> = (1..=12)
        .map(|k| 0.002 * k as Real)
        .chain([0.0245])
        .collect();
    for (step, descent) in descents.iter().enumerate() {
        world.rigid.bodies[body].set_next_kinematic_position(Pose::from_translation(Vec3::new(
            0.04,
            parked - descent,
            0.0,
        )));
        let report = world
            .tick()
            .unwrap_or_else(|e| panic!("press step {step}: {e:?}"));
        assert!(report.cloths[0].1.implicit.is_some_and(|o| o.converged));
        let pose = *world.rigid.bodies[body].position();
        assert!(pose.translation.y - bottom_offset - 0.06 >= 0.0014);
        let cloth = world.cloth.cloth(handle).unwrap();
        for &p in cloth.positions() {
            let to_table = shape_gap(&table, &Pose::IDENTITY, p);
            let to_pad = shape_gap(&pad, &pose, p);
            assert!(
                to_table >= 0.00045,
                "press step {step}: table gap {to_table}"
            );
            assert!(to_pad >= 0.00045, "press step {step}: pad gap {to_pad}");
        }
    }
    // The pressed region sits within the pad's footprint and touches both sides.
    let pose = *world.rigid.bodies[body].position();
    let pressed = world.cloth.cloth(handle).unwrap();
    let squeezed = pressed
        .positions()
        .iter()
        .filter(|p| (p.x - 0.04).abs() < 0.01 && p.z.abs() < 0.01)
        .filter(|&&p| shape_gap(&pad, &pose, p) < 0.0015)
        .count();
    assert!(squeezed > 0);
}
