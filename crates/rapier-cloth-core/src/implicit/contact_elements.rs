//! Evaluate independent parent contacts in parallel, then consume results in
//! their original order. No worker accumulates into the global objective/system.
use super::*;

pub(super) type Slot = Option<Result<primitive::Term, ClothError>>;

/// Fills `results` with the parent contacts' terms in contact order. Returns
/// false, with `results` empty, when the caller evaluates the batch serially.
pub(super) fn evaluate(
    model: &Model,
    x: &[Vec3],
    contacts: &[SurfaceContact],
    results: &mut Vec<Slot>,
) -> Result<bool, ClothError> {
    results.clear();
    if model.implicit.execution != ImplicitExecution::Parallel4
        || contacts.iter().filter(|c| primitive::is_parent(c)).count() < 1024
    {
        return Ok(false);
    }
    let parents: Vec<_> = contacts
        .iter()
        .filter(|c| primitive::is_parent(c))
        .collect();
    results.resize_with(parents.len(), || None);
    let fill = |contacts: &[&SurfaceContact], slots: &mut [Slot]| {
        for (contact, slot) in contacts.iter().zip(slots) {
            // Retain errors in their input slots: report the first numerical
            // failure in contact order, independent of worker completion order.
            // Energy-only trials ignore the curvature fields.
            *slot = Some(primitive::gauss_newton(
                contact,
                x,
                model.cloth.mesh.rest_positions(),
                model.band,
                model.barrier_stiffness,
            ));
        }
    };
    let (n, lanes) = (parents.len(), model.workers.lanes());
    let mut remaining = &mut results[..];
    let mut parts = Vec::with_capacity(lanes);
    for k in 0..lanes {
        let (start, end) = (k * n / lanes, (k + 1) * n / lanes);
        let (slots, rest) = remaining.split_at_mut(end - start);
        remaining = rest;
        parts.push((&parents[start..end], slots));
    }
    let parts = workers::handoff(parts);
    model.workers.run(lanes, |k| {
        let (contacts, slots) = workers::take(&parts, k);
        fill(contacts, slots);
    })?;
    Ok(true)
}

/// Energy and gradient of one friction contact; its blocks are written directly.
pub(super) struct FrictionTerm {
    pub e: Real,
    pub g: [Vec3; 4],
}
pub(super) type FrictionSlot = Option<FrictionTerm>;

/// Tangential offset, smoothed length and load of a lagged friction contact.
fn friction_kinematics(model: &Model, x: &[Vec3], f: &Friction) -> (Vec3, Real, Real) {
    let c = f.contact;
    let delta = c.relative(x) - f.reference;
    let u = delta - c.normal * delta.dot(c.normal);
    let eps = model.implicit.friction_velocity * model.h;
    let len = (u.length_squared() + eps * eps).sqrt();
    (u, len, c.kinetic_friction * f.load)
}

/// Evaluates one friction contact; `blocks` holds exactly its free blocks.
pub(super) fn friction_term(
    model: &Model,
    x: &[Vec3],
    f: &Friction,
    blocks: &mut [Block],
) -> FrictionTerm {
    let c = f.contact;
    let n = c.normal;
    let (u, len, load) = friction_kinematics(model, x, f);
    let eps = model.implicit.friction_velocity * model.h;
    let g = std::array::from_fn(|i| u * (load / len * c.weights[i]));
    let mut index = 0;
    if !blocks.is_empty() {
        for i in 0..4 {
            for j in 0..4 {
                let (pi, pj) = (c.particles[i] as usize, c.particles[j] as usize);
                if c.weights[i] == 0.0 || c.weights[j] == 0.0 {
                    continue;
                }
                let (Some(di), Some(dj)) = (model.dofs[pi], model.dofs[pj]) else {
                    continue;
                };
                if di < dj {
                    continue;
                }
                blocks[index] = Block {
                    row: di / 3,
                    col: dj / 3,
                    value: std::array::from_fn(|r| {
                        std::array::from_fn(|s| {
                            load * c.weights[i]
                                * c.weights[j]
                                * ((if r == s { 1.0 } else { 0.0 })
                                    - n[r] * n[s]
                                    - u[r] * u[s] / (len * len))
                                / len
                        })
                    }),
                };
                index += 1;
            }
        }
    }
    FrictionTerm {
        e: load * (len - eps),
        g,
    }
}

/// Number of free lower blocks of a friction contact.
pub(super) fn friction_blocks(model: &Model, f: &Friction) -> usize {
    let c = f.contact;
    let mut count = 0;
    for i in 0..4 {
        for j in 0..4 {
            if c.weights[i] != 0.0
                && c.weights[j] != 0.0
                && let (Some(di), Some(dj)) = (
                    model.dofs[c.particles[i] as usize],
                    model.dofs[c.particles[j] as usize],
                )
                && di >= dj
            {
                count += 1;
            }
        }
    }
    count
}

/// Evaluates friction contacts on the lanes: energies and gradients into
/// `results`, blocks into `blocks[base..]` at `offsets`. Returns false, with
/// `results` empty, when the caller evaluates them serially.
pub(super) fn evaluate_friction(
    model: &Model,
    x: &[Vec3],
    friction: &[Friction],
    hessian: bool,
    results: &mut Vec<FrictionSlot>,
    offsets: &mut Vec<usize>,
    blocks: &mut Vec<Block>,
) -> Result<bool, ClothError> {
    results.clear();
    if model.implicit.execution != ImplicitExecution::Parallel4 || friction.len() < 512 {
        return Ok(false);
    }
    offsets.clear();
    offsets.push(0);
    for f in friction {
        let next = offsets.last().unwrap()
            + if hessian {
                friction_blocks(model, f)
            } else {
                0
            };
        offsets.push(next);
    }
    let base = blocks.len();
    blocks.resize(
        base + offsets.last().unwrap(),
        Block {
            row: 0,
            col: 0,
            value: [[0.0; 3]; 3],
        },
    );
    results.resize_with(friction.len(), || None);
    let (n, lanes) = (friction.len(), model.workers.lanes());
    let (mut slots_rest, mut blocks_rest) = (&mut results[..], &mut blocks[base..]);
    let mut parts = Vec::with_capacity(lanes);
    for k in 0..lanes {
        let (start, end) = (k * n / lanes, (k + 1) * n / lanes);
        let (slots, rest) = slots_rest.split_at_mut(end - start);
        slots_rest = rest;
        let (lane_blocks, rest) = blocks_rest.split_at_mut(offsets[end] - offsets[start]);
        blocks_rest = rest;
        parts.push((start, slots, lane_blocks));
    }
    let parts = workers::handoff(parts);
    let offsets = &*offsets;
    model.workers.run(lanes, |k| {
        let (start, slots, mut lane_blocks) = workers::take(&parts, k);
        for (i, slot) in slots.iter_mut().enumerate() {
            let f = start + i;
            let (mine, rest) = lane_blocks.split_at_mut(offsets[f + 1] - offsets[f]);
            lane_blocks = rest;
            *slot = Some(friction_term(model, x, &friction[f], mine));
        }
    })?;
    Ok(true)
}

#[cfg(all(test, feature = "f64"))]
mod tests {
    use super::*;
    use crate::{ClothContactSettings, ClothMaterial, ClothMesh, GridBuilder, Solver};

    fn fixture() -> (Cloth, Vec<SurfaceContact>) {
        let patch = GridBuilder::new(12, 12).size(0.22, 0.22).build().unwrap();
        let n = patch.rest_positions().len() as u32;
        let mut x = patch.rest_positions().to_vec();
        x.extend(patch.rest_positions().iter().map(|p| *p + Vec3::Y * 0.0012));
        let mut faces = patch.triangles().to_vec();
        faces.extend(patch.triangles().iter().map(|f| f.map(|i| i + n)));
        let mut cloth =
            Cloth::new(ClothMesh::new(x, faces).unwrap(), ClothMaterial::default()).unwrap();
        let settings = ClothContactSettings {
            thickness: 0.001,
            activation_margin: 0.001,
            continuous_self_collision: true,
            ..Default::default()
        };
        cloth.set_contact_settings(Some(settings)).unwrap();
        let mut engine = SelfCollision::new(cloth.mesh.clone(), settings);
        let mut contacts = Vec::new();
        engine
            .implicit_primitives(cloth.positions(), &mut contacts)
            .unwrap();
        assert!(contacts.len() >= 1024);
        (cloth, contacts)
    }
    fn same(a: Result<Assembly, ClothError>, b: Result<Assembly, ClothError>) {
        match (a, b) {
            (Ok(a), Ok(b)) => {
                assert_eq!(a.energy.to_bits(), b.energy.to_bits());
                assert_eq!(
                    a.gradient.iter().map(|v| v.to_bits()).collect::<Vec<_>>(),
                    b.gradient.iter().map(|v| v.to_bits()).collect::<Vec<_>>()
                );
                let bits = |a: &Assembly| {
                    a.blocks
                        .iter()
                        .map(|b| (b.row, b.col, b.value.map(|r| r.map(Real::to_bits))))
                        .collect::<Vec<_>>()
                };
                assert_eq!(bits(&a), bits(&b));
            }
            (Err(a), Err(b)) => assert_eq!(a, b),
            _ => panic!("parallel assembly changed the result"),
        }
    }
    #[test]
    fn dense_contact_assembly_preserves_bits_order_fixed_blocks_and_errors() {
        let (cloth, mut contacts) = fixture();
        contacts.reverse(); // Accumulation follows input order, including mixed external contacts.
        let x = cloth.positions();
        let mut external = contacts[0];
        external.key.features = [
            crate::SurfaceFeature::Vertex(0),
            crate::SurfaceFeature::External {
                object: 1,
                feature: 0,
            },
        ];
        external.particles = [0; 4];
        external.weights = [1.0, 0.0, 0.0, 0.0];
        external.normal = Vec3::Y;
        external.offset = x[0] - Vec3::Y * 0.0015;
        contacts.insert(17, external);
        for fixed in [vec![], vec![0, 5, 287], (0..x.len() as u32).collect()] {
            let targets: Vec<_> = fixed
                .iter()
                .map(|&i| Target {
                    particle: i,
                    position: x[i as usize],
                    compliance: 0.0,
                })
                .collect();
            let mut model = Model::new(
                &cloth,
                0.1,
                Vec3::ZERO,
                &targets,
                ImplicitSettings::default(),
            )
            .unwrap();
            for hessian in [false, true] {
                for invalid in [false, true] {
                    let mut cs = contacts.clone();
                    if invalid {
                        cs[19].separation = 1.0;
                        cs.last_mut().unwrap().particles = [0; 4];
                    }
                    model.set_execution(ImplicitExecution::Serial);
                    let serial = model.assemble(x, &cs, &[], hessian);
                    if invalid {
                        assert_eq!(
                            serial.as_ref().err(),
                            Some(&ClothError::UnresolvedSurfaceContact)
                        );
                    }
                    model.set_execution(ImplicitExecution::Parallel4);
                    same(serial, model.assemble(x, &cs, &[], hessian));
                }
            }
        }
    }
    #[test]
    fn dense_friction_assembly_preserves_bits_blocks_and_order() {
        let (cloth, contacts) = fixture();
        let x = cloth.positions();
        // Lagged friction contacts with mixed tangential offsets and loads.
        let friction: Vec<Friction> = contacts
            .iter()
            .enumerate()
            .map(|(i, c)| Friction {
                contact: *c,
                reference: c.relative(x) + Vec3::new(1e-4, 0.0, -2e-4) * ((i % 5) as Real),
                load: 0.01 + 0.001 * (i % 7) as Real,
            })
            .collect();
        assert!(friction.len() >= 512);
        for fixed in [vec![], vec![3, 40, 200]] {
            let targets: Vec<_> = fixed
                .iter()
                .map(|&i| Target {
                    particle: i,
                    position: x[i as usize],
                    compliance: 0.0,
                })
                .collect();
            let mut model = Model::new(
                &cloth,
                0.1,
                Vec3::ZERO,
                &targets,
                ImplicitSettings::default(),
            )
            .unwrap();
            for hessian in [false, true] {
                model.set_execution(ImplicitExecution::Serial);
                let serial = model.assemble(x, &[], &friction, hessian).unwrap();
                model.set_execution(ImplicitExecution::Parallel4);
                let parallel = model.assemble(x, &[], &friction, hessian).unwrap();
                same(Ok(serial), Ok(parallel));
            }
        }
    }

    #[test]
    fn contact_worker_creation_failure_is_atomic_and_retry_matches_serial() {
        let (mut original, _) = fixture();
        original
            .set_implicit_solver(Some(ImplicitSettings {
                execution: ImplicitExecution::Parallel4,
                ..Default::default()
            }))
            .unwrap();
        let settings = SolverSettings {
            max_substep: 0.1,
            ..Default::default()
        };
        // This fixture is below the material/sparse thresholds, so the pool
        // is first created for the parent contacts.
        {
            let after = 0;
            let mut cloth = original.clone();
            let result = workers::fault::after(after, || {
                Solver::new().step(&mut cloth, 0.1, Vec3::ZERO, &settings)
            });
            assert!(
                matches!(result, Err(ClothError::ImplicitWorkerSpawnFailed(_))),
                "{result:?}"
            );
            assert_eq!(cloth.positions, original.positions);
            assert_eq!(cloth.previous, original.previous);
            assert_eq!(cloth.velocities, original.velocities);
            assert_eq!(cloth.forces, original.forces);
            assert_eq!(cloth.pins, original.pins);
            assert_eq!(cloth.implicit_settings, original.implicit_settings);
            let actual = Solver::new()
                .step(&mut cloth, 0.1, Vec3::ZERO, &settings)
                .unwrap();
            let mut expected = original.clone();
            expected
                .set_implicit_solver(Some(ImplicitSettings::default()))
                .unwrap();
            let reference = Solver::new()
                .step(&mut expected, 0.1, Vec3::ZERO, &settings)
                .unwrap();
            assert_eq!(cloth.positions, expected.positions);
            assert_eq!(cloth.velocities, expected.velocities);
            assert_eq!(actual.implicit, reference.implicit);
            assert_eq!(actual.surface_collision, reference.surface_collision);
        }
    }
    #[test]
    fn serial_and_small_contact_batches_do_not_spawn() {
        let (cloth, contacts) = fixture();
        for (execution, cs) in [
            (ImplicitExecution::Serial, contacts.as_slice()),
            (ImplicitExecution::Parallel4, &contacts[..1023]),
        ] {
            let model = Model::new(
                &cloth,
                0.1,
                Vec3::ZERO,
                &[],
                ImplicitSettings {
                    execution,
                    ..Default::default()
                },
            )
            .unwrap();
            workers::fault::after(0, || {
                assert!(!evaluate(&model, cloth.positions(), cs, &mut vec![]).unwrap());
            });
        }
    }
}
