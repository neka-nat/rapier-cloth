//! Evaluate independent parent contacts in parallel, then consume results in
//! their original order. No worker accumulates into the global objective/system.
use super::*;

type Evaluation = Result<primitive::Jet<true>, ClothError>;
type Slot = Option<Evaluation>;

pub(super) fn evaluate(
    model: &Model,
    x: &[Vec3],
    contacts: &[SurfaceContact],
    hessian: bool,
) -> Result<Option<Vec<Evaluation>>, ClothError> {
    if model.implicit.execution != ImplicitExecution::Parallel4
        || contacts.iter().filter(|c| primitive::is_parent(c)).count() < 1024
    {
        return Ok(None);
    }
    let parents: Vec<_> = contacts
        .iter()
        .filter(|c| primitive::is_parent(c))
        .collect();
    let mut results: Vec<Slot> = std::iter::repeat_with(|| None)
        .take(parents.len())
        .collect();
    let fill = |contacts: &[&SurfaceContact], slots: &mut [Slot]| {
        for (contact, slot) in contacts.iter().zip(slots) {
            // Retain errors in their input slots: report the first numerical
            // failure in contact order, independent of worker completion order.
            *slot = Some(if hessian {
                primitive::evaluate::<true>(
                    contact,
                    x,
                    model.cloth.mesh.rest_positions(),
                    model.band,
                    model.barrier_stiffness,
                )
                .map(|mut value| {
                    value.h = primitive::project(value.h);
                    value
                })
            } else {
                primitive::evaluate::<false>(
                    contact,
                    x,
                    model.cloth.mesh.rest_positions(),
                    model.band,
                    model.barrier_stiffness,
                )
                .map(|value| primitive::Jet {
                    e: value.e,
                    g: value.g,
                    h: SMatrix::zeros(),
                })
            });
        }
    };
    std::thread::scope(|scope| -> Result<(), ClothError> {
        let n = parents.len();
        let (caller, mut remaining) = results.split_at_mut(n / 4);
        let mut handles = Vec::with_capacity(3);
        for i in 1..4 {
            let start = i * n / 4;
            let end = (i + 1) * n / 4;
            let (slots, rest) = remaining.split_at_mut(end - start);
            remaining = rest;
            let contacts = &parents[start..end];
            let fill = &fill;
            // The scope joins previously started workers if creation fails.
            handles.push(workers::spawn(scope, move || fill(contacts, slots))?);
        }
        fill(&parents[..n / 4], caller);
        for handle in handles {
            workers::join(handle);
        }
        Ok(())
    })?;
    Ok(Some(
        results
            .into_iter()
            .map(|r| r.expect("unfilled parent contact"))
            .collect(),
    ))
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
                assert_eq!(
                    a.triplets
                        .iter()
                        .map(|t| (t.row, t.col, t.val.to_bits()))
                        .collect::<Vec<_>>(),
                    b.triplets
                        .iter()
                        .map(|t| (t.row, t.col, t.val.to_bits()))
                        .collect::<Vec<_>>()
                );
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
                    model.implicit.execution = ImplicitExecution::Serial;
                    let serial = model.assemble(x, &cs, &[], hessian);
                    if invalid {
                        assert_eq!(
                            serial.as_ref().err(),
                            Some(&ClothError::UnresolvedSurfaceContact)
                        );
                    }
                    model.implicit.execution = ImplicitExecution::Parallel4;
                    same(serial, model.assemble(x, &cs, &[], hessian));
                }
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
        // This fixture is below the material/sparse thresholds. The first
        // creation attempts belong to parent contacts; test partial creation too.
        for after in [0, 1, 2] {
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
                assert!(
                    evaluate(&model, cloth.positions(), cs, true)
                        .unwrap()
                        .is_none()
                );
            });
        }
    }
}
