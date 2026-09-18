//! Scoped worker creation with a recoverable operating-system error. Callers
//! retain all jobs inside a scope, which also joins workers on an early error.
use crate::ClothError;
use std::thread::{Scope, ScopedJoinHandle};

pub(super) fn spawn<'scope, 'env, F, T>(
    scope: &'scope Scope<'scope, 'env>,
    work: F,
) -> Result<ScopedJoinHandle<'scope, T>, ClothError>
where
    F: FnOnce() -> T + Send + 'scope,
    T: Send + 'scope,
{
    #[cfg(test)]
    fault::before_spawn()?;
    std::thread::Builder::new()
        .spawn_scoped(scope, work)
        .map_err(|error| ClothError::ImplicitWorkerSpawnFailed(error.to_string()))
}

pub(super) fn join<T>(handle: ScopedJoinHandle<'_, T>) -> T {
    // Programmer panics have the same semantics as the serial path. Do not
    // report a successful solve or disguise them as resource exhaustion.
    handle
        .join()
        .unwrap_or_else(|panic| std::panic::resume_unwind(panic))
}

#[cfg(test)]
pub(super) mod fault {
    use crate::ClothError;
    use std::cell::Cell;

    thread_local! { static REMAINING: Cell<Option<usize>> = const { Cell::new(None) }; }

    pub(super) fn before_spawn() -> Result<(), ClothError> {
        REMAINING.with(|remaining| match remaining.get() {
            Some(0) => Err(ClothError::ImplicitWorkerSpawnFailed(
                "injected creation failure".into(),
            )),
            Some(n) => {
                remaining.set(Some(n - 1));
                Ok(())
            }
            None => Ok(()),
        })
    }

    pub(crate) fn after<T>(successful_spawns: usize, run: impl FnOnce() -> T) -> T {
        struct Reset(Option<usize>);
        impl Drop for Reset {
            fn drop(&mut self) {
                REMAINING.with(|remaining| remaining.set(self.0));
            }
        }
        let _reset = Reset(REMAINING.with(|remaining| remaining.replace(Some(successful_spawns))));
        run()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{
        atomic::{AtomicBool, Ordering},
        mpsc,
    };

    #[test]
    fn partial_creation_failure_joins_started_workers_before_returning() {
        let completed = AtomicBool::new(false);
        let (send, receive) = mpsc::channel();
        let result = fault::after(1, || {
            std::thread::scope(|scope| {
                let completed = &completed;
                let _handle = spawn(scope, move || {
                    receive.recv().unwrap();
                    completed.store(true, Ordering::SeqCst);
                })?;
                send.send(()).unwrap();
                spawn(scope, || ())?;
                Ok::<_, ClothError>(())
            })
        });
        assert!(matches!(
            result,
            Err(ClothError::ImplicitWorkerSpawnFailed(_))
        ));
        assert!(completed.load(Ordering::SeqCst));
        // The injected failure is thread-local and restored after this call.
        std::thread::scope(|scope| join(spawn(scope, || 7).unwrap()));
    }

    #[test]
    fn worker_panics_remain_panics() {
        let result = std::panic::catch_unwind(|| {
            std::thread::scope(|scope| {
                join(spawn(scope, || panic!("worker panic control")).unwrap());
            })
        });
        assert!(result.is_err());
    }

    #[cfg(feature = "f64")]
    mod physical {
        use super::*;
        use crate::*;

        fn cloth(n: usize) -> Cloth {
            let mesh = GridBuilder::new(n, n)
                .size(0.5, 0.5)
                .origin(Vec3::Y)
                .build()
                .unwrap();
            let mut cloth = Cloth::new(mesh, ClothMaterial::default()).unwrap();
            cloth
                .set_contact_settings(Some(ClothContactSettings {
                    self_collision: false,
                    ..Default::default()
                }))
                .unwrap();
            cloth
                .set_implicit_solver(Some(ImplicitSettings {
                    execution: ImplicitExecution::Parallel4,
                    ..Default::default()
                }))
                .unwrap();
            cloth
        }

        #[test]
        fn material_and_sparse_creation_failures_preserve_state_and_allow_retry() {
            let settings = SolverSettings {
                max_substep: 0.1,
                ..Default::default()
            };
            let mut original = cloth(32);
            for (force, mass) in original.forces.iter_mut().zip(&original.masses) {
                *force = Vec3::X * (*mass * 0.1);
            }
            // 0/1: before/after one material worker; 3/4: before/after
            // one sparse worker. Both phases are reached in the first Newton
            // iteration; no OS resource exhaustion is required for the test.
            for after in [0, 1, 3, 4] {
                let mut actual = original.clone();
                let mut solver = Solver::new();
                let result =
                    fault::after(after, || solver.step(&mut actual, 0.1, -Vec3::Y, &settings));
                assert!(
                    matches!(result, Err(ClothError::ImplicitWorkerSpawnFailed(_))),
                    "{after}: {result:?}"
                );
                assert_eq!(actual.positions, original.positions);
                assert_eq!(actual.previous, original.previous);
                assert_eq!(actual.velocities, original.velocities);
                assert_eq!(actual.forces, original.forces);
                assert_eq!(actual.pins, original.pins);
                assert_eq!(actual.implicit_settings, original.implicit_settings);
                assert_eq!(
                    format!("{:?}", actual.contact_history),
                    format!("{:?}", original.contact_history)
                );
                let report = solver.step(&mut actual, 0.1, -Vec3::Y, &settings).unwrap();
                let mut expected = original.clone();
                expected
                    .set_implicit_solver(Some(ImplicitSettings::default()))
                    .unwrap();
                let reference = Solver::new()
                    .step(&mut expected, 0.1, -Vec3::Y, &settings)
                    .unwrap();
                assert_eq!(actual.positions, expected.positions);
                assert_eq!(actual.previous, expected.previous);
                assert_eq!(actual.velocities, expected.velocities);
                assert_eq!(report.iterations, reference.iterations);
                assert_eq!(report.surface_collision, reference.surface_collision);
            }
        }

        #[test]
        fn serial_default_and_small_parallel_inputs_never_spawn() {
            assert_eq!(
                ImplicitSettings::default().execution,
                ImplicitExecution::Serial
            );
            let settings = SolverSettings {
                max_substep: 0.1,
                ..Default::default()
            };
            for (size, execution) in [
                (32, ImplicitExecution::Serial),
                (3, ImplicitExecution::Parallel4),
            ] {
                let mut actual = cloth(size);
                actual
                    .set_implicit_solver(Some(ImplicitSettings {
                        execution,
                        ..Default::default()
                    }))
                    .unwrap();
                fault::after(0, || {
                    Solver::new().step(&mut actual, 0.1, -Vec3::Y, &settings)
                })
                .unwrap();
            }
        }
    }
}
