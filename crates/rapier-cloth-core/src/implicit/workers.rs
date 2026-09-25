//! Lane execution for the parallel mode. A step creates a private thread pool
//! on its first parallel phase and joins it when the step ends. Each `run`
//! executes one job per lane (lane 0 on the caller) with borrowed data and
//! returns the results in lane order.
use crate::ClothError;
use std::sync::{Mutex, OnceLock};

/// Lane count of one physical step; 1 runs everything on the caller.
#[derive(Debug)]
pub(crate) struct Workers {
    lanes: usize,
    pool: OnceLock<rayon::ThreadPool>,
}

impl Default for Workers {
    fn default() -> Self {
        Self::new(1)
    }
}

impl Workers {
    /// `lanes` includes the calling thread.
    pub fn new(lanes: usize) -> Self {
        Self {
            lanes: lanes.max(1),
            pool: OnceLock::new(),
        }
    }
    pub fn lanes(&self) -> usize {
        self.lanes
    }
    /// The step's pool of `lanes - 1` threads, created on first use. A
    /// creation failure returns `ImplicitWorkerSpawnFailed`; a later call
    /// retries the creation.
    fn pool(&self) -> Result<&rayon::ThreadPool, ClothError> {
        if let Some(pool) = self.pool.get() {
            return Ok(pool);
        }
        #[cfg(test)]
        fault::before_spawn()?;
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(self.lanes - 1)
            .thread_name(|i| format!("rapier-cloth-lane-{}", i + 1))
            .build()
            .map_err(|error| ClothError::ImplicitWorkerSpawnFailed(error.to_string()))?;
        Ok(self.pool.get_or_init(|| pool))
    }
    /// Runs `job(k)` for every lane `k < count` and returns the results in
    /// lane order regardless of completion order. A lane panic propagates to
    /// the caller after the other lanes finish, as on the serial path.
    pub fn run<T: Send>(
        &self,
        count: usize,
        job: impl Fn(usize) -> T + Sync,
    ) -> Result<Vec<T>, ClothError> {
        let count = count.clamp(1, self.lanes);
        if count == 1 {
            return Ok(vec![job(0)]);
        }
        let pool = self.pool()?;
        let slots: Vec<Mutex<Option<T>>> = (0..count).map(|_| Mutex::new(None)).collect();
        let job = &job;
        pool.in_place_scope(|scope| {
            for (k, slot) in slots.iter().enumerate().skip(1) {
                scope.spawn(move |_| {
                    let value = job(k);
                    *slot.lock().unwrap() = Some(value);
                });
            }
            let value = job(0);
            *slots[0].lock().unwrap() = Some(value);
        });
        Ok(slots
            .into_iter()
            .map(|slot| slot.into_inner().unwrap().expect("lane result"))
            .collect())
    }
}

/// Moves per-lane data into slots that a shared `Fn` job can take by lane.
pub(crate) fn handoff<T>(parts: Vec<T>) -> Vec<Mutex<Option<T>>> {
    parts
        .into_iter()
        .map(|part| Mutex::new(Some(part)))
        .collect()
}
pub(crate) fn take<T>(parts: &[Mutex<Option<T>>], lane: usize) -> T {
    parts[lane].lock().unwrap().take().expect("lane data")
}

#[cfg(test)]
pub(crate) mod fault {
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
    use std::sync::atomic::{AtomicUsize, Ordering};

    #[test]
    fn lanes_run_once_each_and_results_keep_lane_order() {
        let workers = Workers::new(4);
        let calls = AtomicUsize::new(0);
        for round in 0..20 {
            let data: Vec<usize> = (0..1000).map(|i| i * round).collect();
            let sums = workers
                .run(4, |k| {
                    calls.fetch_add(1, Ordering::SeqCst);
                    data[k * 250..(k + 1) * 250].iter().sum::<usize>()
                })
                .unwrap();
            let expected: Vec<usize> = (0..4)
                .map(|k| data[k * 250..(k + 1) * 250].iter().sum::<usize>())
                .collect();
            assert_eq!(sums, expected);
            assert_eq!(workers.run(2, |k| k * 10).unwrap(), vec![0, 10]);
        }
        assert_eq!(calls.load(Ordering::SeqCst), 80);
        assert_eq!(Workers::new(1).run(4, |k| k).unwrap(), vec![0]);
        let parts = handoff(vec![vec![1], vec![2, 2]]);
        assert_eq!(
            workers.run(2, |k| take(&parts, k).len()).unwrap(),
            vec![1, 2]
        );
    }

    #[test]
    fn creation_failure_is_reported_and_retried() {
        let workers = Workers::new(4);
        let result = fault::after(0, || workers.run(4, |k| k));
        assert!(matches!(
            result,
            Err(ClothError::ImplicitWorkerSpawnFailed(_))
        ));
        assert!(workers.pool.get().is_none());
        // The injected failure is thread-local and restored after the call;
        // the pool is created once and then reused.
        assert_eq!(workers.run(4, |k| k).unwrap(), vec![0, 1, 2, 3]);
        assert_eq!(
            fault::after(0, || workers.run(4, |k| k + 1)).unwrap(),
            vec![1, 2, 3, 4]
        );
        // A single lane creates no thread even under injected failure.
        assert_eq!(
            fault::after(0, || Workers::new(1).run(1, |k| k)).unwrap(),
            vec![0]
        );
    }

    #[test]
    fn worker_panics_remain_panics() {
        let result = std::panic::catch_unwind(|| {
            Workers::new(3)
                .run(3, |k| {
                    if k == 2 {
                        panic!("worker panic control");
                    }
                    k
                })
                .unwrap()
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
        fn lane_creation_failures_preserve_state_and_allow_retry() {
            let settings = SolverSettings {
                max_substep: 0.1,
                ..Default::default()
            };
            let mut original = cloth(32);
            for (force, mass) in original.forces.iter_mut().zip(&original.masses) {
                *force = Vec3::X * (*mass * 0.1);
            }
            // The step's pool is created at its first parallel phase; the
            // failure leaves the state unchanged and the retry commits.
            {
                let after = 0;
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
            // One successful creation serves the whole step.
            let mut actual = original.clone();
            fault::after(1, || {
                Solver::new().step(&mut actual, 0.1, -Vec3::Y, &settings)
            })
            .unwrap();
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
