use super::{ImplicitExecution, failure, workers};
use crate::{ClothError, Real};
use faer::{
    Side,
    sparse::{
        SparseColMat, SparseColMatRef, SymbolicSparseColMat, Triplet,
        linalg::solvers::{Llt, SymbolicLlt},
    },
};

#[derive(Default)]
struct ColumnOrder {
    rows: Vec<usize>,
    order: Vec<usize>,
}

/// Derived data local to one physical step. Numerical values and factors are
/// always rebuilt; no optimizer state or contact history survives the step.
#[derive(Default)]
pub(super) struct SparseSystem {
    execution: ImplicitExecution,
    columns: Vec<ColumnOrder>,
    starts: Vec<usize>,
    next: Vec<usize>,
    entries: Vec<usize>,
    nonzeros: usize,
    factor: Option<(SymbolicSparseColMat<usize>, SymbolicLlt<usize>)>,
}

impl SparseSystem {
    pub(super) fn new(execution: ImplicitExecution) -> Self {
        Self {
            execution,
            ..Self::default()
        }
    }

    pub fn matrix(
        &mut self,
        size: usize,
        triplets: &[Triplet<usize, usize, Real>],
        iteration: usize,
    ) -> Result<SparseColMat<usize, Real>, ClothError> {
        if self.execution == ImplicitExecution::Parallel4 && size >= 1024 {
            self.matrix_parallel(size, triplets, iteration)
        } else {
            self.matrix_serial(size, triplets, iteration)
        }
    }

    fn matrix_serial(
        &mut self,
        size: usize,
        triplets: &[Triplet<usize, usize, Real>],
        iteration: usize,
    ) -> Result<SparseColMat<usize, Real>, ClothError> {
        self.bucket(size, triplets, iteration)?;
        let mut pointers = Vec::with_capacity(size + 1);
        let mut rows = Vec::with_capacity(self.nonzeros);
        let mut values: Vec<Real> = Vec::with_capacity(self.nonzeros);
        pointers.push(0);
        for (col, cached) in self.columns.iter_mut().enumerate() {
            let entries = &self.entries[self.starts[col]..self.starts[col + 1]];
            // Column buckets retain the original triplet order. Reuse sorting
            // only when their entire ordered row sequence matches, including
            // every duplicate; global triplet offsets may change independently.
            let unchanged = cached.rows.len() == entries.len()
                && cached
                    .rows
                    .iter()
                    .zip(entries)
                    .all(|(&r, &i)| r == triplets[i].row);
            if !unchanged {
                cached.rows.clear();
                cached.rows.extend(entries.iter().map(|&i| triplets[i].row));
                cached.order.clear();
                cached.order.extend(0..entries.len());
                // Match faer 0.24's column-local unstable sort and duplicate
                // summation order, rather than changing floating-point sums.
                cached
                    .order
                    .sort_unstable_by_key(|&i| (cached.rows[i], col));
            }
            let mut previous = usize::MAX;
            for &i in &cached.order {
                let t = triplets[entries[i]];
                if t.row == previous {
                    *values.last_mut().unwrap() += t.val;
                } else {
                    rows.push(t.row);
                    values.push(t.val);
                    previous = t.row;
                }
            }
            pointers.push(rows.len());
        }
        self.nonzeros = rows.len();
        let symbolic = SymbolicSparseColMat::new_checked(size, size, pointers, None, rows);
        Ok(SparseColMat::new(symbolic, values))
    }

    fn bucket(
        &mut self,
        size: usize,
        triplets: &[Triplet<usize, usize, Real>],
        iteration: usize,
    ) -> Result<(), ClothError> {
        if self.columns.len() != size {
            self.columns.clear();
            self.columns.resize_with(size, ColumnOrder::default);
        }
        self.starts.clear();
        self.starts.resize(size + 1, 0);
        for t in triplets {
            if t.row >= size || t.col >= size {
                return Err(failure("matrix assembly", iteration));
            }
            self.starts[t.col + 1] += 1;
        }
        for col in 0..size {
            self.starts[col + 1] += self.starts[col];
        }
        self.next.clone_from(&self.starts);
        self.entries.resize(triplets.len(), 0);
        for (i, t) in triplets.iter().enumerate() {
            self.entries[self.next[t.col]] = i;
            self.next[t.col] += 1;
        }
        Ok(())
    }

    pub fn factor(
        &mut self,
        matrix: SparseColMatRef<'_, usize, Real>,
        iteration: usize,
    ) -> Result<Llt<usize, Real>, ClothError> {
        // `matrix` is packed CSC from the constructor above. Equal dimensions,
        // column pointers and row indices establish the complete graph, even
        // when contact multiplicities or the triplet order have changed.
        let pattern = matrix.symbolic();
        let unchanged = self.factor.as_ref().is_some_and(|(old, _)| {
            old.nrows() == pattern.nrows()
                && old.ncols() == pattern.ncols()
                && old.col_ptr() == pattern.col_ptr()
                && old.row_idx() == pattern.row_idx()
        });
        if !unchanged {
            let symbolic = SymbolicLlt::try_new(pattern, Side::Lower)
                .map_err(|_| failure("factorization", iteration))?;
            self.factor = Some((
                pattern
                    .to_owned()
                    .map_err(|_| failure("factorization", iteration))?,
                symbolic,
            ));
        }
        Llt::try_new_with_symbolic(self.factor.as_ref().unwrap().1.clone(), matrix, Side::Lower)
            .map_err(|_| failure("factorization", iteration))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use faer::{Mat, prelude::Solve};

    fn check(cache: &mut SparseSystem, size: usize, entries: &[Triplet<usize, usize, Real>]) {
        let matrix = cache.matrix(size, entries, 0).unwrap();
        let fresh = SparseColMat::try_new_from_triplets(size, size, entries).unwrap();
        assert_eq!(matrix.symbolic().col_ptr(), fresh.symbolic().col_ptr());
        assert_eq!(matrix.symbolic().row_idx(), fresh.symbolic().row_idx());
        assert_eq!(matrix.val(), fresh.val());
        let dense = fresh.to_dense();
        let exact = Mat::from_fn(size, 1, |i, _| (i + 1) as Real * 0.7);
        let rhs = Mat::from_fn(size, 1, |i, _| {
            (0..size)
                .map(|j| dense[(i.max(j), i.min(j))] * exact[(j, 0)])
                .sum::<Real>()
        });
        let solved = cache
            .factor(matrix.as_ref(), 0)
            .unwrap()
            .solve(rhs.as_ref());
        for i in 0..size {
            assert!((solved[(i, 0)] - exact[(i, 0)]).abs() < 128.0 * Real::EPSILON);
        }
    }

    #[test]
    fn repeated_structure_uses_current_values_and_duplicate_order() {
        let mut cache = SparseSystem::default();
        let mut entries = vec![
            Triplet::new(0, 0, 8.0),
            Triplet::new(1, 1, 6.0),
            Triplet::new(1, 0, 0.25),
            Triplet::new(0, 0, 1.0),
            Triplet::new(1, 0, 0.5),
        ];
        check(&mut cache, 2, &entries);
        for t in &mut entries {
            t.val *= 1.7;
        }
        check(&mut cache, 2, &entries);
        entries.swap(0, 1);
        check(&mut cache, 2, &entries);
        entries.push(Triplet::new(0, 0, 2.0));
        check(&mut cache, 2, &entries);
    }

    #[test]
    fn contact_graph_changes_and_release_dimensions_rebuild_the_pattern() {
        let mut cache = SparseSystem::default();
        let mut entries = vec![
            Triplet::new(0, 0, 8.0),
            Triplet::new(1, 1, 6.0),
            Triplet::new(2, 2, 5.0),
            Triplet::new(1, 0, 0.5),
        ];
        check(&mut cache, 3, &entries);
        entries[3] = Triplet::new(2, 0, 0.4);
        check(&mut cache, 3, &entries);
        entries.pop();
        check(&mut cache, 3, &entries);
        entries.pop();
        check(&mut cache, 2, &entries);
    }

    #[test]
    fn a_failed_numeric_factor_is_not_reused_on_retry() {
        let mut cache = SparseSystem::default();
        let entries = [Triplet::new(0, 0, -1.0), Triplet::new(1, 1, 2.0)];
        let matrix = cache.matrix(2, &entries, 0).unwrap();
        assert!(cache.factor(matrix.as_ref(), 0).is_err());
        check(
            &mut cache,
            2,
            &[Triplet::new(0, 0, 4.0), Triplet::new(1, 1, 2.0)],
        );
    }

    #[test]
    fn column_reuse_matches_uncached_roundoff_after_global_offset_changes() {
        let mut cache = SparseSystem::default();
        let mut seed = 17_u64;
        let mut next = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            (seed >> 32) as usize
        };
        let size = 17;
        let mut entries: Vec<_> = (0..size).map(|i| Triplet::new(i, i, 1024.0)).collect();
        for _ in 0..4096 {
            let a = next() % size;
            let b = next() % size;
            let value = (next() % 256) as Real * 0.0139 - 1.78;
            entries.push(Triplet::new(a.max(b), a.min(b), value));
        }
        check(&mut cache, size, &entries);
        // Only column zero gains a contribution, but every original input
        // offset moves. Unchanged columns must still read the current values.
        entries.insert(0, Triplet::new(size - 1, 0, 0.35));
        check(&mut cache, size, &entries);
        for _ in 0..3 {
            for t in &mut entries {
                t.val *= 1.1;
            }
            for i in 0..entries.len() {
                let j = next() % entries.len();
                entries.swap(i, j);
            }
            check(&mut cache, size, &entries);
        }
    }
}

struct ColumnLane {
    pointers: Vec<usize>,
    rows: Vec<usize>,
    values: Vec<Real>,
}

impl SparseSystem {
    fn matrix_parallel(
        &mut self,
        size: usize,
        triplets: &[Triplet<usize, usize, Real>],
        iteration: usize,
    ) -> Result<SparseColMat<usize, Real>, ClothError> {
        let workers = 4;
        self.bucket(size, triplets, iteration)?;
        let starts = &self.starts;
        let entries = &self.entries;
        let capacity = self.nonzeros / workers;
        let columns = &mut self.columns;
        let lanes = std::thread::scope(|scope| -> Result<Vec<ColumnLane>, ClothError> {
            let (first, mut remaining) = columns.split_at_mut(size / workers);
            let mut handles = Vec::with_capacity(workers - 1);
            for lane in 1..workers {
                let start = lane * size / workers;
                let end = (lane + 1) * size / workers;
                let (columns, rest) = remaining.split_at_mut(end - start);
                remaining = rest;
                handles.push(workers::spawn(scope, move || {
                    column_lane(columns, start, starts, entries, triplets, capacity)
                })?);
            }
            let mut results = Vec::with_capacity(workers);
            results.push(column_lane(first, 0, starts, entries, triplets, capacity));
            // Join and concatenate in column order, regardless of completion.
            for handle in handles {
                results.push(workers::join(handle));
            }
            Ok(results)
        })?;
        let nonzeros = lanes.iter().map(|lane| lane.rows.len()).sum();
        let mut pointers = Vec::with_capacity(size + 1);
        let mut rows = Vec::with_capacity(nonzeros);
        let mut values = Vec::with_capacity(nonzeros);
        pointers.push(0);
        for mut lane in lanes {
            let base = rows.len();
            pointers.extend(lane.pointers.iter().skip(1).map(|offset| base + offset));
            rows.append(&mut lane.rows);
            values.append(&mut lane.values);
        }
        self.nonzeros = rows.len();
        let symbolic = SymbolicSparseColMat::new_checked(size, size, pointers, None, rows);
        Ok(SparseColMat::new(symbolic, values))
    }
}

fn column_lane(
    columns: &mut [ColumnOrder],
    first_column: usize,
    starts: &[usize],
    entry_indices: &[usize],
    triplets: &[Triplet<usize, usize, Real>],
    capacity: usize,
) -> ColumnLane {
    let mut pointers = Vec::with_capacity(columns.len() + 1);
    let mut rows = Vec::with_capacity(capacity);
    let mut values: Vec<Real> = Vec::with_capacity(capacity);
    pointers.push(0);
    for (local, cached) in columns.iter_mut().enumerate() {
        let col = first_column + local;
        // Preserve the serial column permutation and duplicate summation order.
        let entries = &entry_indices[starts[col]..starts[col + 1]];
        // Column buckets retain the original triplet order. Reuse sorting
        // only when their entire ordered row sequence matches, including
        // every duplicate; global triplet offsets may change independently.
        let unchanged = cached.rows.len() == entries.len()
            && cached
                .rows
                .iter()
                .zip(entries)
                .all(|(&r, &i)| r == triplets[i].row);
        if !unchanged {
            cached.rows.clear();
            cached.rows.extend(entries.iter().map(|&i| triplets[i].row));
            cached.order.clear();
            cached.order.extend(0..entries.len());
            // Match faer 0.24's column-local unstable sort and duplicate
            // summation order, rather than changing floating-point sums.
            cached
                .order
                .sort_unstable_by_key(|&i| (cached.rows[i], col));
        }
        let mut previous = usize::MAX;
        for &i in &cached.order {
            let t = triplets[entries[i]];
            if t.row == previous {
                *values.last_mut().unwrap() += t.val;
            } else {
                rows.push(t.row);
                values.push(t.val);
                previous = t.row;
            }
        }
        pointers.push(rows.len());
    }
    ColumnLane {
        pointers,
        rows,
        values,
    }
}
#[cfg(test)]
mod parallel_tests {
    use super::*;
    fn identical_matrix(
        a: &Result<SparseColMat<usize, Real>, ClothError>,
        b: &Result<SparseColMat<usize, Real>, ClothError>,
    ) {
        match (a, b) {
            (Ok(a), Ok(b)) => {
                assert_eq!((a.nrows(), a.ncols()), (b.nrows(), b.ncols()));
                assert_eq!(a.symbolic().col_ptr(), b.symbolic().col_ptr());
                assert_eq!(a.symbolic().row_idx(), b.symbolic().row_idx());
                assert_eq!(a.val().len(), b.val().len());
                for (a, b) in a.val().iter().zip(b.val()) {
                    assert_eq!(a.to_bits(), b.to_bits());
                }
            }
            (Err(a), Err(b)) => assert_eq!(format!("{a:?}"), format!("{b:?}")),
            _ => panic!("sparse matrix result mismatch"),
        }
    }

    use faer::{Mat, prelude::Solve};

    type MatrixInput = (usize, Vec<Triplet<usize, usize, Real>>);

    fn sequence(states: Vec<MatrixInput>) {
        let mut original = SparseSystem::default();
        let mut parallel = SparseSystem::default();
        for (iteration, (size, entries)) in states.into_iter().enumerate() {
            let a = original.matrix_serial(size, &entries, iteration);
            let c = parallel.matrix_parallel(size, &entries, iteration);
            identical_matrix(&a, &c);
            if let (Ok(a), Ok(c)) = (a, c) {
                identical_matrix(
                    &Ok(a.clone()),
                    &SparseColMat::try_new_from_triplets(size, size, &entries)
                        .map_err(|_| failure("fresh", iteration)),
                );
                if size == 0 {
                    continue;
                }
                let rhs = Mat::from_fn(size, 1, |i, _| (i as Real * 0.139).sin());
                let first = original
                    .factor(a.as_ref(), iteration)
                    .map(|f| f.solve(rhs.as_ref()));
                let other = parallel
                    .factor(c.as_ref(), iteration)
                    .map(|f| f.solve(rhs.as_ref()));
                match (&first, &other) {
                    (Ok(a), Ok(b)) => {
                        for i in 0..size {
                            assert_eq!(a[(i, 0)].to_bits(), b[(i, 0)].to_bits());
                        }
                    }
                    (Err(a), Err(b)) => assert_eq!(format!("{a:?}"), format!("{b:?}")),
                    _ => panic!("factor/solve result mismatch"),
                }
            }
        }
    }

    #[test]
    fn duplicate_roundoff_order_survives_value_and_global_offset_changes() {
        let mut seed = 23_u64;
        let mut next = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            (seed >> 32) as usize
        };
        let size = 37;
        let mut entries: Vec<_> = (0..size).map(|i| Triplet::new(i, i, 10000.0)).collect();
        for _ in 0..6000 {
            let a = next() % size;
            let b = next() % size;
            entries.push(Triplet::new(
                a.max(b),
                a.min(b),
                (next() % 256) as Real * 0.0139 - 1.78,
            ));
        }
        let mut states = vec![(size, entries.clone())];
        for t in &mut entries {
            t.val *= 1.137;
        }
        states.push((size, entries.clone())); // unchanged ordered row pattern
        entries.insert(0, Triplet::new(size - 1, 0, 0.35));
        states.push((size, entries.clone())); // offsets of every other column move
        for _ in 0..3 {
            for i in 0..entries.len() {
                let j = next() % entries.len();
                entries.swap(i, j);
            }
            states.push((size, entries.clone()));
        }
        sequence(states);
    }

    #[test]
    fn large_columns_contact_changes_and_released_dofs_match_exact_solves() {
        let mut entries: Vec<_> = (0..2051).map(|i| Triplet::new(i, i, 10.0)).collect();
        for col in 0..2050 {
            entries.push(Triplet::new(col + 1, col, -0.5));
        }
        let mut states = vec![(2051, entries.clone())];
        entries.push(Triplet::new(2050, 0, -0.2));
        states.push((2051, entries.clone()));
        entries.pop();
        entries.retain(|t| t.row < 1027 && t.col < 1027);
        states.push((1027, entries.clone()));
        entries.extend((1027..2051).map(|i| Triplet::new(i, i, 12.0)));
        states.push((2051, entries));
        sequence(states);
    }

    #[test]
    fn empty_columns_small_sizes_and_invalid_indices_do_not_poison_retry() {
        sequence(vec![
            (0, vec![]),
            (1, vec![Triplet::new(0, 0, 2.0)]),
            (3, vec![Triplet::new(0, 0, 2.0)]), // empty columns; singular factor
            (3, vec![Triplet::new(3, 0, 1.0)]), // invalid row
            (3, vec![Triplet::new(0, 3, 1.0)]), // invalid column
            (3, (0..3).map(|i| Triplet::new(i, i, 2.0)).collect()),
        ]);
        let mut a = SparseSystem::default();
        let mut b = SparseSystem::new(ImplicitExecution::Parallel4);
        for size in [0, 1, 1023, 1024, 1027] {
            let entries: Vec<_> = (0..size).map(|i| Triplet::new(i, i, 2.0)).collect();
            identical_matrix(
                &a.matrix_serial(size, &entries, 7),
                &b.matrix(size, &entries, 7),
            );
        }
    }

    #[test]
    fn failed_factor_is_recomputed_on_same_pattern_successful_retry() {
        sequence(vec![
            (2, vec![Triplet::new(0, 0, -1.0), Triplet::new(1, 1, 2.0)]),
            (2, vec![Triplet::new(0, 0, 4.0), Triplet::new(1, 1, 2.0)]),
        ]);
    }

    #[test]
    fn nonfinite_and_signed_zero_values_are_preserved_without_factorization() {
        let entries = vec![
            Triplet::new(0, 0, -0.0),
            Triplet::new(1, 1, Real::NAN),
            Triplet::new(2, 2, Real::INFINITY),
            Triplet::new(3, 3, Real::NEG_INFINITY),
        ];
        let original = SparseSystem::default().matrix_serial(4, &entries, 9);
        identical_matrix(
            &original,
            &SparseSystem::default().matrix_parallel(4, &entries, 9),
        );
    }
}
