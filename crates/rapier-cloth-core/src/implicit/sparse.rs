//! Block-sparse Newton system. Assembly records one 3×3 block per element pair;
//! blocks are summed in assembly order into a lower-triangular CSC pattern that
//! only grows during a physical step, so symbolic analysis is rarely repeated.
use super::{failure, workers};
use crate::{ClothError, Real};
use faer::{
    Conj, MatMut, Par, Side,
    dyn_stack::{MemBuffer, MemStack},
    linalg::cholesky::llt::factor::LltRegularization,
    perm::PermRef,
    sparse::{
        SparseColMatRef, SymbolicSparseColMatRef,
        linalg::cholesky::{
            LltRef, SymbolicCholesky, SymmetricOrdering, factorize_symbolic_cholesky,
        },
    },
};

/// A 3×3 contribution between free vertex blocks with `row >= col`. A diagonal
/// block (`row == col`) contributes only its lower triangle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Block {
    pub row: usize,
    pub col: usize,
    pub value: [[Real; 3]; 3],
}

/// Symbolic analysis and numeric work buffers for one pattern.
struct Factor {
    symbolic: SymbolicCholesky<usize>,
    values: Vec<Real>,
    numeric: MemBuffer,
    solve: MemBuffer,
}

/// Derived data local to one physical step. Numerical values and factors are
/// always rebuilt; no optimizer state or contact history survives the step.
#[derive(Default)]
pub(super) struct SparseSystem {
    /// Sorted block rows of every block column; the pattern of this step.
    columns: Vec<Vec<usize>>,
    /// Scalar column starts of each block, parallel to `columns`.
    starts: Vec<Vec<[usize; 3]>>,
    col_ptr: Vec<usize>,
    row_idx: Vec<usize>,
    values: Vec<Real>,
    factor: Option<Factor>,
    /// Threads for accumulation; column ranges are disjoint per lane.
    workers: std::sync::Arc<workers::Workers>,
    /// Fill-reducing ordering and factor size of the latest analysis. A grown
    /// pattern first tries this ordering, which is much cheaper than a new one.
    ordering: Option<(Vec<usize>, usize)>,
}

impl SparseSystem {
    pub fn new(workers: std::sync::Arc<workers::Workers>) -> Self {
        Self {
            workers,
            ..Self::default()
        }
    }

    /// Sums `blocks` in input order into the lower-triangular matrix of
    /// dimension `size` (three per block). Missing blocks of the pattern are zero.
    pub fn load(
        &mut self,
        size: usize,
        blocks: &[Block],
        iteration: usize,
    ) -> Result<(), ClothError> {
        let n = size / 3;
        if n * 3 != size || blocks.iter().any(|b| b.row >= n || b.col > b.row) {
            return Err(failure("matrix assembly", iteration));
        }
        if self.columns.len() != n {
            *self = Self {
                columns: vec![Vec::new(); n],
                workers: std::sync::Arc::clone(&self.workers),
                ..Self::default()
            };
        }
        if self.col_ptr.is_empty() {
            self.grow(blocks.iter().map(|b| (b.row, b.col)));
        }
        // New block pairs grow the pattern; the values are then summed again.
        loop {
            let missing = self.scatter(blocks)?;
            if missing.is_empty() {
                return Ok(());
            }
            self.grow(missing.into_iter());
        }
    }

    /// Adds block pairs to the pattern and rebuilds the CSC structure.
    fn grow(&mut self, pairs: impl Iterator<Item = (usize, usize)>) {
        for (row, col) in pairs {
            let rows = &mut self.columns[col];
            if let Err(k) = rows.binary_search(&row) {
                rows.insert(k, row);
            }
        }
        self.rebuild();
    }

    /// Accumulates `blocks` in input order and returns the block pairs missing
    /// from the pattern (the values are then incomplete). Lanes own disjoint
    /// column ranges, so every value sums its contributions in the same order
    /// as a serial pass.
    fn scatter(&mut self, blocks: &[Block]) -> Result<Vec<(usize, usize)>, ClothError> {
        let n = self.columns.len();
        let (columns, starts) = (&self.columns, &self.starts);
        let add = |values: &mut [Real], base: usize, range: std::ops::Range<usize>| {
            values.fill(0.0);
            let mut missing = Vec::new();
            for b in blocks {
                if !range.contains(&b.col) {
                    continue;
                }
                let Ok(k) = columns[b.col].binary_search(&b.row) else {
                    if missing.last() != Some(&(b.row, b.col)) {
                        missing.push((b.row, b.col));
                    }
                    continue;
                };
                let start = starts[b.col][k];
                for v in 0..3 {
                    let first = if b.row == b.col { v } else { 0 };
                    for u in first..3 {
                        values[start[v] + u - first - base] += b.value[u][v];
                    }
                }
            }
            missing
        };
        let lanes = self.workers.lanes();
        if lanes == 1 || blocks.len() < 4096 {
            return Ok(add(&mut self.values, 0, 0..n));
        }
        let (col_ptr, values) = (&self.col_ptr, &mut self.values);
        let mut rest = &mut values[..];
        let mut parts = Vec::with_capacity(lanes);
        for k in 0..lanes {
            let range = k * n / lanes..(k + 1) * n / lanes;
            let base = col_ptr[3 * range.start];
            let (slice, remaining) = rest.split_at_mut(col_ptr[3 * range.end] - base);
            rest = remaining;
            parts.push((slice, base, range));
        }
        let parts = workers::handoff(parts);
        let missing = self.workers.run(lanes, |k| {
            let (slice, base, range) = workers::take(&parts, k);
            add(slice, base, range)
        })?;
        Ok(missing.into_iter().flatten().collect())
    }

    fn rebuild(&mut self) {
        let n = self.columns.len();
        self.col_ptr.clear();
        self.row_idx.clear();
        self.starts.clear();
        self.col_ptr.push(0);
        let mut starts: Vec<Vec<[usize; 3]>> = self
            .columns
            .iter()
            .map(|rows| vec![[0; 3]; rows.len()])
            .collect();
        for col in 0..n {
            for v in 0..3 {
                for (k, &row) in self.columns[col].iter().enumerate() {
                    starts[col][k][v] = self.row_idx.len();
                    let first = if row == col { v } else { 0 };
                    self.row_idx.extend((first..3).map(|u| 3 * row + u));
                }
                self.col_ptr.push(self.row_idx.len());
            }
        }
        self.starts = starts;
        self.values.clear();
        self.values.resize(self.row_idx.len(), 0.0);
        self.factor = None;
    }

    /// Symbolic analysis of the current pattern. The previous ordering is kept
    /// when it does not enlarge the factor by more than a tenth.
    fn analyze(&self) -> Result<SymbolicCholesky<usize>, faer::sparse::FaerError> {
        let pattern = self.matrix().symbolic();
        let n = pattern.nrows();
        if let Some((forward, previous)) = &self.ordering
            && forward.len() == n
        {
            let mut inverse = vec![0; n];
            for (i, &p) in forward.iter().enumerate() {
                inverse[p] = i;
            }
            let reused = factorize_symbolic_cholesky(
                pattern,
                Side::Lower,
                SymmetricOrdering::Custom(PermRef::new_checked(forward, &inverse, n)),
                Default::default(),
            )?;
            if reused.len_val() <= previous + previous / 10 {
                return Ok(reused);
            }
        }
        factorize_symbolic_cholesky(
            pattern,
            Side::Lower,
            SymmetricOrdering::Amd,
            Default::default(),
        )
    }

    fn matrix(&self) -> SparseColMatRef<'_, usize, Real> {
        let n = self.col_ptr.len() - 1;
        SparseColMatRef::new(
            SymbolicSparseColMatRef::new_checked(n, n, &self.col_ptr, None, &self.row_idx),
            &self.values,
        )
    }

    /// Factorizes the loaded matrix, reusing the symbolic analysis while the
    /// pattern is unchanged, and overwrites `rhs` with the solution.
    pub fn solve(&mut self, rhs: &mut [Real], iteration: usize) -> Result<(), ClothError> {
        if self.col_ptr.len() != rhs.len() + 1 {
            return Err(failure("factorization", iteration));
        }
        if rhs.is_empty() {
            return Ok(());
        }
        if self.factor.is_none() {
            let symbolic = self
                .analyze()
                .map_err(|_| failure("factorization", iteration))?;
            self.ordering = symbolic
                .perm()
                .map(|perm| (perm.arrays().0.to_vec(), symbolic.len_val()));
            let numeric = MemBuffer::try_new(
                symbolic.factorize_numeric_llt_scratch::<Real>(Par::Seq, Default::default()),
            )
            .map_err(|_| failure("factorization", iteration))?;
            let solve = MemBuffer::try_new(symbolic.solve_in_place_scratch::<Real>(1, Par::Seq))
                .map_err(|_| failure("factorization", iteration))?;
            self.factor = Some(Factor {
                values: vec![0.0; symbolic.len_val()],
                symbolic,
                numeric,
                solve,
            });
        }
        let matrix = SparseColMatRef::new(
            SymbolicSparseColMatRef::new_checked(
                rhs.len(),
                rhs.len(),
                &self.col_ptr,
                None,
                &self.row_idx,
            ),
            &self.values,
        );
        let factor = self.factor.as_mut().unwrap();
        factor
            .symbolic
            .factorize_numeric_llt::<Real>(
                &mut factor.values,
                matrix,
                Side::Lower,
                LltRegularization::default(),
                Par::Seq,
                MemStack::new(&mut factor.numeric),
                Default::default(),
            )
            .map_err(|_| failure("factorization", iteration))?;
        let n = rhs.len();
        LltRef::new(&factor.symbolic, &factor.values).solve_in_place_with_conj(
            Conj::No,
            MatMut::from_column_major_slice_mut(rhs, n, 1),
            Par::Seq,
            MemStack::new(&mut factor.solve),
        );
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn block(row: usize, col: usize, scale: Real) -> Block {
        Block {
            row,
            col,
            value: std::array::from_fn(|u| {
                std::array::from_fn(|v| {
                    if row == col && u == v {
                        10.0 * scale
                    } else {
                        // Symmetric within diagonal blocks; arbitrary otherwise.
                        let (a, b) = if row == col {
                            (u.max(v), u.min(v))
                        } else {
                            (u, v)
                        };
                        scale * (0.1 + 0.05 * a as Real - 0.03 * b as Real)
                    }
                })
            }),
        }
    }
    /// Dense lower triangle from the documented block semantics.
    fn dense(size: usize, blocks: &[Block]) -> Vec<Vec<Real>> {
        let mut m = vec![vec![0.0; size]; size];
        for b in blocks {
            for u in 0..3 {
                for v in 0..3 {
                    let (r, c) = (3 * b.row + u, 3 * b.col + v);
                    if r >= c {
                        m[r][c] += b.value[u][v];
                    }
                }
            }
        }
        m
    }
    fn check(system: &mut SparseSystem, size: usize, blocks: &[Block]) {
        system.load(size, blocks, 0).unwrap();
        let expected = dense(size, blocks);
        let matrix = system.matrix();
        let (ptr, rows, vals) = (
            matrix.symbolic().col_ptr(),
            matrix.symbolic().row_idx(),
            matrix.val(),
        );
        let mut stored = vec![vec![0.0; size]; size];
        for c in 0..size {
            let column = &rows[ptr[c]..ptr[c + 1]];
            assert!(column.windows(2).all(|w| w[0] < w[1]), "sorted rows");
            for (k, &r) in column.iter().enumerate() {
                assert!(r >= c);
                stored[r][c] = vals[ptr[c] + k];
            }
        }
        for r in 0..size {
            for c in 0..=r {
                assert!((stored[r][c] - expected[r][c]).abs() <= 1e-12);
            }
        }
        let exact: Vec<Real> = (0..size).map(|i| (i + 1) as Real * 0.7).collect();
        let mut solution: Vec<Real> = (0..size)
            .map(|i| {
                (0..size)
                    .map(|j| expected[i.max(j)][i.min(j)] * exact[j])
                    .sum::<Real>()
            })
            .collect();
        system.solve(&mut solution, 0).unwrap();
        for i in 0..size {
            assert!((solution[i] - exact[i]).abs() < 1e-10);
        }
    }

    #[test]
    fn duplicate_blocks_sum_in_assembly_order_and_diagonals_keep_the_lower_triangle() {
        let mut system = SparseSystem::default();
        let mut blocks = vec![
            block(0, 0, 1.0),
            block(1, 1, 1.0),
            block(2, 2, 1.0),
            block(2, 0, 0.5),
            block(0, 0, 0.25),
            block(2, 0, 0.125),
            block(1, 0, 0.3),
        ];
        check(&mut system, 9, &blocks);
        blocks.swap(3, 6);
        check(&mut system, 9, &blocks);
        for b in &mut blocks {
            for row in &mut b.value {
                for v in row {
                    *v *= 1.7;
                }
            }
        }
        check(&mut system, 9, &blocks);
        // The same inputs always produce the same bits.
        let mut fresh = SparseSystem::default();
        fresh.load(9, &blocks, 0).unwrap();
        assert_eq!(
            fresh.values.iter().map(|v| v.to_bits()).collect::<Vec<_>>(),
            system
                .values
                .iter()
                .map(|v| v.to_bits())
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn the_pattern_grows_within_a_step_and_resets_with_the_dimension() {
        let mut system = SparseSystem::default();
        let mut blocks: Vec<_> = (0..3).map(|i| block(i, i, 1.0)).collect();
        blocks.push(block(1, 0, 0.2));
        check(&mut system, 9, &blocks);
        let first = system.row_idx.clone();
        // A released contact keeps its (zero) block and the symbolic factor.
        blocks.pop();
        check(&mut system, 9, &blocks);
        assert_eq!(system.row_idx, first);
        // A new contact grows the pattern.
        blocks.push(block(2, 0, 0.2));
        check(&mut system, 9, &blocks);
        assert!(system.row_idx.len() > first.len());
        // Releasing a vertex changes the dimension and starts a new pattern.
        blocks.retain(|b| b.row < 2);
        check(&mut system, 6, &blocks);
    }

    #[test]
    fn grown_patterns_reuse_or_replace_the_ordering_and_solve_correctly() {
        let n = 90;
        let mut system = SparseSystem::default();
        let mut blocks: Vec<_> = (0..n).map(|i| block(i, i, 1.0)).collect();
        for i in 1..n {
            blocks.push(block(i, i - 1, 0.1));
        }
        check(&mut system, 3 * n, &blocks);
        let first = system.ordering.clone().unwrap();
        // A nearby new pair keeps the ordering; the factor grows only slightly.
        blocks.push(block(2, 0, 0.05));
        check(&mut system, 3 * n, &blocks);
        let second = system.ordering.clone().unwrap();
        assert_eq!(first.0, second.0);
        assert!(second.1 >= first.1);
        // Dense coupling of every block to the first replaces the ordering.
        for i in 2..n {
            blocks.push(block(i, 0, 0.01));
            blocks.push(block(i, 1, 0.01));
        }
        check(&mut system, 3 * n, &blocks);
        let third = system.ordering.clone().unwrap();
        assert!(third.1 > second.1);
    }

    #[test]
    fn a_failed_numeric_factor_is_not_reused_on_retry() {
        let mut system = SparseSystem::default();
        let mut negative = block(0, 0, 1.0);
        negative.value[0][0] = -1.0;
        system.load(3, &[negative], 0).unwrap();
        assert!(system.solve(&mut [1.0, 1.0, 1.0], 0).is_err());
        check(&mut system, 3, &[block(0, 0, 1.0)]);
    }

    #[test]
    fn lane_accumulation_matches_serial_bits_for_large_systems() {
        // A 40×40 grid-like block pattern with duplicates, beyond the lane threshold.
        let n = 1600;
        let mut blocks = Vec::new();
        for i in 0..n {
            blocks.push(block(i, i, 1.0 + (i % 7) as Real * 0.01));
            if i >= 1 {
                blocks.push(block(i, i - 1, 0.1));
            }
            if i >= 40 {
                blocks.push(block(i, i - 40, 0.05));
                blocks.push(block(i, i - 40, 0.025)); // duplicate pair
            }
        }
        assert!(blocks.len() >= 4096);
        let mut serial = SparseSystem::new(std::sync::Arc::new(workers::Workers::new(1)));
        let mut parallel = SparseSystem::new(std::sync::Arc::new(workers::Workers::new(4)));
        for pass in 0..2 {
            if pass == 1 {
                blocks.push(block(n - 1, 0, 0.001)); // grows the pattern
            }
            serial.load(3 * n, &blocks, pass).unwrap();
            parallel.load(3 * n, &blocks, pass).unwrap();
            assert_eq!(serial.row_idx, parallel.row_idx);
            assert_eq!(
                serial
                    .values
                    .iter()
                    .map(|v| v.to_bits())
                    .collect::<Vec<_>>(),
                parallel
                    .values
                    .iter()
                    .map(|v| v.to_bits())
                    .collect::<Vec<_>>()
            );
            let rhs: Vec<Real> = (0..3 * n).map(|i| (i as Real * 0.3).sin()).collect();
            let (mut a, mut b) = (rhs.clone(), rhs);
            serial.solve(&mut a, pass).unwrap();
            parallel.solve(&mut b, pass).unwrap();
            assert_eq!(a, b);
        }
        // A pool creation failure leaves the input alone and can be retried.
        let mut fresh = SparseSystem::new(std::sync::Arc::new(workers::Workers::new(4)));
        let result = workers::fault::after(0, || fresh.load(3 * n, &blocks, 2));
        assert!(matches!(
            result,
            Err(ClothError::ImplicitWorkerSpawnFailed(_))
        ));
        fresh.load(3 * n, &blocks, 3).unwrap();
        assert_eq!(serial.values, fresh.values);
    }

    #[test]
    fn invalid_blocks_and_dimensions_are_rejected() {
        let mut system = SparseSystem::default();
        for (size, blocks) in [
            (6, vec![block(2, 0, 1.0)]),
            (6, vec![block(0, 1, 1.0)]),
            (5, vec![block(0, 0, 1.0)]),
        ] {
            assert!(system.load(size, &blocks, 3).is_err());
        }
        system.load(0, &[], 0).unwrap();
        system.solve(&mut [], 0).unwrap();
        check(&mut system, 3, &[block(0, 0, 1.0)]);
        assert!(system.solve(&mut [1.0; 6], 0).is_err());
    }

    #[test]
    fn nonfinite_and_signed_zero_values_are_preserved_before_factorization() {
        let mut b = block(0, 0, 1.0);
        b.value[1][1] = Real::NAN;
        b.value[2][2] = Real::INFINITY;
        let mut system = SparseSystem::default();
        system.load(3, &[b], 9).unwrap();
        // Column-major lower triangle: (0,0) (1,0) (2,0) (1,1) (2,1) (2,2).
        assert!(system.values[3].is_nan());
        assert_eq!(system.values[5], Real::INFINITY);
        assert!(system.solve(&mut [1.0; 3], 9).is_err());
    }
}
