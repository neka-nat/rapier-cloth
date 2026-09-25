//! A surface contact coupled to independent one-particle rigid supports.
//!
//! This is a local normal complementarity solve, not a new timestep or global
//! iteration. Eliminate the support rows from the arrowhead Gram matrix. Their
//! multipliers are piecewise linear in the surface multiplier, so at most four
//! breakpoints (five intervals) determine the solution. Inactive supports release.

use crate::{ClothError, Real, SurfaceContact, SurfaceFeature, Vec3};
use glam::DVec3;

#[derive(Clone, Copy)]
pub(crate) struct Support {
    pub contact: SurfaceContact,
    pub lambda: Real,
}

pub(crate) fn single_particle(contact: &SurfaceContact) -> Option<(u32, Real)> {
    if !contact
        .key
        .features
        .iter()
        .any(|f| matches!(f, SurfaceFeature::External { .. }))
    {
        return None;
    }
    let mut support = None;
    for i in 0..4 {
        if contact.weights[i] != 0.0 {
            if support.is_some() || contact.weights[i] < 0.0 {
                return None;
            }
            support = Some((contact.particles[i], contact.weights[i]));
        }
    }
    support
}

#[cfg(not(feature = "f64"))]
fn wide(value: Real) -> f64 {
    f64::from(value)
}
#[cfg(feature = "f64")]
fn wide(value: Real) -> f64 {
    value
}
fn double(v: Vec3) -> DVec3 {
    DVec3::new(wide(v.x), wide(v.y), wide(v.z))
}
fn real(v: DVec3) -> Vec3 {
    Vec3::new(v.x as Real, v.y as Real, v.z as Real)
}

fn gap(c: SurfaceContact, positions: &[Vec3]) -> f64 {
    let mut relative = -double(c.offset);
    for i in 0..4 {
        if c.weights[i] != 0.0 {
            relative += double(positions[c.particles[i] as usize]) * wide(c.weights[i]);
        }
    }
    double(c.normal).dot(relative) - wide(c.separation)
}

/// Return new surface/support multipliers. No position is changed on failure.
pub(crate) fn project(
    contact: SurfaceContact,
    positions: &mut [Vec3],
    inverse_masses: &[Real],
    old_lambda: Real,
    supports: [Option<Support>; 4],
) -> Result<(Real, [Real; 4]), ClothError> {
    let mut mass = [0.0; 4];
    let mut gradient = [DVec3::ZERO; 4];
    let mut support_gradient = [DVec3::ZERO; 4];
    let mut diagonal = [0.0; 4];
    let mut coupling = [0.0; 4];
    let mut q = [0.0; 4];
    let mut before = [0.0; 4];
    let mut a = 0.0;
    for i in 0..4 {
        if contact.weights[i] == 0.0 {
            continue;
        }
        let particle = contact.particles[i] as usize;
        mass[i] = wide(inverse_masses[particle]);
        gradient[i] = double(contact.normal) * wide(contact.weights[i]);
        a += mass[i] * gradient[i].length_squared();
        if let Some(s) = supports[i] {
            let Some((index, weight)) = single_particle(&s.contact) else {
                return Err(ClothError::InvalidSurfaceContact(
                    "invalid normal-block support",
                ));
            };
            if index as usize != particle || mass[i] <= 0.0 {
                return Err(ClothError::InvalidSurfaceContact(
                    "normal-block particle or mass",
                ));
            }
            support_gradient[i] = double(s.contact.normal) * wide(weight);
            diagonal[i] = mass[i] * support_gradient[i].length_squared();
            coupling[i] = mass[i] * gradient[i].dot(support_gradient[i]);
            before[i] = wide(s.lambda);
            q[i] = gap(s.contact, positions)
                - coupling[i] * wide(old_lambda)
                - diagonal[i] * before[i];
        }
    }
    let q0 = gap(contact, positions)
        - a * wide(old_lambda)
        - (0..4).map(|i| coupling[i] * before[i]).sum::<f64>();
    if !a.is_finite()
        || !q0.is_finite()
        || a <= 0.0
        || q.iter()
            .chain(&coupling)
            .chain(&diagonal)
            .any(|v| !v.is_finite())
    {
        return Err(ClothError::NonFiniteState);
    }
    let support_lambda = |t: f64, i: usize| {
        if diagonal[i] == 0.0 {
            0.0
        } else {
            (-(q[i] + coupling[i] * t) / diagonal[i]).max(0.0)
        }
    };
    let value = |t: f64| {
        q0 + a * t
            + (0..4)
                .map(|i| coupling[i] * support_lambda(t, i))
                .sum::<f64>()
    };
    let mut breaks = [f64::INFINITY; 5];
    for i in 0..4 {
        if coupling[i] != 0.0 {
            let t = -q[i] / coupling[i];
            if t > 0.0 {
                breaks[i] = t;
            }
        }
    }
    breaks.sort_by(f64::total_cmp);
    let mut t = 0.0;
    let mut solution = None;
    for end in breaks {
        let residual = value(t);
        if !residual.is_finite() {
            return Err(ClothError::NonFiniteState);
        }
        if residual >= 0.0 {
            solution = Some(t);
            break;
        }
        // Accumulate projected-gradient norms instead of subtracting nearly
        // equal Gram entries for a support parallel to the surface normal.
        let mut slope = 0.0;
        for i in 0..4 {
            // Use the interval to the right of the breakpoint. Cancellation in
            // q+c*t could otherwise retain a support which has just released.
            let active = diagonal[i] > 0.0
                && if coupling[i] > 0.0 {
                    t < -q[i] / coupling[i]
                } else if coupling[i] < 0.0 {
                    t >= -q[i] / coupling[i]
                } else {
                    q[i] < 0.0
                };
            let g = if active {
                gradient[i]
                    - support_gradient[i]
                        * (gradient[i].dot(support_gradient[i])
                            / support_gradient[i].length_squared())
            } else {
                gradient[i]
            };
            slope += mass[i] * g.length_squared();
        }
        if slope > 0.0 {
            let candidate = t - residual / slope;
            if candidate.is_finite() && candidate <= end {
                solution = Some(candidate);
                break;
            }
        }
        if !end.is_finite() {
            break;
        }
        t = end;
    }
    let next = solution.ok_or(ClothError::InfeasibleSurfaceContact)?;
    let next_support = std::array::from_fn::<_, 4, _>(|i| support_lambda(next, i));
    let mut updated = [Vec3::ZERO; 4];
    for i in 0..4 {
        if contact.weights[i] == 0.0 {
            continue;
        }
        let change = gradient[i] * (next - wide(old_lambda))
            + support_gradient[i] * (next_support[i] - before[i]);
        updated[i] = real(double(positions[contact.particles[i] as usize]) + change * mass[i]);
    }
    let next = next as Real;
    let next_support = next_support.map(|v| v as Real);
    if !next.is_finite()
        || next < 0.0
        || next_support.iter().any(|v| !v.is_finite() || *v < 0.0)
        || updated.iter().any(|p| !p.is_finite())
    {
        return Err(ClothError::NonFiniteState);
    }
    for i in 0..4 {
        if contact.weights[i] != 0.0 {
            positions[contact.particles[i] as usize] = updated[i];
        }
    }
    Ok((next, next_support))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::SurfaceContactKey;
    fn pair() -> SurfaceContact {
        SurfaceContact {
            key: SurfaceContactKey {
                other_cloth: None,
                features: [SurfaceFeature::Vertex(0), SurfaceFeature::Vertex(1)],
            },
            particles: [0, 1, 0, 0],
            weights: [1.0, -1.0, 0.0, 0.0],
            normal: Vec3::Y,
            offset: Vec3::ZERO,
            surface_velocity: Vec3::ZERO,
            separation: 1.0,
            static_friction: 0.5,
            kinetic_friction: 0.2,
        }
    }
    fn plane(i: u32, normal: Vec3) -> SurfaceContact {
        SurfaceContact {
            key: SurfaceContactKey {
                other_cloth: None,
                features: [
                    SurfaceFeature::Vertex(i),
                    SurfaceFeature::External {
                        object: 0,
                        feature: 0,
                    },
                ],
            },
            particles: [i, 0, 0, 0],
            weights: [1.0, 0.0, 0.0, 0.0],
            normal,
            separation: 0.5,
            ..pair()
        }
    }
    #[test]
    fn support_carries_both_masses_and_repeated_block_is_idempotent() {
        let mut p = [Vec3::Y * 1.4, Vec3::Y * 0.4];
        let floor = plane(1, Vec3::Y);
        let (lambda, loads) = project(
            pair(),
            &mut p,
            &[1.0, 0.5],
            0.0,
            [
                None,
                Some(Support {
                    contact: floor,
                    lambda: 0.0,
                }),
                None,
                None,
            ],
        )
        .unwrap();
        assert!((lambda - 0.1).abs() < 1e-6 && (loads[1] - 0.3).abs() < 1e-6);
        assert!(p[0].distance(Vec3::Y * 1.5) < 1e-6 && p[1].distance(Vec3::Y * 0.5) < 1e-6);
        let before = p;
        let (next, next_loads) = project(
            pair(),
            &mut p,
            &[1.0, 0.5],
            lambda,
            [
                None,
                Some(Support {
                    contact: floor,
                    lambda: loads[1],
                }),
                None,
                None,
            ],
        )
        .unwrap();
        assert!((next - lambda).abs() < 1e-6 && (next_loads[1] - loads[1]).abs() < 1e-6);
        assert!(p.iter().zip(before).all(|(a, b)| a.distance(b) < 1e-6));
    }
    #[test]
    fn upward_separation_releases_the_upper_support() {
        let mut p = [Vec3::Y * 0.4, Vec3::Y * 0.4];
        let supports = [
            Some(Support {
                contact: plane(0, Vec3::Y),
                lambda: 0.0,
            }),
            Some(Support {
                contact: plane(1, Vec3::Y),
                lambda: 0.0,
            }),
            None,
            None,
        ];
        let (lambda, loads) = project(pair(), &mut p, &[1.0, 0.5], 0.0, supports).unwrap();
        assert!((lambda - 1.1).abs() < 1e-6);
        assert_eq!(loads[0], 0.0);
        assert!((loads[1] - 1.3).abs() < 1e-6);
        assert!(p[0].distance(Vec3::Y * 1.5) < 1e-6 && p[1].distance(Vec3::Y * 0.5) < 1e-6);
    }
    #[test]
    fn angled_support_preserves_free_motion_and_balances_its_reaction() {
        let normal = Vec3::new(1.0, 1.0, 0.0).normalize();
        let floor = plane(1, normal);
        let lower = normal * 0.5;
        let mut p = [lower + Vec3::Y * 0.8, lower];
        let before = p;
        let (lambda, loads) = project(
            pair(),
            &mut p,
            &[1.0, 0.5],
            0.0,
            [
                None,
                Some(Support {
                    contact: floor,
                    lambda: 0.0,
                }),
                None,
                None,
            ],
        )
        .unwrap();
        assert!(lambda > 0.0 && loads[1] > 0.0);
        assert!(pair().gap(&p).abs() < 1e-6 && floor.gap(&p).abs() < 1e-6);
        assert!(p[1].distance(before[1]) > 0.01);
        let momentum = (p[0] - before[0]) + (p[1] - before[1]) * 2.0;
        assert!(momentum.distance(normal * loads[1]) < 1e-6);
    }
    #[test]
    fn contradictory_supports_fail_without_a_partial_update() {
        let mut p = [Vec3::ZERO, Vec3::ZERO];
        // Upper point is constrained below y=-0.5; lower above y=+0.5,
        // while the pair requires the upper point to be one metre higher.
        let before = p;
        let result = project(
            pair(),
            &mut p,
            &[1.0, 1.0],
            0.0,
            [
                Some(Support {
                    contact: plane(0, -Vec3::Y),
                    lambda: 0.0,
                }),
                Some(Support {
                    contact: plane(1, Vec3::Y),
                    lambda: 0.0,
                }),
                None,
                None,
            ],
        );
        assert!(matches!(result, Err(ClothError::InfeasibleSurfaceContact)));
        assert_eq!(p, before);
    }

    // Independent small LCP reference: enumerate active sets and solve each
    // dense Gram submatrix with pivoted elimination. No arrowhead elimination.
    fn reference(
        rows: &[(SurfaceContact, Real)],
        p: &[Vec3; 4],
        inv: &[Real; 4],
    ) -> Option<[DVec3; 4]> {
        let n = rows.len();
        let gradients: Vec<[DVec3; 4]> = rows
            .iter()
            .map(|(c, _)| {
                let mut g = [DVec3::ZERO; 4];
                for k in 0..4 {
                    if c.weights[k] != 0.0 {
                        g[c.particles[k] as usize] += double(c.normal) * wide(c.weights[k]);
                    }
                }
                g
            })
            .collect();
        let mut gram = [[0.0; 5]; 5];
        let mut rhs = [0.0; 5];
        for i in 0..n {
            let c = rows[i].0;
            let mut relative = -double(c.offset);
            for (k, &position) in p.iter().enumerate() {
                relative += double(position)
                    * wide(
                        c.weights
                            .iter()
                            .enumerate()
                            .filter(|&(j, _)| c.particles[j] as usize == k)
                            .map(|(_, w)| *w)
                            .sum(),
                    );
            }
            rhs[i] = double(c.normal).dot(relative) - wide(c.separation);
            for j in 0..n {
                gram[i][j] = (0..4)
                    .map(|k| wide(inv[k]) * gradients[i][k].dot(gradients[j][k]))
                    .sum();
                rhs[i] -= gram[i][j] * wide(rows[j].1);
            }
        }
        for mask in 0..(1 << n) {
            let active: Vec<_> = (0..n).filter(|i| mask & (1 << i) != 0).collect();
            let mut matrix = [[0.0; 6]; 5];
            let count = active.len();
            for (r, &i) in active.iter().enumerate() {
                for (c, &j) in active.iter().enumerate() {
                    matrix[r][c] = gram[i][j];
                }
                matrix[r][count] = -rhs[i];
            }
            let mut valid = true;
            for column in 0..count {
                let pivot = (column..count)
                    .max_by(|&a, &b| matrix[a][column].abs().total_cmp(&matrix[b][column].abs()))
                    .unwrap();
                if matrix[pivot][column].abs() < 1e-12 {
                    valid = false;
                    break;
                }
                matrix.swap(pivot, column);
                let divisor = matrix[column][column];
                for entry in &mut matrix[column][column..=count] {
                    *entry /= divisor;
                }
                let pivot_row = matrix[column];
                for (r, row) in matrix[..count].iter_mut().enumerate() {
                    if r == column {
                        continue;
                    }
                    let factor = row[column];
                    for (entry, &pivot) in row[column..=count]
                        .iter_mut()
                        .zip(&pivot_row[column..=count])
                    {
                        *entry -= factor * pivot;
                    }
                }
            }
            if !valid {
                continue;
            }
            let mut lambda = [0.0; 5];
            for (r, &i) in active.iter().enumerate() {
                lambda[i] = matrix[r][count];
            }
            if (0..n).any(|i| {
                lambda[i] < -1e-8
                    || rhs[i] + (0..n).map(|j| gram[i][j] * lambda[j]).sum::<f64>() < -1e-8
            }) {
                continue;
            }
            return Some(std::array::from_fn(|k| {
                double(p[k])
                    + (0..n)
                        .map(|i| gradients[i][k] * (lambda[i] - wide(rows[i].1)) * wide(inv[k]))
                        .sum::<DVec3>()
            }));
        }
        None
    }

    #[test]
    fn blocks_agree_with_exhaustive_active_sets_including_unloading() {
        let mut seed = 0x312fd65u64;
        let mut sample = || {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (f64::from((seed >> 32) as u32) / f64::from(u32::MAX)) as Real
        };
        let mut solved = 0;
        for case in 0..500 {
            let c = SurfaceContact {
                particles: [0, 1, 2, 3],
                weights: [1.0, -0.2, -0.3, -0.5],
                normal: Vec3::new(sample() - 0.5, 1.0, sample() - 0.5).normalize(),
                ..pair()
            };
            let p =
                std::array::from_fn(|_| Vec3::new(sample() - 0.5, sample() - 0.5, sample() - 0.5));
            let inv = std::array::from_fn(|_| 0.2 + sample() * 2.0);
            let old = sample() * 0.2;
            let supports = std::array::from_fn(|i| {
                if sample() < 0.2 {
                    None
                } else {
                    let sign = if sample() < 0.2 { -1.0 } else { 1.0 };
                    Some(Support {
                        contact: plane(
                            i as u32,
                            Vec3::new(sample() - 0.5, sign, sample() - 0.5).normalize(),
                        ),
                        lambda: sample() * 0.2,
                    })
                }
            });
            let mut rows = vec![(c, old)];
            rows.extend(supports.iter().flatten().map(|s| (s.contact, s.lambda)));
            let expected = reference(&rows, &p, &inv);
            let mut actual = p;
            let result = project(c, &mut actual, &inv, old, supports);
            match (result, expected) {
                (Ok((lambda, loads)), Some(expected)) => {
                    solved += 1;
                    assert!(
                        actual
                            .iter()
                            .zip(expected)
                            .all(|(a, b)| double(*a).distance(b) < 2e-5),
                        "case={case}, actual={actual:?}, reference={expected:?}"
                    );
                    assert!(
                        c.gap(&actual) >= -2e-5 && (lambda * c.gap(&actual)).abs() < 2e-4,
                        "case={case}"
                    );
                    for i in 0..4 {
                        if let Some(s) = supports[i] {
                            assert!(
                                s.contact.gap(&actual) >= -2e-5
                                    && (loads[i] * s.contact.gap(&actual)).abs() < 2e-4,
                                "case={case}"
                            );
                        }
                    }
                }
                (Err(ClothError::InfeasibleSurfaceContact), None) => assert_eq!(actual, p),
                (a, b) => panic!("case={case}, result={a:?}, reference={b:?}"),
            }
        }
        assert!(solved > 400, "only {solved} feasible cases");
    }
}
