//! Experimental global implicit shell solver. Enable with the `implicit` feature.
//! The f64 path supports hard particle grasps and finite-thickness surface contact.
//! Neo-Hookean membrane, discrete shell bending, finite-thickness barrier and
//! lagged regularized Coulomb friction share one position objective.
#![allow(clippy::needless_range_loop)]
// Element tensors use explicit component indices to keep their contractions
// visible; the sparse global system uses checked indices from validated meshes.
mod elements;
mod settings;
mod sparse;
mod workers;
use crate::collision::self_collision::SelfCollision;
use crate::constraints::bend::{angle_and_gradients, angle_difference};
use crate::{
    Cloth, ClothError, ContactMotion, ContactSource, ContactStage, Real, SolverSettings,
    StepReport, SurfaceContact, Target, Vec3,
};
use faer::{Mat, prelude::Solve, sparse::Triplet};
use nalgebra::SMatrix;
pub use settings::{ImplicitExecution, ImplicitSettings, ShellMaterial};
use sparse::SparseSystem;

#[derive(Clone)]
struct Triangle {
    ids: [usize; 3],
    b: [[Real; 2]; 3],
    volume: Real,
}
#[derive(Clone)]
struct Hinge {
    ids: [usize; 4],
    rest: Real,
    stiffness: Real,
}
#[derive(Clone)]
struct Friction {
    contact: SurfaceContact,
    reference: Vec3,
    load: Real,
}
struct Assembly {
    energy: Real,
    gradient: Vec<Real>,
    triplets: Vec<Triplet<usize, usize, Real>>,
}

fn failure(phase: &'static str, iterations: usize) -> ClothError {
    ClothError::ImplicitSolverFailed { phase, iterations }
}

/// Returns barrier energy, its gap derivative and second derivative.
fn barrier(gap: Real, band: Real, stiffness: Real) -> Option<[Real; 3]> {
    if gap <= 0.0 || !gap.is_finite() {
        return None;
    }
    if gap >= band {
        return Some([0.0; 3]);
    }
    let d = gap - band;
    let l = (gap / band).ln();
    Some([
        -stiffness * d * d * l,
        -stiffness * (2.0 * d * l + d * d / gap),
        -stiffness * (2.0 * l + 4.0 * d / gap - d * d / (gap * gap)),
    ])
}

/// Analytic Neo-Hookean membrane derivatives in the six components of F.
fn membrane(
    f: [Vec3; 2],
    mu: Real,
    lambda: Real,
    hessian: bool,
) -> Option<(Real, [Vec3; 2], SMatrix<Real, 6, 6>)> {
    let a = f[0].length_squared();
    let b = f[0].dot(f[1]);
    let c = f[1].length_squared();
    let det = a * c - b * b;
    if det <= 1e-16 || !det.is_finite() {
        return None;
    }
    let inv = [[c / det, -b / det], [-b / det, a / det]];
    let logj = 0.5 * det.ln();
    let q = lambda * logj - mu;
    let fc = [
        f[0] * inv[0][0] + f[1] * inv[1][0],
        f[0] * inv[0][1] + f[1] * inv[1][1],
    ];
    let p = [f[0] * mu + fc[0] * q, f[1] * mu + fc[1] * q];
    let e = 0.5 * mu * (a + c - 2.0) - mu * logj + 0.5 * lambda * logj * logj;
    let mut h = SMatrix::<Real, 6, 6>::zeros();
    if hessian {
        for j in 0..6 {
            let mut df = [Vec3::ZERO; 2];
            df[j / 3][j % 3] = 1.0;
            let dc = [
                [2.0 * f[0].dot(df[0]), f[0].dot(df[1]) + f[1].dot(df[0])],
                [f[0].dot(df[1]) + f[1].dot(df[0]), 2.0 * f[1].dot(df[1])],
            ];
            let dj = fc[0].dot(df[0]) + fc[1].dot(df[1]);
            let mut di = [[0.0; 2]; 2];
            for k in 0..2 {
                for l in 0..2 {
                    for u in 0..2 {
                        for v in 0..2 {
                            di[k][l] -= inv[k][u] * dc[u][v] * inv[v][l];
                        }
                    }
                }
            }
            for k in 0..2 {
                let dp = df[k] * mu
                    + fc[k] * (lambda * dj)
                    + (df[0] * inv[0][k] + df[1] * inv[1][k] + f[0] * di[0][k] + f[1] * di[1][k])
                        * q;
                for axis in 0..3 {
                    h[(k * 3 + axis, j)] = dp[axis];
                }
            }
        }
    }
    Some((e, p, h))
}

struct Model<'a> {
    cloth: &'a Cloth,
    triangles: Vec<Triangle>,
    hinges: Vec<Hinge>,
    dofs: Vec<Option<usize>>,
    count: usize,
    prediction: Vec<Vec3>,
    mu: Real,
    lambda: Real,
    h: Real,
    band: Real,
    barrier_stiffness: Real,
    implicit: ImplicitSettings,
}
impl<'a> Model<'a> {
    fn new(
        cloth: &'a Cloth,
        h: Real,
        gravity: Vec3,
        targets: &[Target],
        implicit: ImplicitSettings,
    ) -> Result<Self, ClothError> {
        let config = cloth.contact_settings.ok_or(ClothError::InvalidParameter(
            "implicit solver requires surface settings",
        ))?;
        let young = implicit.material.youngs_modulus;
        let nu = implicit.material.poisson_ratio;
        let t = implicit.material.thickness;
        let mu = young / (2.0 * (1.0 + nu));
        let lambda = young * nu / (1.0 - nu * nu);
        let mut count = 0;
        let mut fixed = vec![false; cloth.positions.len()];
        for &i in cloth.pins.keys() {
            fixed[i as usize] = true;
        }
        for target in targets {
            if target.compliance != 0.0 {
                return Err(ClothError::InvalidParameter(
                    "implicit solver currently supports hard particle targets only",
                ));
            }
            if target.particle as usize >= fixed.len() || !target.position.is_finite() {
                return Err(ClothError::InvalidParameter("implicit target"));
            }
            if fixed[target.particle as usize] {
                return Err(ClothError::ConflictingTarget(target.particle));
            }
            fixed[target.particle as usize] = true;
        }
        let dofs = fixed
            .iter()
            .map(|&fixed| {
                if fixed {
                    None
                } else {
                    let i = count;
                    count += 3;
                    Some(i)
                }
            })
            .collect();
        let rest = cloth.mesh.rest_positions();
        let triangles = cloth
            .mesh
            .triangles()
            .iter()
            .map(|&ids| {
                let ids = ids.map(|i| i as usize);
                let u = rest[ids[1]] - rest[ids[0]];
                let v = rest[ids[2]] - rest[ids[0]];
                let len = u.length();
                let cross = u.cross(v).length();
                let s = u.dot(v) / len;
                let height = cross / len;
                let b1 = [1.0 / len, -s / (len * height)];
                let b2 = [0.0, 1.0 / height];
                Triangle {
                    ids,
                    b: [[-b1[0] - b2[0], -b1[1] - b2[1]], b1, b2],
                    volume: 0.5 * cross * t,
                }
            })
            .collect();
        let rigidity = young * t.powi(3) / (12.0 * (1.0 - nu * nu));
        let hinges = cloth
            .mesh
            .hinges()
            .iter()
            .map(|hinge| {
                let ids = hinge.vertices.map(|i| i as usize);
                let p = ids.map(|i| rest[i]);
                let edge = p[1] - p[0];
                let area2 = edge.cross(p[2] - p[0]).length() + edge.cross(p[3] - p[0]).length();
                Hinge {
                    ids,
                    rest: hinge.rest_angle,
                    stiffness: rigidity * 6.0 * edge.length_squared() / area2,
                }
            })
            .collect();
        let damp = (-cloth.material.damping * h).exp();
        let prediction = cloth
            .positions
            .iter()
            .enumerate()
            .map(|(i, &p)| {
                p + (cloth.velocities[i]
                    + (gravity + cloth.forces[i] * cloth.inverse_masses[i]) * h)
                    * (damp * h)
            })
            .collect();
        Ok(Self {
            cloth,
            triangles,
            hinges,
            dofs,
            count,
            prediction,
            mu,
            lambda,
            h,
            band: config.activation_margin,
            barrier_stiffness: implicit.barrier_stiffness,
            implicit,
        })
    }
    fn assemble(
        &self,
        x: &[Vec3],
        contacts: &[SurfaceContact],
        friction: &[Friction],
        hessian: bool,
    ) -> Result<Assembly, ClothError> {
        let mut a = Assembly {
            energy: 0.0,
            gradient: vec![0.0; self.count],
            triplets: Vec::new(),
        };
        let mut add_gradient = |i: usize, g: Vec3| {
            if let Some(d) = self.dofs[i] {
                for c in 0..3 {
                    a.gradient[d + c] += g[c];
                }
            }
        };
        // Only free, lower-triangular blocks reach the sparse matrix. Check
        // before constructing their tensors, including on energy-only trials.
        let active_block = |i: usize, j: usize| {
            hessian
                && self.dofs[i]
                    .zip(self.dofs[j])
                    .is_some_and(|(di, dj)| di >= dj)
        };
        let mut add_block = |i: usize, j: usize, block: [[Real; 3]; 3]| {
            if hessian && let (Some(di), Some(dj)) = (self.dofs[i], self.dofs[j]) {
                for u in 0..3 {
                    for v in 0..3 {
                        if di + u >= dj + v {
                            a.triplets.push(Triplet::new(di + u, dj + v, block[u][v]));
                        }
                    }
                }
            }
        };
        for i in 0..x.len() {
            let w = self.cloth.masses[i] / (self.h * self.h);
            let d = x[i] - self.prediction[i];
            a.energy += 0.5 * w * d.length_squared();
            add_gradient(i, d * w);
            add_block(i, i, [[w, 0.0, 0.0], [0.0, w, 0.0], [0.0, 0.0, w]]);
        }
        if hessian
            && self.implicit.execution == ImplicitExecution::Parallel4
            && self.triangles.len() >= 1024
        {
            elements::assemble(self, x, &mut a.energy, &mut add_gradient, &mut add_block)?;
        } else {
            for tri in &self.triangles {
                let mut f = [Vec3::ZERO; 2];
                for k in 0..3 {
                    for c in 0..2 {
                        f[c] += x[tri.ids[k]] * tri.b[k][c];
                    }
                }
                let (e, g, hf) = membrane(f, self.mu, self.lambda, hessian)
                    .ok_or(ClothError::DegenerateConstraint)?;
                a.energy += e * tri.volume;
                for k in 0..3 {
                    add_gradient(
                        tri.ids[k],
                        (g[0] * tri.b[k][0] + g[1] * tri.b[k][1]) * tri.volume,
                    );
                }
                if hessian {
                    let eig = ((hf + hf.transpose()) * 0.5).symmetric_eigen();
                    let d =
                        SMatrix::<Real, 6, 6>::from_diagonal(&eig.eigenvalues.map(|v| v.max(0.0)));
                    let hp = eig.eigenvectors * d * eig.eigenvectors.transpose();
                    for i in 0..3 {
                        for j in 0..3 {
                            if !active_block(tri.ids[i], tri.ids[j]) {
                                continue;
                            }
                            let mut block = [[0.0; 3]; 3];
                            for u in 0..3 {
                                for v in 0..3 {
                                    for r in 0..2 {
                                        for s in 0..2 {
                                            block[u][v] += tri.volume
                                                * tri.b[i][r]
                                                * hp[(r * 3 + u, s * 3 + v)]
                                                * tri.b[j][s];
                                        }
                                    }
                                }
                            }
                            add_block(tri.ids[i], tri.ids[j], block);
                        }
                    }
                }
            }
            for hinge in &self.hinges {
                let (angle, g) = angle_and_gradients(hinge.ids.map(|i| x[i]))
                    .ok_or(ClothError::DegenerateConstraint)?;
                let d = angle_difference(angle, hinge.rest);
                let k = hinge.stiffness;
                a.energy += 0.5 * k * d * d;
                for i in 0..4 {
                    add_gradient(hinge.ids[i], g[i] * (k * d));
                    for j in 0..4 {
                        if !active_block(hinge.ids[i], hinge.ids[j]) {
                            continue;
                        }
                        add_block(
                            hinge.ids[i],
                            hinge.ids[j],
                            std::array::from_fn(|u| std::array::from_fn(|v| k * g[i][u] * g[j][v])),
                        );
                    }
                }
            }
        }
        for c in contacts {
            let b = barrier(c.gap(x), self.band, self.barrier_stiffness)
                .ok_or(ClothError::UnresolvedSurfaceContact)?;
            a.energy += b[0];
            for i in 0..4 {
                if c.weights[i] != 0.0 {
                    add_gradient(c.particles[i] as usize, c.normal * (b[1] * c.weights[i]));
                    for j in 0..4 {
                        if c.weights[j] != 0.0
                            && active_block(c.particles[i] as usize, c.particles[j] as usize)
                        {
                            add_block(
                                c.particles[i] as usize,
                                c.particles[j] as usize,
                                std::array::from_fn(|u| {
                                    std::array::from_fn(|v| {
                                        b[2] * c.normal[u]
                                            * c.normal[v]
                                            * c.weights[i]
                                            * c.weights[j]
                                    })
                                }),
                            );
                        }
                    }
                }
            }
        }
        for f in friction {
            let c = f.contact;
            let n = c.normal;
            let delta = c.relative(x) - f.reference;
            let u = delta - n * delta.dot(n);
            let eps = self.implicit.friction_velocity * self.h;
            let len = (u.length_squared() + eps * eps).sqrt();
            let load = c.kinetic_friction * f.load;
            a.energy += load * (len - eps);
            for i in 0..4 {
                if c.weights[i] != 0.0 {
                    add_gradient(c.particles[i] as usize, u * (load / len * c.weights[i]));
                    for j in 0..4 {
                        if c.weights[j] != 0.0
                            && active_block(c.particles[i] as usize, c.particles[j] as usize)
                        {
                            add_block(
                                c.particles[i] as usize,
                                c.particles[j] as usize,
                                std::array::from_fn(|r| {
                                    std::array::from_fn(|s| {
                                        load * c.weights[i]
                                            * c.weights[j]
                                            * ((if r == s { 1.0 } else { 0.0 })
                                                - n[r] * n[s]
                                                - u[r] * u[s] / (len * len))
                                            / len
                                    })
                                }),
                            );
                        }
                    }
                }
            }
        }
        if !a.energy.is_finite() || a.gradient.iter().any(|g| !g.is_finite()) {
            return Err(ClothError::NonFiniteState);
        }
        Ok(a)
    }
}

fn query(
    source: &mut impl ContactSource,
    engine: &mut SelfCollision,
    old: &[Vec3],
    x: &[Vec3],
    stage: ContactStage,
    cloth: &Cloth,
    limit: usize,
) -> Result<Vec<SurfaceContact>, ClothError> {
    let mut particles = Vec::new();
    source.contacts(old, x, cloth.material.contact_radius, stage, &mut particles)?;
    if !particles.is_empty() {
        return Err(ClothError::InvalidParameter(
            "implicit solver requires surface contact callbacks",
        ));
    }
    let mut out = Vec::new();
    source.surface_contacts_with_work(old, x, stage, &mut out, &mut engine.work)?;
    if cloth.contact_settings.unwrap().self_collision {
        engine.generate(old, x, &mut out, false)?;
    }
    out.sort_by_key(|c| c.key);
    out.dedup_by_key(|c| c.key);
    if out.len() > limit {
        return Err(ClothError::ContactBudgetExceeded { limit });
    }
    engine.work.charge(
        crate::CollisionBudgetKind::RetainedContacts,
        out.len(),
        cloth.contact_settings.unwrap().limits,
    )?;
    for c in &out {
        c.validate(x.len())?;
    }
    Ok(out)
}

fn external_fraction(
    source: &mut impl ContactSource,
    motion: ContactMotion<'_>,
    work: &mut crate::CollisionWork,
) -> Result<Real, ClothError> {
    let fraction = source.motion_fraction(motion, work)?;
    if !fraction.is_finite() || !(0.0..=1.0).contains(&fraction) {
        return Err(ClothError::InvalidSurfaceContact("invalid motion fraction"));
    }
    Ok(fraction)
}

pub(crate) fn step(
    cloth: &mut Cloth,
    h: Real,
    gravity: Vec3,
    settings: &SolverSettings,
    targets: &[Target],
    source: &mut impl ContactSource,
    implicit: ImplicitSettings,
) -> Result<StepReport, ClothError> {
    settings.validate(h)?;
    implicit.validate()?;
    if !gravity.is_finite() {
        return Err(ClothError::InvalidParameter("gravity"));
    }
    let contact = cloth.contact_settings.ok_or(ClothError::InvalidParameter(
        "implicit solver requires surface settings",
    ))?;
    contact.validate()?;
    if contact.activation_margin <= 0.0
        || (contact.self_collision && !contact.continuous_self_collision)
        || (contact.rigid_surface_collision && !source.continuous_motion())
    {
        return Err(ClothError::InvalidParameter(
            "implicit solver requires a positive barrier width and continuous surface contacts",
        ));
    }
    let model = Model::new(cloth, h, gravity, targets, implicit)?;
    let config = cloth.contact_settings.unwrap();
    let mut engine = SelfCollision::new(cloth.mesh.clone(), config);
    engine.begin(cloth.mesh.clone(), config);
    let mut x = cloth.positions.clone();
    for (&i, &p) in &cloth.pins {
        x[i as usize] = p;
    }
    for t in targets {
        x[t.particle as usize] = t.position;
    }
    let initial = query(
        source,
        &mut engine,
        &cloth.positions,
        &cloth.positions,
        ContactStage::Stabilization,
        cloth,
        settings.max_contacts,
    )?;
    if config.self_collision && engine.motion_fraction(&cloth.positions, &x)? < 1.0 {
        return Err(failure("grasp initialization sweep", 0));
    }
    if source.continuous_motion()
        && external_fraction(
            source,
            ContactMotion {
                start: &cloth.positions,
                end: &x,
                stage: ContactStage::Prediction,
            },
            &mut engine.work,
        )? < 1.0
    {
        return Err(failure("initial rigid sweep", 0));
    }
    let mut friction = Vec::new();
    for contact in initial {
        let gap = contact.gap(&cloth.positions);
        let b = barrier(gap, model.band, model.barrier_stiffness)
            .ok_or_else(|| failure("initial contact clearance", 0))?;
        if b[1] < 0.0 {
            let previous = contact.relative(&cloth.positions);
            let reference = if contact
                .key
                .features
                .iter()
                .any(|feature| matches!(feature, crate::SurfaceFeature::External { .. }))
            {
                // `relative` is the weighted vertex sum (without the contact
                // offset), so an external stencil supplies a world-space point.
                source.transport_surface_anchor(&contact, previous, h)?
            } else {
                // Self-contact anchors are relative cloth coordinates; the
                // external source owns only rigid-surface transport.
                previous
            };
            if !reference.is_finite() {
                return Err(ClothError::InvalidSurfaceContact(
                    "non-finite friction anchor",
                ));
            }
            friction.push(Friction {
                contact,
                reference,
                load: -b[1],
            });
        }
    }
    let mut contacts = query(
        source,
        &mut engine,
        &cloth.positions,
        &x,
        ContactStage::Iteration,
        cloth,
        settings.max_contacts,
    )?;
    let mut report = StepReport::default();
    let mut converged = model.count == 0;
    if model.count == 0 {
        model.assemble(&x, &contacts, &friction, false)?;
    }
    // Match the author solver's RMS Newton-displacement / h criterion and its
    // three-iteration window, rather than imposing an unrelated maximum norm.
    let mut residual_window = std::collections::VecDeque::new();
    let mut sparse = SparseSystem::new(implicit.execution);
    for iteration in 0..if model.count == 0 {
        0
    } else {
        implicit.max_iterations
    } {
        let assembled = model.assemble(&x, &contacts, &friction, true)?;
        let matrix = sparse.matrix(model.count, &assembled.triplets, iteration)?;

        let factor = sparse.factor(matrix.as_ref(), iteration)?;
        let rhs = Mat::from_fn(model.count, 1, |i, _| -assembled.gradient[i]);
        let solution = factor.solve(rhs.as_ref());
        let direction: Vec<Vec3> = model
            .dofs
            .iter()
            .map(|i| {
                i.map_or(Vec3::ZERO, |d| {
                    Vec3::new(solution[(d, 0)], solution[(d + 1, 0)], solution[(d + 2, 0)])
                })
            })
            .collect();
        let slope: Real = assembled
            .gradient
            .iter()
            .enumerate()
            .map(|(i, g)| g * solution[(i, 0)])
            .sum();
        report.iterations = iteration + 1;
        let newton_rms = (direction.iter().map(|v| v.length_squared()).sum::<Real>()
            / (model.count / 3) as Real)
            .sqrt()
            / h;
        residual_window.push_back(newton_rms);
        if residual_window.len() > 3 {
            residual_window.pop_front();
        }
        let roundoff_motion = direction
            .iter()
            .all(|d| d.length() <= Real::EPSILON * 128.0 * cloth.mesh.area().sqrt());
        if newton_rms == 0.0
            || roundoff_motion
            || (residual_window.len() == 3
                && residual_window
                    .iter()
                    .all(|&r| r <= implicit.velocity_tolerance))
        {
            converged = true;
            break;
        }
        if slope >= 0.0 || !slope.is_finite() {
            return Err(failure("non-descent direction", iteration));
        }
        let full: Vec<_> = x.iter().zip(&direction).map(|(&p, &d)| p + d).collect();
        let mut alpha = if config.self_collision {
            engine.motion_fraction(&x, &full)?
        } else {
            1.0
        };
        if source.continuous_motion() {
            alpha = alpha.min(external_fraction(
                source,
                ContactMotion {
                    start: &x,
                    end: &full,
                    stage: ContactStage::Iteration,
                },
                &mut engine.work,
            )?);
        }
        let mut accepted = None;
        for _ in 0..implicit.max_line_search_iterations {
            let trial: Vec<_> = x
                .iter()
                .zip(&direction)
                .map(|(&p, &d)| p + d * alpha)
                .collect();
            let trial_contacts = query(
                source,
                &mut engine,
                &cloth.positions,
                &trial,
                ContactStage::Iteration,
                cloth,
                settings.max_contacts,
            )?;
            match model.assemble(&trial, &trial_contacts, &friction, false) {
                Ok(next) if next.energy <= assembled.energy + 0.0001 * alpha * slope => {
                    accepted = Some((trial, trial_contacts));
                    break;
                }
                Err(ClothError::UnresolvedSurfaceContact | ClothError::DegenerateConstraint) => {}
                Err(e) => return Err(e),
                _ => {}
            }
            alpha *= 0.5;
        }
        if let Some((trial, next_contacts)) = accepted {
            x = trial;
            contacts = next_contacts;
        } else {
            return Err(failure("line search", iteration));
        }
    }
    if !converged {
        return Err(failure("Newton iteration budget", implicit.max_iterations));
    }
    if config.self_collision && engine.motion_fraction(&cloth.positions, &x)? < 1.0 {
        return Err(failure("final self sweep", report.iterations));
    }
    if source.continuous_motion()
        && external_fraction(
            source,
            ContactMotion {
                start: &cloth.positions,
                end: &x,
                stage: ContactStage::Final,
            },
            &mut engine.work,
        )? < 1.0
    {
        return Err(failure("final rigid sweep", report.iterations));
    }
    contacts = query(
        source,
        &mut engine,
        &cloth.positions,
        &x,
        ContactStage::Final,
        cloth,
        settings.max_contacts,
    )?;
    model.assemble(&x, &contacts, &friction, false)?;
    report.contacts = contacts.len();
    report.surface_collision = engine.work;
    report.max_stretch = cloth
        .mesh
        .edges()
        .iter()
        .map(|e| x[e.vertices[0] as usize].distance(x[e.vertices[1] as usize]) / e.rest_length)
        .fold(1.0, Real::max);
    let mut stretch: Vec<_> = cloth
        .mesh
        .edges()
        .iter()
        .map(|e| x[e.vertices[0] as usize].distance(x[e.vertices[1] as usize]) / e.rest_length)
        .collect();
    stretch.sort_by(Real::total_cmp);
    report.p95_stretch = stretch[((stretch.len() - 1) as Real * 0.95) as usize];
    report.max_bend_error = cloth
        .mesh
        .hinges()
        .iter()
        .filter_map(|hinge| {
            crate::constraints::bend::angle(hinge.vertices.map(|i| x[i as usize]))
                .map(|angle| angle_difference(angle, hinge.rest_angle).abs())
        })
        .fold(0.0, Real::max);
    drop(model);
    let velocities: Vec<_> = x
        .iter()
        .zip(&cloth.positions)
        .map(|(&p, &old)| (p - old) / h)
        .collect();
    if velocities.iter().any(|v| !v.is_finite()) {
        return Err(ClothError::NonFiniteState);
    }
    cloth.previous.clone_from(&cloth.positions);
    cloth.velocities = velocities;
    cloth.positions = x;
    cloth.contact_history.clear();
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn hard_targets_restrict_the_full_system_without_changing_the_objective() {
        let mesh = crate::GridBuilder::new(3, 3)
            .size(0.25, 0.25)
            .build()
            .unwrap();
        let mut cloth = Cloth::new(mesh, crate::ClothMaterial::default()).unwrap();
        cloth
            .set_contact_settings(Some(crate::ClothContactSettings {
                activation_margin: 0.001,
                ..Default::default()
            }))
            .unwrap();
        let x: Vec<_> = cloth
            .positions
            .iter()
            .map(|p| Vec3::new(p.x * 1.03, 0.2 * p.x * p.z, p.z * 0.98))
            .collect();
        // Reversed support indices exercise both sides of each local tensor.
        let mut contact = SurfaceContact {
            key: crate::SurfaceContactKey {
                other_cloth: None,
                features: [
                    crate::SurfaceFeature::Edge([0, 8]),
                    crate::SurfaceFeature::Edge([2, 6]),
                ],
            },
            particles: [8, 0, 6, 2],
            weights: [0.3, 0.7, -0.4, -0.6],
            normal: Vec3::new(1.0, 2.0, 3.0).normalize(),
            offset: Vec3::ZERO,
            separation: 0.001,
            surface_velocity: Vec3::ZERO,
            static_friction: 0.5,
            kinetic_friction: 0.5,
        };
        contact.offset = contact.relative(&x) - contact.normal * 0.0015;
        let friction = [Friction {
            contact,
            reference: contact.relative(&x) + Vec3::X * 0.0002,
            load: 0.03,
        }];
        let implicit = ImplicitSettings::default();
        let full = Model::new(&cloth, 0.1, Vec3::ZERO, &[], implicit)
            .unwrap()
            .assemble(&x, &[contact], &friction, true)
            .unwrap();
        for fixed in [vec![0, 4, 8], vec![2, 6], vec![], (0..9).collect()] {
            let targets: Vec<_> = fixed
                .iter()
                .map(|&particle| Target {
                    particle,
                    position: x[particle as usize],
                    compliance: 0.0,
                })
                .collect();
            let model = Model::new(&cloth, 0.1, Vec3::ZERO, &targets, implicit).unwrap();
            let reduced = model.assemble(&x, &[contact], &friction, true).unwrap();
            let trial = model.assemble(&x, &[contact], &friction, false).unwrap();
            assert_eq!(reduced.energy, full.energy);
            assert_eq!(trial.energy, full.energy);
            assert_eq!(trial.gradient, reduced.gradient);
            assert!(trial.triplets.is_empty());
            let free: Vec<_> = (0..full.gradient.len())
                .filter(|i| !fixed.contains(&((*i / 3) as u32)))
                .collect();
            assert_eq!(
                reduced.gradient,
                free.iter().map(|&i| full.gradient[i]).collect::<Vec<_>>()
            );
            // Eliminating hard targets must produce the principal submatrix,
            // retaining every contribution and its original addition order.
            let expected: Vec<_> = full
                .triplets
                .iter()
                .filter_map(|t| {
                    Some((
                        free.iter().position(|&i| i == t.row)?,
                        free.iter().position(|&i| i == t.col)?,
                        t.val,
                    ))
                })
                .collect();
            let actual: Vec<_> = reduced
                .triplets
                .iter()
                .map(|t| (t.row, t.col, t.val))
                .collect();
            assert_eq!(actual, expected);
        }
    }

    #[test]
    #[cfg(feature = "f64")]
    fn assembled_material_is_rigid_invariant_and_has_balanced_forces() {
        let mesh = crate::GridBuilder::new(3, 3)
            .size(0.25, 0.25)
            .build()
            .unwrap();
        let mut cloth = Cloth::new(mesh, crate::ClothMaterial::default()).unwrap();
        cloth
            .set_contact_settings(Some(crate::ClothContactSettings {
                activation_margin: 0.001,
                ..Default::default()
            }))
            .unwrap();
        let mut model =
            Model::new(&cloth, 0.04, Vec3::ZERO, &[], ImplicitSettings::default()).unwrap();
        let x: Vec<_> = cloth
            .positions
            .iter()
            .map(|p| Vec3::new(p.x * 1.03, p.y + 0.2 * p.x * p.z, p.z * 0.98))
            .collect();
        model.prediction = x.clone();
        let a = model.assemble(&x, &[], &[], false).unwrap();
        let forces: Vec<_> = a
            .gradient
            .chunks_exact(3)
            .map(|g| Vec3::new(g[0], g[1], g[2]))
            .collect();
        assert!(forces.iter().copied().sum::<Vec3>().length() < 1e-10);
        assert!(
            x.iter()
                .zip(&forces)
                .map(|(p, g)| p.cross(*g))
                .sum::<Vec3>()
                .length()
                < 1e-10
        );
        let axis = Vec3::new(1.0, 2.0, -1.0).normalize();
        let angle: Real = 0.7;
        let rotate = |p: Vec3| {
            p * angle.cos()
                + axis.cross(p) * angle.sin()
                + axis * (axis.dot(p) * (1.0 - angle.cos()))
        };
        let rotated: Vec<_> = x
            .iter()
            .map(|&p| rotate(p) + Vec3::new(0.4, -0.3, 0.7))
            .collect();
        model.prediction = rotated.clone();
        let b = model.assemble(&rotated, &[], &[], false).unwrap();
        assert!((a.energy - b.energy).abs() < 1e-11);
        for (i, g) in b.gradient.chunks_exact(3).enumerate() {
            assert!((Vec3::new(g[0], g[1], g[2]) - rotate(forces[i])).length() < 1e-9);
        }
        model.prediction = x.clone();
        let eps = 1e-7;
        for i in 0..x.len() {
            for axis in 0..3 {
                let mut lo = x.clone();
                let mut hi = x.clone();
                lo[i][axis] -= eps;
                hi[i][axis] += eps;
                let derivative = (model.assemble(&hi, &[], &[], false).unwrap().energy
                    - model.assemble(&lo, &[], &[], false).unwrap().energy)
                    / (2.0 * eps);
                assert!((derivative - forces[i][axis]).abs() < 1e-7);
            }
        }
    }
    #[test]
    fn membrane_gradient_and_hessian_follow_energy() {
        let f = [Vec3::new(1.03, 0.07, 0.13), Vec3::new(-0.05, 0.97, 0.02)];
        let (e, g, h) = membrane(f, 3.0, 2.0, true).unwrap();
        assert!(e > 0.0);
        let eps = if Real::EPSILON < 1e-10 { 1e-5 } else { 1e-3 };
        let tol = if Real::EPSILON < 1e-10 { 1e-7 } else { 1e-3 };
        for j in 0..6 {
            let mut lo = f;
            let mut hi = f;
            lo[j / 3][j % 3] -= eps;
            hi[j / 3][j % 3] += eps;
            let (el, gl, _) = membrane(lo, 3.0, 2.0, false).unwrap();
            let (eh, gh, _) = membrane(hi, 3.0, 2.0, false).unwrap();
            assert!(((eh - el) / (2.0 * eps) - g[j / 3][j % 3]).abs() < tol);
            for i in 0..6 {
                assert!(
                    ((gh[i / 3][i % 3] - gl[i / 3][i % 3]) / (2.0 * eps) - h[(i, j)]).abs() < tol
                );
            }
        }
    }
    #[test]
    fn barrier_derivatives_and_inactive_branch() {
        let g = 0.0002;
        // Central differences need a larger increment at f32 precision to
        // avoid cancellation at this submillimetre contact scale.
        let h = if Real::EPSILON < 1e-10 { 1e-8 } else { 1e-6 };
        let b = barrier(g, 0.000318, 30.0).unwrap();
        let lo = barrier(g - h, 0.000318, 30.0).unwrap();
        let hi = barrier(g + h, 0.000318, 30.0).unwrap();
        assert!(((hi[0] - lo[0]) / (2.0 * h) - b[1]).abs() < 1e-6);
        assert!(((hi[1] - lo[1]) / (2.0 * h) - b[2]).abs() < 0.1);
        assert_eq!(barrier(0.001, 0.000318, 30.0), Some([0.0; 3]));
        assert!(barrier(0.0, 0.000318, 30.0).is_none());
    }
}
