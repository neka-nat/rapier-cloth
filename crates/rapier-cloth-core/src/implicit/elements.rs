//! Parallel material evaluation. Workers write each element's matrix blocks
//! into their final positions of the assembly and keep energies and gradients
//! in per-element slots; the caller then accumulates those in mesh order.
use super::*;

pub(super) struct Element<const N: usize> {
    energy: Real,
    gradient: [Vec3; N],
}
pub(super) type MembraneElement = Element<3>;
pub(super) type BendingElement = Element<4>;

fn active(dofs: &[Option<usize>], i: usize, j: usize) -> bool {
    dofs[i].zip(dofs[j]).is_some_and(|(di, dj)| di >= dj)
}

/// Number of free lower-triangular blocks of an element with these vertices.
fn block_count(dofs: &[Option<usize>], ids: &[usize]) -> usize {
    let free = ids.iter().filter(|&&i| dofs[i].is_some()).count();
    free * (free + 1) / 2
}

fn block(dofs: &[Option<usize>], i: usize, j: usize, value: [[Real; 3]; 3]) -> Block {
    Block {
        row: dofs[i].unwrap() / 3,
        col: dofs[j].unwrap() / 3,
        value,
    }
}

fn triangle_element(
    tri: &Triangle,
    x: &[Vec3],
    dofs: &[Option<usize>],
    mu: Real,
    lambda: Real,
    fiber_stiffness: (Real, Real),
    blocks: &mut [Block],
) -> Result<MembraneElement, ClothError> {
    let mut f = [Vec3::ZERO; 2];
    for k in 0..3 {
        for c in 0..2 {
            f[c] += x[tri.ids[k]] * tri.b[k][c];
        }
    }
    let (mu, lambda) = (mu * tri.scale, lambda * tri.scale);
    let fiber_stiffness = (fiber_stiffness.0 * tri.scale, fiber_stiffness.1 * tri.scale);
    let (mut e, mut g, _) =
        membrane(f, mu, lambda, false).ok_or(ClothError::DegenerateConstraint)?;
    let fiber = tri.fiber.filter(|_| fiber_stiffness != (0.0, 0.0));
    if let Some(a) = fiber {
        let (fe, fg, _) =
            super::fibers(f, a, fiber_stiffness, false).ok_or(ClothError::DegenerateConstraint)?;
        e += fe;
        g[0] += fg[0];
        g[1] += fg[1];
    }
    let energy = e * tri.volume;
    let gradient = std::array::from_fn(|k| (g[0] * tri.b[k][0] + g[1] * tri.b[k][1]) * tri.volume);
    if blocks.is_empty() {
        return Ok(Element { energy, gradient });
    }
    let mut hp =
        membrane::projected_hessian(f, mu, lambda).ok_or(ClothError::DegenerateConstraint)?;
    if let Some(a) = fiber {
        hp += super::fibers(f, a, fiber_stiffness, true)
            .ok_or(ClothError::DegenerateConstraint)?
            .2;
    }
    let mut index = 0;
    for i in 0..3 {
        for j in 0..3 {
            if !active(dofs, tri.ids[i], tri.ids[j]) {
                continue;
            }
            let mut value = [[0.0; 3]; 3];
            for u in 0..3 {
                for v in 0..3 {
                    for r in 0..2 {
                        for s in 0..2 {
                            value[u][v] +=
                                tri.volume * tri.b[i][r] * hp[(r * 3 + u, s * 3 + v)] * tri.b[j][s];
                        }
                    }
                }
            }
            blocks[index] = block(dofs, tri.ids[i], tri.ids[j], value);
            index += 1;
        }
    }
    Ok(Element { energy, gradient })
}

fn hinge_element(
    hinge: &Hinge,
    x: &[Vec3],
    dofs: &[Option<usize>],
    blocks: &mut [Block],
) -> Result<BendingElement, ClothError> {
    let (angle, g) =
        angle_and_gradients(hinge.ids.map(|i| x[i])).ok_or(ClothError::DegenerateConstraint)?;
    let d = angle_difference(angle, hinge.rest);
    let k = hinge.stiffness;
    let energy = 0.5 * k * d * d;
    let gradient = std::array::from_fn(|i| g[i] * (k * d));
    if blocks.is_empty() {
        return Ok(Element { energy, gradient });
    }
    let mut index = 0;
    for i in 0..4 {
        for j in 0..4 {
            if !active(dofs, hinge.ids[i], hinge.ids[j]) {
                continue;
            }
            let value = std::array::from_fn(|u| std::array::from_fn(|v| k * g[i][u] * g[j][v]));
            blocks[index] = block(dofs, hinge.ids[i], hinge.ids[j], value);
            index += 1;
        }
    }
    Ok(Element { energy, gradient })
}

pub(super) type MembraneSlot = Option<Result<MembraneElement, ClothError>>;
pub(super) type BendingSlot = Option<Result<BendingElement, ClothError>>;

/// Reused per-element results and block offsets.
#[derive(Default)]
pub(super) struct ElementBuffers {
    triangles: Vec<MembraneSlot>,
    hinges: Vec<BendingSlot>,
    /// Block offset of every triangle, then of every hinge, relative to the
    /// start of the material blocks; each list ends with its total.
    triangle_offsets: Vec<usize>,
    hinge_offsets: Vec<usize>,
}

impl ElementBuffers {
    /// Prepares the offsets; returns the number of material blocks.
    fn layout(&mut self, model: &Model, hessian: bool) -> usize {
        let count = |ids: &[usize]| {
            if hessian {
                block_count(&model.dofs, ids)
            } else {
                0
            }
        };
        self.triangle_offsets.clear();
        self.triangle_offsets.push(0);
        for tri in &model.triangles {
            let next = self.triangle_offsets.last().unwrap() + count(&tri.ids);
            self.triangle_offsets.push(next);
        }
        self.hinge_offsets.clear();
        self.hinge_offsets.push(0);
        for hinge in &model.hinges {
            let next = self.hinge_offsets.last().unwrap() + count(&hinge.ids);
            self.hinge_offsets.push(next);
        }
        self.triangle_offsets.last().unwrap() + self.hinge_offsets.last().unwrap()
    }
}

/// Fills the element slots in mesh order and every element's blocks into
/// `blocks`, which has the length returned by `layout`.
fn evaluate(
    model: &Model,
    x: &[Vec3],
    buffers: &mut ElementBuffers,
    blocks: &mut [Block],
) -> Result<(), ClothError> {
    let workers = model.workers.lanes();
    let (nt, nh) = (model.triangles.len(), model.hinges.len());
    let (triangles, hinges) = (&mut buffers.triangles, &mut buffers.hinges);
    triangles.clear();
    triangles.resize_with(nt, || None);
    hinges.clear();
    hinges.resize_with(nh, || None);
    let (toffsets, hoffsets) = (&buffers.triangle_offsets, &buffers.hinge_offsets);
    let fill = |first_triangle: usize,
                tout: &mut [MembraneSlot],
                mut tblocks: &mut [Block],
                first_hinge: usize,
                hout: &mut [BendingSlot],
                mut hblocks: &mut [Block]| {
        for (k, slot) in tout.iter_mut().enumerate() {
            let t = first_triangle + k;
            let (mine, rest) = tblocks.split_at_mut(toffsets[t + 1] - toffsets[t]);
            tblocks = rest;
            *slot = Some(triangle_element(
                &model.triangles[t],
                x,
                &model.dofs,
                model.mu,
                model.lambda,
                model.fibers,
                mine,
            ));
        }
        for (k, slot) in hout.iter_mut().enumerate() {
            let h = first_hinge + k;
            let (mine, rest) = hblocks.split_at_mut(hoffsets[h + 1] - hoffsets[h]);
            hblocks = rest;
            *slot = Some(hinge_element(&model.hinges[h], x, &model.dofs, mine));
        }
    };
    let (mut tblocks, mut hblocks) = blocks.split_at_mut(toffsets[nt]);
    let (mut tout, mut hout) = (&mut triangles[..], &mut hinges[..]);
    let mut parts = Vec::with_capacity(workers);
    for k in 0..workers {
        let (ta, tb) = (k * nt / workers, (k + 1) * nt / workers);
        let (ha, hb) = (k * nh / workers, (k + 1) * nh / workers);
        let (t_slots, rest) = tout.split_at_mut(tb - ta);
        tout = rest;
        let (t_blocks, rest) = tblocks.split_at_mut(toffsets[tb] - toffsets[ta]);
        tblocks = rest;
        let (h_slots, rest) = hout.split_at_mut(hb - ha);
        hout = rest;
        let (h_blocks, rest) = hblocks.split_at_mut(hoffsets[hb] - hoffsets[ha]);
        hblocks = rest;
        parts.push((ta, t_slots, t_blocks, ha, h_slots, h_blocks));
    }
    let parts = workers::handoff(parts);
    model.workers.run(workers, |k| {
        let (ta, t_slots, t_blocks, ha, h_slots, h_blocks) = workers::take(&parts, k);
        fill(ta, t_slots, t_blocks, ha, h_slots, h_blocks);
    })?;
    Ok(())
}

/// Evaluates every material element on the workers, appends its blocks to
/// `blocks` in mesh order and accumulates energies and gradients in that order.
pub(super) fn assemble(
    model: &Model,
    x: &[Vec3],
    hessian: bool,
    buffers: &mut ElementBuffers,
    energy: &mut EnergySum,
    mut add_gradient: impl FnMut(usize, Vec3),
    blocks: &mut Vec<Block>,
) -> Result<(), ClothError> {
    let count = buffers.layout(model, hessian);
    let base = blocks.len();
    let placeholder = Block {
        row: 0,
        col: 0,
        value: [[0.0; 3]; 3],
    };
    blocks.resize(base + count, placeholder);
    if let Err(error) = evaluate(model, x, buffers, &mut blocks[base..]) {
        blocks.truncate(base);
        return Err(error);
    }
    for (tri, result) in model.triangles.iter().zip(&mut buffers.triangles) {
        let element = match result.take().expect("unfilled triangle") {
            Ok(element) => element,
            Err(error) => {
                blocks.truncate(base);
                return Err(error);
            }
        };
        energy.add(element.energy);
        for k in 0..3 {
            add_gradient(tri.ids[k], element.gradient[k]);
        }
    }
    for (hinge, result) in model.hinges.iter().zip(&mut buffers.hinges) {
        let element = match result.take().expect("unfilled hinge") {
            Ok(element) => element,
            Err(error) => {
                blocks.truncate(base);
                return Err(error);
            }
        };
        energy.add(element.energy);
        for i in 0..4 {
            add_gradient(hinge.ids[i], element.gradient[i]);
        }
    }
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;
    fn identical(a: &Result<Assembly, ClothError>, b: &Result<Assembly, ClothError>) {
        match (a, b) {
            (Ok(a), Ok(b)) => {
                assert_eq!(a.energy.to_bits(), b.energy.to_bits());
                assert_eq!(a.gradient.len(), b.gradient.len());
                for (a, b) in a.gradient.iter().zip(&b.gradient) {
                    assert_eq!(a.to_bits(), b.to_bits());
                }
                assert_eq!(a.blocks.len(), b.blocks.len());
                for (a, b) in a.blocks.iter().zip(&b.blocks) {
                    assert_eq!((a.row, a.col), (b.row, b.col));
                    for (a, b) in a.value.iter().flatten().zip(b.value.iter().flatten()) {
                        assert_eq!(a.to_bits(), b.to_bits());
                    }
                }
            }
            (Err(a), Err(b)) => assert_eq!(format!("{a:?}"), format!("{b:?}")),
            _ => panic!("assembly result mismatch"),
        }
    }

    fn cloth() -> Cloth {
        let mesh = crate::GridBuilder::new(32, 32)
            .size(0.5, 0.5)
            .build()
            .unwrap();
        let mut c = Cloth::new(mesh, crate::ClothMaterial::default()).unwrap();
        c.set_contact_settings(Some(crate::ClothContactSettings {
            activation_margin: 0.001,
            ..Default::default()
        }))
        .unwrap();
        c
    }
    fn compare(model: &mut Model, x: &[Vec3], contacts: &[SurfaceContact], friction: &[Friction]) {
        for hessian in [false, true] {
            model.set_execution(ImplicitExecution::Serial);
            let reference = model.assemble(x, contacts, friction, hessian);
            model.set_execution(ImplicitExecution::Parallel4);
            identical(&reference, &model.assemble(x, contacts, friction, hessian));
        }
    }
    #[test]
    fn bitwise_assembly_preserves_free_fixed_and_released_blocks() {
        let c = cloth();
        let x: Vec<_> = c
            .positions
            .iter()
            .map(|p| Vec3::new(p.x * 1.03, 0.02 * p.x * p.z, p.z * 0.98))
            .collect();
        for ids in [vec![], vec![0, 17, 500, 1023], (0..1024).collect()] {
            let targets: Vec<_> = ids
                .iter()
                .map(|&i| Target {
                    particle: i,
                    position: x[i as usize],
                    compliance: 0.0,
                })
                .collect();
            let mut model = Model::new(
                &c,
                0.1,
                Vec3::new(0.0, -9.81, 0.0),
                &targets,
                ImplicitSettings::default(),
            )
            .unwrap();
            compare(&mut model, &x, &[], &[]);
        }
    }
    #[test]
    fn bitwise_assembly_preserves_contact_friction_and_invalid_gap() {
        let c = cloth();
        let x = c.positions.clone();
        let mut model = Model::new(&c, 0.1, Vec3::ZERO, &[], ImplicitSettings::default()).unwrap();
        let mut contact = SurfaceContact {
            key: crate::SurfaceContactKey {
                other_cloth: None,
                features: [
                    crate::SurfaceFeature::Vertex(0),
                    crate::SurfaceFeature::External {
                        object: 1,
                        feature: 0,
                    },
                ],
            },
            particles: [0; 4],
            weights: [1.0, 0.0, 0.0, 0.0],
            normal: Vec3::Y,
            offset: x[0] - Vec3::Y * 0.0015,
            separation: 0.001,
            surface_velocity: Vec3::ZERO,
            static_friction: 0.5,
            kinetic_friction: 0.5,
        };
        let friction = [Friction {
            contact,
            reference: x[0] + Vec3::X * 0.0002,
            load: 0.03,
        }];
        compare(&mut model, &x, &[contact], &friction);
        contact.offset = x[0];
        assert!(model.assemble(&x, &[contact], &[], true).is_err());
        compare(&mut model, &x, &[contact], &[]);
    }
    #[test]
    fn degenerate_and_nonfinite_elements_preserve_errors() {
        let c = cloth();
        let mut model = Model::new(&c, 0.1, Vec3::ZERO, &[], ImplicitSettings::default()).unwrap();
        let mut x = c.positions.clone();
        let tri = model.triangles[500].ids;
        x[tri[1]] = x[tri[0]];
        assert!(model.assemble(&x, &[], &[], true).is_err());
        compare(&mut model, &x, &[], &[]);
        x[0].x = Real::NAN;
        compare(&mut model, &x, &[], &[]);
    }
    #[test]
    fn small_mesh_and_empty_hinges_are_supported() {
        let mesh =
            crate::ClothMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Z], vec![[0, 1, 2]]).unwrap();
        let mut c = Cloth::new(mesh, crate::ClothMaterial::default()).unwrap();
        c.set_contact_settings(Some(crate::ClothContactSettings {
            activation_margin: 0.001,
            ..Default::default()
        }))
        .unwrap();
        let mut model = Model::new(&c, 0.1, Vec3::ZERO, &[], ImplicitSettings::default()).unwrap();
        compare(&mut model, &c.positions, &[], &[]);
        let mut buffers = ElementBuffers::default();
        let count = buffers.layout(&model, true);
        assert_eq!(count, 6);
        let mut blocks = vec![
            Block {
                row: 0,
                col: 0,
                value: [[0.0; 3]; 3]
            };
            count
        ];
        evaluate(&model, &c.positions, &mut buffers, &mut blocks).unwrap();
        assert_eq!(buffers.triangles.len(), 1);
        assert!(buffers.triangles[0].as_ref().unwrap().is_ok());
        assert!(buffers.hinges.is_empty());
        assert!(blocks.iter().all(|b| b.row >= b.col));
    }
}
