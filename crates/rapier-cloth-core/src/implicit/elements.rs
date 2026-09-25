use super::*;

struct Element<const N: usize, const B: usize> {
    energy: Real,
    gradient: [Vec3; N],
    blocks: [[[Real; 3]; 3]; B],
}
type MembraneElement = Element<3, 6>;
type BendingElement = Element<4, 10>;

fn active(dofs: &[Option<usize>], i: usize, j: usize) -> bool {
    dofs[i].zip(dofs[j]).is_some_and(|(di, dj)| di >= dj)
}

fn triangle_element(
    tri: &Triangle,
    x: &[Vec3],
    dofs: &[Option<usize>],
    mu: Real,
    lambda: Real,
) -> Result<MembraneElement, ClothError> {
    let mut f = [Vec3::ZERO; 2];
    for k in 0..3 {
        for c in 0..2 {
            f[c] += x[tri.ids[k]] * tri.b[k][c];
        }
    }
    let (e, g, hf) = membrane(f, mu, lambda, true).ok_or(ClothError::DegenerateConstraint)?;
    let energy = e * tri.volume;
    let gradient = std::array::from_fn(|k| (g[0] * tri.b[k][0] + g[1] * tri.b[k][1]) * tri.volume);
    let eig = ((hf + hf.transpose()) * 0.5).symmetric_eigen();
    let d = SMatrix::<Real, 6, 6>::from_diagonal(&eig.eigenvalues.map(|v| v.max(0.0)));
    let hp = eig.eigenvectors * d * eig.eigenvectors.transpose();
    let mut blocks = [[[0.0; 3]; 3]; 6];
    let mut index = 0;
    for i in 0..3 {
        for j in 0..3 {
            if !active(dofs, tri.ids[i], tri.ids[j]) {
                continue;
            }
            let block = &mut blocks[index];
            for u in 0..3 {
                for v in 0..3 {
                    for r in 0..2 {
                        for s in 0..2 {
                            block[u][v] +=
                                tri.volume * tri.b[i][r] * hp[(r * 3 + u, s * 3 + v)] * tri.b[j][s];
                        }
                    }
                }
            }
            index += 1;
        }
    }
    Ok(Element {
        energy,
        gradient,
        blocks,
    })
}

fn hinge_element(
    hinge: &Hinge,
    x: &[Vec3],
    dofs: &[Option<usize>],
) -> Result<BendingElement, ClothError> {
    let (angle, g) =
        angle_and_gradients(hinge.ids.map(|i| x[i])).ok_or(ClothError::DegenerateConstraint)?;
    let d = angle_difference(angle, hinge.rest);
    let k = hinge.stiffness;
    let energy = 0.5 * k * d * d;
    let gradient = std::array::from_fn(|i| g[i] * (k * d));
    let mut blocks = [[[0.0; 3]; 3]; 10];
    let mut index = 0;
    for i in 0..4 {
        for j in 0..4 {
            if !active(dofs, hinge.ids[i], hinge.ids[j]) {
                continue;
            }
            blocks[index] = std::array::from_fn(|u| std::array::from_fn(|v| k * g[i][u] * g[j][v]));
            index += 1;
        }
    }
    Ok(Element {
        energy,
        gradient,
        blocks,
    })
}

type MembraneSlot = Option<Result<MembraneElement, ClothError>>;
type BendingSlot = Option<Result<BendingElement, ClothError>>;

type ElementBuffers = (Vec<MembraneSlot>, Vec<BendingSlot>);

fn evaluate(model: &Model, x: &[Vec3]) -> Result<ElementBuffers, ClothError> {
    let workers = 4;
    let mut triangles: Vec<MembraneSlot> = std::iter::repeat_with(|| None)
        .take(model.triangles.len())
        .collect();
    let mut hinges: Vec<BendingSlot> = std::iter::repeat_with(|| None)
        .take(model.hinges.len())
        .collect();
    let fill = |triangles: &[Triangle],
                hinges: &[Hinge],
                tout: &mut [MembraneSlot],
                hout: &mut [BendingSlot]| {
        for (tri, slot) in triangles.iter().zip(tout) {
            *slot = Some(triangle_element(
                tri,
                x,
                &model.dofs,
                model.mu,
                model.lambda,
            ));
        }
        for (hinge, slot) in hinges.iter().zip(hout) {
            *slot = Some(hinge_element(hinge, x, &model.dofs));
        }
    };
    std::thread::scope(|scope| -> Result<(), ClothError> {
        let nt = model.triangles.len();
        let nh = model.hinges.len();
        let (tout, mut remaining_t) = triangles.split_at_mut(nt / workers);
        let (hout, mut remaining_h) = hinges.split_at_mut(nh / workers);
        let mut handles = Vec::new();
        for i in 1..workers {
            let ta = i * nt / workers;
            let tb = (i + 1) * nt / workers;
            let ha = i * nh / workers;
            let hb = (i + 1) * nh / workers;
            let (tout, rest_t) = remaining_t.split_at_mut(tb - ta);
            let (hout, rest_h) = remaining_h.split_at_mut(hb - ha);
            remaining_t = rest_t;
            remaining_h = rest_h;
            let tris = &model.triangles[ta..tb];
            let bends = &model.hinges[ha..hb];
            let fill = &fill;
            // Scoped lifetimes keep the input/output borrows local. Creation
            // errors join started workers before returning without a commit.
            handles.push(workers::spawn(scope, move || {
                fill(tris, bends, tout, hout)
            })?);
        }
        fill(
            &model.triangles[..nt / workers],
            &model.hinges[..nh / workers],
            tout,
            hout,
        );
        for handle in handles {
            workers::join(handle);
        }
        Ok(())
    })?;
    Ok((triangles, hinges))
}

pub(super) fn assemble(
    model: &Model,
    x: &[Vec3],
    energy: &mut EnergySum,
    mut add_gradient: impl FnMut(usize, Vec3),
    mut add_block: impl FnMut(usize, usize, [[Real; 3]; 3]),
) -> Result<(), ClothError> {
    let (triangles, hinges) = evaluate(model, x)?;
    for (tri, result) in model.triangles.iter().zip(triangles) {
        let element = result.expect("unfilled triangle")?;
        energy.add(element.energy);
        for k in 0..3 {
            add_gradient(tri.ids[k], element.gradient[k]);
        }
        let mut block = 0;
        for i in 0..3 {
            for j in 0..3 {
                if !active(&model.dofs, tri.ids[i], tri.ids[j]) {
                    continue;
                }
                add_block(tri.ids[i], tri.ids[j], element.blocks[block]);
                block += 1;
            }
        }
    }
    for (hinge, result) in model.hinges.iter().zip(hinges) {
        let element = result.expect("unfilled hinge")?;
        energy.add(element.energy);
        let mut block = 0;
        for i in 0..4 {
            add_gradient(hinge.ids[i], element.gradient[i]);
            for j in 0..4 {
                if !active(&model.dofs, hinge.ids[i], hinge.ids[j]) {
                    continue;
                }
                add_block(hinge.ids[i], hinge.ids[j], element.blocks[block]);
                block += 1;
            }
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
                assert_eq!(a.triplets.len(), b.triplets.len());
                for (a, b) in a.triplets.iter().zip(&b.triplets) {
                    assert_eq!(
                        (a.row, a.col, a.val.to_bits()),
                        (b.row, b.col, b.val.to_bits())
                    );
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
            model.implicit.execution = ImplicitExecution::Serial;
            let reference = model.assemble(x, contacts, friction, hessian);
            model.implicit.execution = ImplicitExecution::Parallel4;
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
        let (triangles, hinges) = evaluate(&model, &c.positions).unwrap();
        assert_eq!(triangles.len(), 1);
        assert!(triangles[0].as_ref().unwrap().is_ok());
        assert!(hinges.is_empty());
    }
}
