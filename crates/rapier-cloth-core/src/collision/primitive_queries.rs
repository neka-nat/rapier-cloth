//! Internal implicit-only parent primitive queries. Shared projection queries
//! keep their canonical feature snapping and history orientation unchanged.
use super::*;

impl SelfCollision {
    pub(crate) fn implicit_primitives(
        &mut self,
        x: &[Vec3],
        out: &mut Vec<SurfaceContact>,
    ) -> Result<(), ClothError> {
        // This result never populates the projection contact cache.
        self.contact_cache_valid = false;
        self.refit(x)?;
        if !self.initial_checked {
            self.validate_initial(x)?;
        }
        let activation = self.settings.thickness + self.settings.activation_margin;
        for (vertex, &p) in x.iter().enumerate() {
            self.topology.triangles.query(
                Bounds::point(p).expanded(activation),
                0,
                &self.triangle_bounds,
                &self.triangle_nodes,
                &mut self.stack,
                &mut self.candidates,
                &mut self.work,
                self.settings.limits,
            )?;
            for &face in &self.candidates {
                let triangle = self.mesh.triangles()[face];
                if triangle.contains(&(vertex as u32)) {
                    continue;
                }
                let witness = closest_triangle(p, triangle.map(|i| x[i as usize]))
                    .ok_or(ClothError::DegenerateConstraint)?;
                let ids = [vertex as u32, triangle[0], triangle[1], triangle[2]];
                let weights = [
                    1.0,
                    -witness.barycentric[0],
                    -witness.barycentric[1],
                    -witness.barycentric[2],
                ];
                let features = [
                    SurfaceFeature::Vertex(vertex as u32),
                    SurfaceFeature::Face(face as u32),
                ];
                self.append_implicit(x, ids, weights, features, out)?;
            }
        }
        for a in 0..self.mesh.edges().len() {
            self.topology.edges.query(
                self.edge_bounds[a].expanded(activation),
                a + 1,
                &self.edge_bounds,
                &self.edge_nodes,
                &mut self.stack,
                &mut self.candidates,
                &mut self.work,
                self.settings.limits,
            )?;
            for &b in &self.candidates {
                let mut ea = self.mesh.edges()[a].vertices;
                let mut eb = self.mesh.edges()[b].vertices;
                if ea.iter().any(|i| eb.contains(i)) {
                    continue;
                }
                ea.sort_unstable();
                eb.sort_unstable();
                if ea > eb {
                    std::mem::swap(&mut ea, &mut eb);
                }
                let witness =
                    closest_segments(ea.map(|i| x[i as usize]), eb.map(|i| x[i as usize]))
                        .ok_or(ClothError::DegenerateConstraint)?;
                let [s, t] = witness.parameters;
                self.append_implicit(
                    x,
                    [ea[0], ea[1], eb[0], eb[1]],
                    [1.0 - s, s, t - 1.0, -t],
                    [SurfaceFeature::Edge(ea), SurfaceFeature::Edge(eb)],
                    out,
                )?;
            }
        }
        Ok(())
    }
    fn append_implicit(
        &self,
        x: &[Vec3],
        ids: [u32; 4],
        weights: [Real; 4],
        features: [SurfaceFeature; 2],
        out: &mut Vec<SurfaceContact>,
    ) -> Result<(), ClothError> {
        let delta = (0..4)
            .map(|i| x[ids[i] as usize] * weights[i])
            .sum::<Vec3>();
        let distance = delta.length();
        if distance > self.settings.thickness + self.settings.activation_margin {
            return Ok(());
        }
        if distance <= 0.0 || !distance.is_finite() {
            return Err(ClothError::UnresolvedSurfaceContact);
        }
        if out.len() >= self.settings.limits.retained_contacts {
            return Err(ClothError::ContactBudgetExceeded {
                limit: self.settings.limits.retained_contacts,
            });
        }
        out.push(SurfaceContact {
            key: SurfaceContactKey {
                other_cloth: None,
                features,
            },
            particles: ids,
            weights,
            normal: delta / distance,
            offset: Vec3::ZERO,
            surface_velocity: Vec3::ZERO,
            separation: self.settings.thickness,
            static_friction: self.settings.static_friction,
            kinetic_friction: self.settings.kinetic_friction,
        });
        Ok(())
    }
}

#[cfg(all(test, feature = "f64"))]
mod tests {
    use super::*;
    fn two_layers() -> (Arc<ClothMesh>, Vec<Vec3>, ClothContactSettings) {
        let x = vec![
            Vec3::new(-0.1, 0., -0.1),
            Vec3::new(0.1, 0., -0.1),
            Vec3::new(0., 0., 0.1),
            Vec3::new(-0.1, 0.0015, -0.1),
            Vec3::new(0.1, 0.0015, -0.1),
            Vec3::new(0., 0.0015, 0.1),
        ];
        let mesh = Arc::new(ClothMesh::new(x.clone(), vec![[0, 1, 2], [3, 4, 5]]).unwrap());
        let config = ClothContactSettings {
            thickness: 0.001,
            activation_margin: 0.001,
            continuous_self_collision: true,
            ..Default::default()
        };
        (mesh, x, config)
    }
    #[test]
    fn parent_contacts_do_not_certify_crossing_or_replace_projection_queries() {
        let (mesh, old, config) = two_layers();
        let mut x = old.clone();
        for p in &mut x[3..] {
            p.y = -0.0015;
        }
        let mut engine = SelfCollision::new(mesh.clone(), config);
        let mut cs = vec![];
        engine.implicit_primitives(&x, &mut cs).unwrap();
        assert!(!cs.is_empty());
        assert!(cs.iter().all(|c| c.gap(&x) > 0.));
        assert!(engine.motion_fraction(&old, &x).unwrap() < 1.);
        let mut shared = vec![];
        engine.generate(&old, &x, &mut shared, false).unwrap();
        assert!(shared.iter().any(|c| c.gap(&x) < 0.));
        let mut fresh = SelfCollision::new(mesh, config);
        let mut expected = vec![];
        fresh.generate(&old, &x, &mut expected, false).unwrap();
        assert_eq!(shared.len(), expected.len());
        for (a, b) in shared.iter().zip(&expected) {
            assert_eq!(a.key, b.key);
            assert_eq!(a.weights, b.weights);
            assert_eq!(a.normal, b.normal);
        }
    }
    #[test]
    fn initial_intersection_and_parent_contact_limits_remain_errors() {
        let (mesh, x, mut config) = two_layers();
        config.limits.retained_contacts = 1;
        let mut engine = SelfCollision::new(mesh.clone(), config);
        assert!(matches!(
            engine.implicit_primitives(&x, &mut vec![]),
            Err(ClothError::ContactBudgetExceeded { .. })
        ));
        let mut crossed = x;
        crossed[3].y = -0.0015;
        engine.begin(mesh, ClothContactSettings::default());
        assert!(matches!(
            engine.implicit_primitives(&crossed, &mut vec![]),
            Err(ClothError::InitialSelfIntersection { .. })
        ));
    }
    #[test]
    fn parent_faces_keep_multiplicity_at_a_shared_edge_and_snap_threshold() {
        let x = vec![
            Vec3::ZERO,
            Vec3::X,
            Vec3::Z,
            Vec3::new(0.5, 0., 0.5),
            Vec3::new(0.25, 0.0005, 0.25),
            Vec3::new(0.25, 1., 0.25),
            Vec3::new(0.35, 1., 0.25),
        ];
        let mesh =
            Arc::new(ClothMesh::new(x.clone(), vec![[0, 1, 3], [0, 3, 2], [4, 5, 6]]).unwrap());
        let config = ClothContactSettings {
            thickness: 0.000318,
            activation_margin: 0.000318,
            ..Default::default()
        };
        let mut engine = SelfCollision::new(mesh, config);
        for delta in [-1e-8, 0., 1e-8] {
            let mut q = x.clone();
            q[4].x += delta;
            let mut cs = vec![];
            engine.implicit_primitives(&q, &mut cs).unwrap();
            let selected: Vec<_> = cs
                .iter()
                .filter(|c| {
                    c.key.features[0] == SurfaceFeature::Vertex(4)
                        && matches!(c.key.features[1], SurfaceFeature::Face(0 | 1))
                })
                .collect();
            assert_eq!(selected.len(), 2);
            assert!(
                selected
                    .iter()
                    .all(|c| (c.gap(&q) - (0.0005 - 0.000318)).abs() < 1e-11)
            );
        }
    }
}
