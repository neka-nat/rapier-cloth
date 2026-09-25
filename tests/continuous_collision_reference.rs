#[path = "support/folding.rs"]
mod folding;
#[path = "support/surface_oracle.rs"]
mod oracle;
use folding::{point, real};
use rapier_cloth::{collision::ccd::*, *};

fn distance(feature: CcdFeature, positions: [[f64; 3]; 4]) -> f64 {
    match feature {
        CcdFeature::VertexFace => oracle::point_triangle_distance_squared(
            positions[0],
            [positions[1], positions[2], positions[3]],
        ),
        CcdFeature::EdgeEdge => oracle::segment_distance_squared(
            [positions[0], positions[1]],
            [positions[2], positions[3]],
        ),
    }
    .sqrt()
}

#[test]
fn accepted_sweep_prefixes_preserve_separation_in_independent_holdouts() {
    let mut seed = 702_933_u64;
    let mut random = || {
        Vec3::from_array(std::array::from_fn(|_| {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            real((seed >> 32) as f64 / u32::MAX as f64 - 0.5)
        }))
    };
    for feature in [CcdFeature::VertexFace, CcdFeature::EdgeEdge] {
        let mut limited = 0;
        let mut clear = 0;
        for case in 0..1000 {
            let start = std::array::from_fn(|_| random());
            let end = std::array::from_fn(|_| random());
            let reference_start = start.map(point);
            let reference_end = end.map(point);
            if distance(feature, reference_start) < 0.0105 {
                continue;
            }
            let result = conservative_advance(
                feature,
                start,
                end,
                0.01,
                &mut CollisionWork::default(),
                CollisionLimits::default(),
            )
            .unwrap_or_else(|e| panic!("{feature:?}, case {case}: {e}; {start:?} -> {end:?}"));
            match result {
                CcdResult::Clear => clear += 1,
                CcdResult::Limited { .. } => limited += 1,
            }
            let fraction = point(Vec3::splat(result.fraction()))[0];
            assert!(fraction > 0.0 && fraction <= 1.0);
            // This samples the entire accepted prefix with an independent f64
            // predicate, not only its endpoint. Analytical core fixtures check
            // exact crossing times separately; sampling alone is not CCD proof.
            for sample in 0..=128 {
                let t = fraction * sample as f64 / 128.0;
                let positions = std::array::from_fn(|i| {
                    std::array::from_fn(|k| {
                        reference_start[i][k] + (reference_end[i][k] - reference_start[i][k]) * t
                    })
                });
                let d = distance(feature, positions);
                assert!(
                    d >= 0.01,
                    "{feature:?}, case {case}, t={t}, d={d}, {result:?}"
                );
            }
        }
        assert!(
            clear > 100 && limited > 25,
            "{feature:?}: clear={clear}, limited={limited}"
        );
    }
}
