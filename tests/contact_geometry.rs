#[path = "support/folding.rs"]
mod folding;
#[path = "support/surface_oracle.rs"]
mod oracle;
use folding::{point, real};
use rapier_cloth::{Vec3, collision::geometry::*};

#[test]
fn production_witnesses_match_independent_double_precision_reference() {
    let mut seed = 93_u64;
    let mut random = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        (seed >> 32) as f64 / u32::MAX as f64 - 0.5
    };
    for _ in 0..2000 {
        let points: [Vec3; 4] =
            std::array::from_fn(|_| Vec3::new(real(random()), real(random()), real(random())));
        let reference = points.map(point);
        let triangle = [points[1], points[2], points[3]];
        if let Some(w) = closest_triangle(points[0], triangle) {
            let actual = point(points[0] - w.point)
                .iter()
                .map(|v| v * v)
                .sum::<f64>();
            let expected = oracle::point_triangle_distance_squared(
                reference[0],
                [reference[1], reference[2], reference[3]],
            );
            assert!((actual - expected).abs() < 1.0e-6, "{actual} != {expected}");
            assert!(
                w.barycentric
                    .iter()
                    .all(|v| *v >= -1.0e-6 && *v <= 1.0 + 1.0e-6)
            );
        } else {
            panic!("nondegenerate generated triangle rejected: {triangle:?}");
        }
        let w = closest_segments([points[0], points[1]], [points[2], points[3]]).unwrap();
        let actual = point(w.a - w.b).iter().map(|v| v * v).sum::<f64>();
        let expected = oracle::segment_distance_squared(
            [reference[0], reference[1]],
            [reference[2], reference[3]],
        );
        assert!((actual - expected).abs() < 1.0e-6, "{actual} != {expected}");
    }
}
