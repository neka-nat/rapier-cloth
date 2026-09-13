use parry_reference::math::Pose;
use parry_reference::{
    query::{ShapeCastOptions, cast_shapes},
    shape::{Ball, Triangle},
};
use rapier_cloth::{Real, *};

#[cfg(not(feature = "f64"))]
fn scalar(value: Real) -> f64 {
    f64::from(value)
}
#[cfg(feature = "f64")]
fn scalar(value: Real) -> f64 {
    value
}
fn wide(p: Vec3) -> parry_reference::math::Vector {
    parry_reference::math::Vector::new(scalar(p.x), scalar(p.y), scalar(p.z))
}

#[test]
fn analytic_sphere_approaches_agree_with_independent_parry_shape_casts() {
    let mut seed = 0x61da4u64;
    let mut sample = || {
        seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((seed >> 32) as u32 as f64 / u32::MAX as f64) as Real
    };
    let u = Vec3::new(0.6, 0.8, 0.0);
    let v = Vec3::Z;
    let n = u.cross(v);
    let shift = Vec3::new(2.0, -3.0, 1.0);
    let points = [shift, shift + u, shift + v];
    let view = SurfaceView {
        positions: &points,
        triangles: &[[0, 1, 2]],
    };
    let triangle = Triangle::new(wide(points[0]), wide(points[1]), wide(points[2]));
    let mut hits = 0;
    let mut misses = 0;
    for case in 0..600 {
        let origin =
            shift + u * (sample() * 2.0 - 0.5) + v * (sample() * 2.0 - 0.5) + n * (sample() + 0.5);
        let direction = (u * (sample() - 0.5) * 0.5 + v * (sample() - 0.5) * 0.5 - n).normalize();
        let radius = sample() * 0.09 + 0.01;
        let actual = view
            .sweep_sphere(
                SurfaceRay {
                    origin,
                    direction,
                    max_distance: 3.0,
                },
                radius,
                SurfaceQueryLimits::default(),
            )
            .unwrap();
        let expected = cast_shapes(
            &Pose::from_translation(wide(origin)),
            wide(direction).normalize(),
            &Ball::new(scalar(radius)),
            &Pose::IDENTITY,
            parry_reference::math::Vector::ZERO,
            &triangle,
            ShapeCastOptions::with_max_time_of_impact(3.0),
        )
        .unwrap();
        match (actual, expected) {
            (Some(a), Some(b)) => {
                hits += 1;
                assert!(
                    (a.distance_f64() - b.time_of_impact).abs() < 1.0e-6,
                    "case={case}, actual={a:?}, reference={b:?}, origin={origin:?}, direction={direction:?}, radius={radius}"
                );
                let center = origin + direction * a.distance;
                assert!(
                    (center.distance(a.position) - radius).abs() < 3.0e-6,
                    "case={case}, hit is not on sphere: {a:?}"
                );
            }
            (None, None) => misses += 1,
            (a, b) => panic!(
                "case={case}, actual={a:?}, reference={b:?}, origin={origin:?}, direction={direction:?}, radius={radius}"
            ),
        }
    }
    assert!(hits > 50 && misses > 200, "{hits} hits, {misses} misses");
}
