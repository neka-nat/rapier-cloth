use crate::{Real, Vec3};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TriangleWitness {
    pub point: Vec3,
    /// Weights in the input triangle's vertex order.
    pub barycentric: [Real; 3],
}

/// Closest point using the triangle's Voronoi regions. Degenerate or non-finite
/// input returns None; it is never repaired by silently changing the topology.
pub fn closest_triangle(p: Vec3, triangle: [Vec3; 3]) -> Option<TriangleWitness> {
    if !p.is_finite() || triangle.iter().any(|p| !p.is_finite()) {
        return None;
    }
    let [a, b, c] = triangle;
    let ab = b - a;
    let ac = c - a;
    let scale = ab
        .length_squared()
        .max(ac.length_squared())
        .max((c - b).length_squared());
    let area2 = ab.cross(ac).length_squared();
    if !scale.is_finite()
        || !area2.is_finite()
        || scale <= Real::MIN_POSITIVE
        || area2 <= scale * scale * Real::EPSILON.powi(2)
    {
        return None;
    }
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    let weights = if d1 <= 0.0 && d2 <= 0.0 {
        [1.0, 0.0, 0.0]
    } else {
        let bp = p - b;
        let d3 = ab.dot(bp);
        let d4 = ac.dot(bp);
        if d3 >= 0.0 && d4 <= d3 {
            [0.0, 1.0, 0.0]
        } else {
            let vc = d1 * d4 - d3 * d2;
            if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
                let v = d1 / (d1 - d3);
                [1.0 - v, v, 0.0]
            } else {
                let cp = p - c;
                let d5 = ab.dot(cp);
                let d6 = ac.dot(cp);
                if d6 >= 0.0 && d5 <= d6 {
                    [0.0, 0.0, 1.0]
                } else {
                    let vb = d5 * d2 - d1 * d6;
                    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
                        let w = d2 / (d2 - d6);
                        [1.0 - w, 0.0, w]
                    } else {
                        let va = d3 * d6 - d5 * d4;
                        if va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0 {
                            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                            [0.0, 1.0 - w, w]
                        } else {
                            let inv = 1.0 / (va + vb + vc);
                            let v = vb * inv;
                            let w = vc * inv;
                            [1.0 - v - w, v, w]
                        }
                    }
                }
            }
        }
    };
    let point = a * weights[0] + b * weights[1] + c * weights[2];
    (point.is_finite() && weights.iter().all(|v| v.is_finite())).then_some(TriangleWitness {
        point,
        barycentric: weights,
    })
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SegmentWitness {
    pub a: Vec3,
    pub b: Vec3,
    /// Interpolation parameters on the first and second segment.
    pub parameters: [Real; 2],
}
/// Closest clamped segments, including point segments and parallel edges.
pub fn closest_segments(first: [Vec3; 2], second: [Vec3; 2]) -> Option<SegmentWitness> {
    if first.iter().chain(&second).any(|p| !p.is_finite()) {
        return None;
    }
    let u = first[1] - first[0];
    let v = second[1] - second[0];
    let w = first[0] - second[0];
    let a = u.length_squared();
    let e = v.length_squared();
    let f = v.dot(w);
    if !a.is_finite() || !e.is_finite() || !f.is_finite() {
        return None;
    }
    let (s, t) = if a <= Real::MIN_POSITIVE && e <= Real::MIN_POSITIVE {
        (0.0, 0.0)
    } else if a <= Real::MIN_POSITIVE {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = u.dot(w);
        if e <= Real::MIN_POSITIVE {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = u.dot(v);
            let n = u.cross(v);
            let den = n.length_squared();
            let s = if den > Real::EPSILON.powi(2) * a * e {
                (-w.cross(v).dot(n) / den).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let t = (b * s + f) / e;
            if t < 0.0 {
                ((-c / a).clamp(0.0, 1.0), 0.0)
            } else if t > 1.0 {
                (((b - c) / a).clamp(0.0, 1.0), 1.0)
            } else {
                (s, t)
            }
        }
    };
    let a = first[0] + u * s;
    let b = second[0] + v * t;
    (a.is_finite() && b.is_finite()).then_some(SegmentWitness {
        a,
        b,
        parameters: [s, t],
    })
}

/// Initial-state validation including edge/face piercing. A point/triangle plus
/// edge/edge distance test alone misses triangles which already intersect.
pub fn triangles_intersect(a: [Vec3; 3], b: [Vec3; 3]) -> Option<bool> {
    let scale = (a[1] - a[0])
        .length()
        .max((a[2] - a[0]).length())
        .max((b[1] - b[0]).length())
        .max((b[2] - b[0]).length());
    let epsilon = scale * Real::EPSILON * 8.0;
    let na = (a[1] - a[0]).cross(a[2] - a[0]).try_normalize()?;
    let nb = (b[1] - b[0]).cross(b[2] - b[0]).try_normalize()?;
    for i in 0..3 {
        if a[i].distance_squared(closest_triangle(a[i], b)?.point) <= epsilon * epsilon
            || b[i].distance_squared(closest_triangle(b[i], a)?.point) <= epsilon * epsilon
        {
            return Some(true);
        }
        for (p, q, triangle, n) in [(a[i], a[(i + 1) % 3], b, nb), (b[i], b[(i + 1) % 3], a, na)] {
            let da = (p - triangle[0]).dot(n);
            let db = (q - triangle[0]).dot(n);
            if (da <= 0.0 && db >= 0.0 || da >= 0.0 && db <= 0.0) && da != db {
                let hit = p + (q - p) * (da / (da - db));
                if hit.distance_squared(closest_triangle(hit, triangle)?.point) <= epsilon * epsilon
                {
                    return Some(true);
                }
            }
        }
        for j in 0..3 {
            let w = closest_segments([a[i], a[(i + 1) % 3]], [b[j], b[(j + 1) % 3]])?;
            if w.a.distance_squared(w.b) <= epsilon * epsilon {
                return Some(true);
            }
        }
    }
    Some(false)
}
