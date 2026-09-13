//! Independent double-precision reference geometry for offline task validation.
//! No production collision predicates or broad phase are used here.
#![allow(dead_code)]
use serde::Serialize;
pub type Point = [f64; 3];
fn sub(a: Point, b: Point) -> Point {
    std::array::from_fn(|i| a[i] - b[i])
}
fn add(a: Point, b: Point) -> Point {
    std::array::from_fn(|i| a[i] + b[i])
}
fn mul(a: Point, t: f64) -> Point {
    a.map(|v| v * t)
}
fn dot(a: Point, b: Point) -> f64 {
    a.iter().zip(b).map(|(a, b)| a * b).sum()
}
fn cross(a: Point, b: Point) -> Point {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn norm2(a: Point) -> f64 {
    dot(a, a)
}
fn lerp(a: Point, b: Point, t: f64) -> Point {
    add(a, mul(sub(b, a), t))
}
fn point_segment(p: Point, a: Point, b: Point) -> f64 {
    let e = sub(b, a);
    let t = if norm2(e) > 0.0 {
        (dot(sub(p, a), e) / norm2(e)).clamp(0.0, 1.0)
    } else {
        0.0
    };
    norm2(sub(p, lerp(a, b, t)))
}
/// Interior orthogonal projection, with edge distances as the boundary oracle.
pub fn point_triangle_distance_squared(p: Point, t: [Point; 3]) -> f64 {
    let u = sub(t[1], t[0]);
    let v = sub(t[2], t[0]);
    let w = sub(p, t[0]);
    let n = cross(u, v);
    let nn = norm2(n);
    let mut best = (0..3)
        .map(|i| point_segment(p, t[i], t[(i + 1) % 3]))
        .fold(f64::INFINITY, f64::min);
    if nn > 0.0 {
        let s = dot(cross(w, v), n) / nn;
        let q = dot(cross(u, w), n) / nn;
        if s >= 0.0 && q >= 0.0 && s + q <= 1.0 {
            best = best.min(dot(w, n).powi(2) / nn);
        }
    }
    best
}
/// Enumerate endpoint minima plus the stationary interior line-line solution.
pub fn segment_distance_squared(a: [Point; 2], b: [Point; 2]) -> f64 {
    let mut best = point_segment(a[0], b[0], b[1])
        .min(point_segment(a[1], b[0], b[1]))
        .min(point_segment(b[0], a[0], a[1]))
        .min(point_segment(b[1], a[0], a[1]));
    let u = sub(a[1], a[0]);
    let v = sub(b[1], b[0]);
    let w = sub(a[0], b[0]);
    let uv = dot(u, v);
    let uu = norm2(u);
    let vv = norm2(v);
    let determinant = norm2(cross(u, v));
    if determinant > f64::EPSILON * uu * vv {
        let s = (uv * dot(v, w) - vv * dot(u, w)) / determinant;
        let t = (uu * dot(v, w) - uv * dot(u, w)) / determinant;
        if (0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t) {
            best = best.min(norm2(sub(lerp(a[0], a[1], s), lerp(b[0], b[1], t))));
        }
    }
    best
}
fn pierces(a: Point, b: Point, t: [Point; 3]) -> bool {
    let n = cross(sub(t[1], t[0]), sub(t[2], t[0]));
    let da = dot(sub(a, t[0]), n);
    let db = dot(sub(b, t[0]), n);
    if da == db || (da > 0.0 && db > 0.0) || (da < 0.0 && db < 0.0) {
        return false;
    }
    let p = lerp(a, b, da / (da - db));
    let scale = norm2(sub(t[1], t[0])).max(norm2(sub(t[2], t[0])));
    point_triangle_distance_squared(p, t) <= scale * 1.0e-24
}
pub fn triangle_distance_squared(a: [Point; 3], b: [Point; 3]) -> f64 {
    let mut d = f64::INFINITY;
    for i in 0..3 {
        if pierces(a[i], a[(i + 1) % 3], b) || pierces(b[i], b[(i + 1) % 3], a) {
            return 0.0;
        }
        d = d
            .min(point_triangle_distance_squared(a[i], b))
            .min(point_triangle_distance_squared(b[i], a));
        for j in 0..3 {
            d = d.min(segment_distance_squared(
                [a[i], a[(i + 1) % 3]],
                [b[j], b[(j + 1) % 3]],
            ));
        }
    }
    d
}
#[derive(Default, Debug, Clone, Serialize)]
pub struct SurfaceAudit {
    pub crossing_pairs: usize,
    pub max_separation_deficit: f64,
    pub tested_pairs: usize,
}
/// Sweep-and-prune reference, independent of the production hierarchy.
/// Pairs sharing a vertex are incident; all other triangle pairs are checked.
pub fn audit_surface(positions: &[Point], triangles: &[[u32; 3]], thickness: f64) -> SurfaceAudit {
    let mut boxes: Vec<_> = triangles
        .iter()
        .enumerate()
        .map(|(id, t)| {
            let p = t.map(|i| positions[i as usize]);
            let lo: Point =
                std::array::from_fn(|a| p.iter().map(|v| v[a]).fold(f64::INFINITY, f64::min));
            let hi: Point =
                std::array::from_fn(|a| p.iter().map(|v| v[a]).fold(f64::NEG_INFINITY, f64::max));
            (id, lo, hi)
        })
        .collect();
    boxes.sort_by(|a, b| a.1[0].total_cmp(&b.1[0]).then(a.0.cmp(&b.0)));
    let mut audit = SurfaceAudit::default();
    for (index, &(a, lo, hi)) in boxes.iter().enumerate() {
        for &(b, blo, bhi) in &boxes[index + 1..] {
            if blo[0] > hi[0] + thickness {
                break;
            }
            if (1..3).any(|k| blo[k] > hi[k] + thickness || lo[k] > bhi[k] + thickness)
                || triangles[a].iter().any(|v| triangles[b].contains(v))
            {
                continue;
            }
            audit.tested_pairs += 1;
            let d = triangle_distance_squared(
                triangles[a].map(|i| positions[i as usize]),
                triangles[b].map(|i| positions[i as usize]),
            )
            .sqrt();
            audit.crossing_pairs += usize::from(d <= 1.0e-10);
            audit.max_separation_deficit =
                audit.max_separation_deficit.max((thickness - d).max(0.0));
        }
    }
    audit
}

type P2 = [f64; 2];
type Polygon = Vec<P2>;
fn cross2(a: P2, b: P2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn sub2(a: P2, b: P2) -> P2 {
    [a[0] - b[0], a[1] - b[1]]
}
fn projected_half(
    rest: &[Point],
    positions: &[Point],
    triangles: &[[u32; 3]],
    sign: f64,
) -> Vec<Polygon> {
    let mut polygons = vec![];
    for t in triangles {
        let input: Vec<_> = t
            .iter()
            .map(|&i| (rest[i as usize][2] * sign, positions[i as usize]))
            .collect();
        let mut poly = vec![];
        for i in 0..3 {
            let (d, p) = input[i];
            let (e, q) = input[(i + 1) % 3];
            if d >= 0.0 {
                poly.push([p[0], p[2]]);
            }
            if (d < 0.0 && e > 0.0) || (d > 0.0 && e < 0.0) {
                let x = lerp(p, q, d / (d - e));
                poly.push([x[0], x[2]]);
            }
        }
        if poly.len() >= 3 {
            polygons.push(poly);
        }
    }
    polygons
}
fn intervals(polygons: &[Polygon], x: f64) -> Vec<[f64; 2]> {
    let mut result = vec![];
    for p in polygons {
        let mut ys = vec![];
        for i in 0..p.len() {
            let a = p[i];
            let b = p[(i + 1) % p.len()];
            if (a[0] < x && x < b[0]) || (b[0] < x && x < a[0]) {
                ys.push(a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0]));
            }
        }
        if ys.len() >= 2 {
            result.push([
                ys.iter().copied().fold(f64::INFINITY, f64::min),
                ys.iter().copied().fold(f64::NEG_INFINITY, f64::max),
            ]);
        }
    }
    result.sort_by(|a, b| a[0].total_cmp(&b[0]));
    let mut merged: Vec<[f64; 2]> = vec![];
    for i in result {
        if let Some(last) = merged.last_mut()
            && i[0] <= last[1]
        {
            last[1] = last[1].max(i[1]);
        } else {
            merged.push(i);
        }
    }
    merged
}
/// Exact piecewise-linear vertical integration of polygon unions. Edge crossings
/// split the x slabs so overlapping triangles count once, even after crumpling.
pub fn projected_areas(a: &[Polygon], b: &[Polygon]) -> [f64; 3] {
    let edges: Vec<_> = a
        .iter()
        .chain(b)
        .flat_map(|p| (0..p.len()).map(|i| [p[i], p[(i + 1) % p.len()]]))
        .collect();
    let mut xs: Vec<_> = edges.iter().flat_map(|e| [e[0][0], e[1][0]]).collect();
    for (i, e) in edges.iter().enumerate() {
        for f in &edges[i + 1..] {
            if e[0][0].min(e[1][0]) > f[0][0].max(f[1][0])
                || f[0][0].min(f[1][0]) > e[0][0].max(e[1][0])
            {
                continue;
            }
            let u = sub2(e[1], e[0]);
            let v = sub2(f[1], f[0]);
            let w = sub2(f[0], e[0]);
            let den = cross2(u, v);
            if den.abs() > 1.0e-25 {
                let s = cross2(w, v) / den;
                let t = cross2(w, u) / den;
                if (0.0..1.0).contains(&s) && (0.0..1.0).contains(&t) {
                    xs.push(e[0][0] + s * u[0]);
                }
            }
        }
    }
    xs.sort_by(f64::total_cmp);
    xs.dedup_by(|a, b| (*a - *b).abs() < 1.0e-12);
    let mut areas = [0.0; 3];
    for w in xs.windows(2) {
        let width = w[1] - w[0];
        let x = (w[0] + w[1]) * 0.5;
        let ia = intervals(a, x);
        let ib = intervals(b, x);
        areas[0] += width * ia.iter().map(|v| v[1] - v[0]).sum::<f64>();
        areas[1] += width * ib.iter().map(|v| v[1] - v[0]).sum::<f64>();
        let (mut i, mut j) = (0, 0);
        while i < ia.len() && j < ib.len() {
            areas[2] += width * (ia[i][1].min(ib[j][1]) - ia[i][0].max(ib[j][0])).max(0.0);
            if ia[i][1] < ib[j][1] {
                i += 1;
            } else {
                j += 1;
            }
        }
    }
    areas
}
#[derive(Default, Debug, Clone, Serialize)]
pub struct FoldMetrics {
    pub projected_half_areas: [f64; 2],
    pub projected_overlap: f64,
    pub overlap_ratio: f64,
    pub projected_union_area: f64,
    pub relative_area_error: f64,
    pub max_corner_error: f64,
}
pub fn fold_metrics(
    rest: &[Point],
    positions: &[Point],
    triangles: &[[u32; 3]],
    grid: usize,
    size: f64,
) -> FoldMetrics {
    let areas = projected_areas(
        &projected_half(rest, positions, triangles, -1.0),
        &projected_half(rest, positions, triangles, 1.0),
    );
    let max_corner_error = [(0, (grid - 1) * grid), (grid - 1, grid * grid - 1)]
        .iter()
        .map(|&(a, b)| (positions[a][0] - positions[b][0]).hypot(positions[a][2] - positions[b][2]))
        .fold(0.0, f64::max);
    let union = areas[0] + areas[1] - areas[2];
    FoldMetrics {
        projected_half_areas: [areas[0], areas[1]],
        projected_overlap: areas[2],
        overlap_ratio: if areas[0].min(areas[1]) > 1.0e-20 {
            areas[2] / areas[0].min(areas[1])
        } else {
            0.0
        },
        projected_union_area: union,
        relative_area_error: (union / (size * size * 0.5) - 1.0).abs(),
        max_corner_error,
    }
}
