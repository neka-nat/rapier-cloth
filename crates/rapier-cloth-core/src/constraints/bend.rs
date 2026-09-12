use crate::{Real, Vec3};

/// Signed dihedral angle and its analytic position gradients.
/// Faces are (0,1,2) and (1,0,3); a flat, consistently wound pair has angle 0.
/// Returns None on degenerate geometry. No inverse trig finite differences are
/// used by the solver: the atan2/normalization chain is differentiated exactly.
pub fn angle_and_gradients(p: [Vec3; 4]) -> Option<(Real, [Vec3; 4])> {
    let e = p[1] - p[0];
    let u = p[2] - p[0];
    let v = p[3] - p[0];
    let raw_a = e.cross(u);
    let raw_b = v.cross(e);
    let le = e.length();
    let la = raw_a.length();
    let lb = raw_b.length();
    if [le, la, lb]
        .iter()
        .any(|n| !n.is_finite() || *n <= Real::MIN_POSITIVE)
    {
        return None;
    }
    let t = e / le;
    let a = raw_a / la;
    let b = raw_b / lb;
    let s = t.dot(a.cross(b));
    let c = a.dot(b);
    let denom = s * s + c * c;
    if denom <= Real::MIN_POSITIVE {
        return None;
    }
    let gs = c / denom;
    let gc = -s / denom;
    let ga = b.cross(t) * gs + b * gc;
    let gb = t.cross(a) * gs + a * gc;
    let gt = a.cross(b) * gs;
    let gna = (ga - a * a.dot(ga)) / la;
    let gnb = (gb - b * b.dot(gb)) / lb;
    let ge = (gt - t * t.dot(gt)) / le + u.cross(gna) + gnb.cross(v);
    let gu = gna.cross(e);
    let gv = e.cross(gnb);
    let gradients = [-ge - gu - gv, ge, gu, gv];
    if gradients.iter().any(|g| !g.is_finite()) {
        return None;
    }
    Some((s.atan2(c), gradients))
}

/// Difference of angles with the branch cut at +/- pi handled consistently.
pub fn angle_difference(angle: Real, rest: Real) -> Real {
    let d = angle - rest;
    d.sin().atan2(d.cos())
}
