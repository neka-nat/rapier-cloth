use crate::{Real, Vec3};

/// Signed dihedral angle and its analytic position gradients.
/// Faces are (0,1,2) and (1,0,3); a flat, consistently wound pair has angle 0.
/// Returns None on degenerate geometry. The closed-form gradients use the
/// shared edge and face normals; no inverse trig finite differences are used.
#[inline]
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
    // Opposite-vertex derivatives are the signed normals divided by triangle
    // altitude. Edge derivatives follow from the projections onto that edge.
    let gu = a * (-le / la);
    let gv = b * (-le / lb);
    let ge = -(gu * (u.dot(t) / le) + gv * (v.dot(t) / le));
    let gradients = [-ge - gu - gv, ge, gu, gv];
    if gradients.iter().any(|g| !g.is_finite()) {
        return None;
    }
    Some((s.atan2(c), gradients))
}

/// Signed angle without constructing gradients (for final-state diagnostics).
#[inline]
pub fn angle(p: [Vec3; 4]) -> Option<Real> {
    let e = p[1] - p[0];
    let a = e.cross(p[2] - p[0]);
    let b = (p[3] - p[0]).cross(e);
    let lengths = [e.length(), a.length(), b.length()];
    if lengths
        .iter()
        .any(|n| !n.is_finite() || *n <= Real::MIN_POSITIVE)
    {
        return None;
    }
    let t = e / lengths[0];
    let a = a / lengths[1];
    let b = b / lengths[2];
    Some(t.dot(a.cross(b)).atan2(a.dot(b)))
}

/// Difference of angles with the branch cut at +/- pi handled consistently.
#[inline]
pub fn angle_difference(angle: Real, rest: Real) -> Real {
    let d = angle - rest;
    // Most hinges stay away from the branch cut. Keep the general trig path
    // for wrapped, non-finite and boundary inputs, including f32's rounded pi.
    if d.abs() < std::f64::consts::PI as Real {
        d
    } else {
        d.sin().atan2(d.cos())
    }
}
