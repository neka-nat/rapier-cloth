// The implicit solver's default iteration cap (ImplicitSettings::max_iterations).
const MAX_NEWTON_ITERATIONS = 128;
const vector = (a, n) => Array.isArray(a) && a.length === n && a.every(Number.isFinite);
const index = (i, count) => Number.isSafeInteger(i) && i >= 0 && i < count;
const quaternion = q => vector(q, 4) && Math.abs(q.reduce((s, x) => s + x*x, 0) - 1) < 1e-3;
const nonnegative = x => Number.isFinite(x) && x >= 0;
function requireValue(ok, message) { if (!ok) throw new Error(`Invalid live frame: ${message}`); }

export function validateFrame(frame) {
  requireValue(frame.protocol === 2, 'protocol 2 is required; restart the matching server and viewer');
  requireValue(['drape', 'hanging', 'fold_towel', 'implicit_towel'].includes(frame.scene), 'scene');
  requireValue(['f32', 'f64'].includes(frame.precision), 'precision');
  requireValue(vector(frame.positions, 3072), 'vertex positions');
  requireValue(vector(frame.sphere, 3) && nonnegative(frame.sphere_radius), 'sphere');
  requireValue(Number.isSafeInteger(frame.step) && frame.step >= 0 && nonnegative(frame.time), 'time');
  const implicit = frame.scene === 'implicit_towel';
  const expectedH = implicit ? 0.1 : 1/240, substeps = implicit ? 1 : 4;
  requireValue(Number.isFinite(frame.h) && Math.abs(frame.h - expectedH) < 1e-9
    && Math.abs(frame.time - frame.step * expectedH) < 1e-5, 'step size');
  requireValue(frame.substeps === substeps && index(frame.advanced_substeps, substeps + 1) && frame.advanced_substeps <= frame.step, 'accepted substeps');
  requireValue(implicit ? Number.isSafeInteger(frame.iterations) && frame.iterations >= 0 && frame.iterations <= MAX_NEWTON_ITERATIONS : frame.iterations === 8, 'iterations');
  requireValue(Array.isArray(frame.pins) && frame.pins.length <= 1024 && frame.pins.every(i => index(i, 1024)), 'pins');
  if (frame.triangles !== undefined) requireValue(Array.isArray(frame.triangles) && frame.triangles.length === 5766 && frame.triangles.every(i => index(i, 1024)), 'triangles');
  requireValue(['physics_ms','p95_stretch','max_penetration','max_target_error','contacts'].every(k => nonnegative(frame[k])), 'diagnostics');
  requireValue(frame.options && typeof frame.options.auto_motion === 'boolean'
    && ['sphere_x','sphere_z','wind'].every(k => Number.isFinite(frame.options[k])), 'options');
  requireValue((frame.scene === 'fold_towel') === (frame.folding !== undefined), 'fold scene data');
  if (implicit) { validateImplicit(frame); return; }
  requireValue(frame.implicit === undefined, 'unexpected implicit data');
  if (frame.scene !== 'fold_towel') return;
  const fold = frame.folding;
  requireValue(fold && typeof fold === 'object', 'fold task data');
  requireValue(fold.config?.version === 2 && fold.config.grid === 32 && fold.variant === 0
    && Number.isSafeInteger(fold.config.end_step) && fold.config.end_step > 0, 'fold fixture');
  requireValue(typeof fold.phase === 'string' && typeof fold.manual_release === 'boolean', 'fold task state');
  requireValue(fold.stopped === null || (typeof fold.stopped === 'string' && fold.stopped.length > 0), 'fold failure');
  requireValue(fold.collision_mode && ['self_collision','rigid_surface','continuous_self','continuous_rigid'].every(k => typeof fold.collision_mode[k] === 'boolean'), 'collision modes');
  requireValue(Array.isArray(fold.grippers) && fold.grippers.length === 2, 'grippers');
  fold.grippers.forEach((g, i) => {
    requireValue(g.id === i && typeof g.holding === 'boolean', 'gripper identity');
    requireValue(vector(g.translation, 3) && quaternion(g.rotation) && vector(g.local_translation, 3)
      && quaternion(g.local_rotation) && vector(g.half_extents, 3) && g.half_extents.every(x => x > 0), 'gripper geometry');
  });
  requireValue(Array.isArray(fold.grasps) && fold.grasps.length <= 1024, 'grasps');
  let points = 0;
  for (const grasp of fold.grasps) {
    requireValue(index(grasp.gripper, 2) && Array.isArray(grasp.points) && grasp.points.length > 0, 'grasp owner');
    points += grasp.points.length;
    requireValue(points <= 1024, 'grasp point count');
    for (const p of grasp.points) {
      requireValue(vector(p.position, 3) && vector(p.anchor, 3), 'grasp positions');
      const m = p.material;
      requireValue(m && (m.kind === 'vertex' ? index(m.particle, 1024)
        : m.kind === 'surface' && index(m.triangle, 1922) && vector(m.barycentric, 3)
          && m.barycentric.every(x => x >= 0) && Math.abs(m.barycentric.reduce((s, x) => s + x, 0) - 1) < 1e-5), 'material point');
    }
  }
  fold.grippers.forEach(g => requireValue(g.holding === fold.grasps.some(a => a.gripper === g.id), 'gripper holding state'));
  if (fold.inspection !== null) {
    const inspection = fold.inspection;
    requireValue(inspection?.step === frame.step && inspection.metrics, 'inspection step');
    requireValue(['projected_overlap','overlap_ratio','projected_union_area','relative_area_error','max_corner_error'].every(k => nonnegative(inspection.metrics[k])), 'inspection metrics');
    requireValue(vector(inspection.metrics.projected_half_areas, 2) && inspection.metrics.projected_half_areas.every(nonnegative), 'inspection areas');
  }
}

function validateImplicit(frame) {
  const fold = frame.implicit;
  requireValue(frame.precision === 'f64' && frame.implicit_available === true && fold?.solver === 'implicit', 'implicit solver');
  requireValue(typeof fold.automatic === 'boolean' && typeof fold.completed === 'boolean'
    && fold.end_step === 80 && frame.step <= 80 && fold.completed === (frame.step === 80), 'implicit task state');
  requireValue(typeof fold.phase === 'string' && (fold.stopped === null || typeof fold.stopped === 'string' && fold.stopped.length > 0), 'implicit stop');
  requireValue(nonnegative(fold.floor_height) && Array.isArray(fold.grippers) && fold.grippers.length === 2, 'implicit scene');
  requireValue(frame.pins.length === 0 && Array.isArray(fold.grasps) && fold.grasps.length <= 2, 'implicit grasps');
  const seen = new Set(), owners = new Set();
  fold.grippers.forEach((g,i) => requireValue(g.id === i && typeof g.holding === 'boolean'
    && vector(g.translation,3) && quaternion(g.rotation) && vector(g.local_translation,3)
    && quaternion(g.local_rotation) && vector(g.half_extents,3) && g.half_extents.every(v=>v>0), 'implicit gripper'));
  for (const grasp of fold.grasps) {
    requireValue(index(grasp.gripper,2) && !owners.has(grasp.gripper) && Array.isArray(grasp.points) && grasp.points.length > 0, 'implicit grasp owner');
    owners.add(grasp.gripper);
    for (const p of grasp.points) {
      requireValue(p.material?.kind === 'vertex' && index(p.material.particle,1024) && !seen.has(p.material.particle)
        && vector(p.position,3) && vector(p.anchor,3), 'implicit grasp point');
      seen.add(p.material.particle);
    }
  }
  fold.grippers.forEach(g=>requireValue(g.holding === owners.has(g.id), 'implicit holding state'));
  requireValue(Array.isArray(fold.desired) && fold.desired.length === 2
    && fold.desired.every(p=>vector(p.translation,3) && quaternion(p.rotation)), 'desired poses');
  requireValue(['strict','approximate'].includes(fold.cap_policy)
    && ['nominal','lift_5mm','grasp_inset','left_early','right_late','friction_low','friction_high'].includes(fold.variant), 'implicit configuration');
  const s = fold.sample;
  requireValue(s && s.iterations === frame.iterations && s.step === frame.step && Math.abs(s.time-frame.time)<1e-9 && s.held_vertices === seen.size
    && ['physics_ms','rms_speed','max_speed','max_edge_extension'].every(k=>nonnegative(s[k])), 'implicit diagnostics');
  requireValue(Number.isSafeInteger(s.approximate_steps) && s.approximate_steps >= 0
    && s.approximate_steps <= frame.step && (fold.cap_policy !== 'strict' || s.approximate_steps === 0), 'approximate history');
  if (frame.step === 0) { requireValue(s.outcome === null, 'initial solver outcome'); return; }
  const o = s.outcome;
  requireValue(o && ['converged','approximate_iteration_cap'].includes(o.termination)
    && o.converged === (o.termination === 'converged') && Number.isFinite(o.energy)
    && nonnegative(o.force_rms) && nonnegative(o.force_max), 'implicit outcome');
  if (!o.converged) requireValue(fold.cap_policy === 'approximate' && s.approximate_steps > 0
    && frame.iterations === MAX_NEWTON_ITERATIONS && s.max_edge_extension < 0.03, 'approximate validation');
}
