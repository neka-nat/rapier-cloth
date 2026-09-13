const vector = (a, n) => Array.isArray(a) && a.length === n && a.every(Number.isFinite);
const index = (i, count) => Number.isSafeInteger(i) && i >= 0 && i < count;
const quaternion = q => vector(q, 4) && Math.abs(q.reduce((s, x) => s + x*x, 0) - 1) < 1e-3;
const nonnegative = x => Number.isFinite(x) && x >= 0;
function requireValue(ok, message) { if (!ok) throw new Error(`Invalid live frame: ${message}`); }

export function validateFrame(frame) {
  requireValue(frame.protocol === 2, 'protocol 2 is required; restart the matching server and viewer');
  requireValue(['drape', 'hanging', 'fold_towel'].includes(frame.scene), 'scene');
  requireValue(['f32', 'f64'].includes(frame.precision), 'precision');
  requireValue(vector(frame.positions, 3072), 'vertex positions');
  requireValue(vector(frame.sphere, 3) && nonnegative(frame.sphere_radius), 'sphere');
  requireValue(Number.isSafeInteger(frame.step) && frame.step >= 0 && nonnegative(frame.time), 'time');
  requireValue(Number.isFinite(frame.h) && Math.abs(frame.h - 1/240) < 1e-9
    && Math.abs(frame.time - frame.step/240) < 1e-5, 'step size');
  requireValue(frame.substeps === 4 && index(frame.advanced_substeps, 5) && frame.advanced_substeps <= frame.step, 'accepted substeps');
  requireValue(frame.iterations === 8, 'iterations');
  requireValue(Array.isArray(frame.pins) && frame.pins.length <= 1024 && frame.pins.every(i => index(i, 1024)), 'pins');
  if (frame.triangles !== undefined) requireValue(Array.isArray(frame.triangles) && frame.triangles.length === 5766 && frame.triangles.every(i => index(i, 1024)), 'triangles');
  requireValue(['physics_ms','p95_stretch','max_penetration','max_target_error','contacts'].every(k => nonnegative(frame[k])), 'diagnostics');
  requireValue(frame.options && typeof frame.options.auto_motion === 'boolean'
    && ['sphere_x','sphere_z','wind'].every(k => Number.isFinite(frame.options[k])), 'options');
  requireValue((frame.scene === 'fold_towel') === (frame.folding !== undefined), 'fold scene data');
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
