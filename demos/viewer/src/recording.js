const finiteVector = (v, n) => Array.isArray(v) && v.length === n && v.every(Number.isFinite);
const requireValue = (ok, message) => { if (!ok) throw new Error(`Invalid recording: ${message}`); };
export function validateRecording(record) {
  const implicit = record?.schema_version === 2 && record?.config?.solver === 'implicit';
  requireValue(record?.schema_version === 1 || implicit, 'schema_version 1 or implicit schema_version 2 is required');
  requireValue(['f32', 'f64'].includes(record.precision), 'precision');
  requireValue(Number.isFinite(record.config?.h) && record.config.h > 0, 'h');
  requireValue(Array.isArray(record.frames) && record.frames.length > 0, 'frames');
  const count = record.frames[0]?.positions?.length;
  requireValue(Number.isSafeInteger(count) && count >= 3, 'vertex count');
  requireValue(Array.isArray(record.triangles) && record.triangles.length > 0, 'triangles');
  for (const triangle of record.triangles) requireValue(Array.isArray(triangle) && triangle.length === 3 && triangle.every(i => Number.isSafeInteger(i) && i >= 0 && i < count), 'triangle indices');
  requireValue(Array.isArray(record.shapes), 'shapes');
  const ids = new Set();
  for (const shape of record.shapes) {
    requireValue(Number.isSafeInteger(shape.id) && !ids.has(shape.id), 'shape id'); ids.add(shape.id);
    requireValue(shape.kind === 'box' && finiteVector(shape.half_extents, 3) && shape.half_extents.every(v => v > 0), 'shape');
    requireValue(finiteVector(shape.local_translation, 3) && /^#[\da-f]{6}$/i.test(shape.color), 'shape transform / color');
  }
  let lastTime = -1, lastStep = -1;
  for (const frame of record.frames) {
    requireValue(Number.isFinite(frame.time) && frame.time >= 0 && frame.time > lastTime, 'time ordering');
    requireValue(Number.isSafeInteger(frame.step) && frame.step >= 0 && frame.step > lastStep, 'step ordering');
    requireValue(Math.abs(frame.time - frame.step * record.config.h) < 1e-5, 'step and time');
    lastTime = frame.time; lastStep = frame.step;
    requireValue(typeof frame.phase === 'string', 'phase');
    requireValue(Array.isArray(frame.positions) && frame.positions.length === count && frame.positions.every(p => finiteVector(p, 3)), 'vertex positions');
    requireValue(Array.isArray(frame.bodies) && frame.bodies.length === ids.size, 'body count');
    const seen = new Set();
    for (const body of frame.bodies) {
      requireValue(ids.has(body.id) && !seen.has(body.id), 'body id'); seen.add(body.id);
      requireValue(finiteVector(body.translation, 3) && finiteVector(body.rotation, 4) && Math.abs(body.rotation.reduce((s, x) => s + x*x, 0) - 1) < 1e-3, 'body pose');
    }
    for (const key of ['attached_particles', 'pinned_particles']) requireValue(Array.isArray(frame[key]) && frame[key].every(i => Number.isSafeInteger(i) && i >= 0 && i < count), key);
    requireValue(Array.isArray(frame.anchors) && frame.anchors.length === frame.attached_particles.length && frame.anchors.every(p => finiteVector(p, 3)), 'anchors');
    if (implicit) {
      requireValue((frame.step === 0 && frame.diagnostics === null) ||
        (frame.diagnostics && ['max_edge_extension', 'rms_speed', 'max_speed'].every(key => Number.isFinite(frame.diagnostics[key]) && frame.diagnostics[key] >= 0)), 'implicit diagnostics');
    } else {
      requireValue(frame.diagnostics && ['max_stretch','p95_stretch','max_penetration','max_target_error','contacts'].every(key => Number.isFinite(frame.diagnostics[key]) && frame.diagnostics[key] >= 0), 'diagnostics');
    }
  }
  if (record.outcome !== undefined) {
    const result = record.outcome;
    requireValue(result && ['solver_error', 'step_limit', 'completed'].includes(result.stop_reason), 'outcome stop_reason');
    requireValue(Number.isSafeInteger(result.steps) && result.steps === lastStep, 'outcome accepted steps');
    requireValue(Number.isSafeInteger(result.end_step) && result.end_step > 0 && result.end_step >= result.steps, 'outcome end step');
    requireValue((result.stop_reason === 'completed') === (result.steps === result.end_step), 'outcome completion');
    requireValue(result.stop_reason === 'solver_error'
      ? typeof result.failure === 'string' && result.failure.length > 0
      : result.failure === null, 'outcome failure');
  }
  if (implicit) {
    requireValue(record.precision === 'f64' && record.outcome, 'implicit precision / outcome');
    if (record.outcome.stop_reason === 'completed') {
      requireValue(typeof record.summary?.settled === 'boolean', 'settled summary');
      requireValue(['simulation_wall_seconds', 'simulated_seconds', 'final_window_drift'].every(key => Number.isFinite(record.summary[key]) && record.summary[key] >= 0), 'implicit summary');
      requireValue(Math.abs(record.summary.simulated_seconds - lastTime) < 1e-5, 'summary duration');
    }
  }
  return record;
}
