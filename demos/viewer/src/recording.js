const finiteVector = (v, n) => Array.isArray(v) && v.length === n && v.every(Number.isFinite);
const requireValue = (ok, message) => { if (!ok) throw new Error(`記録の形式が不正です: ${message}`); };
export function validateRecording(record) {
  requireValue(record?.schema_version === 1, 'schema_version 1 が必要です');
  requireValue(['f32', 'f64'].includes(record.precision), 'precision');
  requireValue(Number.isFinite(record.config?.h) && record.config.h > 0, 'h');
  requireValue(Array.isArray(record.frames) && record.frames.length > 0, 'frames');
  const count = record.frames[0]?.positions?.length;
  requireValue(Number.isSafeInteger(count) && count >= 3, '頂点数');
  requireValue(Array.isArray(record.triangles) && record.triangles.length > 0, 'triangles');
  for (const triangle of record.triangles) requireValue(Array.isArray(triangle) && triangle.length === 3 && triangle.every(i => Number.isSafeInteger(i) && i >= 0 && i < count), '面の添字');
  requireValue(Array.isArray(record.shapes), 'shapes');
  const ids = new Set();
  for (const shape of record.shapes) {
    requireValue(Number.isSafeInteger(shape.id) && !ids.has(shape.id), 'shape id'); ids.add(shape.id);
    requireValue(shape.kind === 'box' && finiteVector(shape.half_extents, 3) && shape.half_extents.every(v => v > 0), 'shape');
    requireValue(finiteVector(shape.local_translation, 3) && /^#[\da-f]{6}$/i.test(shape.color), 'shape transform / color');
  }
  let lastTime = -1, lastStep = -1;
  for (const frame of record.frames) {
    requireValue(Number.isFinite(frame.time) && frame.time >= 0 && frame.time > lastTime, '時刻の順序');
    requireValue(Number.isSafeInteger(frame.step) && frame.step >= 0 && frame.step > lastStep, 'step の順序');
    requireValue(Math.abs(frame.time - frame.step * record.config.h) < 1e-5, 'step と時刻');
    lastTime = frame.time; lastStep = frame.step;
    requireValue(typeof frame.phase === 'string', 'phase');
    requireValue(Array.isArray(frame.positions) && frame.positions.length === count && frame.positions.every(p => finiteVector(p, 3)), '頂点位置');
    requireValue(Array.isArray(frame.bodies) && frame.bodies.length === ids.size, 'body 数');
    const seen = new Set();
    for (const body of frame.bodies) {
      requireValue(ids.has(body.id) && !seen.has(body.id), 'body id'); seen.add(body.id);
      requireValue(finiteVector(body.translation, 3) && finiteVector(body.rotation, 4) && Math.abs(body.rotation.reduce((s, x) => s + x*x, 0) - 1) < 1e-3, 'body pose');
    }
    for (const key of ['attached_particles', 'pinned_particles']) requireValue(Array.isArray(frame[key]) && frame[key].every(i => Number.isSafeInteger(i) && i >= 0 && i < count), key);
    requireValue(Array.isArray(frame.anchors) && frame.anchors.length === frame.attached_particles.length && frame.anchors.every(p => finiteVector(p, 3)), 'anchors');
    requireValue(frame.diagnostics && ['max_stretch','p95_stretch','max_penetration','max_target_error','contacts'].every(key => Number.isFinite(frame.diagnostics[key]) && frame.diagnostics[key] >= 0), '診断値');
  }
  return record;
}
