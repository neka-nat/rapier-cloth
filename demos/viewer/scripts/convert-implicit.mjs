// Convert the public towel example's diagnostic JSONL without resampling its states.
import {readFileSync, writeFileSync} from 'node:fs';
import {gunzipSync, gzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import {validateRecording} from '../src/recording.js';

const [source, destination, ...extra] = process.argv.slice(2);
if (!source || !destination || extra.length) {
  console.error('Usage: npm run convert:implicit -- INPUT.jsonl[.gz] NEW_OUTPUT.json[.gz]');
  process.exit(1);
}
const requireValue = (ok, message) => { if (!ok) throw new Error(`Invalid implicit recording: ${message}`); };
const bytes = readFileSync(source);
const text = (source.endsWith('.gz') ? gunzipSync(bytes) : bytes).toString('utf8');
const [config, ...rows] = text.trim().split(/\r?\n/).map(line => JSON.parse(line));
requireValue(config.kind === 'config' && config.solver === 'implicit' && config.substeps === 1, 'configuration');
requireValue([0.1, 0.04].includes(config.h), 'only the public towel fixture is supported');
const asset = new URL(`../../../examples/assets/towel-fold-${config.h === 0.1 ? '10' : '25'}hz.json`, import.meta.url);
const fixture = JSON.parse(readFileSync(asset, 'utf8'));
requireValue(config.thickness === fixture.thickness && JSON.stringify(config.x) === JSON.stringify(fixture.x), 'fixture geometry');
const steps = [];
let terminal;
for (const row of rows) {
  requireValue(!terminal, 'data after terminal record');
  if (row.kind === 'step') {
    requireValue(row.step === steps.length + 1 && row.step <= fixture.targets.length, 'accepted step ordering');
    requireValue(Math.abs(row.t - row.step * config.h) < 1e-9, 'accepted step time');
    const targets = fixture.targets[row.step - 1];
    requireValue(row.pins === (targets ? fixture.grasp.length : 0), 'pin count differs from fixture');
    if (targets) requireValue(fixture.grasp.every((index, i) => targets[i].every((value, axis) =>
      Math.abs(row.x?.[index]?.[axis] - value) < 1e-12)), 'pin targets differ from fixture');
    if (config.cap_policy !== undefined) {
      requireValue(['strict','approximate'].includes(config.cap_policy), 'cap policy');
      const o=row.outcome, count=steps.reduce((n,s)=>n+Number(s.outcome?.converged === false),0);
      requireValue(o && ['converged','approximate_iteration_cap'].includes(o.termination)
        && o.converged === (o.termination === 'converged') && Number.isFinite(o.energy)
        && ['force_rms','force_max'].every(k=>Number.isFinite(o[k]) && o[k]>=0), 'solver outcome');
      requireValue(row.approximate_steps === count + Number(!o.converged)
        && (o.converged || config.cap_policy === 'approximate' && row.iterations === (config.newton_iterations ?? 80) && row.max_edge_extension < 0.03), 'approximate history');
    }
    steps.push(row);
  } else {
    requireValue(['completed', 'failure'].includes(row.kind), 'record kind');
    terminal = row;
  }
}
if (terminal?.kind === 'completed') requireValue(terminal.steps === steps.length && steps.length === fixture.targets.length, 'incomplete completion');
if (terminal?.kind === 'failure') requireValue(terminal.committed === false && terminal.step === steps.length + 1 && typeof terminal.error === 'string' && terminal.error.length > 0, 'failure state');
requireValue(terminal?.kind === 'completed' || steps.length < fixture.targets.length, 'missing completion record');
const body = {id:0, translation:[0, 0, 0], rotation:[0, 0, 0, 1]};
const frame = (step, time, positions, pins, phase, diagnostics) => ({
  step, time, positions, phase, diagnostics, bodies:[body],
  pinned_particles:pins, attached_particles:[], anchors:[],
});
const {x, masses, kind, ...settings} = config;
const record = {
  schema_version:2, precision:'f64', config:settings,
  provenance:{source_sha256:createHash('sha256').update(bytes).digest('hex'), fixture:asset.pathname.split('/').at(-1)},
  triangles:fixture.faces,
  // Display a finite proxy for the Rapier halfspace, with the exact top height.
  shapes:[{id:0, kind:'box', half_extents:[1, 0.025, 1], local_translation:[0, config.thickness / 2 - 0.025, 0], color:'#34483e'}],
  frames:[frame(0, 0, x, [], 'Initial state', null), ...steps.map((row, i) => frame(
    row.step, row.t, row.x, row.pins ? fixture.grasp : [],
    (row.pins ? 'Folding' : (i > 0 && steps[i - 1].pins ? 'Release' : 'Settling'))
      + (row.outcome?.converged === false ? ' · Approximate (not converged)' : '')
      + (row.approximate_steps ? ` · Approximate steps: ${row.approximate_steps}` : ''),
    {max_edge_extension:row.max_edge_extension, rms_speed:row.rms_speed, max_speed:row.max_speed,
      ...(row.outcome ? {outcome:row.outcome, approximate_steps:row.approximate_steps} : {})},
  ))],
  outcome:{stop_reason:terminal?.kind === 'completed' ? 'completed' : terminal ? 'solver_error' : 'step_limit',
    steps:steps.length, end_step:fixture.targets.length, failure:terminal?.kind === 'failure' ? terminal.error : null},
  ...(terminal?.kind === 'completed' ? {summary:terminal} : {}),
};
validateRecording(record);
const output = Buffer.from(JSON.stringify(record));
writeFileSync(destination, destination.endsWith('.gz') ? gzipSync(output, {level:9}) : output, {flag:'wx'});
console.log(`Saved ${record.frames.length} accepted states to ${destination} (${record.outcome.stop_reason}).`);
