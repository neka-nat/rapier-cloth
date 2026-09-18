import {test,expect} from '@playwright/test';
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import path from 'node:path';
import {spawnSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';
const fixture = path.resolve('public/sample-f64.json');
const record = JSON.parse(fs.readFileSync(fixture,'utf8'));
async function seek(page,index) {
  await page.getByRole('slider').fill(String(index));
  await expect.poll(()=>page.evaluate(()=>window.__clothReplay.snapshot()?.frameIndex)).toBe(index);
}
test('Rust frames match actual GPU buffers and body poses through seek, playback and camera controls',async({page},testInfo)=>{
  const errors=[]; page.on('pageerror',e=>errors.push(e.message));page.on('console',m=>{if(m.type()==='error')errors.push(m.text());});
  await page.goto('/');
  await expect(page.locator('#precision')).toHaveText('f64');
  const middle=Math.floor(record.frames.length/2);
  for (const index of [0,middle,record.frames.length-1,middle,0]) {
    await seek(page,index);
    const actual=await page.evaluate(()=>window.__clothReplay.snapshot());const source=record.frames[index];
    expect(actual.positions).toEqual(source.positions.flat().map(Math.fround));
    expect(actual.normals.every(Number.isFinite)).toBe(true);
    for (const axis of [0,1,2]) {
      const values=source.positions.map(p=>Math.fround(p[axis]));
      expect(actual.bounds[axis]).toBe(Math.min(...values));expect(actual.bounds[axis+3]).toBe(Math.max(...values));
    }
    expect(actual.bodies.map(({local_translation,...pose})=>pose)).toEqual(source.bodies);
    expect(actual.bodies.map(b=>b.local_translation)).toEqual(record.shapes.map(s=>s.local_translation));
    expect(actual.anchors).toEqual(source.anchors.flat().map(Math.fround));
    expect(actual.pins).toEqual(source.pinned_particles.flatMap(i=>source.positions[i]).map(Math.fround));
    await expect(page.locator('#stretch')).toHaveText(`${(source.diagnostics.p95_stretch*100).toFixed(3)} %`);
  }
  await page.getByRole('button',{name:'Play',exact:true}).click();
  await expect.poll(()=>page.evaluate(()=>window.__clothReplay.snapshot().frameIndex)).toBeGreaterThan(0);
  await page.getByRole('button',{name:'Pause',exact:true}).click();
  const stopped=await page.evaluate(()=>window.__clothReplay.snapshot().frameIndex);
  await page.waitForTimeout(100);expect(await page.evaluate(()=>window.__clothReplay.snapshot().frameIndex)).toBe(stopped);
  await page.locator('#wireframe').check();expect(await page.evaluate(()=>window.__clothReplay.snapshot().wireframe)).toBe(true);
  await page.locator('#wireframe').uncheck();
  await page.locator('#anchors').uncheck();expect(await page.evaluate(()=>window.__clothReplay.snapshot().anchorsVisible)).toBe(false);
  await page.locator('#anchors').check();
  await seek(page,middle);
  const before=await page.evaluate(()=>window.__clothReplay.snapshot().camera);
  const canvas=await page.locator('canvas').boundingBox();
  await page.mouse.move(canvas.x+300,canvas.y+200);await page.mouse.down();await page.mouse.move(canvas.x+370,canvas.y+230,{steps:10});await page.mouse.up();
  await expect.poll(()=>page.evaluate(()=>window.__clothReplay.snapshot().camera)).not.toEqual(before);
  await page.getByRole('button',{name:'Reset view'}).click();
  await page.screenshot({path:testInfo.outputPath('pick-and-place.png'),fullPage:true});
  await page.getByRole('button',{name:'Restart'}).click();await expect(page.getByRole('slider')).toHaveValue('0');
  expect(errors).toEqual([]);
});
test('file input loads recordings and reports unsupported schemas without destroying the current frame',async({page})=>{
  await page.goto('/');await expect(page.locator('#precision')).toHaveText('f64');
  await seek(page,10);
  await page.locator('#file').setInputFiles({name:'invalid.json',mimeType:'application/json',buffer:Buffer.from('{"schema_version":999}')});
  await expect(page.getByRole('alert')).toContainText('schema_version');
  expect(await page.evaluate(()=>window.__clothReplay.snapshot().frameIndex)).toBe(10);
  await page.locator('#file').setInputFiles(fixture);
  await expect(page.getByRole('alert')).toBeEmpty();await expect(page.getByRole('slider')).toHaveValue('0');
});

for (const precision of ['f32', 'f64']) {
  test(`Rust folding ${precision} recording shows both grippers and its actual stopping state`, async({page}, testInfo) => {
    test.setTimeout(240_000);
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    const supplied = process.env[`CLOTH_FOLD_RECORDING_${precision.toUpperCase()}`];
    const recordingPath = supplied ? path.resolve(supplied) : testInfo.outputPath(`fold-${precision}.json`);
    if (!supplied) {
      fs.mkdirSync(path.dirname(recordingPath), {recursive:true});
      const generated = spawnSync('cargo', ['run', '--locked', '--release', '--no-default-features', '--features', precision,
        '--example', 'fold_towel', '--', '--max-steps', '9', '--record', recordingPath], {
        cwd:fileURLToPath(new URL('../../../', import.meta.url)), encoding:'utf8', timeout:180_000,
      });
      fs.writeFileSync(testInfo.outputPath('rust.stdout'), generated.stdout ?? '');
      fs.writeFileSync(testInfo.outputPath('rust.stderr'), generated.stderr ?? '');
      expect(generated.error).toBeUndefined();
      expect(generated.status, generated.stderr).toBe(2);
    }
    const folding = JSON.parse(fs.readFileSync(recordingPath, 'utf8'));
    expect(folding.precision).toBe(precision);
    expect(folding.shapes.map(shape => shape.id)).toEqual([0, 1, 2]);
    expect(folding.frames[0].positions.length).toBe(1024);
    expect(folding.frames[0].positions).not.toEqual(folding.frames.at(-1).positions);
    await page.goto('/');
    await expect(page.locator('#precision')).toHaveText('f64');
    await page.locator('#file').setInputFiles(recordingPath);
    await expect(page.getByRole('alert')).toBeEmpty();
    await expect(page.locator('#precision')).toHaveText(precision);
    await expect(page.locator('#outcome')).toBeVisible();
    const label = {solver_error:'Simulation stopped', step_limit:'Partial recording', completed:'Trajectory completed'}[folding.outcome.stop_reason];
    await expect(page.locator('#outcome')).toContainText(label);
    if (folding.outcome.failure) await expect(page.locator('#outcome')).toContainText(folding.outcome.failure);
    for (const index of [0, Math.floor(folding.frames.length / 2), folding.frames.length - 1]) {
      await seek(page, index);
      const actual = await page.evaluate(() => window.__clothReplay.snapshot());
      const source = folding.frames[index];
      expect(actual.positions).toEqual(source.positions.flat().map(Math.fround));
      expect(actual.normals.every(Number.isFinite)).toBe(true);
      expect(actual.bodies.map(({local_translation, ...pose}) => pose)).toEqual(source.bodies);
      expect(actual.bodies.map(body => body.local_translation)).toEqual(folding.shapes.map(shape => shape.local_translation));
      expect(actual.anchors).toEqual(source.anchors.flat().map(Math.fround));
      expect(actual.pins).toEqual(source.pinned_particles.flatMap(i => source.positions[i]).map(Math.fround));
      await expect(page.locator('#phase')).toHaveText(source.phase);
    }
    await page.screenshot({path:testInfo.outputPath(`fold-${precision}.png`), fullPage:true});
    const invalid = structuredClone(folding);
    invalid.outcome.steps += 1;
    await page.locator('#file').setInputFiles({name:'inconsistent-outcome.json', mimeType:'application/json', buffer:Buffer.from(JSON.stringify(invalid))});
    await expect(page.getByRole('alert')).toContainText('outcome accepted steps');
    expect(await page.evaluate(() => window.__clothReplay.snapshot().frameIndex)).toBe(folding.frames.length - 1);
    await page.getByRole('button', {name:'Restart'}).click();
    await page.getByRole('button', {name:'Play', exact:true}).click();
    await expect.poll(() => page.evaluate(() => window.__clothReplay.snapshot().frameIndex)).toBeGreaterThan(0);
    await page.locator('#file').setInputFiles(fixture);
    await expect(page.getByRole('alert')).toBeEmpty();
    await expect(page.locator('#outcome')).toBeHidden();
    expect(errors).toEqual([]);
  });
}

const implicitFixture = path.resolve('public/towel-fold-implicit.json.gz');
const implicitRecord = JSON.parse(gunzipSync(fs.readFileSync(implicitFixture)).toString('utf8'));
test('implicit towel replay renders accepted states and releases all pins', async({page}, testInfo) => {
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', message => { if (message.type() === 'error') errors.push(message.text()); });
  await page.goto('/?sample=implicit');
  await expect(page.locator('#solver-info')).toContainText('CPU IMPLICIT');
  await expect(page.getByRole('alert')).toBeEmpty();
  await expect(page.locator('#outcome')).toContainText('Settled under');
  await expect(page.locator('#playback-note')).toContainText('14.80 s to compute');
  for (const index of [0, 20, 40, 41, 80]) {
    await seek(page, index);
    const source = implicitRecord.frames[index];
    const actual = await page.evaluate(() => window.__clothReplay.snapshot());
    expect(actual.positions).toEqual(source.positions.flat().map(Math.fround));
    expect(actual.normals.every(Number.isFinite)).toBe(true);
    expect(actual.pins).toEqual(source.pinned_particles.flatMap(i => source.positions[i]).map(Math.fround));
    await expect(page.locator('#contacts')).toHaveText(String(source.pinned_particles.length));
    await expect(page.locator('#phase')).toHaveText(source.phase);
    await expect(page.locator('#stretch')).toHaveText(source.diagnostics ? `${(source.diagnostics.max_edge_extension * 100).toFixed(3)} %` : '—');
    if ([20, 80].includes(index)) await page.screenshot({path:testInfo.outputPath(`implicit-step-${index}.png`), fullPage:true});
  }
  expect(implicitRecord.frames[40].pinned_particles).toHaveLength(53);
  expect(implicitRecord.frames[41].pinned_particles).toHaveLength(0);
  await page.getByRole('button', {name:'Play', exact:true}).click();
  await expect.poll(() => page.evaluate(() => window.__clothReplay.snapshot().frameIndex)).toBeGreaterThan(0);
  await page.getByRole('button', {name:'Pause', exact:true}).click();
  const paused = await page.evaluate(() => window.__clothReplay.snapshot().frameIndex);
  await page.waitForTimeout(150);
  expect(await page.evaluate(() => window.__clothReplay.snapshot().frameIndex)).toBe(paused);
  await page.locator('#sample').selectOption('pick');
  await expect(page.locator('#solver-info')).toContainText('XPBD');
  await expect(page.locator('#stretch-label')).toHaveText('Stretch · p95');
  await page.locator('#file').setInputFiles(implicitFixture);
  await expect(page.locator('#solver-info')).toContainText('CPU IMPLICIT');
  const partial = structuredClone(implicitRecord);
  partial.frames = partial.frames.slice(0, 3);
  partial.outcome = {stop_reason:'solver_error', steps:2, end_step:80, failure:'Example failure'};
  delete partial.summary;
  await page.locator('#file').setInputFiles({name:'partial.json', mimeType:'application/json', buffer:Buffer.from(JSON.stringify(partial))});
  await expect(page.locator('#outcome')).toContainText('Simulation stopped at step 2');
  await expect(page.getByRole('alert')).toBeEmpty();
  const invalid = structuredClone(implicitRecord);
  invalid.summary.simulated_seconds = 10;
  await page.locator('#file').setInputFiles({name:'invalid.json', mimeType:'application/json', buffer:Buffer.from(JSON.stringify(invalid))});
  await expect(page.getByRole('alert')).toContainText('summary duration');
  await expect(page.locator('#outcome')).toContainText('Simulation stopped at step 2');
  expect(errors).toEqual([]);
});
test('implicit link autoplays to the last recorded state', async({page}) => {
  await page.goto('/?sample=implicit&play=1');
  await expect(page.getByRole('button', {name:'Pause', exact:true})).toBeVisible();
  await expect.poll(() => page.evaluate(() => window.__clothReplay.snapshot()?.frameIndex), {timeout:15_000}).toBe(80);
  await expect(page.getByRole('button', {name:'Play', exact:true})).toBeVisible();
  await expect(page.locator('#time')).toHaveText('8.000 s');
});
