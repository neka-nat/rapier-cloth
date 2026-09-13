import {test,expect} from '@playwright/test';
import fs from 'node:fs';
import path from 'node:path';
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
