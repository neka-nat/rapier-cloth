import {test,expect} from '@playwright/test';
import fs from 'node:fs';
import path from 'node:path';
import {gunzipSync} from 'node:zlib';
import {spawnSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import {validateFrame} from '../../src/live-protocol.js';
const enabled = process.env.CLOTH_LIVE_IMPLICIT === '1';
test.skip(!enabled,'Run with CLOTH_LIVE_IMPLICIT=1 (f64,implicit server)');
const snapshot = page => page.evaluate(()=>window.__clothLive.snapshot());
async function idle(page) {
  await expect.poll(async()=>{const s=await snapshot(page);return !!s?.connected && !s.pending && s.queued===0;}).toBe(true);
}
async function advance(page) {
  const old=(await snapshot(page)).step;
  await page.getByRole('button',{name:'Step frame',exact:true}).click();
  await expect.poll(async()=>(await snapshot(page)).step).toBe(old+1);await idle(page);
}
test('implicit pose commands, independent release and reset use actual Rapier frames',async({page},testInfo)=>{
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.goto('/live.html?scene=implicit_towel&paused=1');await idle(page);
  await expect.poll(async()=>(await snapshot(page)).scene).toBe('implicit_towel');await idle(page);
  const initial=await snapshot(page);
  expect(initial.implicit.cap_policy).toBe("strict");
  expect(initial.implicit.sample.approximate_steps).toBe(0);
  expect(initial.implicit.sample.held_vertices).toBe(53);
  expect(initial.anchors).toHaveLength(159);
  expect(initial.grippers.every(g=>g.visible)).toBe(true);
  const transport=await page.locator('.transport').boundingBox();
  expect(transport.y+transport.height).toBeLessThanOrEqual(1050);
  await page.waitForTimeout(300);
  const renders=(await snapshot(page)).renderCount;
  await page.waitForTimeout(250);expect((await snapshot(page)).renderCount).toBe(renders);
  const y=initial.implicit.grippers[0].translation[1];
  await page.getByRole('button',{name:'Y +',exact:true}).click();await idle(page);
  const queued=await snapshot(page);
  expect(queued.step).toBe(0);expect(queued.positions).toEqual(initial.positions);
  expect(queued.implicit.automatic).toBe(false);
  expect(queued.implicit.desired[0].translation[1]).toBeCloseTo(y+0.005,12);
  await advance(page);
  const moved=await snapshot(page);
  expect(moved.implicit.grippers[0].translation[1]).toBeCloseTo(y+0.005,12);
  expect(moved.positions).toEqual(moved.sourcePositions.map(Math.fround));
  await page.getByRole('button',{name:'Roll +',exact:true}).click();await idle(page);await advance(page);
  expect((await snapshot(page)).implicit.grippers[0].rotation[0]).toBeCloseTo(Math.sin(Math.PI/72),12);
  const held=await snapshot(page);
  await page.getByRole('button',{name:'Release left gripper'}).click();await idle(page);
  const released=await snapshot(page);
  expect(released.positions).toEqual(held.positions);expect(released.step).toBe(held.step);
  expect(released.implicit.grippers.map(g=>g.holding)).toEqual([false,true]);
  expect(released.implicit.sample.held_vertices).toBe(26);
  await page.getByRole('button',{name:'Grasp selected patch'}).click();await idle(page);
  expect((await snapshot(page)).implicit.sample.held_vertices).toBe(53);
  await page.screenshot({path:testInfo.outputPath('implicit-pose-control.png'),fullPage:true});
  await page.getByRole('button',{name:'Reset',exact:true}).click();await idle(page);
  expect((await snapshot(page)).positions).toEqual(initial.positions);expect((await snapshot(page)).step).toBe(0);
  expect((await snapshot(page)).implicit.automatic).toBe(true);
  expect(errors).toEqual([]);
});
for (const variant of ['nominal','lift_5mm']) test(`implicit ${variant} fold preserves outcomes and matches headless states through release and settling`,async({page},testInfo)=>{
  test.setTimeout(600_000);
  const reference=process.env.CLOTH_IMPLICIT_REFERENCE_DIR ? path.join(process.env.CLOTH_IMPLICIT_REFERENCE_DIR,`${variant}.jsonl.gz`) : testInfo.outputPath('headless.jsonl');
  if (!process.env.CLOTH_IMPLICIT_REFERENCE_DIR) {
    fs.mkdirSync(path.dirname(reference),{recursive:true});
    const run=spawnSync('cargo',['run','--locked','--release','--no-default-features','--features','f64,implicit','--example','robot_towel_implicit','--','--case',variant,'--cap-policy','approximate','--output',reference],
      {cwd:fileURLToPath(new URL('../../../../',import.meta.url)),encoding:'utf8',timeout:240000});
    expect(run.status,run.stderr).toBe(0);
  }
  const bytes=fs.readFileSync(reference);
  const rows=(reference.endsWith('.gz') ? gunzipSync(bytes) : bytes).toString('utf8').trim().split('\n').map(JSON.parse);
  const referenceSteps=rows.filter(r=>r.kind==='step');
  const errors=[], frames=[];
  await page.addInitScript(() => {
    const Native = window.WebSocket;
    window.__implicitLatency = {start:null,end:null,responses:[]};
    window.WebSocket = class extends Native {
      constructor(...args) {
        super(...args); this.sent = new Map();
        if (!String(args[0]).includes('/live/ws')) return;
        this.addEventListener('message', ({data}) => {
          const f = JSON.parse(data), now = performance.now(), cost = window.__implicitLatency;
          if (f.type === 'frame' && f.scene === 'implicit_towel' && f.advanced_substeps === 1) {
            cost.responses.push(now - this.sent.get(f.request_id)); cost.end = now;
          }
          this.sent.delete(f.request_id);
        });
      }
      send(data) {
        const request = JSON.parse(data), now = performance.now(), cost = window.__implicitLatency;
        this.sent.set(request.request_id,now);
        if (request.command?.type === 'step' && cost.start === null) cost.start = now;
        super.send(data);
      }
    };
  });

  page.on('pageerror',e=>errors.push(e.message));
  page.on('websocket',socket=>{
    socket.on('framereceived',({payload})=>{
      const f=JSON.parse(String(payload));
      if(f.type==='frame' && f.scene==='implicit_towel' && f.advanced_substeps===1)frames.push(f);
    });
  });
  await page.goto(`/live.html?scene=implicit_towel&paused=1&case=${variant}&cap_policy=approximate`);await idle(page);
  await expect.poll(async()=>(await snapshot(page)).scene).toBe('implicit_towel');await idle(page);
  if (variant === 'lift_5mm') {
    // The former iteration-cap step now converges within the default budget.
    for (let i=0;i<24;i++) await advance(page);
    await expect(page.locator('#solver-state')).toContainText('Converged · Approximate steps: 0');
    await page.screenshot({path:testInfo.outputPath('implicit-lift-step-24.png'),fullPage:true});
  }
  await page.getByRole('button',{name:'Resume',exact:true}).click();
  await expect.poll(async()=>(await snapshot(page)).step,{timeout:240000,intervals:[200]}).toBe(80);
  await idle(page);
  const final=await snapshot(page);
  expect(final.implicit.stopped).toBeNull();expect(final.implicit.completed).toBe(true);
  expect(final.implicit.sample.held_vertices).toBe(0);
  await expect(page.locator('#status')).toHaveText('Completed');
  expect(frames).toHaveLength(80);
  for(let i=0;i<frames.length;i++){
    validateFrame(frames[i]);
    expect(frames[i].positions).toEqual(referenceSteps[i].x.flat());
    for (const key of ['outcome','approximate_steps','iterations']) expect(frames[i].implicit.sample[key]).toEqual(referenceSteps[i].sample[key]);
    expect(frames[i].implicit.grippers.map(g=>({translation:g.translation,rotation:g.rotation}))).toEqual(referenceSteps[i].grippers);
  }
  const approximate = frames.filter(f=>!f.implicit.sample.outcome.converged);
  expect(approximate.length).toBe(0);
  expect(final.implicit.sample.approximate_steps).toBe(approximate.length);
  await expect(page.locator('#solver-state')).toContainText(`Approximate steps: ${approximate.length}`);
  for (const f of approximate) {
    expect(f.implicit.sample.outcome.termination).toBe('approximate_iteration_cap');
    expect(f.advanced_substeps).toBe(1); expect(f.implicit.stopped).toBeNull();
    const corrupted=structuredClone(f);corrupted.implicit.sample.outcome.converged=true;
    expect(()=>validateFrame(corrupted)).toThrow('implicit outcome');
  }
  fs.writeFileSync(testInfo.outputPath('accepted-frames.jsonl'),frames.map(f=>JSON.stringify(f)).join('\n')+'\n');
  expect(frames[39].implicit.sample.held_vertices).toBe(53);
  expect(frames[40].implicit.sample.held_vertices).toBe(0);
  expect(final.positions).toEqual(frames.at(-1).positions.map(Math.fround));
  const cost=await page.evaluate(()=>window.__implicitLatency),response=cost.responses;
  expect(response).toHaveLength(80);
  const sorted=[...response].sort((a,b)=>a-b);
  const metrics={wall_ms:cost.end-cost.start,steps:80,simulated_seconds:8,
    physics_ms:frames.map(f=>f.physics_ms),response_ms:response,
    p95_response_ms:sorted[Math.ceil(sorted.length*.95)-1],max_response_ms:sorted.at(-1)};
  fs.writeFileSync(testInfo.outputPath('implicit-latency.json'),JSON.stringify(metrics,null,2));
  await page.screenshot({path:testInfo.outputPath('implicit-live-settled.png'),fullPage:true});
  await page.getByRole('button',{name:'Reset',exact:true}).click();await idle(page);
  const reset=await snapshot(page);
  expect(reset.step).toBe(0);expect(reset.implicit.sample.approximate_steps).toBe(0);
  expect(reset.implicit.sample.outcome).toBeNull();expect(reset.implicit.cap_policy).toBe('approximate');
  expect(errors).toEqual([]);
});
