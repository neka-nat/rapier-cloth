import {test,expect} from '@playwright/test';

const snapshot = page => page.evaluate(() => window.__clothLive.snapshot());
async function idle(page) { await expect.poll(async () => { const s = await snapshot(page); return !!s?.connected && !s.pending && s.queued === 0; }).toBe(true); }
async function advance(page) {
  const old = (await snapshot(page)).step;
  await page.getByRole('button',{name:'Step frame'}).click();
  await expect.poll(async () => (await snapshot(page)).step).toBe(old + 4);
  await idle(page);
}

test('live Rust frames reach GPU buffers; pause, controls, release, reset and camera work',async({page},testInfo) => {
  const errors = [], frames = new Map(), recordings = [];
  page.on('pageerror',e => errors.push(e.message));
  page.on('request',r => { if (r.url().includes('sample-f64.json')) recordings.push(r.url()); });
  // Observe real wire messages independently of the render code.
  page.on('websocket',socket => socket.on('framereceived',({payload}) => {
    const frame = JSON.parse(String(payload)); if (frame.type === 'frame') frames.set(frame.request_id,frame);
  }));
  await page.goto('/live.html?paused=1'); await idle(page);
  const initial = await snapshot(page);
  expect(initial.step).toBe(0); expect(initial.positions).toHaveLength(3072);
  for (let i=0;i<5;i++) await advance(page);
  const actual = await snapshot(page), wire = frames.get(actual.requestId);
  expect(actual.positions).toEqual(wire.positions.map(Math.fround));
  expect(actual.sphere).toEqual(wire.sphere);
  expect(actual.normals.every(Number.isFinite)).toBe(true);
  for (const axis of [0,1,2]) {
    const values = wire.positions.filter((_,i) => i%3 === axis).map(Math.fround);
    expect(actual.bounds[axis]).toBe(Math.min(...values)); expect(actual.bounds[axis+3]).toBe(Math.max(...values));
  }
  expect(actual.positions).not.toEqual(initial.positions);
  await page.waitForTimeout(200); expect((await snapshot(page)).step).toBe(actual.step);
  await page.locator('#auto-motion').uncheck();
  await page.locator('#sphere-x').fill('0.4'); await idle(page);
  expect((await snapshot(page)).options.sphere_x).toBeCloseTo(0.4,6);
  expect((await snapshot(page)).step).toBe(actual.step);
  await page.getByRole('button',{name:'Resume',exact:true}).click();
  await expect.poll(async () => (await snapshot(page)).sphere[0]).toBeGreaterThan(0.1);
  await page.getByRole('button',{name:'Pause',exact:true}).click(); await idle(page);
  const paused = await snapshot(page); await page.waitForTimeout(200);
  expect((await snapshot(page)).step).toBe(paused.step);
  await page.screenshot({path:testInfo.outputPath('live-drape.png'),fullPage:true});
  await page.getByRole('button',{name:'Reset',exact:true}).click(); await idle(page);
  expect((await snapshot(page)).positions).toEqual(initial.positions);
  expect((await snapshot(page)).step).toBe(0);

  await page.locator('#scene').selectOption('hanging'); await idle(page);
  expect((await snapshot(page)).pins).toHaveLength(32);
  const pinned = await snapshot(page);
  expect(pinned.pinPositions).toEqual(pinned.pins.flatMap(i => pinned.sourcePositions.slice(3*i,3*i+3)).map(Math.fround));
  await page.locator('#wind').fill('1'); await idle(page);
  await page.getByRole('button',{name:'Resume',exact:true}).click();
  await expect.poll(async () => (await snapshot(page)).time).toBeGreaterThan(0.8);
  await page.getByRole('button',{name:'Pause',exact:true}).click(); await idle(page);
  const beforeRelease = await snapshot(page);
  await page.getByRole('button',{name:'Release pins'}).click(); await idle(page);
  expect((await snapshot(page)).pins).toEqual([]);
  expect((await snapshot(page)).pinPositions).toEqual([]);
  expect((await snapshot(page)).positions).toEqual(beforeRelease.positions);
  await advance(page);
  expect((await snapshot(page)).positions[1]).toBeLessThan(beforeRelease.positions[1]);
  await page.locator('#wireframe').check(); expect((await snapshot(page)).wireframe).toBe(true);
  await page.locator('#wireframe').uncheck();
  const camera = (await snapshot(page)).camera;
  const box = await page.locator('canvas').boundingBox();
  await page.mouse.move(box.x+300,box.y+200); await page.mouse.down(); await page.mouse.move(box.x+380,box.y+230,{steps:10}); await page.mouse.up();
  await expect.poll(async () => (await snapshot(page)).camera).not.toEqual(camera);
  await page.getByRole('button',{name:'Reset view'}).click();
  await page.screenshot({path:testInfo.outputPath('live-cloth.png'),fullPage:true});
  await page.evaluate(() => {
    const scene = document.getElementById('scene'), wind = document.getElementById('wind');
    scene.value = 'hanging'; scene.dispatchEvent(new Event('change'));
    wind.value = '0.9'; wind.dispatchEvent(new Event('input'));
    wind.value = '0.6'; wind.dispatchEvent(new Event('input'));
  });
  await idle(page);
  expect((await snapshot(page)).options.wind).toBeCloseTo(0.6,6);
  await expect(page.locator('#wind')).toHaveValue('0.6');
  expect(errors).toEqual([]); expect(recordings).toEqual([]);
});

test('separate connections own state; invalid commands fail without advancing it',async({page,request}) => {
  await page.goto('/live.html?paused=1'); await idle(page); await advance(page);
  const primary = await snapshot(page);
  const result = await page.evaluate(async () => {
    const socket = new WebSocket(`ws://${location.host}/live/ws`);
    const messages = [];
    return await new Promise((resolve,reject) => {
      socket.onerror = () => reject(new Error('test connection failed'));
      socket.onmessage = ({data}) => {
        const frame = JSON.parse(data); messages.push(frame);
        if (messages.length === 1) socket.send(JSON.stringify({request_id:1,command:{type:'set_options',options:{auto_motion:false,sphere_x:99,sphere_z:0,wind:0}}}));
        if (messages.length === 2) socket.send(JSON.stringify({request_id:2,command:{type:'step'}}));
        if (messages.length === 3) { socket.close(); resolve(messages); }
      };
    });
  });
  expect(result[0].step).toBe(0); expect(result[1].type).toBe('error'); expect(result[2].step).toBe(4);
  expect((await snapshot(page)).positions).toEqual(primary.positions);
  const forbidden = await request.get('http://127.0.0.1:9174/live/ws',{headers:{Origin:'https://unrelated.example',Connection:'Upgrade',Upgrade:'websocket','Sec-WebSocket-Version':'13','Sec-WebSocket-Key':'dGhlIHNhbXBsZSBub25jZQ=='}});
  expect(forbidden.status()).toBe(403);
});

test('disconnect freezes the displayed state and reconnect starts a new live world',async({page}) => {
  // Keep the real browser WebSocket/Origin/transport and close it externally.
  // This hook exists only in the test, not in the application's debug API.
  await page.addInitScript(() => {
    const NativeSocket = window.WebSocket;
    window.WebSocket = class extends NativeSocket {
      constructor(...args) { super(...args); window.__testClothSocket = this; }
    };
  });
  await page.goto('/live.html?paused=1'); await idle(page); await advance(page);
  const before = await snapshot(page);
  await page.evaluate(() => window.__testClothSocket.close(1000,'test disconnect'));
  await expect(page.locator('#status')).toHaveText('Disconnected');
  expect((await snapshot(page)).positions).toEqual(before.positions);
  await expect(page.locator('#play')).toBeDisabled();
  await page.getByRole('button',{name:'Reconnect',exact:true}).click(); await idle(page);
  expect((await snapshot(page)).step).toBe(0); await advance(page);
});
