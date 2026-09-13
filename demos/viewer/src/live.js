import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import './style.css';
import './live.css';

const $ = id => document.getElementById(id);
const scene = new THREE.Scene(); scene.background = new THREE.Color('#192522');
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); renderer.shadowMap.enabled = true;
$('viewport').appendChild(renderer.domElement);
const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 30);
const controls = new OrbitControls(camera, renderer.domElement); controls.enableDamping = true;
function resetCamera() { camera.position.set(1.65, 1.65, 2.05); controls.target.set(0, 0.45, 0); controls.update(); }
resetCamera();
scene.add(new THREE.HemisphereLight('#e4fff0', '#657466', 2.6));
const light = new THREE.DirectionalLight('#fff5dd', 3); light.position.set(-1.5, 3, 1.5); light.castShadow = true;
light.shadow.mapSize.set(1024, 1024); light.shadow.camera.left = light.shadow.camera.bottom = -2;
light.shadow.camera.right = light.shadow.camera.top = 2; light.shadow.normalBias = 0.003; scene.add(light);
const floor = new THREE.Mesh(new THREE.PlaneGeometry(5, 5), new THREE.MeshStandardMaterial({color:'#283b31',roughness:1}));
floor.rotation.x = -Math.PI/2; floor.receiveShadow = true; scene.add(floor);
const grid = new THREE.GridHelper(5, 50, '#6d8876', '#43594b'); grid.position.y = 0.001;
grid.material.transparent = true; grid.material.opacity = 0.35; scene.add(grid);
const sphere = new THREE.Mesh(new THREE.SphereGeometry(1, 40, 24), new THREE.MeshStandardMaterial({color:'#deac78',roughness:0.48}));
sphere.castShadow = true; sphere.receiveShadow = true; scene.add(sphere);
const material = new THREE.MeshStandardMaterial({color:'#a2e4ba',side:THREE.DoubleSide,roughness:0.85});
let cloth = null;
const pins = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({color:'#ff98bd',size:0.02,depthTest:false}));
const pinAttribute = new THREE.BufferAttribute(new Float32Array(32*3),3).setUsage(THREE.DynamicDrawUsage);
pins.geometry.setAttribute('position',pinAttribute); pins.geometry.setDrawRange(0,0); pins.frustumCulled = false;
scene.add(pins);
let socket, ready = false, pending = null, queue = [], nextId = 1, source = null;
let running = new URLSearchParams(location.search).get('paused') !== '1';
let nextRequestAt = 0, lastRoundtrip = 0, renderCount = 0;
let metricStart = performance.now(), metricTime = 0, metricRenders = 0;
let connectionTimeout;

function setStatus(text, state = '') { $('status').textContent = text; $('status').dataset.state = state; }
function updateControls() {
  for (const id of ['play','reset','scene','wind','auto-motion']) $(id).disabled = !ready;
  $('step').disabled = !ready || running;
  $('release').disabled = !ready || !source?.pins.length;
  for (const id of ['sphere-x','sphere-z']) $(id).disabled = !ready || $('auto-motion').checked;
  $('play').textContent = running ? 'Pause' : 'Resume';
  if (ready) setStatus(document.hidden ? 'Tab hidden · idle' : running ? 'Running' : pending ? 'Pausing' : 'Paused');
}
function fail(message) {
  running = false; pending = null; queue = [];
  $('error').textContent = message; updateControls(); setStatus('Stopped · error', 'error');
}
function enqueue(command) {
  if (!ready) return;
  if (command.type === 'reset') queue = [];
  // Retain only the latest pending value of each control, including while a
  // frame is in flight. Controls never build an unbounded step backlog.
  queue = queue.filter(item => item.type !== command.type);
  queue.push(command);
  dispatch(performance.now());
}
function dispatch(now) {
  if (!ready || pending || socket?.readyState !== WebSocket.OPEN) return;
  let command = queue.shift();
  if (!command && running && !document.hidden && now + 0.2 >= nextRequestAt) command = {type:'step'};
  if (!command) return;
  const request_id = nextId++;
  pending = {request_id,command,start:now};
  if (command.type === 'step') nextRequestAt = Math.max(nextRequestAt + 1000/60, now + 1);
  socket.send(JSON.stringify({request_id,command}));
  updateControls();
}
function validate(frame) {
  if (frame.protocol !== 1 || frame.positions?.length !== 3072 || !frame.positions.every(Number.isFinite)
      || frame.sphere?.length !== 3 || !frame.sphere.every(Number.isFinite)
      || !Number.isFinite(frame.time) || !Number.isSafeInteger(frame.step)
      || !Array.isArray(frame.pins) || frame.pins.length > 32 || !frame.pins.every(i => Number.isInteger(i) && i >= 0 && i < 1024)) throw new Error('The server sent invalid vertex data.');
  if (frame.triangles && (frame.triangles.length !== 5766 || !frame.triangles.every(i => Number.isInteger(i) && i >= 0 && i < 1024))) throw new Error('Invalid triangle data.');
}
function applyFrame(frame) {
  validate(frame);
  if (frame.triangles) {
    if (cloth) { cloth.geometry.dispose(); scene.remove(cloth); }
    const geometry = new THREE.BufferGeometry(); geometry.setIndex(frame.triangles);
    geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(3072),3).setUsage(THREE.DynamicDrawUsage));
    cloth = new THREE.Mesh(geometry,material); cloth.castShadow = cloth.receiveShadow = true; scene.add(cloth);
    $('scene').value = frame.scene;
    metricStart = performance.now(); metricTime = frame.time; metricRenders = renderCount;
  }
  if (!cloth) throw new Error('Initial mesh is missing. Reconnect to continue.');
  const attribute = cloth.geometry.getAttribute('position'); attribute.array.set(frame.positions); attribute.needsUpdate = true;
  cloth.geometry.computeVertexNormals(); cloth.geometry.computeBoundingBox(); cloth.geometry.computeBoundingSphere();
  sphere.position.fromArray(frame.sphere); sphere.scale.setScalar(frame.sphere_radius);
  frame.pins.forEach((i,j) => pinAttribute.array.set(frame.positions.slice(3*i,3*i+3),j*3));
  pinAttribute.needsUpdate = true; pins.geometry.setDrawRange(0,frame.pins.length); pins.visible = $('pins').checked;
  // A reset may complete while a newer slider value is queued. Preserve that
  // pending intent instead of replacing the controls with the reset defaults.
  const desired = queue.find(command => command.type === 'set_options')?.options ?? frame.options;
  $('auto-motion').checked = desired.auto_motion;
  $('wind').value = desired.wind; $('wind-value').textContent = `${Math.round(desired.wind*100)}%`;
  $('sphere-x').value = desired.sphere_x; $('sphere-z').value = desired.sphere_z;
  source = frame;
  $('precision').textContent = `RUST CPU / ${frame.precision}`;
  $('mesh-info').textContent = `32 × 32 VERTICES · 1/240 s × ${frame.substeps} · ${frame.iterations} ITERATIONS`;
  $('time').textContent = `${frame.time.toFixed(3)} s`; $('frame').textContent = `STEP ${frame.step}`;
  $('physics').textContent = `${frame.physics_ms.toFixed(2)} ms`;
  $('roundtrip').textContent = `${lastRoundtrip.toFixed(1)} ms`;
  $('stretch').textContent = `${(frame.p95_stretch*100).toFixed(2)} %`;
  $('penetration').textContent = `${(frame.max_penetration*1000).toPrecision(2)} mm`;
  $('contacts').textContent = frame.contacts;
  $('error').textContent = '';
}
function connect() {
  const old = socket;
  ready = false; pending = null; queue = []; nextId = 1;
  const current = socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/live/ws`);
  old?.close(); clearTimeout(connectionTimeout); $('reconnect').hidden = true;
  $('error').textContent = ''; updateControls(); setStatus('Connecting');
  connectionTimeout = setTimeout(() => { if (socket === current && !ready) current.close(); }, 10000);
  current.onmessage = event => {
    if (socket !== current) return;
    try {
      const frame = JSON.parse(event.data);
      if (frame.type === 'error') { fail(frame.message); return; }
      if (frame.type !== 'frame' || (ready ? frame.request_id !== pending?.request_id : frame.request_id !== 0)) throw new Error('Unexpected response order. Reconnect to continue.');
      lastRoundtrip = pending ? performance.now() - pending.start : 0;
      pending = null; applyFrame(frame); ready = true; clearTimeout(connectionTimeout); updateControls();
      // Commands can follow immediately; automatic stepping waits for rAF.
      if (queue.length) dispatch(performance.now());
    } catch (error) { fail(error.message); current.close(); }
  };
  current.onclose = () => {
    if (socket !== current) return;
    clearTimeout(connectionTimeout); ready = false;
    fail('Disconnected from the CPU server. Check the server and reconnect.');
    setStatus('Disconnected', 'error'); $('reconnect').hidden = false;
  };
  current.onerror = () => { if (socket === current) $('error').textContent = 'Cannot connect to the CPU server. Start it with npm run live.'; };
}
function optionsChanged() {
  $('wind-value').textContent = `${Math.round(Number($('wind').value)*100)}%`;
  updateControls();
  enqueue({type:'set_options',options:{auto_motion:$('auto-motion').checked,sphere_x:Number($('sphere-x').value),sphere_z:Number($('sphere-z').value),wind:Number($('wind').value)}});
}
for (const id of ['wind','sphere-x','sphere-z']) $(id).oninput = optionsChanged;
$('auto-motion').onchange = optionsChanged;
$('scene').onchange = () => enqueue({type:'reset',scene:$('scene').value});
$('reset').onclick = () => enqueue({type:'reset',scene:$('scene').value});
$('release').onclick = () => enqueue({type:'release'});
$('play').onclick = () => { running = !running; nextRequestAt = performance.now(); updateControls(); };
$('step').onclick = () => enqueue({type:'step'});
$('camera').onclick = resetCamera;
$('reconnect').onclick = connect;
$('wireframe').onchange = () => { material.wireframe = $('wireframe').checked; };
$('pins').onchange = () => { pins.visible = $('pins').checked; };
document.addEventListener('visibilitychange',() => { nextRequestAt = performance.now(); updateControls(); });
window.addEventListener('pagehide',() => socket?.close());
new ResizeObserver(() => {
  const {clientWidth:w,clientHeight:h} = $('viewport'); renderer.setSize(w,h);
  camera.aspect = w/h; camera.updateProjectionMatrix();
}).observe($('viewport'));
renderer.setAnimationLoop(now => {
  dispatch(now); controls.update(); renderer.render(scene,camera); renderCount++;
  if (now - metricStart >= 1000 && source) {
    $('fps').textContent = `${((renderCount-metricRenders)*1000/(now-metricStart)).toFixed(0)} fps`;
    $('speed').textContent = `${((source.time-metricTime)*1000/(now-metricStart)).toFixed(2)}×`;
    metricStart = now; metricTime = source.time; metricRenders = renderCount;
  }
});
// Read-only inspection of received data and actual render buffers for tests.
window.__clothLive = Object.freeze({snapshot:() => !source ? null : ({
  requestId:source.request_id,step:source.step,time:source.time,scene:source.scene,
  positions:Array.from(cloth.geometry.getAttribute('position').array),
  normals:Array.from(cloth.geometry.getAttribute('normal').array),
  bounds:cloth.geometry.boundingBox.min.toArray().concat(cloth.geometry.boundingBox.max.toArray()),
  sphere:sphere.position.toArray(),sourcePositions:source.positions.slice(),
  pins:source.pins.slice(),pinPositions:Array.from(pinAttribute.array.slice(0,pins.geometry.drawRange.count*3)),options:{...source.options},camera:camera.position.toArray(),
  connected:ready,paused:!running,pending:!!pending,queued:queue.length,renderCount,
  wireframe:material.wireframe,
})});
connect();
