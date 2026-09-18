import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { validateFrame } from './live-protocol.js';
import './style.css';
import './live.css';

const $ = id => document.getElementById(id);
const scene = new THREE.Scene(); scene.background = new THREE.Color('#192522');
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); renderer.shadowMap.enabled = true;
renderer.shadowMap.autoUpdate = false;
let renderDirty = true;
$('viewport').appendChild(renderer.domElement);
const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 30);
const controls = new OrbitControls(camera, renderer.domElement); controls.enableDamping = true;
controls.addEventListener('change', () => { renderDirty = true; });
function resetCamera(kind = $('scene').value) {
  if (['fold_towel','implicit_towel'].includes(kind)) { camera.position.set(0.85, 0.85, 1.05); controls.target.set(0, 0.12, 0); }
  else { camera.position.set(1.65, 1.65, 2.05); controls.target.set(0, 0.45, 0); }
  controls.update();
}
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
const grippers = ['#f97316', '#0ea5e9'].map(color => {
  const group = new THREE.Group();
  const box = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshStandardMaterial({color, roughness:0.6}));
  box.castShadow = box.receiveShadow = true; group.add(box); group.visible = false; scene.add(group); return group;
});
const material = new THREE.MeshStandardMaterial({color:'#a2e4ba',side:THREE.DoubleSide,roughness:0.85});
let cloth = null;
const pins = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({color:'#ff98bd',size:0.02,depthTest:false}));
const pinAttribute = new THREE.BufferAttribute(new Float32Array(1024*3),3).setUsage(THREE.DynamicDrawUsage);
pins.geometry.setAttribute('position',pinAttribute); pins.geometry.setDrawRange(0,0); pins.frustumCulled = false;
scene.add(pins);
const anchors = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({color:'#f5a94d', size:0.012, depthTest:false}));
const anchorAttribute = new THREE.BufferAttribute(new Float32Array(1024*3), 3).setUsage(THREE.DynamicDrawUsage);
anchors.geometry.setAttribute('position', anchorAttribute); anchors.geometry.setDrawRange(0, 0); anchors.frustumCulled = false; scene.add(anchors);
let socket, ready = false, pending = null, queue = [], nextId = 1, source = null;
let running = new URLSearchParams(location.search).get('paused') !== '1';
let nextRequestAt = 0, lastRoundtrip = 0, renderCount = 0;
let metricStart = performance.now(), metricTime = 0, metricRenders = 0;
let connectionTimeout;

function setStatus(text, state = '') { $('status').textContent = text; $('status').dataset.state = state; }
const taskData = frame => frame?.implicit ?? frame?.folding;
function updateControls() {
  const implicit = $('scene').value === 'implicit_towel';
  const folding = implicit || $('scene').value === 'fold_towel', task = taskData(source);
  const stopped = !!task?.stopped || !!task?.completed;
  for (const id of ['reset','scene']) $(id).disabled = !ready;
  $('play').disabled = !ready || stopped;
  $('step').disabled = !ready || running || stopped;
  $('free-controls').hidden = folding; $('fold-controls').hidden = !folding;
  for (const id of ['wind','auto-motion']) $(id).disabled = !ready || folding;
  $('release').disabled = !ready || folding || !source?.pins.length;
  for (const id of ['sphere-x','sphere-z']) $(id).disabled = !ready || folding || $('auto-motion').checked;
  for (const [i, id] of ['release-left','release-right'].entries()) $(id).disabled = !ready || !folding || (implicit && stopped) || !task?.grippers[i].holding;
  $('inspect').disabled = !ready || !folding;
  $('inspect').hidden = implicit;
  $('pose-controls').hidden = !implicit;
  for (const button of document.querySelectorAll('.pose-buttons button')) button.disabled = !ready || stopped || !!pending;
  $('grasp').disabled = !ready || !implicit || stopped || !!task?.grippers[Number($('gripper').value)].holding;
  $('scene').querySelector('[value="implicit_towel"]').disabled = !source?.implicit_available;
  $('play').textContent = running ? 'Pause' : 'Resume';
  if (ready) setStatus(task?.completed ? 'Completed' : stopped ? 'Stopped · error' : document.hidden ? 'Tab hidden · idle' : running ? 'Running' : pending ? 'Pausing' : 'Paused', task?.stopped ? 'error' : '');
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
  const key = item => ['release_gripper','set_gripper_pose','grasp_gripper'].includes(item.type) ? `${item.type}:${item.gripper}` : item.type;
  queue = queue.filter(item => key(item) !== key(command));
  queue.push(command);
  dispatch(performance.now());
}
function dispatch(now) {
  if (!ready || pending || socket?.readyState !== WebSocket.OPEN) return;
  let command = queue.shift();
  if (!command && running && !document.hidden && now + 0.2 >= nextRequestAt) command = {type:'step'};
  if (!command) return;
  if (command.type === 'set_gripper_pose') command.at_step = source.step + 1;
  const request_id = nextId++;
  pending = {request_id,command,start:now};
  if (command.type === 'step') nextRequestAt = Math.max(nextRequestAt + 1000 * (source?.h ?? 1/240) * (source?.substeps ?? 4), now + 1);
  socket.send(JSON.stringify({request_id,command}));
  updateControls();
}
function applyFrame(frame) {
  validateFrame(frame);
  renderDirty = true; renderer.shadowMap.needsUpdate = true;
  const task = taskData(frame), implicit = !!frame.implicit;
  floor.position.y = implicit ? task.floor_height : 0;
  grid.visible = !implicit;
  if (frame.triangles) {
    if (cloth) { cloth.geometry.dispose(); scene.remove(cloth); }
    const geometry = new THREE.BufferGeometry(); geometry.setIndex(frame.triangles);
    geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(3072),3).setUsage(THREE.DynamicDrawUsage));
    cloth = new THREE.Mesh(geometry,material); cloth.castShadow = cloth.receiveShadow = true; scene.add(cloth);
    $('scene').value = frame.scene;
    if (source?.scene !== frame.scene) resetCamera(frame.scene);
    metricStart = performance.now(); metricTime = frame.time; metricRenders = renderCount;
  }
  if (!cloth) throw new Error('Initial mesh is missing. Reconnect to continue.');
  const attribute = cloth.geometry.getAttribute('position'); attribute.array.set(frame.positions); attribute.needsUpdate = true;
  cloth.geometry.computeVertexNormals(); cloth.geometry.computeBoundingBox(); cloth.geometry.computeBoundingSphere();
  sphere.position.fromArray(frame.sphere); sphere.scale.setScalar(frame.sphere_radius);
  sphere.visible = !task;
  grippers.forEach((group, i) => {
    group.visible = !!task;
    if (!task) return;
    const g = task.grippers[i], box = group.children[0];
    group.position.fromArray(g.translation); group.quaternion.fromArray(g.rotation);
    box.position.fromArray(g.local_translation); box.quaternion.fromArray(g.local_rotation);
    box.scale.fromArray(g.half_extents.map(x => x*2));
  });
  frame.pins.forEach((i,j) => pinAttribute.array.set(frame.positions.slice(3*i,3*i+3),j*3));
  pinAttribute.needsUpdate = true; pins.geometry.setDrawRange(0,frame.pins.length); pins.visible = $('pins').checked;
  const graspPoints = task?.grasps.flatMap(g => g.points) ?? [];
  graspPoints.forEach((p, i) => anchorAttribute.array.set(p.anchor, i*3));
  anchorAttribute.needsUpdate = true; anchors.geometry.setDrawRange(0, graspPoints.length); anchors.visible = $('pins').checked;
  // A reset may complete while a newer slider value is queued. Preserve that
  // pending intent instead of replacing the controls with the reset defaults.
  const desired = queue.find(command => command.type === 'set_options')?.options ?? frame.options;
  $('auto-motion').checked = desired.auto_motion;
  $('wind').value = desired.wind; $('wind-value').textContent = `${Math.round(desired.wind*100)}%`;
  $('sphere-x').value = desired.sphere_x; $('sphere-z').value = desired.sphere_z;
  source = frame;
  $('precision').textContent = `RUST CPU / ${frame.precision}`;
  $('mesh-info').textContent = `32 × 32 VERTICES · ${implicit ? 'IMPLICIT · 0.1 s × 1' : `XPBD · 1/240 s × ${frame.substeps}`} · ${frame.iterations} ITERATIONS`;
  $('time').textContent = `${frame.time.toFixed(3)} s`; $('frame').textContent = `STEP ${frame.step}`;
  $('physics').textContent = `${frame.physics_ms.toFixed(2)} ms`;
  $('roundtrip').textContent = `${lastRoundtrip.toFixed(1)} ms`;
  $('stretch').textContent = `${(frame.p95_stretch*100).toFixed(2)} %`;
  $('penetration').textContent = `${(frame.max_penetration*1000).toPrecision(2)} mm`;
  $('contacts').textContent = frame.contacts;
  $('target').textContent = `${(frame.max_target_error*1000).toPrecision(2)} mm`;
  $('task-phase').hidden = !task; $('task-phase').textContent = task?.phase ?? '';
  $('task-override').hidden = implicit || !task?.manual_release;
  $('grasp-state').textContent = task ? task.grippers.map((g, i) => `${i === 0 ? 'Left' : 'Right'}: ${g.holding ? 'holding' : 'free'}`).join(' · ') : '';
  const inspection = task?.inspection;
  $('fold-description').textContent = implicit
    ? 'Live Rust implicit physics. One 0.1 s solve per step. Pose commands are ideal grasps; some changed trajectories can fail and stop.'
    : 'A scripted two-gripper fold. Complete folding and real-time performance remain unqualified.';
  $('fold-measurements').hidden = implicit;
  $('fold-measurements').nextElementSibling.hidden = implicit;
  for (const [id,label] of Object.entries(implicit
    ? {stretch:'Max edge extension',penetration:'RMS speed',contacts:'Held vertices',target:'Max speed'}
    : {stretch:'Stretch · p95',penetration:'Max penetration',contacts:'Contacts',target:'Attachment error'})) $(`${id}-label`).textContent = label;
  if (implicit) {
    $('stretch').textContent = `${(task.sample.max_edge_extension*100).toFixed(3)} %`;
    $('penetration').textContent = `${(task.sample.rms_speed*1000).toFixed(3)} mm/s`;
    $('contacts').textContent = task.sample.held_vertices;
    $('target').textContent = `${(task.sample.max_speed*1000).toFixed(3)} mm/s`;
  }
  $('live-note').textContent = implicit
    ? 'Computed on demand, not recording playback. Camera controls remain responsive during a solve; unchanged frames are not redrawn. There is one outstanding request and no catch-up backlog. A rate below 1× means simulation progresses slower than wall time.'
    : 'Four 1/240 s substeps per requested frame. A simulation rate below 1× means physics is running slower than real time. Hidden tabs pause automatic stepping.';
  $('fold-measurements').textContent = inspection
    ? `Step ${inspection.step} · Corner error ${(inspection.metrics.max_corner_error*1000).toFixed(1)} mm · Overlap ${(inspection.metrics.overlap_ratio*100).toFixed(1)}% · Footprint error ${(inspection.metrics.relative_area_error*100).toFixed(1)}%`
    : 'Pause and measure to inspect the current shape.';
  $('error').textContent = task?.stopped ?? '';
  if (task?.stopped || task?.completed) {
    running = false;
    queue = queue.filter(command => command.type !== 'step');
  }
}
function connect() {
  const old = socket;
  ready = false; pending = null; queue = []; nextId = 1;
  let selectInitialScene = new URLSearchParams(location.search).get('scene') === 'implicit_towel';
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
      if (selectInitialScene) {
        selectInitialScene = false;
        enqueue({type:'reset',scene:'implicit_towel'});
      }
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
$('release-left').onclick = () => enqueue({type:'release_gripper', gripper:0});
$('release-right').onclick = () => enqueue({type:'release_gripper', gripper:1});
$('gripper').onchange = updateControls;
for (const button of document.querySelectorAll('.pose-buttons button')) button.onclick = () => {
  if (!source?.implicit) return;
  const gripper = Number($('gripper').value);
  const pose = structuredClone(source.implicit.desired[gripper]);
  if (button.dataset.axis !== undefined) pose.translation[Number(button.dataset.axis)] += Number(button.dataset.delta);
  else {
    const increment = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0), Number(button.dataset.roll)*Math.PI/180);
    pose.rotation = increment.multiply(new THREE.Quaternion().fromArray(pose.rotation)).toArray();
  }
  enqueue({type:'set_gripper_pose',gripper,...pose});
};
$('grasp').onclick = () => enqueue({type:'grasp_gripper',gripper:Number($('gripper').value)});
$('inspect').onclick = () => { running = false; queue = queue.filter(command => command.type !== 'step'); enqueue({type:'inspect'}); updateControls(); };
$('play').onclick = () => { running = !running; nextRequestAt = performance.now(); updateControls(); };
$('step').onclick = () => enqueue({type:'step'});
$('camera').onclick = () => resetCamera();
$('reconnect').onclick = connect;
$('wireframe').onchange = () => { material.wireframe = $('wireframe').checked; renderDirty = true; renderer.shadowMap.needsUpdate = true; };
$('pins').onchange = () => { pins.visible = anchors.visible = $('pins').checked; renderDirty = true; };
document.addEventListener('visibilitychange',() => { nextRequestAt = performance.now(); updateControls(); });
window.addEventListener('pagehide',() => socket?.close());
new ResizeObserver(() => {
  const {clientWidth:w,clientHeight:h} = $('viewport'); renderer.setSize(w,h);
  camera.aspect = w/h; camera.updateProjectionMatrix(); renderDirty = true;
}).observe($('viewport'));
renderer.setAnimationLoop(now => {
  dispatch(now); controls.update();
  // Physics, camera input and view options invalidate the picture. Keeping rAF
  // lightweight lets the CPU solver run while an unchanged frame is displayed.
  if (renderDirty) { renderer.render(scene,camera); renderCount++; renderDirty = false; }
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
  connected:ready,paused:!running,pending:!!pending,queued:queue.length,renderCount,roundtripMs:lastRoundtrip,
  wireframe:material.wireframe,
  advancedSubsteps:source.advanced_substeps,folding:source.folding ? structuredClone(source.folding) : null,
  implicit:source.implicit ? structuredClone(source.implicit) : null,
  grippers:grippers.map((group, id) => ({id,visible:group.visible,translation:group.position.toArray(),rotation:group.quaternion.toArray(),local_translation:group.children[0].position.toArray(),local_rotation:group.children[0].quaternion.toArray(),half_extents:group.children[0].scale.toArray().map(x => x/2)})),
  anchors:Array.from(anchorAttribute.array.slice(0, anchors.geometry.drawRange.count*3)),anchorsVisible:anchors.visible,sphereVisible:sphere.visible,
})});
connect();
