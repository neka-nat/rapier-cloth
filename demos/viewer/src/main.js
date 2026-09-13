import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { validateRecording } from './recording.js';
import './style.css';

const $ = id => document.getElementById(id);
const scene = new THREE.Scene(); scene.background = new THREE.Color('#192522');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFShadowMap;
$('viewport').appendChild(renderer.domElement);
const camera = new THREE.PerspectiveCamera(38, 1, 0.01, 40);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
function resetCamera() { camera.position.set(1.05, 0.95, 1.25); controls.target.set(0.2, 0.13, -0.1); controls.update(); }
resetCamera();
scene.add(new THREE.HemisphereLight('#e4fff0', '#657466', 2.4));
const light = new THREE.DirectionalLight('#fff5dd', 3.2); light.position.set(-0.8, 2.5, 1.2); light.castShadow = true;
light.shadow.mapSize.set(2048, 2048); light.shadow.camera.left = -2; light.shadow.camera.right = 2;
light.shadow.camera.top = 2; light.shadow.camera.bottom = -2; light.shadow.normalBias = 0.002;
scene.add(light);
const grid = new THREE.GridHelper(4, 40, '#6d8876', '#43594b'); grid.position.y = 0.001; grid.material.transparent = true; grid.material.opacity = 0.28; scene.add(grid);
const content = new THREE.Group(); scene.add(content);
let record, mesh, anchorPoints, pinPoints, bodies = new Map(), current = 0, playing = false, playOrigin = 0, startTime = 0;
function makePoints(color) { return new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ color, size: 0.012, depthTest: false })); }
function disposeContent() {
  content.traverse(object => { object.geometry?.dispose(); object.material?.dispose(); });
  content.clear(); bodies.clear();
}
function stop() { playing = false; $('play').textContent = '再生'; }
function load(data, name) {
  const valid = validateRecording(data); // Keep the previous scene if validation fails.
  stop(); disposeContent(); record = valid;
  const geometry = new THREE.BufferGeometry();
  geometry.setIndex(record.triangles.flat());
  geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(record.frames[0].positions.length * 3), 3).setUsage(THREE.DynamicDrawUsage));
  mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({color:'#a2e4ba', side:THREE.DoubleSide, roughness:0.86, metalness:0, wireframe:$('wireframe').checked}));
  mesh.castShadow = true; mesh.receiveShadow = true; content.add(mesh);
  for (const shape of record.shapes) {
    const group = new THREE.Group();
    const box = new THREE.Mesh(new THREE.BoxGeometry(...shape.half_extents.map(x => 2*x)), new THREE.MeshStandardMaterial({color:shape.color, roughness:0.9}));
    box.position.fromArray(shape.local_translation); box.castShadow = true; box.receiveShadow = true;
    group.add(box); bodies.set(shape.id, group); content.add(group);
  }
  anchorPoints = makePoints('#f5a94d'); pinPoints = makePoints('#f898b2'); content.add(anchorPoints, pinPoints);
  $('timeline').max = record.frames.length - 1; $('play').disabled = false;
  $('precision').textContent = record.precision;
  $('mesh-info').textContent = `${name} · ${record.frames[0].positions.length} VERTICES · ${record.triangles.length} TRIANGLES`;
  $('error').textContent = ''; showFrame(0);
}
function updatePoints(object, values) {
  object.geometry.setAttribute('position', new THREE.Float32BufferAttribute(values.flat(), 3));
  object.geometry.computeBoundingSphere(); object.visible = $('anchors').checked;
}
function showFrame(index) {
  if (!record) return;
  current = Math.max(0, Math.min(record.frames.length - 1, index));
  const frame = record.frames[current];
  // Original numbers remain JS doubles. Only the GPU-facing buffer is f32.
  const attribute = mesh.geometry.getAttribute('position');
  attribute.array.set(frame.positions.flat()); attribute.needsUpdate = true;
  mesh.geometry.computeVertexNormals(); mesh.geometry.computeBoundingBox(); mesh.geometry.computeBoundingSphere();
  for (const body of frame.bodies) { const group = bodies.get(body.id); group.position.fromArray(body.translation); group.quaternion.fromArray(body.rotation); }
  updatePoints(anchorPoints, frame.anchors);
  updatePoints(pinPoints, frame.pinned_particles.map(i => frame.positions[i]));
  $('timeline').value = current; $('time').textContent = `${frame.time.toFixed(3)} s`;
  $('frame').textContent = `FRAME ${current + 1} / ${record.frames.length} · STEP ${frame.step}`;
  $('phase').textContent = ({settle:'初期化',lift:'持ち上げ',transport:'運搬',release:'解放',drop:'落下・静止'})[frame.phase] ?? frame.phase;
  $('stretch').textContent = `${(frame.diagnostics.p95_stretch * 100).toFixed(3)} %`;
  $('penetration').textContent = `${(frame.diagnostics.max_penetration * 1000).toPrecision(3)} mm`;
  $('target').textContent = `${(frame.diagnostics.max_target_error * 1000).toPrecision(3)} mm`;
  $('contacts').textContent = frame.diagnostics.contacts;
}
$('play').onclick = () => {
  if (!record) return;
  if (playing) { stop(); return; }
  if (current === record.frames.length - 1) showFrame(0);
  playing = true; $('play').textContent = '停止'; startTime = performance.now(); playOrigin = record.frames[current].time;
};
$('timeline').oninput = () => { stop(); showFrame(Number($('timeline').value)); };
$('reset').onclick = () => { stop(); showFrame(0); };
$('camera').onclick = resetCamera;
$('wireframe').onchange = () => { if (mesh) mesh.material.wireframe = $('wireframe').checked; };
$('anchors').onchange = () => { if (anchorPoints) { anchorPoints.visible = pinPoints.visible = $('anchors').checked; } };
$('file').onchange = async event => {
  const file = event.target.files[0]; if (!file) return;
  try { load(JSON.parse(await file.text()), file.name); } catch (error) { $('error').textContent = error.message; }
  event.target.value = '';
};
new ResizeObserver(() => {
  const {clientWidth:width,clientHeight:height} = $('viewport'); renderer.setSize(width,height);
  camera.aspect = width / height; camera.updateProjectionMatrix();
}).observe($('viewport'));
renderer.setAnimationLoop(now => {
  if (playing && record) {
    const time = playOrigin + (now - startTime)/1000;
    let next = current;
    while (next + 1 < record.frames.length && record.frames[next + 1].time <= time) next++;
    if (next !== current) showFrame(next);
    if (current === record.frames.length - 1) stop();
  }
  controls.update(); renderer.render(scene,camera);
});
// Read-only inspection of actual render buffers for recorded-data parity tests.
window.__clothReplay = Object.freeze({ snapshot: () => !record ? null : ({
  frameIndex:current, positions:Array.from(mesh.geometry.getAttribute('position').array),
  normals:Array.from(mesh.geometry.getAttribute('normal').array),
  bounds:mesh.geometry.boundingBox.min.toArray().concat(mesh.geometry.boundingBox.max.toArray()),
  bodies:[...bodies].map(([id,group]) => ({id,translation:group.position.toArray(),rotation:group.quaternion.toArray(),local_translation:group.children[0].position.toArray()})),
  anchors:Array.from(anchorPoints.geometry.getAttribute('position').array),
  pins:Array.from(pinPoints.geometry.getAttribute('position').array), camera:camera.position.toArray(),
  wireframe:mesh.material.wireframe, anchorsVisible:anchorPoints.visible,
}) });
try {
  const response = await fetch(`${import.meta.env.BASE_URL}sample-f64.json`);
  if (!response.ok) throw new Error('同梱記録がありません。「記録を開く」からRustのJSON記録を選択してください。');
  load(await response.json(), 'pick-and-place / f64');
} catch (error) { $('error').textContent = error.message; }
