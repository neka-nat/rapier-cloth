import { spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import path from 'node:path';

const viewer = fileURLToPath(new URL('..', import.meta.url));
const root = path.resolve(viewer, '../..');
const serverPort = Number(process.env.CLOTH_SERVER_PORT ?? 9100);
const viewerPort = Number(process.env.CLOTH_VIEWER_PORT ?? 5173);
for (const port of [serverPort,viewerPort]) if (!Number.isInteger(port) || port < 1 || port > 65535) throw new Error('Invalid port');
const url = `http://127.0.0.1:${viewerPort}`;
const children = new Set();
let stopping = false;
function stop(code = 0) {
  if (stopping) return;
  stopping = true;
  for (const child of children) child.kill('SIGTERM');
  process.exitCode = code;
}
function start(command,args,options = {}) {
  const child = spawn(command,args,{cwd:root,stdio:'inherit',...options}); children.add(child);
  child.on('error',error => { console.error(error.message); stop(1); });
  child.on('exit',code => { children.delete(child); if (!stopping) stop(code || 1); });
  return child;
}
process.on('SIGINT',() => stop()); process.on('SIGTERM',() => stop());
try {
  // Build first and start the executable directly so Ctrl+C owns both the
  // server and Vite processes, rather than orphaning a cargo child process.
  await new Promise((resolve,reject) => {
    const build = spawn('cargo',['build','--locked','--release','--manifest-path','demos/live-server/Cargo.toml','--target-dir',path.join(root,'demos/live-server/target')],{cwd:root,stdio:'inherit'});
    children.add(build); build.on('error',reject);
    build.on('exit',code => { children.delete(build); code === 0 && !stopping ? resolve() : reject(new Error('Rust build stopped')); });
  });
  if (!stopping) {
    const binary = path.join(root,'demos/live-server/target/release',process.platform === 'win32' ? 'rapier-cloth-live.exe' : 'rapier-cloth-live');
    start(binary,['--port',String(serverPort),'--origin',url]);
    let healthy = false;
    for (let attempt = 0; attempt < 100 && !stopping; attempt++) {
      await new Promise(resolve => setTimeout(resolve,100));
      try { const res = await fetch(`http://127.0.0.1:${serverPort}/health`,{signal:AbortSignal.timeout(500)}); healthy = res.ok && (await res.json()).name === 'rapier-cloth-live'; } catch { /* wait for bind */ }
      if (healthy) break;
    }
    if (!healthy || stopping) throw new Error('CPU server did not start');
    start(process.execPath,[path.join(viewer,'node_modules/vite/bin/vite.js'),'--host','127.0.0.1','--port',String(viewerPort),'--strictPort'],{cwd:viewer,env:{...process.env,LIVE_SERVER_URL:`http://127.0.0.1:${serverPort}`}});
    console.log(`\nLive cloth: ${url}/live.html\nCtrl+C stops both servers.\n`);
  }
} catch (error) { if (!stopping) console.error(error.message); stop(1); }
