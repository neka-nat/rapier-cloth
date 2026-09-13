import { defineConfig } from 'vite';
import { fileURLToPath } from 'node:url';
const proxy = { '/live/ws': {target:process.env.LIVE_SERVER_URL ?? 'http://127.0.0.1:9100',ws:true} };
export default defineConfig({
  server:{proxy}, preview:{proxy},
  build:{rolldownOptions:{input:{replay:fileURLToPath(new URL('./index.html',import.meta.url)),live:fileURLToPath(new URL('./live.html',import.meta.url))}}},
});
