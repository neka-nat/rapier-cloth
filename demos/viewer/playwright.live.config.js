import { defineConfig } from '@playwright/test';
const precision = process.env.CLOTH_LIVE_PRECISION ?? 'f32';
if (!['f32','f64'].includes(precision)) throw new Error('Invalid CLOTH_LIVE_PRECISION');
export default defineConfig({
  testDir:'./tests/live',timeout:90_000,expect:{timeout:15_000},workers:1,fullyParallel:false,
  use:{baseURL:'http://127.0.0.1:4174',headless:true,viewport:{width:1440,height:1050},
    launchOptions:{...(process.env.CHROME_PATH ? {executablePath:process.env.CHROME_PATH}:{}),args:['--enable-unsafe-swiftshader']},
  },
  webServer:[
    {command:`cargo run --locked --release --manifest-path ../live-server/Cargo.toml --no-default-features --features ${precision} -- --port 9174 --origin http://127.0.0.1:4174`,url:'http://127.0.0.1:9174/health',reuseExistingServer:false,timeout:180_000},
    {command:'npm run build && npm run preview -- --port 4174 --strictPort',url:'http://127.0.0.1:4174/live.html',env:{LIVE_SERVER_URL:'http://127.0.0.1:9174'},reuseExistingServer:false},
  ],
  reporter:[['list'],['json',{outputFile:'test-results/live-results.json'}]],
});
