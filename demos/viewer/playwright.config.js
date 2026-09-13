import { defineConfig } from '@playwright/test';
export default defineConfig({
  testDir:'./tests', testMatch:'replay.spec.js', timeout:30_000, fullyParallel:false,
  use:{baseURL:'http://127.0.0.1:4173',headless:true,viewport:{width:1280,height:850},
    launchOptions:{...(process.env.CHROME_PATH ? {executablePath:process.env.CHROME_PATH}:{}),args:['--enable-unsafe-swiftshader']},
  },
  webServer:{command:'npm run dev -- --port 4173 --strictPort',url:'http://127.0.0.1:4173',reuseExistingServer:!process.env.CI},
  reporter:[['list'],['json',{outputFile:'test-results/results.json'}]],
});
