// SPDX-License-Identifier: Apache-2.0
// Requires Playwright; CHROMIUM_PATH optionally selects system Chromium.
const {chromium}=require('playwright');
const fs=require('fs');const assert=require('assert');
(async()=>{const browser=await chromium.launch({...(process.env.CHROMIUM_PATH?{executablePath:process.env.CHROMIUM_PATH}:{}),headless:true,args:['--no-sandbox']});const page=await browser.newPage({viewport:{width:390,height:844}});const errors=[];page.on('pageerror',e=>errors.push(e.message));
let config={revision:1,shuntOhm:.001,shuntConfirmed:0,energyJ:0,maxCurrentA:30,minVoltageV:9,motorMs:25000,flightMs:180000,escOffUs:1000,escRunUs:1300,servoEnabled:1,idleUs:[1500,1500,1500],motorUs:[1500,1500,1500],glideUs:[1500,1500,1500],dtUs:[2000,1500,1500]};
let pending=false,dt=false,posts=0;
await page.route('http://f1q.test/**',async route=>{const r=route.request(),p=new URL(r.url()).pathname;let body;
if(p==='/'){await route.fulfill({contentType:'text/html',body:fs.readFileSync(require('path').join(__dirname,'../ground/main.cpp'),'utf8').split('R"HTML(')[1].split(')HTML"')[0]});return;}
if(p==='/api/config'){if(r.method()==='POST'){config={...r.postDataJSON(),revision:2};pending=true;posts++;body={revision:2}}else body=config;}
else if(p==='/api/dt'){dt=true;body={sending:true}}
else body={radioReady:true,seen:true,link:true,ageMs:50,pending,desiredRevision:config.revision,appliedRevision:pending?1:config.revision,state:dt?4:0,reason:dt?3:0,sensorOk:true,voltageV:12.1,currentA:.2,energyJ:2,flightMs:0,gpsValid:true,fixAgeMs:100,latitude:55.75,longitude:37.62,satellites:10};
await route.fulfill({contentType:'application/json',body:JSON.stringify(body)});});
await page.goto('http://f1q.test/');await page.waitForFunction(()=>document.getElementById('motorMs').value==='25000');
await page.fill('#motorMs','30000');await page.check('#confirmed');await page.click('button[type=submit]');await page.waitForTimeout(600);assert.equal(posts,1);assert.equal(config.motorMs,30000);assert.equal(config.shuntConfirmed,1);
assert((await page.locator('#result').textContent()).includes('ревизии 2'));
pending=false;await page.waitForTimeout(600);assert((await page.locator('#result').textContent()).includes('подтверждена ревизия 2'));
await page.click('#dt');await page.waitForTimeout(600);assert((await page.locator('#dtResult').textContent()).includes('Борт подтвердил'));
assert((await page.locator('#map').getAttribute('href')).includes('55.75'));
assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);assert.deepEqual(errors,[]);
await browser.close();console.log('Browser checks passed: mobile layout, full settings submission, board ACK, DT ACK, GPS link, no JS errors');})().catch(e=>{console.error(e);process.exit(1)});
