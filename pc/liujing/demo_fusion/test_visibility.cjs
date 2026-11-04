// Synthetic protocol regression only. Never a sensor or performance result.
const fs=require('fs'),path=require('path'),http=require('http'),assert=require('assert');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'C:/Users/oyc/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
let latest=1,held=null;const shown=[],errors=[];
function packet(version,base){
 const h=Buffer.from(JSON.stringify({source:'committed_fusion',frame:'camera_init',stride:20,count:1,map_count:1,deleted_count:0,session:'synthetic-visibility-test',version,base_version:base,full:base===0,rgb_id:version,depth_id:version,source_stamp:100,committed_mono:101,send_mono:102,source_age_s:.1}));
 const b=Buffer.alloc(8+h.length+20);b.write('LJCD');b.writeUInt32LE(h.length,4);h.copy(b,8);b.writeUInt32LE(0,8+h.length);b.writeFloatLE(version,12+h.length);b.writeFloatLE(1,16+h.length);b.writeFloatLE(2,20+h.length);b.writeUInt32LE(0x332211,24+h.length);return b;
}
const server=http.createServer((req,res)=>{
 const url=new URL(req.url,'http://fixture');
 if(url.pathname==='/api/color'){
  const base=Number(url.searchParams.get('after'));
  if(base===1&&latest===1){held=res;return;}
  if(base>=latest){setTimeout(()=>{res.writeHead(204);res.end();},100);return;}
  res.writeHead(200,{'Content-Type':'application/octet-stream'});res.end(packet(latest,base));return;
 }
 if(url.pathname==='/api/status'){res.setHeader('Content-Type','application/json');res.end(JSON.stringify({session:'synthetic-visibility-test',mono:102,backend:'synthetic_test',fusion_commit_hz_10s:0}));return;}
 if(url.pathname==='/client-metrics'){let b='';req.on('data',x=>b+=x);req.on('end',()=>{shown.push(...JSON.parse(b).new_versions.map(x=>x.version));res.end('{}');});return;}
 const root=url.pathname.startsWith('/vendor/')?path.resolve(__dirname,'../demo/web'):path.resolve(__dirname,'web');
 const file=path.resolve(root,'.'+(url.pathname==='/'?'/index.html':url.pathname));
 if(!file.startsWith(root+path.sep)||!fs.existsSync(file)||!fs.statSync(file).isFile()){res.writeHead(404);res.end();return;}
 res.setHeader('Content-Type',file.endsWith('.js')?'text/javascript':file.endsWith('.html')?'text/html':'text/css');res.end(fs.readFileSync(file));
});
(async()=>{
 await new Promise(r=>server.listen(0,'127.0.0.1',r));
 const browser=await chromium.launch({headless:true,executablePath:process.env.EDGE_BINARY||'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe'});
 try{
  const page=await browser.newPage();page.on('pageerror',e=>errors.push(e.message));
  await page.addInitScript(()=>{window.testHidden=false;Object.defineProperty(document,'hidden',{get:()=>window.testHidden});});
  await page.goto(`http://127.0.0.1:${server.address().port}`);
  await page.waitForFunction(()=>document.getElementById('version').textContent==='1');
  assert(held,'next version request is outstanding');
  await page.evaluate(()=>{window.testHidden=true;document.dispatchEvent(new Event('visibilitychange'));});
  held.writeHead(200,{'Content-Type':'application/octet-stream'});held.end(packet(2,1));held=null;
  await page.waitForTimeout(200);latest=3;
  await page.evaluate(()=>{window.testHidden=false;document.dispatchEvent(new Event('visibilitychange'));});
  await page.waitForFunction(()=>document.getElementById('version').textContent==='3');
  assert(shown.includes(1)&&shown.includes(3));assert(!shown.includes(2),'hidden pending version must not be displayed on return');assert.deepEqual(errors,[]);
  console.log('PASS synthetic browser visibility: committed 1 -> 3, stale pending 2 discarded, no geometry interpolation');
 }finally{await browser.close();server.closeAllConnections();server.close();}
})().catch(e=>{console.error(e);server.closeAllConnections();server.close();process.exitCode=1;});
