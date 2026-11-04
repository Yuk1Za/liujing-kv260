// Automated browser evidence, not evidence of physical monitor scanout.
const fs=require('fs'),path=require('path');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'C:/Users/oyc/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const out=path.resolve(process.argv[2]||''),seconds=Number(process.argv[3]||120);
if(!process.argv[2]||!Number.isFinite(seconds)||seconds<10||seconds>660)throw Error('record.cjs OUTPUT [10..660 seconds]');
fs.mkdirSync(out,{recursive:true});
const errors=[];
(async()=>{
 const browser=await chromium.launch({headless:true,executablePath:process.env.EDGE_PATH||'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe'});
 let video,started;
 try{
  const context=await browser.newContext({viewport:{width:1440,height:900},recordVideo:{dir:out,size:{width:1440,height:900}}});
  const page=await context.newPage();page.on('pageerror',e=>errors.push(String(e)));video=page.video();
  await page.goto('http://127.0.0.1:8097/',{waitUntil:'domcontentloaded'});started=new Date().toISOString();
  await page.waitForTimeout(Math.min(20,seconds)*1000);await page.screenshot({path:path.join(out,'warmup.png')});
  console.log('Automated recording active; shown version '+await page.locator('#version').innerText());
  await page.waitForTimeout(Math.max(0,seconds-20)*1000);await page.screenshot({path:path.join(out,'final.png')});
  const final={version:await page.locator('#version').innerText(),health:await page.locator('#health').innerText()};
  await context.close();await video.saveAs(path.join(out,'display_recording.webm'));await video.delete();
  fs.writeFileSync(path.join(out,'video_manifest.json'),JSON.stringify({scope:'automated browser; not physical display acceptance',started,seconds,final,errors},null,2));
  console.log(JSON.stringify(final));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
