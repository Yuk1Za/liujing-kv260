// Run with Node. Produces a real PLY using the browser's exact encoder.
// No device access or browser automation is performed.
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import assert from 'node:assert/strict';
import {fileURLToPath} from 'node:url';
const root=path.dirname(fileURLToPath(import.meta.url));
const out=path.resolve(root,'../evidence/viewer_redesign_20261028');
const load=async name=>import('data:text/javascript;base64,'+fs.readFileSync(path.join(root,'web',name)).toString('base64'));
const {encodeMapPLY}=await load('map-export.js');
const {backendLabel,plScope,mapTelemetry}=await load('view-state.js');
let decoded;
const scope={TextDecoder,Float32Array,Uint32Array,Uint8Array,DataView,self:{postMessage:value=>decoded=value}};
vm.createContext(scope);vm.runInContext(fs.readFileSync(path.join(root,'web/color-worker.js'),'utf8'),scope);
const packet=fs.readFileSync(path.join(out,'committed_snapshot.ljcd'));
scope.self.onmessage({data:packet.buffer.slice(packet.byteOffset,packet.byteOffset+packet.byteLength)});
assert.ok(!decoded.error,decoded.error);
assert.equal(decoded.meta.map_count,54876);assert.equal(decoded.meta.version,120);
const output=path.join(out,'liujing-map-v120.ply');
fs.writeFileSync(output,encodeMapPLY(decoded.positions,decoded.packedColors,decoded.meta.map_count,decoded.meta.version));
assert.equal(backendLabel('pl_variable_association_v1'),'PL · 可变长邻域查找与表面匹配');
assert.match(plScope,/PS 计算：地图更新与版本提交/);
const events=Array.from({length:21},(_,i)=>({at:1000+i*450,version:i+1}));
const base={state:{session:'fixture',backend:'pl_variable_association_v1',fusion_commit_hz_10s:3.4},session:'fixture',now:10000,lastStatusAt:9900,lastMapAt:9900,events,lastReport:9000,snapshot:false};
const cases={live:mapTelemetry(base),snapshot:mapTelemetry({...base,snapshot:true}),unclassified:mapTelemetry({...base,state:{},lastStatusAt:0}),disconnected:mapTelemetry({...base,lastStatusAt:7000}),stopped:mapTelemetry({...base,lastMapAt:8000}),invalidRate:mapTelemetry({...base,state:{...base.state,fusion_commit_hz_10s:null}}),expiredWindow:mapTelemetry({...base,now:21000})};
assert.equal(cases.live.fusion_commit_hz_10s,3.4);assert.equal(cases.live.display_hz_10s,2.1);assert.equal(cases.live.new_versions.length,3);
for(const kind of ['snapshot','unclassified']){assert.equal(cases[kind].fusion_commit_hz_10s,null);assert.equal(cases[kind].display_hz_10s,0);assert.equal(cases[kind].new_versions.length,0);assert.equal(cases[kind].live_source_age,false);}
for(const kind of ['disconnected','stopped','invalidRate','expiredWindow'])assert.equal(cases[kind].fusion_commit_hz_10s,null);
assert.equal(cases.expiredWindow.display_hz_10s,0);
fs.writeFileSync(path.join(out,'export_and_status_validation.json'),JSON.stringify({scope:'Saved-map export and pure status logic; fixtures are unit cases, not live measurements',generated_by:'Browser encodeMapPLY after actual color-worker decode',output,vertices:decoded.meta.map_count,version:decoded.meta.version,backend_label:backendLabel('pl_variable_association_v1'),backend_scope:plScope,cases,passed:true},null,2));
console.log(JSON.stringify({output,vertices:decoded.meta.map_count,status_cases:Object.keys(cases).length,passed:true}));
