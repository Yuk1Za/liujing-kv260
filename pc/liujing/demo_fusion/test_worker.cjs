const fs=require('fs'),vm=require('vm'),assert=require('assert');
let result;const context={self:{postMessage:x=>result=x},DataView,Uint8Array,Uint32Array,Float32Array,TextDecoder,Number,JSON,String,Error};
vm.runInNewContext(fs.readFileSync(__dirname+'/web/color-worker.js','utf8'),context);
function packet(change={}){const h=Buffer.from(JSON.stringify({source:'committed_fusion',session:'test',version:1,base_version:0,full:true,rgb_id:7,depth_id:9,source_stamp:100,committed_mono:101,send_mono:102,frame:'camera_init',stride:20,count:1,map_count:1,deleted_count:0,...change}));const b=Buffer.alloc(8+h.length+20);b.write('LJCD');b.writeUInt32LE(h.length,4);h.copy(b,8);const off=8+h.length;b.writeUInt32LE(0,off);b.writeFloatLE(1,off+4);b.writeFloatLE(2,off+8);b.writeFloatLE(3,off+12);b.writeUInt32LE(0x332211,off+16);return b.buffer.slice(b.byteOffset,b.byteOffset+b.length);}
context.self.onmessage({data:packet()});assert(!result.error);assert.equal(result.ids[0],0);assert.equal(result.positions[2],3);
for(const change of [{source:'unfused'},{frame:'map'},{map_count:0},{stride:24},{deleted_count:1},{full:true,map_count:2},{version:0},{base_version:1},{session:''},{count:-1},{count:1.5},{source_stamp:'bad'},{rgb_id:undefined},{depth_id:-1},{rgb_id:1.2},{send_mono:100}]){context.self.onmessage({data:packet(change)});assert(result.error);}
const bad=packet();context.self.onmessage({data:bad.slice(0,-1)});assert(result.error);console.log('PASS committed-only payload, bounds, coordinate contract and truncation');

// Highest stable ID at the expanded capacity must survive incremental decoding.
const expanded=packet({full:false,version:2,base_version:1,map_count:2000000});
const view=new DataView(expanded);view.setUint32(8+view.getUint32(4,true),1999999,true);
context.self.onmessage({data:expanded});assert(!result.error);assert.equal(result.ids[0],1999999);
context.self.onmessage({data:packet({full:false,version:2,base_version:1,map_count:2000001})});assert(result.error);
console.log('PASS 2m boundary ID and rejection above capacity');
