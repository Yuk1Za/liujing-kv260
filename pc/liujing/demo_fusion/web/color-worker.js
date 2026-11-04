// Decode only a complete committed map version. No raw observations enter here.
const linear=Float32Array.from({length:256},(_,i)=>{const c=i/255;return c<=.04045?c/12.92:((c+.055)/1.055)**2.4;});
self.onmessage=({data})=>{
 try {
  const v=new DataView(data),magic=String.fromCharCode(...new Uint8Array(data,0,4));if(magic!=='LJCD')throw Error('magic');
  const length=v.getUint32(4,true);if(length>65536||8+length>data.byteLength)throw Error('header');
  const m=JSON.parse(new TextDecoder().decode(new Uint8Array(data,8,length))),n=Number(m.count),total=Number(m.map_count),off=8+length;
  if(m.source!=='committed_fusion'||m.frame!=='camera_init'||Number(m.stride)!==20||!Number.isSafeInteger(n)||n<0||!Number.isSafeInteger(total)||total<0||n>total||n>2000000||total>2000000||Number(m.deleted_count)!==0||off+n*20!==data.byteLength)throw Error('contract');
  const full=m.full===true||m.full==='true';const version=Number(m.version),base=Number(m.base_version);if(!Number.isSafeInteger(version)||version<1||!Number.isSafeInteger(base)||base<0||base>=version||typeof m.session!=='string'||!m.session||!Number.isFinite(Number(m.source_stamp))||!Number.isFinite(Number(m.committed_mono))||!Number.isFinite(Number(m.send_mono))||(full&&base!==0))throw Error('version metadata');if(full&&n!==total)throw Error('incomplete snapshot');
  if(!['rgb_id','depth_id'].every(key=>Number.isSafeInteger(Number(m[key]))&&Number(m[key])>=0)||Number(m.send_mono)<Number(m.committed_mono))throw Error('source metadata');
  const ids=new Uint32Array(n),positions=new Float32Array(n*3),colors=new Float32Array(n*3),packedColors=new Uint32Array(n);
  for(let i=0;i<n;i++){const k=off+i*20,id=v.getUint32(k,true);if(id>=total||(i&&id<=ids[i-1]))throw Error('id/order');ids[i]=id;for(let j=0;j<3;j++){const x=v.getFloat32(k+4+j*4,true);if(!Number.isFinite(x))throw Error('coordinate');positions[i*3+j]=x;}const rgb=v.getUint32(k+16,true);packedColors[i]=rgb;colors[i*3]=linear[rgb&255];colors[i*3+1]=linear[(rgb>>8)&255];colors[i*3+2]=linear[(rgb>>16)&255];}
  self.postMessage({meta:m,ids,positions,colors,packedColors},[ids.buffer,positions.buffer,colors.buffer,packedColors.buffer]);
 }catch(e){self.postMessage({error:String(e)});}
};
