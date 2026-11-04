// Shared by the download action and the offline byte-for-byte verification.
export function encodeMapPLY(positions, packedColors, count, version) {
 if(!Number.isSafeInteger(count)||count<0||positions.length<count*3||packedColors.length<count)throw Error('invalid map buffers');
 const header=new TextEncoder().encode('ply\nformat binary_little_endian 1.0\ncomment LiuJing committed RGB-D map version '+version+'\nelement vertex '+count+'\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n');
 const bytes=new Uint8Array(header.length+count*15);bytes.set(header);
 const view=new DataView(bytes.buffer,header.length);
 for(let i=0;i<count;i++){
  for(let k=0;k<3;k++)view.setFloat32(i*15+k*4,positions[i*3+k],true);
  view.setUint8(i*15+12,packedColors[i]&255);
  view.setUint8(i*15+13,(packedColors[i]>>8)&255);
  view.setUint8(i*15+14,(packedColors[i]>>16)&255);
 }
 return bytes;
}
