#include "raster.hpp"
#include <hls_stream.h>
#include <ap_int.h>
#include <cmath>
#include <cstring>
// Experimental qrcp_v1 color contract (CPU mode 5), not stream_v4 compatible.
// Radius squared must be a positive normal finite FP32 value; PS validates it.
// FP32 geometry, non-contracted operations. Q18 weights, Q12 colors and
// uint64 accumulation: <=1e6*32*2^18*255*2^12 < 2^63, no saturation.
// No DEPENDENCE inter false: memory recurrence is scheduled by HLS.
#ifndef LIUJING_LANES
#define LIUJING_LANES 2
#endif
constexpr int LANES=LIUJING_LANES,CACHE=512,PIXELS=256/LANES;
static_assert(LANES==1||LANES==2||LANES==4,"supported pixel banks: 1, 2, 4");
struct Desc {Surfel s;uint32_t id;float numerator,radius2,inverse_radius2;uint32_t r,g,b;bool end;};
struct Request {Desc d;float rx,ry;ap_uint<8> pixel;bool end;};
struct Fragment {float z;uint32_t weight,r,g,b,id;ap_uint<8> pixel;bool valid,end;};
static uint32_t bits(float x){uint32_t u;std::memcpy(&u,&x,4);return u;}
static float value(uint32_t x){float f;std::memcpy(&f,&x,4);return f;}
template<typename T> static void write_tile(T*dst,const T tile[256],int tx,int ty,int width,int height){
#pragma HLS INLINE off
 const int columns=std::min(16,width-tx*16),rows=std::min(16,height-ty*16);
 for(int y=0;y<rows;y++){
  const int base=(ty*16+y)*width+tx*16;
  for(int x=0;x<columns;x++){
#pragma HLS PIPELINE II=1
   dst[base+x]=tile[y*16+x];
  }
 }
}
static void descriptors(const Surfel*world,const uint32_t*indices,uint32_t begin,uint32_t count,
 const Surfel cached[CACHE],const uint32_t cached_ids[CACHE],hls::stream<Desc> out[LANES]){
#pragma HLS INLINE off
 for(uint32_t j=0;j<count;j++){
#pragma HLS PIPELINE II=1
  Desc d;d.id=count<=CACHE?cached_ids[j]:indices[begin+j];d.s=count<=CACHE?cached[j]:world[d.id];
  d.numerator=(d.s.nx*d.s.x+d.s.ny*d.s.y)+d.s.nz*d.s.z;d.radius2=d.s.radius*d.s.radius;
  d.inverse_radius2=1.f/d.radius2;
  d.r=(uint32_t)(d.s.r*4096.f+.5f);d.g=(uint32_t)(d.s.g*4096.f+.5f);d.b=(uint32_t)(d.s.b*4096.f+.5f);d.end=false;
  for(int l=0;l<LANES;l++){
#pragma HLS UNROLL
   out[l].write(d);
  }
 }
 Desc stop={};stop.end=true;
 for(int l=0;l<LANES;l++){
#pragma HLS UNROLL
  out[l].write(stop);
 }
}
template<int L> static void enumerate(hls::stream<Desc>&in,hls::stream<Request>&out,int tx,int ty,const float rayx[640],const float rayy[480]){
#pragma HLS INLINE off
 // Exported boxes are clipped to 640x480. Include the terminal x+LANES and
 // y+1 values (<=643 and <=480); these widths do not wrap at image edges.
 Desc d={};ap_uint<10> x=0,x0=0,x1=0;ap_uint<9> y=0,y1=0;bool active=false;
 while(true){
#pragma HLS PIPELINE II=1
  if(!active){
   d=in.read();if(d.end)break;
   x0=std::max((int32_t)bits(d.s.submap),tx*16);x1=std::min((int32_t)bits(d.s.pad0),tx*16+15);y=std::max((int32_t)bits(d.s.pad1),ty*16);y1=std::min((int32_t)bits(d.s.pad2),ty*16+15);
   x0+=(L-(int(x0)%LANES)+LANES)%LANES;x=x0;active=x0<=x1&&y<=y1;
  }else{
   Request r;r.d=d;r.rx=rayx[x];r.ry=rayy[y];r.pixel=((y-ty*16)*16+x-tx*16)/LANES;r.end=false;out.write(r);
   x+=LANES;if(x>x1){x=x0;y++;}active=y<=y1;
  }
 }
 Request stop={};stop.end=true;out.write(stop);
}
template<int L> static void geometry(hls::stream<Request>&in,hls::stream<Fragment>&out,float nearz,float farz){
#pragma HLS INLINE off
 while(true){
#pragma HLS PIPELINE II=1
  Request r=in.read();Fragment f={};f.end=r.end;
  if(r.end){out.write(f);break;}
  const Surfel&s=r.d.s;float den=(s.nx*r.rx+s.ny*r.ry)+s.nz;
  float z=r.d.numerator/den;float dx=r.rx*z-s.x,dy=r.ry*z-s.y,dz=z-s.z;
  float distance2=(dx*dx+dy*dy)+dz*dz,q=distance2*r.d.inverse_radius2;
  // For nonnegative FP32 a and positive FP32 b, round(a/b)<1 iff a<b:
  // even the predecessor of b is at least 2^-24*b below it, farther than
  // the rounding midpoint 1-2^-25. Coverage and z retain their exact owners.
  // The reciprocal approximation is used ONLY for color weights.
  f.valid=std::fabs(den)>=1e-6f&&s.radius>0&&z>=nearz&&z<=farz&&distance2<r.d.radius2;
  f.z=z;f.id=r.d.id;f.pixel=r.pixel;float w=(1.f-q)*std::fmin(32.f,std::fmax(1.f,s.weight));
  f.weight=f.valid?(uint32_t)(w*262144.f+.5f):0;f.r=r.d.r;f.g=r.d.g;f.b=r.d.b;
  // Rejected bounding-box samples have no state effect; do not consume scarce
  // tile-RAM update slots for them. The end token always passes through.
  if(f.valid)out.write(f);
 }
}
template<int L> static void accumulate(hls::stream<Fragment>&in,uint32_t zbuf[PIXELS],int32_t ids[PIXELS],
 uint64_t sw[PIXELS],uint64_t sr[PIXELS],uint64_t sg[PIXELS],uint64_t sb[PIXELS],int pass,float band){
#pragma HLS INLINE off
 while(true){
#pragma HLS PIPELINE II=2
  Fragment f=in.read();if(f.end)break;if(!f.valid)continue;unsigned p=f.pixel;
  if(pass==0){uint32_t z=bits(f.z),old=zbuf[p];int32_t id=ids[p];bool take=(z<old)|((z==old)&((id<0)|(f.id<(uint32_t)id)));zbuf[p]=take?z:old;ids[p]=take?(int32_t)f.id:id;}
  else if(std::fabs(f.z-value(zbuf[p]))<=band){sw[p]+=f.weight;sr[p]+=(uint64_t)f.weight*f.r;sg[p]+=(uint64_t)f.weight*f.g;sb[p]+=(uint64_t)f.weight*f.b;}
 }
}
static void process_pass(const Surfel*surf,const uint32_t*indices,uint32_t begin,uint32_t count,const Surfel cache[CACHE],const uint32_t cacheids[CACHE],
 int tx,int ty,const float rx[LANES][640],const float ry[LANES][480],float nearz,float farz,float band,int pass,
 uint32_t z[LANES][PIXELS],int32_t ids[LANES][PIXELS],uint64_t w[LANES][PIXELS],uint64_t r[LANES][PIXELS],uint64_t g[LANES][PIXELS],uint64_t b[LANES][PIXELS]){
#pragma HLS INLINE off
#pragma HLS DATAFLOW
 hls::stream<Desc> ds[LANES];
 hls::stream<Request> qs[LANES];
 hls::stream<Fragment> fs[LANES];
#pragma HLS ARRAY_PARTITION variable=ds complete
#pragma HLS ARRAY_PARTITION variable=qs complete
#pragma HLS ARRAY_PARTITION variable=fs complete
#pragma HLS STREAM variable=ds depth=32
#pragma HLS STREAM variable=qs depth=32
#pragma HLS STREAM variable=fs depth=32
#pragma HLS BIND_STORAGE variable=ds type=fifo impl=lutram
#pragma HLS BIND_STORAGE variable=qs type=fifo impl=lutram
#pragma HLS BIND_STORAGE variable=fs type=fifo impl=lutram
 descriptors(surf,indices,begin,count,cache,cacheids,ds);
 enumerate<0>(ds[0],qs[0],tx,ty,rx[0],ry[0]);
 geometry<0>(qs[0],fs[0],nearz,farz);
 accumulate<0>(fs[0],z[0],ids[0],w[0],r[0],g[0],b[0],pass,band);
#if LIUJING_LANES >= 2
 enumerate<1>(ds[1],qs[1],tx,ty,rx[1],ry[1]);
 geometry<1>(qs[1],fs[1],nearz,farz);
 accumulate<1>(fs[1],z[1],ids[1],w[1],r[1],g[1],b[1],pass,band);
#endif
#if LIUJING_LANES == 4
 enumerate<2>(ds[2],qs[2],tx,ty,rx[2],ry[2]);
 geometry<2>(qs[2],fs[2],nearz,farz);
 accumulate<2>(fs[2],z[2],ids[2],w[2],r[2],g[2],b[2],pass,band);
 enumerate<3>(ds[3],qs[3],tx,ty,rx[3],ry[3]);
 geometry<3>(qs[3],fs[3],nearz,farz);
 accumulate<3>(fs[3],z[3],ids[3],w[3],r[3],g[3],b[3],pass,band);
#endif
}
extern "C" void lj_raster(const Surfel*camera_surfels,const uint32_t*offsets,const uint32_t*indices,float*depth,int32_t*ids,uint32_t*rgb,
 int width,int height,float fx,float fy,float cx,float cy,float near_z,float far_z,float blend_band,int mode){
#pragma HLS INTERFACE m_axi port=camera_surfels offset=slave bundle=surf depth=4096
#pragma HLS INTERFACE m_axi port=offsets offset=slave bundle=index depth=1201
#pragma HLS INTERFACE m_axi port=indices offset=slave bundle=index depth=65536
#pragma HLS INTERFACE m_axi port=depth offset=slave bundle=output_mem depth=307200
#pragma HLS INTERFACE m_axi port=ids offset=slave bundle=output_mem depth=307200
#pragma HLS INTERFACE m_axi port=rgb offset=slave bundle=output_mem depth=307200
#pragma HLS INTERFACE s_axilite port=return bundle=control
#pragma HLS INTERFACE s_axilite port=camera_surfels bundle=control
#pragma HLS INTERFACE s_axilite port=offsets bundle=control
#pragma HLS INTERFACE s_axilite port=indices bundle=control
#pragma HLS INTERFACE s_axilite port=depth bundle=control
#pragma HLS INTERFACE s_axilite port=ids bundle=control
#pragma HLS INTERFACE s_axilite port=rgb bundle=control
#pragma HLS INTERFACE s_axilite port=width bundle=control
#pragma HLS INTERFACE s_axilite port=height bundle=control
#pragma HLS INTERFACE s_axilite port=fx bundle=control
#pragma HLS INTERFACE s_axilite port=fy bundle=control
#pragma HLS INTERFACE s_axilite port=cx bundle=control
#pragma HLS INTERFACE s_axilite port=cy bundle=control
#pragma HLS INTERFACE s_axilite port=near_z bundle=control
#pragma HLS INTERFACE s_axilite port=far_z bundle=control
#pragma HLS INTERFACE s_axilite port=blend_band bundle=control
#pragma HLS INTERFACE s_axilite port=mode bundle=control
 if(width<=0||height<=0||width>640||height>480||!(fx>0)||!(fy>0))return;
 float rx[LANES][640],ry[LANES][480];
#pragma HLS ARRAY_PARTITION variable=rx complete dim=1
#pragma HLS ARRAY_PARTITION variable=ry complete dim=1
 for(int x=0;x<width;x++){
#pragma HLS PIPELINE II=1
  float v=(x-cx)/fx;
  for(int l=0;l<LANES;l++){
#pragma HLS UNROLL
   rx[l][x]=v;
  }
 }
 for(int y=0;y<height;y++){
#pragma HLS PIPELINE II=1
  float v=(y-cy)/fy;
  for(int l=0;l<LANES;l++){
#pragma HLS UNROLL
   ry[l][y]=v;
  }
 }
 Surfel cache[CACHE];uint32_t cacheids[CACHE];
#pragma HLS BIND_STORAGE variable=cache type=ram_2p impl=bram
 uint32_t z[LANES][PIXELS];int32_t owner[LANES][PIXELS];uint64_t sw[LANES][PIXELS],sr[LANES][PIXELS],sg[LANES][PIXELS],sb[LANES][PIXELS];
#pragma HLS ARRAY_PARTITION variable=z complete dim=1
#pragma HLS ARRAY_PARTITION variable=owner complete dim=1
#pragma HLS ARRAY_PARTITION variable=sw complete dim=1
#pragma HLS ARRAY_PARTITION variable=sr complete dim=1
#pragma HLS ARRAY_PARTITION variable=sg complete dim=1
#pragma HLS ARRAY_PARTITION variable=sb complete dim=1
#pragma HLS BIND_STORAGE variable=z type=ram_t2p impl=bram latency=1
#pragma HLS BIND_STORAGE variable=owner type=ram_t2p impl=bram latency=1
#pragma HLS BIND_STORAGE variable=sw type=ram_t2p impl=bram latency=1
#pragma HLS BIND_STORAGE variable=sr type=ram_t2p impl=bram latency=1
#pragma HLS BIND_STORAGE variable=sg type=ram_t2p impl=bram latency=1
#pragma HLS BIND_STORAGE variable=sb type=ram_t2p impl=bram latency=1
 const int txs=(width+15)/16,tys=(height+15)/16;
 for(int ty=0;ty<tys;ty++)for(int tx=0;tx<txs;tx++){
  int tile=ty*txs+tx;uint32_t begin=offsets[tile],count=offsets[tile+1]-begin;
  if(count<=CACHE)for(uint32_t j=0;j<count;j++){
#pragma HLS PIPELINE II=1
   uint32_t id=indices[begin+j];cacheids[j]=id;cache[j]=camera_surfels[id];
  }
  for(int p=0;p<PIXELS;p++){
#pragma HLS PIPELINE II=1
   for(int l=0;l<LANES;l++){
#pragma HLS UNROLL
    z[l][p]=bits(far_z+1);owner[l][p]=-1;sw[l][p]=sr[l][p]=sg[l][p]=sb[l][p]=0;
   }
  }
  for(int pass=0;pass<(mode==3?1:2);pass++)process_pass(camera_surfels,indices,begin,count,cache,cacheids,tx,ty,rx,ry,near_z,far_z,blend_band,pass,z,owner,sw,sr,sg,sb);
  float tile_depth[256];int32_t tile_ids[256];uint32_t tile_rgb[256];
  for(int p=0;p<256;p++){
#pragma HLS PIPELINE II=1
   int l=p%LANES,k=p/LANES;
   tile_ids[p]=owner[l][k];tile_depth[p]=owner[l][k]>=0?value(z[l][k]):0;
   uint32_t r=0,g=0,b=0;if(sw[l][k]){float inverse=1.f/(float(sw[l][k])*4096.f);
    r=(uint32_t)std::fmin(255.f,float(sr[l][k])*inverse+.5f);g=(uint32_t)std::fmin(255.f,float(sg[l][k])*inverse+.5f);b=(uint32_t)std::fmin(255.f,float(sb[l][k])*inverse+.5f);}
   tile_rgb[p]=r|(g<<8)|(b<<16);
  }
  // Separate burst regions on the shared AXI write channel; bounded rows also
  // cover partial 17x17 tiles. Association jobs never transfer unused color.
  write_tile(depth,tile_depth,tx,ty,width,height);
  write_tile(ids,tile_ids,tx,ty,width,height);
  if(mode!=3)write_tile(rgb,tile_rgb,tx,ty,width,height);
 }
}
