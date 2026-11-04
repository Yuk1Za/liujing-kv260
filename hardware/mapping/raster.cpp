#include "raster.hpp"
#include <cmath>
// No alpha ordering: first nearest surface, then weighted color on that surface.
// Offsets/indices are validated by the host, inclusive sentinel at tile_count.
static bool fragment(const Surfel&s,float rx,float ry,float nearz,float farz,float&z,float&w){
 const float denom=s.nx*rx+s.ny*ry+s.nz;
 if(std::fabs(denom)<1e-6f || !(s.radius>0)) return false;
 z=(s.nx*s.x+s.ny*s.y+s.nz*s.z)/denom;
 if(!(z>=nearz && z<=farz)) return false;
 float dx=rx*z-s.x,dy=ry*z-s.y,dz=z-s.z;
 float q=(dx*dx+dy*dy+dz*dz)/(s.radius*s.radius);
 if(!(q<1.0f)) return false;
 w=(1.0f-q)*std::fmin(32.0f,std::fmax(1.0f,s.weight));
 return true;
}
extern "C" void lj_raster(const Surfel*camera_surfels,const uint32_t*offsets,
 const uint32_t*indices,float*depth,int32_t*ids,uint32_t*rgb,
 int width,int height,float fx,float fy,float cx,float cy,float near_z,float far_z,float blend_band){
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
 if(width<=0||height<=0||width>MAX_WIDTH||height>MAX_HEIGHT||!(fx>0)||!(fy>0))return;
 const int txs=(width+15)/16,tys=(height+15)/16;
 float zbuf[256],sw[256],sr[256],sg[256],sb[256]; int ibuf[256];
 for(int ty=0;ty<tys;ty++)for(int tx=0;tx<txs;tx++){
  const int tile=ty*txs+tx;
  for(int p=0;p<256;p++){
#pragma HLS PIPELINE
   zbuf[p]=far_z+1;ibuf[p]=-1;sw[p]=sr[p]=sg[p]=sb[p]=0;
  }
  for(int pass=0;pass<2;pass++){
   for(uint32_t j=offsets[tile];j<offsets[tile+1];j++){
    const uint32_t id=indices[j]; const Surfel s=camera_surfels[id];
    int x0=(int)s.submap,x1=(int)s.pad0,y0=(int)s.pad1,y1=(int)s.pad2;
    x0=x0>tx*16?x0:tx*16;x1=x1<tx*16+15?x1:tx*16+15;
    y0=y0>ty*16?y0:ty*16;y1=y1<ty*16+15?y1:ty*16+15;
    int x=x0,y=y0,count=(x1-x0+1)*(y1-y0+1);
    if(x0>x1||y0>y1)count=0;
    for(int fragment_index=0;fragment_index<count;fragment_index++){
#pragma HLS PIPELINE
     int px=x,py=y,p=(py-ty*16)*16+px-tx*16;
     if(x==x1){x=x0;y++;}else x++;
     if(px>=width||py>=height)continue;
     float z,w;
     if(!fragment(s,(px-cx)/fx,(py-cy)/fy,near_z,far_z,z,w))continue;
     if(pass==0){
      if(z<zbuf[p] || (z==zbuf[p]&&(ibuf[p]<0||id<(uint32_t)ibuf[p]))){zbuf[p]=z;ibuf[p]=id;}
     }else if(std::fabs(z-zbuf[p])<=blend_band){
      sw[p]+=w;sr[p]+=w*s.r;sg[p]+=w*s.g;sb[p]+=w*s.b;
     }
    }
   }
  }
  for(int p=0;p<256;p++){
#pragma HLS PIPELINE
   int x=tx*16+p%16,y=ty*16+p/16;
   if(x>=width||y>=height)continue;
   const int o=y*width+x; ids[o]=ibuf[p];depth[o]=ibuf[p]>=0?zbuf[p]:0;
   uint32_t r=0,g=0,b=0;
   if(sw[p]>0){r=(uint32_t)std::fmin(255.f,std::fmax(0.f,sr[p]/sw[p]+.5f));g=(uint32_t)std::fmin(255.f,std::fmax(0.f,sg[p]/sw[p]+.5f));b=(uint32_t)std::fmin(255.f,std::fmax(0.f,sb[p]/sw[p]+.5f));}
   rgb[o]=r|(g<<8)|(b<<16);
  }
 }
}
