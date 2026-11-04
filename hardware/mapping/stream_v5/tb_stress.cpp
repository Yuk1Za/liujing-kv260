#include "core.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <cstring>
static uint32_t seed=17;
static float random01(){seed=1664525*seed+1013904223;return float(seed>>8)/16777216.f;}
int main(){
 static Surfel source[4096]={},projected[4096]={};
 static uint32_t off[1201]={},idx[65536]={},rgb[307200]={},ref_rgb[307200]={};
 static float depth[307200]={},ref_depth[307200]={};static int32_t ids[307200]={},ref_ids[307200]={};
 static float fp_depth[307200];static int32_t fp_ids[307200];static uint32_t fp_rgb[307200];
 float pose[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};void*map=lj_create(4096,2);
 for(int iteration=0;iteration<10;iteration++){
  int n=iteration==2?0:((iteration==3||iteration==4)?513:100);int mode=iteration==4?3:0;
  Camera c={iteration==0?32:17,iteration==0?24:17,35,35,8,8,.1,5};
  for(int i=0;i<n;i++){
   Surfel&s=source[i];s={};s.z=1.8f+random01()*.2f;s.x=(random01()-.5f)*.9f;s.y=(random01()-.5f)*.6f;s.nx=(random01()-.5f)*.3f;s.ny=(random01()-.5f)*.3f;s.nz=-std::sqrt(1-s.nx*s.nx-s.ny*s.ny);s.radius=.1f+random01()*.2f;
   if(iteration==3||iteration==4){s.x=0;s.y=0;s.z=2;s.nx=0;s.ny=0;s.nz=-1;s.radius=.04f;}
   if(iteration==5){s.x*=.04f;s.y*=.04f;s.z=.100001f;s.nx=0;s.ny=0;s.nz=-1;s.radius=.02f;}
   if(iteration==6){s.x*=.1f;s.y*=.1f;s.z=.2f;s.nx=.99995f;s.ny=0;s.nz=-.01f;s.radius=.1f;}
   if(iteration==7){s.x=0;s.y=0;s.z=1;s.nx=0;s.ny=0;s.nz=-1;s.radius=.00001f;}
   if(iteration==8){s.z=(i&1)?1.f:1.2f;s.nx=0;s.ny=0;s.nz=-1;s.radius=.1f;}
   if(iteration==9){s.x=0;s.y=0;s.z=std::nextafter(1.f,2.f);s.nx=0;s.ny=0;s.nz=-1;s.radius=.03f;if(i==99)s.z=1.f;}
   s.r=random01()*255;s.g=random01()*255;s.b=random01()*255;s.weight=1+random01()*31;
  }
  if(lj_set(map,source,n)<0||lj_prepare(map,&c,pose,projected,off,idx,65536)<0)return 1;
  if(lj_render(map,&c,pose,ref_depth,ref_ids,ref_rgb,mode==3?3:5)<0)return 2;
  if(lj_render(map,&c,pose,fp_depth,fp_ids,fp_rgb,0)<0)return 4;
  double squared_error=0;int components=0;
  for(int k=0;k<c.width*c.height;k++){
   if(ref_ids[k]!=fp_ids[k]||std::memcmp(&ref_depth[k],&fp_depth[k],4))return 5;
   if(mode!=3&&fp_ids[k]>=0)for(int ch=0;ch<3;ch++){int error=int((fp_rgb[k]>>(8*ch))&255)-int((ref_rgb[k]>>(8*ch))&255);squared_error+=error*error;components++;}
  }
  if(components&&squared_error/components>6.5025){std::printf("FP32 quality failure case=%d MSE=%f\n",iteration,squared_error/components);return 6;}
  // Same integer accumulation and stable IDs must survive reordered candidates.
  if(iteration>=3)for(int t=0;t<4;t++)std::reverse(idx+off[t],idx+off[t+1]);
  lj_raster(projected,off,idx,depth,ids,rgb,c.width,c.height,c.fx,c.fy,c.cx,c.cy,c.near_z,c.far_z,.01,mode);
  for(int k=0;k<c.width*c.height;k++)if(ids[k]!=ref_ids[k]||std::memcmp(&depth[k],&ref_depth[k],4)||(mode!=3&&rgb[k]!=ref_rgb[k])){std::printf("mismatch case=%d pixel=%d id=%d ref=%d rgb=%x ref=%x\n",iteration,k,ids[k],ref_ids[k],rgb[k],ref_rgb[k]);return 3;}
 }
 lj_destroy(map);std::puts("Exact qrcp CPU parity: overlap, 17x17 edges, empty reset, 513 ties, reversed lists, association, near clipping, grazing, tiny discs, depth edge and one-ULP depth ordering");return 0;
}
