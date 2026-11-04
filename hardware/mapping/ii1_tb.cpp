#include "core.hpp"
#include <cmath>
#include <cstdio>
static uint32_t seed=17;
static float random01(){seed=1664525*seed+1013904223;return float(seed>>8)/16777216.f;}
int main(){
 static Surfel source[4096]={},projected[4096]={};
 static uint32_t offsets[1201]={},indices[65536]={},rgb[307200]={},reference_rgb[307200]={};
 static float depth[307200]={},reference_depth[307200]={};
 static int32_t ids[307200]={},reference_ids[307200]={};
 float pose[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
 void*map=lj_create(4096,2);
 for(int iteration=0;iteration<3;iteration++){
  const int count=iteration==2?0:100;
  Camera camera={iteration==0?32:17,iteration==0?24:17,35,35,15.5,11.5,.1,5};
  for(int i=0;i<count;i++){
   Surfel&s=source[i];s.z=1.8f+random01()*.2f;s.x=(random01()-.5f)*.9f;s.y=(random01()-.5f)*.6f;
   s.nx=(random01()-.5f)*.3f;s.ny=(random01()-.5f)*.3f;s.nz=-std::sqrt(1-s.nx*s.nx-s.ny*s.ny);s.radius=.1f+random01()*.2f;
   s.r=random01()*255;s.g=random01()*255;s.b=random01()*255;s.weight=1+random01()*8;
  }
  if(lj_set(map,source,count)<0)return 1;
  if(lj_prepare(map,&camera,pose,projected,offsets,indices,65536)<0)return 2;
  if(lj_render(map,&camera,pose,reference_depth,reference_ids,reference_rgb,0)<0)return 3;
  lj_raster(projected,offsets,indices,depth,ids,rgb,camera.width,camera.height,camera.fx,camera.fy,camera.cx,camera.cy,camera.near_z,camera.far_z,.01);
  for(int i=0;i<camera.width*camera.height;i++){
   if(ids[i]!=reference_ids[i]||std::fabs(depth[i]-reference_depth[i])>1e-5){std::printf("geometry mismatch iteration=%d pixel=%d\n",iteration,i);return 4;}
   for(int ch=0;ch<3;ch++)if(std::abs(int((rgb[i]>>(ch*8))&255)-int((reference_rgb[i]>>(ch*8))&255))>1)return 5;
  }
 }
 lj_destroy(map);std::puts("Overlapping surfels, partial tiles and empty reset match independent CPU reference");return 0;
}
