#include "raster.hpp"
#include <cmath>
#include <cstdio>
int main(){
 // Co-simulation snapshots the declared interface depth, not just used elements.
 static Surfel s[4096]={};s[0].z=2;s[0].nz=-1;s[0].radius=1;s[0].r=240;s[0].weight=1;
 s[0].submap=0;s[0].pad0=15;s[0].pad1=0;s[0].pad2=15;
 s[1]=s[0];s[1].z=1;s[1].r=0;s[1].g=180;
 static uint32_t off[1201]={0,2},idx[65536]={0,1},rgb[307200]={};static float depth[307200]={};static int32_t id[307200]={};
 lj_raster(s,off,idx,depth,id,rgb,16,16,30,30,7.5,7.5,.1,5,.01);
 for(int i=0;i<256;i++)if(id[i]!=1||std::fabs(depth[i]-1)>1e-5||rgb[i]!=(180u<<8))return 1;
 off[1]=0;lj_raster(s,off,idx,depth,id,rgb,16,16,30,30,7.5,7.5,.1,5,.01);
 for(int i=0;i<256;i++)if(id[i]!=-1||depth[i]!=0||rgb[i]!=0)return 2;
 std::puts("HLS C: occlusion and full reset PASS");return 0;
}
