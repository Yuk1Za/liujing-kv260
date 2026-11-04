#pragma once
#include <cstdint>
// Optical camera coordinates: +x right, +y down, +z forward. RGB in [0,255].
// Four 128-bit words per surfel; native and accelerator ABI share 64-byte records.
// int_bounds_v1: descriptor words 12..15 contain int32 x0,x1,y0,y1 bit patterns,
// NOT floating point numbers. Only lj_prepare_packed produces this layout.
// World-map records retain submap/reserved meanings; never write descriptors back.
struct Surfel { float x,y,z,nx,ny,nz,radius,r,g,b,weight,last_seen,submap,pad0,pad1,pad2; };
static_assert(sizeof(Surfel)==64,"surfel ABI");
struct Camera { int width,height; float fx,fy,cx,cy,near_z,far_z; };
constexpr int TILE=16;
constexpr int MAX_WIDTH=640,MAX_HEIGHT=480;
extern "C" void lj_raster(const Surfel* camera_surfels,const uint32_t* offsets,
 const uint32_t* indices,float* depth,int32_t* ids,uint32_t* rgb,
 int width,int height,float fx,float fy,float cx,float cy,
 float near_z,float far_z,float blend_band,int mode=0);
