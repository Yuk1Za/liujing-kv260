#pragma once
#include <cstdint>
namespace lj2 {
// opaque_v1: IEEE FP32 geometry, R in bits 7:0, stable unsigned owner ID.
struct Surface {float x,y,z,nx,ny,nz,radius;uint32_t rgb,id;float weight;int32_t x0,x1,y0,y1;uint32_t reserved[2];};
static_assert(sizeof(Surface)==64,"DMA contract");
}
extern "C" void lj_raster(const lj2::Surface*,const uint32_t*,const uint32_t*,float*,int32_t*,uint32_t*,int,int,float,float,float,float,float,float,int);
