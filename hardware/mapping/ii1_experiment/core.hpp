#pragma once
#include "raster.hpp"
#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif
extern "C" {
API void* lj_create(int capacity,int stride);
API void lj_destroy(void*);
// Row-major T_world_camera. Registered, rectified depth/RGB, metres/RGB bytes.
// Returns added count; negative => rejected, no map changes.
API int lj_integrate(void*,const float*depth,const uint8_t*rgb,const Camera*,const float*T,double stamp);
API int lj_render(void*,const Camera*,const float*T,float*depth,int32_t*ids,uint32_t*rgb,int mode);
API uint32_t lj_count(void*);
API uint64_t lj_version(void*);
API uint64_t lj_capacity_drops(void*);
API int lj_threads(int requested);
API int lj_copy(void*,Surfel*out,int capacity);
API int lj_set(void*,const Surfel*in,int count);
API int lj_prepare(void*,const Camera*,const float*T,Surfel*out,uint32_t*offsets,uint32_t*indices,int index_capacity);
}
