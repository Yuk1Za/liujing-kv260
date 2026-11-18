#pragma once
#include "matched_commit.hpp"
#include <cmath>
#ifdef NATIONAL_INTEGER_RGB
#include <array>
// Stored weights start at 1 and increment by one up to 32. For a single
// channel, n = old*w + sample is exact in binary32. Rounding n/d to the
// nearest integer (ties up) gives floor((n + floor(d/2))/d), d=w+1.
static constexpr std::array<unsigned,33> color_reciprocals() {
    std::array<unsigned,33> table{};
    for(unsigned d=2;d<=32;++d)table[d]=((1u<<20)+d-1)/d;
    return table;
}
static unsigned merge_rgb(unsigned previous, unsigned sample, unsigned weight) {
    static constexpr auto reciprocal=color_reciprocals();
    unsigned d=weight+1,scale=reciprocal[d],rgb=0;
    for(unsigned shift=0;shift<24;shift+=8){
#pragma HLS UNROLL
        unsigned a=(previous>>shift)&255,b=(sample>>shift)&255;
        unsigned numerator=a*weight+b+d/2;
        // Reciprocal rounding adds < 8177/2^20 < 1/32 to the quotient.
        // The largest product is below (256*2^20), so 32 bits suffice.
        rgb|=((numerator*scale)>>20)<<shift;
    }
    return rgb;
}
#endif
inline SurfaceWord merge_surface(SurfaceWord a, const SurfaceWord& b) {
    const float w = a.weight < 31.f ? a.weight : 31.f;
    const float divisor = w + 1.f;
    a.x = (a.x*w+b.x)/divisor;
    a.y = (a.y*w+b.y)/divisor;
    a.z = (a.z*w+b.z)/divisor;
#ifdef NATIONAL_INTEGER_RGB
    a.rgb=merge_rgb(a.rgb,b.rgb,unsigned(w));
#else
    uint32_t color=0;
    for (unsigned shift=0;shift<24;shift+=8) {
        const float old_component=float((a.rgb>>shift)&255u);
        const float new_component=float((b.rgb>>shift)&255u);
        color |= uint32_t((old_component*w+new_component)/divisor+.5f)<<shift;
    }
    a.rgb=color;
#endif
    float nx=a.nx*w+b.nx, ny=a.ny*w+b.ny, nz=a.nz*w+b.nz;
    const float norm=std::sqrt((nx*nx+ny*ny)+nz*nz);
    a.nx=nx/norm; a.ny=ny/norm; a.nz=nz/norm;
    a.weight=divisor;
    return a;
}
