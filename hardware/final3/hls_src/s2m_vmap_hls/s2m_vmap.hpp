// s2m_vmap.hpp —— 内核与 testbench 共用的常量与原型（避免 tb 里复制一份定义）
#pragma once
#include <ap_int.h>

#ifndef WX
#define WX 32
#endif
#ifndef WY
#define WY 32
#endif
#ifndef WZ
#define WZ 8
#endif
#define WMAX  (WX * WY * WZ)
// ★ valid 位图的长度（**字节**）。ap_memory 的深度必须编译期已知，否则 HLS 会退化成
//   `ap_none` 的 8 位单口 —— 硬件里 idx>>3 的随机读就永远读到同一个字节（C 仿真看不出来）。
//   8192 B = 65,536 格，够 90×90×8（L=1.6 时 ±72 m × ±6.4 m）这类窗口用；
//   约束：dimx*dimy*dimz <= 65536。
#define S2M_VBLEN 8192
#define NCELL 27

extern "C" void s2m_vmap(
    const float          *q,      // [nq*4]
    const ap_uint<32>    *vbw,    // [S2M_VBLEN/4] valid 位图（按 32 位字传，stage-1 先拷进内部 BRAM）
    const float          *tab,    // [WMAX*6] 图元表（平面用前 4 个）
    const float          *parq,   // [12] R(9) | t(3)        —— 邻域扫描用
    const float          *parm,   // [27] J 旋转列系数        —— 残差/累加用
    float                *Hb,     // [27] H 上三角 21 + b 6
    int nq, int mode,
    float L, float invL, float rmax,
    float orgx, float orgy, float orgz,
    int dimx, int dimy, int dimz,
    int rescan);
