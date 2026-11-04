// =====================================================================================
// pl_s2m.hpp —— map 残差块 PL 的 **CPU 侧驱动**（v2a）
//
// 三件事：
//   ① 把 A8/A9 的哈希表（PlaneVoxelMap / LineVoxelMap）**打包成窗口形态**
//      （valid 位图 + 平铺的 tab[6]），也就是 PL 要从 DDR 读到的那两块 buffer；
//   ② 每次迭代算出 J 的 27 个旋转系数 M（只依赖 R 的 6 个三角函数，CPU 顺手算）；
//   ③ 一份**与内核同式的 float 参考实现** —— 用来在接 PL 之前先证明
//      "打包 + M 系数 + 查询规则"这套逻辑是对的（否则上板后出错分不清是硬件还是驱动）。
//
// 窗口约定（与内核严格一致）：
//   cellIdx(p) = floor(p / L)；窗口第 i 格的世界坐标区间是 [org + i*L, org + (i+1)*L)
//   org = orgCell * L  ⇒ 窗口索引 = 世界格索引 − orgCell
//   ★ 这样 CPU 打包时只做整数减法，不碰浮点 floor。
// =====================================================================================
#pragma once
#include <cstdint>
#include <cstring>
#include <vector>
#include <cmath>
#include <algorithm>

namespace pls2m
{

// ---------------- key 解包（对应 map_optimization.cpp 的 pack：x | y<<21 | z<<42）----
static inline int s21(int64_t v)
{
    int t = (int)(v & 0x1FFFFF);
    return (t & 0x100000) ? (t - 0x200000) : t;
}
static inline void unpack_key(int64_t k, int &x, int &y, int &z)
{
    x = s21(k);
    y = s21(k >> 21);
    z = s21(k >> 42);
}

// 27 邻域偏移（与内核同序，索引对拍时要一致）
static const signed char DI27[27] = {-1,-1,-1,-1,-1,-1,-1,-1,-1, 0,0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1};
static const signed char DJ27[27] = {-1,-1,-1, 0,0,0, 1,1,1,-1,-1,-1, 0,0,0, 1,1,1,-1,-1,-1, 0,0,0, 1,1,1};
static const signed char DK27[27] = {-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1};

// 21 项上三角 + 6 项 b 的下标（与内核 HI/HJ 同一张表）
static const signed char HI27[27] = {0,0,0,0,0,0, 1,1,1,1,1, 2,2,2,2, 3,3,3, 4,4, 5, 0,1,2,3,4,5};
static const signed char HJ27[27] = {0,1,2,3,4,5, 1,2,3,4,5, 2,3,4,5, 3,4,5, 4,5, 5, 0,0,0,0,0,0};

// ---------------- 窗口 ----------------
struct PlWin
{
    int   dimx = 64, dimy = 64, dimz = 16;
    float L = 1.6f, invL = 1.0f / 1.6f;
    int   ocx = 0, ocy = 0, ocz = 0;      // 窗口原点的**格索引**
    float orgx = 0.f, orgy = 0.f, orgz = 0.f;

    std::vector<uint8_t> vb;              // dimx*dimy*dimz/8
    std::vector<float>   tab;             // dimx*dimy*dimz*6（平面只用前 4 个）
    long nvalid = 0, ncells = 0;

    void reset(int dx, int dy, int dz, float leaf, int ox, int oy, int oz)
    {
        dimx = dx; dimy = dy; dimz = dz;
        L = leaf; invL = 1.f / leaf;
        ocx = ox; ocy = oy; ocz = oz;
        orgx = (float)ox * leaf;
        orgy = (float)oy * leaf;
        orgz = (float)oz * leaf;
        const size_t n = (size_t)dimx * dimy * dimz;
        vb.assign((n + 7) / 8, 0);
        tab.assign(n * 6, 0.f);
        nvalid = 0;
        ncells = 0;
    }
    inline int idx(int i, int j, int k) const { return (i * dimy + j) * dimz + k; }

    // 从 map 的（keys, valid, 扁平表）填窗口
    //   src 是 stride_in float/格（平面 4、直线 6），写进 tab 的前 6 个
    void pack(const std::vector<int64_t> &keys, const std::vector<unsigned char> &valid,
              const float *src, int stride_in)
    {
        std::memset(vb.data(), 0, vb.size());
        std::fill(tab.begin(), tab.end(), 0.f);
        nvalid = 0;
        ncells = (long)keys.size();
        for (size_t b = 0; b < keys.size(); ++b)
        {
            if (!valid[b]) continue;
            int cx, cy, cz;
            unpack_key(keys[b], cx, cy, cz);
            const int i = cx - ocx, j = cy - ocy, k = cz - ocz;
            if (i < 0 || j < 0 || k < 0 || i >= dimx || j >= dimy || k >= dimz) continue;
            const int t = idx(i, j, k);
            vb[t >> 3] |= (uint8_t)(1u << (t & 7));
            for (int q = 0; q < stride_in; ++q) tab[(size_t)t * 6 + q] = src[(size_t)b * stride_in + q];
            ++nvalid;
        }
    }
};

// ---------------- J 的 27 个旋转系数 ----------------
// 原码（map_optimization.cpp 的 surf/cornerOptimization）：
//   arx = (a1*x + a2*y + a3*z)*cx + (b1*x + b2*y + b3*z)*cy + (c1*x + c2*y + c3*z)*cz
//   ary = (....)*cx +                              (....)*cz      ← **缺 cy 整项**
//   arz = (d1*x + d2*y)*cx + (e1*x + e2*y)*cy + (f1*x + f2*y)*cz  ← 三项都缺 z
// 内核的 pc = (y, z, x)（lidar→camera 换轴）⇒ 每个 3 元组按 (y,z,x) 重排。
// R 的前 6 个元素约定为 (srx, crx, sry, cry, srz, crz) —— 与 computeTrans 的角序对应。
static inline void calc_M(const float *srx_crx, float M[27])
{
    const float srx = srx_crx[0], crx = srx_crx[1], sry = srx_crx[2],
                cry = srx_crx[3], srz = srx_crx[4], crz = srx_crx[5];

    // ★ 三组依次是 **coeff.x 项 / coeff.y 项 / coeff.z 项**（不是 y,z,x！）
    //   因为 LMOptimization 的循环里已经做过 lidar→camera 换轴：
    //   coeff.x = coeffSel.y、coeff.y = coeffSel.z、coeff.z = coeffSel.x，
    //   而内核的 cc0/cc1/cc2（= 原始 coeff 的 y/z/x）恰好等于换轴后的 cL.x/cL.y/cL.z。
    // ---- arz → M[0..8]（三项都缺 z 分量）----
    M[0] = -cry * crz - srx * sry * srz;   // coeff.x × y (pc0)
    M[1] = 0.f;                            // coeff.x × z (pc1)   ← 原码缺 z
    M[2] = crz * srx * sry - cry * srz;    // coeff.x × x (pc2)
    M[3] = -crx * srz;                     // coeff.y × y
    M[4] = 0.f;                            // coeff.y × z         ← 原码缺 z
    M[5] = crx * crz;                      // coeff.y × x
    M[6] = crz * sry - cry * srx * srz;    // coeff.z × y
    M[7] = 0.f;                            // coeff.z × z         ← 原码缺 z
    M[8] = sry * srz + cry * crz * srx;    // coeff.z × x

    // ---- arx → M[9..17] ----
    M[9]  = crx * crz * sry;               // coeff.x × y
    M[10] = -srx * sry;                    // coeff.x × z
    M[11] = crx * sry * srz;               // coeff.x × x
    M[12] = -crz * srx;                    // coeff.y × y
    M[13] = -crx;                          // coeff.y × z
    M[14] = -srx * srz;                    // coeff.y × x
    M[15] = crx * cry * crz;               // coeff.z × y
    M[16] = -cry * srx;                    // coeff.z × z
    M[17] = crx * cry * srz;               // coeff.z × x

    // ---- ary → M[18..26]（**coeff.y 整项缺** ⇒ M[21..23]=0）----
    M[18] = sry * srz + cry * crz * srx;   // coeff.x × y
    M[19] = crx * cry;                     // coeff.x × z
    M[20] = cry * srx * srz - crz * sry;   // coeff.x × x
    M[21] = 0.f;
    M[22] = 0.f;
    M[23] = 0.f;
    M[24] = cry * srz - crz * srx * sry;   // coeff.z × y
    M[25] = -crx * sry;                    // coeff.z × z
    M[26] = -cry * crz - srx * sry * srz;  // coeff.z × x
}

// ---------------- 参考实现（与内核同式，float）----------------
struct RefOut
{
    float H[21];
    float b[6];
    long  nq = 0, nhit = 0;
    // 诊断：miss 的原因要分开数，否则"命中率低"没法定位
    long  nmiss_cell = 0;       // 27 邻域里一个有效格都没有
    long  nmiss_rmax = 0;       // 选了格但 |r| > rmax
    long  nmiss_s    = 0;       // 过了 rmax 但 s <= 0.1
    long  ndiff_cell = 0;       // 两种格索引公式给出不同格的点数（浮点边界）
    float sum_abs_r = 0.f;      // 诊断：|rr| 均值
};

// qs: [n*3] 传感器系点；ps: [n*3] 世界系点（调用方已做好 R|t 变换）
// mode: 0=平面（tab 前 4 个 nx,ny,nz,d），1=直线（tab 6 个 c, t）
static inline void ref_Hb(const PlWin &w,
                          const float *qs, const float *ps, int nq,
                          const float M[27], int mode, float rmax, RefOut &out)
{
    for (int i = 0; i < 21; ++i) out.H[i] = 0.f;
    for (int i = 0; i < 6; ++i) out.b[i] = 0.f;
    out.nq = nq; out.nhit = 0; out.sum_abs_r = 0.f;
    out.nmiss_cell = out.nmiss_rmax = out.nmiss_s = 0; out.ndiff_cell = 0;

    const float invL = w.invL, L = w.L;
    for (int t = 0; t < nq; ++t)
    {
        const float px = ps[3 * t + 0], py = ps[3 * t + 1], pz = ps[3 * t + 2];
        const float qx = qs[3 * t + 0], qy = qs[3 * t + 1], qz = qs[3 * t + 2];

        // 窗口格。★ 用与 A8 `cellIdx()` 完全同式的 floor(p * invL) 再减 orgCell ——
        //   写成 floor((p - org) * invL) 数学上等价但**浮点上不等价**，落在格边界的点会差一格。
        const int ciA = (int)std::floor(px * invL) - w.ocx;
        const int cjA = (int)std::floor(py * invL) - w.ocy;
        const int ckA = (int)std::floor(pz * invL) - w.ocz;
        const int ciB = (int)std::floor((px - w.orgx) * invL);
        const int cjB = (int)std::floor((py - w.orgy) * invL);
        const int ckB = (int)std::floor((pz - w.orgz) * invL);
        if (ciA != ciB || cjA != cjB || ckA != ckB) ++out.ndiff_cell;
        const int ci = ciA, cj = cjA, ck = ckA;

        // 27 邻域：**先过残差门限、再比格心距离** —— 与 A8 的 query() 完全同序。
        //   ★ 顺序不能反：A8 在循环里就 `if (fabs(r) > rmax) continue;`，
        //     若这里改成"先选最近格再判 rmax"，最近的那格不合格就整点丢掉（实测少 615/3380 个）。
        int   bidx = -1;
        float best = 1e30f, bestR = 0.f;
        for (int n = 0; n < 27; ++n)
        {
            const int i = ci + DI27[n], j = cj + DJ27[n], k = ck + DK27[n];
            if (i < 0 || j < 0 || k < 0 || i >= w.dimx || j >= w.dimy || k >= w.dimz) continue;
            const int id = w.idx(i, j, k);
            if (((w.vb[id >> 3] >> (id & 7)) & 1u) == 0) continue;
            const float *a0 = &w.tab[(size_t)id * 6];
            float r;
            if (mode == 0)
            {
                r = a0[0] * px + a0[1] * py + a0[2] * pz + a0[3];
                if (std::fabs(r) > rmax) continue;
            }
            else
            {
                const float vx = px - a0[0], vy = py - a0[1], vz = pz - a0[2];
                const float pr = vx * a0[3] + vy * a0[4] + vz * a0[5];
                const float ux = vx - pr * a0[3], uy = vy - pr * a0[4], uz = vz - pr * a0[5];
                r = std::sqrt(ux * ux + uy * uy + uz * uz);
                if (!(r > 1e-9f) || r > rmax) continue;
            }
            const float gx = w.orgx + ((float)i + 0.5f) * L - px;
            const float gy = w.orgy + ((float)j + 0.5f) * L - py;
            const float gz = w.orgz + ((float)k + 0.5f) * L - pz;
            const float d2 = gx * gx + gy * gy + gz * gz;
            if (d2 < best) { best = d2; bidx = id; bestR = r; }
        }
        if (bidx < 0) { ++out.nmiss_cell; continue; }

        const float *a = &w.tab[(size_t)bidx * 6];
        float co0, co1, co2, rr;
        if (mode == 0)
        {
            rr = bestR;                                  // 查询时已算过，不再重复
            const float s = 1.f - 0.9f * std::fabs(rr) /
                            std::sqrt(std::sqrt(px * px + py * py + pz * pz));
            if (!(s > 0.1f)) { ++out.nmiss_s; continue; }
            co0 = s * a[0]; co1 = s * a[1]; co2 = s * a[2];
            rr = s * rr;
        }
        else
        {
            const float ld = bestR;
            const float s = 1.f - 0.9f * ld;
            if (!(s > 0.1f)) { ++out.nmiss_s; continue; }
            const float vx = px - a[0], vy = py - a[1], vz = pz - a[2];
            const float pr = vx * a[3] + vy * a[4] + vz * a[5];
            const float wg = s / ld;
            co0 = wg * (vx - pr * a[3]); co1 = wg * (vy - pr * a[4]); co2 = wg * (vz - pr * a[5]);
            rr = s * ld;
        }
        out.sum_abs_r += std::fabs(rr);

        // J = [arz, arx, ary, coeff.z, coeff.x, coeff.y]
        const float pc0 = qy, pc1 = qz, pc2 = qx;
        const float cc0 = co1, cc1 = co2, cc2 = co0;      // = cL.x / cL.y / cL.z（LMO 换轴后）
        float J[6];
        J[0] = (M[0] * pc0 + M[1] * pc1 + M[2] * pc2) * cc0
             + (M[3] * pc0 + M[4] * pc1 + M[5] * pc2) * cc1
             + (M[6] * pc0 + M[7] * pc1 + M[8] * pc2) * cc2;      // arz
        J[1] = (M[9] * pc0 + M[10] * pc1 + M[11] * pc2) * cc0
             + (M[12] * pc0 + M[13] * pc1 + M[14] * pc2) * cc1
             + (M[15] * pc0 + M[16] * pc1 + M[17] * pc2) * cc2;   // arx
        J[2] = (M[18] * pc0 + M[19] * pc1 + M[20] * pc2) * cc0
             + (M[21] * pc0 + M[22] * pc1 + M[23] * pc2) * cc1
             + (M[24] * pc0 + M[25] * pc1 + M[26] * pc2) * cc2;   // ary
        J[3] = cc1; J[4] = cc2; J[5] = cc0;                       // coeff.z / .x / .y
        const float bv = -rr;

        for (int n = 0; n < 21; ++n) out.H[n] += J[HI27[n]] * J[HJ27[n]];
        for (int n = 0; n < 6; ++n)  out.b[n] += J[n] * bv;
        ++out.nhit;
    }
}

} // namespace pls2m
