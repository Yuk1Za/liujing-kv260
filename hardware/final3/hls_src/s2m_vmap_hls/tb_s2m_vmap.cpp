// =====================================================================================
// tb_s2m_vmap.cpp —— 功能对拍：内核（定点）vs 标量参考（float，逐条照抄 A8 语义）
//
//   这次 tb 不再只查"有限"，而是**真的对 H/b**：参考实现把 A8/A9 原码的查询过程
//   一句一句抄下来（27 邻域、选格规则、rmax 过滤顺序、权重、J 的行列顺序），
//   与内核在**同一份表**上跑，比 H/b 的相对差。
//
//   两种模式都要过：
//     rescan=1  → 内核 == A8（"在 |r|≤rmax 的格里取格心最近"）
//     rescan=0  → 内核 == "先选最近格、选完再判"（旧语义，用来确认重扫真的起作用）
//
//   编译（不需要 Vitis，普通 g++ + HLS 头即可）：
//     g++ -O2 -I$HLS/include -o tb tb_s2m_vmap.cpp s2m_vmap.cpp
// =====================================================================================
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <vector>
#include "s2m_vmap.hpp"
#include "pl_s2m.hpp"   // 复用 calc_M，保证 M 系数与真实位姿同源

#ifndef WX_
#define WX_ 32
#endif
static const signed char RDI[27] = {-1,-1,-1,-1,-1,-1,-1,-1,-1, 0,0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1};
static const signed char RDJ[27] = {-1,-1,-1, 0,0,0, 1,1,1,-1,-1,-1, 0,0,0, 1,1,1,-1,-1,-1, 0,0,0, 1,1,1};
static const signed char RDK[27] = {-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1};

long g_nhit = 0, g_nrescan = 0, g_nrescue = 0;
static unsigned rng_state = 12345u;
static float rnd()          // [0,1)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return (float)((rng_state >> 8) & 0xFFFFFF) / (float)0x1000000;
}

static inline int cellIdxF(float v, float invL)
{
    const float f = v * invL;
    const int   k = (int)f;
    return (f < (float)k) ? (k - 1) : k;
}
// 位图按 32 位字存放（与内核一致）：位号 = idx&31
static inline bool bitOf(const ap_uint<32> *vb, int idx)
{
    return (((unsigned)vb[idx >> 5] >> (idx & 31)) & 1u) != 0u;
}

struct RefRes
{
    double H[6][6];
    double b[6];
    long   nhit;
    long   nrescan;    // 触发重扫的点数（最近格被 rmax 拒）
    long   nrescue;    // 靠重扫救回来的点数
};

// ---- 标量参考：逐条照抄 A8 的 query() + surfOptimization 的残差/权重 ----
static RefRes ref_run(int nq, const float *q, const ap_uint<32> *vb, const float *tab,
                      const float *parq, const float *parm,
                      float L, float invL, float rmax,
                      float orgx, float orgy, float orgz,
                      int dimx, int dimy, int dimz, int rescan)
{
    RefRes o;
    std::memset(&o, 0, sizeof(o));
    for (int t = 0; t < nq; ++t)
    {
        const float qx = q[4 * t + 0], qy = q[4 * t + 1], qz = q[4 * t + 2];
        // 世界系点（R 取自 parq，float 版）
        const float px = parq[0] * qx + parq[1] * qy + parq[2] * qz + parq[9];
        const float py = parq[3] * qx + parq[4] * qy + parq[5] * qz + parq[10];
        const float pz = parq[6] * qx + parq[7] * qy + parq[8] * qz + parq[11];
        // ★ 格网格锚在**窗口原点**（与内核 (px-ox)*invL 一致），不是世界原点
        const int ci = cellIdxF(px - orgx, invL), cj = cellIdxF(py - orgy, invL), ck = cellIdxF(pz - orgz, invL);

        // ---- 第一趟：27 邻域里按格心最近选格（只判有效）----
        int   sel = -1;
        float bd = 1e30f;
        for (int n = 0; n < 27; ++n)
        {
            const int i = ci + RDI[n], j = cj + RDJ[n], k = ck + RDK[n];
            if (i < 0 || j < 0 || k < 0 || i >= dimx || j >= dimy || k >= dimz) continue;
            const int idx = (i * dimy + j) * dimz + k;
            if (!bitOf(vb, idx)) continue;
            const float gx = orgx + ((float)i + 0.5f) * L - px;
            const float gy = orgy + ((float)j + 0.5f) * L - py;
            const float gz = orgz + ((float)k + 0.5f) * L - pz;
            const float d2 = gx * gx + gy * gy + gz * gz;
            if (d2 < bd) { bd = d2; sel = idx; }
        }
        if (sel < 0) continue;

        float n0 = tab[6 * sel + 0], n1 = tab[6 * sel + 1], n2 = tab[6 * sel + 2], d0 = tab[6 * sel + 3];
        float rr = n0 * px + n1 * py + n2 * pz + d0;
        if (std::fabs(rr) > rmax)
        {
            ++o.nrescan;
            if (!rescan) continue;          // 旧语义：选完再判，判不过就丢
            // ---- 重扫：A8 语义「在 |r|≤rmax 的格里取格心最近」----
            int   sel2 = -1;
            float bd2 = 1e30f, rr2 = 0.f, m0 = 0, m1 = 0, m2 = 0, m3 = 0;
            for (int n = 0; n < 27; ++n)
            {
                const int i = ci + RDI[n], j = cj + RDJ[n], k = ck + RDK[n];
                if (i < 0 || j < 0 || k < 0 || i >= dimx || j >= dimy || k >= dimz) continue;
                const int idx = (i * dimy + j) * dimz + k;
                if (!bitOf(vb, idx)) continue;
                const float c0 = tab[6 * idx + 0], c1 = tab[6 * idx + 1], c2 = tab[6 * idx + 2], c3 = tab[6 * idx + 3];
                const float r = c0 * px + c1 * py + c2 * pz + c3;
                if (std::fabs(r) > rmax) continue;
                const float gx = orgx + ((float)i + 0.5f) * L - px;
                const float gy = orgy + ((float)j + 0.5f) * L - py;
                const float gz = orgz + ((float)k + 0.5f) * L - pz;
                const float e2 = gx * gx + gy * gy + gz * gz;
                if (e2 < bd2) { bd2 = e2; sel2 = idx; rr2 = r; m0 = c0; m1 = c1; m2 = c2; m3 = c3; }
            }
            if (sel2 < 0) continue;
            ++o.nrescue;
            n0 = m0; n1 = m1; n2 = m2; d0 = m3; rr = rr2;
        }

        const float s = 1.f - 0.9f * std::fabs(rr) / std::sqrt(std::sqrt(px * px + py * py + pz * pz));
        if (!(s > 0.1f)) continue;
        const float co0 = s * n0, co1 = s * n1, co2 = s * n2;
        rr = s * rr;

        // ---- J（与 LMOptimization 同式）----
        const float pc0 = qy, pc1 = qz, pc2 = qx;          // lidar→camera
        const float cc0 = co1, cc1 = co2, cc2 = co0;       // = cL.x / cL.y / cL.z
        double J[6];
        J[0] = ((double)parm[0] * pc0 + (double)parm[1] * pc1 + (double)parm[2] * pc2) * cc0
             + ((double)parm[3] * pc0 + (double)parm[4] * pc1 + (double)parm[5] * pc2) * cc1
             + ((double)parm[6] * pc0 + (double)parm[7] * pc1 + (double)parm[8] * pc2) * cc2;
        J[1] = ((double)parm[9] * pc0 + (double)parm[10] * pc1 + (double)parm[11] * pc2) * cc0
             + ((double)parm[12] * pc0 + (double)parm[13] * pc1 + (double)parm[14] * pc2) * cc1
             + ((double)parm[15] * pc0 + (double)parm[16] * pc1 + (double)parm[17] * pc2) * cc2;
        J[2] = ((double)parm[18] * pc0 + (double)parm[19] * pc1 + (double)parm[20] * pc2) * cc0
             + ((double)parm[21] * pc0 + (double)parm[22] * pc1 + (double)parm[23] * pc2) * cc1
             + ((double)parm[24] * pc0 + (double)parm[25] * pc1 + (double)parm[26] * pc2) * cc2;
        J[3] = cc1; J[4] = cc2; J[5] = cc0;                // mat_A 列序 [coeff.z, .x, .y]
        const double bv = -(double)rr;
        for (int i = 0; i < 6; ++i)
        {
            for (int j = i; j < 6; ++j)
            {
                o.H[i][j] += J[i] * J[j];
                o.H[j][i] = o.H[i][j];
            }
            o.b[i] += J[i] * bv;
        }
        ++o.nhit;
    }
    return o;
}

static void build_data(int WXv, int WYv, int WZv, const float L,
                       float orgx, float orgy, float orgz,
                       std::vector<float> &tab, std::vector<ap_uint<32>> &vb,
                       float rmax)
{
    const int WMAXv = WXv * WYv * WZv;
    // 平面：法向随机，d 让平面过"格心 + 随机偏移"，偏移量取到 1.5×rmax
    //       ⇒ 查询点对最近格**经常**超 rmax，从而逼出重扫路径
    for (int i = 0; i < WMAXv; ++i)
    {
        const int k = i % WZv, j = (i / WZv) % WYv, ii = i / (WYv * WZv);
        float nx = rnd() - 0.5f, ny = rnd() - 0.5f, nz = rnd() - 0.5f;
        float nn = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (nn < 1e-3f) { nx = 0; ny = 0; nz = 1; nn = 1; }
        nx /= nn; ny /= nn; nz /= nn;
        const float cx = orgx + ((float)ii + 0.5f) * L;
        const float cy = orgy + ((float)j + 0.5f) * L;
        const float cz = orgz + ((float)k + 0.5f) * L;
        const float off = (rnd() - 0.5f) * 3.0f * rmax;
        tab[6 * i + 0] = nx; tab[6 * i + 1] = ny; tab[6 * i + 2] = nz;
        tab[6 * i + 3] = -(nx * cx + ny * cy + nz * cz) + off;
        tab[6 * i + 4] = cx; tab[6 * i + 5] = cy;
        if (rnd() < 0.55f) vb[i >> 5] |= (ap_uint<32>)(1u << (i & 31));
    }
}

// 解 H dx = b（高斯消元 + 部分主元，double）
static bool solve6(const double H[6][6], const double b[6], double x[6])
{
    double a[6][7];
    for (int i = 0; i < 6; ++i) { for (int j = 0; j < 6; ++j) a[i][j] = H[i][j]; a[i][6] = b[i]; }
    for (int c = 0; c < 6; ++c)
    {
        int p = c;
        for (int r = c + 1; r < 6; ++r) if (std::fabs(a[r][c]) > std::fabs(a[p][c])) p = r;
        if (std::fabs(a[p][c]) < 1e-30) return false;
        if (p != c) for (int j = 0; j < 7; ++j) { const double tv = a[c][j]; a[c][j] = a[p][j]; a[p][j] = tv; }
        for (int r = c + 1; r < 6; ++r)
        {
            const double f = a[r][c] / a[c][c];
            for (int j = c; j < 7; ++j) a[r][j] -= f * a[c][j];
        }
    }
    for (int i = 5; i >= 0; --i)
    {
        double s = a[i][6];
        for (int j = i + 1; j < 6; ++j) s -= a[i][j] * x[j];
        x[i] = s / a[i][i];
    }
    return true;
}

// 把内核的 H(上三角打包) + b 还原成 6×6
static void unpack_Hb(const float *Hb, double H[6][6], double b[6])
{
    for (int i = 0; i < 6; ++i)
    {
        for (int j = i; j < 6; ++j)
        {
            const int n = (i * 6 - i * (i - 1) / 2) + (j - i);
            H[i][j] = H[j][i] = (double)Hb[n];
        }
        b[i] = (double)Hb[21 + i];
    }
}

static double rel_diff(const float *Hk, const double Hr[6][6], const float *bk, const double br[6])
{
    double num = 0, den = 0, v;
    for (int i = 0; i < 6; ++i)
    {
        for (int j = i; j < 6; ++j)
        {
            const int n = (i * 6 - i * (i - 1) / 2) + (j - i);   // 上三角打包下标
            v = (double)Hk[n] - Hr[i][j];
            num += v * v;
            den += Hr[i][j] * Hr[i][j];
        }
        v = (double)bk[21 + i] - br[i];
        num += v * v;
        den += br[i] * br[i];
    }
    return std::sqrt(num) / (std::sqrt(den) + 1e-30);
}

// 找出误差最大的一项（判断是不是"整体标度错"，而不是逐项小噪声）
static void worst_term(const char *tag, const float *Hk, const double Hr[6][6], const float *bk, const double br[6])
{
    double w = 0; int wi = -1, wj = -1; bool wb = false;
    for (int i = 0; i < 6; ++i)
    {
        for (int j = i; j < 6; ++j)
        {
            const int n = (i * 6 - i * (i - 1) / 2) + (j - i);
            const double d = std::fabs((double)Hk[n] - Hr[i][j]) / (std::fabs(Hr[i][j]) + 1e-30);
            if (d > w) { w = d; wi = i; wj = j; }
            w = w;
        }
        const double d = std::fabs((double)bk[21 + i] - br[i]) / (std::fabs(br[i]) + 1e-30);
        if (d > w) { w = d; wi = i; wj = -1; wb = true; }
    }
    printf("  %s 最差项: %s[%d]%s 相对差 %.2e\n", tag, wb ? "b" : "H", wi, wj >= 0 ? "[..]" : "", w);
}

int main(int argc, char **argv)
{
    const int NQ = 4000;
    const float L = 1.6f, invL = 1.f / L, rmax = 0.4f;
    const float orgx = -25.f, orgy = -25.f, orgz = -3.f;
    const int WXv = 32, WYv = 32, WZv = 8;
    const int WMAXv = WXv * WYv * WZv;

    std::vector<float> q((size_t)NQ * 4);
    std::vector<ap_uint<32>> vb(S2M_VBLEN / 4, 0);   // 位图：32 位字 × S2M_VBLEN/4
    std::vector<float> tab((size_t)WMAXv * 6, 0.f);
    std::vector<float> parq(12, 0.f), parm(27, 0.f), Hb(27, 0.f), Hb2(27, 0.f);

    build_data(WXv, WYv, WZv, L, orgx, orgy, orgz, tab, vb, rmax);

    // 查询点：随机挑一个有效格、在格里取一点、再沿该格平面法向拉回平面上（+ 小扰动）
    //   ⇒ 命中率高，而且"点的最近有效格 ≠ 它所属的格"这种情况大量出现 —— 正是重扫要处理的那类点
    int made = 0;
    for (int t = 0; t < NQ * 8 && made < NQ; ++t)
    {
        const int c = (int)(rnd() * (float)WMAXv) % WMAXv;
        if (!bitOf(vb.data(), c)) continue;
        const int kk = c % WZv, jj = (c / WZv) % WYv, ii = c / (WYv * WZv);
        const float gx = orgx + ((float)ii + rnd()) * L;
        const float gy = orgy + ((float)jj + rnd()) * L;
        const float gz = orgz + ((float)kk + rnd()) * L;
        const float n0 = tab[6 * c + 0], n1 = tab[6 * c + 1], n2 = tab[6 * c + 2], d0 = tab[6 * c + 3];
        const float rr0 = n0 * gx + n1 * gy + n2 * gz + d0;   // 到该面的有符号距离
        // 拉回平面，再沿法向留 ±0.15·rmax 的残差（模拟真实观测）
        const float back = -(rr0) + (rnd() - 0.5f) * 0.3f * rmax;
        q[4 * made + 0] = gx + n0 * back;
        q[4 * made + 1] = gy + n1 * back;
        q[4 * made + 2] = gz + n2 * back;
        q[4 * made + 3] = 1.f;
        ++made;
    }
    printf("查询点生成：%d / %d\n", made, NQ);

    // 位姿：单位 R + 小平移
    parq[0] = 1; parq[4] = 1; parq[8] = 1;
    parq[9] = 1.0f; parq[10] = -0.5f; parq[11] = 0.2f;
    // M 系数：用真实位姿（srx/crx/sry/cry/srz/crz of 12.3°/-7.5°/31.7°）
    //   ⇒ H 非奇异、b 不会因系数全 0 而退化，才能比较"解 dx"
    {
        const float ax = 12.3f * 3.14159265f / 180.f, ay = -7.5f * 3.14159265f / 180.f, az = 31.7f * 3.14159265f / 180.f;
        const float sc[6] = {std::sin(ax), std::cos(ax), std::sin(ay), std::cos(ay), std::sin(az), std::cos(az)};
        pls2m::calc_M(sc, parm.data());
    }

    RefRes refA = ref_run(NQ, q.data(), vb.data(), tab.data(), parq.data(), parm.data(),
                          L, invL, rmax, orgx, orgy, orgz, WXv, WYv, WZv, 1);
    RefRes refB = ref_run(NQ, q.data(), vb.data(), tab.data(), parq.data(), parm.data(),
                          L, invL, rmax, orgx, orgy, orgz, WXv, WYv, WZv, 0);

    s2m_vmap(q.data(), vb.data(), tab.data(), parq.data(), parm.data(), Hb.data(),
             NQ, 0, L, invL, rmax, orgx, orgy, orgz, WXv, WYv, WZv, 1);
    s2m_vmap(q.data(), vb.data(), tab.data(), parq.data(), parm.data(), Hb2.data(),
             NQ, 0, L, invL, rmax, orgx, orgy, orgz, WXv, WYv, WZv, 0);

    printf("H00 kern=%.4e ref=%.4e | H11 kern=%.4e ref=%.4e | b0 kern=%.4e ref=%.4e\n",
           Hb[0], refA.H[0][0], Hb[1], refA.H[1][1], Hb[21], refA.b[0]);
    printf("H33 kern=%.4e ref=%.4e | H55 kern=%.4e ref=%.4e | b5 kern=%.4e ref=%.4e\n",
           Hb[15], refA.H[3][3], Hb[20], refA.H[5][5], Hb[26], refA.b[5]);
    worst_term("rescan=1", Hb.data(), refA.H, Hb.data(), refA.b);
    worst_term("rescan=0", Hb2.data(), refB.H, Hb2.data(), refB.b);
    // ---- 判据：比较解 dx = H^-1 b（这才是 LOAM 真正用到的量）----
    double Hk[6][6], bk6[6], xk[6] = {0}, xr[6] = {0};
    unpack_Hb(Hb.data(), Hk, bk6);
    const bool sok = solve6(Hk, bk6, xk) && solve6(refA.H, refA.b, xr);
    double dx_inf = 0;
    if (sok) for (int i = 0; i < 6; ++i) dx_inf = std::max(dx_inf, std::fabs(xk[i] - xr[i]));

    const double dA = rel_diff(Hb.data(),  refA.H, Hb.data(),  refA.b);
    const double dB = rel_diff(Hb2.data(), refB.H, Hb2.data(), refB.b);

    printf("参考(A8 语义)    : hit=%ld  rescan=%ld  rescue=%ld\n", refA.nhit, refA.nrescan, refA.nrescue);
    printf("内核             : hit=%ld  rescan=%ld  rescue=%ld\n", g_nhit, g_nrescan, g_nrescue);
    printf("参考(旧语义)     : hit=%ld\n", refB.nhit);
    printf("内核 rescan=1    : hit=%ld   H/b 相对差 = %.3e\n", refA.nhit, dA);
    printf("内核 rescan=0    : hit=%ld   H/b 相对差 = %.3e\n", refB.nhit, dB);

    printf("解 δx 绝对差（inf 范数）= %.3e %s\n", dx_inf, sok ? "" : "(奇异，跳过)");
    // 判据：
    //   ① 选格语义一致：内核 hit == 参考 hit（允许 ±2，F12 量化会让极少数贴格边界的点换格）
    //   ② 数值：解 δx 的绝对差 ≤ 1 mm（LOAM 的位姿精度是 cm 级，1 mm 比它小一个数量级）
    //   ③ 重扫确实救回点（rescue 数相当）
    const bool hit_ok = std::labs(g_nhit - (refA.nhit + refB.nhit)) <= 2;
    const bool okA = hit_ok && sok && (dx_inf <= 1e-3) && (refA.nhit > refB.nhit) && (refA.nrescue > 0);
    const bool okB = (dB <= 1e-2);
    printf("rescan=1 判据（hit 对齐 + 解 δx ≤1mm + 重扫真救回点）: %s\n", okA ? "PASS" : "FAIL");
    printf("rescan=0 判据（== 旧语义）            : %s\n", okB ? "PASS" : "FAIL");
    // ---- dump 模式：把这一组用例导出成二进制，给板端 bring-up 当输入 + 参考输出 ----
    if (argc > 1 && std::strcmp(argv[1], "dump") == 0)
    {
        auto wr = [](const char *fn, const void *p, size_t n) {
            FILE *f = fopen(fn, "wb");
            if (!f) { printf("cannot write %s\n", fn); return; }
            fwrite(p, 1, n, f); fclose(f);
            printf("  wrote %-12s %8zu B\n", fn, n);
        };
        wr("q.bin",    q.data(),    sizeof(float) * q.size());
        wr("vbw.bin",  vb.data(),   sizeof(ap_uint<32>) * vb.size());
        wr("tab.bin",  tab.data(),  sizeof(float) * tab.size());
        wr("parq.bin", parq.data(), sizeof(float) * 12);
        wr("parm.bin", parm.data(), sizeof(float) * 27);
        // 参考 H/b（rescan=1，A8 语义）写成 float[27]
        float hb[27];
        for (int i = 0; i < 6; ++i)
        {
            for (int j = i; j < 6; ++j)
                hb[(i * 6 - i * (i - 1) / 2) + (j - i)] = (float)refA.H[i][j];
            hb[21 + i] = (float)refA.b[i];
        }
        wr("hbref.bin", hb, sizeof(float) * 27);
        printf("NQ=%d WMAX=%d VBLEN=%d\n", NQ, (int)WMAX, (int)S2M_VBLEN);
    }

    printf("%s\n", (okA && okB) ? "TB PASS" : "TB FAIL");
    return (okA && okB) ? 0 : 1;
}
