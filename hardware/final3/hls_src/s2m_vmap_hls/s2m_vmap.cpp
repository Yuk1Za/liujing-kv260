// =====================================================================================
// s2m_vmap.cpp —— scan-to-map 残差块的 PL 内核（v1：两趟 DATAFLOW + 串行 MAC）
//
// v0b 的教训（csynth 实测）：单趟结构里"扫完 27 格才发那一次图元读"，那次 m_axi 往返无法与
//   计算重叠 ⇒ 每点 120–252 拍、并行累加器 21+6 个浮点加法器 ⇒ 19.3k LUT（折算实现 29–44k）。
//
// v1 的两处改动：
//   ① **两趟**（`DATAFLOW`）：stage-1 只做"27 邻域扫描 + 选最近有效格"（27 拍/点，只读 BRAM 位图），
//      结果经 stream 交给 stage-2；stage-2 做"读图元（1 次 m_axi，可流水）+ 残差 + 权重 + 累加"。
//      两趟重叠 ⇒ 吞吐 = max(27, eval) ≈ 35–40 拍/点（对 v0b 的 120–252 是 3–6×）。
//   ② **串行 MAC**：H 的 21 项 + b 的 6 项按固定顺序逐个更新（Hubb[n] += …，n 即循环变量），
//      把 27 个并行浮点加法器压成 1 个 ⇒ csynth LUT 从 19.3k 降到个位数 k。
//
// v2 / E 路线（FIXED_ACC=1，默认）：**J 与 H/b 全定点**。
//   动机（v1 csynth 实证）：HLS 里每个 float 运算都是独立 IP（≈200–350 LUT + 2 DSP），
//   27 次 mul + 27 次 add 的 MAC 光加法器就要 1 万 LUT；定点乘加是 1–2 个 DSP、几乎无 LUT。
//   残差 rr / 权重 s 仍留 float（sqrt、除法、rmax 门限都在这一层，精度敏感）。
//
// 与原码的对应：残差/权重/J 三部分与 `cornerOptimization` / `surfOptimization` 同式，
//   `LMOptimization` 的 mat_A 行序（含 lidar→camera 换轴、原码 ary 缺项）照抄。
// =====================================================================================
#ifndef FORCE_PLANE
#define FORCE_PLANE 0
#endif
// FORCE_NORS=1：把重扫分支整段编译掉 —— 只用来量"不含重扫"的每点周期基线
#ifndef FORCE_NORS
#define FORCE_NORS 0
#endif
// NOWEIGHT=1 只是**减法实验**：把权重 s 定成常数，用来量 "2 次 sqrt + 1 次 div" 到底吃多少 LUT。
// 不能作为最终设计（精度会变）。
#ifndef NOWEIGHT
#define NOWEIGHT 0
#endif
#ifndef FIXED_ACC
#define FIXED_ACC 1
#endif
#include "s2m_vmap.hpp"
#include <hls_math.h>
#include <hls_stream.h>
#include <cstdint>
#ifndef __SYNTHESIS__
// 纯 g++ 功能仿真（不综合）时 hls::fabs/sqrt 没有定义 ⇒ 用 std 顶上。
// 综合时走 hls_math.h 的真 IP，这段不参与。
#include <cmath>
namespace hls {
inline float fabs(float x) { return ::fabsf(x); }
inline float sqrt(float x) { return ::sqrtf(x); }
}
#endif
#include <ap_int.h>
// =====================================================================================
// 定点格式（★ 用 ap_int + 显式标度，不用 ap_fixed）
//   原因：ap_fixed 的隐式定标让 HLS 把乘法综合成 **LUT 乘法器**（18×18 ≈ 324 LUT），
//        实测 LUT 25.5k 几乎没降；ap_int 的乘法才走 DSP48。
//   位宽铁律：DSP48E2 = 27×18。**乘法操作数全部 ≤18 位** ⇒ 1 个 DSP、无拼接逻辑；
//        26×26 会拆 2 个 DSP + 52 位拼接加法器（实测比纯 float 还贵）。
//   J 的旋转列（≤|p|≈200）与平移列（≤1）量级差 200 倍，**必须分两种标度**。
// =====================================================================================
typedef ap_int<18> rT;     // 旋转矩阵元素   × 2^-16  (≤1)
typedef ap_int<18> mT;     // M 系数          × 2^-16  (±2,    分辨率 1.53e-5)
// ★ 坐标不必挤进 18 位：它只和 M(18 位) 相乘，走 DSP48 的 **A 口(27 位)** 仍是 1 个 DSP。
typedef ap_int<24> pcT;    // 世界系点坐标     × 2^-12  (±2048, 分辨率 2.44e-4)
typedef ap_int<26> pT;     // 行点积 M_row·p  × 2^-14  (±2048)
typedef ap_int<18> jaT;    // J 旋转列         × 2^-7   (±1024, 分辨率 7.8e-3)
typedef ap_int<18> jcT;    // J 平移列/coeff/bv × 2^-16  (±2)
// ★ 累加器定标取 2^-32 = 三类乘积中最细的那个（cc 块），**所有项都靠左移对齐**：
//   算术右移是向 -inf 取整，2000 个点同向截断会累积成 DC 偏置（实测 b 的相对差被推到 9e-4）。
//   左移是无损的（只扩位宽），所以这里一个 bit 都不丢。
typedef ap_int<64> accT;   // H / b 累加       × 2^-32  (±9.2e18, 分辨率 2.3e-10)

// stage-1 → stage-2 的载体（坐标已是定点 F12，stage-2 不用再转）
struct Pkt
{
    ap_int<32>  px, py, pz;    // 世界系坐标 × 2^-12（残差 rr = n·p + d 用）
    ap_int<24>  qx, qy, qz;    // 传感器系坐标 × 2^-12（J = M·p_cam·coeff 用）
                               // ★ 原码的 arx/ary/arz 用的是 pointOri（传感器系），
                               //   不是 map 系点 —— 两者混用会让 J 完全错。
    // ★ 不用 ci/cj/ck 而是直接传"本格平铺下标 + 格心相对点的偏移"：
    //   stage-2 的重扫前导原来要做 (ci*dimy+cj)*dimz+ck 两个 32 位乘法 + 3 个 ci*Lf，
    //   实测这是 impl 后的最差路径（ap_CS_fsm → idx0_reg，两级 DSP 串联）。
    ap_int<20>  i0;            // 本格在窗口里的平铺下标
    ap_int<16>  dx, dy, dz;    // 本格格心 − 查询点（F12，|·| ≤ 0.87L）
    ap_uint<5>  sel;           // 选中的格在 27 邻域里的序号（0..26）⇒ 下标 = i0 + 偏移
    ap_uint<1>  found;         // 27 邻域里是否有至少一个有效格
    ap_uint<27> vmask;         // 27 邻域的有效位（重扫时免读位图 / 免跨模块访问 BRAM）
};

// 27 邻域的偏移只有 -1/0/+1 ⇒ "偏移 × 步长"用一个三选一（mux）实现，别让 HLS 生成乘法器。
//   实测：每格 3 个乘法（地址 2 个 + 格心 1 个）挤在 27 拍的链上，是 P&R 后时序的最差路径。
static inline int sel3i(int d, int m) { return (d > 0) ? m : ((d < 0) ? -m : 0); }

// ---- 27 邻域偏移 ROM（81 B）----
static const signed char DI[27] = {-1,-1,-1,-1,-1,-1,-1,-1,-1, 0,0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1};
static const signed char DJ[27] = {-1,-1,-1, 0,0,0, 1,1,1,-1,-1,-1, 0,0,0, 1,1,1,-1,-1,-1, 0,0,0, 1,1,1};
static const signed char DK[27] = {-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1};
// H 上三角的分块下标（aa 6 项 / cc 6 项；ac 的 9 项用 (n-6)/3 与 (n-6)%3 直接算）
static const signed char AA_I[6] = {0, 0, 0, 1, 1, 2};
static const signed char AA_J[6] = {0, 1, 2, 1, 2, 2};
#define NACC 27
#ifndef __SYNTHESIS__
// 功能仿真用的计数（综合时不参与）
extern long g_nhit;
extern long g_nrescan;
extern long g_nrescue;
#endif
// H 上三角 21 项的行列（00..05, 11..15, 22..25, 33..35, 44,45, 55）+ b 的 6 项
static const signed char HI[NACC] = {0,0,0,0,0,0, 1,1,1,1,1, 2,2,2,2, 3,3,3, 4,4, 5, 0,1,2,3,4,5};
static const signed char HJ[NACC] = {0,1,2,3,4,5, 1,2,3,4,5, 2,3,4,5, 3,4,5, 4,5, 5, 0,0,0,0,0,0};
static const signed char AC_I[9] = {0, 0, 0, 1, 1, 1, 2, 2, 2};
static const signed char AC_J[9] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
static const signed char CC_I[6] = {0, 0, 0, 1, 1, 2};
static const signed char CC_J[6] = {0, 1, 2, 1, 2, 2};

static inline int cellIdx(float v, float invL)
{
    const float f = v * invL;
    const int   k = (int)f;
    return (f < (float)k) ? (k - 1) : k;
}

// =====================================================================================
// stage-1：每点 27 拍扫描（不访存，只读 BRAM 位图）—— 全定点
//   原来这一段的 9 个 fmul + 6 个 fadd + 3 个 f2i + 距离比较是 **7.8k LUT**，
//   换成定点后只是几个整数乘加（DSP + 进位链），坐标精度 2^-12 = 0.24 mm。
// =====================================================================================
static void scan_points(const float        *q,
                        const ap_uint<32>  *vbw,     // DDR 里的位图（m_axi），先整体拷进内部 BRAM
                        const float        *parq,     // [12] R(9)|t(3)
                        hls::stream<Pkt>   &s,
                        int nq, int invL16,
                        int ox12, int oy12, int oz12, int L12,
                        int dimx, int dimy, int dimz)
{
#pragma HLS INLINE off
    // ★ 位图先整体拷进内部 BRAM：8192 B / 4 = 2048 拍 ≈ 14 µs（占单帧 2.3 ms 的 0.6%）。
    //   换来的是扫描循环里的一次内部 BRAM 读（II=1、无 AXI 延迟），
    //   而且 BD 里不需要 axi_bram_ctrl + 真双口 RAM（HLS 的 ap_memory 会给 2 个口）。
    ap_uint<32> vbuf[S2M_VBLEN / 4];
LDB:
    for (int i = 0; i < S2M_VBLEN / 4; ++i)
    {
#pragma HLS PIPELINE II=1
        vbuf[i] = vbw[i];
    }
    rT R[9];
    ap_int<32> T[3];
#pragma HLS ARRAY_PARTITION variable=R complete dim=1
#pragma HLS ARRAY_PARTITION variable=T complete dim=1
LDP:
    for (int i = 0; i < 12; ++i)
    {
#pragma HLS PIPELINE II=1
        if (i < 9) R[i] = (rT)(parq[i] * 65536.0f);        // F16
        else       T[i - 9] = (ap_int<32>)(parq[i] * 4096.0f);   // F12
    }
    // 定标常量由顶层统一算好再传进来（float→int 转换器只实例化 4 个，两趟共用）
    const ap_int<32> ox = (ap_int<32>)ox12;
    const ap_int<32> oy = (ap_int<32>)oy12;
    const ap_int<32> oz = (ap_int<32>)oz12;
    const ap_int<24> Lf    = (ap_int<24>)L12;                 // F12
    const ap_int<24> invLf = (ap_int<24>)invL16;              // F16

SCAN:
    for (int t = 0; t < nq; ++t)
    {
#pragma HLS LOOP_TRIPCOUNT min=1 max=4096 avg=2000
        ap_int<24> qv[3];
#pragma HLS ARRAY_PARTITION variable=qv complete dim=1
LDQ:
        for (int k = 0; k < 4; ++k)          // 读 4 个保住 16 B burst
        {
#pragma HLS PIPELINE II=1
            const float v = q[4 * t + k];
            if (k < 3) qv[k] = (ap_int<24>)(v * 4096.0f);     // F12
        }
        // R(F16)·q(F12) = F28 ⇒ >>16 得 F12，再 + T(F12)
        const ap_int<32> px = (ap_int<32>)(((ap_int<44>)(R[0] * qv[0]) + (ap_int<44>)(R[1] * qv[1])
                                          + (ap_int<44>)(R[2] * qv[2])) >> 16) + T[0];
        const ap_int<32> py = (ap_int<32>)(((ap_int<44>)(R[3] * qv[0]) + (ap_int<44>)(R[4] * qv[1])
                                          + (ap_int<44>)(R[5] * qv[2])) >> 16) + T[1];
        const ap_int<32> pz = (ap_int<32>)(((ap_int<44>)(R[6] * qv[0]) + (ap_int<44>)(R[7] * qv[1])
                                          + (ap_int<44>)(R[8] * qv[2])) >> 16) + T[2];
        // cellIdx：F12 × F16 = F28 ⇒ 算术右移 28 = floor（负号也向 -inf，与原码 floor 一致）
        const int ci = (int)(((ap_int<56>)(px - ox) * (ap_int<56>)invLf) >> 28);
        const int cj = (int)(((ap_int<56>)(py - oy) * (ap_int<56>)invLf) >> 28);
        const int ck = (int)(((ap_int<56>)(pz - oz) * (ap_int<56>)invLf) >> 28);

        int   sel = 0;
        // ★ 距离用 32 位就够：|g| ≤ 2L = 3.2 m（F12 ⇒ 13107，14 位），3·g² ≤ 5.2e8 < 2^31。
        //   原来用 ap_int<40> ⇒ 40 位加法链 + 40 位比较器，是时序最差路径的一部分。
        //   （也别用 (ap_int<N>)1<<(N-1) 当"无穷大"：那会溢出成负数，见 7.6.4 bug#1。）
        ap_int<32> best = (ap_int<32>)0x7FFFFFFF;
        ap_uint<27> vm = 0;
        bool  found = false;
        // ★ 27 个邻格的地址与格心都是"本格 + 固定偏移" ⇒ 每点只算一次基准，循环里只做选择/加减。
        //   原来每格要做 i*dimy、*dimz、(2i+1)*Lf 三个乘法 ⇒ 3 个乘法器 × 27 拍挤在同一条链上，
        //   实测就是 P&R 后时序的最差路径（6.874 ns，其中 3.06 ns 是布线）。
        const int sdi = dimy * dimz;                 // i 方向的下标步长
        const int sdj = dimz;                        // j 方向的步长（k 方向是 1）
        const int idx0 = (ci * dimy + cj) * dimz + ck;
        const int hLf  = (int)Lf >> 1;               // L/2（F12）；(2i+1)·Lf>>1 == i·Lf + Lf>>1，整数上恒等
        const ap_int<32> dx0 = (ap_int<32>)(ox + (ap_int<32>)(ci * (int)Lf + hLf)) - px;
        const ap_int<32> dy0 = (ap_int<32>)(oy + (ap_int<32>)(cj * (int)Lf + hLf)) - py;
        const ap_int<32> dz0 = (ap_int<32>)(oz + (ap_int<32>)(ck * (int)Lf + hLf)) - pz;
NB:
        for (int n = 0; n < NCELL; ++n)
        {
#pragma HLS PIPELINE II=1
            const int i = ci + DI[n], j = cj + DJ[n], k = ck + DK[n];
            if (i < 0 || j < 0 || k < 0 || i >= dimx || j >= dimy || k >= dimz) continue;
            const int idx = idx0 + sel3i(DI[n], sdi) + sel3i(DJ[n], sdj) + DK[n];
            const ap_uint<32> vbw32 = vbuf[idx >> 5];        // 32 位字，位号 = idx&31
            if (((vbw32 >> (idx & 31)) & 1) == 0) continue;
            vm[n] = 1;                      // 有效位（给 stage-2 的重扫用）
            // 格心与点之差 = 基准差 + 固定偏移（±L），F12；|g| ≤ 2L ⇒ 16 位（±8 m）够
            const ap_int<16> gx = (ap_int<16>)(dx0 + sel3i(DI[n], (int)Lf));
            const ap_int<16> gy = (ap_int<16>)(dy0 + sel3i(DJ[n], (int)Lf));
            const ap_int<16> gz = (ap_int<16>)(dz0 + sel3i(DK[n], (int)Lf));
            const ap_int<32> d2 = (ap_int<32>)(gx * gx) + (ap_int<32>)(gy * gy) + (ap_int<32>)(gz * gz);  // F24
            if (d2 < best) { best = d2; sel = n; found = true; }
        }
        Pkt pkt;
        pkt.px = px; pkt.py = py; pkt.pz = pz;
        pkt.qx = qv[0]; pkt.qy = qv[1]; pkt.qz = qv[2];
        pkt.i0 = (ap_int<20>)idx0;
        pkt.dx = (ap_int<16>)dx0; pkt.dy = (ap_int<16>)dy0; pkt.dz = (ap_int<16>)dz0;
        pkt.sel = (ap_uint<5>)sel;
        pkt.vmask = vm;
        pkt.found = found ? 1 : 0;
        s.write(pkt);
    }
}

// =====================================================================================
// stage-2：读图元（1 次 m_axi）+ 残差 + 串行 MAC
// =====================================================================================
static void eval_points(hls::stream<Pkt> &s,
                        const float      *tab,
                        const float      *parm,   // [27] J 旋转列的 3×9 系数
                        float            *Hb,
                        int nq, int mode, float rmax,
                        int dimx, int dimy, int dimz,
                        int rescan, int ox12, int oy12, int oz12, int L12)
{
#pragma HLS INLINE off
    // M 系数在入口就量化成定点（循环里只有 1 个 float→fixed 转换器，27 拍读完）
    mT M[27];
#pragma HLS ARRAY_PARTITION variable=M complete dim=1
LDM:
    for (int i = 0; i < 27; ++i)
    {
#pragma HLS PIPELINE II=1
        M[i] = (mT)(parm[i] * 65536.0f);   // ×2^16
    }

    // H/b 累加表放 BRAM（27 x 64 bit = 1 个 18K BRAM），不分区
    accT Hbram[NACC];
#pragma HLS ARRAY_PARTITION variable=Hbram dim=1 off
#pragma HLS RESOURCE variable=Hbram core=RAM_S2P_BRAM
INIT:
    for (int i = 0; i < NACC; ++i)
    {
#pragma HLS PIPELINE II=1
        Hbram[i] = (accT)0;
    }

EVAL:
    for (int t = 0; t < nq; ++t)
    {
#pragma HLS LOOP_TRIPCOUNT min=1 max=4096 avg=2000
        const Pkt pkt = s.read();
        if (pkt.found == 0) continue;
        // stage-1 只回传"选中第几个邻格"（5 bit）⇒ 这里还原成格下标
        const int sel  = (int)pkt.sel;
        const int sdi0 = dimy * dimz, sdj0 = dimz;
        const int idx  = (int)pkt.i0 + sel3i(DI[sel], sdi0) + sel3i(DJ[sel], sdj0) + DK[sel];
        // Pkt 里的坐标是定点 F12：残差/权重仍要 float（sqrt、除法），转 3 次；
        // 但 J 用的 pc0/pc1/pc2 直接拿定点值 ⇒ **省掉 3 个 float→fixed 转换器**
        const float px = (float)pkt.px * 2.44140625e-4f;
        const float py = (float)pkt.py * 2.44140625e-4f;
        const float pz = (float)pkt.pz * 2.44140625e-4f;

        // ---- 读选中图元（6 个 float，m_axi burst；平面只用前 4 个）----
        float a[6];
#pragma HLS ARRAY_PARTITION variable=a complete dim=1
RD:
        for (int k = 0; k < 6; ++k)
        {
#pragma HLS PIPELINE II=1
            a[k] = tab[6 * idx + k];
        }

        // ---- 残差 + 权重（与 CPU 同式）----
        // FORCE_PLANE=1 时把直线分支编译掉（用来量"双几何"到底占多少 LUT）
        float co0, co1, co2, rr;
#if FORCE_PLANE
        {
#else
        if (mode == 0)
        {
#endif
            rr = a[0] * px + a[1] * py + a[2] * pz + a[3];
            // ---- 方案 D：被 rmax 拒的点，重扫 27 邻域 ----
            //   A8 的 query() 在 27 邻域**循环里**就判 |r|>rmax（即"在合格的格里取格心最近"）；
            //   stage-1 只读位图、看不到残差，只能按格心最近选格，选完再判就会丢掉
            //   "最近格超限、但邻格有合格平面"的点。实测这类点占 8.8%，丢掉它末端误差 ×6。
            //   这里补一趟：只对失败的点扫这 27 格，取 |r|≤rmax 中格心最近的那个。
#if FORCE_NORS
            if (false)
#else
            if (rescan != 0 && hls::fabs(rr) > rmax)
#endif
            {
                const ap_int<24> Lf = (ap_int<24>)L12;   // 只用来做 ±L 的三选一
                // ★ 基准地址/格心偏移都由 stage-1 直接传来 ⇒ 这里**一个乘法都不用**
                const int sdi = dimy * dimz, sdj = dimz;
                const int idx0 = (int)pkt.i0;
                const ap_int<32> dx0 = (ap_int<32>)pkt.dx;
                const ap_int<32> dy0 = (ap_int<32>)pkt.dy;
                const ap_int<32> dz0 = (ap_int<32>)pkt.dz;
                // ★ 循环里**只保留 bd 与赢家序号**（5 bit 的 mux），不在流水里选 4 个 float：
                //   原来 `if(d2<bd){ bd=d2; a[0..3]=c4; rr=r2; }` 等于在 27 次迭代上做 160 bit 的
                //   2:1 选择，实测是整条时序的最差路径（`gx_reg → select_ln303`，6.874 ns / 16 级）。
                ap_int<32> bd = (ap_int<32>)0x7FFFFFFF;
                int  sel2 = 0;
                bool f2   = false;
RS:
                for (int n = 0; n < NCELL; ++n)
                {
#pragma HLS PIPELINE II=4
                    // 边界与有效性都已在 stage-1 压进 vmask ⇒ 这里只判 1 bit，连 i/j/k 都不用算
                    if (pkt.vmask[n] == 0) continue;
                    const int idx2 = idx0 + sel3i(DI[n], sdi) + sel3i(DJ[n], sdj) + DK[n];
                    float c4[4];
#pragma HLS ARRAY_PARTITION variable=c4 complete dim=1
                    for (int q2 = 0; q2 < 4; ++q2) c4[q2] = tab[6 * idx2 + q2];
                    const float r2 = c4[0] * px + c4[1] * py + c4[2] * pz + c4[3];
                    if (hls::fabs(r2) > rmax) continue;
                    const ap_int<16> gx = (ap_int<16>)(dx0 + sel3i(DI[n], (int)Lf));
                    const ap_int<16> gy = (ap_int<16>)(dy0 + sel3i(DJ[n], (int)Lf));
                    const ap_int<16> gz = (ap_int<16>)(dz0 + sel3i(DK[n], (int)Lf));
                    const ap_int<32> d2 = (ap_int<32>)(gx * gx) + (ap_int<32>)(gy * gy) + (ap_int<32>)(gz * gz);
                    if (d2 < bd) { bd = d2; sel2 = n; f2 = true; }
                }
#ifndef __SYNTHESIS__
                ++g_nrescan;
                if (f2) ++g_nrescue;
#endif
                if (!f2) continue;     // 重扫也没有合格平面 ⇒ 这个点本来就不该有观测量
                // 赢家只在 27 格里 ⇒ 循环外再读一次（每帧只多 0.088×24 B，可忽略）
                {
                    const int idx2 = idx0 + sel3i(DI[sel2], sdi) + sel3i(DJ[sel2], sdj) + DK[sel2];
                    float c4[4];
#pragma HLS ARRAY_PARTITION variable=c4 complete dim=1
RDR:
                    for (int q2 = 0; q2 < 4; ++q2)
                    {
#pragma HLS PIPELINE II=1
                        c4[q2] = tab[6 * idx2 + q2];
                    }
                    a[0] = c4[0]; a[1] = c4[1]; a[2] = c4[2]; a[3] = c4[3];
                    rr = c4[0] * px + c4[1] * py + c4[2] * pz + c4[3];
                }
            }
            if (hls::fabs(rr) > rmax) continue;
#if NOWEIGHT
            const float s = 0.5f;
#else
            const float s = 1.f - 0.9f * hls::fabs(rr) /
                            hls::sqrt(hls::sqrt(px * px + py * py + pz * pz));
#endif
            if (!(s > 0.1f)) continue;
            co0 = s * a[0]; co1 = s * a[1]; co2 = s * a[2];
            rr  = s * rr;
        }
#if !FORCE_PLANE
        else
        {
            const float vx = px - a[0], vy = py - a[1], vz = pz - a[2];
            const float pr = vx * a[3] + vy * a[4] + vz * a[5];
            const float ux = vx - pr * a[3], uy = vy - pr * a[4], uz = vz - pr * a[5];
            const float ld = hls::sqrt(ux * ux + uy * uy + uz * uz);
            if (!(ld > 1e-9f) || ld > rmax) continue;
#if NOWEIGHT
            const float s = 0.5f;
#else
            const float s = 1.f - 0.9f * ld;
#endif
            if (!(s > 0.1f)) continue;
            const float w = s / ld;   // 只除一次（写成 s*ux/ld 三次会实例化 3 个 fdiv）
            co0 = w * ux; co1 = w * uy; co2 = w * uz;
            rr  = s * ld;
        }
#endif

        // ---- J：旋转列 = (M_row · p_cam) · coeff；平移列 = coeff 换轴 ----
        // 标度链：M(2^-16)·p(2^-12) = 2^-28 →右移14→ pT(2^-14)
        //         pT(2^-14)·cc(2^-16) = 2^-30 →右移23→ jaT(2^-7)
        // J 的输入是传感器系点（原码 pointOri），换轴 (y,z,x) 与原码 lidar->camera 一致
        const pcT pc0 = (pcT)pkt.qy, pc1 = (pcT)pkt.qz, pc2 = (pcT)pkt.qx;
        const jcT cc0 = (jcT)(co1 * 65536.0f), cc1 = (jcT)(co2 * 65536.0f), cc2 = (jcT)(co0 * 65536.0f);
        jaT Ja[3];
        jcT Jc[3];
#pragma HLS ARRAY_PARTITION variable=Ja complete dim=1
#pragma HLS ARRAY_PARTITION variable=Jc complete dim=1
        {
            const pT r00 = (pT)((M[0] * pc0) >> 14) + (pT)((M[1] * pc1) >> 14) + (pT)((M[2] * pc2) >> 14);
            const pT r01 = (pT)((M[3] * pc0) >> 14) + (pT)((M[4] * pc1) >> 14) + (pT)((M[5] * pc2) >> 14);
            const pT r02 = (pT)((M[6] * pc0) >> 14) + (pT)((M[7] * pc1) >> 14) + (pT)((M[8] * pc2) >> 14);
            const pT r10 = (pT)((M[9] * pc0) >> 14) + (pT)((M[10] * pc1) >> 14) + (pT)((M[11] * pc2) >> 14);
            const pT r11 = (pT)((M[12] * pc0) >> 14) + (pT)((M[13] * pc1) >> 14) + (pT)((M[14] * pc2) >> 14);
            const pT r12 = (pT)((M[15] * pc0) >> 14) + (pT)((M[16] * pc1) >> 14) + (pT)((M[17] * pc2) >> 14);
            const pT r20 = (pT)((M[18] * pc0) >> 14) + (pT)((M[19] * pc1) >> 14) + (pT)((M[20] * pc2) >> 14);
            const pT r21 = (pT)((M[21] * pc0) >> 14) + (pT)((M[22] * pc1) >> 14) + (pT)((M[23] * pc2) >> 14);
            const pT r22 = (pT)((M[24] * pc0) >> 14) + (pT)((M[25] * pc1) >> 14) + (pT)((M[26] * pc2) >> 14);
            // 先加后移（只截断一次，比三次分别截断少 1 bit 误差、少 3 个移位器）
            Ja[0] = (jaT)((r00 * cc0 + r01 * cc1 + r02 * cc2) >> 23);   // arz
            Ja[1] = (jaT)((r10 * cc0 + r11 * cc1 + r12 * cc2) >> 23);   // arx
            Ja[2] = (jaT)((r20 * cc0 + r21 * cc1 + r22 * cc2) >> 23);   // ary
        }
        // ★ 原码 mat_A 的第 4~6 列是 [coeff.z, coeff.x, coeff.y]（不是 x,y,z）：
        //   cc0=coeff.y、cc1=coeff.z、cc2=coeff.x ⇒ 取 [cc1, cc2, cc0]
        Jc[0] = cc1; Jc[1] = cc2; Jc[2] = cc0;
        const jcT bv = (jcT)(-rr * 65536.0f);
#ifndef __SYNTHESIS__
        ++g_nhit;
#endif

        // ★ 统一标度到 2^-7：J6 前 3 个是旋转列(已是 2^-7)，后 3 个把 coeff(2^-16) 右移 9 位。
        //   这样 27 项的乘积标度一致，串行 MAC 里就不需要按块分支了。
        ap_int<27> J6[6];
#pragma HLS ARRAY_PARTITION variable=J6 complete dim=1
        J6[0] = (ap_int<27>)Ja[0]; J6[1] = (ap_int<27>)Ja[1]; J6[2] = (ap_int<27>)Ja[2];
        // ★ 2^-16 → 2^-7 是**右移 9**（左移会差 2^18 倍）。Jc 最大 2^16 ⇒ >>9 后 2^7，放得进 18 位。
        J6[3] = (ap_int<27>)Jc[0] >> 9; J6[4] = (ap_int<27>)Jc[1] >> 9; J6[5] = (ap_int<27>)Jc[2] >> 9;

        // ---- MAC：分三块（标度不同 ⇒ 移位量不同），索引全编译期常量 ⇒ 无 mux ----
        //   Ja(2^-7)²          = 2^-14 ⇒ 左移 18 → 2^-32
        //   Ja(2^-7)·Jc(2^-16) = 2^-23 ⇒ 左移  9 → 2^-32
        //   Jc(2^-16)²         = 2^-32 ⇒ 不动      → 2^-32
        //   ★ 这里的 18/9 是由 jaT 的小数位(7) 决定的，改标度必须同步改 —— 漏改会整体差 2^n 倍。
        // 上三角 21 项与原码 mat_A 行序一致：00..05, 11..15, 22..25, 33..35, 44,45, 55
        // ---- 串行 MAC：每点 27 拍，1 个乘法器 + BRAM 累加表 ----
        //   原先是 27 个乘法器 + 27 个 64 位加法器并行（DSP glue + 寄存器约 4.9k LUT）。
        //   ★ b 不能跟着 J6 走 2^-7：rr 典型只有 1e-3~1e-5 m，bv=-rr 量化到 2^-7（7.8e-3）后
        //     整个 b 会被截成 0（实测最小用例 b3=0）。所以 b 单独一个循环，保持 2^-16×2^-16=2^-32。
MACH:
        for (int n = 0; n < 21; ++n)
        {
#pragma HLS PIPELINE II=1
            Hbram[n] = Hbram[n] + ((accT)(J6[HI[n]] * J6[HJ[n]]) << 18);   // 2^-14 -> 2^-32
        }
        //   b 也分两段：b[0..2] 配旋转列(J6,2^-7)，b[3..5] 配 coeff(Jc,2^-16)。
        //   （Jc 只有 3 个元素，写成 Jc[k] k<6 会越界读栈）
MACBA:
        for (int k = 0; k < 3; ++k)
        {
#pragma HLS PIPELINE II=1
            Hbram[21 + k] = Hbram[21 + k] + ((accT)(J6[k] * bv) << 9);     // 2^-23 -> 2^-32
        }
MACBC:
        for (int k = 0; k < 3; ++k)
        {
#pragma HLS PIPELINE II=1
            Hbram[24 + k] = Hbram[24 + k] + (accT)(Jc[k] * bv);            // 2^-16 × 2^-16 = 2^-32
        }
    }

OUT:
    for (int n = 0; n < 27; ++n)
    {
#pragma HLS PIPELINE II=1
        Hb[n] = (float)Hbram[n] * 2.3283064365386963e-10f;   // x2^-32
    }
}

// =====================================================================================
extern "C" void s2m_vmap(
    const float      *q,      // [nq*4]  传感器系查询点
    const ap_uint<32> *vbw,             // valid 位图（32 位字，长度 S2M_VBLEN/4）
    const float      *tab,    // [WMAX*6] 图元表（平面用前 4 个）
    const float      *parq,   // [12] R(9) | t(3)          —— stage-1
    const float      *parm,   // [27] J 旋转列系数           —— stage-2
    float            *Hb,     // [27] H 上三角 21 + b 6
    int nq, int mode,
    float L, float invL, float rmax,
    float orgx, float orgy, float orgz,
    int dimx, int dimy, int dimz,
    int rescan)
{
#pragma HLS INTERFACE m_axi     port=q    offset=slave bundle=g0 depth=16384
#pragma HLS INTERFACE m_axi     port=tab  offset=slave bundle=g1 depth=WMAX*6
#pragma HLS INTERFACE m_axi     port=parq offset=slave bundle=g0 depth=12
#pragma HLS INTERFACE m_axi     port=parm offset=slave bundle=g1 depth=27
#pragma HLS INTERFACE m_axi     port=Hb   offset=slave bundle=g1 depth=27
#pragma HLS INTERFACE m_axi     port=vbw offset=slave bundle=g0 depth=S2M_VBLEN/4
#pragma HLS INTERFACE s_axilite port=nq
#pragma HLS INTERFACE s_axilite port=mode
#pragma HLS INTERFACE s_axilite port=L
#pragma HLS INTERFACE s_axilite port=invL
#pragma HLS INTERFACE s_axilite port=rmax
#pragma HLS INTERFACE s_axilite port=orgx
#pragma HLS INTERFACE s_axilite port=orgy
#pragma HLS INTERFACE s_axilite port=orgz
#pragma HLS INTERFACE s_axilite port=dimx
#pragma HLS INTERFACE s_axilite port=dimy
#pragma HLS INTERFACE s_axilite port=dimz
#pragma HLS INTERFACE s_axilite port=rescan
#pragma HLS INTERFACE s_axilite port=return

    // 定标常量在这里算一次，两趟共用（float→int 转换器只实例化 4 个）
    const int ox12 = (int)(orgx * 4096.0f), oy12 = (int)(orgy * 4096.0f), oz12 = (int)(orgz * 4096.0f);
    const int L12  = (int)(L * 4096.0f);
    const int invL16 = (int)(invL * 65536.0f);

#pragma HLS DATAFLOW
    hls::stream<Pkt> s;
#pragma HLS STREAM variable=s depth=32

    scan_points(q, vbw, parq, s, nq, invL16, ox12, oy12, oz12, L12, dimx, dimy, dimz);
    eval_points(s, tab, parm, Hb, nq, mode, rmax, dimx, dimy, dimz, rescan, ox12, oy12, oz12, L12);
}
