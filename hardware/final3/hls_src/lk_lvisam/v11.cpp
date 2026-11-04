/*
 * sparse_lk.cpp v11 —— 稀疏 LK 光流（单层 + 初值；金字塔由软件从粗到细编排）
 *
 * v9 相对 v5 只做一件事：把外层循环按 **4 个角点一组** 批处理，压掉标量事务。
 *
 * 依据（`LK光流加速-实测报告.md` §8.5）：v5 实测 16.26 µs/角点 = 2,325 周期，
 * 而 HLS 模型只给出 1,271 周期（载入 264 + G 207 + 迭代 5×160）⇒ 有 **1,054 周期
 * 不在任何流水循环里**。这笔账只能落在外层 `for i` 未流水化：每角点开头等
 * `corners[i]`、`flow_io[i]` 两次读（各一次串行 DDR 往返），结尾 `flow_io[i]`、
 * `status[i]` 两次写。1054/4 ≈ 263 周期/事务，与一次 DDR 往返量级吻合。
 *
 * ⇒ 每 4 个角点：`corners` 读 1 次 burst(4)、`flow_io` 读 1 次、`flow_io` 写 1 次、
 *   `status` 写 1 次 = **4 个事务服务 4 个角点**，每角点摊销从 ~4 降到 ~1。
 *
 * ★ 与 v5 逐位相同的两条保证：
 *   ① 组内 **顺序** 处理（t=0..3 各自完整跑一遍 v5 的算法体），复用同一份
 *      Pb/Cb/Ix/Iy 缓冲 ⇒ 浮点运算顺序、patch 内容、迭代次数全部不变；
 *   ② v5 在 `tracked==0` 时 **不写** `flow_io[i]`；v9 必须整组写回，所以
 *      `f0b[t]` 先装读到的原值，只在 tracked 时覆盖 ⇒ 数组内容同 v5 完全一致。
 *
 * ★★ 调用方契约（与 v5 不同，务必遵守）：
 *   `corners` / `flow_io` / `status` 三个数组各需 **多分配 3 个元素**（u32），
 *   因为尾组会被整组读写。多出来的 status 会被写成 0，flow_io 写回原值，
 *   越界元素不参与任何计算（`i+t < N` 守卫），也不会拿它们去索引图像。
 *   N 不必是 4 的倍数，但数组必须够长。
 */
#include <ap_int.h>
#include <hls_math.h>
#include <math.h>

#define WIN       11                    /* 窗口边长（奇数） */
#define WHALF     (WIN / 2)             /* 5  */
#define SEARCH    4                     /* 单次调用允许的残差搜索半径 */
#define PMAX      (WHALF + SEARCH + 1)  /* 10 */
#define PSIZE     (2 * PMAX + 1)        /* 21 */
#define NW        (((3 + PSIZE - 1) >> 2) + 1)   /* 每行需读的 32 位字数（最坏情况 = 6） */
#define EPS_FLOW  0.01f
#define MIN_DET   1e-6f
#define FIX_SCALE 64.0f
#define BATCH     4                     /* ★ v9：每组角点数 */

typedef ap_uint<32> u32;

static short tofix(float x) {
#pragma HLS INLINE
    float s = x * FIX_SCALE;
    if (s > 511.0f) s = 511.0f;
    if (s < -512.0f) s = -512.0f;
    int r = (s >= 0.0f) ? (int)(s + 0.5f) : (int)(s - 0.5f);
    return (short)r;
}

void lk_v11(u32* img_prev, u32* img_cur,
               u32* corners, u32* flow_io, u32* status,
               int N, int rows, int cols, int max_iter) {
#pragma HLS INTERFACE m_axi port=img_prev offset=slave bundle=gmem1 depth=1000000
#pragma HLS INTERFACE m_axi port=img_cur  offset=slave bundle=gmem1 depth=1000000
#pragma HLS INTERFACE m_axi port=corners  offset=slave bundle=gmem1 depth=8192
#pragma HLS INTERFACE m_axi port=flow_io  offset=slave bundle=gmem1 depth=8192
#pragma HLS INTERFACE m_axi port=status   offset=slave bundle=gmem1 depth=8192
/* ★ 标量一律不写 bundle=，全部落默认 control → 单一 AXI-Lite（规避坑 A） */
#pragma HLS INTERFACE s_axilite port=N
#pragma HLS INTERFACE s_axilite port=rows
#pragma HLS INTERFACE s_axilite port=cols
#pragma HLS INTERFACE s_axilite port=max_iter
#pragma HLS INTERFACE s_axilite port=return

    const int WROW = cols >> 2;   /* 每行 32 位字数 */

    for (int i = 0; i < N; i += BATCH) {
        /* ---- ⓪ 整组标量读：1 次 burst(4) 取代 4 次单拍 ---- */
        u32 cvb[BATCH];
        u32 f0b[BATCH];
        u32 stb[BATCH];
#pragma HLS ARRAY_PARTITION variable=cvb complete dim=1
#pragma HLS ARRAY_PARTITION variable=f0b complete dim=1
#pragma HLS ARRAY_PARTITION variable=stb complete dim=1
        for (int t = 0; t < BATCH; t++) {
#pragma HLS PIPELINE II=1
            cvb[t] = corners[i + t];
        }
        for (int t = 0; t < BATCH; t++) {
#pragma HLS PIPELINE II=1
            f0b[t] = flow_io[i + t];   /* 原值先装着：未跟踪的角点要原样写回 */
        }
        for (int t = 0; t < BATCH; t++) {
#pragma HLS UNROLL
            stb[t] = 0;
        }

        /* ---- ① 组内顺序处理（★ 不 unroll：复用同一份缓冲，寄存器不×4） ---- */
        for (int t = 0; t < BATCH; t++) {
            if (i + t < N) {
                unsigned char Pb[PSIZE][PSIZE];
                unsigned char Cb[PSIZE][PSIZE];
                float Ix[WIN][WIN];
                float Iy[WIN][WIN];

                const u32 cv_ = cvb[t];
                const u32 f0  = f0b[t];
                short x0 = (short)(cv_ & 0xFFFF);
                short y0 = (short)((cv_ >> 16) & 0xFFFF);
                float u = (float)(short)((f0 >> 16) & 0xFFFF) / FIX_SCALE;
                float v = (float)(short)(f0 & 0xFFFF) / FIX_SCALE;

                int tracked = 1;
                if (x0 < PMAX || y0 < PMAX || x0 >= cols - PMAX || y0 >= rows - PMAX) tracked = 0;

                if (tracked) {
                    const int ybase = y0 - PMAX;
                    const int xbase = x0 - PMAX;
                    const int wstart = xbase >> 2;
                    const short sh = (short)(xbase & 3);

                    /* ---- 32 位整字读入 + 片上解包（同 v5） ---- */
                    for (int r = 0; r < PSIZE; r++) {
#pragma HLS PIPELINE II=12
                        u32 wp[NW], wc[NW];
#pragma HLS ARRAY_PARTITION variable=wp complete dim=1
#pragma HLS ARRAY_PARTITION variable=wc complete dim=1
                        const int base = (ybase + r) * WROW + wstart;
                        for (int w = 0; w < NW; w++) {
                            wp[w] = img_prev[base + w];
                            wc[w] = img_cur[base + w];
                        }
                        for (int p = 0; p < PSIZE; p++) {
#pragma HLS PIPELINE II=1
                            const int k = sh + p;
                            Pb[r][p] = (unsigned char)((wp[k >> 2] >> ((k & 3) << 3)) & 0xFF);
                            Cb[r][p] = (unsigned char)((wc[k >> 2] >> ((k & 3) << 3)) & 0xFF);
                        }
                    }

                    /* ---- 一阶导与 G 矩阵（只算一次；4 路展开同 v4/v5） ---- */
                    float g11a = 0.0f, g11b = 0.0f, g11c = 0.0f, g11d = 0.0f;
                    float g12a = 0.0f, g12b = 0.0f, g12c = 0.0f, g12d = 0.0f;
                    float g22a = 0.0f, g22b = 0.0f, g22c = 0.0f, g22d = 0.0f;
                    for (int k = 0; k < WIN * WIN; k += 4) {
#pragma HLS PIPELINE II=1
                        for (int tt = 0; tt < 4; tt++) {
#pragma HLS UNROLL
                            const int idx = k + tt;
                            if (idx < WIN * WIN) {
                                const int dy = idx / WIN - WHALF;
                                const int dx = idx % WIN - WHALF;
                                const int rr = PMAX + dy;
                                const int cc = PMAX + dx;
                                const float ix = ((float)Pb[rr][cc + 1] - (float)Pb[rr][cc - 1]) * 0.5f;
                                const float iy = ((float)Pb[rr + 1][cc] - (float)Pb[rr - 1][cc]) * 0.5f;
                                Ix[dy + WHALF][dx + WHALF] = ix;
                                Iy[dy + WHALF][dx + WHALF] = iy;
                                const float p11 = ix * ix, p12 = ix * iy, p22 = iy * iy;
                                if (tt == 0)      { g11a += p11; g12a += p12; g22a += p22; }
                                else if (tt == 1) { g11b += p11; g12b += p12; g22b += p22; }
                                else if (tt == 2) { g11c += p11; g12c += p12; g22c += p22; }
                                else              { g11d += p11; g12d += p12; g22d += p22; }
                            }
                        }
                    }
                    const float g11 = (g11a + g11b) + (g11c + g11d);
                    const float g12 = (g12a + g12b) + (g12c + g12d);
                    const float g22 = (g22a + g22b) + (g22c + g22d);

                    const float det = g11 * g22 - g12 * g12;
                    if (det < MIN_DET) tracked = 0;

                    if (tracked) {
                        /* ---- 迭代：Δ = -G^-1 Σ(r·∇I)，r = J(p+u) - I(p) ---- */
                        for (int it = 0; it < max_iter; it++) {
                            float b0a = 0.0f, b0b = 0.0f, b0c = 0.0f, b0d = 0.0f;
                            float b1a = 0.0f, b1b = 0.0f, b1c = 0.0f, b1d = 0.0f;
                            for (int k = 0; k < WIN * WIN; k += 4) {
#pragma HLS PIPELINE II=1
                                for (int tt = 0; tt < 4; tt++) {
#pragma HLS UNROLL
                                    const int idx = k + tt;
                                    if (idx < WIN * WIN) {
                                        const int dy = idx / WIN - WHALF;
                                        const int dx = idx % WIN - WHALF;
                                        const float sx = (float)(x0 + dx) + u;
                                        const float sy = (float)(y0 + dy) + v;
                                        const int ix0 = (int)sx;
                                        const int iy0 = (int)sy;
                                        const float ax = sx - (float)ix0;
                                        const float ay = sy - (float)iy0;
                                        const int cj = ix0 - x0 + PMAX;
                                        const int ci = iy0 - y0 + PMAX;
                                        const float v00 = (float)Cb[ci][cj];
                                        const float v01 = (float)Cb[ci][cj + 1];
                                        const float v10 = (float)Cb[ci + 1][cj];
                                        const float v11 = (float)Cb[ci + 1][cj + 1];
                                        const float jv = (1.0f - ax) * (1.0f - ay) * v00 + ax * (1.0f - ay) * v01 +
                                                         (1.0f - ax) * ay * v10 + ax * ay * v11;
                                        const float dt = jv - (float)Pb[PMAX + dy][PMAX + dx];
                                        const float p0 = Ix[dy + WHALF][dx + WHALF] * dt;
                                        const float p1 = Iy[dy + WHALF][dx + WHALF] * dt;
                                        if (tt == 0)      { b0a += p0; b1a += p1; }
                                        else if (tt == 1) { b0b += p0; b1b += p1; }
                                        else if (tt == 2) { b0c += p0; b1c += p1; }
                                        else              { b0d += p0; b1d += p1; }
                                    }
                                }
                            }
                            const float b0 = (b0a + b0b) + (b0c + b0d);
                            const float b1 = (b1a + b1b) + (b1c + b1d);
                            const float d0 = (g22 * b0 - g12 * b1) / det;
                            const float d1 = (g11 * b1 - g12 * b0) / det;
                            u -= d0;
                            v -= d1;
                            if (d0 < EPS_FLOW && d0 > -EPS_FLOW && d1 < EPS_FLOW && d1 > -EPS_FLOW) break;
                        }
                    }
                }

                const float fx = (float)x0 + u;
                const float fy = (float)y0 + v;
                if (fx < 0.0f || fy < 0.0f || fx >= (float)cols || fy >= (float)rows) tracked = 0;

                if (tracked) {
                    f0b[t] = ((u32)(unsigned short)tofix(u) << 16) | (u32)(unsigned short)tofix(v);
                }
                stb[t] = (u32)(tracked & 1);
            }
        }

        /* ---- ② 整组标量写：1 次 burst(4) 取代 4 次单拍 ---- */
        for (int t = 0; t < BATCH; t++) {
#pragma HLS PIPELINE II=1
            flow_io[i + t] = f0b[t];
        }
        for (int t = 0; t < BATCH; t++) {
#pragma HLS PIPELINE II=1
            status[i + t] = stb[t];
        }
    }
}
