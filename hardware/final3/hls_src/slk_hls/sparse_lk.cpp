/*
 * sparse_lk.cpp v2 —— 稀疏 LK 光流（单层 + 初值；金字塔由软件从粗到细编排）
 *
 * v2 相对 v1 的两处修正：
 *   ① ★ 修正 Gauss-Newton 的符号：r = J(p+u) - I(p) 时，Δ = -G^-1 * Σ(r·∇I)。
 *      v1 写成了 + 号，导致 flow 收敛到 -真值 一侧（与真值不符）。
 *   ② ★ patch 载入改为「32 位整字读取 + 片上解包」：
 *      v1 每读 1 个像素就发一次 DDR 事务（29x29x2 = 1682 次），latency 完全暴露；
 *      v2 每行只读 NW 个 32 位字（≤6 次），再在片上按字节解包。
 *   ③ SEARCH 8 → 4（PSIZE 29 → 21），载入量再降一半。
 *      —— 单层可修正 ±4 px 残差；大位移靠软件金字塔，这是设计意图。
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

typedef ap_uint<32> u32;

static short tofix(float x) {
#pragma HLS INLINE
    float s = x * FIX_SCALE;
    if (s > 511.0f) s = 511.0f;
    if (s < -512.0f) s = -512.0f;
    int r = (s >= 0.0f) ? (int)(s + 0.5f) : (int)(s - 0.5f);
    return (short)r;
}

void sparse_lk(u32* img_prev, u32* img_cur,
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

    for (int i = 0; i < N; i++) {
        unsigned char Pb[PSIZE][PSIZE];
        unsigned char Cb[PSIZE][PSIZE];
        float Ix[WIN][WIN];
        float Iy[WIN][WIN];

        u32 cv_ = corners[i];
        u32 f0  = flow_io[i];
        short x0 = (short)(cv_ & 0xFFFF);
        short y0 = (short)((cv_ >> 16) & 0xFFFF);
        float u = (float)(short)((f0 >> 16) & 0xFFFF) / FIX_SCALE;
        float v = (float)(short)(f0 & 0xFFFF) / FIX_SCALE;

        int tracked = 1;
        if (x0 < PMAX || y0 < PMAX || x0 >= cols - PMAX || y0 >= rows - PMAX) tracked = 0;

        if (tracked) {
            const int ybase = y0 - PMAX;
            const int xbase = x0 - PMAX;
            const int wstart = xbase >> 2;      /* 起始字（xbase 落在这个字内或正好对齐） */
            const short sh = (short)(xbase & 3);/* 字内字节偏移 0..3 */

            /* ---- ① 32 位整字读入 + 片上解包 ---- */
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

            /* ---- ② 一阶导与 G 矩阵（只算一次） ---- */
            /* v4：展平为 WIN*WIN 的单循环 + 4 路展开 + 4 组独立部分和
             * —— 打断 HLS 报出的浮点累加链（carried dependence on g11/g12/g22） */
            float g11a = 0.0f, g11b = 0.0f, g11c = 0.0f, g11d = 0.0f;
            float g12a = 0.0f, g12b = 0.0f, g12c = 0.0f, g12d = 0.0f;
            float g22a = 0.0f, g22b = 0.0f, g22c = 0.0f, g22d = 0.0f;
            for (int k = 0; k < WIN * WIN; k += 4) {
#pragma HLS PIPELINE II=1
                for (int t = 0; t < 4; t++) {
#pragma HLS UNROLL
                    const int idx = k + t;
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
                        if (t == 0)      { g11a += p11; g12a += p12; g22a += p22; }
                        else if (t == 1) { g11b += p11; g12b += p12; g22b += p22; }
                        else if (t == 2) { g11c += p11; g12c += p12; g22c += p22; }
                        else             { g11d += p11; g12d += p12; g22d += p22; }
                    }
                }
            }
            const float g11 = (g11a + g11b) + (g11c + g11d);
            const float g12 = (g12a + g12b) + (g12c + g12d);
            const float g22 = (g22a + g22b) + (g22c + g22d);

            const float det = g11 * g22 - g12 * g12;
            if (det < MIN_DET) tracked = 0;

            if (tracked) {
                /* ---- ③ 迭代：Δ = -G^-1 Σ(r·∇I)，r = J(p+u) - I(p) ---- */
                for (int it = 0; it < max_iter; it++) {
                    /* v4：同上，4 路展开打断 b0/b1 的浮点累加链 */
                    float b0a = 0.0f, b0b = 0.0f, b0c = 0.0f, b0d = 0.0f;
                    float b1a = 0.0f, b1b = 0.0f, b1c = 0.0f, b1d = 0.0f;
                    for (int k = 0; k < WIN * WIN; k += 4) {
#pragma HLS PIPELINE II=1
                        for (int t = 0; t < 4; t++) {
#pragma HLS UNROLL
                            const int idx = k + t;
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
                                if (t == 0)      { b0a += p0; b1a += p1; }
                                else if (t == 1) { b0b += p0; b1b += p1; }
                                else if (t == 2) { b0c += p0; b1c += p1; }
                                else             { b0d += p0; b1d += p1; }
                            }
                        }
                    }
                    const float b0 = (b0a + b0b) + (b0c + b0d);
                    const float b1 = (b1a + b1b) + (b1c + b1d);
                    const float d0 = (g22 * b0 - g12 * b1) / det;
                    const float d1 = (g11 * b1 - g12 * b0) / det;
                    u -= d0;      /* ★ v2：负号 */
                    v -= d1;
                    if (d0 < EPS_FLOW && d0 > -EPS_FLOW && d1 < EPS_FLOW && d1 > -EPS_FLOW) break;
                }
            }
        }

        const float fx = (float)x0 + u;
        const float fy = (float)y0 + v;
        if (fx < 0.0f || fy < 0.0f || fx >= (float)cols || fy >= (float)rows) tracked = 0;

        if (tracked) {
            flow_io[i] = ((u32)(unsigned short)tofix(u) << 16) | (u32)(unsigned short)tofix(v);
        }
        status[i] = (u32)(tracked & 1);
    }
}
