/*
 * lk_lvisam.cpp —— LVI-SAM 原版视觉前端的 LK 内核（忠实实现，不自创参数）
 *
 * 算法的唯一依据是上游源码 `src/visual_odometry/visual_feature/feature_tracker.cpp`：
 *
 *     cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts,
 *                              status, err, cv::Size(21, 21), 3);
 *
 * 即 **窗口 21×21、金字塔 3 层（maxLevel=3 ⇒ 4 级）、TermCriteria 用 OpenCV 默认
 * 的 (EPS|COUNT, 30, 0.01)**；minEigThreshold 用默认 1e-4。上式的第 8 个参数
 * （TermCriteria）省略了，所以是默认值，不是"5 次"或"10 次"。
 *
 * 与旧核（sparse_lk_v11）的差别 —— 旧核是自创简化，这里是原版语义
 * ------------------------------------------------------------------
 * 旧核：窗口 11×11、SEARCH=4、patch **恒以角点为中心**，`flow` 初值只当迭代起点。
 *       ⇒ 采样下标 `cj = dx + u + PMAX`，`|u|>5` 就越出 patch，单层只能跟 ±5 px，
 *         且 prev/cur 共用一个窗口基址，patch 不跟初值走（"金字塔"形同虚设）。
 * 本核：窗口 21×21；**每次迭代都按当前估计 (u,v) 重新定心并重载 cur 的 21×21 patch**
 *       ⇒ 采样下标只有亚像素残差 `dx + (u - round(u))`，恒在窗内。
 *       这与 cv2 每次迭代直接对图像做双线性采样是同一语义，因此**捕获范围不再受窗口限制**，
 *       只受"初值误差 ≤ 金字塔放大后的窗口半宽"限制（第 l 层约 ±10·2^l px）。
 *
 * 关键实现点
 * ----------
 * ① prev 模板固定以角点 (x0,y0) 为中心（跟的是正确的那个点），ASIZE=23 留 ±1 边算梯度；
 * ② 梯度用 **3×3 Sobel**（与 OpenCV KLT 一致：先平滑再中心差分），旧核用的是裸中心差分；
 * ③ 退化判据用 OpenCV 的 minEigThreshold：minEig(G)/窗口面积 < 1e-4 ⇒ status=0；
 * ④ 迭代终止：|du|<0.01 && |dv|<0.01（EPS_FLOW），或 30 次（MAX_IT，运行期由寄存器给）；
 * ⑤ 输出仍是 Q6 定点的**总位移**（相对角点），契约与旧核一致，软件侧照旧 ×2^l 上采样。
 *
 * 调用契约（与旧核一致，便于复用软件侧）
 *   corners : (y<<16)|x，**必须是 prev 帧坐标**
 *   flow_io : in/out，**Q4**（FIX_SCALE=16），(u<<16)|v，可表示 ±128 px
 *   status  : bit0=1 表示跟踪成功
 *   corners/flow_io/status 三个数组各**多留 3 个元素**（BATCH=4 尾组整组读写）
 */
#include <ap_int.h>
#include <hls_math.h>
#include <math.h>

#define WIN       21                    /* 窗口边长（= cv::Size(21,21)） */
#define WHALF     (WIN / 2)             /* 10 */
#define PMAX      10
#define PSIZE     21                    /* cur patch 覆盖 x0±10 */
#define ASIZE     23                    /* prev patch 覆盖 x0±11（±1 给梯度） */
#define ITER_MAX0 30                    /* TermCriteria COUNT */
#define EPS_FLOW  0.01f                 /* TermCriteria EPS */
#define MIN_EIG   1e-4f                 /* OpenCV minEigThreshold 默认值 */
/* ★ 定点标度：旧核用 Q6（FIX_SCALE=64），int16 只能表示 ±511/64 = **±7.98 px**，
 *   位移一超过就被饱和 —— 实测 L0 的流恒为 8.00 px（真值 25 px）就是这个钳位，
 *   与算法无关。改 Q4（16）：范围 ±2048/16 = **±128 px**，分辨率 0.0625 px
 *   （PnP 的 RANSAC 阈值是 2 px，0.06 px 量化可忽略）。 */
#define FIX_SCALE 16.0f
#define BATCH     4
#define BORDER    1                     /* feature_tracker.cpp 的 inBorder BORDER_SIZE */

typedef ap_uint<32> u32;

/* Q4 定点：饱和到 int16（±2048 ⇒ ±128 px） */
static short tofix(float x) {
#pragma HLS INLINE
    float s = x * FIX_SCALE;
    if (s > 2047.0f) s = 2047.0f;
    if (s < -2048.0f) s = -2048.0f;
    int r = (s >= 0.0f) ? (int)(s + 0.5f) : (int)(s - 0.5f);
    return (short)r;
}

/* 每行需要读的 32 位字数（起点可能不对齐 4 像素） */
#define NW_A  (((3 + ASIZE - 1) >> 2) + 1)   /* 23 -> 7 */
#define NW_C  (((3 + PSIZE - 1) >> 2) + 1)   /* 21 -> 6 */

void lk_lvisam(u32* img_prev, u32* img_cur,
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
        /* ---- 整组标量读：1 次 burst 取代 4 次单拍 ---- */
        u32 cvb[BATCH], f0b[BATCH], stb[BATCH];
#pragma HLS ARRAY_PARTITION variable=cvb complete dim=1
#pragma HLS ARRAY_PARTITION variable=f0b complete dim=1
#pragma HLS ARRAY_PARTITION variable=stb complete dim=1
        for (int t = 0; t < BATCH; t++) {
#pragma HLS PIPELINE II=1
            cvb[t] = corners[i + t];
        }
        for (int t = 0; t < BATCH; t++) {
#pragma HLS PIPELINE II=1
            f0b[t] = flow_io[i + t];   /* 未跟踪的点要原样写回 */
        }
        for (int t = 0; t < BATCH; t++) {
#pragma HLS UNROLL
            stb[t] = 0;
        }

        /* ---- 组内顺序处理（不 unroll：复用同一份缓冲，寄存器不 ×4） ---- */
        for (int t = 0; t < BATCH; t++) {
            if (i + t < N) {
                unsigned char A[ASIZE][ASIZE];      /* prev 模板（含梯度边） */
                unsigned char Bp[PSIZE][PSIZE];     /* cur patch（每轮重载） */
                float Ix[WIN][WIN];
                float Iy[WIN][WIN];

                const u32 cv_ = cvb[t];
                const u32 f0  = f0b[t];
                const short x0 = (short)(cv_ & 0xFFFF);
                const short y0 = (short)((cv_ >> 16) & 0xFFFF);
                float u = (float)(short)((f0 >> 16) & 0xFFFF) / FIX_SCALE;
                float v = (float)(short)(f0 & 0xFFFF) / FIX_SCALE;

                int tracked = 1;
                /* inBorder（BORDER_SIZE=1）+ 模板窗完整 */
                if (x0 < WHALF + 1 || y0 < WHALF + 1 ||
                    x0 >= cols - WHALF - 1 || y0 >= rows - WHALF - 1) tracked = 0;

                if (tracked) {
                    /* ---- ① prev 模板：固定以 (x0,y0) 为中心，ASIZE×ASIZE ---- */
                    const int ay0 = y0 - WHALF - 1;      /* = y0-11 */
                    const int ax0 = x0 - WHALF - 1;
                    const int aws = ax0 >> 2;
                    const short ash = (short)(ax0 & 3);
                    for (int r = 0; r < ASIZE; r++) {
#pragma HLS PIPELINE II=12
                        u32 wp[NW_A];
#pragma HLS ARRAY_PARTITION variable=wp complete dim=1
                        const int base = (ay0 + r) * WROW + aws;
                        for (int w = 0; w < NW_A; w++) {
#pragma HLS PIPELINE II=1
                            wp[w] = img_prev[base + w];
                        }
                        for (int p = 0; p < ASIZE; p++) {
#pragma HLS PIPELINE II=1
                            const int k = ash + p;
                            A[r][p] = (unsigned char)((wp[k >> 2] >> ((k & 3) << 3)) & 0xFF);
                        }
                    }

                    /* ---- ② 梯度（3×3 Sobel，与 OpenCV KLT 一致）+ G 矩阵 ---- */
                    /* A 的下标：模板 (dx,dy) 在 A[1+dy+WHALF][1+dx+WHALF] */
                    float g11 = 0.f, g12 = 0.f, g22 = 0.f;
                    for (int dy = -WHALF; dy <= WHALF; dy++) {
                        for (int dx = -WHALF; dx <= WHALF; dx++) {
#pragma HLS PIPELINE II=1
                            const int rr = 1 + dy + WHALF;
                            const int cc = 1 + dx + WHALF;
                            /* Sobel x: [-1 0 1; -2 0 2; -1 0 1] / 8
                               Sobel y: [-1 -2 -1; 0 0 0; 1 2 1] / 8 */
                            const float ix = ((float)A[rr-1][cc+1] + 2.f*(float)A[rr][cc+1] + (float)A[rr+1][cc+1]
                                            - (float)A[rr-1][cc-1] - 2.f*(float)A[rr][cc-1] - (float)A[rr+1][cc-1]) * 0.125f;
                            const float iy = ((float)A[rr+1][cc-1] + 2.f*(float)A[rr+1][cc] + (float)A[rr+1][cc+1]
                                            - (float)A[rr-1][cc-1] - 2.f*(float)A[rr-1][cc] - (float)A[rr-1][cc+1]) * 0.125f;
                            Ix[dy + WHALF][dx + WHALF] = ix;
                            Iy[dy + WHALF][dx + WHALF] = iy;
                            g11 += ix * ix;
                            g12 += ix * iy;
                            g22 += iy * iy;
                        }
                    }
                    /* minEigThreshold：min 特征值 / 窗口面积 */
                    const float tr = g11 + g22;
                    const float df = g11 * g22 - g12 * g12;
                    const float disc = tr * tr * 0.25f - df;
                    const float sq = (disc > 0.f) ? hls::sqrtf(disc) : 0.f;
                    const float minEig = tr * 0.5f - sq;
                    if (minEig < MIN_EIG * (float)(WIN * WIN)) tracked = 0;

                    /* ---- ③ 迭代：按当前估计定心 cur patch（整数中心变了才重载） ---- */
                    if (tracked) {
                        int cx = 0, cy = 0, cx_loaded = 0x7FFFFFFF, cy_loaded = 0x7FFFFFFF;
                        for (int it = 0; it < max_iter; it++) {
                            cx = x0 + (int)floorf(u + 0.5f);
                            cy = y0 + (int)floorf(v + 0.5f);
                            if (cx < PMAX || cy < PMAX || cx >= cols - PMAX || cy >= rows - PMAX) {
                                tracked = 0;
                                break;
                            }
                            /* ★ 同一整数中心下重载结果完全相同 ⇒ 只在中心变化时读 DDR。
                               这不改变语义（cv2 每轮也是从同一幅图取同样的像素），
                               但把 30 轮的载入压到 1–3 轮。 */
                            if (cx != cx_loaded || cy != cy_loaded) {
                                cx_loaded = cx; cy_loaded = cy;
                                const int cy0 = cy - PMAX;
                                const int cx0 = cx - PMAX;
                                const int cws = cx0 >> 2;
                                const short csh = (short)(cx0 & 3);
                                for (int r = 0; r < PSIZE; r++) {
#pragma HLS PIPELINE II=12
                                    u32 wc[NW_C];
#pragma HLS ARRAY_PARTITION variable=wc complete dim=1
                                    const int base = (cy0 + r) * WROW + cws;
                                    for (int w = 0; w < NW_C; w++) {
#pragma HLS PIPELINE II=1
                                        wc[w] = img_cur[base + w];
                                    }
                                    for (int p = 0; p < PSIZE; p++) {
#pragma HLS PIPELINE II=1
                                        const int k = csh + p;
                                        Bp[r][p] = (unsigned char)((wc[k >> 2] >> ((k & 3) << 3)) & 0xFF);
                                    }
                                }
                            }

                            /* 压平成 k=0..WIN*WIN-1、4 路部分和（与 v11 同一套已验证写法）。
                               ★ 不能按 dx 成对展开：21 列是奇数，dx0=10 时 dx 会到 11，
                                 越出 21 宽的 Ix/Iy 数组（多读整整一列，实测偏差 ~1e-2）。 */
                            float b0a = 0.f, b0b = 0.f, b0c = 0.f, b0d = 0.f;
                            float b1a = 0.f, b1b = 0.f, b1c = 0.f, b1d = 0.f;
                            for (int k = 0; k < WIN * WIN; k += 2) {
#pragma HLS PIPELINE II=1
                                for (int tt = 0; tt < 2; tt++) {
#pragma HLS UNROLL
                                    const int idx = k + tt;
                                    if (idx < WIN * WIN) {
                                        const int dy = idx / WIN - WHALF;
                                        const int dx = idx % WIN - WHALF;
                                        const float sx = (float)(x0 + dx) + u;
                                        const float sy = (float)(y0 + dy) + v;
                                        const int ix0 = (int)floorf(sx);
                                        const int iy0 = (int)floorf(sy);
                                        const float axf = sx - (float)ix0;
                                        const float ayf = sy - (float)iy0;
                                        const int cj = ix0 - cx + PMAX;
                                        const int ci = iy0 - cy + PMAX;
                                        float jv;
                                        if (cj < 0 || cj >= PSIZE - 1 || ci < 0 || ci >= PSIZE - 1) {
                                            jv = (float)Bp[PMAX][PMAX];   /* 越界退化为中心像素 */
                                        } else {
                                            const float v00 = (float)Bp[ci][cj];
                                            const float v01 = (float)Bp[ci][cj + 1];
                                            const float v10 = (float)Bp[ci + 1][cj];
                                            const float v11 = (float)Bp[ci + 1][cj + 1];
                                            jv = (1.f-axf)*(1.f-ayf)*v00 + axf*(1.f-ayf)*v01
                                               + (1.f-axf)*ayf*v10 + axf*ayf*v11;
                                        }
                                        const float It = jv - (float)A[1 + dy + WHALF][1 + dx + WHALF];
                                        const float px_ = Ix[dy + WHALF][dx + WHALF] * It;
                                        const float py_ = Iy[dy + WHALF][dx + WHALF] * It;
                                        if (tt == 0) { b0a += px_; b1a += py_; }
                                        else         { b0b += px_; b1b += py_; }
                                    }
                                }
                            }
                            const float b0 = b0a + b0b;
                            const float b1 = b1a + b1b;
                            const float det = g11 * g22 - g12 * g12;
                            if (det < 1e-9f) { tracked = 0; break; }
                            const float du = (g22 * b0 - g12 * b1) / det;
                            const float dv = (g11 * b1 - g12 * b0) / det;
                            u -= du;                 /* ★ GN 更新是 -=（+号会收敛到 -真值） */
                            v -= dv;
                            if (du < EPS_FLOW && du > -EPS_FLOW &&
                                dv < EPS_FLOW && dv > -EPS_FLOW) break;
                        }
                    }

                    /* ---- ④ 落点检查 + 写回 ---- */
                    if (tracked) {
                        const float fx = (float)x0 + u;
                        const float fy = (float)y0 + v;
                        if (fx < (float)BORDER || fy < (float)BORDER ||
                            fx >= (float)(cols - BORDER) || fy >= (float)(rows - BORDER)) tracked = 0;
                    }
                    if (tracked) f0b[t] = ((u32)(unsigned short)tofix(u) << 16) |
                                          (u32)(unsigned short)tofix(v);
                    stb[t] = (u32)(tracked & 1);
                }
            }
        }

        /* ---- 整组写回 ---- */
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
