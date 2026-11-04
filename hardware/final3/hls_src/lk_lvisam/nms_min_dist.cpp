/*
 * nms_min_dist.cpp —— 把 Shi-Tomasi 响应图的「MIN_DIST 选点」搬进 PL。
 *
 * 为什么需要它（板上实测，2026-10-23）：
 *   ST IP 出响应图只要 2.5 ms，但把「降序 → MIN_DIST 欧氏抑制 → 前 60」放在 Python 里要
 *   **7.8–11 ms**（受 Python↔numpy 每次调用 ~10 µs 的地板限制，四版实现都压不下去）。
 *   60 点 × 8 次调用 × 60 轮 ≈ 硬地板 ⇒ 只能整体搬硬件。
 *
 * 算法（与 cv2.goodFeaturesToTrack 的后处理**逐点等价**，不是近似）：
 *   ① 流式扫描响应图：跳过 border 安全区（LK 每层要 ±PMAX）与低于 thr 的像素，
 *      把 (x,y,score) 依次写入 BRAM 候选表（容量 CAP，溢出置 flag 让软件抬阈值重跑）；
 *   ② 贪心 max_n 轮：每轮线性扫描候选表，找出**分数最高且未被抑制**者输出，
 *      同时在同一次扫描里把「与上一轮选中点距离 < min_dist」的候选打上抑制标记。
 *      用「上一轮选中点」做增量屏蔽是等价的：一旦被抑制就永久置位，而新点的抑制在下一轮生效。
 *   评分相同时保留**先出现**的（行优先序）—— 与 numpy 的 stable argsort + argmin(blocked) 一致。
 *
 * 复杂度：流式 rows*cols/4 拍 + 贪心 max_n × n 拍
 *   ⇒ 480×640 图 + 2000 候选 + 60 点 ≈ 192k 拍 ≈ **1.3 ms @143 MHz**（实测待验）。
 *
 * 输出契约：out_xy[i] = (y << 16) | x，与 sparse_lk 的 corners 编码一致；
 *          out_n[0] = 点数 | (溢出标志 << 31)。
 */
#include <ap_int.h>
#include <stdint.h>

#define CAP 4096

typedef ap_uint<32> u32;

void nms_min_dist(const u32 *resp, u32 *out_xy, u32 *out_n,
                  int rows, int cols, int thr, int min_dist, int border, int max_n)
{
/* ★ m_axi 用 offset=slave + **标量不带 bundle=**：这是项目里已验证的写法（LK/ST 同）。
   写成 `s_axilite port=rows bundle=control` 会让 HLS 生成**两个** AXI-Lite 口
   （s_axi_control 放标量、s_axi_control_r 放 m_axi 基址），PYNQ 只认一个 ⇒ 基址写不进去。 */
#pragma HLS INTERFACE m_axi port=resp   offset=slave bundle=gmem1 depth=76800
#pragma HLS INTERFACE m_axi port=out_xy offset=slave bundle=gmem1 depth=64
#pragma HLS INTERFACE m_axi port=out_n  offset=slave bundle=gmem1 depth=1
#pragma HLS INTERFACE s_axilite port=rows
#pragma HLS INTERFACE s_axilite port=cols
#pragma HLS INTERFACE s_axilite port=thr
#pragma HLS INTERFACE s_axilite port=min_dist
#pragma HLS INTERFACE s_axilite port=border
#pragma HLS INTERFACE s_axilite port=max_n
#pragma HLS INTERFACE s_axilite port=return

    static unsigned short cx[CAP];
    static unsigned short cy[CAP];
    static unsigned char  cs[CAP];
    static unsigned char  blk[CAP];

    const int wpr = cols >> 2;
    int n = 0;
    int overflow = 0;

    /* ---------------- ① 流式收集候选 ---------------- */
    for (int y = 0; y < rows; ++y) {
#pragma HLS LOOP_TRIPCOUNT min=480 max=480
        if (y >= border && y < rows - border) {
            for (int w = 0; w < wpr; ++w) {
#pragma HLS LOOP_TRIPCOUNT min=160 max=160
#pragma HLS PIPELINE II=1
                u32 word = resp[y * wpr + w];
                for (int j = 0; j < 4; ++j) {
#pragma HLS UNROLL
                    const int x = (w << 2) + j;
                    const unsigned s = (unsigned)((word >> (8 * j)) & 0xFFu);
                    if (x >= border && x < cols - border && thr > 0 && s >= (unsigned)thr) {
                        if (n < CAP) {
                            cx[n] = (unsigned short)x;
                            cy[n] = (unsigned short)y;
                            cs[n] = (unsigned char)s;
                            blk[n] = 0;
                            n = n + 1;
                        } else {
                            overflow = 1;
                        }
                    }
                }
            }
        }
    }

    /* ---------------- ② 带增量屏蔽的贪心 ---------------- */
    const int r2 = min_dist * min_dist;
    int lastx = -1, lasty = -1, m = 0;
    for (int it = 0; it < max_n; ++it) {
#pragma HLS LOOP_TRIPCOUNT min=1 max=64
        int best = -1, bests = -1;
        for (int i = 0; i < n; ++i) {
#pragma HLS PIPELINE II=1
            if (blk[i] == 0) {
                if (lastx >= 0) {
                    const int dx = (int)cx[i] - lastx;
                    const int dy = (int)cy[i] - lasty;
                    if (dx * dx + dy * dy < r2) {
                        blk[i] = 1;
                    }
                }
                if (blk[i] == 0 && (int)cs[i] > bests) {
                    bests = (int)cs[i];
                    best = i;
                }
            }
        }
        if (best < 0) break;
        blk[best] = 1;
        out_xy[m] = ((u32)cy[best] << 16) | (u32)cx[best];
        m = m + 1;
        lastx = (int)cx[best];
        lasty = (int)cy[best];
    }

    out_n[0] = (u32)m | ((u32)overflow << 31);
}
