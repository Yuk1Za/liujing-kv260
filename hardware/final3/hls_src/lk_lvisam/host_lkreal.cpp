/* host_lkreal.cpp —— 用真实 KITTI 数据验证 lk_lvisam 内核（纯 g++，不需要 OpenCV）
 *
 * 参考值由 dump_lk_ref.py 用 cv2 现算并落盘（就是 LVI-SAM 那一行
 * calcOpticalFlowPyrLK(..., Size(21,21), 3) 的结果）。
 *
 * 编排方式与板上软件一致：从最粗层开始，init=0；每往细一层把上一层的流 ×2 当初值。
 *
 * 输出：按参考流大小分桶比较（重点看 10–20 px、>20 px 两桶 —— 旧核就是在这里崩的）。
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

typedef unsigned int u32;

void lk_lvisam(u32*, u32*, u32*, u32*, u32*, int, int, int, int);

#define PAD 8
#define MAXIT 30

static short tofix(float x) {
    float s = x * 16.0f;
    if (s > 2047.0f) s = 2047.0f;
    if (s < -2048.0f) s = -2048.0f;
    int r = (s >= 0.0f) ? (int)(s + 0.5f) : (int)(s - 0.5f);
    return (short)r;
}

int main(int argc, char** argv) {
    const char* dir = (argc > 1) ? argv[1] : "/home/srtp2025_nie/lk_real";
    const int pairs[] = {3, 11, 19, 31, 63, 131};
    const int np = 6;
    const int BUCK[] = {0, 2, 5, 10, 20, 100000};
    std::vector<std::vector<double>> bucket_err(5);
    int tot_track = 0, tot_pts = 0, tot_ref = 0, agree = 0;

    for (int ip = 0; ip < np; ip++) {
        char fn[512];
        snprintf(fn, sizeof(fn), "%s/p%03d.bin", dir, pairs[ip]);
        FILE* f = fopen(fn, "rb");
        if (!f) { printf("[跳过] %s\n", fn); continue; }
        int hdr[4];
        if (fread(hdr, sizeof(int), 4, f) != 4) { fclose(f); continue; }
        const int rows = hdr[0], cols = hdr[1], L = hdr[2], N = hdr[3];
        std::vector<std::vector<unsigned char>> pv(L), cv(L);
        for (int l = 0; l < L; l++) {
            int r = rows >> l, c = cols >> l;
            pv[l].resize((size_t)r * c); cv[l].resize((size_t)r * c);
            if (fread(pv[l].data(), 1, pv[l].size(), f) != pv[l].size()) { printf("读错\n"); return 1; }
        }
        for (int l = 0; l < L; l++) {
            int r = rows >> l, c = cols >> l;
            if (fread(cv[l].data(), 1, (size_t)r * c, f) != (size_t)(r * c)) { printf("读错\n"); return 1; }
        }
        std::vector<u32> cor(N);
        if (fread(cor.data(), 4, N, f) != (size_t)N) { printf("读错\n"); return 1; }
        std::vector<float> ru(N), rv(N);
        std::vector<unsigned char> rst(N);
        if (fread(ru.data(), 4, N, f) != (size_t)N) { printf("读错\n"); return 1; }
        if (fread(rv.data(), 4, N, f) != (size_t)N) { printf("读错\n"); return 1; }
        if (fread(rst.data(), 1, N, f) != (size_t)N) { printf("读错\n"); return 1; }
        fclose(f);

        /* 逐层编排：coarse -> fine，流按层缩放 */
        std::vector<float> uf(N, 0.f), vf(N, 0.f);   /* level-0 坐标下的估计 */
        std::vector<unsigned char> okf(N, 1);
        for (int l = L - 1; l >= 0; l--) {
            const int r = rows >> l, c = cols >> l;
            std::vector<u32> corl(N + PAD), fl(N + PAD), st(N + PAD);
            for (int i = 0; i < N; i++) {
                int x = (int)((cor[i] & 0xFFFF) + (1 << l) / 2) >> l;   /* round(x/2^l) */
                int y = (int)(((cor[i] >> 16) & 0xFFFF) + (1 << l) / 2) >> l;
                corl[i] = ((u32)(y & 0xFFFF) << 16) | (u32)(x & 0xFFFF);
                fl[i] = ((u32)(unsigned short)tofix(uf[i] / (float)(1 << l)) << 16) |
                        (u32)(unsigned short)tofix(vf[i] / (float)(1 << l));
            }
            for (int i = N; i < N + PAD; i++) { corl[i] = 0; fl[i] = 0xDEADBEEF; st[i] = 0; }
            lk_lvisam((u32*)pv[l].data(), (u32*)cv[l].data(), corl.data(), fl.data(), st.data(),
                      N, r, c, MAXIT);
            /* 逐层打点：本层跟踪数 + 流中位 vs 参考/2^l */
            {
                std::vector<double> m1, m2;
                int ntr = 0;
                for (int i = 0; i < N; i++) {
                    if (!(st[i] & 1)) continue;
                    ntr++;
                    short uu = (short)((fl[i] >> 16) & 0xFFFF), vv = (short)(fl[i] & 0xFFFF);
                    m1.push_back(sqrt((double)uu * uu + (double)vv * vv) / 16.0);
                    if (rst[i]) m2.push_back(sqrt((double)ru[i]*ru[i] + (double)rv[i]*rv[i]) / (1 << l));
                }
                std::sort(m1.begin(), m1.end()); std::sort(m2.begin(), m2.end());
                if (!m1.empty() && !m2.empty())
                    printf("    pair %3d L%d: 跟踪 %2d  |流|中位 %7.2f  参考/2^l %7.2f\n",
                           pairs[ip], l, ntr, m1[m1.size()/2], m2[m2.size()/2]);
            }
            for (int i = 0; i < N; i++) {
                if (st[i] & 1) {
                    short uu = (short)((fl[i] >> 16) & 0xFFFF);
                    short vv = (short)(fl[i] & 0xFFFF);
                    uf[i] = (float)uu / 16.0f * (float)(1 << l);
                    vf[i] = (float)vv / 16.0f * (float)(1 << l);
                } else if (l == 0) {
                    okf[i] = 0;
                }
                /* 粗层没跟上的点不判死：保留原初值，留到更细的层再试（与板上软件一致） */
            }
        }

        /* 统计 */
        int ntr = 0, nref = 0, nagree = 0;
        double dmax = 0;
        for (int i = 0; i < N; i++) {
            tot_pts++;
            if (rst[i]) tot_ref++;
            if (!okf[i]) continue;
            ntr++; tot_track++;
            if (!rst[i]) continue;                        /* 参考侧失败，不计误差 */
            nref++;
            double du = uf[i] - ru[i], dv = vf[i] - rv[i];
            double e = sqrt(du * du + dv * dv);
            if (e < 1.0) nagree++;
            double mag = sqrt((double)ru[i] * ru[i] + (double)rv[i] * rv[i]);
            for (int b = 0; b < 5; b++)
                if (mag >= BUCK[b] && mag < BUCK[b + 1]) { bucket_err[b].push_back(e); break; }
            if (e > dmax) dmax = e;
        }
        printf("pair %3d: 内核跟踪 %2d/%2d  参考成功 %2d  两边都在 %2d  <1px 一致 %2d  max %.2f\n",
               pairs[ip], ntr, N, nref, nref, nagree, dmax);
        agree += nagree;
    }

    printf("\n=== 按参考流大小分桶（误差 = |内核流 − cv2 流|, px）===\n");
    const char* nm[] = {"[0,2)", "[2,5)", "[5,10)", "[10,20)", "[20,inf)"};
    for (int b = 0; b < 5; b++) {
        auto& v = bucket_err[b];
        if (v.empty()) { printf("  %-10s  (无)\n", nm[b]); continue; }
        std::sort(v.begin(), v.end());
        double med = v[v.size() / 2], p90 = v[(size_t)(v.size() * 0.9)];
        printf("  %-10s n=%4zu  中位 %7.3f  p90 %7.3f  max %7.2f  <1px %5.1f%%\n",
               nm[b], v.size(), med, p90, v.back(), 100.0 * std::count_if(v.begin(), v.end(),
               [](double x) { return x < 1.0; }) / v.size());
    }
    printf("\n合计: 点 %d，参考成功 %d，内核跟踪 %d，两边都在且 <1px 一致 %d\n",
           tot_pts, tot_ref, tot_track, agree);
    return 0;
}
