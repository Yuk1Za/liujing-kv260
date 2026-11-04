/* lk_tb.cpp —— cosim 测试台：跑一个真实 pair 的 4 层，只为拿**精确拍数**
 * 用法：cosim_design 后看 "Latency: xxx cycles"
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include "ap_int.h"

typedef ap_uint<32> u32;
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

int main() {
    const char* fn = "/home/srtp2025_nie/lk_real/p019.bin";
    FILE* f = fopen(fn, "rb");
    if (!f) { printf("[tb] 打不开 %s\n", fn); return 1; }
    int hdr[4];
    if (fread(hdr, sizeof(int), 4, f) != 4) return 1;
    const int rows = hdr[0], cols = hdr[1], L = hdr[2], N = hdr[3];
    std::vector<std::vector<unsigned char>> pv(L), cv(L);
    for (int l = 0; l < L; l++) {
        int r = rows >> l, c = cols >> l;
        pv[l].resize((size_t)r * c); cv[l].resize((size_t)r * c);
        if (fread(pv[l].data(), 1, pv[l].size(), f) != pv[l].size()) return 1;
    }
    for (int l = 0; l < L; l++) {
        int r = rows >> l, c = cols >> l;
        if (fread(cv[l].data(), 1, (size_t)r * c, f) != (size_t)(r * c)) return 1;
    }
    std::vector<u32> cor(N);
    if (fread(cor.data(), 4, N, f) != (size_t)N) return 1;
    std::vector<float> ru(N), rv(N);
    std::vector<unsigned char> rst(N);
    fread(ru.data(), 4, N, f); fread(rv.data(), 4, N, f); fread(rst.data(), 1, N, f);
    fclose(f);

    std::vector<float> uf(N, 0.f), vf(N, 0.f);
    std::vector<unsigned char> okf(N, 1);
    for (int l = L - 1; l >= 0; l--) {
        const int r = rows >> l, c = cols >> l;
        std::vector<u32> corl(N + PAD), fl(N + PAD), st(N + PAD);
        for (int i = 0; i < N; i++) {
            int x = (int)((cor[i] & 0xFFFF) + (1 << l) / 2) >> l;
            int y = (int)(((cor[i] >> 16) & 0xFFFF) + (1 << l) / 2) >> l;
            corl[i] = ((u32)(y & 0xFFFF) << 16) | (u32)(x & 0xFFFF);
            fl[i] = ((u32)(unsigned short)tofix(uf[i] / (float)(1 << l)) << 16) |
                    (u32)(unsigned short)tofix(vf[i] / (float)(1 << l));
        }
        for (int i = N; i < N + PAD; i++) { corl[i] = 0; fl[i] = 0; st[i] = 0; }
        /* ★ 图像是 4 像素/32 位字打包：小端下 unsigned char* 直接重解释成 u32* 就行
           （核里按 (k&3) 取字节，正是内存里的第 k 个字节）。HLS 编译比 g++ 严格，必须显式转。 */
        lk_lvisam((u32*)pv[l].data(), (u32*)cv[l].data(), corl.data(), fl.data(), st.data(),
                  N, r, c, MAXIT);
        int ntr = 0;
        for (int i = 0; i < N; i++) {
            if (st[i] & 1) {
                ntr++;
                short uu = (short)((fl[i] >> 16) & 0xFFFF), vv = (short)(fl[i] & 0xFFFF);
                uf[i] = (float)uu / 16.0f * (float)(1 << l);
                vf[i] = (float)vv / 16.0f * (float)(1 << l);
            } else if (l == 0) okf[i] = 0;
        }
        printf("[tb] L%d 跟踪 %d/%d\n", l, ntr, N);
    }
    int ok = 0; double sum = 0;
    for (int i = 0; i < N; i++) {
        if (!okf[i] || !rst[i]) continue;
        double du = uf[i] - ru[i], dv = vf[i] - rv[i];
        sum += sqrt(du * du + dv * dv);
        if (sqrt(du * du + dv * dv) < 1.0) ok++;
    }
    printf("[tb] <1px 一致 %d, 平均误差 %.3f px\n", ok, sum / (N ? N : 1));
    return 0;
}
