// =====================================================================================
// host_kernel.cpp —— 用**内核源码本身**（s2m_vmap.cpp，g++ 编译）跑同一份真实帧数据
//
// 目的：定位 "PL vs CPU ref_Hb 不一致" 到底发生在哪一层
//   ① 若 主机内核 == PL      ⇒ PL 忠实复现内核，差异全在"内核定点 vs CPU float"，属设计取舍
//   ② 若 主机内核 != PL      ⇒ 硬件/接口有 bug
//   ③ 顺带打印 主机内核 vs golden(ref_Hb) 的偏差，量化定点代价
//
// 编译：
//   g++ -O2 -I. -I/tools/Xilinx/Vitis/2024.2/include -o /tmp/host_kernel host_kernel.cpp s2m_vmap.cpp
// 用法：
//   /tmp/host_kernel ~/s2mreal/small
// =====================================================================================
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <ap_int.h>

extern "C" void s2m_vmap(const float *q, const ap_uint<32> *vbw, const float *tab,
                         const float *parq, const float *parm, float *Hb,
                         int nq, int mode, float L, float invL, float rmax,
                         float orgx, float orgy, float orgz,
                         int dimx, int dimy, int dimz, int rescan);

// 内核里的调试计数器（定义在 tb 文件，这里自备）
long g_nhit = 0, g_nrescan = 0, g_nrescue = 0;

static std::string DIR;

static bool rd(const char *name, void *dst, size_t bytes)
{
    char p[512];
    std::snprintf(p, sizeof(p), "%s/%s", DIR.c_str(), name);
    FILE *f = std::fopen(p, "rb");
    if (!f) { std::printf("!! 打不开 %s\n", p); return false; }
    const size_t n = std::fread(dst, 1, bytes, f);
    std::fclose(f);
    if (n != bytes) { std::printf("!! 长度不符 %s\n", p); return false; }
    return true;
}

static double mval(const std::string &kv, const char *key, double def)
{
    for (int pass = 0; pass < 2; ++pass)
    {
        const std::string pat = std::string(" ") + key + (pass == 0 ? "=" : " ");
        const size_t i = kv.find(pat);
        if (i != std::string::npos) return std::atof(kv.c_str() + i + pat.size());
    }
    return def;
}

int main(int argc, char **argv)
{
    DIR = (argc > 1) ? argv[1] : std::string(std::getenv("HOME")) + "/s2mreal/small";

    char mp[512];
    std::snprintf(mp, sizeof(mp), "%s/meta.txt", DIR.c_str());
    FILE *fm = std::fopen(mp, "r");
    if (!fm) { std::printf("!! 打不开 %s\n", mp); return 1; }
    std::string kv;
    {
        char line[512];
        while (std::fgets(line, sizeof(line), fm)) { kv += " "; kv += line; }
    }
    std::fclose(fm);
    for (char &c : kv) if (c == '\n' || c == '\r' || c == '\t') c = ' ';

    const int nq = (int)mval(kv, "NQ", 0);
    const float L = (float)mval(kv, "L", 1.6);
    const float invL = (float)mval(kv, "invL", 1.0 / 1.6);
    const float rmax = (float)mval(kv, "rmax", 0.4);
    const int WMAX = (int)mval(kv, "WMAX", 8192);
    const int VW = (int)mval(kv, "vbw_words", 256);
    const int dimx = (int)mval(kv, "dimx", 32);
    const int dimy = (int)mval(kv, "dimy", 32);
    const int dimz = (int)mval(kv, "dimz", 8);
    // org 有两种写法：`org=x y z`（导出版）与 `orgx x` 分行（原始版）
    float orgx = 0.f, orgy = 0.f, orgz = 0.f;
    {
        const size_t i = kv.find(" org=");
        if (i != std::string::npos)
            std::sscanf(kv.c_str() + i + 5, "%f %f %f", &orgx, &orgy, &orgz);
        else
        {
            orgx = (float)mval(kv, "orgx", 0);
            orgy = (float)mval(kv, "orgy", 0);
            orgz = (float)mval(kv, "orgz", 0);
        }
    }
    std::printf("输入 nq=%d dim=%dx%dx%d L=%.3f rmax=%.3f org=(%.2f,%.2f,%.2f)\n",
                nq, dimx, dimy, dimz, L, rmax, orgx, orgy, orgz);

    std::vector<float> q((size_t)nq * 4), tab((size_t)WMAX * 6), parq(12), parm(27), gold(27);
    std::vector<uint32_t> vbw_u((size_t)VW);
    if (!rd("q.bin", q.data(), sizeof(float) * q.size())) return 1;
    if (!rd("tab.bin", tab.data(), sizeof(float) * tab.size())) return 1;
    if (!rd("vbw.bin", vbw_u.data(), sizeof(uint32_t) * vbw_u.size())) return 1;
    if (!rd("parq.bin", parq.data(), sizeof(float) * 12)) return 1;
    if (!rd("parm.bin", parm.data(), sizeof(float) * 27)) return 1;
    if (!rd("golden.bin", gold.data(), sizeof(float) * 27)) return 1;

    std::vector<ap_uint<32>> vbw((size_t)VW);
    for (int i = 0; i < VW; ++i) vbw[i] = ap_uint<32>(vbw_u[i]);

    std::vector<float> Hb(27, 0.f);
    s2m_vmap(q.data(), vbw.data(), tab.data(), parq.data(), parm.data(), Hb.data(),
             nq, 0, L, invL, rmax, orgx, orgy, orgz, dimx, dimy, dimz, 0);

    std::printf("\n--- Hb: golden(ref_Hb float) | 主机内核 ---\n");
    double mx = 0.0;
    int worst = -1;
    double num = 0.0, den = 0.0;
    for (int i = 0; i < 27; ++i)
    {
        const double g = gold[i], h = Hb[i];
        const double rel = std::fabs(h - g) / std::max(std::fabs(g), 1e-12);
        if (std::fabs(g) > 1e-6 && rel > mx) { mx = rel; worst = i; }
        num += g * h; den += g * g;
        std::printf("  %2d  %+.6e  %+.6e  rel=%.3e\n", i, g, h, rel);
    }
    std::printf("\n最大相对误差 %.4e (第%d项) | 最优尺度 k=%.6f | 内核命中 g_nhit=%ld\n",
                mx, worst, (den > 1e-30 ? num / den : 0.0), g_nhit);

    // 若有 golden_korder.bin（与内核同序的 float 参考），再对一次：这才是同一判定顺序下的比较
    {
        char gp[512];
        std::snprintf(gp, sizeof(gp), "%s/golden_korder.bin", DIR.c_str());
        FILE *fg = std::fopen(gp, "rb");
        if (fg)
        {
            std::vector<float> gk(27);
            std::fread(gk.data(), sizeof(float), 27, fg);
            std::fclose(fg);
            double mx2 = 0.0;
            int w2 = -1;
            double n2 = 0.0, d2 = 0.0;
            for (int i = 0; i < 27; ++i)
            {
                const double g = gk[i];
                const double rel = std::fabs(Hb[i] - g) / std::max(std::fabs(g), 1e-12);
                if (std::fabs(g) > 1e-6 && rel > mx2) { mx2 = rel; w2 = i; }
                n2 += g * Hb[i]; d2 += g * g;
            }
            std::printf("vs golden_korder(同序 float): 最大相对误差 %.4e (第%d项) | k=%.6f\n",
                        mx2, w2, (d2 > 1e-30 ? n2 / d2 : 0.0));
        }
    }
    return 0;
}
