// nms_tb.cpp —— nms_min_dist 的等价性对拍台：与 Python 参考实现的输出**逐点、逐序**比较。
// 数据由板端 collect_resp.py 导出：resp_XXXX.bin（原始响应图）、ref_XXXX.txt（参考点集）、
// meta_XXXX.txt（thr / n_cand / n_pts）。图形参数与板端一致：480×640、border=10、min_dist=20、max_n=60。
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

// 直接声明（不引头文件）：g++ 与 HLS 都用同一份函数签名
typedef unsigned int u32;
void nms_min_dist(const u32 *resp, u32 *out_xy, u32 *out_n,
                  int rows, int cols, int thr, int min_dist, int border, int max_n);

int main(int argc, char **argv)
{
    const int ROWS = 480, COLS = 640, BORDER = 10, MIND = 20, MAXN = 60;
    const char *dir = (argc > 1) ? argv[1] : ".";
    int frames[] = {0, 60, 120, 180, 240, 300};
    int tot_ok = 0, tot_bad = 0;

    for (int fi = 0; fi < 6; ++fi) {
        char fn[512];
        std::vector<unsigned char> img((size_t)ROWS * COLS);
        snprintf(fn, sizeof(fn), "%s/resp_%04d.bin", dir, frames[fi]);
        FILE *f = fopen(fn, "rb");
        if (!f) { printf("缺 %s\n", fn); return 1; }
        if (fread(img.data(), 1, img.size(), f) != img.size()) { printf("读短 %s\n", fn); return 1; }
        fclose(f);

        int thr = 0, n_cand = 0, n_pts = 0;
        snprintf(fn, sizeof(fn), "%s/meta_%04d.txt", dir, frames[fi]);
        f = fopen(fn, "r");
        if (!f) { printf("缺 %s\n", fn); return 1; }
        if (fscanf(f, "%d %d %d", &thr, &n_cand, &n_pts) != 3) { printf("meta 坏\n"); return 1; }
        fclose(f);

        std::vector<int> refx, refy;
        snprintf(fn, sizeof(fn), "%s/ref_%04d.txt", dir, frames[fi]);
        f = fopen(fn, "r");
        if (!f) { printf("缺 %s\n", fn); return 1; }
        for (int i = 0; i < n_pts; ++i) {
            int x, y;
            if (fscanf(f, "%d %d", &x, &y) != 2) break;
            refx.push_back(x);
            refy.push_back(y);
        }
        fclose(f);

        // 输入按 32 位字打包（4 像素/字，小端），与 IP 读 DDR 的方式一致
        std::vector<unsigned int> packed((size_t)ROWS * COLS / 4, 0u);
        for (size_t i = 0; i < img.size(); ++i)
            packed[i >> 2] |= ((unsigned int)img[i]) << (8 * (i & 3));

        unsigned int out_xy[MAXN * 2 + 8];
        unsigned int out_n[2] = {0, 0};
        for (int i = 0; i < (int)(sizeof(out_xy) / sizeof(out_xy[0])); ++i) out_xy[i] = 0xDEADBEEFu;

        nms_min_dist((const u32 *)packed.data(), (u32 *)out_xy, (u32 *)out_n,
                     ROWS, COLS, thr, MIND, BORDER, MAXN);

        int got = (int)(out_n[0] & 0x7FFFFFFF);
        int ovf = (int)((out_n[0] >> 31) & 1);
        int bad = 0;
        if (got != n_pts) bad++;
        for (int i = 0; i < got && i < n_pts; ++i) {
            int x = (int)(out_xy[i] & 0xFFFF);
            int y = (int)((out_xy[i] >> 16) & 0xFFFF);
            if (x != refx[i] || y != refy[i]) {
                if (bad < 4)
                    printf("    #%d 核(%d,%d) vs 参考(%d,%d)\n", i, x, y, refx[i], refy[i]);
                bad++;
            }
        }
        printf("帧 %3d: 候选 %4d 阈值 %2d | 核 %2d 点（溢出 %d）参考 %2d 点 | %s\n",
               frames[fi], n_cand, thr, got, ovf, n_pts, bad ? "不一致" : "逐点逐序相同");
        if (bad) tot_bad++; else tot_ok++;
    }
    printf("\n等价性：%d/%d 帧完全一致\n", tot_ok, tot_ok + tot_bad);
    return tot_bad ? 1 : 0;
}
