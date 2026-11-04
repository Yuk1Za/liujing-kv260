// =====================================================================================
// test_pack_kernel.cpp —— 真实一帧（S2M1 dump）上的「pack → 内核」端到端对拍
//
// 为什么需要这个（在合成数据测试之外）：
//   已有的 tb_s2m_vmap.cpp 用的是**随机合成表**，它验证的是内核内部的算术与选格逻辑。
//   但从 CPU 到 PL 真正要走的那条链路是：
//       哈希表（keys/valid/4float）   --PlWin::pack-->   位图 + 平铺 tab    --内核-->
//   这条链路的**布局**（窗口格索引怎么映射、平铺下标怎么算、位图 32 位字怎么打包）
//   在整个测试里从没端到端跑过 —— 一旦错位，表现是"板上命中率莫名很低"，
//   而 C 仿真永远是绿的。这正是在板子回来之前必须消掉的风险。
//
// 数据：work_mapopt/a8/s2m_dump.bin（A4 时代 MAP_DUMP 落的真实一帧）
//   magic 'S2M1' | ver | n[4] | pose[6] | 4 个点云（各 4 float/点）
//   pose = [roll, pitch, yaw, tx, ty, tz]（与 transformTobeMapped 同序）
//
// ⚠️ 诚实的局限：dump 里是 **DS 后的局部地图云**，不是 A3 桶的原始点，
//    所以按 A8 的口径建表时有效平面数远少于生产（A8 生产用的是原始点二阶矩）。
//    本测试的判据是**链路一致性**（同一份表，内核与 float 参考给出同样的 H/b 与解），
//    不是精度、不是命中率。
// =====================================================================================
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>
#include <map>
#include <algorithm>
#include <Eigen/Dense>

#include "s2m_vmap.hpp"
#include "pl_s2m.hpp"

// 内核里的功能仿真计数（s2m_vmap.cpp 的 #ifndef __SYNTHESIS__ 分支 extern 了它们）
long g_nhit = 0, g_nrescan = 0, g_nrescue = 0;
// ---------------- dump 读取 ----------------
struct Dump
{
    int   n[4] = {0, 0, 0, 0};
    float pose[6] = {0, 0, 0, 0, 0, 0};
    std::vector<Eigen::Vector4f> pc[4];   // 0=corMap 1=surMap 2=corQry 3=surQry
};

static bool read_dump(const char *path, Dump &d)
{
    FILE *f = fopen(path, "rb");
    if (!f) { printf("打不开 %s\n", path); return false; }
    int32_t magic = 0, ver = 0;
    if (fread(&magic, 4, 1, f) != 1 || fread(&ver, 4, 1, f) != 1) { fclose(f); return false; }
    if (magic != 0x314D3253) { printf("magic 不对：0x%08x\n", (unsigned)magic); fclose(f); return false; }
    if (fread(d.n, 4, 4, f) != 4 || fread(d.pose, 4, 6, f) != 6) { fclose(f); return false; }
    for (int c = 0; c < 4; ++c)
    {
        d.pc[c].resize(d.n[c]);
        for (int i = 0; i < d.n[c]; ++i)
        {
            float v[4];
            if (fread(v, 4, 4, f) != 4) { fclose(f); return false; }
            d.pc[c][i] = Eigen::Vector4f(v[0], v[1], v[2], v[3]);
        }
    }
    fclose(f);
    return true;
}

// ---------------- 与 pcl::getTransformation 同式：R = Rx*Ry*Rz ----------------
static Eigen::Matrix3f rot_from_pose(const float *p)
{
    const float roll = p[0], pitch = p[1], yaw = p[2];
    Eigen::AngleAxisf ax(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf ay(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf az(yaw, Eigen::Vector3f::UnitZ());
    return (ax * ay * az).toRotationMatrix();
}

// ---------------- 建平面表（与 A8 同口径：λ0 ≤ lam·λ1）----------------
struct PlaneTable
{
    std::vector<int64_t>       keys;
    std::vector<unsigned char> valid;
    std::vector<float>         src;      // 4 float/格
    float leaf = 1.6f;
    long  ncells = 0, nvalid = 0;
    float rms_sum = 0.f;
};

static int64_t pack_key(int x, int y, int z)
{
    return (int64_t)(x & 0x1FFFFF) | ((int64_t)(y & 0x1FFFFF) << 21) | ((int64_t)(z & 0x1FFFFF) << 42);
}

static void build_planes(const std::vector<Eigen::Vector4f> &pts, float leaf, int minpts, float lam,
                         PlaneTable &t)
{
    t.keys.clear(); t.valid.clear(); t.src.clear();
    t.leaf = leaf;
    const float inv = 1.f / leaf;
    struct Agg
    {
        int n = 0;
        double sx = 0, sy = 0, sz = 0;
        double sxx = 0, sxy = 0, sxz = 0, syy = 0, syz = 0, szz = 0;
    };
    std::map<int64_t, Agg> acc;
    for (size_t i = 0; i < pts.size(); ++i)
    {
        const float x = pts[i].x(), y = pts[i].y(), z = pts[i].z();
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
        const int cx = (int)std::floor(x * inv), cy = (int)std::floor(y * inv), cz = (int)std::floor(z * inv);
        Agg &a = acc[pack_key(cx, cy, cz)];
        a.n++;
        a.sx += x; a.sy += y; a.sz += z;
        a.sxx += (double)x * x; a.sxy += (double)x * y; a.sxz += (double)x * z;
        a.syy += (double)y * y; a.syz += (double)y * z; a.szz += (double)z * z;
    }
    double rms = 0.0;
    for (auto &kv : acc)
    {
        const Agg &a = kv.second;
        float nx = 0, ny = 0, nz = 0, dd = 0;
        unsigned char ok = 0;
        if (a.n >= minpts)
        {
            const double mx = a.sx / a.n, my = a.sy / a.n, mz = a.sz / a.n;
            Eigen::Matrix3f cov;
            cov << (float)(a.sxx / a.n - mx * mx), (float)(a.sxy / a.n - mx * my), (float)(a.sxz / a.n - mx * mz),
                   (float)(a.sxy / a.n - mx * my), (float)(a.syy / a.n - my * my), (float)(a.syz / a.n - my * mz),
                   (float)(a.sxz / a.n - mx * mz), (float)(a.syz / a.n - my * mz), (float)(a.szz / a.n - mz * mz);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
            if (es.info() == Eigen::Success)
            {
                const float l0 = es.eigenvalues()(0), l1 = es.eigenvalues()(1);
                if (l1 > 1e-12f && l0 <= lam * l1)
                {
                    Eigen::Vector3f v = es.eigenvectors().col(0);
                    const float vn = v.norm();
                    if (vn > 1e-12f)
                    {
                        v /= vn;
                        nx = v.x(); ny = v.y(); nz = v.z();
                        dd = -(float)(nx * mx + ny * my + nz * mz);
                        ok = 1;
                        rms += std::sqrt(std::max(0.0, (double)l0));
                    }
                }
            }
        }
        t.keys.push_back(kv.first);
        t.valid.push_back(ok);
        t.src.push_back(nx); t.src.push_back(ny); t.src.push_back(nz); t.src.push_back(dd);
        if (ok) t.nvalid++;
    }
    t.ncells = (long)t.keys.size();
    t.rms_sum = t.nvalid ? (float)(rms / t.nvalid) : 0.f;
}

// ---------------- 解 H dx = b（高斯消元 + 部分主元）----------------
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

int main(int argc, char **argv)
{
    const char *dump = (argc > 1) ? argv[1] : "s2m_dump.bin";
    const float leaf = (argc > 2) ? (float)atof(argv[2]) : 1.6f;
    const int   minpts = (argc > 3) ? atoi(argv[3]) : 4;
    const float lam = (argc > 4) ? (float)atof(argv[4]) : 0.3f;
    const float rmax = (argc > 5) ? (float)atof(argv[5]) : 0.4f;

    Dump d;
    if (!read_dump(dump, d)) return 1;
    printf("dump: corMap=%d surMap=%d corQry=%d surQry=%d\n", d.n[0], d.n[1], d.n[2], d.n[3]);

    // ---- R|t 与 J 系数 ----
    const Eigen::Matrix3f R = rot_from_pose(d.pose);
    const Eigen::Vector3f T(d.pose[3], d.pose[4], d.pose[5]);
    float parq[12];
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) parq[r * 3 + c] = R(r, c);
    parq[9] = T.x(); parq[10] = T.y(); parq[11] = T.z();

    // M 系数：calc_M 要 (srx,crx,sry,cry,srz,crz)
    const float roll = d.pose[0], pitch = d.pose[1], yaw = d.pose[2];
    float sc[6] = {std::sin(roll), std::cos(roll), std::sin(pitch), std::cos(pitch), std::sin(yaw), std::cos(yaw)};
    float parm[27];
    pls2m::calc_M(sc, parm);

    // ---- 建平面表（数据源：surMap）----
    PlaneTable pt;
    build_planes(d.pc[1], leaf, minpts, lam, pt);
    printf("平面表: leaf=%.2f minpts=%d lam=%.2f => cells=%ld valid=%ld (%.1f%%) 平面rms=%.1f mm\n",
           leaf, minpts, lam, pt.ncells, pt.nvalid,
           pt.ncells ? 100.0 * pt.nvalid / pt.ncells : 0.0, pt.rms_sum * 1000.f);

    // ---- 查询点：传感器系 + 世界系 ----
    const int nq = d.n[3];
    std::vector<float> q4((size_t)nq * 4), qs((size_t)nq * 3), ps((size_t)nq * 3);
    for (int i = 0; i < nq; ++i)
    {
        const Eigen::Vector3f q = d.pc[3][i].head<3>();
        const Eigen::Vector3f p = R * q + T;
        q4[4 * i + 0] = q.x(); q4[4 * i + 1] = q.y(); q4[4 * i + 2] = q.z(); q4[4 * i + 3] = 1.f;
        qs[3 * i + 0] = q.x(); qs[3 * i + 1] = q.y(); qs[3 * i + 2] = q.z();
        ps[3 * i + 0] = p.x(); ps[3 * i + 1] = p.y(); ps[3 * i + 2] = p.z();
    }

    // ---- 定窗口：以查询点世界坐标包围盒中心为原点，尺寸自动取到 65536 格以内 ----
    float mnx = 1e30f, mxx = -1e30f, mny = 1e30f, mxy = -1e30f, mnz = 1e30f, mxz = -1e30f;
    for (int i = 0; i < nq; ++i)
    {
        mnx = std::min(mnx, ps[3 * i + 0]); mxx = std::max(mxx, ps[3 * i + 0]);
        mny = std::min(mny, ps[3 * i + 1]); mxy = std::max(mxy, ps[3 * i + 1]);
        mnz = std::min(mnz, ps[3 * i + 2]); mxz = std::max(mxz, ps[3 * i + 2]);
    }
    const float cx = 0.5f * (mnx + mxx), cy = 0.5f * (mny + mxy), cz = 0.5f * (mnz + mxz);
    // 需求格数 + 每边留 1 格余量（27 邻域边界）
    int needx = (int)std::ceil((mxx - mnx) / leaf) + 3;
    int needy = (int)std::ceil((mxy - mny) / leaf) + 3;
    int needz = (int)std::ceil((mxz - mnz) / leaf) + 3;
    // 按 S2M_VBLEN 约束等比压缩（只压 x，因为 x 跨度最大）
    const long cap = S2M_VBLEN * 8L;
    printf("查询点世界系范围: x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f]  => 需要 %dx%dx%d 格 (cap=%ld)\n",
           mnx, mxx, mny, mxy, mnz, mxz, needx, needy, needz, cap);
    while ((long)needx * needy * needz > cap) needx -= 2;
    if (needx < 8) { printf("窗口装不下，放弃\n"); return 1; }
    const int dimx = needx, dimy = needy, dimz = needz;
    const int ocx = (int)std::floor(cx / leaf) - dimx / 2;
    const int ocy = (int)std::floor(cy / leaf) - dimy / 2;
    const int ocz = (int)std::floor(cz / leaf) - dimz / 2;
    printf("窗口: %dx%dx%d = %ld 格, org=(%.2f,%.2f,%.2f)\n",
           dimx, dimy, dimz, (long)dimx * dimy * dimz,
           (float)ocx * leaf, (float)ocy * leaf, (float)ocz * leaf);

    // ---- pack：哈希表 → 位图 + 平铺表 ----
    pls2m::PlWin w;
    w.reset(dimx, dimy, dimz, leaf, ocx, ocy, ocz);
    w.pack(pt.keys, pt.valid, pt.src.data(), 4);
    printf("pack: 表内 valid=%ld, 拷进窗口 %ld 格 (窗口 %ld 格)\n",
           pt.nvalid, w.nvalid, (long)dimx * dimy * dimz);

    // 位图 → 32 位字（内核吃 ap_uint<32>，长度 S2M_VBLEN/4；小端下与按字节的布局逐位等价）
    std::vector<ap_uint<32>> vbw(S2M_VBLEN / 4, 0);
    std::memcpy((void *)vbw.data(), w.vb.data(), std::min(w.vb.size(), (size_t)S2M_VBLEN));

    // ---- 参考（float，与 A8 的 query 同序）----
    pls2m::RefOut ro;
    pls2m::ref_Hb(w, qs.data(), ps.data(), nq, parm, 0, rmax, ro);
    printf("参考: hit=%ld/%d (%.1f%%)  miss(cell=%ld rmax=%ld s=%ld)  |r|均值=%.4f  diffcell=%ld\n",
           ro.nhit, nq, 100.0 * ro.nhit / nq, ro.nmiss_cell, ro.nmiss_rmax, ro.nmiss_s,
           ro.sum_abs_r / std::max(1L, ro.nhit), ro.ndiff_cell);

    // ---- 内核（rescan=1，A8 语义）----
    std::vector<float> Hb(27, 0.f);
    s2m_vmap(q4.data(), vbw.data(), w.tab.data(), parq, parm, Hb.data(),
             nq, 0, leaf, 1.f / leaf, rmax,
             w.orgx, w.orgy, w.orgz, dimx, dimy, dimz, 1);

    // ---- 比较 ----
    double Hk[6][6], bk6[6], Hr[6][6], br[6], xk[6] = {0}, xr[6] = {0};
    unpack_Hb(Hb.data(), Hk, bk6);
    for (int i = 0; i < 6; ++i)
    {
        for (int j = i; j < 6; ++j)
        {
            const int n = (i * 6 - i * (i - 1) / 2) + (j - i);
            Hr[i][j] = Hr[j][i] = (double)ro.H[n];
        }
        br[i] = (double)ro.b[i];
    }
    double nH = 0, dH = 0, nb = 0, db = 0;
    for (int i = 0; i < 6; ++i)
    {
        for (int j = i; j < 6; ++j) { nH += Hr[i][j] * Hr[i][j]; dH += (Hk[i][j] - Hr[i][j]) * (Hk[i][j] - Hr[i][j]); }
        nb += br[i] * br[i]; db += (bk6[i] - br[i]) * (bk6[i] - br[i]);
    }
    const double relH = std::sqrt(dH) / (std::sqrt(nH) + 1e-30);
    const double relb = std::sqrt(db) / (std::sqrt(nb) + 1e-30);
    const bool sok = solve6(Hk, bk6, xk) && solve6(Hr, br, xr);
    double dx_inf = 0;
    if (sok) for (int i = 0; i < 6; ++i) dx_inf = std::max(dx_inf, std::fabs(xk[i] - xr[i]));

    printf("\n=== 内核 vs 参考 ===\n");
    printf("H[0][0]  kern=%.6e  ref=%.6e\n", Hk[0][0], Hr[0][0]);
    printf("H[3][3]  kern=%.6e  ref=%.6e\n", Hk[3][3], Hr[3][3]);
    printf("H[5][5]  kern=%.6e  ref=%.6e\n", Hk[5][5], Hr[5][5]);
    printf("b[0]     kern=%.6e  ref=%.6e\n", bk6[0], br[0]);
    printf("b[5]     kern=%.6e  ref=%.6e\n", bk6[5], br[5]);
    printf("relH=%.3e   relb=%.3e   解 δx 绝对差(inf)=%.3e %s\n",
           relH, relb, dx_inf, sok ? "" : "(H 奇异)");

    // ★ 判据以「对解 δx 的绝对影响」为主（与 pl/fx_check.py 同口径）：
    //   LOAM 的位姿精度是 cm 级，δx 是 LM 的迭代增量；0.1 mm 的增量误差比它小两个数量级即为安全。
    //   relH 只作辅助观察 —— 真实数据下 H 的元素跨 3 个数量级（H[0][0]~3e5 vs H[5][5]~2e2），
    //   Frobenius 相对差被大元素主导，F12（0.24 mm）量化在 |p|~100 m 时给出 ~5e-3 是正常量级，
    //   **不是布局/链路错误**（合成数据测试里 |p|≤200 m 时 relH 也只到 1e-5 量级，那是分布均匀的结果）。
    const bool ok = sok && (dx_inf <= 1e-3);
    printf("\n判据（解 δx<=1mm，relH 仅观察）：%s\n", ok ? "PASS" : "FAIL");
    if (!ok) printf("  （若 δx 超标，先查 hit 数是否与参考一致；若 hit 也对不上，才是布局错位）\n");

    // ---- 可选：把这份真实用例导出成板端 bring-up 用的 bin（S2M_EXPORT=<目录>）----
    //   导出后板上就能用**同一份真实表**测时间，从而拿到"生产口径"的拍数，
    //   而不是用随机合成表的悲观结果。
    if (const char *ex = std::getenv("S2M_EXPORT"))
    {
        printf("\n=== 导出到 %s ===\n", ex);
        auto wr = [&](const char *n, const void *p, size_t bytes)
        {
            char path[512];
            snprintf(path, sizeof(path), "%s/%s", ex, n);
            FILE *g = fopen(path, "wb");
            if (!g) { printf("  写不了 %s\n", path); return; }
            fwrite(p, 1, bytes, g);
            fclose(g);
            printf("  %-11s %zu B\n", n, bytes);
        };
        wr("q.bin", q4.data(), (size_t)nq * 4 * sizeof(float));
        wr("vbw.bin", (const void *)vbw.data(), (size_t)(S2M_VBLEN / 4) * 4);
        // ★ 必须用 w.tab.size()：真实窗口可能远大于编译期 WMAX（32*32*8=8192 格），
        //   按 WMAX*6 导出会把 dim>8192 格的表项全截掉 ⇒ 板上命中率断崖式下降。
        wr("tab.bin", w.tab.data(), w.tab.size() * sizeof(float));
        wr("parq.bin", parq, 12 * sizeof(float));
        wr("parm.bin", parm, 27 * sizeof(float));
        float hbref[27];
        for (int i = 0; i < 21; ++i) hbref[i] = (float)ro.H[i];
        for (int i = 0; i < 6; ++i) hbref[21 + i] = (float)ro.b[i];
        wr("hbref.bin", hbref, 27 * sizeof(float));

        char mp[512];
        snprintf(mp, sizeof(mp), "%s/meta.txt", ex);
        if (FILE *g = fopen(mp, "w")) {
            fprintf(g, "nq %d\nleaf %.6f\nrmax %.6f\ndimx %d\ndimy %d\ndimz %d\n"
                       "orgx %.6f\norgy %.6f\norgz %.6f\nnhit %ld\nnqry %d\n",
                    nq, leaf, rmax, dimx, dimy, dimz,
                    (float)ocx * leaf, (float)ocy * leaf, (float)ocz * leaf, ro.nhit, nq);
            fclose(g);
            printf("  meta.txt\n");
        }
    }
    return ok ? 0 : 1;
}
