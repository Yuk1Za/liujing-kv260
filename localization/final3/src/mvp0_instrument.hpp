// ============================================================================
// mvp0_instrument.hpp  —— MVP-0 单头文件插桩 + 自适应观测权重
//
// 用法（三步）：
//   1) 在 laserMapping.cpp 顶部加：  #include "mvp0_instrument.hpp"
//   2) 在 h_share_model() 结束前加： MVP0_OBSERVE(s, ekfom_data, lidar_end_time);
//   3) 编译加 -DMVP0_ENABLE（不加则全部为空操作，零副作用）
//   （第 2 步可由 tools/patch_fastlio2.py 自动完成）
//
// 环境变量：MVP0_OUT / MVP0_CONFIG / MVP0_MODE
//
// ---------------------------------------------------------------------------
// ★ 关键设计：分块处理（平移块 / 旋转块）
//
//   LiDAR 点到面信息矩阵 H = Σ [n; p×n][n; p×n]^T 存在【量纲失衡】：
//       平移块量级  ~ N
//       旋转块量级  ~ N·|p|²        （|p| 是点到雷达的距离）
//   20 m 距离时两者相差约 400 倍。因此用全局 trace/6 作为阈值会
//   被旋转项主导，阈值高到把【所有平移更新】都误杀。
//
//   解决：把 H 拆成 Htt (3x3 平移) 与 Hrr (3x3 旋转)，各自独立
//   特征分解 + 独立阈值。这与本课题的核心病根（隧道【沿程平移】
//   不可观测）完全对应。
//
//   kappa / lambda_min 仍按 6x6 全局计算，仅用于【观测与记录】，
//   不用于阈值判定。
// ---------------------------------------------------------------------------
//
// 重要说明（FAST-LIO2 结构限制）：
//   其更新接口只接受【标量】点噪声 LASER_POINT_COV，无法表达各向异性。
//   故：
//     形式1 = adaptive_point_cov()  -> 整体降权（一行改动，直接可用）
//     形式2 = remap_delta_pose()    -> 分块子空间投影（真正处理方向性）
// ============================================================================

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

#include <Eigen/Dense>

namespace mvp0 {

// ===========================================================================
// 配置
// ===========================================================================
enum class Mode { SCALAR = 0, MATRIX = 1, DIRECTIONAL = 2 };

struct Config {
    Mode   mode                 = Mode::SCALAR;
    // 自动阈值：floor = lambda_floor_scale * trace(块)/3（分块各自计算）
    // ★ MVP-0 结论：这个参数不是"传感器常数"，而是【场景级设计值】。
    //   它同时承担两个相差 4000 倍的职责：
    //     ① 排除虚假可观性（物理下界 2/kappa_sat ≈ 1.14e-4）
    //     ② 硬投影的启动阈值 kappa_crit = (n-f)/f
    //   f=0.1 → kappa_crit=19 ；f=0.5 → kappa_crit=3（覆盖所有真实退化）
    //   仿真 floor 扫描表明：有偏置时 f 大更优，且损失高度不对称（160x vs 1.6x）
    double lambda_floor_scale   = 0.5;
    double lambda_floor_tt      = -1.0;  // >0 时覆盖平移块自动阈值
    double lambda_floor_rr      = -1.0;  // >0 时覆盖旋转块自动阈值
    double kappa_ref            = 1.0;   // 标量模式参考条件数（正常段整定）
    double beta                 = 1.0;   // 标量模式增益
    double cov_max_scale        = 1e4;   // LASER_POINT_COV 放大上限
    double residual_sigma_scale = 1.0;   // 矩阵模式标度
    double R_max_scale          = 1e6;
    bool   enable_dump          = true;
    // ★ 形式2 首选实现：投影雅可比（在 h_share_model 内改 h_x，不动 esekfom）
    //   新的信息矩阵 = P^T (h_x^T h_x) P，只在可观测子空间更新，
    //   且协方差不会在不可观测方向上虚假收缩（优于投影增量 dx 的做法）
    bool   project_hx           = false;
    // 是否在投影前保留原始 h_x 的快照用于诊断（默认不保留）
    bool   dump_pre_projection  = true;

    // ---- ★ 场景级退化检测器（CUSUM + 迟滞）+ 慢时平均方向估计
    //   动机（MVP-0 仿真结论）：逐帧阈值判据会被"结构扰动"（横通道/标志牌/停车）骗到——
    //   扰动期间几何退化消失，判据判为可观测，但【已累积的沿程偏置并未消失】，
    //   偏置继续注入，隧道段 ATE 从 0.013 m 劣化到 1.93 m。
    //   改用「时域累积的退化状态 + 慢时平均的方向估计」后，
    //   反而优于"上帝视角的瞬时 oracle"（kappa=100 时好 117 倍）。
    bool   use_detector         = true;   // false = 退回逐帧阈值（退化基线，用于 A/B 对比）
    double cusum_delta          = 0.2;    // 漂移容限：m > f-δ 的扰动不触发退出
    double cusum_h_on           = 3.0;    // 进入/退出阈值（双向各一累加器）
    double cusum_s_max          = 50.0;   // ★ 累积上限【不能】等于 h_on！它决定能跨越多长扰动
    double hbar_a_fast          = 0.70;   // H̄ 信息量【下降】时的采纳速度（≈3 帧，消收敛延迟）
    double hbar_a_slow          = 0.99;   // H̄ 信息量【回升】时的采纳速度（≈100 帧，抗扰动）
    double f_phys               = 1.0e-4; // 安全下限 2/kappa_sat（C16 约 1.14e-4）
    // ★ m 的滤波权重。CUSUM 本身已是时域积分器，再叠前置低通会把延迟翻倍
    //   （实测 α=0.1 时快进延迟从 10 帧劣化到 28 帧、快出从 10 帧到 39 帧）。
    //   故默认 1.0 = 不滤波；如需抑制单帧毛刺可调小。
    double m_alpha              = 1.0;

    std::string out_dir         = "/tmp/mvp0";
};

inline void mkdir_p(const std::string& d) {
    if (d.empty()) return;
    std::string cur;
    for (size_t i = 0; i < d.size(); ++i) {
        cur += d[i];
        if (d[i] == '/' || i + 1 == d.size()) {
            if (cur != "/" && !cur.empty()) mkdir(cur.c_str(), 0755);
        }
    }
}

inline Mode parse_mode(const std::string& s) {
    if (s == "scalar")      return Mode::SCALAR;
    if (s == "matrix")      return Mode::MATRIX;
    if (s == "directional") return Mode::DIRECTIONAL;
    return Mode::SCALAR;
}

inline std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n\"");
    size_t b = s.find_last_not_of(" \t\r\n\"");
    return (a == std::string::npos) ? std::string() : s.substr(a, b - a + 1);
}

inline Config load_config(const std::string& path) {
    Config c;
    std::ifstream f(path.c_str());
    if (!f.is_open()) return c;
    std::string line;
    while (std::getline(f, line)) {
        size_t h = line.find('#');
        if (h != std::string::npos) line = line.substr(0, h);
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string k = trim(line.substr(0, eq)), v = trim(line.substr(eq + 1));
        if      (k == "mode")                 c.mode = parse_mode(v);
        else if (k == "lambda_floor_scale")   c.lambda_floor_scale = atof(v.c_str());
        else if (k == "lambda_floor_tt")      c.lambda_floor_tt = atof(v.c_str());
        else if (k == "lambda_floor_rr")      c.lambda_floor_rr = atof(v.c_str());
        else if (k == "kappa_ref" || k == "kappa0") c.kappa_ref = atof(v.c_str());
        else if (k == "beta")                 c.beta = atof(v.c_str());
        else if (k == "cov_max_scale")        c.cov_max_scale = atof(v.c_str());
        else if (k == "residual_sigma_scale") c.residual_sigma_scale = atof(v.c_str());
        else if (k == "R_max_scale")          c.R_max_scale = atof(v.c_str());
        else if (k == "project_hx")           c.project_hx = (atoi(v.c_str()) != 0 ||
                                                              v == "true" || v == "yes");
        else if (k == "dump_pre_projection")  c.dump_pre_projection = (atoi(v.c_str()) != 0 ||
                                                                      v == "true" || v == "yes");
        else if (k == "use_detector")         c.use_detector = (atoi(v.c_str()) != 0 ||
                                                                v == "true" || v == "yes");
        else if (k == "cusum_delta")          c.cusum_delta = atof(v.c_str());
        else if (k == "cusum_h_on")           c.cusum_h_on = atof(v.c_str());
        else if (k == "cusum_s_max")          c.cusum_s_max = atof(v.c_str());
        else if (k == "hbar_a_fast")          c.hbar_a_fast = atof(v.c_str());
        else if (k == "hbar_a_slow")          c.hbar_a_slow = atof(v.c_str());
        else if (k == "f_phys")               c.f_phys = atof(v.c_str());
        else if (k == "m_alpha")              c.m_alpha = atof(v.c_str());
        else if (k == "out_dir")              c.out_dir = v;
    }
    return c;
}

inline Config& config() {
    static Config c;
    static bool inited = false;
    if (!inited) {
        const char* cfgp = std::getenv("MVP0_CONFIG");
        if (cfgp) c = load_config(cfgp);
        const char* outp = std::getenv("MVP0_OUT");
        if (outp) c.out_dir = outp;
        const char* modep = std::getenv("MVP0_MODE");
        if (modep) c.mode = parse_mode(modep);
        inited = true;
    }
    return c;
}

// ===========================================================================
// (A) 退化判据（分块）
// ===========================================================================
struct Metrics {
    // ---- 全局 6x6（仅用于观测与记录，不用于阈值）
    Eigen::Matrix<double, 6, 1> lambda = Eigen::Matrix<double, 6, 1>::Zero();  // 降序
    double lambda_min = 0.0;
    double kappa = 0.0;
    double d = 0.0;
    double trace = 0.0;

    // ---- 平移块 Htt（本课题的核心）
    Eigen::Matrix3d Vtt = Eigen::Matrix3d::Identity();
    Eigen::Vector3d lam_tt = Eigen::Vector3d::Zero();   // 降序
    double lam_tt_min = 0.0;
    double lam_tt_floor = 0.0;
    double kappa_tt = 0.0;                              // 平移块条件数【推荐用这个】

    // ---- 旋转块 Hrr
    Eigen::Matrix3d Vrr = Eigen::Matrix3d::Identity();
    Eigen::Vector3d lam_rr = Eigen::Vector3d::Zero();   // 降序
    double lam_rr_min = 0.0;
    double lam_rr_floor = 0.0;
    double kappa_rr = 0.0;

    // ---- 平移块 Htt 的原始矩阵（供场景级检测器的 H̄ 快攻慢放使用）
    Eigen::Matrix3d Htt = Eigen::Matrix3d::Zero();

    // ---- 隧道专用量
    double I_along = NAN, I_min = NAN, A = NAN, a = NAN;

    int n_points = 0;
};

// 3x3 对称块：降序特征分解
inline void eig3_desc(const Eigen::Matrix3d& A,
                      Eigen::Vector3d& lam_desc, Eigen::Matrix3d& V_desc)
{
    Eigen::Matrix3d As = 0.5 * (A + A.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(As);
    Eigen::Vector3d lam = es.eigenvalues();          // 升序
    Eigen::Matrix3d V = es.eigenvectors();
    for (int i = 0; i < 3; ++i) {
        lam_desc(i) = lam(2 - i);
        V_desc.col(i) = V.col(2 - i);
    }
}

inline Metrics analyze(const Eigen::Matrix<double, 6, 6>& H,
                       const Eigen::Vector3d& vel,
                       int n_points = 0)
{
    Metrics m;
    m.n_points = n_points;

    Eigen::Matrix<double, 6, 6> Hs = 0.5 * (H + H.transpose());
    m.trace = Hs.trace();

    // ---- 全局 6x6（记录用）
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> > es6(Hs);
    Eigen::Matrix<double, 6, 1> lam = es6.eigenvalues();   // 升序
    for (int i = 0; i < 6; ++i) m.lambda(i) = lam(5 - i);
    const double lmax = std::max(m.lambda(0), 1e-30);
    const double lmin = std::max(m.lambda(5), 1e-30);
    m.lambda_min = m.lambda(5);
    m.kappa = lmax / lmin;
    m.d = m.lambda(5) / lmax;

    // ---- 平移块
    Eigen::Matrix3d Htt = Hs.block<3, 3>(0, 0);
    m.Htt = Htt;
    eig3_desc(Htt, m.lam_tt, m.Vtt);
    m.lam_tt_min = m.lam_tt(2);
    m.kappa_tt = std::max(m.lam_tt(0), 1e-30) / std::max(m.lam_tt(2), 1e-30);
    m.lam_tt_floor = (config().lambda_floor_tt > 0)
                   ? config().lambda_floor_tt
                   : config().lambda_floor_scale * (Htt.trace() / 3.0);

    // ---- 旋转块
    Eigen::Matrix3d Hrr = Hs.block<3, 3>(3, 3);
    eig3_desc(Hrr, m.lam_rr, m.Vrr);
    m.lam_rr_min = m.lam_rr(2);
    m.kappa_rr = std::max(m.lam_rr(0), 1e-30) / std::max(m.lam_rr(2), 1e-30);
    m.lam_rr_floor = (config().lambda_floor_rr > 0)
                   ? config().lambda_floor_rr
                   : config().lambda_floor_scale * (Hrr.trace() / 3.0);

    // ---- 隧道专用量（基于平移块）
    m.I_min = m.lam_tt_min;
    const double vn = vel.norm();
    if (vn > 1e-6) {
        const Eigen::Vector3d u = vel / vn;
        m.I_along = u.dot(Htt * u);
        const double tr = Htt.trace();
        m.A = (std::fabs(tr) > 1e-30) ? (m.I_along / tr) : NAN;
        m.a = std::fabs(u.dot(m.Vtt.col(2)));        // Vtt.col(2) = 最小特征方向
    }
    return m;
}

// ===========================================================================
// 检测器诊断快照（POD）——定义前置，供 dump() 使用；实现见 (D) 节
// ===========================================================================
struct DetectorSnapshot {
    double m_est, m_smooth, S_in, S_out, floor_bar, lam_bar_min;
    int    deg, n_proj_tt;
};
DetectorSnapshot detector_snapshot();

// ===========================================================================
// 跨函数传递：h_share_model() 算出 H，但更新发生在 esekfom 内部。
// 这个单例把两者桥接起来。
//
// 线程安全说明：FAST-LIO2 的 LiDAR 更新是单线程串行的，故用 static 是安全的。
// 若你的 fork 是多线程的，需要改成每线程实例或加锁。
// ===========================================================================
struct SharedState {
    bool                       valid        = false;
    Metrics                    m;
    Eigen::Matrix<double, 6, 6> H            = Eigen::Matrix<double, 6, 6>::Zero();
    double                     base_cov     = 0.001;
    double                     adaptive_cov = 0.001;
};

inline SharedState& shared() {
    static SharedState s;
    return s;
}

// ===========================================================================
// (B) dump
// ===========================================================================
inline void dump(const Eigen::Matrix<double, 6, 6>& H,
                 const Eigen::Vector3d& vel,
                 double t,
                 const Metrics* m = NULL)
{
    Config& cfg = config();
    if (!cfg.enable_dump) return;

    static std::ofstream fh, fv, fm;
    if (!fh.is_open()) {
        mkdir_p(cfg.out_dir);
        fh.open((cfg.out_dir + "/H.csv").c_str());
        fh << "t";
        for (int i = 0; i < 36; ++i) fh << ",H" << i;
        fh << "\n";
        fv.open((cfg.out_dir + "/vel.csv").c_str());
        fv << "t,vx,vy,vz\n";
        fm.open((cfg.out_dir + "/metrics.csv").c_str());
        fm << "t,lambda_min,kappa,d,lam_tt_min,kappa_tt,lam_tt_floor,"
              "lam_rr_min,kappa_rr,lam_rr_floor,I_along,I_min,A,a,trace,n_points,"
              "m_est,m_smooth,S_in,S_out,deg,floor_bar,lam_bar_min,n_proj_tt\n";
    }

    char buf[1024];                  // 字段增多后需放宽，避免 -Wformat-truncation
    std::snprintf(buf, sizeof(buf), "%.6f", t);
    fh << buf;
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) {
            std::snprintf(buf, sizeof(buf), ",%.8e", H(r, c));
            fh << buf;
        }
    fh << "\n"; fh.flush();

    std::snprintf(buf, sizeof(buf), "%.6f,%.6f,%.6f,%.6f\n", t, vel(0), vel(1), vel(2));
    fv << buf; fv.flush();

    if (m) {
        const DetectorSnapshot ds = detector_snapshot();
        std::snprintf(buf, sizeof(buf),
            "%.6f,%.8e,%.6f,%.8e,%.8e,%.6f,%.8e,%.8e,%.6f,%.8e,%.8e,%.8e,%.6f,%.6f,%.8e,%d,"
            "%.8e,%.8e,%.6f,%.6f,%d,%.8e,%.8e,%d\n",
            t, m->lambda_min, m->kappa, m->d,
            m->lam_tt_min, m->kappa_tt, m->lam_tt_floor,
            m->lam_rr_min, m->kappa_rr, m->lam_rr_floor,
            m->I_along, m->I_min, m->A, m->a, m->trace, m->n_points,
            ds.m_est, ds.m_smooth, ds.S_in, ds.S_out,
            ds.deg, ds.floor_bar, ds.lam_bar_min, ds.n_proj_tt);
        fm << buf; fm.flush();
    }
}

// ===========================================================================
// (C1) 形式1：标量整体降权
//     推荐用【平移块条件数】kappa_tt 作为退化指标（空间量纲一致）
// ===========================================================================
inline double adaptive_point_cov(const Metrics& m, double base_cov)
{
    const Config& cfg = config();
    const double ref = std::max(1.0, cfg.kappa_ref);
    const double g = std::max(0.0, m.kappa_tt / ref - 1.0);
    double sc = 1.0 + cfg.beta * g;
    sc = std::min(sc, cfg.cov_max_scale);
    return base_cov * sc;
}

// ===========================================================================
// (C2) 形式2：分块 solution remapping
//      对平移、旋转分别投影到各自的可观测子空间。
//      dx_pose 顺序必须与 H 的列顺序一致：[平移(0:3); 旋转(3:6)]
// ===========================================================================
inline void remap_delta_pose(Eigen::Matrix<double, 6, 1>& dx_pose, const Metrics& m)
{
    Config& cfg = config();
    if (cfg.mode == Mode::SCALAR) return;

    // ---- 平移块
    Eigen::Matrix3d Ptt = Eigen::Matrix3d::Zero();
    int n_tt = 0;
    for (int i = 0; i < 3; ++i)
        if (m.lam_tt(i) > m.lam_tt_floor) { Ptt += m.Vtt.col(i) * m.Vtt.col(i).transpose(); ++n_tt; }
    dx_pose.head<3>() = (n_tt == 0) ? Eigen::Vector3d::Zero().eval()
                                    : (Ptt * dx_pose.head<3>()).eval();

    // ---- 旋转块
    Eigen::Matrix3d Prr = Eigen::Matrix3d::Zero();
    int n_rr = 0;
    for (int i = 0; i < 3; ++i)
        if (m.lam_rr(i) > m.lam_rr_floor) { Prr += m.Vrr.col(i) * m.Vrr.col(i).transpose(); ++n_rr; }
    dx_pose.tail<3>() = (n_rr == 0) ? Eigen::Vector3d::Zero().eval()
                                    : (Prr * dx_pose.tail<3>()).eval();
}

// 便利函数：返回可观测自由度个数（1..6），可用于日志
inline int observable_dof(const Metrics& m)
{
    int n = 0;
    for (int i = 0; i < 3; ++i) if (m.lam_tt(i) > m.lam_tt_floor) ++n;
    for (int i = 0; i < 3; ++i) if (m.lam_rr(i) > m.lam_rr_floor) ++n;
    return n;
}

// ===========================================================================
// 形式1 的接入点：在 FAST-LIO2 的更新调用处替换 LASER_POINT_COV
//   用法： kf.update_iterated_dyn_share_modified(MVP0_EFF_COV(LASER_POINT_COV), solve_H_time);
// ===========================================================================
inline double effective_point_cov(double base_cov)
{
    SharedState& s = shared();
    if (!s.valid) return base_cov;
    s.base_cov = base_cov;
    s.adaptive_cov = adaptive_point_cov(s.m, base_cov);
    return s.adaptive_cov;
}

// ===========================================================================
// 形式2 的接入点：在 esekfom 内、把增量写入状态之前调用
//   dx 的前 6 维顺序必须与 H 的列顺序一致：[平移(0:3); 旋转(3:6)]
// ===========================================================================
inline void remap_dx_pose6(Eigen::Matrix<double, 6, 1>& dx6)
{
    SharedState& s = shared();
    if (!s.valid) return;
    remap_delta_pose(dx6, s.m);
}

// 面向"完整误差状态向量"的便利包装（FAST-LIO2 的 dx 是 24 维或 23 维）
template <typename Derived>
inline void remap_dx_head6(Eigen::MatrixBase<Derived>& dx)
{
    if (dx.size() < 6) return;
    Eigen::Matrix<double, 6, 1> p = dx.template head<6>();
    remap_dx_pose6(p);
    dx.template head<6>() = p;
}

// ===========================================================================
// ★ 形式2 首选实现：投影雅可比
//
//   目标：只在【可观测子空间】内做更新。
//   做法：令 J' = J P  (P 为可观测子空间投影算子，对称幂等)
//         则 J'^T J' = P^T (J^T J) P = P H P
//         -> 不可观测方向的信息被清掉
//         -> 且 K 的列落在可观测子空间，协方差【不会】在不可观测方向虚假收缩
//
//   对比：投影增量 dx（remap_dx_head6）虽然也不更新该方向，但协方差仍会
//         被 (I-KH) 收缩 -> 滤波器过度自信 -> 后续真发生变化时纠正不回来。
//
//   工程优势：只需改 h_share_model 里的 h_x，完全不用碰 esekfom。
//
//   h_x 列序约定：0:3 = 平移，3:6 = 旋转（与 FAST-LIO2 一致，需自行核对）
// ===========================================================================
inline Eigen::Matrix<double, 6, 6> build_projection(const Metrics& m)
{
    Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();
    for (int i = 0; i < 3; ++i)
        if (m.lam_tt(i) > m.lam_tt_floor)
            P.block<3, 3>(0, 0) += m.Vtt.col(i) * m.Vtt.col(i).transpose();
    for (int i = 0; i < 3; ++i)
        if (m.lam_rr(i) > m.lam_rr_floor)
            P.block<3, 3>(3, 3) += m.Vrr.col(i) * m.Vrr.col(i).transpose();
    return P;
}

// 就地投影雅可比的前 6 列。返回是否做了投影。
inline bool project_hx_pose(Eigen::MatrixXd& h_x, const Metrics& m)
{
    if (h_x.cols() < 6 || h_x.rows() == 0) return false;
    const Eigen::Matrix<double, 6, 6> P = build_projection(m);
    h_x.leftCols(6) = h_x.leftCols(6) * P;
    return true;
}

// ===========================================================================
// (D) ★ 场景级退化检测器（双向 CUSUM + 迟滞）+ 慢时平均方向估计
//
// 动机（MVP-0 仿真实测）
//   逐帧阈值判据会被"结构扰动"（横通道/标志牌/停放车辆/灯具）骗到：
//   扰动期间几何退化确实消失，判据【正确地】判为"可观测"——但
//   【已累积的沿程偏置并没有消失】，偏置继续注入，隧道段 ATE 从
//   0.013 m 劣化到 1.93 m。这是【瞬时判据】与【累积误差】的错配。
//
//   改用「时域累积的退化状态 + 慢时平均的方向估计」后，反而优于
//   "上帝视角的瞬时 oracle"（kappa=100 时好 117 倍）。
//
// 三个必须遵守的设计点（都是仿真踩坑得来的）
//   ① s_max 【不能】等于 h_on —— 它决定检测器能跨越多长的结构扰动：
//        T_bridge = (s_max - 0)/|f - m_pert - delta|   [帧]
//      设小了 → 扰动期间被"骗"回非退化态 → 偏置继续污染。
//   ② 投影方向必须取自【观测几何 H̄】，不能取自【状态估计的行进方向】——
//      后者会与航向误差耦合形成正反馈，实测横向误差恶化 538%。
//   ③ H̄ 用【快攻慢放】：信息量下降立即采纳（消收敛延迟），回升缓慢采纳（抗扰动）。
// ===========================================================================
struct Detector {
    // ---- 状态
    double S_in     = 0.0;    // 进入累加器
    double S_out    = 0.0;    // 退出累加器（与进入独立 → 快进 + 快出 + 跨扰动）
    bool   deg      = false;  // 当前是否判为"退化中"
    int    n_frames = 0;

    double m_est    = 1.0;    // 瞬时退化测度 m = lam_min(Htt) / (trace(Htt)/3)
    double m_smooth = 1.0;    // 轻平滑后的 m（抑制单帧毛刺）

    // ---- 慢时平均信息矩阵（归一化 trace=1）及其特征分解
    Eigen::Matrix3d Hbar    = Eigen::Matrix3d::Zero();
    Eigen::Vector3d lam_bar = Eigen::Vector3d::Zero();     // 降序
    Eigen::Matrix3d V_bar   = Eigen::Matrix3d::Identity();

    int    n_proj_tt = 3;     // 上一帧保留的平移自由度数（诊断）
    double floor_bar = 0.0;   // 上一帧的 H̄ 阈值（诊断）

    // 有效阈值 = max(场景级设计值, 传感器物理下界 2/kappa_sat)
    double f_eff() const {
        const Config& cfg = config();
        return std::max(cfg.lambda_floor_scale, cfg.f_phys);
    }
};

inline Detector& detector()
{
    static Detector d;
    return d;
}

// 复位检测器（测试 / 重新开始一段数据时用）
inline void detector_reset() { detector() = Detector(); }

// 每帧调用一次（analyze() 之后、投影之前）
inline void detector_step(const Metrics& m)
{
    Detector& g = detector();
    const Config& cfg = config();

    // ---- 瞬时退化测度（与 analyze() 的 lam_tt_floor 同一量纲）
    //      m = λ_min(Htt) / (tr(Htt)/3)  →  1=各向同性，0=该方向被完全压制
    const double tr = m.lam_tt.sum();
    g.m_est = (tr > 1e-30) ? (m.lam_tt(2) / (tr / 3.0)) : 1.0;
    // CUSUM 已是时域积分器 → 默认不再前置滤波（m_alpha=1.0）
    g.m_smooth = (g.n_frames == 0)
               ? g.m_est
               : ((1.0 - cfg.m_alpha) * g.m_smooth + cfg.m_alpha * g.m_est);

    const double f = g.f_eff();

    // ---- 双向 CUSUM（进入 / 退出各用独立累加器）
    g.S_in  = std::min(cfg.cusum_s_max,
                       std::max(0.0, g.S_in  + ((f - g.m_smooth) - cfg.cusum_delta)));
    g.S_out = std::min(cfg.cusum_s_max,
                       std::max(0.0, g.S_out + ((g.m_smooth - f) - cfg.cusum_delta)));
    if (!g.deg) {
        g.S_out = 0.0;
        if (g.S_in >= cfg.cusum_h_on) g.deg = true;
    } else {
        g.S_in = 0.0;
        if (g.S_out >= cfg.cusum_h_on) g.deg = false;
    }

    // ---- H̄ 快攻慢放（归一化 trace=1；逐元素、纯几何）
    if (!cfg.use_detector) { g.deg = (g.m_smooth < f); }   // A/B 基线：逐帧阈值
    Eigen::Matrix3d Hn = m.Htt;
    const double trn = std::max(std::fabs(Hn.trace()), 1e-30);
    Hn = 0.5 * (Hn + Hn.transpose()) / trn;
    if (g.n_frames == 0) {
        g.Hbar = Hn;
    } else {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_n(Hn);
        const double lmin_n = es_n.eigenvalues()(0);
        const double lmin_b = g.lam_bar(2);
        const double a = (lmin_n < lmin_b) ? cfg.hbar_a_fast : cfg.hbar_a_slow;
        g.Hbar = a * g.Hbar + (1.0 - a) * Hn;
        g.Hbar = 0.5 * (g.Hbar + g.Hbar.transpose());
    }
    eig3_desc(g.Hbar, g.lam_bar, g.V_bar);
    ++g.n_frames;
}

// ===========================================================================
// (D2) 形式2 推荐实现：检测器触发 + H̄ 定方向 的雅可比投影
//      J' = J P  →  J'^T J' = P H P  →  不可观测方向的信息被清掉，
//      且 K 的列落在可观测子空间，协方差不会在不可观测方向虚假收缩。
// ===========================================================================
inline bool project_hx_detector(Eigen::MatrixXd& h_x, const Metrics& m)
{
    Detector& g = detector();
    g.n_proj_tt = 3;
    if (h_x.cols() < 6 || h_x.rows() == 0) return false;
    if (!g.deg) return false;                      // 未判为退化 → 不投影

    const double floor_bar = g.f_eff() / 3.0;      // trace(H̄)=1 → tr/3 = 1/3
    g.floor_bar = floor_bar;

    // ---- 平移块：方向取自 H̄（抗结构扰动，且与状态估计解耦）
    Eigen::Matrix3d Ptt = Eigen::Matrix3d::Zero();
    int n_tt = 0;
    for (int i = 0; i < 3; ++i)
        if (g.lam_bar(i) > floor_bar) {
            Ptt += g.V_bar.col(i) * g.V_bar.col(i).transpose();
            ++n_tt;
        }
    if (n_tt == 0) {                               // 全被切则保留最强方向
        Ptt = g.V_bar.col(0) * g.V_bar.col(0).transpose();
        n_tt = 1;
    }
    g.n_proj_tt = n_tt;

    // ---- 旋转块：仍用瞬时量（仓体旋转一般不退化；如需可另加 H̄rr）
    Eigen::Matrix3d Prr = Eigen::Matrix3d::Zero();
    int n_rr = 0;
    for (int i = 0; i < 3; ++i)
        if (m.lam_rr(i) > m.lam_rr_floor) {
            Prr += m.Vrr.col(i) * m.Vrr.col(i).transpose();
            ++n_rr;
        }
    if (n_rr == 0) Prr = m.Vrr.col(0) * m.Vrr.col(0).transpose();

    Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();
    P.block<3, 3>(0, 0) = Ptt;
    P.block<3, 3>(3, 3) = Prr;

    h_x.leftCols(6) = h_x.leftCols(6) * P;
    return true;
}

// 检测器状态快照（供 dump 使用；结构体与前置声明见 (A) 节之后）
inline DetectorSnapshot detector_snapshot()
{
    const Detector& g = detector();
    DetectorSnapshot s;
    s.m_est       = g.m_est;
    s.m_smooth    = g.m_smooth;
    s.S_in        = g.S_in;
    s.S_out       = g.S_out;
    s.floor_bar   = g.floor_bar;
    s.lam_bar_min = g.lam_bar(2);
    s.deg         = g.deg ? 1 : 0;
    s.n_proj_tt   = g.n_proj_tt;
    return s;
}

// ===========================================================================
// 主入口宏
// ===========================================================================
#ifdef MVP0_ENABLE

#define MVP0_OBSERVE(STATE, EKFDATA, T)                                        \
    do {                                                                       \
        if ((EKFDATA).valid && (EKFDATA).h_x.rows() > 0 &&                     \
            (EKFDATA).h_x.cols() >= 6) {                                       \
            Eigen::Matrix<double, 6, 6> _H =                                   \
                (EKFDATA).h_x.leftCols(6).transpose() *                        \
                (EKFDATA).h_x.leftCols(6);                                     \
            Eigen::Vector3d _v((STATE).vel(0), (STATE).vel(1), (STATE).vel(2));\
            ::mvp0::Metrics _m = ::mvp0::analyze(_H, _v, (EKFDATA).h_x.rows());\
            ::mvp0::detector_step(_m);           /* ★ 场景级退化检测器 */      \
            ::mvp0::dump(_H, _v, (T), &_m);      /* 投影前的真实 H，供诊断 */   \
            ::mvp0::SharedState& _ss = ::mvp0::shared();                       \
            _ss.valid = true; _ss.m = _m; _ss.H = _H;                          \
            MVP0_PROJECT_HX(EKFDATA);            /* 形式2：投影雅可比 */        \
        }                                                                      \
    } while (0)

// 形式2 首选：检测器触发 + H̄ 定方向的就地雅可比投影（受 config().project_hx 控制）
#define MVP0_PROJECT_HX(EKFDATA)                                               \
    do {                                                                       \
        if (::mvp0::config().project_hx &&                                     \
            (EKFDATA).h_x.cols() >= 6 && (EKFDATA).h_x.rows() > 0) {           \
            ::mvp0::project_hx_detector((EKFDATA).h_x, ::mvp0::shared().m);    \
        }                                                                      \
    } while (0)

// 更新调用点：替换 LASER_POINT_COV（形式1）
#define MVP0_EFF_COV(BASE) ::mvp0::effective_point_cov(BASE)

// 形式2 备选：增量写入状态前投影（需要改 esekfom，协方差一致性略差）
#define MVP0_REMAP_DX(DX)  ::mvp0::remap_dx_head6(DX)

#else

#define MVP0_OBSERVE(STATE, EKFDATA, T) do {} while (0)
#define MVP0_PROJECT_HX(EKFDATA)        do {} while (0)
#define MVP0_EFF_COV(BASE)              (BASE)
#define MVP0_REMAP_DX(DX)               do {} while (0)

#endif  // MVP0_ENABLE

}  // namespace mvp0
