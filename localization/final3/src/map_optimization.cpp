// 只用纯激光雷达，去掉IMU和GPS，估计激光里程计，可达到近似LIO-SAM的效果
// 参考LIO-SAM的mapOptimization.cpp和A-LOAM的laserMapping.cpp
// 将A-LOAM中的scan-to-scan的粗略激光里程计估计结果作为scan-to-map的初始值（原本的LIO-SAM采用的IMU预积分作为初始值）


// ROS 2
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// TF2（ROS2中推荐使用tf2_ros而非tf）
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>      // tf2::Transform
#include <tf2/LinearMath/Vector3.h>        // tf2::Vector3

// OpenCV
#include <opencv2/imgproc.hpp>

// Eigen（自伴随 3x3 特征分解，[A1] 用；定长、栈上、无堆分配）
#include <Eigen/Eigenvalues>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "pl_s2m.hpp"   // [PL v2a] CPU 侧驱动
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

// 标准库
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <chrono>
#include <map>
#include <memory>

// GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/linear/NoiseModel.h>     // [VIS] Robust / mEstimator::Cauchy
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

// 自定义工具
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "lo_sam/msg/cloud_info.hpp"
#include "mvp0_instrument.hpp"   // [MVP0] CUSUM 检测器 + H̄ 方向估计（来自 mvp0/tools）

// ============================================================================
// [MVP0-H-DUMP] 逐帧落盘 Gauss-Newton 正规方程 H = J^T J（scan-to-map 信息矩阵）
//   格式与 mvp0/tools/mvp0_instrument.hpp 的 H.csv 一致：
//   t,H00,...,H55（37 列行主序），外加 eig0..eig5（升序特征值，便于离线分析）
//   开关：设 MVP0_OUT=<dir> 启用；未设置则零开销跳过
// ============================================================================
static void mvp0DumpH(double t,
                      const Eigen::Matrix<float, 6, 6> &H,
                      const Eigen::Matrix<float, 1, 6> &eig)
{
    const char *out = std::getenv("MVP0_OUT");
    if (!out) return;
    static std::ofstream *fh = nullptr;
    if (!fh) {
        ::mkdir(out, 0755);
        fh = new std::ofstream(std::string(out) + "/H.csv");
        *fh << "t";
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) *fh << ",H" << i << j;
        for (int i = 0; i < 6; ++i) *fh << ",eig" << i;
        *fh << "\n";
    }
    *fh << std::fixed << std::setprecision(6) << t;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) *fh << ',' << H(i, j);
    for (int i = 0; i < 6; ++i) *fh << ',' << eig(i);
    *fh << '\n';
    fh->flush();
}


using namespace gtsam;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

int numberOfCores = 4;
//  A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)6D位姿点云结构定义
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY; // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))
typedef PointXYZIRPYT PointTypePose;

// typedef pcl::PointXYZI PointType;

using FactorParam = Eigen::Matrix<double, 3, 4>;

// =====================================================================================
// [A2] 体素栅格最近邻（voxel grid + 固定邻域）—— 替代 PCL FLANN 的 5-NN 树搜索
//
// 动机（详 `FPGA加速点-全管线候选分析.md` §12.4 / §12.5 第 2 条）：
//   scan-to-map 的 surf 168.6 + cor 117.9 ms 占 leaf0 单帧的 75.8%，全部耗在
//   `nearestKSearch(pointSel, 5, ...)` 上（每次 6–9 µs，A53）。树搜索是 FPGA 最不擅长的
//   形态；"体素化 + 固定邻域查表"正好相反 —— 这是**唯一能把 NN 变成 FPGA 友好形态**的路。
//
// ★ 语义保证（默认不改任何结果）：
//   设体素边长 L、查询点所在体素 C。任一与查询点距离 <= d 的点，其体素索引在每轴上与 C
//   相差 <= ceil(d/L)（长度 2d 的区间最多跨 ceil(d/L)+1 格）。故按 Chebyshev 半径
//   r = ceil(1.0/L) 展开全部 (2r+1)^3 个邻格，**必然**包含所有与查询点距离 <= 1.0 m 的点 ——
//   而原码的接受门限正是 `pointSearchSqDis[4] < 1.0`。⇒ 只要 d5 < 1.0，本实现给出的 5 个点
//   与 FLANN 的 5-NN **集合完全相同**（仅距离并列时的先后可能不同；拟合只用集合，不用次序）。
//   实际按 r=1,2,... 逐圈扩张，一旦 d5 <= r*L 立即返回（此时必然已精确）。
//
// 开关（默认 = 原版行为，不设变量时逐位等价）：
//   MAP_NN=voxel|kdtree   默认 kdtree
//   MAP_NN_LEAF=1.0       体素边长（m）；r 自动取 ceil(1.0/LEAF)
//                         ⇒ 1.0 时 r=1，**单圈 27 格就覆盖满 1.0 m**，一次查表定案
//                         （实测 0.5 ⇒ r=2、125 格、1.09 µs；1.0 ⇒ 27 格、0.36 µs，两者结果相同）
//   MAP_NN_CHECK=N        前 N 帧做一次 voxel vs kdtree 的逐点对拍（诊断用，勿用于计时）
// =====================================================================================
static const std::string A2_NN_MODE = []() {
        const char *v = std::getenv("MAP_NN");
        return std::string(v ? v : "kdtree");
    }();
static const bool A2_USE_VOXEL = (A2_NN_MODE == "voxel");
static const float A2_LEAF = []() {
        const char *v = std::getenv("MAP_NN_LEAF");
        float f = v ? (float)std::atof(v) : 1.0f;
        return f > 0.02f ? f : 1.0f;
    }();
static const int A2_RING = (int)std::ceil(1.0f / A2_LEAF); // 保证覆盖 1.0 m（= 原码门限）
static const int A2_CHECK_N = []() {
        const char *v = std::getenv("MAP_NN_CHECK");
        return v ? std::atoi(v) : 0;
    }();

// =====================================================================================
// [VIS] 视觉相对位姿因子（PnP-VO → ISAM2 位姿图）
//   VIS_FACTOR=1      启用（**默认 1**；显式 `VIS_FACTOR=0` 才关，用于纯激光对照）
//   VIS_SIG_ROT=0.06  旋转 sigma，rad/√s（按关键帧间隔开方放大）
//   VIS_SIG_TR=0.09   平移 sigma，m/√s
//   ★ sigma 默认值 = 板上 200 帧/122 m A/B 选出来的（`视觉因子接入-板上A-B实测.md`）。
//     视觉实测每对误差 ~0.04 m，0.05/0.02 那组是**过度信任**视觉：ATE 略好但朝向
//     从 0.65° 退化到 0.79°；0.09/0.06 组末位姿最好且朝向回到 0.69° ⇒ 取后者。
//   ★ 没有视觉节点在跑时（q 为空）每次插值都失败、直接跳过，**不加因子也不报错**，
//     所以默认开不会拖累纯雷达运行。
//   VIS_ROBUST=1.0    Cauchy 核 k（0=不加鲁棒核）
//   VIS_TOL=0.25      采样与目标时刻的最大容差（s），超出则该帧不加因子
//   VIS_MAXEXTRAP=2.5 末样本之后允许外推的最大倍数（× 采样间隔）
//   VIS_MAX_TR=3.0    单条约束平移上限（m），超过视为离谱直接丢
//   VIS_STAT=1        每 50 次打印一次「加/丢」统计
//   ★ 视觉位姿由 vo_pnp_node 以 **body(=velo) 系**发布（--out_frame velo），这里不碰外参。
//   ★ 因子加在 (n-1, n) 这对关键帧上，与激光里程计因子**并行**，A/B 时只差这一项。
// =====================================================================================
static const bool VIS_ON = []() {
        const char *v = std::getenv("VIS_FACTOR");
        // ★ 默认**开**（视觉因子是正常功能，不是实验开关）；要纯激光对照就显式 VIS_FACTOR=0。
        return !v || std::atoi(v) != 0;
    }();
static const double VIS_SIG_ROT = []() {
        const char *v = std::getenv("VIS_SIG_ROT");
        return v ? std::atof(v) : 0.06;
    }();
static const double VIS_SIG_TR = []() {
        const char *v = std::getenv("VIS_SIG_TR");
        return v ? std::atof(v) : 0.09;
    }();
static const double VIS_ROBUST = []() {
        const char *v = std::getenv("VIS_ROBUST");
        return v ? std::atof(v) : 1.0;
    }();
static const double VIS_TOL = []() {
        const char *v = std::getenv("VIS_TOL");
        return v ? std::atof(v) : 0.25;
    }();
static const double VIS_MAXEXTRAP = []() {
        const char *v = std::getenv("VIS_MAXEXTRAP");
        return v ? std::atof(v) : 2.5;
    }();
static const double VIS_MAX_TR = []() {
        const char *v = std::getenv("VIS_MAX_TR");
        return v ? std::atof(v) : 3.0;
    }();
static const bool VIS_STAT = []() {
        const char *v = std::getenv("VIS_STAT");
        return v && std::atoi(v) != 0;
    }();

// 有序键 + CSR 定长桶 + 开放寻址哈希索引：
//   `keys/beg/ids` 给出"体素 → 点索引"的连续分组（PL 侧就是"体素号直接寻址 + 桶内固定窗口"）；
//   `hkey/hval` 把"找体素"从 O(log n) 二分压成 1–2 次探测。
//   实测（服务器 x86, 2.5 万点地图）：二分 125 次探测 2.4 µs/查询 → 哈希 0.6 µs/查询。
struct VoxelGridNN
{
    static constexpr int64_t EMPTY = INT64_MIN;
    float  leaf = 1.0f;
    float  inv  = 1.0f;
    size_t npts = 0;
    int    hbits = 4;
    int    hmask = 15;
    const pcl::PointCloud<PointType> *pc = nullptr;
    std::vector<int64_t> keys;                // 每格键，升序（同时作哈希的校验值）
    std::vector<int>     beg;                 // CSR：nkeys+1
    std::vector<int>     ids;                 // CSR：按 key 分组的点索引
    std::vector<int64_t> hkey;                // 哈希槽 → 键
    std::vector<int>     hval;                // 哈希槽 → 桶号
    std::vector<std::pair<int64_t, int>> kv;  // build 暂存，复用容量免每帧 malloc
#ifdef A2_COUNT_CAND                          // 仅供 a2/test_voxelnn.cpp 统计候选点数，正式构建不定义
    mutable long dbg_cand = 0;
#endif

    static inline int64_t pack(int x, int y, int z)
    {
        return ((int64_t)(x & 0x1FFFFF))
             | ((int64_t)(y & 0x1FFFFF) << 21)
             | ((int64_t)(z & 0x1FFFFF) << 42);
    }
    inline int cellIdx(float v) const // = floor(v/L)，避开 std::floor 的调用开销
    {
        const float f = v * inv;
        const int   i = (int)f;
        return (f < (float)i) ? (i - 1) : i;
    }
    void build(const pcl::PointCloud<PointType> &c, float lf)
    {
        leaf = lf; inv = 1.0f / lf; pc = &c;
        keys.clear(); beg.clear(); ids.clear(); kv.clear();
        size_t m = 0;
        kv.reserve(c.points.size());
        for (size_t i = 0; i < c.points.size(); ++i)
        {
            const PointType &p = c.points[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            kv.emplace_back(pack(cellIdx(p.x), cellIdx(p.y), cellIdx(p.z)), (int)i);
            ++m;
        }
        npts = m;
        if (m == 0) { hkey.clear(); hval.clear(); return; }
        std::sort(kv.begin(), kv.end(),
                  [](const std::pair<int64_t, int> &a, const std::pair<int64_t, int> &b) {
                      return (a.first != b.first) ? (a.first < b.first) : (a.second < b.second);
                  });
        ids.resize(m);
        keys.reserve(m); beg.reserve(m + 1);
        int64_t cur = kv[0].first;
        keys.push_back(cur); beg.push_back(0);
        for (size_t t = 0; t < m; ++t)
        {
            if (kv[t].first != cur) { cur = kv[t].first; keys.push_back(cur); beg.push_back((int)t); }
            ids[t] = kv[t].second;
        }
        beg.push_back((int)m);

        // 哈希表：槽数取 >= 2*nkeys 的 2 的幂（装载因子 <= 0.5，线性探测 1–2 次命中）
        const size_t nc = keys.size();
        hbits = 4;
        while (((size_t)1 << hbits) < nc * 2 && hbits < 22) ++hbits;
        hmask = (1 << hbits) - 1;
        hkey.assign((size_t)1 << hbits, EMPTY);
        hval.assign((size_t)1 << hbits, -1);
        const int shift = 64 - hbits;
        for (size_t b = 0; b < nc; ++b)
        {
            const uint64_t h = (uint64_t)keys[b] * 0x9E3779B97F4A7C15ULL;
            int i = (int)(h >> shift);
            while (hkey[i] != EMPTY) i = (i + 1) & hmask;
            hkey[i] = keys[b];
            hval[i] = (int)b;
        }
    }
    inline int find(int64_t k) const
    {
        const uint64_t h = (uint64_t)k * 0x9E3779B97F4A7C15ULL;
        int i = (int)(h >> (64 - hbits));
        for (;;)
        {
            const int64_t kk = hkey[i];
            if (kk == k) return hval[i];
            if (kk == EMPTY) return -1;
            i = (i + 1) & hmask;
        }
    }
    // 从小到大维护最近 5 个（定长数组 + 插入排序，无动态分配）
    static inline void ins5(int i, float d2, int *oi, float *od, int &cnt)
    {
        if (cnt < 5)
        {
            int q = cnt;
            while (q > 0 && od[q - 1] > d2) { od[q] = od[q - 1]; oi[q] = oi[q - 1]; --q; }
            od[q] = d2; oi[q] = i; ++cnt;
        }
        else if (d2 < od[4])
        {
            int q = 4;
            while (q > 0 && od[q - 1] > d2) { od[q] = od[q - 1]; oi[q] = oi[q - 1]; --q; }
            od[q] = d2; oi[q] = i;
        }
    }
    // 返回候选点数；>=5 才可用（<5 ⇒ 1.0 m 内不足 5 点，与 kdtree 分支同样判"不匹配"）
    int knn5(const PointType &q, int *oi, float *od, int *ringUsed = nullptr) const
    {
        if (npts < 5) return 0;
        const int cx = cellIdx(q.x), cy = cellIdx(q.y), cz = cellIdx(q.z);
        int cnt = 0;
        for (int r = 1; r <= A2_RING; ++r)
        {
            for (int dx = -r; dx <= r; ++dx)
                for (int dy = -r; dy <= r; ++dy)
                    for (int dz = -r; dz <= r; ++dz)
                    {
                        // 内圈已扫过的格不再重复扫
                        if (r > 1 && std::max(std::abs(dx), std::max(std::abs(dy), std::abs(dz))) != r) continue;
                        const int b = find(pack(cx + dx, cy + dy, cz + dz));
                        if (b < 0) continue;
                        for (int t = beg[b]; t < beg[b + 1]; ++t)
                        {
                            const PointType &p = pc->points[ids[t]];
                            const float ax = p.x - q.x, ay = p.y - q.y, az = p.z - q.z;
                            ins5(ids[t], ax * ax + ay * ay + az * az, oi, od, cnt);
#ifdef A2_COUNT_CAND
                            ++dbg_cand;
#endif
                        }
                    }
            // ★ od[] 存的**平方**距离，故覆盖半径必须平方后再比（这里曾经写成线性比对，
            //    导致提前返回、丢掉更近的点；独立对拍 1.9% 的集合不一致即由此暴露）
            if (cnt >= 5 && od[4] <= (float)r * leaf * (float)r * leaf) { if (ringUsed) *ringUsed = r; return cnt; }
        }
        if (ringUsed) *ringUsed = -1;
        return cnt;
    }
};

// =====================================================================================
// [A3] 增量局部地图 —— 干掉"每帧重拼所有相邻关键帧 + 全量 VoxelGrid"这笔固定开销
//
// 病灶（§十五 实测）：A2 之后末帧 tot 109.3 ms 里 `eskf`（= extractSurroundingKeyFrames
//   → extractCloud）还要 53.3 ms，占 49%。它做的三件事每帧都全量重做：
//     ① 把 ~38 个相邻关键帧的世界系点云 `+=` 拼成一条（角点 ~3.8 万点、面点 ~10.6 万点）；
//     ② 对这条大点云跑一次 VoxelGrid（叶子 0.2 / 0.4），得到 8.6k / 9.1k 点的 DS 地图；
//     ③ 每帧都从头算一遍。
//   而实际上相邻关键帧集合每帧只变动 0~2 个 ⇒ **只有变化的那几帧的点需要处理**。
//
// 做法：维护"体素 → 各关键帧贡献量"的表，按关键帧做增量加/减，质心按 kf 升序重算。
//   语义与 PCL VoxelGrid 一致（质心 = 桶内所有点的均值，intensity 同样取均值），差别只有
//   浮点求和的分组顺序（原码的拼接顺序随 radiusSearch 结果次序漂移，本身不是固定的）。
//   验证方式见 `MAP_ESKF_CHECK`：同一帧两条路径各出一份 DS 地图，逐点做最近邻比对。
//
// 开关：MAP_ESKF=inc|base （默认 base = 原实现）
//       MAP_ESKF_CHECK=N  前 N 帧同时跑两条路径并比对（诊断，勿用于计时）
// =====================================================================================
static const bool A3_INC = []() {
        const char *v = std::getenv("MAP_ESKF");
        return v && std::string(v) == "inc";
    }();
static const int A3_CHECK_N = []() {
        const char *v = std::getenv("MAP_ESKF_CHECK");
        return v ? std::atoi(v) : 0;
    }();

// ---------------------------------------------------------------------------------
// [A4] 落盘：把某一帧 scan-to-map 的**真实输入**导出来（PL 侧 csim 的黄金数据）。
//   MAP_DUMP=n  第 n 次 scan2MapOptimization 时写 /dev/shm/s2m_dump.bin（0=关）
//   内容：地图角点/面点 DS 云、当前帧角点/面点 DS 云（均为传感器坐标）、6D 位姿。
//   格式见下方 dumpS2M()，纯 float32 小端，便于 C++/HLS testbench 直接读。
// ---------------------------------------------------------------------------------
static const int A4_DUMP = []() {
        const char *v = std::getenv("MAP_DUMP");
        return v ? std::atoi(v) : 0;
    }();

// ---------------------------------------------------------------------------------
// [A5] 平面求解器可换：MAP_PLANE=cd 时用"3×3 法方程 + double Cramer"代替 Eigen 的
//      colPivHouseholderQr。目的不是加速，而是**先量出"换求解器"到底会不会动轨迹**
//      —— PL 上不可能复刻 Eigen 的 QR，这个数决定要不要在 HLS 里硬啃 QR。
//      float 版 Cramer 已证不可用（病态 d=1 参数化丢 3~4 位，coeff p99 差 12%）。
// ---------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------
// [A1] cornerOptimization 里那个 3×3 对称阵的特征分解。
//   板端实测（真实 4775 点、A53）：cv::eigen 合计 **10.91 µs/点**，其中
//     3 个 cv::Mat(3,3)+Scalar::all(0) 构造          = 2.89 µs（纯 malloc+memset）
//     cv::eigen 本身（Mat 复用后）                    = 6.75 µs（OpenCV 内部 JacobiSVD 固定开销）
//     5 点协方差计算（对照）                          = 0.11 µs
//   ⇒ 这个特征分解占了 corner 分支几乎全部时间，而它数学上只有 ~50 flop。
//   MAP_COR=reuse  复用线程私有 Mat（Mat 只建一次），仍走 cv::eigen —— **逐位相同**
//   MAP_COR=saes   换成 Eigen::SelfAdjointEigenSolver（定长矩阵、栈、无任何 malloc）
//   默认 base = 原样（每点新建 3 个 Mat），保证默认行为与改动前一致。
// ---------------------------------------------------------------------------------
static const int COR_EIG = []() {
        const char *v = std::getenv("MAP_COR");
        if (!v) return 0;
        const std::string m(v);
        if (m == "reuse") return 1;
        if (m == "saes")  return 2;
        return 0;
    }();

static const bool A5_CRAMER_D = []() {
        const char *v = std::getenv("MAP_PLANE");
        return v && std::string(v) == "cd";
    }();

// =====================================================================================
// [A8] 体素平面地图（VoxelMap）—— 用"每格一张预存平面"取代"5-NN + 每点现拟合"
//
// 动机（§十七 P4 的判死结论）：忠实复刻"27 格 5-NN"上 PL 要 6 条 lane = 151k LUT
//   > 整颗 XCK26 的 117k ⇒ 不可行。根因是"选 5 个最近点"本身：候选枚举 + 选择网络 +
//   循环携带依赖（ins5 的 od[4]，II 卡 5.25）。
//   ⇒ 换表示：地图侧预先给每个体素算好 (n,d)；查询侧退化成 **30 flop + 1 次查表**，
//     没有候选、没有选择、没有依赖链 ⇒ 单 lane 就能 II=1，PL 可直接出 H/b。
//
// 语义差异（写进 limitation，不许含糊）：
//   ① 平面由**体素内全部地图点**聚合，原码用**最近 5 点** ⇒ 小尺度结构（树、杆、边缘）
//      上平面更"糊"；
//   ② 点少或退化（λ0 > LAM·λ1）的体素置 valid=0 ⇒ 该处查询直接跳过 = **丢观测量**。
//      `MAP_PLANE_DBG=1` 打印的 miss 比例就是这个风险的量；
//   ③ 原码"5 点全部落在平面 0.2 m 内"这道校验没有了，改由平面度门限 + RMAX 把关。
//
// 开关（默认全关 ⇒ 与改动前逐位一致；没有 map 侧以外的任何副作用）：
//   MAP_PLANE=vmap        打开本支（"cd" 仍是 A5 的 double Cramer，两者不冲突）
//   MAP_PLANE_LEAF=1.2    平面体素边长（m）；建议 >= 3×mappingSurfLeafSize，否则每格点太少
//   MAP_PLANE_MINPTS=5    建平面所需的最少地图点数
//   MAP_PLANE_LAM=0.10    平面度门限 λ0 <= LAM·λ1（λ0 = 最小特征值，比例量纲无关）
//   MAP_PLANE_RMAX=1.0    查询点到平面的最大距离（m），超过即判不匹配
//   MAP_PLANE_NB=0        0=只查本格（PL 的目标形态）；1=本格无效时在 27 邻格取 |r| 最小者
//   MAP_PLANE_FB=0        1=本格无有效平面时回退原码 5-NN（混合模式，§7.5 回退 A；用来分离
//                        "丢观测量"与"平面糊"两种误差来源）
//   MAP_PLANE_DBG=0       每帧打印统计（格数 / 有效平面数 / 命中率 / 建表耗时）
// =====================================================================================
static const std::string A8_MODE = []() {
        const char *v = std::getenv("MAP_PLANE");
        return std::string(v ? v : "");
    }();
static const bool  A8_VMAP = (A8_MODE == "vmap");
static const float A8_LEAF = []() {
        const char *v = std::getenv("MAP_PLANE_LEAF");
        float f = v ? (float)std::atof(v) : 1.2f;
        return f > 0.05f ? f : 1.2f;
    }();
static const int A8_MINPTS = []() {
        const char *v = std::getenv("MAP_PLANE_MINPTS");
        int n = v ? std::atoi(v) : 5;
        return n >= 3 ? n : 3;
    }();
static const float A8_LAM = []() {
        const char *v = std::getenv("MAP_PLANE_LAM");
        float f = v ? (float)std::atof(v) : 0.10f;
        return f > 0.f ? f : 0.10f;
    }();
static const float A8_RMAX = []() {
        const char *v = std::getenv("MAP_PLANE_RMAX");
        float f = v ? (float)std::atof(v) : 1.0f;
        return f > 0.f ? f : 1.0f;
    }();
static const int A8_NB = []() {
        const char *v = std::getenv("MAP_PLANE_NB");
        // 实测（s1 vs s4）：单格命中 16.7%，27 邻域 47.1% —— 邻域是必需的，
        // 因为 62% 的体素因平面度/点数不足被置 invalid。故默认 1。
        // 对 PL 无害：27 个固定偏移读（与 A2 同一套 off[27]），只需一个 min 树，无选择网络。
        return v ? std::atoi(v) : 1;
    }();
static const int A8_DBG = []() {
        const char *v = std::getenv("MAP_PLANE_DBG");
        return v ? std::atoi(v) : 0;
    }();
// MAP_PLANE_FB=1：本格无有效平面时**回退到原码 5-NN**（"混合模式"，计划 §7.5 的回退 A）。
//   用途是分离误差来源：若 FB 能把精度拉回基线，说明差的是"丢观测量"，不是平面本身糊。
static const int A8_FB = []() {
        const char *v = std::getenv("MAP_PLANE_FB");
        return v ? std::atoi(v) : 0;
    }();
// MAP_PLANE_PICK=0|1  —— 27 邻域里**选哪张平面**
//   0 = 取 |r| 最小的（默认；命中率高但**有选择偏置**：等于专挑"离查询点最近"的那张面，
//       把残差系统性做小 ⇒ 该方向的约束被削弱）
//   1 = 取**格心离查询点最近**的有效平面（不看 r，无偏；命中率会低一些）
static const int A8_PICK = []() {
        const char *v = std::getenv("MAP_PLANE_PICK");
        return v ? std::atoi(v) : 0;
    }();
// MAP_PLANE_QORDER=0|1 —— 选格顺序（只影响 PICK=1）
//   0 = A8 原版：27 邻域里**先判 |r|>rmax**，再比格心距离（最近且合格者胜）
//   1 = PL 内核语义：只按格心最近选（不看 r），选完再判 rmax（最近者不合格就丢点）
//   实测两者观测量差 ~10.7%，用 MAP_PLANE_QORDER=1 可量化它对轨迹的影响。
static const int A8_QORDER = []() {
        const char *v = std::getenv("MAP_PLANE_QORDER");
        return v ? std::atoi(v) : 0;
    }();
// MAP_PLANE_INC=1|0  —— 平面表**增量维护**（默认 1）
//   全量版每帧遍历全部细格（~9.7k）并对每个平面格（1–3.4k）做一次 3×3 自伴特征分解 ⇒ 18–39 ms。
//   增量版只重算"本帧被改动过的细格所属的平面格"（A3 早就维护了脏桶），其余缓存复用。
//   `MAP_PLANE_INC=0` 保留全量路径（对拍与回退用）。
// MAP_PLANE_CHK=n   前 n 帧同时跑全量并逐格对比（键集合 / 点数 / valid / Δd / 法向夹角）
static const int A8_INC = []() {
        const char *v = std::getenv("MAP_PLANE_INC");
        return v ? std::atoi(v) : 1;
    }();
static const int A8_CHK = []() {
        const char *v = std::getenv("MAP_PLANE_CHK");
        return v ? std::atoi(v) : 0;
    }();
// 帧内统计（在 omp 区内用**线程私有量 + reduction** 归并；不用 atomic ——
// 实测 atomic 打在同一缓存行上，4 线程 2331 点/迭代能吃到 2.7 µs/点，比纯查表贵 50 倍）
static long  a8_nq = 0, a8_nhit = 0, a8_nframe = 0;
static long  a8_nbase_nn = 0;     // 基线路径里"5-NN 门限通过"的查询数（供与 vmap 命中率对比）
static size_t a8_ncell = 0, a8_nplane = 0;
static double a8_build_ms = 0.0;
static float a8_eff_leaf = 0.f;   // 实际生效的平面格边长 = round(req/leaf)*leaf
static float a8_plane_rms = 0.f;  // 有效平面的 sqrt(λ0) 均值（m）—— 平面本身有多"平"
// [A8] 增量版的三个分项（ms）与脏平面格数：看清"省掉的到底是哪一段"
static double a8_t_agg = 0.0, a8_t_eig = 0.0, a8_t_out = 0.0;
static long   a8_npdirty = 0, a8_npall = 0;

// 输出形态刻意做成"平铺数组"：第 b 格的平面就是 pl[4b..4b+3]（= PL 侧 base + idx*16B），
// 与 A2 的体素栅格共用同一套 pack/cellIdx 约定。哈希表只用来把"格键"压成下标。
struct PlaneVoxelMap
{
    static constexpr int64_t EMPTY = INT64_MIN;
    float leaf = 1.2f, inv = 1.0f / 1.2f;
    int   minpts = 5;
    float lam_max = 0.10f;

    std::vector<int64_t>       keys;   // 升序格键（= 平铺下标 b）
    std::vector<float>         pl;     // 4 float/格：nx, ny, nz, d（平面已归一化：n·p+d=0）
    std::vector<unsigned char> valid;  // 1 = 可用
    std::vector<int>           cnt;    // 格内点数（诊断）
    std::vector<int64_t>       hkey;
    std::vector<int>           hval;
    std::vector<std::pair<int64_t, int>> kv;
    int    hbits = 4, hmask = 15;

    static inline int64_t pack(int x, int y, int z)
    {
        return ((int64_t)(x & 0x1FFFFF))
             | ((int64_t)(y & 0x1FFFFF) << 21)
             | ((int64_t)(z & 0x1FFFFF) << 42);
    }
    inline int cellIdx(float v) const // = floor(v/L)，同 A2/A3
    {
        const float f = v * inv;
        const int   i = (int)f;
        return (f < (float)i) ? (i - 1) : i;
    }
    inline int find(int64_t k) const
    {
        if (hkey.empty()) return -1;
        const uint64_t h = (uint64_t)k * 0x9E3779B97F4A7C15ULL;
        int i = (int)(h >> (64 - hbits));
        for (;;)
        {
            const int64_t kk = hkey[i];
            if (kk == k) return hval[i];
            if (kk == EMPTY) return -1;
            i = (i + 1) & hmask;
        }
    }
    // 单趟：分组 → 每股算协方差 → 3×3 特征分解取最小特征向量当法向
    void build(const pcl::PointCloud<PointType> &c, float lf, int mp, float lm)
    {
        leaf = lf; inv = 1.0f / lf; minpts = mp; lam_max = lm;
        keys.clear(); pl.clear(); valid.clear(); cnt.clear(); kv.clear();
        kv.reserve(c.points.size());
        for (size_t i = 0; i < c.points.size(); ++i)
        {
            const PointType &p = c.points[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            kv.emplace_back(pack(cellIdx(p.x), cellIdx(p.y), cellIdx(p.z)), (int)i);
        }
        if (kv.empty()) { hkey.clear(); hval.clear(); return; }
        std::sort(kv.begin(), kv.end(),
                  [](const std::pair<int64_t, int> &a, const std::pair<int64_t, int> &b) {
                      return (a.first != b.first) ? (a.first < b.first) : (a.second < b.second);
                  });
        size_t ng = 0;                       // 有点的格数
        size_t np = 0;                       // 通过平面度门限的格数
        size_t t = 0;
        while (t < kv.size())
        {
            const int64_t k = kv[t].first;
            size_t e = t;
            while (e < kv.size() && kv[e].first == k) ++e;
            const int n = (int)(e - t);
            keys.push_back(k);
            cnt.push_back(n);
            ++ng;

            float nx = 0.f, ny = 0.f, nz = 0.f, dc = 0.f;
            unsigned char ok = 0;
            if (n >= minpts)
            {
                double mx = 0, my = 0, mz = 0;
                for (size_t u = t; u < e; ++u)
                {
                    const PointType &p = c.points[kv[u].second];
                    mx += p.x; my += p.y; mz += p.z;
                }
                mx /= n; my /= n; mz /= n;
                double sxx = 0, sxy = 0, sxz = 0, syy = 0, syz = 0, szz = 0;
                for (size_t u = t; u < e; ++u)
                {
                    const PointType &p = c.points[kv[u].second];
                    const double dx = p.x - mx, dy = p.y - my, dz = p.z - mz;
                    sxx += dx * dx; sxy += dx * dy; sxz += dx * dz;
                    syy += dy * dy; syz += dy * dz; szz += dz * dz;
                }
                Eigen::Matrix3f cov;
                cov << (float)(sxx / n), (float)(sxy / n), (float)(sxz / n),
                       (float)(sxy / n), (float)(syy / n), (float)(syz / n),
                       (float)(sxz / n), (float)(syz / n), (float)(szz / n);
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
                if (es.info() == Eigen::Success)
                {
                    const float l0 = es.eigenvalues()(0);   // 升序 ⇒ (0) 最小
                    const float l1 = es.eigenvalues()(1);
                    if (l1 > 1e-12f && l0 <= lam_max * l1)
                    {
                        Eigen::Vector3f v = es.eigenvectors().col(0);
                        const float vn = v.norm();
                        if (vn > 1e-12f)
                        {
                            v /= vn;
                            nx = v.x(); ny = v.y(); nz = v.z();
                            dc = -(float)(nx * mx + ny * my + nz * mz);
                            ok = 1; ++np;
                        }
                    }
                }
            }
            pl.push_back(nx); pl.push_back(ny); pl.push_back(nz); pl.push_back(dc);
            valid.push_back(ok);
            t = e;
        }
        // 哈希：槽数取 >= 2*ng 的 2 的幂
        finalize();
        a8_ncell = ng;
        a8_nplane = np;
    }
    // keys 填好之后重建哈希索引（build / buildPlanes 共用）
    void finalize()
    {
        hbits = 4;
        while (((size_t)1 << hbits) < keys.size() * 2 && hbits < 22) ++hbits;
        hmask = (1 << hbits) - 1;
        hkey.assign((size_t)1 << hbits, EMPTY);
        hval.assign((size_t)1 << hbits, -1);
        const int shift = 64 - hbits;
        for (size_t b = 0; b < keys.size(); ++b)
        {
            const uint64_t h = (uint64_t)keys[b] * 0x9E3779B97F4A7C15ULL;
            int i = (int)(h >> shift);
            while (hkey[i] != EMPTY) i = (i + 1) & hmask;
            hkey[i] = keys[b];
            hval[i] = (int)b;
        }
    }
    // [A8] 增量写表：保证装载因子 <= 0.5（必要时扩容重建哈希）
    void ensureHash(size_t need)
    {
        if (((size_t)1 << hbits) >= need * 2) return;
        while (((size_t)1 << hbits) < need * 2 && hbits < 22) ++hbits;
        hmask = (1 << hbits) - 1;
        hkey.assign((size_t)1 << hbits, EMPTY);
        hval.assign((size_t)1 << hbits, -1);
        const int shift = 64 - hbits;
        for (size_t b = 0; b < keys.size(); ++b)
        {
            const uint64_t h = (uint64_t)keys[b] * 0x9E3779B97F4A7C15ULL;
            int i = (int)(h >> shift);
            while (hkey[i] != EMPTY) i = (i + 1) & hmask;
            hkey[i] = keys[b];
            hval[i] = (int)b;
        }
    }
    // 追加一格：数组 + 哈希（顺序无关；PL 侧只要 base+idx 一致即可）
    void appendCell(int64_t key, float nx, float ny, float nz, float dc,
                    unsigned char ok, int n)
    {
        pushCell(key, nx, ny, nz, dc, ok, n);
        ensureHash(keys.size());
        const int b = (int)keys.size() - 1;
        const uint64_t h = (uint64_t)key * 0x9E3779B97F4A7C15ULL;
        int i = (int)(h >> (64 - hbits));
        while (hkey[i] != EMPTY) i = (i + 1) & hmask;
        hkey[i] = key;
        hval[i] = b;
    }
    inline void setCell(int b, float nx, float ny, float nz, float dc,
                        unsigned char ok, int n)
    {
        pl[4 * b + 0] = nx; pl[4 * b + 1] = ny; pl[4 * b + 2] = nz; pl[4 * b + 3] = dc;
        valid[b] = ok;
        cnt[b] = n;
    }
    inline void pushCell(int64_t key, float nx, float ny, float nz, float dc,
                         unsigned char ok, int n)
    {
        keys.push_back(key);
        pl.push_back(nx); pl.push_back(ny); pl.push_back(nz); pl.push_back(dc);
        valid.push_back(ok);
        cnt.push_back(n);
    }
    // 返回格下标 b（pl[4b..] 可用），-1 = 无有效平面；*pr 回带带符号距离 n·p+d
    inline int query(const PointType &p, int nb, float rmax, float *pr) const
    {
        if (keys.empty()) return -1;
        const int cx = cellIdx(p.x), cy = cellIdx(p.y), cz = cellIdx(p.z);
        if (nb <= 0)
        {
            const int b = find(pack(cx, cy, cz));
            if (b < 0 || !valid[b]) return -1;
            const float r = pl[4 * b] * p.x + pl[4 * b + 1] * p.y + pl[4 * b + 2] * p.z + pl[4 * b + 3];
            if (std::fabs(r) > rmax) return -1;
            *pr = r;
            return b;
        }
        int   best = -1;
        float bestAbs = rmax, bestR = 0.f;
        float bestCell = 1e30f;
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz)
                {
                    const int b = find(pack(cx + dx, cy + dy, cz + dz));
                    if (b < 0 || !valid[b]) continue;
                    const float r = pl[4 * b] * p.x + pl[4 * b + 1] * p.y + pl[4 * b + 2] * p.z + pl[4 * b + 3];
                    // QORDER=0：原版（先过滤 rmax）；QORDER=1：内核语义（先选最近，选完再判）
                    if (A8_QORDER == 0 && std::fabs(r) > rmax) continue;
                    if (A8_PICK == 1)
                    {
                        // 按"格心到查询点的距离"选：与 r 无关，不会把残差做小
                        const float gx = ((float)(cx + dx) + 0.5f) * leaf - p.x;
                        const float gy = ((float)(cy + dy) + 0.5f) * leaf - p.y;
                        const float gz = ((float)(cz + dz) + 0.5f) * leaf - p.z;
                        const float dc2 = gx * gx + gy * gy + gz * gz;
                        if (dc2 < bestCell) { bestCell = dc2; best = b; bestR = r; }
                    }
                    else
                    {
                        if (std::fabs(r) > rmax) continue;   // min|r| 模式必须先过滤，QORDER 不适用
                        const float ar = std::fabs(r);
                        if (ar < bestAbs) { bestAbs = ar; best = b; bestR = r; }
                    }
                }
        if (best < 0) return -1;
        if (A8_QORDER == 1 && A8_PICK == 1 && std::fabs(bestR) > rmax) return -1;   // 内核语义：选完再判
        *pr = bestR;
        return best;
    }
};

// =====================================================================================
// [A9] 体素直线地图（corner 分支）—— A8 的同构版本：把"5-NN 现拟合直线"换成"每格预存直线"
//
// 依据：A8 收口后 `cor` 槽（33.3 ms）已经和 surf（33.3 ms）一样大，而两者的原码结构相同
//   （5-NN → 协方差 → 特征分解 → 点到面/线距离）。所以 corner 可以照搬同一套：
//   地图侧每格存 **(质心 c, 单位方向 t)**（6 float），查询侧算
//       v = p − c ;  d = v − (v·t)·t ;  ld2 = |d| ;  (la,lb,lc) = d/ld2
//   与原码"三角形高"的构造**几何等价**（原码也是点到过 c、方向 t 的直线距离）。
//
// 判线状性：原码用 `d0 > 3·d1`（最大特征值 > 3×次大），这里换成 λ1 ≤ LAM·λ2（LAM 默认 1/3）。
// 语义差异（比 surf 更敏感，必须写进 limitation）：
//   ① 体素里若有**两条边**（树的两侧、墙角），协方差的最大特征向量没有意义 ⇒ 靠 λ1 ≤ LAM·λ2 挡，
//      但挡不干净；这是 corner 换体素表示的最大风险。
//   ② 体素内的线只有一小段，查询却用**无限直线** ⇒ 必须用 RMAX 限住（默认 1.0，同原码 s>0.1 的隐含门限）。
//
// 开关（默认全关 ⇒ 原版 5-NN 路径逐位不变）：
//   MAP_CORELINE=vmap      打开
//   MAP_CORELINE_LEAF=0.8  线格边长（m）；corner 细格是 0.4，故实际取整到 0.8
//   MAP_CORELINE_MINPTS=5  MAP_CORELINE_LAM=0.333  MAP_CORELINE_RMAX=1.0
//   MAP_CORELINE_NB=1（27 邻域）  MAP_CORELINE_PICK=1（按格心选）  MAP_CORELINE_FB=0（回退 5-NN）
//   MAP_CORELINE_INC=1（增量建表）  MAP_CORELINE_DBG=0
// =====================================================================================
static const std::string A9_MODE = []() {
        const char *v = std::getenv("MAP_CORELINE");
        return std::string(v ? v : "");
    }();
static const bool  A9_VLINE = (A9_MODE == "vmap");
static const float A9_LEAF = []() {
        const char *v = std::getenv("MAP_CORELINE_LEAF");
        float f = v ? (float)std::atof(v) : 0.8f;
        return f > 0.05f ? f : 0.8f;
    }();
static const int A9_MINPTS = []() {
        const char *v = std::getenv("MAP_CORELINE_MINPTS");
        int n = v ? std::atoi(v) : 5;
        return n >= 3 ? n : 3;
    }();
static const float A9_LAM = []() {
        const char *v = std::getenv("MAP_CORELINE_LAM");
        float f = v ? (float)std::atof(v) : 0.333f;
        return f > 0.f ? f : 0.333f;
    }();
static const float A9_RMAX = []() {
        const char *v = std::getenv("MAP_CORELINE_RMAX");
        float f = v ? (float)std::atof(v) : 1.0f;
        return f > 0.f ? f : 1.0f;
    }();
static const int A9_NB = []() {
        const char *v = std::getenv("MAP_CORELINE_NB");
        return v ? std::atoi(v) : 1;
    }();
static const int A9_PICK = []() {
        const char *v = std::getenv("MAP_CORELINE_PICK");
        return v ? std::atoi(v) : 1;
    }();
static const int A9_FB = []() {
        const char *v = std::getenv("MAP_CORELINE_FB");
        return v ? std::atoi(v) : 0;
    }();
static const int A9_INC = []() {
        const char *v = std::getenv("MAP_CORELINE_INC");
        return v ? std::atoi(v) : 1;
    }();
static const int A9_DBG = []() {
        const char *v = std::getenv("MAP_CORELINE_DBG");
        return v ? std::atoi(v) : 0;
    }();
// MAP_CORELINE_CHK=n：前 n 帧同时跑全量直线并与增量逐格对比（键/点数/valid/质心/方向）
static const int A9_CHK = []() {
        const char *v = std::getenv("MAP_CORELINE_CHK");
        return v ? std::atoi(v) : 0;
    }();
static long   c9_nq = 0, c9_nhit = 0, c9_nframe = 0, c9_nbase_nn = 0;
static size_t c9_ncell = 0, c9_nline = 0;
static double c9_build_ms = 0.0, c9_t_agg = 0.0, c9_t_eig = 0.0, c9_t_out = 0.0;
static long   c9_ndirty = 0, c9_nall = 0;
static float  c9_eff_leaf = 0.f, c9_line_rms = 0.f;

// 每格 6 float：cx,cy,cz,tx,ty,tz（PL 侧就是 base + idx*24B）
struct LineVoxelMap
{
    static constexpr int64_t EMPTY = INT64_MIN;
    float leaf = 0.8f, inv = 1.25f;
    int   minpts = 5;
    float lam_max = 0.333f;
    std::vector<int64_t>       keys;
    std::vector<float>         ln;
    std::vector<unsigned char> valid;
    std::vector<int>           cnt;
    std::vector<int64_t>       hkey;
    std::vector<int>           hval;
    int hbits = 4, hmask = 15;

    struct Hit { int b = -1; float ld2 = 0.f, la = 0.f, lb = 0.f, lc = 0.f; };
    static inline int64_t pack(int x, int y, int z)
    {
        return ((int64_t)(x & 0x1FFFFF))
             | ((int64_t)(y & 0x1FFFFF) << 21)
             | ((int64_t)(z & 0x1FFFFF) << 42);
    }
    inline int cellIdx(float v) const
    {
        const float f = v * inv;
        const int   i = (int)f;
        return (f < (float)i) ? (i - 1) : i;
    }
    inline int find(int64_t k) const
    {
        if (hkey.empty()) return -1;
        const uint64_t h = (uint64_t)k * 0x9E3779B97F4A7C15ULL;
        int i = (int)(h >> (64 - hbits));
        for (;;)
        {
            const int64_t kk = hkey[i];
            if (kk == k) return hval[i];
            if (kk == EMPTY) return -1;
            i = (i + 1) & hmask;
        }
    }
    void finalize()
    {
        hbits = 4;
        while (((size_t)1 << hbits) < keys.size() * 2 && hbits < 22) ++hbits;
        hmask = (1 << hbits) - 1;
        hkey.assign((size_t)1 << hbits, EMPTY);
        hval.assign((size_t)1 << hbits, -1);
        const int shift = 64 - hbits;
        for (size_t b = 0; b < keys.size(); ++b)
        {
            const uint64_t h = (uint64_t)keys[b] * 0x9E3779B97F4A7C15ULL;
            int i = (int)(h >> shift);
            while (hkey[i] != EMPTY) i = (i + 1) & hmask;
            hkey[i] = keys[b];
            hval[i] = (int)b;
        }
    }
    void ensureHash(size_t need)
    {
        if (((size_t)1 << hbits) >= need * 2) return;
        while (((size_t)1 << hbits) < need * 2 && hbits < 22) ++hbits;
        hmask = (1 << hbits) - 1;
        hkey.assign((size_t)1 << hbits, EMPTY);
        hval.assign((size_t)1 << hbits, -1);
        const int shift = 64 - hbits;
        for (size_t b = 0; b < keys.size(); ++b)
        {
            const uint64_t h = (uint64_t)keys[b] * 0x9E3779B97F4A7C15ULL;
            int i = (int)(h >> shift);
            while (hkey[i] != EMPTY) i = (i + 1) & hmask;
            hkey[i] = keys[b];
            hval[i] = (int)b;
        }
    }
    inline void pushCell(int64_t k, const float *v6, unsigned char ok, int n)
    {
        keys.push_back(k);
        for (int i = 0; i < 6; ++i) ln.push_back(v6[i]);
        valid.push_back(ok);
        cnt.push_back(n);
    }
    void appendCell(int64_t k, const float *v6, unsigned char ok, int n)
    {
        pushCell(k, v6, ok, n);
        ensureHash(keys.size());
        const int b = (int)keys.size() - 1;
        const uint64_t h = (uint64_t)k * 0x9E3779B97F4A7C15ULL;
        int i = (int)(h >> (64 - hbits));
        while (hkey[i] != EMPTY) i = (i + 1) & hmask;
        hkey[i] = k;
        hval[i] = b;
    }
    inline void setCell(int b, const float *v6, unsigned char ok, int n)
    {
        for (int i = 0; i < 6; ++i) ln[6 * b + i] = v6[i];
        valid[b] = ok;
        cnt[b] = n;
    }
    // 返回 true 表示拿到一条可用直线；hit.ld2 = 点到线距离，(la,lb,lc) = 垂线单位方向（原码同式）
    inline bool query(const PointType &p, int nb, float rmax, Hit &hit) const
    {
        if (keys.empty()) return false;
        const int cx = cellIdx(p.x), cy = cellIdx(p.y), cz = cellIdx(p.z);
        int   best = -1;
        float bestCell = 1e30f, bestAbs = rmax;
        for (int dx = (nb > 0 ? -1 : 0); dx <= (nb > 0 ? 1 : 0); ++dx)
            for (int dy = (nb > 0 ? -1 : 0); dy <= (nb > 0 ? 1 : 0); ++dy)
                for (int dz = (nb > 0 ? -1 : 0); dz <= (nb > 0 ? 1 : 0); ++dz)
                {
                    const int b = find(pack(cx + dx, cy + dy, cz + dz));
                    if (b < 0 || !valid[b]) continue;
                    if (A9_PICK == 1)
                    {
                        const float gx = ((float)(cx + dx) + 0.5f) * leaf - p.x;
                        const float gy = ((float)(cy + dy) + 0.5f) * leaf - p.y;
                        const float gz = ((float)(cz + dz) + 0.5f) * leaf - p.z;
                        const float dc2 = gx * gx + gy * gy + gz * gz;
                        if (dc2 < bestCell) { bestCell = dc2; best = b; }
                    }
                    else
                    {
                        const float vx = p.x - ln[6 * b + 0], vy = p.y - ln[6 * b + 1], vz = p.z - ln[6 * b + 2];
                        const float tx = ln[6 * b + 3], ty = ln[6 * b + 4], tz = ln[6 * b + 5];
                        const float pr = vx * tx + vy * ty + vz * tz;
                        const float ax = vx - pr * tx, ay = vy - pr * ty, az = vz - pr * tz;
                        const float ar = std::sqrt(ax * ax + ay * ay + az * az);
                        if (ar < bestAbs) { bestAbs = ar; best = b; }
                    }
                }
        if (best < 0) return false;
        const float vx = p.x - ln[6 * best + 0], vy = p.y - ln[6 * best + 1], vz = p.z - ln[6 * best + 2];
        const float tx = ln[6 * best + 3], ty = ln[6 * best + 4], tz = ln[6 * best + 5];
        const float pr = vx * tx + vy * ty + vz * tz;
        const float ax = vx - pr * tx, ay = vy - pr * ty, az = vz - pr * tz;
        const float ar = std::sqrt(ax * ax + ay * ay + az * az);
        if (!(ar > 1e-9f) || ar > rmax) return false;
        hit.b = best;
        hit.ld2 = ar;
        hit.la = ax / ar; hit.lb = ay / ar; hit.lc = az / ar;
        return true;
    }
};

struct IncLocalMap
{
    float leaf = 0.5f, inv = 2.0f;
    // 每个桶里按关键帧分别记账：n 点、Σx/Σy/Σz/Σi。增/减只动这几项。
    // [A8] 追加二阶矩 Σxx/Σxy/Σxz/Σyy/Σyz/Σzz —— 体素平面地图要的协方差直接从这里聚合，
    //      代价是每点 6 次乘加（相对 A3 原有的 4 次加法可忽略），且**不必物化原始点云**。
    struct Contrib { int kf; int n; float sx, sy, sz, si;
                     float sxx, sxy, sxz, syy, syz, szz; };
    // 一桶一槽：slot 指向 out->points 里的位置，质心只在该桶被改动时重算（脏表驱动）。
    struct Cell    { std::vector<Contrib> c; int slot = -1; bool dirty = false;
                     int cx = 0, cy = 0, cz = 0; };   // [A8] 体素下标（供平面栅格按整数除法归并）
    std::unordered_map<int64_t, Cell> cells;
    std::unordered_map<int, int>      active;   // kf → 在 extract 列表里出现几次
    std::vector<int64_t>              slotKey;  // slot → 桶键
    std::vector<int64_t>              dirty;    // 本帧需要重算质心的桶键
    pcl::PointCloud<PointType>::Ptr   out{new pcl::PointCloud<PointType>()};

    // ========================= [A8] 平面栅格的增量状态 =========================
    // 二级结构：细格（leaf，A3 原有）→ 平面格（leaf*planeK）。
    //   planeCells   平面格键 → 其下细格键（插入顺序，重算时按此顺序求和）
    //   planeGeom    平面格键 → 已算好的 (n,d)+valid+点数（不脏就不动）
    //   planeDirty   本帧需要重算的平面格键（去重队列）
    // 任何"细格内容变了 / 细格新增 / 细格被删"都要让所属平面格变脏。
    struct PAgg  { double n = 0, sx = 0, sy = 0, sz = 0,
                          sxx = 0, sxy = 0, sxz = 0, syy = 0, syz = 0, szz = 0; };
    struct PGeom { float nx = 0, ny = 0, nz = 0, d = 0; unsigned char ok = 0; int n = 0; float rms = 0.f; };
    int          planeK = 0;                                   // 0 = 还没配置过
    size_t       livePlanes = 0;                               // 有效平面数（增量维护）
    double       liveRms = 0.0;                                // 有效平面 sqrt(λ0) 之和
    std::unordered_map<int64_t, std::vector<int64_t>> planeCells;
    std::unordered_map<int64_t, PGeom>                planeGeom;
    std::unordered_map<int64_t, unsigned char>        planeMark;   // 去重标记
    std::vector<int64_t>                              planeDirty;

    static inline int64_t pack(int x, int y, int z)
    {
        return ((int64_t)(x & 0x1FFFFF))
             | ((int64_t)(y & 0x1FFFFF) << 21)
             | ((int64_t)(z & 0x1FFFFF) << 42);
    }
    inline int cellIdx(float v) const
    {
        const float f = v * inv;
        const int   i = (int)f;
        return (f < (float)i) ? (i - 1) : i;
    }
    void reset(float lf)
    {
        leaf = lf; inv = 1.0f / lf;
        cells.clear(); active.clear(); slotKey.clear(); dirty.clear();
        out->clear();
        // [A8] 平面栅格跟着一起清；planeK 归零 ⇒ 下次 buildPlanesInc 会整体重建索引
        planeK = 0;
        planeCells.clear(); planeGeom.clear(); planeMark.clear(); planeDirty.clear();
    }
    // [A8] 细格 → 所属平面格键
    inline int64_t pkeyOfCell(const Cell &c) const
    {
        return pack(fdiv(c.cx, planeK), fdiv(c.cy, planeK), fdiv(c.cz, planeK));
    }
    inline void markPlane(int64_t pkey)
    {
        unsigned char &f = planeMark[pkey];
        if (!f) { f = 1; planeDirty.push_back(pkey); }
    }
    inline Cell *getCell(int64_t k, int cx, int cy, int cz)
    {
        auto it = cells.find(k);
        if (it != cells.end()) return &it->second;
        Cell &c = cells[k];
        c.slot = (int)out->points.size();
        c.cx = cx; c.cy = cy; c.cz = cz;      // [A8] 平面栅格要靠它做整数除法归并
        PointType p0;
        p0.x = 0.f; p0.y = 0.f; p0.z = 0.f; p0.intensity = 0.f;
        out->points.push_back(p0);
        slotKey.push_back(k);
        if (planeK > 0)                       // [A8] 新细格立刻登记进它所属的平面格
        {
            const int64_t pk = pkeyOfCell(c);
            planeCells[pk].push_back(k);
            markPlane(pk);
        }
        return &c;
    }
    // 桶空了：把最后一个槽搬过来顶替，O(1)
    inline void dropCell(int64_t k)
    {
        auto it = cells.find(k);
        if (it == cells.end()) return;
        if (planeK > 0)                       // [A8] 从平面格的成员里摘掉（列表很短，线性扫）
        {
            const int64_t pk = pkeyOfCell(it->second);
            auto pt = planeCells.find(pk);
            if (pt != planeCells.end())
            {
                std::vector<int64_t> &v = pt->second;
                for (size_t i = 0; i < v.size(); ++i)
                    if (v[i] == k) { v[i] = v.back(); v.pop_back(); break; }
            }
            markPlane(pk);
        }
        const int s = it->second.slot;
        const int last = (int)slotKey.size() - 1;
        if (s >= 0 && s != last)
        {
            out->points[s] = out->points[last];
            const int64_t kl = slotKey[last];
            slotKey[s] = kl;
            auto jt = cells.find(kl);
            if (jt != cells.end()) jt->second.slot = s;
        }
        if (last >= 0) { slotKey.pop_back(); out->points.pop_back(); }
        cells.erase(it);
    }
    inline void touch(int64_t k, Cell &c)
    {
        if (!c.dirty) { c.dirty = true; dirty.push_back(k); }
        if (planeK > 0) markPlane(pkeyOfCell(c));   // [A8] 内容变了 ⇒ 所属平面格要重算
    }
    void addKf(int kf, const pcl::PointCloud<PointType> &wc)
    {
        for (size_t i = 0; i < wc.size(); ++i)
        {
            const PointType &p = wc.points[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            const int cx = cellIdx(p.x), cy = cellIdx(p.y), cz = cellIdx(p.z);
            const int64_t k = pack(cx, cy, cz);
            Cell *cell = getCell(k, cx, cy, cz);
            Contrib *cb = nullptr;
            for (size_t j = 0; j < cell->c.size(); ++j)
                if (cell->c[j].kf == kf) { cb = &cell->c[j]; break; }
            if (!cb) { cell->c.push_back(Contrib{kf, 0, 0.f, 0.f, 0.f, 0.f}); cb = &cell->c.back(); }
            cb->n += 1; cb->sx += p.x; cb->sy += p.y; cb->sz += p.z; cb->si += p.intensity;
            cb->sxx += p.x * p.x; cb->sxy += p.x * p.y; cb->sxz += p.x * p.z;
            cb->syy += p.y * p.y; cb->syz += p.y * p.z; cb->szz += p.z * p.z;
            touch(k, *cell);
        }
    }
    // 撤销某个关键帧的一份贡献；返回 false 表示账对不上（调用方应整体重建）
    bool removeKf(int kf, const pcl::PointCloud<PointType> &wc)
    {
        for (size_t i = 0; i < wc.size(); ++i)
        {
            const PointType &p = wc.points[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            const int64_t k = pack(cellIdx(p.x), cellIdx(p.y), cellIdx(p.z));
            auto it = cells.find(k);
            if (it == cells.end()) return false;
            Cell &cell = it->second;
            int j = -1;
            for (size_t t = 0; t < cell.c.size(); ++t)
                if (cell.c[t].kf == kf) { j = (int)t; break; }
            if (j < 0) return false;
            Contrib &cb = cell.c[j];
            cb.n -= 1; cb.sx -= p.x; cb.sy -= p.y; cb.sz -= p.z; cb.si -= p.intensity;
            cb.sxx -= p.x * p.x; cb.sxy -= p.x * p.y; cb.sxz -= p.x * p.z;
            cb.syy -= p.y * p.y; cb.syz -= p.y * p.z; cb.szz -= p.z * p.z;
            if (cb.n <= 0) { cell.c[j] = cell.c.back(); cell.c.pop_back(); }
            if (cell.c.empty()) dropCell(k);   // 注意：之后 cell 引用已失效
            else touch(k, cell);
        }
        return true;
    }
    // 只重算脏桶的质心并写回自己的槽
    void flush()
    {
        for (size_t d = 0; d < dirty.size(); ++d)
        {
            auto it = cells.find(dirty[d]);
            if (it == cells.end()) continue;
            Cell &cell = it->second;
            cell.dirty = false;
            if (cell.c.size() > 1)
                std::sort(cell.c.begin(), cell.c.end(),
                          [](const Contrib &x, const Contrib &y) { return x.kf < y.kf; });
            float sx = 0.f, sy = 0.f, sz = 0.f, si = 0.f;
            int   n = 0;
            for (size_t t = 0; t < cell.c.size(); ++t)
            {
                const Contrib &b = cell.c[t];
                sx += b.sx; sy += b.sy; sz += b.sz; si += b.si; n += b.n;
            }
            if (n <= 0 || cell.slot < 0) continue;
            PointType &p = out->points[cell.slot];
            p.x = sx / (float)n; p.y = sy / (float)n; p.z = sz / (float)n;
            p.intensity = si / (float)n;
        }
        dirty.clear();
        out->width = (uint32_t)out->points.size();
        out->height = 1;
        out->is_dense = false;
    }

    // ---------------------------------------------------------------------------------
    // [A8] 由桶内二阶矩聚合出"体素平面表"（每帧一次，纯 CPU；PL 侧对应"CPU 备好表 → 上传"）
    //
    //   平面体素 = k×k×k 个细格（k = round(Lp/leaf)），按**细格下标整除**归并 ⇒ 边界精确。
    //   聚合的是这些细格里**原始点**的一/二阶矩 ⇒ 等价于"把原始点全物化后再拟合"，
    //   但一个原始点都不用存（这也是 A3 增量表的额外红利）。
    //
    //   ★ 为什么不能用 DS 云建表：DS 之后 0.4 m 格内只剩 1 个质心点，实测（a8/precheck.py）
    //     1.2 m 平面格内中位 2 点、有效平面仅 1.4%、查询命中 2.9% —— 完全不可用。
    //     原始点则有 ~12 点/细格 ⇒ 1.2 m 格内约 300 点。
    // ---------------------------------------------------------------------------------
    static inline int fdiv(int a, int b)   // floor(a/b)，负数也向 -inf 取整
    {
        int q = a / b;
        if ((a % b) != 0 && ((a < 0) != (b < 0))) --q;
        return q;
    }
    // 由一/二阶矩算平面：**全量与增量共用这一处**，保证两条路径语义完全相同
    static PGeom geomOf(const PAgg &a, int minpts, float lam, double *rmsOut)
    {
        PGeom g;
        const int n = (int)(a.n + 0.5);
        g.n = n;
        if (n < minpts) return g;
        const double mx = a.sx / n, my = a.sy / n, mz = a.sz / n;
        Eigen::Matrix3f cov;
        cov << (float)(a.sxx / n - mx * mx), (float)(a.sxy / n - mx * my), (float)(a.sxz / n - mx * mz),
               (float)(a.sxy / n - mx * my), (float)(a.syy / n - my * my), (float)(a.syz / n - my * mz),
               (float)(a.sxz / n - mx * mz), (float)(a.syz / n - my * mz), (float)(a.szz / n - mz * mz);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
        if (es.info() != Eigen::Success) return g;
        const float l0 = es.eigenvalues()(0);   // 升序 ⇒ (0) 最小
        const float l1 = es.eigenvalues()(1);
        if (!(l1 > 1e-12f && l0 <= lam * l1)) return g;
        Eigen::Vector3f v = es.eigenvectors().col(0);
        const float vn = v.norm();
        if (!(vn > 1e-12f)) return g;
        v /= vn;
        g.nx = v.x(); g.ny = v.y(); g.nz = v.z();
        g.d = -(float)(g.nx * mx + g.ny * my + g.nz * mz);
        g.ok = 1;
        if (rmsOut) *rmsOut = std::sqrt(std::max(0.0, (double)l0));   // λ0 可能因舍入为负
        return g;
    }
    // 汇总一个平面格下所有细格的一/二阶矩（顺序 = planeCells 的插入顺序）
    PAgg aggOf(const std::vector<int64_t> &fks) const
    {
        PAgg a;
        for (size_t i = 0; i < fks.size(); ++i)
        {
            auto it = cells.find(fks[i]);
            if (it == cells.end()) continue;
            const Cell &c = it->second;
            for (size_t t = 0; t < c.c.size(); ++t)
            {
                const Contrib &b = c.c[t];
                a.n += b.n;
                a.sx += b.sx; a.sy += b.sy; a.sz += b.sz;
                a.sxx += b.sxx; a.sxy += b.sxy; a.sxz += b.sxz;
                a.syy += b.syy; a.syz += b.syz; a.szz += b.szz;
            }
        }
        return a;
    }
    // 几何缓存（键升序）→ 平铺输出表（PL 侧就是 base + idx*16B）
    static void emitTable(PlaneVoxelMap &out, const std::unordered_map<int64_t, PGeom> &gm,
                          float effLeaf, int minpts, float lam)
    {
        out.leaf = effLeaf;
        out.inv  = 1.0f / effLeaf;
        out.minpts = minpts; out.lam_max = lam;
        out.keys.clear(); out.pl.clear(); out.valid.clear(); out.cnt.clear();
        std::vector<int64_t> ks;
        ks.reserve(gm.size());
        for (auto &kv : gm) ks.push_back(kv.first);
        std::sort(ks.begin(), ks.end());
        size_t np = 0;
        double rms = 0.0;
        for (size_t i = 0; i < ks.size(); ++i)
        {
            const PGeom &g = gm.find(ks[i])->second;
            out.pushCell(ks[i], g.nx, g.ny, g.nz, g.d, g.ok, g.n);
            if (g.ok) { ++np; rms += (double)g.rms; }
        }
        out.finalize();
        a8_ncell = ks.size();
        a8_nplane = np;
        a8_plane_rms = np ? (float)(rms / (double)np) : 0.f;
    }
    // [A8] 增量写表：只动"本帧变过的格"，不再整表重建（整表重建 3k 格 ≈ 4 ms / 9.6k 格 ≈ 14 ms）
    //   消失的格置 valid=0（保留键，query 会跳过），靠紧凑化回收。
    static void emitIncremental(PlaneVoxelMap &out, const std::unordered_map<int64_t, PGeom> &gm,
                                const std::vector<int64_t> &touched, float effLeaf,
                                int minpts, float lam)
    {
        out.leaf = effLeaf;
        out.inv  = 1.0f / effLeaf;
        out.minpts = minpts; out.lam_max = lam;
        for (size_t i = 0; i < touched.size(); ++i)
        {
            const int64_t k = touched[i];
            auto it = gm.find(k);
            const int b = out.find(k);
            if (it == gm.end())
            {
                if (b >= 0) { out.valid[b] = 0; out.cnt[b] = 0; }
                continue;
            }
            const PGeom &g = it->second;
            if (b < 0) out.appendCell(k, g.nx, g.ny, g.nz, g.d, g.ok, g.n);
            else       out.setCell(b, g.nx, g.ny, g.nz, g.d, g.ok, g.n);
        }
    }
    // ---- 全量：每帧把全部细格重新聚合、每个平面格重新分解（对拍基准 / MAP_PLANE_INC=0）----
    void buildPlanesFull(float planeLeaf, int minpts, float lam, PlaneVoxelMap &out) const
    {
        int k = (int)(planeLeaf / leaf + 0.5f);
        if (k < 1) k = 1;
        const float eff = leaf * (float)k;
        a8_eff_leaf = eff;

        auto t0 = std::chrono::steady_clock::now();
        std::unordered_map<int64_t, PAgg> acc;
        acc.reserve(cells.size() / (size_t)k + 16);
        for (auto &kv : cells)
        {
            const Cell &cell = kv.second;
            PAgg ca;
            for (size_t t = 0; t < cell.c.size(); ++t)
            {
                const Contrib &b = cell.c[t];
                ca.n += b.n;
                ca.sx += b.sx; ca.sy += b.sy; ca.sz += b.sz;
                ca.sxx += b.sxx; ca.sxy += b.sxy; ca.sxz += b.sxz;
                ca.syy += b.syy; ca.syz += b.syz; ca.szz += b.szz;
            }
            if (ca.n <= 0) continue;
            PAgg &a = acc[pack(fdiv(cell.cx, k), fdiv(cell.cy, k), fdiv(cell.cz, k))];
            a.n += ca.n;
            a.sx += ca.sx; a.sy += ca.sy; a.sz += ca.sz;
            a.sxx += ca.sxx; a.sxy += ca.sxy; a.sxz += ca.sxz;
            a.syy += ca.syy; a.syz += ca.syz; a.szz += ca.szz;
        }
        auto t1 = std::chrono::steady_clock::now();
        std::unordered_map<int64_t, PGeom> gm;
        gm.reserve(acc.size());
        for (auto &kv : acc)
        {
            double rms = 0.0;
            PGeom g = geomOf(kv.second, minpts, lam, &rms);
            g.rms = (float)rms;
            gm[kv.first] = g;
        }
        auto t2 = std::chrono::steady_clock::now();
        emitTable(out, gm, eff, minpts, lam);
        auto t3 = std::chrono::steady_clock::now();
        a8_t_agg = std::chrono::duration<double, std::milli>(t1 - t0).count();
        a8_t_eig = std::chrono::duration<double, std::milli>(t2 - t1).count();
        a8_t_out = std::chrono::duration<double, std::milli>(t3 - t2).count();
        a8_npdirty = 0;
        a8_npall = (long)acc.size();
    }
    // ---- 增量：只重算"本帧被改动过的细格"所属的平面格 ----
    void buildPlanesInc(float planeLeaf, int minpts, float lam, PlaneVoxelMap &out)
    {
        int k = (int)(planeLeaf / leaf + 0.5f);
        if (k < 1) k = 1;
        const float eff = leaf * (float)k;
        a8_eff_leaf = eff;

        bool needFull = false;
        if (planeK != k)   // 首次 / 改了 Leaf：整体建索引（一次性 O(N)）
        {
            planeK = k;
            planeCells.clear(); planeGeom.clear(); planeMark.clear(); planeDirty.clear();
            planeCells.reserve(cells.size());
            for (auto &kv : cells)
            {
                const int64_t pk = pkeyOfCell(kv.second);
                planeCells[pk].push_back(kv.first);
                markPlane(pk);
            }
            livePlanes = 0; liveRms = 0.0;
            needFull = true;
        }

        auto t0 = std::chrono::steady_clock::now();
        double t_agg = 0.0, t_eig = 0.0;
        for (size_t d = 0; d < planeDirty.size(); ++d)
        {
            const int64_t pk = planeDirty[d];
            auto it = planeCells.find(pk);
            auto oit = planeGeom.find(pk);
            if (it == planeCells.end() || it->second.empty())
            {
                if (oit != planeGeom.end())      // 平面格空了（细格全被删）：撤账后删掉
                {
                    if (oit->second.ok) { --livePlanes; liveRms -= (double)oit->second.rms; }
                    planeGeom.erase(oit);
                }
                continue;
            }
            auto ta = std::chrono::steady_clock::now();
            PAgg a = aggOf(it->second);
            auto tb = std::chrono::steady_clock::now();
            double rms = 0.0;
            PGeom g = geomOf(a, minpts, lam, &rms);
            g.rms = (float)rms;
            auto tc = std::chrono::steady_clock::now();
            t_agg += std::chrono::duration<double, std::milli>(tb - ta).count();
            t_eig += std::chrono::duration<double, std::milli>(tc - tb).count();
            if (oit != planeGeom.end())          // 有效平面计数增量维护
            {
                if (oit->second.ok) { --livePlanes; liveRms -= (double)oit->second.rms; }
                oit->second = g;
            }
            else planeGeom.emplace(pk, g);
            if (g.ok) { ++livePlanes; liveRms += (double)g.rms; }
        }
        a8_npdirty = (long)planeDirty.size();
        auto t1 = std::chrono::steady_clock::now();
        // 写表：整表重建只在"索引刚建"或"陈旧格过多"时做，其余帧只动变过的格
        if (needFull || out.keys.empty() ||
            out.keys.size() > (size_t)(1.5 * (double)planeGeom.size()) + 64)
            emitTable(out, planeGeom, eff, minpts, lam);
        else
        {
            emitIncremental(out, planeGeom, planeDirty, eff, minpts, lam);
            a8_ncell = planeGeom.size();
            a8_nplane = livePlanes;
            a8_plane_rms = livePlanes ? (float)(liveRms / (double)livePlanes) : 0.f;
        }
        planeDirty.clear();
        planeMark.clear();
        auto t2 = std::chrono::steady_clock::now();
        a8_t_agg = t_agg;
        a8_t_eig = t_eig;
        a8_t_out = std::chrono::duration<double, std::milli>(t2 - t1).count()
                 + std::chrono::duration<double, std::milli>(t1 - t0).count() - t_agg - t_eig;
        a8_npall = (long)planeGeom.size();
    }
    void buildPlanes(float planeLeaf, int minpts, float lam, PlaneVoxelMap &out)
    {
        if (A8_INC) buildPlanesInc(planeLeaf, minpts, lam, out);
        else        buildPlanesFull(planeLeaf, minpts, lam, out);
    }
    // 供 scan2Map 的对拍调用（全量版是 const，不碰增量状态）
    void buildPlanesRef(float planeLeaf, int minpts, float lam, PlaneVoxelMap &out) const
    {
        buildPlanesFull(planeLeaf, minpts, lam, out);
    }

    // ===================== [A9] 体素直线（corner）—— 与上面的平面版同构 =====================
    // ★ 注意：索引成员（planeCells/planeMark/planeDirty/planeK）是**平面与直线共用**的，
    //   因为一个 IncLocalMap 实例只会用一种模式（incSurf 走平面、incCorner 走直线）。
    struct PGeomL { float cx = 0, cy = 0, cz = 0, tx = 0, ty = 0, tz = 0;
                    unsigned char ok = 0; int n = 0; float rms = 0.f; };
    std::unordered_map<int64_t, PGeomL> lineGeom;
    size_t liveLines = 0;
    double liveLineRms = 0.0;

    // λ1 ≤ LAM·λ2 ⇒ 点沿一个方向延展（线状）；方向取**最大**特征值的特征向量
    static PGeomL geomOfLine(const PAgg &a, int minpts, float lam, double *rmsOut)
    {
        PGeomL g;
        const int n = (int)(a.n + 0.5);
        g.n = n;
        if (n < minpts) return g;
        const double mx = a.sx / n, my = a.sy / n, mz = a.sz / n;
        Eigen::Matrix3f cov;
        cov << (float)(a.sxx / n - mx * mx), (float)(a.sxy / n - mx * my), (float)(a.sxz / n - mx * mz),
               (float)(a.sxy / n - mx * my), (float)(a.syy / n - my * my), (float)(a.syz / n - my * mz),
               (float)(a.sxz / n - mx * mz), (float)(a.syz / n - my * mz), (float)(a.szz / n - mz * mz);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
        if (es.info() != Eigen::Success) return g;
        const float l2 = es.eigenvalues()(2);   // 升序 ⇒ (2) 最大 = 线的方向
        const float l1 = es.eigenvalues()(1);
        if (!(l2 > 1e-12f && l1 <= lam * l2)) return g;
        Eigen::Vector3f v = es.eigenvectors().col(2);
        const float vn = v.norm();
        if (!(vn > 1e-12f)) return g;
        v /= vn;
        g.cx = (float)mx; g.cy = (float)my; g.cz = (float)mz;
        g.tx = v.x(); g.ty = v.y(); g.tz = v.z();
        g.ok = 1;
        if (rmsOut) *rmsOut = std::sqrt(std::max(0.0, (double)l1));   // 线的"粗细" ≈ sqrt(λ1)
        return g;
    }
    static void emitLineTable(LineVoxelMap &out, const std::unordered_map<int64_t, PGeomL> &gm,
                              float effLeaf, int minpts, float lam)
    {
        out.leaf = effLeaf; out.inv = 1.0f / effLeaf;
        out.minpts = minpts; out.lam_max = lam;
        out.keys.clear(); out.ln.clear(); out.valid.clear(); out.cnt.clear();
        std::vector<int64_t> ks;
        ks.reserve(gm.size());
        for (auto &kv : gm) ks.push_back(kv.first);
        std::sort(ks.begin(), ks.end());
        size_t np = 0;
        double rms = 0.0;
        for (size_t i = 0; i < ks.size(); ++i)
        {
            const PGeomL &g = gm.find(ks[i])->second;
            const float v6[6] = {g.cx, g.cy, g.cz, g.tx, g.ty, g.tz};
            out.pushCell(ks[i], v6, g.ok, g.n);
            if (g.ok) { ++np; rms += (double)g.rms; }
        }
        out.finalize();
        c9_ncell = ks.size();
        c9_nline = np;
        c9_line_rms = np ? (float)(rms / (double)np) : 0.f;
    }
    static void emitLineIncremental(LineVoxelMap &out, const std::unordered_map<int64_t, PGeomL> &gm,
                                    const std::vector<int64_t> &touched, float effLeaf,
                                    int minpts, float lam)
    {
        out.leaf = effLeaf; out.inv = 1.0f / effLeaf;
        out.minpts = minpts; out.lam_max = lam;
        for (size_t i = 0; i < touched.size(); ++i)
        {
            const int64_t k = touched[i];
            auto it = gm.find(k);
            const int b = out.find(k);
            if (it == gm.end())
            {
                if (b >= 0) { out.valid[b] = 0; out.cnt[b] = 0; }
                continue;
            }
            const PGeomL &g = it->second;
            const float v6[6] = {g.cx, g.cy, g.cz, g.tx, g.ty, g.tz};
            if (b < 0) out.appendCell(k, v6, g.ok, g.n);
            else       out.setCell(b, v6, g.ok, g.n);
        }
    }
    // 全量直线（对拍基准）：每帧把全部细格重新聚合、每个线格重新分解
    void buildLinesFull(float lineLeaf, int minpts, float lam, LineVoxelMap &out) const
    {
        int k = (int)(lineLeaf / leaf + 0.5f);
        if (k < 1) k = 1;
        const float eff = leaf * (float)k;
        c9_eff_leaf = eff;

        auto t0 = std::chrono::steady_clock::now();
        std::unordered_map<int64_t, PAgg> acc;
        acc.reserve(cells.size() / (size_t)k + 16);
        for (auto &kv : cells)
        {
            const Cell &cell = kv.second;
            PAgg ca;
            for (size_t t = 0; t < cell.c.size(); ++t)
            {
                const Contrib &b = cell.c[t];
                ca.n += b.n;
                ca.sx += b.sx; ca.sy += b.sy; ca.sz += b.sz;
                ca.sxx += b.sxx; ca.sxy += b.sxy; ca.sxz += b.sxz;
                ca.syy += b.syy; ca.syz += b.syz; ca.szz += b.szz;
            }
            if (ca.n <= 0) continue;
            PAgg &a = acc[pack(fdiv(cell.cx, k), fdiv(cell.cy, k), fdiv(cell.cz, k))];
            a.n += ca.n;
            a.sx += ca.sx; a.sy += ca.sy; a.sz += ca.sz;
            a.sxx += ca.sxx; a.sxy += ca.sxy; a.sxz += ca.sxz;
            a.syy += ca.syy; a.syz += ca.syz; a.szz += ca.szz;
        }
        auto t1 = std::chrono::steady_clock::now();
        std::unordered_map<int64_t, PGeomL> gm;
        gm.reserve(acc.size());
        for (auto &kv : acc)
        {
            double rms = 0.0;
            PGeomL g = geomOfLine(kv.second, minpts, lam, &rms);
            g.rms = (float)rms;
            gm[kv.first] = g;
        }
        auto t2 = std::chrono::steady_clock::now();
        emitLineTable(out, gm, eff, minpts, lam);
        auto t3 = std::chrono::steady_clock::now();
        c9_t_agg = std::chrono::duration<double, std::milli>(t1 - t0).count();
        c9_t_eig = std::chrono::duration<double, std::milli>(t2 - t1).count();
        c9_t_out = std::chrono::duration<double, std::milli>(t3 - t2).count();
        c9_ndirty = 0;
        c9_nall = (long)acc.size();
    }
    void buildLinesInc(float lineLeaf, int minpts, float lam, LineVoxelMap &out)
    {
        int k = (int)(lineLeaf / leaf + 0.5f);
        if (k < 1) k = 1;
        const float eff = leaf * (float)k;
        c9_eff_leaf = eff;

        bool needFull = false;
        if (planeK != k)
        {
            planeK = k;
            planeCells.clear(); lineGeom.clear(); planeMark.clear(); planeDirty.clear();
            planeCells.reserve(cells.size());
            for (auto &kv : cells)
            {
                const int64_t pk = pkeyOfCell(kv.second);
                planeCells[pk].push_back(kv.first);
                markPlane(pk);
            }
            liveLines = 0; liveLineRms = 0.0;
            needFull = true;
        }
        auto t0 = std::chrono::steady_clock::now();
        double t_agg = 0.0, t_eig = 0.0;
        for (size_t d = 0; d < planeDirty.size(); ++d)
        {
            const int64_t pk = planeDirty[d];
            auto it = planeCells.find(pk);
            auto oit = lineGeom.find(pk);
            if (it == planeCells.end() || it->second.empty())
            {
                if (oit != lineGeom.end())
                {
                    if (oit->second.ok) { --liveLines; liveLineRms -= (double)oit->second.rms; }
                    lineGeom.erase(oit);
                }
                continue;
            }
            auto ta = std::chrono::steady_clock::now();
            PAgg a = aggOf(it->second);
            auto tb = std::chrono::steady_clock::now();
            double rms = 0.0;
            PGeomL g = geomOfLine(a, minpts, lam, &rms);
            g.rms = (float)rms;
            auto tc = std::chrono::steady_clock::now();
            t_agg += std::chrono::duration<double, std::milli>(tb - ta).count();
            t_eig += std::chrono::duration<double, std::milli>(tc - tb).count();
            if (oit != lineGeom.end())
            {
                if (oit->second.ok) { --liveLines; liveLineRms -= (double)oit->second.rms; }
                oit->second = g;
            }
            else lineGeom.emplace(pk, g);
            if (g.ok) { ++liveLines; liveLineRms += (double)g.rms; }
        }
        c9_ndirty = (long)planeDirty.size();
        auto t1 = std::chrono::steady_clock::now();
        if (needFull || out.keys.empty() ||
            out.keys.size() > (size_t)(1.5 * (double)lineGeom.size()) + 64)
            emitLineTable(out, lineGeom, eff, minpts, lam);
        else
        {
            emitLineIncremental(out, lineGeom, planeDirty, eff, minpts, lam);
            c9_ncell = lineGeom.size();
            c9_nline = liveLines;
            c9_line_rms = liveLines ? (float)(liveLineRms / (double)liveLines) : 0.f;
        }
        planeDirty.clear();
        planeMark.clear();
        auto t2 = std::chrono::steady_clock::now();
        c9_t_agg = t_agg;
        c9_t_eig = t_eig;
        c9_t_out = std::chrono::duration<double, std::milli>(t2 - t1).count()
                 + std::chrono::duration<double, std::milli>(t1 - t0).count() - t_agg - t_eig;
        c9_nall = (long)lineGeom.size();
    }
};

class Map_Optimization: public rclcpp::Node

{
public:

    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryGlobal;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryIncremental;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyPoses;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubHistoryKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrame;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudRegisteredRaw;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrame_corner;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_select_corner;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_select_surf;
  
    std::string save_map_dir;
    
    rclcpp::Subscription<lo_sam::msg::CloudInfo>::SharedPtr odom_cloud_sub;

    // [MVP0-IMU] 陀螺仪积分状态（帧间旋转初值）
    bool imu_enabled_ = false;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::mutex imu_mtx_;
    double imu_t_last_ = -1.0;
    Eigen::Matrix3d imu_dR_ = Eigen::Matrix3d::Identity();   // 自上次消费以来的 body 系增量
    bool imu_has_data_ = false;   // [MVP0-IMU-FIX] 是否已收到过 IMU

    // [VIS] 视觉相对位姿因子：订阅 /vo_pnp/abs（body 系累积位姿），在关键帧时刻插值后加 BetweenFactor
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vis_abs_sub_;
    std::mutex vis_mtx_;
    std::deque<std::pair<double, gtsam::Pose3>> vis_q_;
    int vis_add_ = 0, vis_skip_ = 0;
    int imu_dR_n_ = 0;            // [MVP0-IMU-FIX] 自上次消费以来的样本数（0 => 本帧无 IMU，回退激光增量）


    std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> edge_cloud_queue; // 存储边缘点云
    std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> surf_cloud_queue;  // 存储平面点云
    std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> full_cloud_queue;  // 存储全部点云
    std::deque<nav_msgs::msg::Odometry::SharedPtr> odom_init_queue;;        // 存储初始里程计


    std::mutex mutex; // 线程锁

    // pcl存储点云
    // pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterCorner; // 将采样

    pcl::PointCloud<PointType>::Ptr edge_cloud_curr; // 存储最新时刻的边缘点云
    pcl::PointCloud<PointType>::Ptr surf_cloud_curr; // 存储最新时刻的平面点云

    nav_msgs::msg::Odometry odom_init_curr; // 当前时刻的初始里程计

    tf2::Transform tf_odom_last;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::vector<gtsam::Symbol> X_node; // 用于存储里程计序号
 
    int graph_pose_num = 0;

    // 配置参数
    bool loopClosureEnableFlag = true; // 是否开启回环检测
    double loopClosureFrequency = 1.0; // 回环检测频率
    bool savePCD = false;              // 是否保存点云地图
    int N_SCAN = 32;
    int Horizon_SCAN = 1024;
    double mappingProcessInterval = 0.15;
    double globalMapVisualizationSearchRadius = 1000;
    double globalMapVisualizationPoseDensity = 10;
    double globalMapVisualizationLeafSize = 1.0;
    std::string odometryFrame = "camera_init";
    int surroundingKeyframeSize = 50;
    int surroundingKeyframeSearchRadius = 50;
    int edgeFeatureMinValidNum = 10;
    int surfFeatureMinValidNum = 100;
    double z_tollerance = 1000;
    double rotation_tollerance = 1000;
    int historyKeyframeSearchNum = 25;
    double surroundingkeyframeAddingAngleThreshold = 0.2;
    double historyKeyframeSearchRadius = 10.0;
    double historyKeyframeSearchTimeDiff = 30.0;
    double surroundingkeyframeAddingDistThreshold = 1.0;
    double historyKeyframeFitnessScore = 0.3;
    double odometrySurfLeafSize = 0.5;
    double mappingCornerLeafSize = 0.25;
    double mappingSurfLeafSize = 0.5;
    double surroundingKeyframeDensity = 2;


    Map_Optimization(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~Map_Optimization();

    void allocateMemory(); // 分配点云指针内存空间函数

    void odom_cloud_handler(const std::shared_ptr<const lo_sam::msg::CloudInfo> & odom_cloud_msg);

    // [MVP0-IMU]
    void imu_handler(const sensor_msgs::msg::Imu::SharedPtr msg);

    sensor_msgs::msg::PointCloud2 publishCloud(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub,
        pcl::PointCloud<PointType>::Ptr thisCloud,
        const rclcpp::Time & thisStamp,
        const std::string & thisFrame);

    void updateInitialGuess();

    void extractSurroundingKeyFrames();
    void extractNearby();
    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract);
    void extractForLoopClosure();

    void downsampleCurrentScan();

    void scan2MapOptimization();
    void updatePointAssociateToMap();
    void cornerOptimization();
    void surfOptimization();
    void combineOptimizationCoeffs();
    inline Eigen::Matrix<float, 6, 6> transMatrix(Eigen::Matrix<float, 6, 6> &M);
    bool LMOptimization(int iterCount);

    // ---- [PL-S2M] 平面残差块交给 PL（经 /dev/shm 与 Python 服务通信）----
    void pl_s2m_solve();
    int  pl_shm_fd_ = -1;
    unsigned char *pl_shm_ = nullptr;
    size_t pl_shm_sz_ = 0;
    int  pl_shm_seq_ = 0;
    bool pl_s2m_ready_ = false;
    bool pl_s2m_bad_ = false;
    int  pl_ncorner_ = 0;       // laserCloudOri 里 corner 的行数（切分点）
    float pl_H_[21] = {0};
    float pl_b_[6] = {0};
    bool pl_H_valid_ = false;
    long pl_s2m_it_ = 0;
    int  pl_fail_ = 0;
    double pl_s2m_ms_ = 0.0;
    void transformUpdate();

    void saveKeyFramesAndFactor();
    bool saveFrame();
    void addOdomFactor();

    void addVisualFactor();
    void vis_abs_handler(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    bool visPoseAt(const std::deque<std::pair<double, gtsam::Pose3>> &q, double t,
                   gtsam::Pose3 &out) const;
    void addLoopFactor();
    void updatePath(const PointTypePose &pose_in);

    void correctPoses();

    void publishOdometry();

    void publishFrames();

    void pointAssociateToMap(PointType const *const pi, PointType *const po);

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);

    gtsam::Pose3 trans2gtsamPose(float transformIn[]);

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint);

    Eigen::Affine3f trans2Affine3f(float transformIn[]);

    PointTypePose trans2PointTypePose(float transformIn[]);

    void loopClosureThread();
    void performLoopClosure();
    void visualizeLoopClosure();
    bool detectLoopClosureDistance(int *latestID, int *closestID);
    bool detectLoopClosureExternal(int *latestID, int *closestID);
    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum);

    void visualizeGlobalMapThread();
    void publishGlobalMap();

    void saveMap();

    /**************************************************************************************/

    std::deque<nav_msgs::msg::Odometry> gpsQueue;
    // LO_SAM::cloud_info cloudInfo;

    std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames; // 历史所有关键帧的角点集合（降采样)
    std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;   // 历史所有关键帧的平面点集合（降采样）

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;     // 历史关键帧位姿（只有3D位置）
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D; // 历史关键帧位姿（6DOF）
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;   // corner feature set from odoOptimization // 当前激光帧角点集合
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;     // surf feature set from odoOptimization  // 当前激光帧平面点集合
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner featuer set from odoOptimization // 当前激光帧角点集合，降采样
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;   // downsampled surf featuer set from odoOptimization // 当前激光帧平面点集合，降采样

    // 当前帧与局部map匹配上了的角点、平面点，加入同一集合；后面是对应点的参数
    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    // 当前帧与局部map匹配上了的角点、参数、标记
    std::vector<PointType> laserCloudOriCornerVec; // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    // 当前帧与局部map匹配上了的平面点、参数、标记
    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    std::map<int, std::pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;   // 局部map的角点集合
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;     // 局部map的平面点集合
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS; // 局部map的角点集合，降采样
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;   // 局部map的平面点集合，降采样

    // 局部关键帧构建的map点云，对应kdtree，用于scan-to-map找相邻点
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    // [A2] 体素栅格 NN（MAP_NN=voxel 时代替上面两棵 kdtree；默认不启用，行为与原版一致）
    VoxelGridNN nnCornerMap, nnSurfMap;
    long        a2_frame = 0;
    void        a2ParityCheck();

    // [A8] 体素平面地图（MAP_PLANE=vmap 时代替 surf 的 5-NN + 现拟合；默认不启用）
    PlaneVoxelMap planeSurfMap;
    // [A9] 体素直线地图（MAP_CORELINE=vmap 时代替 corner 的 5-NN + 现拟合；默认不启用）
    LineVoxelMap  cornerLineMap;

    // [A3] 增量局部地图（MAP_ESKF=inc 时启用；默认走原实现）
    IncLocalMap incCorner, incSurf;
    std::map<int, std::array<float, 6>> incPose;  // kf → 加入时的位姿（位姿一变就必须整体重建）
    long        a3_frame = 0;
    void        extractCloudBase(pcl::PointCloud<PointType>::Ptr cloudToExtract);
    void        extractCloudInc(pcl::PointCloud<PointType>::Ptr cloudToExtract);
    void        a3Check(const pcl::PointCloud<PointType> &baseCorner,
                        const pcl::PointCloud<PointType> &baseSurf);

    // [A4] 真实数据落盘（PL csim 的黄金数据）
    long        a4_frame = 0;
    void        dumpS2M(const char *path);

    // 降采样
    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

    rclcpp::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6]; // r, p, y, x, y, z

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool isDegenerate = false;
    cv::Mat matP;

    int laserCloudCornerFromMapDSNum = 0; // 局部map角点数量
    int laserCloudSurfFromMapDSNum = 0;   // 局部map平面点数量
    int laserCloudCornerLastDSNum = 0;    // 当前激光帧角点数量
    int laserCloudSurfLastDSNum = 0;      // 当前激光帧面点数量

    bool aLoopIsClosed = false;
    std::map<int, int> loopIndexContainer; // from new to old
    std::vector<std::pair<int, int>> loopIndexQueue;
    std::vector<gtsam::Pose3> loopPoseQueue;
    std::vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    std::deque<std_msgs::msg::Float64MultiArray> loopInfoVec;

    nav_msgs::msg::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;       // 当前帧位姿
    Eigen::Affine3f incrementalOdometryAffineFront; // 前一帧位姿
    Eigen::Affine3f incrementalOdometryAffineBack;  // 当前帧位姿

    std::vector<FactorParam> corner_factor, surf_factor;

    std::string ns_, ns_2;

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    void convert_factor_to_cloud(const std::vector<FactorParam> &params, pcl::PointCloud<PointType> &cloud, int color)
    {
        PointType p;
        Eigen::Vector3d p1, p2;
        p.intensity = color;
        for (auto factor : params)
        {
            p.x = factor.col(0).x();
            p.y = factor.col(0).y();
            p.z = factor.col(0).z();
            cloud.push_back(p);
            for (int i = 1; i <= 5; ++i)
            {
                p1 = factor.col(0) + factor.col(1) * 0.1 * i;
                p2 = factor.col(0) - factor.col(1) * 0.1 * i;
                p.x = p1.x(), p.y = p1.y(), p.z = p1.z();
                cloud.push_back(p);
                p.x = p2.x(), p.y = p2.y(), p.z = p2.z();
                cloud.push_back(p);
            }
        }
    }

    void visualize_factor(const std::vector<FactorParam> &param, const rclcpp::Time &stamp, int color, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub)
    {
        if (param.empty())
            return;

        static pcl::PointCloud<PointType> cloud;
        static sensor_msgs::msg::PointCloud2 msg;
        cloud.clear();

        if (pub->get_subscription_count() > 0)
        {
            convert_factor_to_cloud(param, cloud, color);
            pcl::toROSMsg(cloud, msg);
            msg.header.frame_id = odometryFrame;
            msg.header.stamp = stamp;
            pub->publish(msg);
        }
    }
};

Map_Optimization::Map_Optimization(const rclcpp::NodeOptions & options)
: Node("map_optimization_node", options)
{
    declare_parameter<bool>("loopClosureEnableFlag", false);
    declare_parameter<double>("loopClosureFrequency", 1.0);
    declare_parameter<bool>("savePCD", false);
    declare_parameter<int>("N_SCAN", 16);
    declare_parameter<int>("Horizon_SCAN", 1800);
    declare_parameter<double>("mappingProcessInterval", 0.15);
    declare_parameter<double>("globalMapVisualizationSearchRadius", 1000.0);
    declare_parameter<double>("globalMapVisualizationPoseDensity", 15.0);
    declare_parameter<double>("globalMapVisualizationLeafSize", 1.5);
    declare_parameter<std::string>("odometryFrame", "camera_init");
    declare_parameter<int>("surroundingKeyframeSize", 1800);
    declare_parameter<int>("surroundingKeyframeSearchRadius", 50);
    declare_parameter<int>("edgeFeatureMinValidNum", 10);
    declare_parameter<int>("surfFeatureMinValidNum", 200);
    declare_parameter<double>("z_tollerance", 100.0);
    declare_parameter<double>("rotation_tollerance", 100.0);
    declare_parameter<int>("historyKeyframeSearchNum", 15);
    declare_parameter<double>("surroundingkeyframeAddingAngleThreshold", 1.5);
    declare_parameter<double>("historyKeyframeSearchRadius", 15.0);
    declare_parameter<double>("historyKeyframeSearchTimeDiff", 1000.0);
    declare_parameter<double>("surroundingkeyframeAddingDistThreshold", 1.5);
    declare_parameter<double>("historyKeyframeFitnessScore", 0.5);
    declare_parameter<double>("odometrySurfLeafSize", 0.5);
    declare_parameter<double>("mappingCornerLeafSize", 0.2);
    declare_parameter<double>("mappingSurfLeafSize", 0.4);
    declare_parameter<double>("surroundingKeyframeDensity", 2.0);

    // 设置参数
    get_parameter("loopClosureEnableFlag", loopClosureEnableFlag);
    get_parameter("loopClosureFrequency", loopClosureFrequency);
    get_parameter("savePCD", savePCD);
    get_parameter("N_SCAN", N_SCAN);
    get_parameter("Horizon_SCAN", Horizon_SCAN);
    get_parameter("mappingProcessInterval", mappingProcessInterval);
    get_parameter("globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius);
    get_parameter("globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity);
    get_parameter("globalMapVisualizationLeafSize", globalMapVisualizationLeafSize);
    get_parameter("odometryFrame", odometryFrame);
    get_parameter("surroundingKeyframeSize", surroundingKeyframeSize);
    get_parameter("surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius);
    get_parameter("edgeFeatureMinValidNum", edgeFeatureMinValidNum);
    get_parameter("surfFeatureMinValidNum", surfFeatureMinValidNum);
    get_parameter("z_tollerance", z_tollerance);
    get_parameter("rotation_tollerance", rotation_tollerance);
    get_parameter("historyKeyframeSearchNum", historyKeyframeSearchNum);
    get_parameter("surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold);
    get_parameter("historyKeyframeSearchRadius", historyKeyframeSearchRadius);
    get_parameter("historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff);
    get_parameter("surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold);
    get_parameter("historyKeyframeFitnessScore", historyKeyframeFitnessScore);
    get_parameter("odometrySurfLeafSize", odometrySurfLeafSize);
    get_parameter("mappingCornerLeafSize", mappingCornerLeafSize);
    get_parameter("mappingSurfLeafSize", mappingSurfLeafSize);
    get_parameter("surroundingKeyframeDensity", surroundingKeyframeDensity);                             //


    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 订阅者初始化，消息均来源于laserOdometry
    odom_cloud_sub = this->create_subscription<lo_sam::msg::CloudInfo>(
        "odom_init/odom_cloud",
        rclcpp::QoS(100),
        std::bind(&Map_Optimization::odom_cloud_handler, this, std::placeholders::_1)); // 订阅初始激光里程计

    // [MVP0-IMU] MVP0_USE_IMU=1 时订阅 IMU，用陀螺仪积分提供帧间旋转初值
    if (std::getenv("MVP0_USE_IMU")) {
        std::string imu_topic = this->declare_parameter<std::string>("imu_topic", "/livox/imu");
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 200,
            std::bind(&Map_Optimization::imu_handler, this, std::placeholders::_1));
        imu_enabled_ = true;
        RCLCPP_INFO(this->get_logger(), "[MVP0-IMU] enabled, imu_topic=%s", imu_topic.c_str());
    }

    // [VIS] VIS_FACTOR=1 时订阅视觉累积位姿（body 系），供 addVisualFactor() 插值使用
    if (VIS_ON) {
        vis_abs_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vo_pnp/abs", 50,
            std::bind(&Map_Optimization::vis_abs_handler, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "[VIS] enabled: /vo_pnp/abs  sig_rot=%.3f sig_tr=%.3f robust=%.2f tol=%.2f",
                    VIS_SIG_ROT, VIS_SIG_TR, VIS_ROBUST, VIS_TOL);
    }


    pubKeyPoses = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapping/trajectory", 1);                     // 发布历史关键帧里程计
    pubLaserCloudSurround = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapping/map_global", 1);           // 发布局部关键帧map的特征点云
    pubLaserOdometryGlobal = this->create_publisher<nav_msgs::msg::Odometry>("mapping/odometry", 1);
    pubLaserOdometryIncremental = this->create_publisher<nav_msgs::msg::Odometry>("mapping/odometry_incremental", 1);
    pubPath = this->create_publisher<nav_msgs::msg::Path>("mapping/path", 1);

    pubHistoryKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapping/icp_loop_closure_history_cloud", 1);
    pubIcpKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapping/icp_loop_closure_corrected_cloud", 1);
    pubLoopConstraintEdge = this->create_publisher<visualization_msgs::msg::MarkerArray>("mapping/loop_closure_constraints", 1);

    pubRecentKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapping/map_local", 1);
    pubRecentKeyFrame = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapping/cloud_registered", 1);
    pubCloudRegisteredRaw = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapping/cloud_registered_raw", 1);

    pubRecentKeyFrame_corner = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapping/cloud_registered_corner", 1);

    pub_select_corner = this->create_publisher<sensor_msgs::msg::PointCloud2>("select_corner", 3);
    pub_select_surf = this->create_publisher<sensor_msgs::msg::PointCloud2>("select_surf", 3);

    downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
    downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

    allocateMemory();
}

Map_Optimization::~Map_Optimization()
{
    if (savePCD == true)
    {
        saveMap();
    }
}

// 初始化点云指针的内存空间
void Map_Optimization::allocateMemory()
{
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

    kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

    laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());   // corner feature set from odoOptimization
    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());     // surf feature set from odoOptimization
    laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
    laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());   // downsampled surf featuer set from odoOptimization

    laserCloudOri.reset(new pcl::PointCloud<PointType>());
    coeffSel.reset(new pcl::PointCloud<PointType>());

    laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
    coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
    laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
    laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
    coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
    laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

    std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

    laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

    kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

    for (int i = 0; i < 6; ++i)
    {
        transformTobeMapped[i] = 0;
    }

    matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

    edge_cloud_curr.reset(new pcl::PointCloud<PointType>());
    surf_cloud_curr.reset(new pcl::PointCloud<PointType>());
}

// =====================================================================================
// [MPROF] map_optimization 分段计时。MAP_PROF=1 打开；输出到 **stderr**（不缓冲，
//         避免 laserOdometry 那边裸 printf 的 4 KB 块缓冲坑），格式与 ROS 日志不冲突。
//         只用于逐帧分项耗时，不算速率（速率一律以 ros2 topic hz 为准）。
// =====================================================================================
#include <chrono>
#include <cstdio>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <map>
#include <cstdlib>
static const bool MP_ON = (std::getenv("MAP_PROF") != nullptr);

// ---------------------------------------------------------------------------------
// [A1] 迭代上限与收敛判据 —— 运行时环境变量，默认与原版完全一致（30 / 0.05 / 0.05）。
//      MAP_ITER_MAX  默认 30，钳到 [1,30]
//      MAP_CONV_R    默认 0.05（度）
//      MAP_CONV_T    默认 0.05（cm，代码里是 *100 后的量纲）
//      理由：实测 leaf0 最贵帧 tot 914 ms 且 it 打满 30 —— 即"没收敛还跑满 30 次"。
// ---------------------------------------------------------------------------------
static const int MP_ITER_MAX = []() {
        const char *v = std::getenv("MAP_ITER_MAX");
        int n = v ? std::atoi(v) : 30;
        if (n < 1) n = 1;
        if (n > 30) n = 30;
        return n;
    }();
static const double MP_CONV_R = []() {
        const char *v = std::getenv("MAP_CONV_R");
        double d = v ? std::atof(v) : 0.05;
        return d > 0 ? d : 0.05;
    }();
static const double MP_CONV_T = []() {
        const char *v = std::getenv("MAP_CONV_T");
        double d = v ? std::atof(v) : 0.05;
        return d > 0 ? d : 0.05;
    }();
enum {
    S_DESER, S_INIT, S_ESKF, S_DS, S_S2M, S_SKF, S_CORR, S_PUBO, S_PUBF,
    S_KDT, S_COR, S_SURF, S_COMB, S_LM,
    S_KADD, S_KUPD, S_KEST, S_KMARG, S_KCLOUD,
    S_PKP, S_PLOC, S_PREG, S_PPATH, S_N
};
static double mp_a[S_N];
static long   mp_n = 0, mp_it = 0, mp_kf = 0, mp_nsc = 0;
static size_t mp_dscor = 0, mp_nsurf = 0, mp_ncM = 0, mp_nsM = 0, mp_nkf = 0;
static inline void mp_tick(int s, std::chrono::steady_clock::time_point &t) {
    if (!MP_ON) return;
    auto n = std::chrono::steady_clock::now();
    mp_a[s] += std::chrono::duration<double, std::milli>(n - t).count();
    t = n;
}
static void mp_flush(double tot) {
    if (!MP_ON) return;
    fprintf(stderr,
        "[MPROF] n=%ld tot=%.1f | deser=%.1f init=%.1f eskf=%.1f ds=%.1f "
        "s2m=%.1f[it=%ld kdt=%.1f cor=%.1f surf=%.1f comb=%.1f lm=%.1f] "
        "skf=%.1f[add=%.1f upd=%.1f est=%.1f marg=%.1f kcld=%.1f] "
        "corr=%.1f pubo=%.1f pubf=%.1f[kp=%.1f loc=%.1f reg=%.1f path=%.1f] "
        "| nsc=%ld cor=%zu surf=%zu cM=%zu sM=%zu kf=%zu\n",
        mp_n, tot,
        mp_a[S_DESER], mp_a[S_INIT], mp_a[S_ESKF], mp_a[S_DS],
        mp_a[S_S2M], mp_it, mp_a[S_KDT], mp_a[S_COR], mp_a[S_SURF], mp_a[S_COMB], mp_a[S_LM],
        mp_a[S_SKF], mp_a[S_KADD], mp_a[S_KUPD], mp_a[S_KEST], mp_a[S_KMARG], mp_a[S_KCLOUD],
        mp_a[S_CORR], mp_a[S_PUBO], mp_a[S_PUBF],
        mp_a[S_PKP], mp_a[S_PLOC], mp_a[S_PREG], mp_a[S_PPATH],
        mp_nsc, mp_dscor, mp_nsurf, mp_ncM, mp_nsM, mp_nkf);
    fflush(stderr);
    for (int i = 0; i < S_N; ++i) mp_a[i] = 0.0;
    mp_it = 0; mp_nsc = 0;
}

void Map_Optimization::odom_cloud_handler(const std::shared_ptr<const lo_sam::msg::CloudInfo> & odom_cloud_msg)
{
    auto mp_fstart = std::chrono::steady_clock::now();
    auto mp_t0 = mp_fstart;
    pcl::fromROSMsg(odom_cloud_msg->cloud_edge, *laserCloudCornerLast);
    pcl::fromROSMsg(odom_cloud_msg->cloud_surf, *laserCloudSurfLast);
    mp_tick(S_DESER, mp_t0);

    // laserCloudCornerLast = odom_cloud_msg.;
    // laserCloudSurfLast = surf_cloud_curr;

    timeLaserInfoStamp = rclcpp::Time(odom_cloud_msg->header.stamp);
    timeLaserInfoCur = timeLaserInfoStamp.seconds();


    odom_init_curr = odom_cloud_msg->odom_init;

   

    std::lock_guard<std::mutex> lock(mtx);
    // mapping执行频率控制

    static double timeLastProcessing = -1;

    // std::cout << "time diff:" << timeLaserInfoCur - timeLastProcessing << std::endl;

    if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
    {
        timeLastProcessing = timeLaserInfoCur;
        auto clock = this->get_clock();
        rclcpp::Time t1 = clock->now();
        // 1.当前帧位姿初始化： 采用来自laserOdometry的初始激光里程计来初始位姿
        updateInitialGuess();
        mp_tick(S_INIT, mp_t0);

        // std::cout << "init trans(x,y,z): " << transformTobeMapped[3] << ", " << transformTobeMapped[4] << ", " << transformTobeMapped[5] << std::endl;

        // 2.提取局部角点、平面点云集合，加入局部map
        // (1)对最近的一帧关键帧，搜索时空维度上相邻的关键帧集合，降采样一下
        // (2)对关键帧集合中的每一帧，提取对应的角点、平面点，加入局部map中
        extractSurroundingKeyFrames();
        mp_tick(S_ESKF, mp_t0);

        // 3.当前激光帧角点、平面点集合降采样
        downsampleCurrentScan();
        mp_tick(S_DS, mp_t0);

        // 4.scan-to-map优化当前帧位姿
        // (1)要求当前帧特征点数量足够多，且匹配的点数够多，才执行优化
        // (2)迭代30次（上限）优化
        //    1) 当前激光帧角点寻找局部map匹配点
        //       a.更新当前帧位姿，将当前帧角点坐标变换到map系下，在局部map中查找5个最近点，距离小于1m，且5个点构成直线（用距离中心点的协方差矩阵，特征值进行判断），则认为匹配上了
        //       b.计算当前帧角点到直线的距离、垂线的单位向量，存储为角点参数
        //    2) 当前激光帧平面点寻找局部map匹配点
        //       a.更新当前帧位姿，将当前帧平面点坐标变换到map系下，在局部map中查找5个最近点，距离小于1m，且5个点构成平面（最小二乘拟合平面），则认为匹配上了
        //       b.计算当前帧平面点到平面的距离、垂线的单位向量，存储为平面点参数
        //    3) 提取当前帧中与局部map匹配上了的角点、平面点，加入同一集合
        //    4) 对匹配特征点计算Jacobian矩阵，观测值为特征点到直线、平面的距离，构建高斯牛顿方程，迭代优化当前位姿，存transformTobeMapped
        scan2MapOptimization();
        mp_tick(S_S2M, mp_t0);

        // std::cout << "scan-to-map optimization trans(x,y,z): " << transformTobeMapped[3] << ", " << transformTobeMapped[4] << ", " << transformTobeMapped[5] << std::endl;

        // 5.设置当前帧为关键帧并执行因子图优化
        // (1)计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
        // (2)添加激光里程计因子、闭环因子、杆件路标因子
        // (3)执行因子图优化
        // (4)得到当前帧优化后位姿，位姿协方差
        // (5)添加cloudKeyPoses3D，cloudKeyPoses6D，更新transformTobeMapped，添加当前关键帧的角点、平面点集合
        saveKeyFramesAndFactor();
        mp_tick(S_SKF, mp_t0);

        // 6.更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿，更新里程计轨迹
        correctPoses();
        mp_tick(S_CORR, mp_t0);

        // 7.发布激光里程计
        publishOdometry();
        mp_tick(S_PUBO, mp_t0);

        // 8.发布里程计、点云、轨迹
        // (1)发布历史关键帧位姿集合
        // (2)发布局部map的降采样平面点集合
        // (3)发布历史帧（累加的）的角点、平面点降采样集合
        // (4)发布里程计轨迹
        publishFrames();
        mp_tick(S_PUBF, mp_t0);
        rclcpp::Time t2 = this->now();
        if (MP_ON) {
            ++mp_n;
            mp_dscor = laserCloudCornerLastDSNum;
            mp_nsurf = laserCloudSurfLastDSNum;
            mp_ncM   = laserCloudCornerFromMapDSNum;
            mp_nsM   = laserCloudSurfFromMapDSNum;
            mp_nkf   = cloudKeyPoses3D->size();
            mp_flush(std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - mp_fstart).count());
        }
    }
    else {
        if (MP_ON) ++mp_nsc;   // 被 mappingProcessInterval 挡下的帧（只计 deser）
    }
}

sensor_msgs::msg::PointCloud2 Map_Optimization::publishCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub,
    const pcl::PointCloud<PointType>::Ptr thisCloud,
    const rclcpp::Time & thisStamp,
    const std::string & thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);

    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;

    if (thisPub && thisPub->get_subscription_count() > 0) {
        thisPub->publish(tempCloud);
    }

    return tempCloud;
}

// [MVP0-IMU] 陀螺仪流式积分：累积自上次消费以来的 body 系旋转增量
void Map_Optimization::imu_handler(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    const double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    const Eigen::Vector3d w(msg->angular_velocity.x, msg->angular_velocity.y,
                            msg->angular_velocity.z);
    std::lock_guard<std::mutex> lk(imu_mtx_);
    if (imu_t_last_ > 0) {
        const double dt = t - imu_t_last_;
        if (dt > 0 && dt < 0.5) {
            const double ang = w.norm() * dt;
            Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
            if (ang > 1e-12)
                dR = Eigen::AngleAxisd(ang, w / w.norm()).toRotationMatrix();
            imu_dR_ = imu_dR_ * dR;   // body 系右乘累积
            imu_dR_n_++;
        }
    }
    imu_t_last_ = t;
    imu_has_data_ = true;
}

// 当前帧位姿初始化
void Map_Optimization::updateInitialGuess()
{

    Eigen::Vector3d odom_init_t = Eigen::Vector3d(odom_init_curr.pose.pose.position.x, odom_init_curr.pose.pose.position.y, odom_init_curr.pose.pose.position.z);
    Eigen::Quaterniond odom_init_q(odom_init_curr.pose.pose.orientation.w, odom_init_curr.pose.pose.orientation.x, odom_init_curr.pose.pose.orientation.y, odom_init_curr.pose.pose.orientation.z);

    // 这里都采用来自laserOdometry的初始激光里程计进行初始化
    if (cloudKeyPoses3D->points.empty())
    {

        Eigen::Vector3d eular = odom_init_q.matrix().eulerAngles(2, 1, 0); // Yaw, Pitch, Roll
        transformTobeMapped[3] = odom_init_t(0);                           // x
        transformTobeMapped[4] = odom_init_t(1);                           // y
        transformTobeMapped[5] = odom_init_t(2);                           // z
        transformTobeMapped[0] = eular(2);                                 // roll
        transformTobeMapped[1] = eular(1);                                 // pitch
        transformTobeMapped[2] = eular(0);                                 // yaw

        tf_odom_last.setOrigin(tf2::Vector3(odom_init_t(0), odom_init_t(1), odom_init_t(2)));
        tf_odom_last.setRotation(tf2::Quaternion(odom_init_q.x(), odom_init_q.y(), odom_init_q.z(), odom_init_q.w()));
        return;
    }

    tf2::Transform tf_odom_curr;
    tf_odom_curr.setOrigin(tf2::Vector3(odom_init_t(0), odom_init_t(1), odom_init_t(2)));
    tf_odom_curr.setRotation(tf2::Quaternion(odom_init_q.x(), odom_init_q.y(), odom_init_q.z(), odom_init_q.w()));

    tf2::Transform tf_odom_increment = tf_odom_last.inverse() * tf_odom_curr;
    // [MVP0-IMU-FIX] 在【源头】把增量的旋转部分换成陀螺仪积分（平移仍用 laserOdometry）。
    // 原实现是在本函数末尾对 tf_odom_mapped_curr 左乘 dR —— 那里 transformTobeMapped[0..2]
    // 早已从 tf_odom_mapped_curr 提取完毕，且该变量是局部变量、改完再无人读 => 完全无效；
    // 而且那种「再乘一次」的写法会把帧间旋转累加两遍（laser 一遍 + IMU 一遍）。
    if (imu_enabled_)
    {
        Eigen::Matrix3d dR;
        int n_smp = 0;
        bool got = false;
        {
            std::lock_guard<std::mutex> lk(imu_mtx_);
            got = imu_has_data_;
            n_smp = imu_dR_n_;
            dR = imu_dR_;
            if (n_smp > 0) {
                imu_dR_.setIdentity();
                imu_dR_n_ = 0;
            }
        }
        // 只有确实积到样本才替换；否则保留激光增量，避免用 Identity 打坏初值
        if (got && n_smp > 0)
        {
            tf2::Quaternion q_dR;
            tf2::Matrix3x3(dR(0, 0), dR(0, 1), dR(0, 2),
                           dR(1, 0), dR(1, 1), dR(1, 2),
                           dR(2, 0), dR(2, 1), dR(2, 2)).getRotation(q_dR);
            q_dR.normalize();
            tf_odom_increment.setRotation(q_dR);
        }
    }

    tf2::Transform tf_odom_mapped_last;
    tf_odom_mapped_last.setOrigin(tf2::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
    tf2::Quaternion q;
    q.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    tf_odom_mapped_last.setRotation(q);

    tf2::Transform tf_odom_mapped_curr = tf_odom_mapped_last * tf_odom_increment;
    Eigen::Quaterniond q_curr(tf_odom_mapped_curr.getRotation().w(), tf_odom_mapped_curr.getRotation().x(),
                              tf_odom_mapped_curr.getRotation().y(), tf_odom_mapped_curr.getRotation().z());

    Eigen::Vector3d eulerAngle = q_curr.matrix().eulerAngles(2, 1, 0);
    transformTobeMapped[0] = eulerAngle(2);                       // roll
    transformTobeMapped[1] = eulerAngle(1);                       // pitch
    transformTobeMapped[2] = eulerAngle(0);                       // yaw
    transformTobeMapped[3] = tf_odom_mapped_curr.getOrigin().x(); // x
    transformTobeMapped[4] = tf_odom_mapped_curr.getOrigin().y(); // y
    transformTobeMapped[5] = tf_odom_mapped_curr.getOrigin().z(); // z

    tf_odom_last = tf_odom_curr;

    // [MVP0-IMU-FIX] 原实现在此处（对本函数局部变量 tf_odom_mapped_curr 左乘 dR）已删除 ——
    // 那是死代码：transformTobeMapped[0..2] 在上面就已提取完毕，之后改这个局部变量没有任何效果。
    // 现改到 tf_odom_increment 的源头替换（见上方 [MVP0-IMU-FIX]）。

    // // save current transformation before any processing
    // // 前一帧的位姿，注：这里指lidar的位姿，后面都简写成位姿
    // incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);
    // // 前一帧的初始化姿态角（来自原始imu数据），用于估计第一帧的位姿（旋转部分）
    // static Eigen::Affine3f lastImuTransformation;
    // // initialization
    // // 如果关键帧集合为空，继续进行初始化
    // if (cloudKeyPoses3D->points.empty())
    // {
    //     // 当前帧位姿的旋转部分，用激光帧信息中的RPY（来自imu原始数据）初始化
    //     transformTobeMapped[0] = cloudInfo.imuRollInit;
    //     transformTobeMapped[1] = cloudInfo.imuPitchInit;
    //     transformTobeMapped[2] = cloudInfo.imuYawInit;

    //     if (!useImuHeadingInitialization)
    //         transformTobeMapped[2] = 0;

    //     lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
    //     return;
    // }

    // // use imu pre-integration estimation for pose guess
    // // 用当前帧和前一帧对应的imu里程计计算相对位姿变换，再用前一帧的位姿与相对变换，计算当前帧的位姿，存transformTobeMapped
    // static bool lastImuPreTransAvailable = false;
    // static Eigen::Affine3f lastImuPreTransformation;
    // //odomAvailable和imuAvailable均来源于imageProjection.cpp中赋值，
    // //imuAvailable是遍历激光帧前后起止时刻0.01s之内的imu数据，
    // //如果都没有那就是false，因为imu频率一般比激光帧快，因此这里应该是都有的。
    // //odomAvailable同理，是监听imu里程计的位姿，如果没有紧挨着激光帧的imu里程计数据，那么就是false；
    // //这俩应该一般都有
    // if (cloudInfo.odomAvailable == true)
    // {
    //     // cloudInfo来自featureExtraction.cpp发布的lio_sam/feature/cloud_info,
    //     //而其中的initialGuessX等信息本质上来源于ImageProjection.cpp发布的deskew/cloud_info信息，
    //     //而deskew/cloud_info中的initialGuessX则来源于ImageProjection.cpp中的回调函数odometryHandler，
    //     //odometryHandler订阅的是imuPreintegration.cpp发布的odometry/imu_incremental话题，
    //     //该话题发布的xyz是imu在前一帧雷达基础上的增量位姿
    //     //纠正一个观点：增量位姿，指的绝不是预积分位姿！！是在前一帧雷达的基础上(包括该基础!!)的（基础不是0）的位姿
    //     //当前帧的初始估计位姿（来自imu里程计），后面用来计算增量位姿变换
    //     Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX,    cloudInfo.initialGuessY,     cloudInfo.initialGuessZ,
    //                                                        cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
    //     if (lastImuPreTransAvailable == false)
    //     {
    //         // 赋值给前一帧
    //         //lastImuPreTransAvailable是一个静态变量，初始被设置为false,之后就变成了true
    //         //也就是说这段只调用一次，就是初始时，把imu位姿赋值给lastImuPreTransformation
    //         lastImuPreTransformation = transBack;
    //         lastImuPreTransAvailable = true;
    //     } else {
    //         // 当前帧相对于前一帧的位姿变换，imu里程计计算得到
    //         //lastImuPreTransformation就是上一帧激光时刻的imu位姿,transBack是这一帧时刻的imu位姿
    //         //求完逆相乘以后才是增量，绝不可把imu_incremental发布的当成是两激光间的增量
    //         Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;

    //         // 前一帧的位姿
    //         Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);

    //         // 当前帧的位姿
    //         Eigen::Affine3f transFinal = transTobe * transIncre;
    //         //将transFinal传入，结果输出至transformTobeMapped中
    //         pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
    //                                                       transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    //         // 当前帧初始位姿赋值作为前一帧
    //         lastImuPreTransformation = transBack;

    //         lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
    //         return;
    //     }
    // }

    // // use imu incremental estimation for pose guess (only rotation)
    // // 只在第一帧调用（注意上面的return），用imu数据初始化当前帧位姿，仅初始化旋转部分
    // if (cloudInfo.imuAvailable == true)
    // {
    //     //注：这一时刻的transBack和之前if (cloudInfo.odomAvailable == true)内部的transBack不同，
    //     //之前获得的是initialGuessRoll等，但是在这里是imuRollInit，它来源于imageProjection中的imuQueue，直接存储原始imu数据的。
    //     //那么对于第一帧数据，目前的lastImuTransformation是initialGuessX等，即imu里程计的数据；
    //     //而transBack是imuRollInit是imu的瞬时原始数据roll、pitch和yaw三个角。
    //     //那么imuRollInit和initialGuessRoll这两者有啥区别呢？
    //     //imuRollInit是imu姿态角，在imageProjection中一收到，就马上赋值给它要发布的cloud_info，
    //     //而initialGuessRoll是imu里程计发布的姿态角。
    //     //直观上来说，imu原始数据收到速度是应该快于imu里程计的数据的，因此感觉二者之间应该有一个增量，
    //     //那么lastImuTransformation.inverse() * transBack算出增量，增量再和原先的transformTobeMapped计算,
    //     //结果依旧以transformTobeMapped来保存
    //     //感觉这里写的非常奇怪
    //     Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
    //     Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

    //     Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
    //     Eigen::Affine3f transFinal = transTobe * transIncre;
    //     pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
    //                                                   transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

    //     lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
    //     return;
    // }
}

void Map_Optimization::extractSurroundingKeyFrames()
{
    if (cloudKeyPoses3D->points.empty() == true)
    {
        return;
    }

    extractNearby();
}

void Map_Optimization::extractNearby()
{
    pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    // extract all the nearby key poses and downsample them
    // kdtree的输入，全局关键帧位姿集合（历史所有关键帧集合）
    kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
    // 创建Kd树然后搜索  半径在配置文件中
    // 指定半径范围查找近邻
    // 球状固定距离半径近邻搜索
    //  surroundingKeyframeSearchRadius是搜索半径，pointSearchInd应该是返回的index，pointSearchSqDis应该是依次距离中心点的距离
    kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);

    for (int i = 0; i < (int)pointSearchInd.size(); ++i)
    {
        int id = pointSearchInd[i];

        // 保存附近关键帧,加入相邻关键帧位姿集合中
        surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
    }
    // 降采样
    // 把相邻关键帧位姿集合，进行下采样，滤波后存入surroundingKeyPosesDS
    downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
    downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
    for (auto &pt : surroundingKeyPosesDS->points)
    {
        // k近邻搜索,找出最近的k个节点（这里是1）
        kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
        // 把强度替换掉，注意是从原始关键帧数据中替换的,相当于是把下采样以后的点的强度，换成是原始点强度
        // 注意，这里的intensity应该不是强度，因为在函数saveKeyFramesAndFactor中:
        //  thisPose3D.intensity = cloudKeyPoses3D->size();
        // 就是索引，只不过这里借用intensity结构来存放
        pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
    }

    // also extract some latest key frames in case the robot rotates in one position
    // 提取了一些最新的关键帧，以防机器人在一个位置原地旋转
    int numPoses = cloudKeyPoses3D->size();
    // 把10s内的关键帧也加到surroundingKeyPosesDS中,注意是“也”，原先已经装了下采样的位姿(位置)
    for (int i = numPoses - 1; i >= 0; --i)
    {
        if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
            surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
        else
            break;
    }
    // 对降采样后的点云进行提取出边缘点和平面点对应的localmap
    extractCloud(surroundingKeyPosesDS);
}

// 将相邻关键帧集合对应的角点、平面点，加入到局部map中，作为scan-to-map匹配的局部点云地图
// [A3] 分发：`MAP_ESKF=inc` 时走增量维护（extractCloudInc），否则走原实现（extractCloudBase）。
void Map_Optimization::extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
{
    if (!A3_INC) { extractCloudBase(cloudToExtract); return; }
    if (A3_CHECK_N > 0 && a3_frame < A3_CHECK_N)
    {
        // 对拍帧：先跑基线，留一份副本，再跑增量，然后比对两份 DS 地图。
        // ⚠ 基线路径会往 laserCloudCornerFromMapDS 里 filter；inc 模式下这个 Ptr 已指向
        //   增量表自己的输出云，必须先换回独立对象，否则会把增量输出覆盖掉。
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());
        extractCloudBase(cloudToExtract);
        pcl::PointCloud<PointType> baseCorner = *laserCloudCornerFromMapDS;
        pcl::PointCloud<PointType> baseSurf   = *laserCloudSurfFromMapDS;
        extractCloudInc(cloudToExtract);
        a3Check(baseCorner, baseSurf);
        ++a3_frame;
        return;
    }
    extractCloudInc(cloudToExtract);
}

void Map_Optimization::extractCloudBase(pcl::PointCloud<PointType>::Ptr cloudToExtract)
{
    // fuse the map
    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear();
    // 遍历当前帧（实际是取最近的一个关键帧来找它相邻的关键帧集合）时空维度上相邻的关键帧集合
    for (int i = 0; i < (int)cloudToExtract->size(); ++i)
    {
        // 距离超过阈值，丢弃
        if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
            continue;

        // 相邻关键帧索引
        int thisKeyInd = (int)cloudToExtract->points[i].intensity;
        if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end())
        {
            // transformed cloud available
            *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
            *laserCloudSurfFromMap += laserCloudMapContainer[thisKeyInd].second;
        }
        else
        {
            // transformed cloud not available
            // 相邻关键帧对应的角点、平面点云，通过6D位姿变换到世界坐标系下
            // transformPointCloud输入的两个形参，分别为点云和变换，返回变换位姿后的点
            pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
            pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
            // 加入局部map
            *laserCloudCornerFromMap += laserCloudCornerTemp;
            *laserCloudSurfFromMap += laserCloudSurfTemp;
            laserCloudMapContainer[thisKeyInd] = std::make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
        }
    }

    // Downsample the surrounding corner key frames (or map)
    // 降采样局部角点map
    downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
    downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
    laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
    // Downsample the surrounding surf key frames (or map)
    // 降采样局部平面点map
    downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
    downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
    laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

    // clear map cache if too large
    // 太大了，清空一下内存
    if (laserCloudMapContainer.size() > 1000)
        laserCloudMapContainer.clear();
}

// =====================================================================================
// [A3] 增量维护局部地图：只处理"本帧新进来 / 退出去"的关键帧
//   每帧成本从 O(全部相邻关键帧点数 + 全量 VoxelGrid) 降到 O(变化的点数 + 输出点数)。
// =====================================================================================
void Map_Optimization::extractCloudInc(pcl::PointCloud<PointType>::Ptr cloudToExtract)
{
    static bool _a3_print = false;
    if (!_a3_print)
    {
        _a3_print = true;
        RCLCPP_INFO(this->get_logger(), "[A3] eskf=inc cornerLeaf=%.2f surfLeaf=%.2f check_n=%d",
                    mappingCornerLeafSize, mappingSurfLeafSize, A3_CHECK_N);
    }

    // ---- 1. 本帧想要的关键帧集合（kf → 出现次数）；距离过滤与原码一致 ----
    std::map<int, int> want;
    const PointType &latest = cloudKeyPoses3D->back();
    for (size_t i = 0; i < cloudToExtract->size(); ++i)
    {
        if (pointDistance(cloudToExtract->points[i], latest) > surroundingKeyframeSearchRadius)
            continue;
        ++want[(int)cloudToExtract->points[i].intensity];
    }

    // 首次调用（或被执行过 reset）：按当前 leaf 初始化
    if (incCorner.cells.empty() && incCorner.active.empty())
    {
        incCorner.reset((float)mappingCornerLeafSize);
        incSurf.reset((float)mappingSurfLeafSize);
        incPose.clear();
    }

    // 取世界系点云：命中缓存直接用，否则现算并回填缓存（原码也是这套缓存）
    auto getWorld = [&](int kf) -> bool {
        if (laserCloudMapContainer.find(kf) != laserCloudMapContainer.end()) return true;
        if (kf < 0 || kf >= (int)cornerCloudKeyFrames.size() || kf >= (int)surfCloudKeyFrames.size())
            return false;
        pcl::PointCloud<PointType> c = *transformPointCloud(cornerCloudKeyFrames[kf], &cloudKeyPoses6D->points[kf]);
        pcl::PointCloud<PointType> s = *transformPointCloud(surfCloudKeyFrames[kf], &cloudKeyPoses6D->points[kf]);
        laserCloudMapContainer[kf] = std::make_pair(c, s);
        return true;
    };
    auto notePose = [&](int kf) {
        const PointTypePose &p = cloudKeyPoses6D->points[kf];
        incPose[kf] = {p.x, p.y, p.z, p.roll, p.pitch, p.yaw};
    };

    // ---- 2. 位姿若被闭环优化改过，增量账全部作废 ----
    bool poseChanged = false;
    for (auto &kv : want)
    {
        const int kf = kv.first;
        if (kf < 0 || kf >= (int)cloudKeyPoses6D->size()) { poseChanged = true; break; }
        auto it = incPose.find(kf);
        if (it == incPose.end()) continue;
        const PointTypePose &p = cloudKeyPoses6D->points[kf];
        const std::array<float, 6> &q = it->second;
        if (q[0] != p.x || q[1] != p.y || q[2] != p.z ||
            q[3] != p.roll || q[4] != p.pitch || q[5] != p.yaw) { poseChanged = true; break; }
    }

    bool ok = !poseChanged;
    size_t n_add = 0, n_del = 0;
    bool fullRebuilt = false;
    if (ok)
    {
        // ---- 3. 与上一帧的集合求差 ----
        std::vector<int> toRemove;
        std::vector<std::pair<int, int>> toAdd;
        for (auto &kv : incCorner.active)
            if (want.find(kv.first) == want.end()) toRemove.push_back(kv.first);
        for (auto &kv : want)
        {
            auto it = incCorner.active.find(kv.first);
            const int have = (it == incCorner.active.end()) ? 0 : it->second;
            if (have != kv.second) { if (have > 0) toRemove.push_back(kv.first); toAdd.push_back(kv); }
        }
        for (size_t t = 0; t < toRemove.size() && ok; ++t)
        {
            const int kf = toRemove[t];
            if (!getWorld(kf)) { ok = false; break; }
            const int m = incCorner.active[kf];
            for (int r = 0; r < m && ok; ++r)
                ok = incCorner.removeKf(kf, laserCloudMapContainer[kf].first)
                  && incSurf.removeKf(kf, laserCloudMapContainer[kf].second);
            incCorner.active.erase(kf);
            incSurf.active.erase(kf);
            incPose.erase(kf);
            ++n_del;
        }
        for (size_t t = 0; t < toAdd.size() && ok; ++t)
        {
            const int kf = toAdd[t].first, m = toAdd[t].second;
            if (!getWorld(kf)) { ok = false; break; }
            for (int r = 0; r < m; ++r)
            {
                incCorner.addKf(kf, laserCloudMapContainer[kf].first);
                incSurf.addKf(kf, laserCloudMapContainer[kf].second);
            }
            incCorner.active[kf] = m;
            incSurf.active[kf]  = m;
            notePose(kf);
            ++n_add;
        }
    }

    // ---- 4. 兜底：整体重建（位姿变化 / 账对不上 / 缓存被清空）----
    if (!ok)
    {
        fullRebuilt = true;
        incCorner.reset((float)mappingCornerLeafSize);
        incSurf.reset((float)mappingSurfLeafSize);
        incPose.clear();
        for (auto &kv : want)
        {
            const int kf = kv.first, m = kv.second;
            if (!getWorld(kf)) continue;
            for (int r = 0; r < m; ++r)
            {
                incCorner.addKf(kf, laserCloudMapContainer[kf].first);
                incSurf.addKf(kf, laserCloudMapContainer[kf].second);
            }
            incCorner.active[kf] = m;
            incSurf.active[kf]  = m;
            notePose(kf);
        }
    }

    // ---- 5. 只刷新脏桶，输出云直接挂到成员 Ptr（不拷贝）----
    const auto t_rb0 = std::chrono::steady_clock::now();
    incCorner.flush();
    incSurf.flush();
    const double rb_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - t_rb0).count();
    laserCloudCornerFromMapDS = incCorner.out;
    laserCloudSurfFromMapDS   = incSurf.out;
    laserCloudCornerFromMapDSNum = (int)laserCloudCornerFromMapDS->size();
    laserCloudSurfFromMapDSNum   = (int)laserCloudSurfFromMapDS->size();

    if (MP_ON)
    {
        RCLCPP_INFO(this->get_logger(),
                    "[A3-D] want=%zu add=%zu del=%zu full=%d cCell=%zu sCell=%zu rb=%.2fms",
                    want.size(), n_add, n_del, (int)fullRebuilt,
                    incCorner.cells.size(), incSurf.cells.size(),
                    incCorner.dirty.size() + incSurf.dirty.size(), rb_ms);
    }
}

// =====================================================================================
// [A3] 诊断：同一帧两条路径各出一份 DS 地图，互为最近邻比对。
//   nnd_max 就是"同一体素质心因浮点求和分组不同而产生的偏差"，量级应在 1e-6 m。
// =====================================================================================
void Map_Optimization::a3Check(const pcl::PointCloud<PointType> &baseCorner,
                               const pcl::PointCloud<PointType> &baseSurf)
{
    auto one = [&](const char *tag,
                   const pcl::PointCloud<PointType> &a,
                   const pcl::PointCloud<PointType> &b) {
        if (a.empty() || b.empty())
        {
            RCLCPP_INFO(this->get_logger(), "[A3-CHK] frame=%ld %s base=%zu inc=%zu (空，跳过)",
                        a3_frame, tag, a.size(), b.size());
            return;
        }
        pcl::PointCloud<PointType>::Ptr pa(new pcl::PointCloud<PointType>(a));
        pcl::PointCloud<PointType>::Ptr pb(new pcl::PointCloud<PointType>(b));
        pcl::KdTreeFLANN<PointType> kdA, kdB;
        kdA.setInputCloud(pa);
        kdB.setInputCloud(pb);
        double maxAB = 0.0, maxBA = 0.0;
        long gt1e4 = 0, gt1e3 = 0;
        std::vector<int>   idx;
        std::vector<float> d2;
        for (size_t i = 0; i < a.size(); ++i)
        {
            kdB.nearestKSearch(a.points[i], 1, idx, d2);
            const double d = std::sqrt((double)d2[0]);
            if (d > maxAB) maxAB = d;
            if (d > 1e-4) ++gt1e4;
            if (d > 1e-3) ++gt1e3;
        }
        for (size_t i = 0; i < b.size(); ++i)
        {
            kdA.nearestKSearch(b.points[i], 1, idx, d2);
            const double d = std::sqrt((double)d2[0]);
            if (d > maxBA) maxBA = d;
        }
        RCLCPP_INFO(this->get_logger(),
                    "[A3-CHK] frame=%ld %s base=%zu inc=%zu nn_max=%.7f (逆)%.7f gt1e-4=%ld gt1e-3=%ld",
                    a3_frame, tag, a.size(), b.size(), maxAB, maxBA, gt1e4, gt1e3);
    };
    one("corner", baseCorner, *laserCloudCornerFromMapDS);
    one("surf",   baseSurf,   *laserCloudSurfFromMapDS);
}

// 提取局部角点、平面点云集合，加入局部map
void Map_Optimization::extractForLoopClosure()
{
    // 1、对最近的一帧关键帧，搜索时空维度上相邻的关键帧集合，降采样一下
    // 2、对关键帧集合中的每一帧，提取对应的角点、平面点，加入局部map中
    pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
    int numPoses = cloudKeyPoses3D->size();
    for (int i = numPoses - 1; i >= 0; --i)
    {
        if ((int)cloudToExtract->size() <= surroundingKeyframeSize)
            cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
        else
            break;
    }

    // [A3] 闭环重建的集合与每帧的相邻集合不同，强制走基线实现，避免增量表来回切换。
    // 同理先给基线换回独立输出对象，免得写进增量表的输出云。
    if (A3_INC)
    {
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());
    }
    extractCloudBase(cloudToExtract);
}

// 对激光帧角点和平面点集合进行降采样
void Map_Optimization::downsampleCurrentScan()
{

    // Downsample cloud from current scan
    // 对当前帧点云降采样  刚刚完成了周围关键帧的降采样
    // 大量的降采样工作无非是为了使点云稀疏化 加快匹配以及实时性要求
    laserCloudCornerLastDS->clear();
    downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
    downSizeFilterCorner.filter(*laserCloudCornerLastDS);
    laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

    laserCloudSurfLastDS->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfLastDS);
    laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
}

// scan-to-map优化当前帧位姿
// =====================================================================================
// [A2] 诊断：逐点对拍 voxel 栅格与 kdtree 的 5-NN。
//   只在 MAP_NN=voxel 且帧序号 < MAP_NN_CHECK 时执行（串行，不影响正常计时口径）。
//   判读：set_diff=0 且 gate_diff=0 ⇒ 两种 NN 给出的配准点集完全一致 ⇒ 轨迹应逐位相同。
// =====================================================================================
void Map_Optimization::a2ParityCheck()
{
    updatePointAssociateToMap();

    struct Res {
        long q = 0, kacc = 0, vacc = 0, same = 0, diff = 0, gate = 0, r1 = 0, r2 = 0, rn = 0, few = 0;
        double d5max = 0.0;
    };

    auto run = [&](const char *tag,
                   const pcl::PointCloud<PointType>::Ptr &qcloud,
                   const VoxelGridNN &nn,
                   const pcl::KdTreeFLANN<PointType>::Ptr &kd) {
        Res z;
        std::vector<int>   ki;
        std::vector<float> kd2;
        int   vi[5];
        float vd[5];
        for (size_t i = 0; i < qcloud->size(); ++i)
        {
            PointType ps;
            pointAssociateToMap(&qcloud->points[i], &ps);
            kd->nearestKSearch(ps, 5, ki, kd2);
            if (kd2.size() < 5) { ++z.few; continue; }
            ++z.q;
            int ru = -1;
            const int c = nn.knn5(ps, vi, vd, &ru);
            const bool kOk = (kd2[4] < 1.0f);
            const bool vOk = (c >= 5) && (vd[4] < 1.0f);
            if (kOk) ++z.kacc;
            if (vOk) ++z.vacc;
            if (kOk != vOk) ++z.gate;
            if (ru == 1) ++z.r1; else if (ru == 2) ++z.r2; else if (ru > 2) ++z.rn;
            if (!kOk && !vOk) continue; // 两者都不接受，点集差异无意义
            if (c < 5) { ++z.diff; continue; }
            const double dd = std::fabs((double)kd2[4] - (double)vd[4]);
            if (dd > z.d5max) z.d5max = dd;
            int a[5], b[5];
            for (int j = 0; j < 5; ++j) { a[j] = ki[j]; b[j] = vi[j]; }
            std::sort(a, a + 5);
            std::sort(b, b + 5);
            bool eq = true;
            for (int j = 0; j < 5; ++j) if (a[j] != b[j]) { eq = false; break; }
            if (eq) ++z.same; else ++z.diff;
        }
        RCLCPP_INFO(this->get_logger(),
                    "[A2-CHK] frame=%ld %s q=%ld kacc=%ld vacc=%ld same=%ld set_diff=%ld "
                    "gate_diff=%ld d5max=%.6f ring1=%ld ring2=%ld ringN=%ld few=%ld",
                    a2_frame, tag, z.q, z.kacc, z.vacc, z.same, z.diff, z.gate,
                    z.d5max, z.r1, z.r2, z.rn, z.few);
    };

    run("corner", laserCloudCornerLastDS, nnCornerMap, kdtreeCornerFromMap);
    run("surf",   laserCloudSurfLastDS,   nnSurfMap,   kdtreeSurfFromMap);
    ++a2_frame;
}

// [A4] 把一帧 scan-to-map 的四条点云 + 位姿写成裸 float32：
//   int32 magic 'S2M1', int32 版本=1
//   int32 nCorMap, nSurMap, nCorQry, nSurQry
//   float32 pose[6]   (roll,pitch,yaw,x,y,z  与 transformTobeMapped 同序)
//   float32 corMap[nCorMap][4]  (x,y,z,intensity)
//   float32 surMap[nSurMap][4]
//   float32 corQry[nCorQry][4]  (传感器坐标，未变换)
//   float32 surQry[nSurQry][4]
void Map_Optimization::dumpS2M(const char *path)
{
    FILE *f = fopen(path, "wb");
    if (!f) { RCLCPP_WARN(this->get_logger(), "[A4] 打不开 %s", path); return; }
    const int32_t magic = 0x314D3253; // 'S2M1'
    const int32_t ver = 1;
    const int32_t n[4] = {(int32_t)laserCloudCornerFromMapDS->size(),
                          (int32_t)laserCloudSurfFromMapDS->size(),
                          (int32_t)laserCloudCornerLastDS->size(),
                          (int32_t)laserCloudSurfLastDS->size()};
    fwrite(&magic, 4, 1, f); fwrite(&ver, 4, 1, f); fwrite(n, 4, 4, f);
    float pose[6];
    for (int i = 0; i < 6; ++i) pose[i] = (float)transformTobeMapped[i];
    fwrite(pose, 4, 6, f);
    auto dumpCloud = [&](const pcl::PointCloud<PointType>::Ptr &c) {
        for (size_t i = 0; i < c->size(); ++i)
        {
            float v[4] = {c->points[i].x, c->points[i].y, c->points[i].z, c->points[i].intensity};
            fwrite(v, 4, 4, f);
        }
    };
    dumpCloud(laserCloudCornerFromMapDS);
    dumpCloud(laserCloudSurfFromMapDS);
    dumpCloud(laserCloudCornerLastDS);
    dumpCloud(laserCloudSurfLastDS);
    fclose(f);
    RCLCPP_INFO(this->get_logger(), "[A4] 已写 %s : corMap=%d surMap=%d corQry=%d surQry=%d",
                path, n[0], n[1], n[2], n[3]);
}

// =====================================================================================
// [PL v2a] 参考路径：窗口打包 + 27 邻域查表 + 残差 + J，与 CPU 原路径逐项对拍。
// 目的不是替代，而是**在接硬件之前**证明"CPU 侧要交给 PL 的那一套"是对的：
//   ① 哈希表拷成「位图 + 平铺表」不丢格；
//   ② 27 邻域 / 格心最近 / RMAX 的查询与 A8 同口径（命中率应接近 a8_nhit/a8_nq）；
//   ③ 残差与 J 与原码同式（J 的部分另由 pl/test_m.cpp 单独验证，max|dJ|=3.8e-6）。
// 用 MAP_PL_REF=n 打开前 n 帧。
// =====================================================================================
static const int PL_REF_N = []() {
    const char *v = std::getenv("MAP_PL_REF");
    return v ? std::atoi(v) : 0;
}();

static void pl_ref_check_surf(const pcl::PointCloud<PointType> &qcloud,
                              const PlaneVoxelMap &pmap,
                              float leaf, float rmax,
                              const float s6[6],
                              const Eigen::Affine3f &T,
                              const std::vector<PointType> &oriVec,
                              const std::vector<PointType> &csVec,
                              const std::vector<bool> &flag,
                              int nSurf,
                              long frame)
{
    const int n = (int)qcloud.size();
    if (n < 50) return;

    // ---- 1) q（传感器系）/ p（世界系） + 包围盒 ----
    std::vector<float> qs(3 * n), ps(3 * n);
    float mnx = 1e30f, mxx = -1e30f, mny = 1e30f, mxy = -1e30f, mnz = 1e30f, mxz = -1e30f;
    for (int i = 0; i < n; ++i)
    {
        const float qx = qcloud.points[i].x, qy = qcloud.points[i].y, qz = qcloud.points[i].z;
        qs[3 * i + 0] = qx; qs[3 * i + 1] = qy; qs[3 * i + 2] = qz;
        const Eigen::Vector3f pp = T * Eigen::Vector3f(qx, qy, qz);
        ps[3 * i + 0] = pp.x(); ps[3 * i + 1] = pp.y(); ps[3 * i + 2] = pp.z();
        mnx = std::min(mnx, pp.x()); mxx = std::max(mxx, pp.x());
        mny = std::min(mny, pp.y()); mxy = std::max(mxy, pp.y());
        mnz = std::min(mnz, pp.z()); mxz = std::max(mxz, pp.z());
    }

    // ---- 2) 窗口（用点云包围盒中心，保证本帧的点全落在窗口内）----
    // ★ 必须用 pmap.leaf（表**实际生效**的格边长 = round(req/leaf)*leaf），不能用环境变量
    //   请求的 A8_LEAF —— 两者不一致时格索引会整体错位，窗口里就查不到格（命中率被砍到 1/3）。
    const float LF = pmap.leaf;
    // ★ 窗口尺寸必须与 PL 内核的编译期 WX/WY/WZ 一致，否则命中率被窗口外点拉低。
    //   默认 128x128x24（原行为）；S2M_WIN=32,32,8 可切到内核当前配置。
    static int WXW = 128, WYW = 128, WZW = 24;
    static bool _win_init = false;
    if (!_win_init)
    {
        _win_init = true;
        const char *we = std::getenv("S2M_WIN");
        if (we && *we)
        {
            int a = 0, b = 0, c = 0;
            if (std::sscanf(we, "%d,%d,%d", &a, &b, &c) == 3 && a > 0 && b > 0 && c > 0)
            {
                WXW = a; WYW = b; WZW = c;
                RCLCPP_INFO(rclcpp::get_logger("map_optimization"),
                            "[PL-REF] window override = %dx%dx%d (must match PL WX/WY/WZ)", WXW, WYW, WZW);
            }
        }
    }
    const int ci0 = (int)std::floor(0.5f * (mnx + mxx) / LF) - WXW / 2;
    const int cj0 = (int)std::floor(0.5f * (mny + mxy) / LF) - WYW / 2;
    const int ck0 = (int)std::floor(0.5f * (mnz + mxz) / LF) - WZW / 2;

    static pls2m::PlWin w;
    static long last_frame = -1;
    static long packed_valid = 0, packed_all = 0;
    static bool _diag = false;
    if (!_diag)
    {
        _diag = true;
        long v_all = 0;
        for (size_t bi = 0; bi < pmap.valid.size(); ++bi) if (pmap.valid[bi]) ++v_all;
        RCLCPP_INFO(rclcpp::get_logger("map_optimization"),
                    "[PL-REF] leaf req=%.3f table=%.3f | cells=%ld valid_all=%ld | bbox=(%.1f,%.1f,%.1f) span=(%.1f,%.1f,%.1f)",
                    (double)leaf, (double)pmap.leaf, (long)pmap.keys.size(), v_all,
                    0.5 * (mnx + mxx), 0.5 * (mny + mxy), 0.5 * (mnz + mxz),
                    (double)(mxx - mnx), (double)(mxy - mny), (double)(mxz - mnz));
    }
    if (last_frame != frame)
    {
        w.reset(WXW, WYW, WZW, LF, ci0, cj0, ck0);
        w.pack(pmap.keys, pmap.valid, pmap.pl.data(), 4);
        packed_valid = w.nvalid;
        packed_all = w.ncells;
        last_frame = frame;
    }

    // ---- 3) 参考实现 ----
    float M[27];
    pls2m::calc_M(s6, M);
    pls2m::RefOut ro;
    pls2m::ref_Hb(w, qs.data(), ps.data(), n, M, 0, rmax, ro);

    // ---- 4) 原路径的 H/b（只取 surf 部分、且只取落在窗口内的点，与 ref 同口径）----
    float H0[21], b0[6];
    for (int k = 0; k < 21; ++k) H0[k] = 0.f;
    for (int k = 0; k < 6; ++k)  b0[k] = 0.f;
    int nsel = 0, nout = 0;
    float hmax = 0.f;
    const float wx0 = w.orgx, wy0 = w.orgy, wz0 = w.orgz;
    const float wx1 = w.orgx + w.dimx * LF, wy1 = w.orgy + w.dimy * LF, wz1 = w.orgz + w.dimz * LF;
    for (int i = 0; i < nSurf; ++i)
    {
        if (!flag[i]) continue;
        const float _px = ps[3 * i + 0], _py = ps[3 * i + 1], _pz = ps[3 * i + 2];
        if (_px < wx0 || _px >= wx1 || _py < wy0 || _py >= wy1 || _pz < wz0 || _pz >= wz1) { ++nout; continue; }
        ++nsel;
        // laserCloudOriSurfVec 存的是**未换轴**的传感器系点与 coeff，这里按 LMOptimization 换轴
        const float pc0 = oriVec[i].y, pc1 = oriVec[i].z, pc2 = oriVec[i].x;
        const float cc0 = csVec[i].y,  cc1 = csVec[i].z,  cc2 = csVec[i].x;
        const float bv  = -csVec[i].intensity;
        float J[6];
        J[0] = (M[0] * pc0 + M[1] * pc1 + M[2] * pc2) * cc0
             + (M[3] * pc0 + M[4] * pc1 + M[5] * pc2) * cc1
             + (M[6] * pc0 + M[7] * pc1 + M[8] * pc2) * cc2;
        J[1] = (M[9] * pc0 + M[10] * pc1 + M[11] * pc2) * cc0
             + (M[12] * pc0 + M[13] * pc1 + M[14] * pc2) * cc1
             + (M[15] * pc0 + M[16] * pc1 + M[17] * pc2) * cc2;
        J[2] = (M[18] * pc0 + M[19] * pc1 + M[20] * pc2) * cc0
             + (M[21] * pc0 + M[22] * pc1 + M[23] * pc2) * cc1
             + (M[24] * pc0 + M[25] * pc1 + M[26] * pc2) * cc2;
        J[3] = cc1; J[4] = cc2; J[5] = cc0;   // coeff.z / .x / .y
        for (int k = 0; k < 21; ++k)
        {
            H0[k] += J[pls2m::HI27[k]] * J[pls2m::HJ27[k]];
            if (std::fabs(H0[k]) > hmax) hmax = std::fabs(H0[k]);
        }
        for (int k = 0; k < 6; ++k) b0[k] += J[k] * bv;
    }

    // ---- [PL-DUMP] 导出 PL 输入集 + 两组黄金值（S2M_DUMP_DIR 设了才做）----
    //   只导出指定帧，避免每帧写盘。q/vbw/tab/parq/parm 就是喂 IP 的那一份字节。
    {
        const char *dd = std::getenv("S2M_DUMP_DIR");
        static int dump_frame = -99999;
        if (dump_frame == -99999)
        {
            const char *fe = std::getenv("S2M_DUMP_FRAME");
            dump_frame = fe ? std::atoi(fe) : 0;
        }
        if (dd && *dd && (int)frame == dump_frame)
        {
            const int DX = w.dimx, DY = w.dimy, DZ = w.dimz;
            const long WM = (long)DX * DY * DZ;
            const int VW = (int)((WM + 31) / 32);
            auto wopen = [&](const char *nm, const char *mode) -> FILE * {
                char pth[512];
                std::snprintf(pth, sizeof(pth), "%s/%s", dd, nm);
                return std::fopen(pth, mode);
            };

            if (FILE *f = wopen("meta.txt", "w"))
            {
                std::fprintf(f, "NQ=%d mode=0 rescan=0\n", n);
                std::fprintf(f, "L=%.9g\ninvL=%.9g\nrmax=%.9g\n",
                             (double)w.L, (double)w.invL, (double)rmax);
                std::fprintf(f, "WMAX=%ld\nvbw_words=%d\n", WM, VW);
                std::fprintf(f, "org=%.9g %.9g %.9g\n",
                             (double)w.orgx, (double)w.orgy, (double)w.orgz);
                std::fprintf(f, "dim=%d %d %d\n", DX, DY, DZ);
                std::fclose(f);
            }

            // q：内核按 4 分量读（q[4t+k]），第 4 个填 1
            if (FILE *f = wopen("q.bin", "wb"))
            {
                std::vector<float> qb((size_t)n * 4);
                for (int i = 0; i < n; ++i)
                {
                    qb[(size_t)4 * i + 0] = qs[3 * i + 0];
                    qb[(size_t)4 * i + 1] = qs[3 * i + 1];
                    qb[(size_t)4 * i + 2] = qs[3 * i + 2];
                    qb[(size_t)4 * i + 3] = 1.f;
                }
                std::fwrite(qb.data(), sizeof(float), qb.size(), f);
                std::fclose(f);
            }

            // vbw：uint8 位图 → uint32 字（显式拼装，不依赖端序）
            if (FILE *f = wopen("vbw.bin", "wb"))
            {
                std::vector<uint32_t> vbw((size_t)VW, 0u);
                const uint8_t *srcb = w.vb.data();
                for (int j = 0; j < VW; ++j)
                {
                    uint32_t v = 0;
                    for (int b = 0; b < 4; ++b)
                    {
                        const size_t bi = (size_t)j * 4 + b;
                        if (bi < w.vb.size()) v |= ((uint32_t)srcb[bi]) << (8 * b);
                    }
                    vbw[(size_t)j] = v;
                }
                std::fwrite(vbw.data(), sizeof(uint32_t), vbw.size(), f);
                std::fclose(f);
            }

            if (FILE *f = wopen("tab.bin", "wb"))
            {
                std::fwrite(w.tab.data(), sizeof(float), w.tab.size(), f);
                std::fclose(f);
            }

            // parq：R row-major 9 + t 3（内核 R 用 F16、t 用 F12 定点）
            if (FILE *f = wopen("parq.bin", "wb"))
            {
                float pq[12];
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c) pq[r * 3 + c] = T.linear()(r, c);
                pq[9]  = T.translation().x();
                pq[10] = T.translation().y();
                pq[11] = T.translation().z();
                std::fwrite(pq, sizeof(float), 12, f);
                std::fclose(f);
            }

            // parm：内核 for i<27 全部读 ⇒ 必须整 27 项（只填前 9 项会静默丢掉 arx/ary 行）
            if (FILE *f = wopen("parm.bin", "wb"))
            {
                std::fwrite(M, sizeof(float), 27, f);
                std::fclose(f);
            }

            // golden：ref_Hb 口径；golden_orig：CPU 原路径口径（仅窗口内点）
            if (FILE *f = wopen("golden.bin", "wb"))
            {
                float g[27];
                for (int k = 0; k < 21; ++k) g[k] = ro.H[k];
                for (int k = 0; k < 6; ++k)  g[21 + k] = ro.b[k];
                std::fwrite(g, sizeof(float), 27, f);
                std::fclose(f);
            }
            if (FILE *f = wopen("golden_orig.bin", "wb"))
            {
                float g[27];
                for (int k = 0; k < 21; ++k) g[k] = H0[k];
                for (int k = 0; k < 6; ++k)  g[21 + k] = b0[k];
                std::fwrite(g, sizeof(float), 27, f);
                std::fclose(f);
            }

            RCLCPP_INFO(rclcpp::get_logger("map_optimization"),
                        "[PL-DUMP] f=%ld -> %s | nq=%d win=%dx%dx%d valid=%ld hit=%ld nsel=%d",
                        frame, dd, n, DX, DY, DZ, (long)w.nvalid, ro.nhit, nsel);
        }
    }

    double nH = 0.0, dH = 0.0, nb = 0.0, db = 0.0;
    for (int k = 0; k < 21; ++k) { nH += (double)H0[k] * H0[k]; dH += (double)(H0[k] - ro.H[k]) * (H0[k] - ro.H[k]); }
    for (int k = 0; k < 6; ++k)  { nb += (double)b0[k] * b0[k]; db += (double)(b0[k] - ro.b[k]) * (b0[k] - ro.b[k]); }
    RCLCPP_INFO(rclcpp::get_logger("map_optimization"),
                "[PL-REF] f=%ld q=%d ref_hit=%ld orig_sel=%d(out=%d) | miss(cell=%ld rmax=%ld s=%ld) diffcell=%ld | relH=%.3e relb=%.3e",
                frame, n, ro.nhit, nsel, nout,
                ro.nmiss_cell, ro.nmiss_rmax, ro.nmiss_s, ro.ndiff_cell,
                std::sqrt(dH / std::max(nH, 1e-30)), std::sqrt(db / std::max(nb, 1e-30)));
}

// =====================================================================================
// [PL-S2M] 平面残差块交给 PL：打包 32x32x8 子窗（按查询点分块）→ 写 /dev/shm → 等 → 累加 H/b
//
// 分块规则：桶步长 (30,30,6)，桶内点全局格号 ∈ [base, base+29/29/5]，
//           27 邻域 ⊂ 窗口 [base-1, base+31/31/7) 正好装下（已验证：Python 分块累加 3.8e-8）。
// 窗口尺寸必须与内核编译期 WX/WY/WZ = 32/32/8 一致。
// =====================================================================================
static const bool PL_S2M_ON = []() {
    const char *v = std::getenv("MAP_PL_S2M");
    return v && std::atoi(v) != 0;
}();
static const int kShmHdr  = 4096;
static const int kShmQMax = 4096 * 4;          // float 数
static const int kWX = 32, kWY = 32, kWZ = 8;
static const int kWMAX = kWX * kWY * kWZ;      // 8192
static const int kWVW = kWMAX / 32;            // 256
static const int kSCI = 30, kSCJ = 30, kSCK = 6;
static const size_t kOffQ   = (size_t)kShmHdr;
static const size_t kOffVbw = kOffQ + (size_t)kShmQMax * 4;
static const size_t kOffTab = kOffVbw + (size_t)kWVW * 4;
static const size_t kShmMin = kOffTab + (size_t)kWMAX * 6 * 4;

static inline void plshm_wi(unsigned char *p, size_t off, int v)   { std::memcpy(p + off, &v, 4); }
static inline void plshm_wf(unsigned char *p, size_t off, float v) { std::memcpy(p + off, &v, 4); }
static inline int  plshm_ri(const unsigned char *p, size_t off) { int v;    std::memcpy(&v, p + off, 4); return v; }

void Map_Optimization::pl_s2m_solve()
{
    pl_H_valid_ = false;
    if (!PL_S2M_ON || pl_s2m_bad_) return;
    // 启动预热窗：s2m 服务是回放起来之后才连上的，早于它的调用一律跳过（不计失败），
    // 否则会被熔断成"永久回退 CPU"。
    static const auto t_node0 = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_node0).count() < 25.0) return;
    if (laserCloudSurfLastDS->size() < 1) return;

    if (!pl_s2m_ready_)
    {
        const char *path = "/dev/shm/s2m_shm";
        pl_shm_fd_ = ::open(path, O_RDWR);
        if (pl_shm_fd_ < 0)
        {
            pl_s2m_bad_ = true;
            RCLCPP_WARN(this->get_logger(),
                        "[PL-S2M] 打不开 %s（服务未起？）⇒ 回退 CPU", path);
            return;
        }
        struct stat st{};
        if (::fstat(pl_shm_fd_, &st) != 0 || (size_t)st.st_size < kShmMin)
        {
            pl_s2m_bad_ = true;
            RCLCPP_WARN(this->get_logger(), "[PL-S2M] shm 尺寸不对（%ld < %zu）⇒ 回退 CPU",
                        (long)st.st_size, kShmMin);
            return;
        }
        pl_shm_sz_ = (size_t)st.st_size;
        void *m = ::mmap(nullptr, pl_shm_sz_, PROT_READ | PROT_WRITE, MAP_SHARED, pl_shm_fd_, 0);
        if (m == MAP_FAILED)
        {
            pl_s2m_bad_ = true;
            RCLCPP_WARN(this->get_logger(), "[PL-S2M] mmap 失败 ⇒ 回退 CPU");
            return;
        }
        pl_shm_ = (unsigned char *)m;
        pl_s2m_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "[PL-S2M] 已连上 shm（%.1f KB），surf 走 PL",
                    (double)pl_shm_sz_ / 1024.0);
    }

    const auto t_start = std::chrono::steady_clock::now();
    const float LF = planeSurfMap.leaf;
    if (!(LF > 0.01f) || planeSurfMap.keys.empty()) return;
    const float invLF = 1.f / LF;

    // M 系数（与 ref_Hb / 内核同式）
    float s6[6] = {(float)std::sin(transformTobeMapped[1]), (float)std::cos(transformTobeMapped[1]),
                   (float)std::sin(transformTobeMapped[2]), (float)std::cos(transformTobeMapped[2]),
                   (float)std::sin(transformTobeMapped[0]), (float)std::cos(transformTobeMapped[0])};
    float M[27];
    pls2m::calc_M(s6, M);

    float parq[12];
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) parq[r * 3 + c] = transPointAssociateToMap.linear()(r, c);
    parq[9]  = transPointAssociateToMap.translation().x();
    parq[10] = transPointAssociateToMap.translation().y();
    parq[11] = transPointAssociateToMap.translation().z();
    std::memcpy(pl_shm_ + 176, parq, sizeof(parq));
    std::memcpy(pl_shm_ + 224, M, sizeof(M));
    plshm_wf(pl_shm_, 8,  LF);
    plshm_wf(pl_shm_, 12, invLF);
    plshm_wf(pl_shm_, 16, A8_RMAX);

    // 每个点的全局格号 + 分桶
    const int n = (int)laserCloudSurfLastDS->size();
    std::vector<int> gi(n), gj(n), gk(n);
    std::map<long, std::vector<int>> buckets;
    for (int i = 0; i < n; ++i)
    {
        const auto &pt = laserCloudSurfLastDS->points[i];
        const Eigen::Vector3f pp = transPointAssociateToMap *
                                   Eigen::Vector3f(pt.x, pt.y, pt.z);
        gi[i] = (int)std::floor(pp.x() * invLF);
        gj[i] = (int)std::floor(pp.y() * invLF);
        gk[i] = (int)std::floor(pp.z() * invLF);
        const long key = (long)(gi[i] / kSCI) * 1000003L
                       + (long)(gj[i] / kSCJ) * 1009L + (long)(gk[i] / kSCK);
        buckets[key].push_back(i);
    }

    float Hacc[21] = {0};
    float bacc[6]  = {0};
    int nblk = 0;
    // ★ 整个 solve 共用一个预算，绝不能每块各算一份：10 块 × 100 ms = 1 s/迭代
    //   ⇒ 每帧数秒 ⇒ DDS 积压 ⇒ 板子 OOM（2026-10-29 踩过）。
    const auto t_budget = std::chrono::steady_clock::now();
    const double budget_ms = 60.0;
    for (auto &kv : buckets)
    {
        const std::vector<int> &idxs = kv.second;
        if (idxs.empty()) continue;
        const int bi = gi[idxs[0]] / kSCI;
        const int bj = gj[idxs[0]] / kSCJ;
        const int bk = gk[idxs[0]] / kSCK;
        const int base_i = bi * kSCI - 1, base_j = bj * kSCJ - 1, base_k = bk * kSCK - 1;

        // 子窗：直接从 planeSurfMap 重打包（pls2m::PlWin 已按窗口裁）
        pls2m::PlWin w;
        w.reset(kWX, kWY, kWZ, LF, base_i, base_j, base_k);
        w.pack(planeSurfMap.keys, planeSurfMap.valid, planeSurfMap.pl.data(), 4);
        if (w.nvalid == 0) continue;             // 本块无有效格 ⇒ 全 miss，省一次往返

        uint32_t *vbw = (uint32_t *)(pl_shm_ + kOffVbw);
        std::memset(vbw, 0, (size_t)kWVW * 4);
        for (int t = 0; t < kWMAX; ++t)
            if ((w.vb[(size_t)t >> 3] >> (t & 7)) & 1u) vbw[t >> 5] |= (1u << (t & 31));
        std::memcpy(pl_shm_ + kOffTab, w.tab.data(), (size_t)kWMAX * 6 * 4);

        float *qbuf = (float *)(pl_shm_ + kOffQ);
        const int nb = (int)idxs.size();
        for (int k = 0; k < nb; ++k)
        {
            const auto &pt = laserCloudSurfLastDS->points[idxs[k]];
            qbuf[4 * k + 0] = pt.x;
            qbuf[4 * k + 1] = pt.y;
            qbuf[4 * k + 2] = pt.z;
            qbuf[4 * k + 3] = 1.f;
        }
        plshm_wi(pl_shm_, 4, nb);
        plshm_wf(pl_shm_, 128, (float)base_i * LF);
        plshm_wf(pl_shm_, 136, (float)base_j * LF);
        plshm_wf(pl_shm_, 144, (float)base_k * LF);

        int req = pl_shm_seq_ + 1;
        if ((req & 1) == 0) ++req;
        plshm_wi(pl_shm_, 0, req);

        while (plshm_ri(pl_shm_, 0) == req)
        {
            ::usleep(20);                        // 不忙等：板子 4 核，忙等会吃满一核
            const double el = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - t_budget).count();
            if (el > budget_ms)
            {
                ++pl_fail_;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "[PL-S2M] 预算 %.0f ms 用尽（已连续失败 %d 次），本次回退 CPU",
                                     budget_ms, pl_fail_);
                if (pl_fail_ >= 3)
                {
                    pl_s2m_bad_ = true;
                    RCLCPP_ERROR(this->get_logger(),
                                 "[PL-S2M] 连续 3 次失败 ⇒ 永久回退 CPU（本进程不再尝试）");
                }
                return;
            }
        }
        pl_fail_ = 0;
        pl_shm_seq_ = req;                       // 服务端已写 req+1

        const float *hb = (const float *)(pl_shm_ + 64);
        for (int t = 0; t < 21; ++t) Hacc[t] += hb[t];
        for (int t = 0; t < 6; ++t)  bacc[t]  += hb[21 + t];
        ++nblk;
    }

    if (nblk == 0) return;
    for (int t = 0; t < 21; ++t) pl_H_[t] = Hacc[t];
    for (int t = 0; t < 6; ++t)  pl_b_[t]  = bacc[t];
    pl_H_valid_ = true;

    pl_s2m_ms_ += std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - t_start).count();
    if ((++pl_s2m_it_ % 40) == 0)
    {
        const int nsurf = laserCloudOri
                              ? (int)laserCloudOri->size() - pl_ncorner_ : -1;
        RCLCPP_INFO(this->get_logger(),
                    "[PL-S2M] it=%ld nq=%d blocks=%d corner=%d surf=%d avg=%.2f ms/it",
                    pl_s2m_it_, n, nblk, pl_ncorner_, nsurf, pl_s2m_ms_ / 40.0);
        pl_s2m_ms_ = 0.0;
    }
}

void Map_Optimization::scan2MapOptimization()
{
    // 根据现有地图与最新点云数据进行配准从而更新机器人精确位姿与融合建图，
    // 它分为角点优化、平面点优化、配准与更新等部分。
    // 优化的过程与里程计的计算类似，是通过计算点到直线或平面的距离，构建优化公式再用LM法求解。
    if (cloudKeyPoses3D->points.empty())
        return;

    if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
    {
        static bool _a1_printed = false;
        if (!_a1_printed) {
            _a1_printed = true;
            RCLCPP_INFO(this->get_logger(), "[A1] scan2map iter_max=%d conv_r=%.3f conv_t=%.3f",
                        MP_ITER_MAX, MP_CONV_R, MP_CONV_T);
        }
        static bool _a2_printed = false;
        if (!_a2_printed && A2_USE_VOXEL) {
            _a2_printed = true;
            RCLCPP_INFO(this->get_logger(), "[A2] nn=voxel leaf=%.2f ring=%d check_n=%d",
                        (double)A2_LEAF, A2_RING, A2_CHECK_N);
        }
        // 构建kdtree（A2：改为构建体素栅格）
        auto mp_t = std::chrono::steady_clock::now();
        a8_nq = a8_nhit = a8_nbase_nn = 0;   // [A8] 帧内计数清零（两种模式都要，便于同口径对比）
        c9_nq = c9_nhit = c9_nbase_nn = 0;   // [A9] 同上（corner 分支）
        // [A9] 建体素直线表（corner）：同样只重算本帧变过的细格所属的线格
        if (A9_VLINE)
        {
            static bool _a9w = false;
            if (!A3_INC && !_a9w)
            {
                _a9w = true;
                RCLCPP_WARN(this->get_logger(),
                            "[A9] MAP_CORELINE=vmap 的直线表来自 A3 增量桶 ⇒ 必须同时 MAP_ESKF=inc");
            }
            auto t9 = std::chrono::steady_clock::now();
            incCorner.buildLinesInc(A9_LEAF, A9_MINPTS, A9_LAM, cornerLineMap);
            c9_build_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - t9).count();
            // [A9] 对拍：同一帧再跑一遍全量直线，逐格比 键集合/点数/valid/质心/方向
            static long _a9chk_n = 0;
            if (A9_CHK > 0 && _a9chk_n < (long)A9_CHK)
            {
                static LineVoxelMap refLine;
                const double s_agg = c9_t_agg, s_eig = c9_t_eig, s_out = c9_t_out;
                const size_t s_cell = c9_ncell, s_line = c9_nline;
                const float  s_rms = c9_line_rms;
                incCorner.buildLinesFull(A9_LEAF, A9_MINPTS, A9_LAM, refLine);
                c9_t_agg = s_agg; c9_t_eig = s_eig; c9_t_out = s_out;
                c9_ncell = s_cell; c9_nline = s_line; c9_line_rms = s_rms;   // 别让对拍污染统计
                long nmiss = 0, nextra = 0, nbad = 0;
                double maxdc = 0.0, maxang = 0.0;
                for (size_t b = 0; b < refLine.keys.size(); ++b)
                {
                    const int b2i = cornerLineMap.find(refLine.keys[b]);
                    if (b2i < 0) { ++nmiss; continue; }
                    if (refLine.valid[b] != cornerLineMap.valid[b2i] ||
                        refLine.cnt[b] != cornerLineMap.cnt[b2i]) { ++nbad; continue; }
                    if (!refLine.valid[b]) continue;
                    const double dx = (double)refLine.ln[6 * b + 0] - (double)cornerLineMap.ln[6 * b2i + 0];
                    const double dy = (double)refLine.ln[6 * b + 1] - (double)cornerLineMap.ln[6 * b2i + 1];
                    const double dz = (double)refLine.ln[6 * b + 2] - (double)cornerLineMap.ln[6 * b2i + 2];
                    const double dc = std::sqrt(dx * dx + dy * dy + dz * dz);
                    if (dc > maxdc) maxdc = dc;
                    const double dot = std::fabs(
                        (double)refLine.ln[6 * b + 3] * (double)cornerLineMap.ln[6 * b2i + 3] +
                        (double)refLine.ln[6 * b + 4] * (double)cornerLineMap.ln[6 * b2i + 4] +
                        (double)refLine.ln[6 * b + 5] * (double)cornerLineMap.ln[6 * b2i + 5]);
                    const double ang = std::acos(std::min(1.0, dot)) * 180.0 / M_PI;
                    if (ang > maxang) maxang = ang;
                }
                for (size_t b2i = 0; b2i < cornerLineMap.keys.size(); ++b2i)
                    if (cornerLineMap.cnt[b2i] > 0 &&
                        refLine.find(cornerLineMap.keys[b2i]) < 0) ++nextra;
                RCLCPP_INFO(this->get_logger(),
                            "[A9-CHK] n=%ld ref=%zu inc=%zu | miss=%ld extra=%ld bad=%ld "
                            "max|dc|=%.3e m max_ang=%.3e deg",
                            _a9chk_n, refLine.keys.size(), cornerLineMap.keys.size(),
                            nmiss, nextra, nbad, maxdc, maxang);
                ++_a9chk_n;
            }
        }
        // [A8] 建体素平面表：数据源是 A3 桶里的**原始点二阶矩**（不是 DS 云，DS 云太稀）。
        //      放在 S_KDT 计时区间内 —— 两者都是"本帧地图侧准备"，vmap 下不再有 NN 索引可建。
        if (A8_VMAP)
        {
            static bool _a8w = false;
            if (!A3_INC && !_a8w)
            {
                _a8w = true;
                RCLCPP_WARN(this->get_logger(),
                            "[A8] MAP_PLANE=vmap 的平面表来自 A3 增量桶 ⇒ 必须同时 MAP_ESKF=inc，"
                            "否则每格都无平面（当前未开 inc，查询会全部 miss）");
            }
            auto t8 = std::chrono::steady_clock::now();
            incSurf.buildPlanes(A8_LEAF, A8_MINPTS, A8_LAM, planeSurfMap);
            a8_build_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - t8).count();
            // [A8] 对拍：同一帧再跑一遍全量，逐格比"键集合 / 点数 / valid / Δd / 法向夹角"
            static long _a8chk_n = 0;
            if (A8_CHK > 0 && _a8chk_n < (long)A8_CHK)
            {
                static PlaneVoxelMap refPlane;
                const double sav_tagg = a8_t_agg, sav_teig = a8_t_eig, sav_tout = a8_t_out;
                incSurf.buildPlanesRef(A8_LEAF, A8_MINPTS, A8_LAM, refPlane);
                a8_t_agg = sav_tagg; a8_t_eig = sav_teig; a8_t_out = sav_tout;   // 别让对拍污染计时
                long nmiss = 0, nextra = 0, nbad = 0;
                double maxdd = 0.0, maxang = 0.0;
                for (size_t b = 0; b < refPlane.keys.size(); ++b)
                {
                    const int b2 = planeSurfMap.find(refPlane.keys[b]);
                    if (b2 < 0) { ++nmiss; continue; }
                    if (refPlane.valid[b] != planeSurfMap.valid[b2] ||
                        refPlane.cnt[b] != planeSurfMap.cnt[b2]) { ++nbad; continue; }
                    if (!refPlane.valid[b]) continue;
                    const double dd = std::fabs((double)refPlane.pl[4 * b + 3] -
                                                (double)planeSurfMap.pl[4 * b2 + 3]);
                    if (dd > maxdd) maxdd = dd;
                    const double dot = std::fabs(
                        (double)refPlane.pl[4 * b] * (double)planeSurfMap.pl[4 * b2] +
                        (double)refPlane.pl[4 * b + 1] * (double)planeSurfMap.pl[4 * b2 + 1] +
                        (double)refPlane.pl[4 * b + 2] * (double)planeSurfMap.pl[4 * b2 + 2]);
                    const double ang = std::acos(std::min(1.0, dot)) * 180.0 / M_PI;
                    if (ang > maxang) maxang = ang;
                }
                for (size_t b2 = 0; b2 < planeSurfMap.keys.size(); ++b2)
                    if (planeSurfMap.cnt[b2] > 0 &&                 // cnt=0 是"已消失但键还留着"的陈旧格
                        refPlane.find(planeSurfMap.keys[b2]) < 0) ++nextra;
                RCLCPP_INFO(this->get_logger(),
                            "[A8-CHK] n=%ld ref=%zu inc=%zu | miss=%ld extra=%ld bad=%ld "
                            "max|dd|=%.3e m max_ang=%.3e deg",
                            _a8chk_n, refPlane.keys.size(), planeSurfMap.keys.size(),
                            nmiss, nextra, nbad, maxdd, maxang);
                ++_a8chk_n;
            }
        }
        const bool a2_check = (A2_USE_VOXEL && !A8_VMAP && !A9_VLINE && A2_CHECK_N > 0 && a2_frame < A2_CHECK_N);
        if (A2_USE_VOXEL)
        {
            // [A9] corner 走体素直线且不回退时，这份 NN 索引也是白建
            if (!A9_VLINE || A9_FB) nnCornerMap.build(*laserCloudCornerFromMapDS, A2_LEAF);
            // vmap 下 surf 不再查 NN ⇒ 这一份索引白建，直接跳过（省一次 build）；
            // 但开了 FB（混合回退）时仍需要它。
            if (!A8_VMAP || A8_FB) nnSurfMap.build(*laserCloudSurfFromMapDS, A2_LEAF);
            if (a2_check) // 对拍需要 kdtree 同时在场（只在对拍帧付这份钱）
            {
                kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
                kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
            }
        }
        else
        {
            if (!A9_VLINE || A9_FB) kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            if (!A8_VMAP || A8_FB) kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
        }
        mp_tick(S_KDT, mp_t);
        if (a2_check) a2ParityCheck();        if (A4_DUMP > 0 && a4_frame == (long)A4_DUMP - 1)
        {
            RCLCPP_INFO(this->get_logger(), "[A4] dump 第 %ld 帧", a4_frame);
            dumpS2M("/dev/shm/s2m_dump.bin");
        }
        ++a4_frame;
        // 迭代（上限 30，可用 MAP_ITER_MAX 覆盖，见文件头 A1 说明）
        static long pl_ref_frame = 0;   // [PL v2a] 参考对拍用的帧计数
        int iterCount;
        for (iterCount = 0; iterCount < MP_ITER_MAX; iterCount++)
        {
            laserCloudOri->clear();
            coeffSel->clear();
            // 边缘点匹配优化
            cornerOptimization();
            mp_tick(S_COR, mp_t);
            // 平面点匹配优化
            surfOptimization();
            mp_tick(S_SURF, mp_t);
            // [PL v2a] 参考对拍（MAP_PL_REF=n 打开前 n 帧）
            if (PL_REF_N > 0 && pl_ref_frame < PL_REF_N && A8_VMAP)
            {
                const float _s6[6] = {(float)sin(transformTobeMapped[1]), (float)cos(transformTobeMapped[1]),
                                      (float)sin(transformTobeMapped[2]), (float)cos(transformTobeMapped[2]),
                                      (float)sin(transformTobeMapped[0]), (float)cos(transformTobeMapped[0])};
                pl_ref_check_surf(*laserCloudSurfLastDS, planeSurfMap, A8_LEAF, A8_RMAX,
                                  _s6, transPointAssociateToMap,
                                  laserCloudOriSurfVec, coeffSelSurfVec, laserCloudOriSurfFlag,
                                  laserCloudSurfLastDSNum, pl_ref_frame);
                ++pl_ref_frame;
            }
            // 组合优化多项式系数
            combineOptimizationCoeffs();
            mp_tick(S_COMB, mp_t);
            pl_s2m_solve();          // [PL-S2M] surf 的 H/b 交给 PL（MAP_PL_S2M=1）

            bool lmDone = LMOptimization(iterCount);
            mp_tick(S_LM, mp_t);
            if (MP_ON) ++mp_it;
            if (lmDone == true)
                break;
        }

        transformUpdate();
        if (A8_DBG)
        {
            static bool _a8_head = false;
            if (!_a8_head)
            {
                _a8_head = true;
                RCLCPP_INFO(this->get_logger(),
                            "[A8] plane=%s req_leaf=%.2f eff_leaf=%.2f minpts=%d lam=%.2f rmax=%.2f nb=%d fb=%d pick=%d inc=%d chk=%d",
                            A8_VMAP ? "vmap" : (A3_INC ? "base(5nn)" : "base(5nn,kd)"),
                            (double)A8_LEAF, (double)a8_eff_leaf, A8_MINPTS, (double)A8_LAM,
                            (double)A8_RMAX, A8_NB, A8_FB, A8_PICK, A8_INC, A8_CHK);
            }
            // hit = 真正进 H/b 的平面观测量（两条路径同一口径）；nn5 = 基线里 5-NN 门限通过数
            if (A8_VMAP)
                RCLCPP_INFO(this->get_logger(),
                            "[A8] n=%ld cells=%zu planes=%zu(%.0f%%) rms=%.3fm build=%.2fms "
                            "inc=%d dirty=%ld/%ld[agg=%.2f eig=%.2f out=%.2f] | q=%ld hit=%ld "
                            "hit=%.1f%% it=%d",
                            a8_nframe, a8_ncell, a8_nplane,
                            a8_ncell ? 100.0 * (double)a8_nplane / (double)a8_ncell : 0.0,
                            (double)a8_plane_rms, a8_build_ms,
                            A8_INC, a8_npdirty, a8_npall,
                            a8_t_agg, a8_t_eig, a8_t_out,
                            a8_nq, a8_nhit,
                            a8_nq ? 100.0 * (double)a8_nhit / (double)a8_nq : 0.0, iterCount);
            else
                RCLCPP_INFO(this->get_logger(),
                            "[A8] n=%ld (base) | q=%ld hit=%ld hit=%.1f%% nn5=%ld nn5=%.1f%% it=%d",
                            a8_nframe, a8_nq, a8_nhit,
                            a8_nq ? 100.0 * (double)a8_nhit / (double)a8_nq : 0.0,
                            a8_nbase_nn,
                            a8_nq ? 100.0 * (double)a8_nbase_nn / (double)a8_nq : 0.0, iterCount);
            ++a8_nframe;
        }
        // [A9] corner 分支的统计（与 A8 同口径：hit = 真正进 H/b 的边观测量）
        if (A9_DBG)
        {
            static bool _a9_head = false;
            if (!_a9_head)
            {
                _a9_head = true;
                RCLCPP_INFO(this->get_logger(),
                            "[A9] corner=line req_leaf=%.2f eff_leaf=%.2f minpts=%d lam=%.3f rmax=%.2f "
                            "nb=%d pick=%d fb=%d inc=%d",
                            (double)A9_LEAF, (double)c9_eff_leaf, A9_MINPTS, (double)A9_LAM,
                            (double)A9_RMAX, A9_NB, A9_PICK, A9_FB, A9_INC);
            }
            if (A9_VLINE)
                RCLCPP_INFO(this->get_logger(),
                            "[A9] n=%ld cells=%zu lines=%zu(%.0f%%) rms=%.3fm build=%.2fms "
                            "dirty=%ld/%ld[agg=%.2f eig=%.2f out=%.2f] | q=%ld hit=%ld hit=%.1f%% it=%d",
                            c9_nframe, c9_ncell, c9_nline,
                            c9_ncell ? 100.0 * (double)c9_nline / (double)c9_ncell : 0.0,
                            (double)c9_line_rms, c9_build_ms, c9_ndirty, c9_nall,
                            c9_t_agg, c9_t_eig, c9_t_out,
                            c9_nq, c9_nhit,
                            c9_nq ? 100.0 * (double)c9_nhit / (double)c9_nq : 0.0, iterCount);
            else
                RCLCPP_INFO(this->get_logger(),
                            "[A9] n=%ld (base-5nn) | q=%ld hit=%ld hit=%.1f%% nn5=%ld nn5=%.1f%% it=%d",
                            c9_nframe, c9_nq, c9_nhit,
                            c9_nq ? 100.0 * (double)c9_nhit / (double)c9_nq : 0.0,
                            c9_nbase_nn,
                            c9_nq ? 100.0 * (double)c9_nbase_nn / (double)c9_nq : 0.0, iterCount);
            ++c9_nframe;
        }
        // visualize_factor(surf_factor, timeLaserInfoStamp, 250, pub_select_surf);
        // visualize_factor(corner_factor, timeLaserInfoStamp, 100, pub_select_corner);
        // std::cout << "iter: " << iterCount << std::endl;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
    }
}

// 将当前位姿transformTobeMapped存储为仿射矩阵
void Map_Optimization::updatePointAssociateToMap()
{
    transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
}

// 边缘点匹配优化
void Map_Optimization::cornerOptimization()
{
    corner_factor.clear();
    // 实现transformTobeMapped的仿射矩阵形式转换 下面调用的函数就一行就不展开了  工具类函数
    //  把结果存入transPointAssociateToMap中
    updatePointAssociateToMap();

    // [A1] 显式并行区：Mat / vector 抬成线程私有，循环内不再每点 malloc
    const int nCornerPts = laserCloudCornerLastDSNum;
    static bool _cor_printed = false;
    if (!_cor_printed)
    {
        _cor_printed = true;
        RCLCPP_INFO(this->get_logger(), "[A1] corner 特征分解 mode=%d（0=base/fresh Mat, 1=reuse, 2=saes）", COR_EIG);
    }
    // [A9] 三个计数：线程私有 + reduction 归并
    long lq = 0, lnn = 0, lhit = 0;
#pragma omp parallel num_threads(numberOfCores) reduction(+:lq,lnn,lhit)
    {
    std::vector<int>   pointSearchInd;
    std::vector<float> pointSearchSqDis;
    cv::Mat thA(3, 3, CV_32F), thD(1, 3, CV_32F), thV(3, 3, CV_32F);
    Eigen::Matrix3f cov3;

#pragma omp for schedule(static)
    for (int i = 0; i < nCornerPts; i++)
    {
        PointType pointOri, pointSel, coeff;

        pointOri = laserCloudCornerLastDS->points[i];
        // 第i帧的点转换到第一帧坐标系下
        // 这里就调用了第一步中updatePointAssociateToMap中实现的transPointAssociateToMap，
        // 然后利用这个变量，把pointOri的点转换到pointSel下，pointSel作为输出
        pointAssociateToMap(&pointOri, &pointSel);
        ++lq;
        // [A9] 体素直线：不查 5-NN、不现拟合，只读本格预存的（质心 c, 方向 t），
        //      几何与原码"点到过 c、方向 t 的直线距离"完全同式（la/lb/lc = 垂线单位方向）。
        if (A9_VLINE)
        {
            LineVoxelMap::Hit h9;
            if (cornerLineMap.query(pointSel, A9_NB, A9_RMAX, h9))
            {
                const float s9 = 1 - 0.9f * fabsf(h9.ld2);
                if (s9 > 0.1f)
                {
                    ++lhit;
                    coeff.x = s9 * h9.la;
                    coeff.y = s9 * h9.lb;
                    coeff.z = s9 * h9.lc;
                    coeff.intensity = s9 * h9.ld2;
                    laserCloudOriCornerVec[i] = pointOri;
                    coeffSelCornerVec[i] = coeff;
                    laserCloudOriCornerFlag[i] = true;
                }
                continue;
            }
            if (!A9_FB) continue;      // 无回退：这条边直接丢
        }
        // 最近邻 5 点：默认走 kd 树（原版）；MAP_NN=voxel 时走体素栅格 + 固定邻域。
        // 两者在 d5 < 1.0 时点集完全等价（见文件头 [A2] 保证），故下游一行都不用改。
        int          a2i[5];
        float        a2d[5];
        int          a2n = 5;
        const int   *pSearchInd;
        const float *pSearchD2;
        if (A2_USE_VOXEL)
        {
            a2n = nnCornerMap.knn5(pointSel, a2i, a2d);
            pSearchInd = a2i; pSearchD2 = a2d;
        }
        else
        {
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
            pSearchInd = pointSearchInd.data(); pSearchD2 = pointSearchSqDis.data();
        }

        if (a2n >= 5 && pSearchD2[4] < 1.0f)
        {
            ++lnn;   // [A9] 基线：5-NN 门限通过
            float cx = 0, cy = 0, cz = 0;
            // 先求5个样本的平均值
            for (int j = 0; j < 5; j++)
            {
                cx += laserCloudCornerFromMapDS->points[pSearchInd[j]].x;
                cy += laserCloudCornerFromMapDS->points[pSearchInd[j]].y;
                cz += laserCloudCornerFromMapDS->points[pSearchInd[j]].z;
            }
            cx /= 5;
            cy /= 5;
            cz /= 5;

            // 下面求矩阵matA1=[ax,ay,az]^t*[ax,ay,az]
            // 更准确地说应该是在求协方差matA1
            float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
            for (int j = 0; j < 5; j++)
            {
                float ax = laserCloudCornerFromMapDS->points[pSearchInd[j]].x - cx;
                float ay = laserCloudCornerFromMapDS->points[pSearchInd[j]].y - cy;
                float az = laserCloudCornerFromMapDS->points[pSearchInd[j]].z - cz;

                a11 += ax * ax;
                a12 += ax * ay;
                a13 += ax * az;
                a22 += ay * ay;
                a23 += ay * az;
                a33 += az * az;
            }
            a11 /= 5;
            a12 /= 5;
            a13 /= 5;
            a22 /= 5;
            a23 /= 5;
            a33 /= 5;

            // 求正交阵的特征值和特征向量（特征值降序，v 为最大特征值对应的特征向量）
            //   [A1] 三种实现产出同一组 (d0,d1,vx,vy,vz)，下游一行不用改：
            //     cv::eigen 给的是"行 0 = 最大特征值的特征向量"、特征值降序；
            //     SelfAdjointEigenSolver 给的是升序、且特征向量按列 ⇒ 取 (2) 列 / (2)(1) 元素。
            float d0 = 0.f, d1 = 0.f, vx = 0.f, vy = 0.f, vz = 0.f;
            if (COR_EIG == 1)
            {
                thA.at<float>(0, 0) = a11; thA.at<float>(0, 1) = a12; thA.at<float>(0, 2) = a13;
                thA.at<float>(1, 0) = a12; thA.at<float>(1, 1) = a22; thA.at<float>(1, 2) = a23;
                thA.at<float>(2, 0) = a13; thA.at<float>(2, 1) = a23; thA.at<float>(2, 2) = a33;
                cv::eigen(thA, thD, thV);
                d0 = thD.at<float>(0, 0); d1 = thD.at<float>(0, 1);
                vx = thV.at<float>(0, 0); vy = thV.at<float>(0, 1); vz = thV.at<float>(0, 2);
            }
            else if (COR_EIG == 2)
            {
                cov3 << a11, a12, a13,
                        a12, a22, a23,
                        a13, a23, a33;
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov3);
                d0 = es.eigenvalues()(2);  d1 = es.eigenvalues()(1);
                vx = es.eigenvectors()(0, 2); vy = es.eigenvectors()(1, 2); vz = es.eigenvectors()(2, 2);
            }
            else
            {
                cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
                cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
                cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
                matA1.at<float>(0, 0) = a11;
                matA1.at<float>(0, 1) = a12;
                matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12;
                matA1.at<float>(1, 1) = a22;
                matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13;
                matA1.at<float>(2, 1) = a23;
                matA1.at<float>(2, 2) = a33;
                cv::eigen(matA1, matD1, matV1);
                d0 = matD1.at<float>(0, 0); d1 = matD1.at<float>(0, 1);
                vx = matV1.at<float>(0, 0); vy = matV1.at<float>(0, 1); vz = matV1.at<float>(0, 2);
            }
            // 边缘：与较大特征值相对应的特征向量代表边缘线的方向（一大两小，大方向）
            // 以下这一大块是在计算点到边缘的距离，最后通过系数s来判断是否距离很近
            // 如果距离很近就认为这个点在边缘上，需要放到laserCloudOri中
            // 如果最大的特征值相比次大特征值，大很多，认为构成了线，角点是合格的
            if (d0 > 3 * d1)
            {
                // 当前帧角点坐标（map系下）
                float x0 = pointSel.x;
                float y0 = pointSel.y;
                float z0 = pointSel.z;
                float x1 = cx + 0.1 * vx;
                float y1 = cy + 0.1 * vy;
                float z1 = cz + 0.1 * vz;
                float x2 = cx - 0.1 * vx;
                float y2 = cy - 0.1 * vy;
                float z2 = cz - 0.1 * vz;

                // 局部map对应中心角点，沿着特征向量（直线方向）方向，前后各取一个点
                float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));
                // line_12，底边边长
                float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
                // 两次叉积，得到点到直线的垂线段单位向量，x分量，下面同理
                // 求叉乘结果[la',lb',lc']=[(x1-x2),(y1-y2),(z1-z2)]x[XXX,YYY,ZZZ]
                // [la,lb,lc]=[la',lb',lc']/a012/l12
                // 得到底边上的高的方向向量[la,lb,lc]
                // LLL=[la,lb,lc]是V1[0]这条高上的单位法向量。||LLL||=1；

                // 如不理解则看图：
                //        A
                //   B        C
                // 这里ABxAC，代表垂直于ABC面的法向量，其模长为平行四边形面积
                // 因此BCx(ABxAC),代表了BC和（ABC平面的法向量）的叉乘，那么其实这个向量就是A到BC的垂线的方向向量
                // 那么(ABxAC)/|ABxAC|,代表着ABC平面的单位法向量
                // BCxABC平面单位法向量，即为一个长度为|BC|的（A到BC垂线的方向向量），因此再除以|BC|，得到A到BC垂线的单位方向向量

                float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;

                float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

                float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

                // 三角形的高，也就是点到直线距离
                // 计算点pointSel到直线的距离
                // 这里需要特别说明的是ld2代表的是点pointSel到过点[cx,cy,cz]的方向向量直线的距离
                float ld2 = a012 / l12;
                // 下面涉及到一个鲁棒核函数，作者简单地设计了这个核函数。
                // 距离越大，s越小，是个距离惩罚因子（权重
                float s = 1 - 0.9 * fabs(ld2);
                // coeff代表系数的意思
                // coff用于保存距离的方向向量
                coeff.x = s * la;
                coeff.y = s * lb;
                coeff.z = s * lc;
                // intensity本质上构成了一个核函数，ld2越接近于1，增长越慢
                // intensity=(1-0.9*ld2)*ld2=ld2-0.9*ld2*ld2
                coeff.intensity = s * ld2;
                // 程序末尾根据s的值来判断是否将点云点放入点云集合laserCloudOri以及coeffSel中。
                // 所以就应该认为这个点是边缘点
                // s>0.1 也就是要求点到直线的距离ld2要小于1m
                // s越大说明ld2越小(离边缘线越近)，这样就说明点pointOri在直线上
                if (s > 0.1)
                {
                    ++lhit;   // [A9] 真正进 H/b 的边观测量
                    laserCloudOriCornerVec[i] = pointOri;
                    coeffSelCornerVec[i] = coeff;
                    laserCloudOriCornerFlag[i] = true;

                    // FactorParam factor;
                    // factor.col(0) << cx, cy, cz;
                    // factor.col(1) << vx, vy, vz;
                    // factor.col(2) << pointOri.x, pointOri.y, pointOri.z;
                    // factor.col(3)[0] = s * ld2;

                    // corner_factor.push_back(factor);
                }
            }
        }
    }
    }   // #pragma omp parallel —— [A1] 线程私有缓冲随该区域结束
    c9_nq += lq;
    c9_nhit += lhit;
    c9_nbase_nn += lnn;
}

// 平面点优化
void Map_Optimization::surfOptimization()
{
    updatePointAssociateToMap();

    // [A1] 同 corner：vector 抬成线程私有（kd 模式下 nearestKSearch 每点会 resize 两次）
    const int nSurfPts = laserCloudSurfLastDSNum;
    // [A8] 三个计数用 reduction 归并（线程私有 + 循环末合并），不用 atomic
    long lq = 0, lnn = 0, lhit = 0;
#pragma omp parallel num_threads(numberOfCores) reduction(+:lq,lnn,lhit)
    {
    std::vector<int>   pointSearchInd;
    std::vector<float> pointSearchSqDis;

#pragma omp for schedule(static)
    for (int i = 0; i < nSurfPts; i++)
    {
        PointType pointOri, pointSel, coeff;

        pointOri = laserCloudSurfLastDS->points[i];
        pointAssociateToMap(&pointOri, &pointSel);
        ++lq;
        // [A8] 体素平面：不查 5-NN、不现拟合，只读本格预存平面 —— 这一支就是将来 PL 里
        //      的那个数据通路（r = n·p + d，30 flop），其余（权重/coeff/入队）与原码同式。
        if (A8_VMAP)
        {
            float r8 = 0.f;
            const int b8 = planeSurfMap.query(pointSel, A8_NB, A8_RMAX, &r8);
            if (b8 >= 0)
            {
                const float pa = planeSurfMap.pl[4 * b8 + 0];
                const float pb = planeSurfMap.pl[4 * b8 + 1];
                const float pc = planeSurfMap.pl[4 * b8 + 2];
                const float pd2 = r8;   // 平面已归一化 ⇒ 这就是带符号的点到面距离
                const float s = 1 - 0.9f * fabsf(pd2) /
                                    sqrtf(sqrtf(pointSel.x * pointSel.x + pointSel.y * pointSel.y +
                                                pointSel.z * pointSel.z));
                if (s > 0.1f)
                {
                    ++lhit;
                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;
                    laserCloudOriSurfVec[i] = pointOri;
                    coeffSelSurfVec[i] = coeff;
                    laserCloudOriSurfFlag[i] = true;
                }
                continue;
            }
            // 本格无有效平面：MAP_PLANE_FB=1 时落到下面的 5-NN 分支（混合模式）
            if (!A8_FB) continue;
        }
        // 最近邻 5 点：默认 kd 树；MAP_NN=voxel 时走体素栅格（语义等价，见 [A2]）
        int          a2i[5];
        float        a2d[5];
        int          a2n = 5;
        const int   *pSearchInd;
        const float *pSearchD2;
        if (A2_USE_VOXEL)
        {
            a2n = nnSurfMap.knn5(pointSel, a2i, a2d);
            pSearchInd = a2i; pSearchD2 = a2d;
        }
        else
        {
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
            pSearchInd = pointSearchInd.data(); pSearchD2 = pointSearchSqDis.data();
        }

        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0;
        Eigen::Vector3f matX0;

        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();

        if (a2n >= 5 && pSearchD2[4] < 1.0f)
        {
            ++lnn;   // [A8] 基线路径的"5-NN 门限通过"计数（与 vmap 命中率同一口径对比用）
            float pa, pb, pc;
            float pd = 1;
            if (A5_CRAMER_D)
            {
                // 同一个最小二乘问题（min Σ(a·x+b·y+c·z+1)²）用 double 解 3×3 法方程
                double sxx = 0, sxy = 0, sxz = 0, syy = 0, syz = 0, szz = 0, rx = 0, ry = 0, rz = 0;
                for (int j = 0; j < 5; ++j)
                {
                    const double X = laserCloudSurfFromMapDS->points[pSearchInd[j]].x;
                    const double Y = laserCloudSurfFromMapDS->points[pSearchInd[j]].y;
                    const double Z = laserCloudSurfFromMapDS->points[pSearchInd[j]].z;
                    sxx += X * X; sxy += X * Y; sxz += X * Z;
                    syy += Y * Y; syz += Y * Z; szz += Z * Z;
                    rx -= X; ry -= Y; rz -= Z;
                }
                const double k00 = syy * szz - syz * syz;
                const double k01 = sxz * syz - sxy * szz;
                const double k02 = sxy * syz - sxz * syy;
                const double det = sxx * k00 + sxy * k01 + sxz * k02;
                const double k11 = sxx * szz - sxz * sxz;
                const double k12 = sxy * sxz - sxx * syz;
                const double k22 = sxx * syy - sxy * sxy;
                const double idet = (std::fabs(det) > 1e-30) ? (1.0 / det) : 0.0;
                pa = (float)((k00 * rx + k01 * ry + k02 * rz) * idet);
                pb = (float)((k01 * rx + k11 * ry + k12 * rz) * idet);
                pc = (float)((k02 * rx + k12 * ry + k22 * rz) * idet);
            }
            else
            {
                for (int j = 0; j < 5; j++)
                {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pSearchInd[j]].z;
                }
                matX0 = matA0.colPivHouseholderQr().solve(matB0);
                pa = matX0(0, 0);
                pb = matX0(1, 0);
                pc = matX0(2, 0);
            }

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;

            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
                if (fabs(pa * laserCloudSurfFromMapDS->points[pSearchInd[j]].x +
                         pb * laserCloudSurfFromMapDS->points[pSearchInd[j]].y +
                         pc * laserCloudSurfFromMapDS->points[pSearchInd[j]].z + pd) > 0.2)
                {
                    planeValid = false;
                    break;
                }
            }

            if (planeValid)
            {
                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                coeff.x = s * pa;
                coeff.y = s * pb;
                coeff.z = s * pc;
                coeff.intensity = s * pd2;

                if (s > 0.1)
                {
                    ++lhit;   // [A8] 真正进 H/b 的观测量（两条路径同一口径）
                    laserCloudOriSurfVec[i] = pointOri;
                    coeffSelSurfVec[i] = coeff;
                    laserCloudOriSurfFlag[i] = true;
                }
            }
        }
    }
    }   // #pragma omp parallel —— [A1]
    // [A8] 归并本帧（跨迭代累加；每帧开头在 scan2MapOptimization 里清零）
    a8_nq += lq;
    a8_nhit += lhit;
    a8_nbase_nn += lnn;
}

// 组合优化多项式系数
void Map_Optimization::combineOptimizationCoeffs()
{
    int num_edge = 0, num_surf = 0;
    // combine corner coeffs
    // 遍历当前帧角点集合，提取出与局部map匹配上了的角点
    for (int i = 0; i < laserCloudCornerLastDSNum; ++i)
    {
        if (laserCloudOriCornerFlag[i] == true)
        {
            laserCloudOri->push_back(laserCloudOriCornerVec[i]);
            coeffSel->push_back(coeffSelCornerVec[i]);
            num_edge++;
        }
    }
    // combine surf coeffs
    // 遍历当前帧平面点集合，提取出与局部map匹配上了的平面点
    for (int i = 0; i < laserCloudSurfLastDSNum; ++i)
    {
        if (laserCloudOriSurfFlag[i] == true)
        {
            laserCloudOri->push_back(laserCloudOriSurfVec[i]);
            coeffSel->push_back(coeffSelSurfVec[i]);
            num_surf++;
        }
    }
    pl_ncorner_ = num_edge;      // [PL-S2M] surf 从这一行开始
    // std::cout << "select " << num_edge << " corner and " << num_surf << " surf points" << std::endl;
    // reset flag for next iteration
    // 清空标记
    std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
}

inline Eigen::Matrix<float, 6, 6> Map_Optimization::transMatrix(Eigen::Matrix<float, 6, 6> &M)
{
    Eigen::Matrix<float, 6, 6> out;
    out.setZero();
    for (int i = 5; i >= 0; i--)
        for (int j = 0; j <= 5; j++)
            out(5 - i, j) = M(j, i);
    return out;
}

// #define USE_CV_MAT_ 1

// scan-to-map优化
bool Map_Optimization::LMOptimization(int iterCount)
{
    // 对匹配特征点计算Jacobian矩阵，观测值为特征点到直线、平面的距离，构建高斯牛顿方程，迭代优化当前位姿，存transformTobeMapped

    // 由于LOAM里雷达的特殊坐标系 所以这里也转了一次
    // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
    // lidar <- camera      ---     camera <- lidar
    // x = z                ---     x = y
    // y = x                ---     y = z
    // z = y                ---     z = x
    // roll = yaw           ---     roll = pitch
    // pitch = roll         ---     pitch = yaw
    // yaw = pitch          ---     yaw = roll

    // lidar -> camera
    float srx = sin(transformTobeMapped[1]);
    float crx = cos(transformTobeMapped[1]);
    float sry = sin(transformTobeMapped[2]);
    float cry = cos(transformTobeMapped[2]);
    float srz = sin(transformTobeMapped[0]);
    float crz = cos(transformTobeMapped[0]);

    // 当前帧匹配特征点数太少
    int laserCloudSelNum = laserCloudOri->size();
    if (laserCloudSelNum < 50)
    {
        return false;
    }

    Eigen::Matrix<float, Eigen::Dynamic, 6> mat_A(laserCloudSelNum, 6);
    Eigen::Matrix<float, 6, Eigen::Dynamic> mat_A_transpose(6, laserCloudSelNum);
    Eigen::Matrix<float, 6, 6> mat_A_transpose_A;
    Eigen::VectorXf mat_B(laserCloudSelNum);
    Eigen::Matrix<float, 6, 1> mat_A_transpose_B;
    Eigen::Matrix<float, 6, 1> mat_X;
    Eigen::Matrix<float, 6, 6> mat_P_;
    mat_P_.setZero();

    PointType pointOri, coeff;
    // 遍历匹配特征点，构建Jacobian矩阵
    for (int i = 0; i < laserCloudSelNum; i++)
    {
        // lidar -> camera
        pointOri.x = laserCloudOri->points[i].y;
        pointOri.y = laserCloudOri->points[i].z;
        pointOri.z = laserCloudOri->points[i].x;
        // lidar -> camera
        coeff.x = coeffSel->points[i].y;
        coeff.y = coeffSel->points[i].z;
        coeff.z = coeffSel->points[i].x;
        coeff.intensity = coeffSel->points[i].intensity;
        // in camera
        // https://wykxwyc.github.io/2019/08/01/The-Math-Formula-in-LeGO-LOAM/
        // 求雅克比矩阵中的元素，距离d对roll角度的偏导量即d(d)/d(roll)
        // 各种cos sin的是旋转矩阵对roll求导，pointOri.x是点的坐标，coeff.x等是距离到局部点的偏导，也就是法向量（建议看链接）
        // 注意：链接当中的R0-5公式中，ex和ey是反的
        // 另一个链接https://blog.csdn.net/weixin_37835423/article/details/111587379#commentBox当中写的更好
        float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;
        // 同上，求解的是对pitch的偏导量
        float ary = ((cry * srx * srz - crz * sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x + ((-cry * crz - srx * sry * srz) * pointOri.x + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;

        float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y + ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;

        // lidar -> camera
        mat_A(i, 0) = arz;
        mat_A(i, 1) = arx;
        mat_A(i, 2) = ary;
        mat_A(i, 3) = coeff.z;
        mat_A(i, 4) = coeff.x;
        mat_A(i, 5) = coeff.y;
        mat_B(i, 0) = -coeff.intensity;
    }
    // [PL-S2M] surf 段（从 pl_ncorner_ 起）清零 ⇒ AᵀA 只剩 corner，再把 PL 的 surf 加上
    const bool _use_pl = PL_S2M_ON && pl_H_valid_ &&
                         pl_ncorner_ >= 0 && pl_ncorner_ < laserCloudSelNum;
    if (_use_pl) mat_A.bottomRows(laserCloudSelNum - pl_ncorner_).setZero();
    mat_A_transpose = mat_A.transpose();
    mat_A_transpose_A = mat_A_transpose * mat_A;
    mat_A_transpose_B = mat_A_transpose * mat_B;
    if (_use_pl)
    {
        for (int n = 0; n < 21; ++n)
        {
            mat_A_transpose_A(pls2m::HI27[n], pls2m::HJ27[n]) += pl_H_[n];
            if (pls2m::HJ27[n] != pls2m::HI27[n])
                mat_A_transpose_A(pls2m::HJ27[n], pls2m::HI27[n]) += pl_H_[n];
        }
        for (int n = 0; n < 6; ++n) mat_A_transpose_B(n) += pl_b_[n];
    }
    mat_X = mat_A_transpose_A.colPivHouseholderQr().solve(mat_A_transpose_B);

    if (iterCount == 0)
    {
        Eigen::Matrix<float, 1, 6> mat_E;
        Eigen::Matrix<float, 6, 6> mat_V;
        Eigen::Matrix<float, 6, 6> mat_V_temp;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>> esolver(mat_A_transpose_A);
        mat_E = esolver.eigenvalues().real(); //  eigenvalue from small to large here
        mat_V = esolver.eigenvectors().real();

        mvp0DumpH(timeLaserInfoCur, mat_A_transpose_A, mat_E);   // [MVP0-H-DUMP]

        // ====================================================================
        // [MVP0-DEG] 退化判定与响应（环境变量 MVP0_DEG_MODE 控制）
        //   off   : 保持作者原状（原循环是死代码 → isDegenerate 恒 false，无退化处理）
        //   fixed : 修复作者 bug（特征值升序，应从最小端遍历）+ 固定阈值 100
        //   cusum : MVP0 双向 CUSUM 检测器 + H̄ 慢时平均定方向（解重映射式投影）
        // ====================================================================
        mat_V_temp = mat_V;
        isDegenerate = false;

        const char *mode_env = std::getenv("MVP0_DEG_MODE");
        const std::string deg_mode = mode_env ? mode_env : "off";

        if (deg_mode == "fixed" || deg_mode == "cusum")
        {
            // 修复：mat_E 升序，应从 i=0（最小）向上检查，高于阈值即停
            const float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 0; i < 6; ++i)
            {
                if (mat_E(0, i) >= eignThre[i]) break;
                for (int j = 0; j < 6; ++j) mat_V_temp(j, i) = 0;
                isDegenerate = true;
            }
            if (isDegenerate)
                RCLCPP_WARN(rclcpp::get_logger("map_optimization"),
                            "[MVP0-DEG:fixed] degenerate frame (eig_min=%.1f)", mat_E(0, 0));
            mat_P_ = transMatrix(mat_V).inverse() * transMatrix(mat_V_temp);
        }

        if (deg_mode == "cusum")
        {
            // ---- MVP0 检测器：列序置换 [r,p,y,z,x,y] → [t,t,t,r,r,r] ----
            Eigen::Matrix<double, 6, 6> H6;
            const int perm[6] = {3, 4, 5, 0, 1, 2};   // 自逆置换
            for (int i = 0; i < 6; ++i)
                for (int j = 0; j < 6; ++j)
                    H6(i, j) = mat_A_transpose_A(perm[i], perm[j]);

            mvp0::Metrics m = mvp0::analyze(H6, Eigen::Vector3d::Zero(), laserCloudSelNum);
            mvp0::detector_step(m);
            mvp0::Detector &g = mvp0::detector();

            if (g.deg)
            {
                // 形式2（解重映射）：平移方向取自 H̄（抗结构扰动），旋转方向取瞬时量
                const double floor_bar = g.f_eff() / 3.0;
                Eigen::Matrix3d Ptt = Eigen::Matrix3d::Zero();
                int n_tt = 0;
                for (int i = 0; i < 3; ++i)
                    if (g.lam_bar(i) > floor_bar)
                    {
                        Ptt += g.V_bar.col(i) * g.V_bar.col(i).transpose();
                        ++n_tt;
                    }
                if (n_tt == 0) { Ptt = g.V_bar.col(0) * g.V_bar.col(0).transpose(); n_tt = 1; }

                Eigen::Matrix3d Hrr = H6.block<3, 3>(3, 3);
                Eigen::Vector3d lam_rr_desc; Eigen::Matrix3d Vrr;
                mvp0::eig3_desc(Hrr, lam_rr_desc, Vrr);
                const double floor_rr = mvp0::config().lambda_floor_scale * (Hrr.trace() / 3.0);
                Eigen::Matrix3d Prr = Eigen::Matrix3d::Zero();
                int n_rr = 0;
                for (int i = 0; i < 3; ++i)
                    if (lam_rr_desc(i) > floor_rr)
                    {
                        Prr += Vrr.col(i) * Vrr.col(i).transpose();
                        ++n_rr;
                    }
                if (n_rr == 0) Prr = Vrr.col(0) * Vrr.col(0).transpose();

                Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();
                P.block<3, 3>(0, 0) = Ptt;
                P.block<3, 3>(3, 3) = Prr;

                // 增量置换到 mvp0 序 → 投影 → 换回
                Eigen::Matrix<float, 6, 1> xm;
                for (int i = 0; i < 6; ++i) xm(i) = mat_X(perm[i]);
                Eigen::Matrix<double, 6, 1> xd =
                    P * xm.cast<double>();
                for (int i = 0; i < 6; ++i) mat_X(i) = xd(perm[i]);

                isDegenerate = true;
                RCLCPP_WARN(rclcpp::get_logger("map_optimization"),
                            "[MVP0-DEG:cusum] deg=%d m=%.3f n_proj_tt=%d",
                            (int)g.deg, g.m_smooth, g.n_proj_tt);
            }
        }
    }

    if (isDegenerate)
    {
        Eigen::Matrix<float, 6, 1> matX2(mat_X);
        matX2 = mat_X;
        mat_X = mat_P_ * matX2;
    }

    // std::cout << "mat_X(delta r,p,y,x,y,z):\n" << mat_X(0, 0) << ", " << mat_X(1, 0) << ", " << mat_X(2, 0) << ", " << mat_X(3, 0) << ", " << mat_X(4, 0)
    //         << ", "  << mat_X(5, 0)  << std::endl;

    transformTobeMapped[0] += mat_X(0, 0);
    transformTobeMapped[1] += mat_X(1, 0);
    transformTobeMapped[2] += mat_X(2, 0);
    transformTobeMapped[3] += mat_X(3, 0);
    transformTobeMapped[4] += mat_X(4, 0);
    transformTobeMapped[5] += mat_X(5, 0);

    float deltaR = sqrt(
        pow(pcl::rad2deg(mat_X(0, 0)), 2) +
        pow(pcl::rad2deg(mat_X(1, 0)), 2) +
        pow(pcl::rad2deg(mat_X(2, 0)), 2));
    float deltaT = sqrt(
        pow(mat_X(3, 0) * 100, 2) +
        pow(mat_X(4, 0) * 100, 2) +
        pow(mat_X(5, 0) * 100, 2));

    if (deltaR < MP_CONV_R && deltaT < MP_CONV_T)
    {
        return true; // converged
    }
    return false; // keep optimizing
}

void Map_Optimization::transformUpdate()
{
    // 当前帧位姿
    incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
}

// 设置当前帧为关键帧并执行因子图优化
void Map_Optimization::saveKeyFramesAndFactor()
{
    // 1、计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
    // 2、添加激光里程计因子、GPS因子、闭环因子
    // 3、执行因子图优化
    // 4、得到当前帧优化后位姿，位姿协方差
    // 5、添加cloudKeyPoses3D，cloudKeyPoses6D，更新transformTobeMapped，添加当前关键帧的角点、平面点集合

    // 计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
    auto mp_t = std::chrono::steady_clock::now();
    if (saveFrame() == false)
    {
        mp_tick(S_KADD, mp_t);
        return;
    }
    if (MP_ON) ++mp_kf;

    // 添加激光里程计因子
    addOdomFactor();
    // [VIS] 视觉相对位姿因子（VIS_FACTOR=1 时生效）
    addVisualFactor();

    // 添加回环因子
    addLoopFactor();
    mp_tick(S_KADD, mp_t);

    // cout << "****************************************************" << endl;
    // gtSAMgraph.print("GTSAM Graph:\n");

    // std::cout << "init error: " << gtSAMgraph.error(initialEstimate) << std::endl;

    // update iSAM
    // 执行优化
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();

    // Values result = LevenbergMarquardtOptimizer(gtSAMgraph, initialEstimate).optimize();
    // std::cout << "final error: " << gtSAMgraph.error(result) << std::endl;

    if (aLoopIsClosed == true)
    {
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
    }
    mp_tick(S_KUPD, mp_t);
    // update之后要清空一下保存的因子图，注：历史数据不会清掉，ISAM保存起来了
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    // save key poses
    PointType thisPose3D;
    PointTypePose thisPose6D;
    Pose3 latestEstimate;

    // 优化结果
    isamCurrentEstimate = isam->calculateEstimate();
    mp_tick(S_KEST, mp_t);

    // std::cout << "isamCurrentEstimate.size: " << isamCurrentEstimate.size() << std::endl;
    // 当前帧位姿结果
    // latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);
    latestEstimate = isamCurrentEstimate.at<Pose3>(X_node[graph_pose_num - 1]);

   
    // cout << "****************************************************" << endl;
    // isamCurrentEstimate.print("Current estimate: ");

    // std::cout << "latestEstimate.x " << latestEstimate.translation().x() << std::endl;

    // cloudKeyPoses3D加入当前帧位姿
    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
    cloudKeyPoses3D->push_back(thisPose3D);

    // cloudKeyPoses6D加入当前帧位姿
    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    thisPose6D.time = timeLaserInfoCur;
    cloudKeyPoses6D->push_back(thisPose6D);

    // cout << "****************************************************" << endl;
    // cout << "Pose covariance:" << endl;
    // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
    // 位姿协方差
    // poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);
    poseCovariance = isam->marginalCovariance(X_node[graph_pose_num - 1]);
    mp_tick(S_KMARG, mp_t);

    // save updated transform
    transformTobeMapped[0] = latestEstimate.rotation().roll();
    transformTobeMapped[1] = latestEstimate.rotation().pitch();
    transformTobeMapped[2] = latestEstimate.rotation().yaw();
    transformTobeMapped[3] = latestEstimate.translation().x();
    transformTobeMapped[4] = latestEstimate.translation().y();
    transformTobeMapped[5] = latestEstimate.translation().z();

    // save all the received edge and surf points
    // 当前帧激光角点、平面点，降采样集合
    pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
    pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);

    // save key frame cloud
    cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);

    // save path for visualization
    // 更新里程计轨迹
    updatePath(thisPose6D);
    mp_tick(S_KCLOUD, mp_t);

    // std::cout << std::endl;
}

// 计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
bool Map_Optimization::saveFrame()
{
    if (cloudKeyPoses3D->points.empty())
        return true;

    // 前一帧位姿
    // 注：最开始没有的时候，在函数extractCloud里面有
    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    // 当前帧位姿
    Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                        transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    // 位姿变换增量
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);
    // 旋转和平移量都较小，当前帧不设为关键帧
    if (abs(roll) < surroundingkeyframeAddingAngleThreshold &&
        abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
        abs(yaw) < surroundingkeyframeAddingAngleThreshold &&
        sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold)
        return false;

    return true;
}

// 添加激光里程计因子
void Map_Optimization::addOdomFactor()
{
    if (cloudKeyPoses3D->points.empty())
    {
        // 第一帧初始化先验因子
        noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
        X_node.push_back(gtsam::Symbol('x', 0));
        // gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
        gtSAMgraph.add(PriorFactor<Pose3>(X_node[0], trans2gtsamPose(transformTobeMapped), priorNoise));
        // 变量节点设置初始值
        initialEstimate.insert(X_node[0], trans2gtsamPose(transformTobeMapped));
        graph_pose_num++;
    }
    else
    {
        // 添加激光里程计因子
        noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
        gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);

        while (X_node.size() <= cloudKeyPoses3D->size())
        {
            X_node.push_back(gtsam::Symbol('x', X_node.size()));
        }

        gtSAMgraph.add(BetweenFactor<Pose3>(X_node[cloudKeyPoses3D->size() - 1], X_node[cloudKeyPoses3D->size()], poseFrom.between(poseTo), odometryNoise));
        initialEstimate.insert(X_node[cloudKeyPoses3D->size()], poseTo);
        graph_pose_num++;
        // // 参数：前一帧id，当前帧id，前一帧与当前帧的位姿变换（作为观测值），噪声协方差
        // gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
        // // 变量节点设置初始值
        // initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
    }
}


// =====================================================================================
// [VIS] 视觉累积位姿回调：只做缓存（不加因子），保证不阻塞激光回调
// =====================================================================================
void Map_Optimization::vis_abs_handler(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    const double t = rclcpp::Time(msg->header.stamp).seconds();
    const auto &p = msg->pose.position;
    const auto &q = msg->pose.orientation;
    gtsam::Rot3 R = gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z);
    gtsam::Pose3 T(R, gtsam::Point3(p.x, p.y, p.z));
    std::lock_guard<std::mutex> lk(vis_mtx_);
    if (!vis_q_.empty() && t <= vis_q_.back().first) return;   // 乱序保护
    vis_q_.emplace_back(t, T);
    while (vis_q_.size() > 80) vis_q_.pop_front();             // ~8 s @10 Hz
}

// =====================================================================================
// [VIS] 在时刻 t 取视觉位姿：区间内插值 / 末样本后常速外推
// =====================================================================================
bool Map_Optimization::visPoseAt(const std::deque<std::pair<double, gtsam::Pose3>> &q,
                                 double t, gtsam::Pose3 &out) const
{
    // ★ GTSAM 4.2 的 Pose3/Rot3 **没有成员 .log()**，只有静态 Logmap()：
    //     gtsam::Pose3::Logmap(T) -> Vector6、gtsam::Rot3::Logmap(R) -> Vector3
    //   （已记在册的 GTSAM 4.2 四坑之一；写成成员函数会编译失败）

    if (q.size() < 2) return false;
    if (t < q.front().first - VIS_TOL) return false;          // 太早：没有样本
    if (t <= q.front().first) { out = q.front().second; return true; }
    if (t > q.back().first) {
        const auto &a = q[q.size() - 2];
        const auto &b = q.back();
        const double dt = b.first - a.first;
        if (dt <= 1e-6) return false;
        const double beta = (t - b.first) / dt;
        if (beta > VIS_MAXEXTRAP) return false;               // 外推太远：放弃
        out = b.second * gtsam::Pose3::Expmap(beta * gtsam::Pose3::Logmap(a.second.between(b.second)));
        return true;
    }
    size_t i = 0;
    while (i + 2 < q.size() && q[i + 1].first < t) ++i;
    const auto &a = q[i];
    const auto &b = q[i + 1];
    const double dt = b.first - a.first;
    if (dt <= 1e-6) { out = b.second; return true; }
    const double alpha = (t - a.first) / dt;
    out = a.second * gtsam::Pose3::Expmap(alpha * gtsam::Pose3::Logmap(a.second.between(b.second)));
    return true;
}

// =====================================================================================
// [VIS] 把「视觉在 (t_prev, t_cur) 上的相对位姿」加成 (n-1, n) 之间的 BetweenFactor
//       —— 必须在 addOdomFactor() 之后调用（那时 X_node[n] 已插入、初始值已给）
// =====================================================================================
void Map_Optimization::addVisualFactor()
{
    if (!VIS_ON) return;
    if (cloudKeyPoses3D->points.empty()) return;
    const size_t n = cloudKeyPoses3D->size();
    if (X_node.size() <= n) return;
    const double t_prev = cloudKeyPoses6D->points.back().time;
    const double t_cur = timeLaserInfoCur;
    const double span = t_cur - t_prev;
    if (span <= 1e-3) return;

    std::deque<std::pair<double, gtsam::Pose3>> q;
    {
        std::lock_guard<std::mutex> lk(vis_mtx_);
        q = vis_q_;
    }
    gtsam::Pose3 Pa, Pb;
    const bool okA = visPoseAt(q, t_prev, Pa);
    const bool okB = visPoseAt(q, t_cur, Pb);
    if (VIS_STAT)
    {
        // ★ 每帧一次（而不是每 50 帧一次）：诊断"因子为什么一条都没加上"时，
        //   稀疏打印会恰好落空（本段关键帧不足 50 个 ⇒ 一条都不打，看起来像没调用）。
        std::cout << "[VISDBG] n=" << n << " X=" << X_node.size()
                  << " q=" << q.size()
                  << " tprev=" << t_prev << " tcur=" << t_cur
                  << " span=" << span
                  << " qspan=[" << (q.empty() ? 0.0 : q.front().first) << ","
                  << (q.empty() ? 0.0 : q.back().first) << "]"
                  << " okA=" << (int)okA << " okB=" << (int)okB
                  << std::endl;
    }
    if (!okA || !okB) {
        ++vis_skip_;
        if (VIS_STAT && (vis_add_ + vis_skip_) % 50 == 0)
            std::cout << "[VIS] 加 " << vis_add_ << " 丢 " << vis_skip_
                      << "（该帧插值不可用 span=" << span << "）" << std::endl;
        return;
    }
    const gtsam::Pose3 Trel = Pa.between(Pb);
    if (Trel.translation().norm() > VIS_MAX_TR) {
        ++vis_skip_;
        if (VIS_STAT)
            std::cout << "[VISDBG] 丢弃：|Tvis|=" << Trel.translation().norm()
                      << " > VIS_MAX_TR=" << VIS_MAX_TR << std::endl;
        return;
    }

    Eigen::Matrix<double, 6, 1> sig;
    const double s = std::sqrt(span);
    sig << VIS_SIG_ROT * s, VIS_SIG_ROT * s, VIS_SIG_ROT * s,
           VIS_SIG_TR * s, VIS_SIG_TR * s, VIS_SIG_TR * s;
    gtsam::SharedNoiseModel base = gtsam::noiseModel::Diagonal::Sigmas(sig);
    gtsam::SharedNoiseModel noise = base;
    if (VIS_ROBUST > 0.0)
        noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(VIS_ROBUST), base);

    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(X_node[n - 1], X_node[n], Trel, noise));
    ++vis_add_;
    // ★ 诊断（比 A/B 更快回答"融合有没有可能帮忙"）：同一区间上，
    //   视觉约束 vs 激光里程计约束 差多少？系统性偏差说明标定/尺度有问题，
    //   那种情况下无论怎么调 sigma 都不会改善轨迹。
    if (VIS_STAT) {
        const gtsam::Pose3 Tlidar =
            pclPointTogtsamPose3(cloudKeyPoses6D->points.back()).between(trans2gtsamPose(transformTobeMapped));
        const gtsam::Pose3 d = Tlidar.between(Trel);
        std::cout << "[VIS] span=" << span
                  << "  dT=" << d.translation().norm()
                  << "  dR=" << gtsam::Rot3::Logmap(d.rotation()).norm() * 180.0 / M_PI
                  << "  |Tvis|=" << Trel.translation().norm() << std::endl;
    }
    if (VIS_STAT && (vis_add_ + vis_skip_) % 50 == 0)
        std::cout << "[VIS] 加 " << vis_add_ << " 丢 " << vis_skip_ << std::endl;
}

// 添加闭环因子
void Map_Optimization::addLoopFactor()
{
    if (loopIndexQueue.empty())
        return;

    // 闭环队列
    for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
    {
        // 闭环边对应两帧的索引
        int indexFrom = loopIndexQueue[i].first;
        int indexTo = loopIndexQueue[i].second;
        // 闭环边的位姿变换
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        while (X_node.size() <= cloudKeyPoses3D->size())
        {
            X_node.push_back(gtsam::Symbol('x', X_node.size()));
        }
        gtSAMgraph.add(BetweenFactor<Pose3>(X_node[indexFrom], X_node[indexTo], poseBetween, noiseBetween));
        // gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
}

// 更新里程计轨迹
void Map_Optimization::updatePath(const PointTypePose &pose_in)
{

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = rclcpp::Time(pose_in.time * 1e9); // pose_in.time为秒，乘nanosec

    pose_stamped.header.frame_id = odometryFrame;
    pose_stamped.pose.position.x = pose_in.x;
    pose_stamped.pose.position.y = pose_in.y;
    pose_stamped.pose.position.z = pose_in.z;
    tf2::Quaternion q;
    q.setRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);  // 设置旋转角，顺序为 roll, pitch, yaw
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    globalPath.poses.push_back(pose_stamped);
}

// 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿，更新里程计轨迹
void Map_Optimization::correctPoses()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    if (aLoopIsClosed == true)
    {
        // clear map cache
        // 清空局部map
        laserCloudMapContainer.clear();
        // [A3] 位姿全变了，增量局部地图必须整体重建
        incCorner.reset((float)mappingCornerLeafSize);
        incSurf.reset((float)mappingSurfLeafSize);
        incPose.clear();
        // clear path
        // 清空里程计轨迹
        globalPath.poses.clear();
        // update key poses
        // 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿
        // int numPoses = isamCurrentEstimate.size();
        int numPoses = graph_pose_num;
        for (int i = 0; i < numPoses; ++i)
        {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(X_node[i]).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(X_node[i]).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(X_node[i]).translation().z();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<Pose3>(X_node[i]).rotation().roll();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(X_node[i]).rotation().pitch();
            cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<Pose3>(X_node[i]).rotation().yaw();

            updatePath(cloudKeyPoses6D->points[i]);
        }

        aLoopIsClosed = false;
    }
}

// 发布激光里程计
void Map_Optimization::publishOdometry()
{
    // Publish odometry for ROS (global)
    // 发布激光里程计，odom等价map
    nav_msgs::msg::Odometry laserOdometryROS;
    laserOdometryROS.header.stamp = timeLaserInfoStamp;
    laserOdometryROS.header.frame_id = odometryFrame;
    laserOdometryROS.child_frame_id = "odom_mapping";
    laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
    laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
    laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
    tf2::Quaternion q;
    q.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    laserOdometryROS.pose.pose.orientation = tf2::toMsg(q);
    pubLaserOdometryGlobal->publish(laserOdometryROS);

    // Publish TF
    // 发布TF，odom->lidar

    geometry_msgs::msg::TransformStamped trans_odom_to_lidar;
    trans_odom_to_lidar.header.stamp = timeLaserInfoStamp;
    trans_odom_to_lidar.header.frame_id = odometryFrame;
    trans_odom_to_lidar.child_frame_id = "base_link";

    trans_odom_to_lidar.transform.translation.x = transformTobeMapped[3];
    trans_odom_to_lidar.transform.translation.y = transformTobeMapped[4];
    trans_odom_to_lidar.transform.translation.z = transformTobeMapped[5];
    trans_odom_to_lidar.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(trans_odom_to_lidar);

    // Publish odometry for ROS (incremental)
    static bool lastIncreOdomPubFlag = false;
    static nav_msgs::msg::Odometry laserOdomIncremental; // incremental odometry msg
    static Eigen::Affine3f increOdomAffine;         // incremental odometry in affine
    // 第一次数据直接用全局里程计初始化
    if (lastIncreOdomPubFlag == false)
    {
        lastIncreOdomPubFlag = true;
        laserOdomIncremental = laserOdometryROS;
        increOdomAffine = trans2Affine3f(transformTobeMapped);
    }
    else
    {
        Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
        increOdomAffine = increOdomAffine * affineIncre;

        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(increOdomAffine, x, y, z, roll, pitch, yaw);

        laserOdomIncremental.header.stamp = timeLaserInfoStamp;
        laserOdomIncremental.header.frame_id = odometryFrame;
        laserOdomIncremental.child_frame_id = "odom_mapping";

        laserOdomIncremental.pose.pose.position.x = x;
        laserOdomIncremental.pose.pose.position.y = y;
        laserOdomIncremental.pose.pose.position.z = z;

        tf2::Quaternion q_incre;
        q_incre.setRPY(roll, pitch, yaw);
        laserOdomIncremental.pose.pose.orientation = tf2::toMsg(q_incre);

        laserOdomIncremental.pose.covariance[0] = isDegenerate ? 1.0 : 0.0;
    }
    pubLaserOdometryIncremental->publish(laserOdomIncremental);
}

// 发布里程计、点云、轨迹
void Map_Optimization::publishFrames()
{
    // 1、发布历史关键帧位姿集合
    // 2、发布局部map的降采样平面点集合
    // 3、发布历史帧（累加的）的角点、平面点降采样集合
    // 4、发布里程计轨迹

    if (cloudKeyPoses3D->points.empty())
        return;
    auto mp_t = std::chrono::steady_clock::now();
    // publish key poses
    publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
    mp_tick(S_PKP, mp_t);
    // Publish surrounding key frames
    publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);
    mp_tick(S_PLOC, mp_t);
    // publish registered key frame

    if (pubRecentKeyFrame->get_subscription_count() != 0)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
        *cloudOut += *transformPointCloud(laserCloudCornerLastDS, &thisPose6D);
        *cloudOut += *transformPointCloud(laserCloudSurfLastDS, &thisPose6D);
        publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
    }
    if (pubRecentKeyFrame_corner->get_subscription_count() != 0)
    {
        PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
        pcl::PointCloud<PointType>::Ptr corner_cloudOut(new pcl::PointCloud<PointType>());
        corner_cloudOut = transformPointCloud(laserCloudCornerLastDS, &thisPose6D);
        // 发布边缘点集合（累积）
        publishCloud(pubRecentKeyFrame_corner, corner_cloudOut, timeLaserInfoStamp, odometryFrame);
    }
    // publish registered high-res raw cloud
    if (pubCloudRegisteredRaw->get_subscription_count() != 0)
    {
        // pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        // pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
        // PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
        // *cloudOut = *transformPointCloud(cloudOut, &thisPose6D);
        // publishCloud(&pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
    }
    mp_tick(S_PREG, mp_t);
    // publish path
    if (pubPath->get_subscription_count() != 0)
    {
        globalPath.header.stamp = timeLaserInfoStamp;
        globalPath.header.frame_id = odometryFrame;
        pubPath->publish(globalPath);
    }
    mp_tick(S_PPATH, mp_t);
}

void Map_Optimization::pointAssociateToMap(PointType const *const pi, PointType *const po)
{
    po->x = transPointAssociateToMap(0, 0) * pi->x + transPointAssociateToMap(0, 1) * pi->y + transPointAssociateToMap(0, 2) * pi->z + transPointAssociateToMap(0, 3);
    po->y = transPointAssociateToMap(1, 0) * pi->x + transPointAssociateToMap(1, 1) * pi->y + transPointAssociateToMap(1, 2) * pi->z + transPointAssociateToMap(1, 3);
    po->z = transPointAssociateToMap(2, 0) * pi->x + transPointAssociateToMap(2, 1) * pi->y + transPointAssociateToMap(2, 2) * pi->z + transPointAssociateToMap(2, 3);
    po->intensity = pi->intensity;
}

pcl::PointCloud<PointType>::Ptr Map_Optimization::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

gtsam::Pose3 Map_Optimization::pclPointTogtsamPose3(PointTypePose thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                        gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}

gtsam::Pose3 Map_Optimization::trans2gtsamPose(float transformIn[])
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

Eigen::Affine3f Map_Optimization::pclPointToAffine3f(PointTypePose thisPoint)
{
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

Eigen::Affine3f Map_Optimization::trans2Affine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

PointTypePose Map_Optimization::trans2PointTypePose(float transformIn[])
{
    PointTypePose thisPose6D;
    thisPose6D.x = transformIn[3];
    thisPose6D.y = transformIn[4];
    thisPose6D.z = transformIn[5];
    thisPose6D.roll = transformIn[0];
    thisPose6D.pitch = transformIn[1];
    thisPose6D.yaw = transformIn[2];
    return thisPose6D;
}

// 闭环线程
void Map_Optimization::loopClosureThread()
{
    //    1、闭环scan-to-map，icp优化位姿
    //  *   1) 在历史关键帧中查找与当前关键帧距离最近的关键帧集合，选择时间相隔较远的一帧作为候选闭环帧
    //  *   2) 提取当前关键帧特征点集合，降采样；提取闭环匹配关键帧前后相邻若干帧的关键帧特征点集合，降采样
    //  *   3) 执行scan-to-map优化，调用icp方法，得到优化后位姿，构造闭环因子需要的数据，在因子图优化中一并加入更新位姿
    //  * 2、rviz展示闭环边

    if (loopClosureEnableFlag == false)
        return;

    rclcpp::Rate rate(loopClosureFrequency);
    while (rclcpp::ok())
    {
        rate.sleep();
        performLoopClosure();
        visualizeLoopClosure();
    }
}

// 闭环scan-to-map，icp优化位姿
void Map_Optimization::performLoopClosure()
{
    //    1、在历史关键帧中查找与当前关键帧距离最近的关键帧集合，选择时间相隔较远的一帧作为候选闭环帧
    //  * 2、提取当前关键帧特征点集合，降采样；提取闭环匹配关键帧前后相邻若干帧的关键帧特征点集合，降采样
    //  * 3、执行scan-to-map优化，调用icp方法，得到优化后位姿，构造闭环因子需要的数据，在因子图优化中一并加入更新位姿
    //  * 注：闭环的时候没有立即更新当前帧的位姿，而是添加闭环因子，让图优化去更新位姿

    if (cloudKeyPoses3D->points.empty() == true)
        return;

    mtx.lock();
    *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
    *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    mtx.unlock();

    // find keys
    // 当前关键帧索引，候选闭环匹配帧索引
    int loopKeyCur;
    int loopKeyPre;
    if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
        if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
            return;

    // extract cloud
    // 提取
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
    {
        loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
        loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
        if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
            return;
        if (pubHistoryKeyFrames->get_subscription_count() != 0)
            publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
    }

    // ICP Settings
    // ICP参数设置
    static pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align clouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    // 未收敛，或者匹配不够好
    if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        return;

    // publish corrected cloud
    // 发布当前关键帧经过闭环优化后的位姿变换之后的特征点云
    if (pubIcpKeyFrames->get_subscription_count() != 0)
    {
        pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
        publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
    }

    // Get pose transformation
    // 闭环优化得到的当前关键帧与闭环关键帧之间的位姿变换
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    // transform from world origin to wrong pose
    // 闭环优化前当前帧位姿
    Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
    // transform from world origin to corrected pose
    // 闭环优化后当前帧位姿
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    // 闭环匹配帧的位姿
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore();
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

    // std::cout << "icp Score:" << icp.getFitnessScore() << std::endl;
    // std::cout << "x: " << x << ", y: " << y << ", z: " << z << ", roll:" << roll << ", pitch: "  << pitch << ", yaw: " << yaw  << std::endl;

    // Add pose constraint
    // 添加闭环因子需要的数据
    // 这些内容会在函数addLoopFactor中用到
    mtx.lock();
    loopIndexQueue.push_back(std::make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(constraintNoise);
    mtx.unlock();

    // add loop constriant
    loopIndexContainer[loopKeyCur] = loopKeyPre;
}

// rviz展示闭环边
void Map_Optimization::visualizeLoopClosure()
{
    if (loopIndexContainer.empty())
        return;

    visualization_msgs::msg::MarkerArray markerArray;
    // loop nodes
    visualization_msgs::msg::Marker markerNode;
    markerNode.header.frame_id = odometryFrame;
    markerNode.header.stamp = timeLaserInfoStamp;
    markerNode.action = visualization_msgs::msg::Marker::ADD;
    markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3;
    markerNode.scale.y = 0.3;
    markerNode.scale.z = 0.3;
    markerNode.color.r = 0;
    markerNode.color.g = 0.8;
    markerNode.color.b = 1;
    markerNode.color.a = 1;
    // loop edges
    visualization_msgs::msg::Marker markerEdge;
    markerEdge.header.frame_id = odometryFrame;
    markerEdge.header.stamp = timeLaserInfoStamp;
    markerEdge.action = visualization_msgs::msg::Marker::ADD;
    markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0.9;
    markerEdge.color.b = 0;
    markerEdge.color.a = 1;
    // 遍历闭环
    for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
    {
        int key_cur = it->first;
        int key_pre = it->second;
        geometry_msgs::msg::Point p;
        p.x = copy_cloudKeyPoses6D->points[key_cur].x;
        p.y = copy_cloudKeyPoses6D->points[key_cur].y;
        p.z = copy_cloudKeyPoses6D->points[key_cur].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = copy_cloudKeyPoses6D->points[key_pre].x;
        p.y = copy_cloudKeyPoses6D->points[key_pre].y;
        p.z = copy_cloudKeyPoses6D->points[key_pre].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubLoopConstraintEdge->publish(markerArray);
}

// 在历史关键帧中查找与当前关键帧距离最近的关键帧集合，选择时间相隔较远的一帧作为候选闭环帧
bool Map_Optimization::detectLoopClosureDistance(int *latestID, int *closestID)
{
    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
    int loopKeyPre = -1;

    // check loop constraint added before
    // 当前帧已经添加过闭环对应关系，不再继续添加
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;

    // find the closest history key frame
    // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合
    // 配置文件中默认historyKeyframeSearchRadius=15m
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
    kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

    // 在候选关键帧集合中，找到与当前帧时间相隔较远的帧，设为候选匹配帧
    // 配置文件中默认30s
    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
    {
        int id = pointSearchIndLoop[i];
        if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff)
        {
            loopKeyPre = id;
            break;
        }
    }

    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
        return false;

    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    return true;
}

// 来自外部闭环检测程序提供的闭环匹配索引对
bool Map_Optimization::detectLoopClosureExternal(int *latestID, int *closestID)
{
    // this function is not used yet, please ignore it
    int loopKeyCur = -1;
    int loopKeyPre = -1;

    std::lock_guard<std::mutex> lock(mtxLoopInfo);
    if (loopInfoVec.empty())
        return false;

    double loopTimeCur = loopInfoVec.front().data[0];
    double loopTimePre = loopInfoVec.front().data[1];
    loopInfoVec.pop_front();

    if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
        return false;

    int cloudSize = copy_cloudKeyPoses6D->size();
    if (cloudSize < 2)
        return false;

    // latest key
    loopKeyCur = cloudSize - 1;
    for (int i = cloudSize - 1; i >= 0; --i)
    {
        if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
            loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
        else
            break;
    }

    // previous key
    loopKeyPre = 0;
    for (int i = 0; i < cloudSize; ++i)
    {
        if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
            loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
        else
            break;
    }

    if (loopKeyCur == loopKeyPre)
        return false;

    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;

    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    return true;
}

// 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合，降采样
void Map_Optimization::loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum)
{
    // extract near keyframes
    // 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合
    nearKeyframes->clear();
    int cloudSize = copy_cloudKeyPoses6D->size();
    // 通过-searchNum 到 +searchNum，搜索key两侧内容
    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize)
            continue;
        *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
        *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

// 展示线程
void Map_Optimization::visualizeGlobalMapThread()
{
    // 1、发布局部关键帧map的特征点云
    // 2、保存全局关键帧特征点集合

    rclcpp::Rate rate(0.2);
    while (rclcpp::ok())
    {
        rate.sleep();
        publishGlobalMap();
    }

    if (savePCD == false)
        return;

    // LO_SAM::save_mapRequest req;
    // LO_SAM::save_mapResponse res;

    // if (!saveMapService(req, res))
    // {
    //     cout << "Fail to save map" << endl;
    // }
}

// 发布局部关键帧map的特征点云
void Map_Optimization::publishGlobalMap()
{
    if (pubLaserCloudSurround->get_subscription_count() == 0)
        return;

    if (cloudKeyPoses3D->points.empty() == true)
        return;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

    // kd-tree to find near key frames to visualize
    // kdtree查找最近一帧关键帧相邻的关键帧集合
    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;
    // search near key frames to visualize
    mtx.lock();
    kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
    kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
    mtx.unlock();

    for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
        globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
    // downsample near selected key frames
    // 降采样
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;                                                                                            // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
    downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
    for (auto &pt : globalMapKeyPosesDS->points)
    {
        kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
        pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
    }

    // extract visualized and downsampled key frames
    // 提取局部相邻关键帧对应的特征点云
    for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i)
    {
        if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
            continue;
        int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
        *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
        *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    }
    // downsample visualized points
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;                                                                                   // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
    publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
}

void Map_Optimization::saveMap()
{
    // 保存全局关键帧特征点集合

    std::cout << "****************************************************" << std::endl;
    std::cout << "Saving map to pcd files ..." << std::endl;

    std::cout << "Save destination: " << (std::string(ROOT_DIR) + "PCD/") << std::endl;
    // create directory and remove old files;

    // save key frame transformations
    pcl::io::savePCDFileBinary(std::string(ROOT_DIR) + "PCD/trajectory.pcd", *cloudKeyPoses3D);
    pcl::io::savePCDFileBinary(std::string(ROOT_DIR) + "PCD/transformations.pcd", *cloudKeyPoses6D);
    // extract global point cloud map
    pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++)
    {
        *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
        *globalSurfCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
        std::cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
    }
    std::cout << std::endl;

    // save corner cloud
    pcl::io::savePCDFileBinary(std::string(ROOT_DIR) + "PCD/CornerMap.pcd", *globalCornerCloud);
    // save surf cloud
    pcl::io::savePCDFileBinary(std::string(ROOT_DIR) + "PCD/SurfMap.pcd", *globalSurfCloud);

    std::cout << "Saving map to pcd files completed" << std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Map_Optimization>(rclcpp::NodeOptions());

    RCLCPP_INFO(rclcpp::get_logger("MapOptimization"), "\033[1;32m----> Map Optimization Started.\033[0m");

    // 启动线程，传入智能指针
    std::thread loopthread(&Map_Optimization::loopClosureThread, node);
    std::thread visualizeMapThread(&Map_Optimization::visualizeGlobalMapThread, node);

    rclcpp::spin(node);

    // 等待线程退出
    loopthread.join();
    visualizeMapThread.join();

    rclcpp::shutdown();
    return 0;
}
