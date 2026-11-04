#include <cmath>
#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <algorithm>
#include <mutex>
#include <queue>
#include <map>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdlib>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "lidarFactor.hpp"
#include "lo_sam/msg/cloud_info.hpp"


#define DISTORTION 0

using std::placeholders::_1;

// =====================================================================================
// [A6] laserOdometry 的 1-NN：PCL FLANN 树 → 体素栅格 + 逐圈扩张
//
// 动机（`FPGA加速点…` §十一 / 「未修清单」第 2 条）：
//   默认演示（`ODOM_SURF_LEAF` 不设 = 不降采样）下，surf 上一帧云约 10.8 万点，
//   `kdtreeSurfLast->setInputCloud()` 每帧重建 FLANN 要 **265 ms**，whole 帧 **591 ms**；
//   加 ODOM_SURF_LEAF=0.4 降到 155 ms，但那只是把点砍到 6 千 —— 树仍然每帧重建。
//   本改动**不动任何算法参数**：仍是对"上一帧点云"做精确 1-NN，只换数据结构。
//
// ★ 语义保证（默认不改任何结果）：
//   设体素边长 L，查询点所在体素 C。与查询点距离 <= d 的点，其体素索引每轴与 C 相差
//   <= ceil(d/L)。故按 Chebyshev 半径 r 扫过的立方体**必然**包含所有距离 <= r*L 的点 ——
//   一旦已找到的最近距离 d1 <= r*L，答案就精确（圈外不可能更近）。
//   ⇒ 与原码 `nearestKSearch(q,1,...)` 给出**同一个最近点**（距离并列时可能取到另一个
//   等距点，概率为零；逐点对拍可验）。
//   圈数上限 Rmax = ceil(5.0/L)：原码门限是 `pointSearchSqDis[0] < 25`（即 5 m），
//   5 m 内一个点都没有时上层本来就会拒绝，故不必再往外扩。
//
// 开关（默认 = 原版行为）：
//   ODOM_NN=voxel|kdtree      默认 kdtree
//   ODOM_NN_LEAF=1.0          体素边长（m）
//   ODOM_NN_CHECK=N           前 N 帧逐点对拍 voxel vs FLANN（诊断，勿用于计时）
//   ODOM_NN_DBG=1             每帧打印栅格规模与建表耗时
// =====================================================================================
static const std::string A6_NN_MODE = []() {
        const char *v = std::getenv("ODOM_NN");
        return std::string(v ? v : "kdtree");
    }();
static const bool A6_USE_VOXEL = (A6_NN_MODE == "voxel");
static const float A6_LEAF = []() {
        const char *v = std::getenv("ODOM_NN_LEAF");
        float f = v ? (float)std::atof(v) : 1.0f;
        return f > 0.02f ? f : 1.0f;
    }();
// 原码门限 DISTANCE_SQ_THRESHOLD = 25 ⇒ 半径 5 m；扫到这一圈就够，再远必被拒
static const int A6_RING = []() {
        const char *v = std::getenv("ODOM_NN_RMAX");       // 仅诊断用：强行限制圈数
        if (v != nullptr) { int r = std::atoi(v); return r < 1 ? 1 : r; }
        int r = (int)std::ceil(5.0f / A6_LEAF);
        return r < 1 ? 1 : r;
    }();
// corner 云很稀（~1450 点 / ~550 格@1m），叶子太细反而要扩很多圈 ⇒ 单独可配，默认 1.0
static const float A6_LEAF_COR = []() {
        const char *v = std::getenv("ODOM_NN_LEAF_COR");
        float f = v ? (float)std::atof(v) : 1.0f;
        return f > 0.02f ? f : 1.0f;
    }();
static const int A6_CHECK_N = []() {
        const char *v = std::getenv("ODOM_NN_CHECK");
        return v ? std::atoi(v) : 0;
    }();
static const bool A6_DBG = (std::getenv("ODOM_NN_DBG") != nullptr);

// [A7] 相邻线搜索：walk = 原版"沿数组走"（默认）；bucket = 按线号分桶，与数组顺序无关
static const bool A7_BUCKET = []() {
        const char *v = std::getenv("ODOM_LINE");
        return v != nullptr && std::string(v) == "bucket";
    }();

// 有序键 + CSR 分组 + 开放寻址哈希（与 map_optimization 的 [A2] 同一套结构，见其 §十五）
struct VoxelGridNN1
{
    static constexpr int64_t EMPTY = INT64_MIN;
    float  leaf = 1.0f, inv = 1.0f;
    size_t npts = 0;
    int    hbits = 4, hmask = 15;
    const pcl::PointCloud<PointType> *pc = nullptr;
    std::vector<int64_t> keys;                 // 每格键（哈希校验值）
    std::vector<int>     beg;                  // CSR：nkeys+1
    std::vector<int>     ids;                  // CSR：按 key 分组的点索引
    std::vector<int64_t> hkey;                 // 哈希槽 → 键
    std::vector<int>     hval;                 // 哈希槽 → 桶号
    std::vector<std::pair<int64_t, int>> kv;    // build 暂存，复用容量

    static inline int64_t pack(int x, int y, int z)
    {
        return ((int64_t)(x & 0x1FFFFF))
             | ((int64_t)(y & 0x1FFFFF) << 21)
             | ((int64_t)(z & 0x1FFFFF) << 42);
    }
    inline int cellIdx(float v) const   // = floor(v/L)
    {
        const float f = v * inv;
        const int   i = (int)f;
        return (f < (float)i) ? (i - 1) : i;
    }
    void build(const pcl::PointCloud<PointType> &c, float lf)
    {
        leaf = lf; inv = 1.0f / lf; pc = &c;
        keys.clear(); beg.clear(); ids.clear(); kv.clear();
        kv.reserve(c.points.size());
        size_t m = 0;
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
    int ncells() const { return (int)keys.size(); }
    // 诊断：按 (x,y,z) 的**浮点位模式**统计完全相同的点数（O(n log n)，只在 ODOM_NN_DBG 时调用）
    static size_t countExactDup(const pcl::PointCloud<PointType> &c, size_t *nDistinct)
    {
        std::vector<std::array<uint32_t, 3>> v;
        v.reserve(c.points.size());
        for (const auto &p : c.points)
        {
            std::array<uint32_t, 3> a;
            std::memcpy(&a[0], &p.x, 4);
            std::memcpy(&a[1], &p.y, 4);
            std::memcpy(&a[2], &p.z, 4);
            v.push_back(a);
        }
        std::sort(v.begin(), v.end());
        size_t d = v.empty() ? 0 : 1;
        for (size_t i = 1; i < v.size(); ++i)
            if (v[i] != v[i - 1]) ++d;
        if (nDistinct) *nDistinct = d;
        return v.size() - d;
    }
    // 精确 1-NN。返回 1 = 找到一个（可能是 5 m 内无点时的"最好的那个"，上层门限会拒），0 = 云为空。
    int nn1(const PointType &q, int *oi, float *od, int *ringUsed = nullptr) const
    {
        if (npts == 0) { *oi = -1; *od = std::numeric_limits<float>::max(); if (ringUsed) *ringUsed = -1; return 0; }
        const int cx = cellIdx(q.x), cy = cellIdx(q.y), cz = cellIdx(q.z);
        int   bi = -1;
        float bd = std::numeric_limits<float>::max();
        for (int r = 1; r <= A6_RING; ++r)
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
                            const float d2 = ax * ax + ay * ay + az * az;
                            if (d2 < bd) { bd = d2; bi = ids[t]; }
                        }
                    }
            // ★ bd 是**平方**距离，覆盖半径必须平方后再比（A2 就是在这里踩过坑）
            if (bi >= 0 && bd <= (float)r * leaf * (float)r * leaf) { if (ringUsed) *ringUsed = r; break; }
        }
        *oi = bi; *od = bd;
        return 1;
    }
};

class LaserOdometryNode : public rclcpp::Node
{
public:
    LaserOdometryNode()
    : Node("laserOdometry"),
      skipFrameNum(5),
      systemInited(false),
      corner_correspondence(0),
      plane_correspondence(0),
      SCAN_PERIOD(0.1),
      DISTANCE_SQ_THRESHOLD(25),
      NEARBY_SCAN(2.5),
      frameCount(0)
    {
        this->declare_parameter<int>("mapping_skip_frame", 2);
        this->get_parameter("mapping_skip_frame", skipFrameNum);

        subCornerPointsSharp = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "feature/cloud_sharp", 100, std::bind(&LaserOdometryNode::laserCloudSharpHandler, this, _1));
        subCornerPointsLessSharp = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "feature/cloud_less_sharp", 100, std::bind(&LaserOdometryNode::laserCloudLessSharpHandler, this, _1));
        subSurfPointsFlat = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "feature/cloud_flat", 100, std::bind(&LaserOdometryNode::laserCloudFlatHandler, this, _1));
        subSurfPointsLessFlat = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "feature/cloud_less_flat", 100, std::bind(&LaserOdometryNode::laserCloudLessFlatHandler, this, _1));
        subLaserCloudFullRes = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "feature/cloud_full", 100, std::bind(&LaserOdometryNode::laserCloudFullResHandler, this, _1));

        pubLaserCloudCornerLast = this->create_publisher<sensor_msgs::msg::PointCloud2>("odom_init/cloud_edge", 100);
        pubLaserCloudSurfLast = this->create_publisher<sensor_msgs::msg::PointCloud2>("odom_init/cloud_surf", 100);
        pubLaserCloudFullRes = this->create_publisher<sensor_msgs::msg::PointCloud2>("odom_init/cloud_full", 100);
        pubLaserOdometry = this->create_publisher<nav_msgs::msg::Odometry>("odom_init/laser_odom", 100);
        pubLaserPath = this->create_publisher<nav_msgs::msg::Path>("odom_init/path", 100);
        odom_cloud_pub = this->create_publisher<lo_sam::msg::CloudInfo>("odom_init/odom_cloud", 100);

        kdtreeCornerLast.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
        kdtreeSurfLast.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());

        cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
        cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
        surfPointsFlat.reset(new pcl::PointCloud<PointType>());
        surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
        laserCloudFullRes.reset(new pcl::PointCloud<PointType>());

        para_q[0] = 0; para_q[1] = 0; para_q[2] = 0; para_q[3] = 1;
        para_t[0] = 0; para_t[1] = 0; para_t[2] = 0;
        q_last_curr = Eigen::Map<Eigen::Quaterniond>(para_q);
        t_last_curr = Eigen::Map<Eigen::Vector3d>(para_t);

        // [SYNC] 轮询周期可配（2026-10-17）：原为硬编码 200 ms => 每帧最多 200 ms 空转，
        //   且 200 ms 即 5 Hz 硬上限。默认 5 ms（200 Hz，对齐 A-LOAM 的 100 Hz 轮询式设计）。
        int poll_ms = 5;
        { const char *e = std::getenv("ODOM_POLL_MS"); if (e != nullptr) poll_ms = std::atoi(e); if (poll_ms < 1) poll_ms = 1; }
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(poll_ms),
            std::bind(&LaserOdometryNode::process, this));
        RCLCPP_INFO(this->get_logger(), "[SYNC] laserOdometry poll period = %d ms", poll_ms);
    }

private:
    // Callback handlers just push data into buffers with mutex locking
    void laserCloudSharpHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mBuf);
        cornerSharpBuf.push(msg);
    }
    void laserCloudLessSharpHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mBuf);
        cornerLessSharpBuf.push(msg);
    }
    void laserCloudFlatHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mBuf);
        surfFlatBuf.push(msg);
    }
    void laserCloudLessFlatHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mBuf);
        surfLessFlatBuf.push(msg);
    }
    void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mBuf);
        fullPointsBuf.push(msg);
    }

    // undistort lidar point
    void TransformToStart(const PointType *pi, PointType *po)
    {
        double s = DISTORTION ? (pi->intensity - int(pi->intensity)) / SCAN_PERIOD : 1.0;
        Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
        Eigen::Vector3d t_point_last = s * t_last_curr;
        Eigen::Vector3d point(pi->x, pi->y, pi->z);
        Eigen::Vector3d un_point = q_point_last * point + t_point_last;

        po->x = un_point.x();
        po->y = un_point.y();
        po->z = un_point.z();
        po->intensity = pi->intensity;
    }
    void TransformToEnd(const PointType *pi, PointType *po)
    {
        PointType un_point_tmp;
        TransformToStart(pi, &un_point_tmp);

        Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
        Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

        po->x = point_end.x();
        po->y = point_end.y();
        po->z = point_end.z();
        po->intensity = int(pi->intensity);
    }

    void process()
    {
        if (!rclcpp::ok())
            return;

        std::lock_guard<std::mutex> lock(mBuf);

        // [SYNC] 五话题按 stamp 对齐 + 失步重同步（2026-10-17）
        //   原实现：前队戳不一致 -> warn + return，且**一条都不 pop**
        //   => 任一路被 DDS 丢帧后五条队列前队永久错位、静默死锁
        //      （实测 6.87 Hz 回放：308 条告警、只处理 26/628 帧、CPU 17.8% 在空等）
        //   现实现：丢弃"最旧的那一路"前队（可能多路并列），逐次推进直到五路前队对齐。
        int sync_drop = 0;
        bool synced = false;
        for (int attempt = 0; attempt < 512; ++attempt)
        {
            if (cornerSharpBuf.empty() || cornerLessSharpBuf.empty() ||
                surfFlatBuf.empty() || surfLessFlatBuf.empty() ||
                fullPointsBuf.empty())
                return;

            auto key = [](const auto &t) { return std::make_pair(t.sec, t.nanosec); };
            const auto k0 = key(cornerSharpBuf.front()->header.stamp);
            const auto k1 = key(cornerLessSharpBuf.front()->header.stamp);
            const auto k2 = key(surfFlatBuf.front()->header.stamp);
            const auto k3 = key(surfLessFlatBuf.front()->header.stamp);
            const auto k4 = key(fullPointsBuf.front()->header.stamp);

            if (k0 == k4 && k1 == k4 && k2 == k4 && k3 == k4) { synced = true; break; }

            auto kmin = k0;
            if (k1 < kmin) kmin = k1;
            if (k2 < kmin) kmin = k2;
            if (k3 < kmin) kmin = k3;
            if (k4 < kmin) kmin = k4;
            if (k0 == kmin) cornerSharpBuf.pop();
            if (k1 == kmin) cornerLessSharpBuf.pop();
            if (k2 == kmin) surfFlatBuf.pop();
            if (k3 == kmin) surfLessFlatBuf.pop();
            if (k4 == kmin) fullPointsBuf.pop();
            ++sync_drop;
        }
        if (!synced)
        {
            RCLCPP_WARN(this->get_logger(), "Cloud info resync failed after 512 drops");
            return;
        }
        if (sync_drop > 0)
            RCLCPP_WARN(this->get_logger(), "[SYNC] resync: dropped %d stale msg(s)", sync_drop);

        auto timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp;
        auto timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp;
        auto timeSurfPointsFlat = surfFlatBuf.front()->header.stamp;
        auto timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp;
        auto timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp;

        TicToc t_whole;
        // [PROF] 分段计时（2026-10-15）
        TicToc t_deser;
        double prof_deser = 0.0, prof_tree = 0.0, prof_pub = 0.0;
        double prof_corner = 0.0, prof_surf = 0.0, prof_solve = 0.0, prof_opt = 0.0;
        double prof_ds = 0.0;
        // [ITER] Ceres 迭代/残差诊断（2026-10-16）
        int prof_iters = 0, prof_nres = 0, prof_nr = 0, prof_term = 0;
        double prof_cost0 = 0.0, prof_cost1 = 0.0;
        
        pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
        cornerSharpBuf.pop();

        pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
        cornerLessSharpBuf.pop();

        pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
        surfFlatBuf.pop();

        pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
        surfLessFlatBuf.pop();

        pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
        fullPointsBuf.pop();
        prof_deser = t_deser.toc();
        // [DS] 体素降采样：复原 canonical LOAM scanRegistration 的 downSizeFilterSurf
        //   env ODOM_SURF_LEAF 未设或 <=0 = 关闭；>0 = 启用（单位 m）。用 env 便于不重编译做 A/B
        TicToc t_ds;
        double ds_leaf = 0.0;
        {
            const char *env_ds = std::getenv("ODOM_SURF_LEAF");
            if (env_ds != nullptr) ds_leaf = std::atof(env_ds);
        }
        int ds_surf_in = static_cast<int>(surfPointsLessFlat->points.size());
        int ds_surf_out = ds_surf_in;
        if (ds_leaf > 0.0 && ds_surf_in > 0)
        {
            pcl::VoxelGrid<PointType> vg_surf;
            vg_surf.setInputCloud(surfPointsLessFlat);
            vg_surf.setLeafSize(ds_leaf, ds_leaf, ds_leaf);
            pcl::PointCloud<PointType>::Ptr ds_ptr(new pcl::PointCloud<PointType>());
            vg_surf.filter(*ds_ptr);
            surfPointsLessFlat = ds_ptr;
            ds_surf_out = static_cast<int>(surfPointsLessFlat->points.size());
        }
        prof_ds = t_ds.toc();
        // [ORD] 目标云数组的扫描线有序性签名（2026-10-16）
        {
          const auto &cl = laserCloudSurfLast->points;
          if (cl.size() > 2) {
            double sabs = 0.0, imin = cl[0].intensity, imax = cl[0].intensity; long long mono = 0, nrev = 0, n = 0;
            for (size_t i = 0; i + 1 < cl.size(); ++i) {
              double d = double(cl[i+1].intensity) - double(cl[i].intensity);
              sabs += std::fabs(d); if (d >= 0.0) mono++; if (d < -0.5) nrev++; n++;
              if (cl[i+1].intensity < imin) imin = cl[i+1].intensity;
              if (cl[i+1].intensity > imax) imax = cl[i+1].intensity;
            }
            printf("[ORD] n=%zu meanabsd=%.4f mono=%.3f rev=%.3f imin=%.2f imax=%.2f\n",
                   cl.size(), sabs/double(n), double(mono)/double(n), double(nrev)/double(n), imin, imax);
          }
        }

        // main odometry processing
        
        if (!systemInited)
        {
            systemInited = true;
            RCLCPP_INFO(this->get_logger(), "Initialization finished");
            return;
        }
        else
        {
            int cornerPointsSharpNum = cornerPointsSharp->points.size();
            int surfPointsFlatNum = surfPointsFlat->points.size();

            // Ceres optimization
            TicToc t_opt;
            for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
            {
                TicToc t_corner_blk;
                corner_correspondence = 0;
                plane_correspondence = 0;

                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                // [MVP0] Ceres 版本兼容：2.1+ 用 Manifold，2.0 及以下用 LocalParameterization
#if CERES_VERSION_MAJOR > 2 || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
                ceres::Manifold *q_parameterization = new ceres::EigenQuaternionManifold();
#else
                ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
#endif
                ceres::Problem::Options problem_options;
                ceres::Problem problem(problem_options);

                problem.AddParameterBlock(para_q, 4, q_parameterization);
                problem.AddParameterBlock(para_t, 3);

                pcl::PointXYZI pointSel;
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;

                // Find corner correspondences
                for (int i = 0; i < cornerPointsSharpNum; ++i)
                {
                    TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);
                    TicToc t_a7n;
                    const int a7_ret = a6Query(true, pointSel, pointSearchInd, pointSearchSqDis);
                    prof_nn += t_a7n.toc();
                    if (a7_ret > 0)
                    {
                        int closestPointInd = -1, minPointInd2 = -1;
                        float a7_bsel = std::numeric_limits<float>::max();
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            closestPointInd = pointSearchInd[0];
                            const int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);

                            TicToc t_a7b;
                            if (A7_BUCKET)
                            {
                                float d1v = 0.f, d2v = 0.f;
                                const int c1 = nearestInLines(true, pointSel, closestPointScanID + 1, closestPointScanID + 2, -1, &d1v);
                                const int c2 = nearestInLines(true, pointSel, closestPointScanID - 2, closestPointScanID - 1, -1, &d2v);
                                if (c1 >= 0 && d1v < DISTANCE_SQ_THRESHOLD && (c2 < 0 || d1v <= d2v)) minPointInd2 = c1;
                                else if (c2 >= 0 && d2v < DISTANCE_SQ_THRESHOLD) minPointInd2 = c2;
                                a7_bsel = d1v;
                                if (c2 >= 0 && (c1 < 0 || d2v < d1v)) a7_bsel = d2v;
                            }
                            else
                            {
                                double dtmp = 0;
                                walkCornerNb(pointSel, closestPointInd, &minPointInd2, &dtmp);
                                a7_bsel = (float)dtmp;
                            }
                            prof_lsrch += t_a7b.toc();
                            if (A7_BUCKET && A6_CHECK_N > 0 && a6_frame < (long)A6_CHECK_N)
                            {
                                int w2 = -1; double wd2 = 0;
                                walkCornerNb(pointSel, closestPointInd, &w2, &wd2);
                                ++a7_chk_q;
                                if (w2 < 0 && minPointInd2 < 0) ++a7_same;
                                else if (w2 < 0) ++a7_bkt_better;
                                else if (minPointInd2 < 0) ++a7_walk_only;
                                else if (w2 == minPointInd2) ++a7_same;
                                else if ((double)a7_bsel <= wd2 + 1e-9) ++a7_bkt_better;
                                else ++a7_walk_better;
                            }
                            }

                        if (minPointInd2 >= 0)
                        {
                            Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                       cornerPointsSharp->points[i].y,
                                                       cornerPointsSharp->points[i].z);
                            Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                         laserCloudCornerLast->points[closestPointInd].y,
                                                         laserCloudCornerLast->points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                         laserCloudCornerLast->points[minPointInd2].y,
                                                         laserCloudCornerLast->points[minPointInd2].z);

                            double s = DISTORTION ?
                                (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD : 1.0;

                            TicToc t_a7r;
                            ceres::CostFunction *cost_function =
                                LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            prof_res += t_a7r.toc();
                            corner_correspondence++;
                        }
                    }
                }

                // Find plane correspondences
                prof_corner += t_corner_blk.toc();
                TicToc t_surf_blk;
                for (int i = 0; i < surfPointsFlatNum; ++i)
                {
                    TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
                    TicToc t_a7n;
                    const int a7_ret = a6Query(false, pointSel, pointSearchInd, pointSearchSqDis);
                    prof_nn += t_a7n.toc();
                    if (a7_ret > 0)
                    {
                        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                        float a7_bsel = std::numeric_limits<float>::max();
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            closestPointInd = pointSearchInd[0];
                            const int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);

                            TicToc t_a7b;
                            if (A7_BUCKET)
                            {
                                float d2v = 0.f, d3v = 0.f;
                                minPointInd2 = nearestInLines(false, pointSel, closestPointScanID - 2, closestPointScanID, closestPointInd, &d2v);
                                if (minPointInd2 >= 0 && !(d2v < DISTANCE_SQ_THRESHOLD)) minPointInd2 = -1;
                                minPointInd3 = nearestInLines(false, pointSel, closestPointScanID + 1, closestPointScanID + 2, closestPointInd, &d3v);
                                if (minPointInd3 >= 0 && !(d3v < DISTANCE_SQ_THRESHOLD)) minPointInd3 = -1;
                                a7_bsel = d2v;
                            }
                            else
                            {
                                double dt2 = 0, dt3 = 0;
                                walkSurfNb(pointSel, closestPointInd, &minPointInd2, &minPointInd3, &dt2, &dt3);
                                a7_bsel = (float)dt2;
                            }
                            prof_lsrch += t_a7b.toc();
                            if (A7_BUCKET && A6_CHECK_N > 0 && a6_frame < (long)A6_CHECK_N)
                            {
                                int w2 = -1, w3 = -1; double wd2 = 0, wd3 = 0;
                                walkSurfNb(pointSel, closestPointInd, &w2, &w3, &wd2, &wd3);
                                ++a7_chk_q;
                                const bool b_ok = (minPointInd2 >= 0), w_ok = (w2 >= 0);
                                const bool b_ok3 = (minPointInd3 >= 0), w_ok3 = (w3 >= 0);
                                if (!b_ok && !w_ok && !b_ok3 && !w_ok3) ++a7_same;
                                else if (b_ok == w_ok && b_ok3 == w_ok3 && minPointInd2 == w2 && minPointInd3 == w3) ++a7_same;
                                else if ((b_ok && w_ok && (double)a7_bsel <= wd2 + 1e-9) || (b_ok && !w_ok)) ++a7_bkt_better;
                                else if (w_ok && !b_ok) ++a7_walk_only;
                                else ++a7_walk_better;
                            }
                            }

                        if (minPointInd2 >= 0 && minPointInd3 >= 0)
                        {
                            TicToc t_a7r;
                            Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                       surfPointsFlat->points[i].y,
                                                       surfPointsFlat->points[i].z);
                            Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                        laserCloudSurfLast->points[closestPointInd].y,
                                                        laserCloudSurfLast->points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                        laserCloudSurfLast->points[minPointInd2].y,
                                                        laserCloudSurfLast->points[minPointInd2].z);
                            Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                        laserCloudSurfLast->points[minPointInd3].y,
                                                        laserCloudSurfLast->points[minPointInd3].z);

                            double s = DISTORTION ?
                                (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD : 1.0;

                            ceres::CostFunction *cost_function =
                                LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            prof_res += t_a7r.toc();
                            plane_correspondence++;
                        }
                    }
                }

                if ((corner_correspondence + plane_correspondence) < 10)
                {
                    RCLCPP_WARN(this->get_logger(), "Less correspondence in optimization");
                }

                prof_surf += t_surf_blk.toc();
                TicToc t_solve_blk;
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
                options.minimizer_progress_to_stdout = false;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
                prof_iters += static_cast<int>(summary.iterations.size());
                prof_nres   = static_cast<int>(summary.num_residual_blocks);
                prof_nr     = static_cast<int>(summary.num_residuals);
                prof_term   = static_cast<int>(summary.termination_type);
                prof_cost0 += summary.initial_cost;
                prof_cost1 += summary.final_cost;
                printf("[CERES] opti=%zu it=%d nres=%d nr=%d term=%d ok=%d un=%d tt=%.2f lst=%.2f cC=%d pC=%d nSharp=%d nFlat=%d\n",
                       opti_counter,
                       static_cast<int>(summary.iterations.size()),
                       static_cast<int>(summary.num_residual_blocks),
                       static_cast<int>(summary.num_residuals),
                       static_cast<int>(summary.termination_type),
                       summary.num_successful_steps,
                       summary.num_unsuccessful_steps,
                       summary.total_time_in_seconds,
                       summary.linear_solver_time_in_seconds,
                       corner_correspondence,
                       plane_correspondence,
                       cornerPointsSharpNum,
                       surfPointsFlatNum);
                prof_solve += t_solve_blk.toc();
            }

            t_w_curr = t_w_curr + q_w_curr * t_last_curr;
            q_w_curr = q_w_curr * q_last_curr;
            prof_opt = t_opt.toc();
        }

        // Publish result
        TicToc t_pub;
        nav_msgs::msg::Odometry laserOdometry;
        laserOdometry.header.frame_id = "camera_init";
        laserOdometry.child_frame_id = "laser_odom";
        laserOdometry.header.stamp = time_from_builtin(timeSurfPointsLessFlat);
        laserOdometry.pose.pose.orientation.x = q_w_curr.x();
        laserOdometry.pose.pose.orientation.y = q_w_curr.y();
        laserOdometry.pose.pose.orientation.z = q_w_curr.z();
        laserOdometry.pose.pose.orientation.w = q_w_curr.w();
        laserOdometry.pose.pose.position.x = t_w_curr.x();
        laserOdometry.pose.pose.position.y = t_w_curr.y();
        laserOdometry.pose.pose.position.z = t_w_curr.z();
        pubLaserOdometry->publish(laserOdometry);

        geometry_msgs::msg::PoseStamped laserPose;
        laserPose.header = laserOdometry.header;
        laserPose.pose = laserOdometry.pose.pose;
        laserPath.header.stamp = laserOdometry.header.stamp;
        laserPath.poses.push_back(laserPose);
        laserPath.header.frame_id = "camera_init";
        // pubLaserPath->publish(laserPath);

        // Swap point clouds
        TicToc t_tree;
        auto laserCloudTemp = cornerPointsLessSharp;
        cornerPointsLessSharp = laserCloudCornerLast;
        laserCloudCornerLast = laserCloudTemp;

        laserCloudTemp = surfPointsLessFlat;
        surfPointsLessFlat = laserCloudSurfLast;
        laserCloudSurfLast = laserCloudTemp;

        laserCloudCornerLastNum = static_cast<int>(laserCloudCornerLast->points.size());
        laserCloudSurfLastNum = static_cast<int>(laserCloudSurfLast->points.size());

        // [A6] 默认仍建 FLANN 树；ODOM_NN=voxel 时改建体素栅格（O(n) 建表，替代 kd 树）
        //      ODOM_NN_CHECK>0 时两者都建，供逐点对拍（诊断模式，勿用于计时）
        const bool a6_need_kd = (!A6_USE_VOXEL) || (A6_CHECK_N > 0);
        const bool a6_need_vx = A6_USE_VOXEL || (A6_CHECK_N > 0);
        if (a6_need_kd)
        {
            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
        }
        if (a6_need_vx)
        {
            nnCornerLast.build(*laserCloudCornerLast, A6_LEAF_COR);
            nnSurfLast.build(*laserCloudSurfLast, A6_LEAF);
        }
        // [A7] 按线号分桶（O(n)，仅 bucket 模式）
        if (A7_BUCKET)
        {
            buildLineBuckets(*laserCloudCornerLast, lineBegCor, linePtsCor, lineOrdCor, a7_nlines_cor);
            buildLineBuckets(*laserCloudSurfLast, lineBegSurf, linePtsSurf, lineOrdSurf, a7_nlines_surf);
        }
        prof_tree = t_tree.toc();
        // [A6] 诊断汇总（每帧一行；对拍窗口外也会打印实际栅格规模）
        if (A6_DBG || A6_CHECK_N > 0)
        {
            size_t nx = 0, nd = 0;
            const size_t dupx = VoxelGridNN1::countExactDup(*laserCloudSurfLast, &nx);
            const size_t dupc = VoxelGridNN1::countExactDup(*laserCloudCornerLast, &nd);
            printf("[A6] 重复点：surf %zu 点中 %zu 个是精确重复（唯一点 %zu）；corner %zu 点中 %zu（唯一点 %zu）\n",
                   laserCloudSurfLast->size(), dupx, nx, laserCloudCornerLast->size(), dupc, nd);
            printf("[A6] f%ld leaf %.2f R%d | cor %zu/%d surf %zu/%d | q %ld empty %ld | idx_diff %ld (dup %ld tie %ld BAD %ld) d_diff %ld dmax %.1e | ring %ld/%ld/%ld/%ld/%ld\n",
                   a6_frame, (double)A6_LEAF, A6_RING,
                   nnCornerLast.npts, nnCornerLast.ncells(),
                   nnSurfLast.npts, nnSurfLast.ncells(),
                   a6_q, a6_empty, a6_idx_diff, a6_dup, a6_tie, a6_bad, a6_d_diff, (double)a6_dmax,
                   a6_ring[0], a6_ring[1], a6_ring[2], a6_ring[3], a6_ring[4]);
            if (a6_bad > 0)
                printf("[A6]  !! BAD 细分：两边都拒 %ld，FLANN 更近 %ld，我们更近 %ld，max gap %.3e\n",
                       a6_gate_rej, a6_kd_closer, a6_we_closer, (double)a6_gap);
            a6_q = a6_empty = a6_idx_diff = a6_dup = a6_tie = a6_bad = a6_d_diff = 0;
            a6_gate_rej = a6_kd_closer = a6_we_closer = 0; a6_gap = 0.f;
            a6_dmax = 0.f;
            for (int t = 0; t < 8; ++t) a6_ring[t] = 0;
        }
        // [A7] 对拍汇总 + 细分计时（每帧一行）
        if (A6_DBG || A6_CHECK_N > 0 || A7_BUCKET)
        {
            printf("[A7] f%ld lines cor %d surf %d | q %ld same %ld bkt_better %ld walk_only %ld walk_better %ld | nn %.1f lsrch %.1f res %.1f ms\n",
                   a6_frame, a7_nlines_cor, a7_nlines_surf,
                   a7_chk_q, a7_same, a7_bkt_better, a7_walk_only, a7_walk_better,
                   prof_nn, prof_lsrch, prof_res);
            a7_chk_q = a7_same = a7_bkt_better = a7_walk_only = a7_walk_better = 0;
        }
        prof_nn = prof_lsrch = prof_res = 0.0;
        ++a6_frame;

        // Publish point clouds
        sensor_msgs::msg::PointCloud2 laserCloudCornerLast2;
        pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
        laserCloudCornerLast2.header.stamp = time_from_builtin(timeSurfPointsLessFlat);
        laserCloudCornerLast2.header.frame_id = "camera_init";
        pubLaserCloudCornerLast->publish(laserCloudCornerLast2);

        sensor_msgs::msg::PointCloud2 laserCloudSurfLast2;
        pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
        laserCloudSurfLast2.header.stamp = time_from_builtin(timeSurfPointsLessFlat);
        laserCloudSurfLast2.header.frame_id = "camera_init";
        pubLaserCloudSurfLast->publish(laserCloudSurfLast2);

        sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
        pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp = time_from_builtin(timeSurfPointsLessFlat);
        laserCloudFullRes3.header.frame_id = "camera_init";
        pubLaserCloudFullRes->publish(laserCloudFullRes3);

        lo_sam::msg::CloudInfo cloud_odom_msg;
        cloud_odom_msg.header = laserCloudCornerLast2.header;
        cloud_odom_msg.odom_init = laserOdometry;
        cloud_odom_msg.cloud_full = laserCloudFullRes3;
        cloud_odom_msg.cloud_surf = laserCloudSurfLast2;
        cloud_odom_msg.cloud_edge = laserCloudCornerLast2;
        odom_cloud_pub->publish(cloud_odom_msg);

        prof_pub = t_pub.toc();
        printf("[PROF] whole %7.1f | deser %6.1f | ds %6.1f | opt %7.1f (corner %6.1f surf %6.1f solve %6.1f) | tree %6.1f | pub %6.1f ms | leaf %.2f surf %d->%d | it %d nres %d nr %d term %d cost %.0f->%.0f | drop %d\n",
                       t_whole.toc(), prof_deser, prof_ds, prof_opt, prof_corner, prof_surf, prof_solve, prof_tree, prof_pub,
                       ds_leaf, ds_surf_in, ds_surf_out, prof_iters, prof_nres, prof_nr, prof_term, prof_cost0, prof_cost1, sync_drop);

        if (t_whole.toc() > 200)
        {
            RCLCPP_WARN(this->get_logger(), "Odometry process over 200ms");
        }

        frameCount++;
    }

    rclcpp::Time time_from_builtin(const builtin_interfaces::msg::Time & t)
    {
        return rclcpp::Time(t.sec, t.nanosec);
    }

private:
    int corner_correspondence, plane_correspondence;
    const double SCAN_PERIOD;
    const double DISTANCE_SQ_THRESHOLD;
    const double NEARBY_SCAN;

    int skipFrameNum;
    bool systemInited;

    double timeCornerPointsSharp;
    double timeCornerPointsLessSharp;
    double timeSurfPointsFlat;
    double timeSurfPointsLessFlat;
    double timeLaserCloudFullRes;

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast;

    // [A6] 体素栅格 1-NN（与上面两棵树二选一，见文件头 ODOM_NN 说明）
    VoxelGridNN1 nnCornerLast, nnSurfLast;

    // [A7] 扫描线分桶 + 细分计时
    std::vector<int> lineBegCor, lineOrdCor, lineBegSurf, lineOrdSurf;
    // [A7-v2] 线桶的 SoA 形式：点按线连续拷贝一份，扫 5 条线 = 5 段**顺序**读，无随机访存
    //   （v1 用 ord[t] 间接取 c.points[j]，每候选一次 cache miss，11.2 万点云上 4× 于 walk）
    std::vector<PointType> linePtsCor, linePtsSurf;
    int  a7_nlines_cor = 0, a7_nlines_surf = 0;
    long a7_chk_q = 0, a7_same = 0, a7_bkt_better = 0, a7_walk_only = 0, a7_walk_better = 0;
    double prof_nn = 0.0, prof_lsrch = 0.0, prof_res = 0.0;
    long  a6_frame = 0;
    long  a6_q = 0;          // 对拍查询数
    long  a6_empty = 0;      // 两边都判定"空云/无点"
    long  a6_d_diff = 0;     // 索引相同但 d2 相对差 > 1e-6
    long  a6_idx_diff = 0;   // 索引不同
    long  a6_dup = 0;        //   └ 其中两点坐标完全相同（重复点，选谁都一样）
    long  a6_tie = 0;        //   └ 其中 d2 相等（等距，几何等价）
    long  a6_bad = 0;        //   └ 其余 = 真不一致
    long  a6_gate_rej = 0;   // 两边都被 5 m 门限拒掉（无害）
    long  a6_kd_closer = 0;  // FLANN 更近（我们漏了 → 真错）
    long  a6_we_closer = 0;  // 我们更近（FLANN 不该漏 → 云不一致？）
    float a6_gap = 0.f;      // 两侧 d2 差的相对值（BAD 档里最大）
    float a6_dmax = 0.f;     // d2 最大相对差
    long  a6_ring[8] = {0, 0, 0, 0, 0, 0, 0, 0};


    // ===================== [A7] 扫描线相邻点搜索 =====================
    // 原版实现 = "沿数组走、线号出 ±2.5 就 break"，**隐含数组按扫描线有序**。
    // 实测该数组并非有序（[ORD] mono=0.855）：走查会提前 break（漏候选）或扫过大半数组（慢）。
    // ODOM_LINE=bucket 时改为"按线号分桶（CSR）+ 只扫线号 ∈ ±2 的桶"，与数组顺序无关。

    // 原版走查（corner）：min2 必须来自与 closest 不同的相邻线（±1、±2），两方向取更近者
    void walkCornerNb(const PointType &pointSel, int closestPointInd, int *out2, double *d2out)
    {
        const pcl::PointCloud<PointType> &c = *laserCloudCornerLast;
        const int closestPointScanID = int(c.points[closestPointInd].intensity);
        double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
        int minPointInd2 = -1;
{
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                            // Search upwards scan lines
                            for (int j = closestPointInd + 1; j < (int)c.points.size(); ++j)
                            {
                                if (int(c.points[j].intensity) <= closestPointScanID)
                                    continue;
                                if (int(c.points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                double pointSqDis = (c.points[j].x - pointSel.x) *
                                                        (c.points[j].x - pointSel.x) +
                                                    (c.points[j].y - pointSel.y) *
                                                        (c.points[j].y - pointSel.y) +
                                                    (c.points[j].z - pointSel.z) *
                                                        (c.points[j].z - pointSel.z);
                                if (pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }
                            // Search downwards scan lines
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                if (int(c.points[j].intensity) >= closestPointScanID)
                                    continue;
                                if (int(c.points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (c.points[j].x - pointSel.x) *
                                                        (c.points[j].x - pointSel.x) +
                                                    (c.points[j].y - pointSel.y) *
                                                        (c.points[j].y - pointSel.y) +
                                                    (c.points[j].z - pointSel.z) *
                                                        (c.points[j].z - pointSel.z);
                                if (pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }
}
        *out2 = minPointInd2;
        *d2out = minPointSqDis2;
    }

    // 原版走查（surf）：min2 ∈ 线号 [s-2, s]（含同线、不含 closest 本身），min3 ∈ [s+1, s+2]
    void walkSurfNb(const PointType &pointSel, int closestPointInd, int *out2, int *out3,
                    double *d2out, double *d3out)
    {
        const pcl::PointCloud<PointType> &c = *laserCloudSurfLast;
        const int closestPointScanID = int(c.points[closestPointInd].intensity);
        double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;
        int minPointInd2 = -1, minPointInd3 = -1;
{
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;
                            // Search increasing scan lines
                            for (int j = closestPointInd + 1; j < (int)c.points.size(); ++j)
                            {
                                if (int(c.points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                double pointSqDis = (c.points[j].x - pointSel.x) *
                                                        (c.points[j].x - pointSel.x) +
                                                    (c.points[j].y - pointSel.y) *
                                                        (c.points[j].y - pointSel.y) +
                                                    (c.points[j].z - pointSel.z) *
                                                        (c.points[j].z - pointSel.z);

                                if (int(c.points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                else if (int(c.points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }
                            // Search decreasing scan lines
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                if (int(c.points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (c.points[j].x - pointSel.x) *
                                                        (c.points[j].x - pointSel.x) +
                                                    (c.points[j].y - pointSel.y) *
                                                        (c.points[j].y - pointSel.y) +
                                                    (c.points[j].z - pointSel.z) *
                                                        (c.points[j].z - pointSel.z);

                                if (int(c.points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                else if (int(c.points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }
}
        *out2 = minPointInd2; *out3 = minPointInd3;
        *d2out = minPointSqDis2; *d3out = minPointSqDis3;
    }

    // 按线号 CSR 桶，在线号 ∈ [lo, hi]（含端点）内找最近点；exclIdx 跳过；没有则 (-1, FLT_MAX)
    int nearestInLines(bool isCorner, const PointType &q, int lo, int hi, int exclIdx, float *bestD2)
    {
        const std::vector<int>      &beg = isCorner ? lineBegCor  : lineBegSurf;
        const std::vector<PointType> &pa = isCorner ? linePtsCor  : linePtsSurf;
        const std::vector<int>      &idx = isCorner ? lineOrdCor  : lineOrdSurf;
        const int nl = isCorner ? a7_nlines_cor : a7_nlines_surf;
        if (lo < 0) lo = 0;
        if (hi > nl - 1) hi = nl - 1;
        int   best = -1;
        float bd   = std::numeric_limits<float>::max();
        for (int ln = lo; ln <= hi; ++ln)
        {
            const int e = beg[ln + 1];
            for (int t = beg[ln]; t < e; ++t)
            {
                if (idx[t] == exclIdx) continue;
                const PointType &p = pa[t];            // 顺序读，硬件预取友好
                const float ax = p.x - q.x, ay = p.y - q.y, az = p.z - q.z;
                const float d2 = ax * ax + ay * ay + az * az;
                if (d2 < bd) { bd = d2; best = idx[t]; }
            }
        }
        *bestD2 = bd;
        return best;
    }

    // 按线号计数排序（O(n)）：ord 里是按线分组的"点索引"，beg 是每线起点
    void buildLineBuckets(const pcl::PointCloud<PointType> &c, std::vector<int> &beg,
                          std::vector<PointType> &pts, std::vector<int> &ord, int &nl)
    {
        int mx = -1;
        for (const auto &p : c.points) { const int id = int(p.intensity); if (id > mx) mx = id; }
        nl = mx + 1; if (nl < 1) nl = 1;
        beg.assign(nl + 1, 0);
        for (const auto &p : c.points) { const int id = int(p.intensity); if (id >= 0 && id < nl) ++beg[id + 1]; }
        for (int i = 0; i < nl; ++i) beg[i + 1] += beg[i];
        std::vector<int> cur(beg.begin(), beg.end() - 1);
        pts.resize(c.points.size());
        ord.resize(c.points.size());
        for (size_t i = 0; i < c.points.size(); ++i)
        {
            const int id = int(c.points[i].intensity);
            if (id < 0 || id >= nl) continue;      // 越界线号进不了桶（计入 beg[nl] 之后的空隙）
            const int t = cur[id]++;
            pts[t] = c.points[i];                  // 连续拷贝，扫线时顺序读
            ord[t] = (int)i;
        }
    }

    // 1-NN 统一入口：**返回语义与 `nearestKSearch(q,1,ind,d2)` 完全一致**
    //   （>0 = 找到一个；ind[0]/d2[0] 已填）。voxel 分支在 5 m 内无点时给 d2[0]=FLT_MAX、
    //   ind[0]=-1 —— 上层 `if (pointSearchSqDis[0] < 25)` 必然拒绝，与原码路径等价。
    int a6Query(bool isCorner, const PointType &q, std::vector<int> &ind, std::vector<float> &d2)
    {
        const bool chk = (A6_CHECK_N > 0 && a6_frame < (long)A6_CHECK_N);
        int   ki = -1, vi = -1, ring = -1;
        float kd = 0.f, vd = 0.f;
        int   kret = 0, vret = 0;

        if (A6_USE_VOXEL)
        {
            if (isCorner) vret = nnCornerLast.nn1(q, &vi, &vd, &ring);
            else          vret = nnSurfLast.nn1(q, &vi, &vd, &ring);
            if (chk)
            {
                ki = -1; kd = 0.f;
                kret = isCorner ? kdtreeCornerLast->nearestKSearch(q, 1, ind, d2)
                                : kdtreeSurfLast->nearestKSearch(q, 1, ind, d2);
                if (kret > 0) { ki = ind[0]; kd = d2[0]; }
                a6Compare(isCorner ? *laserCloudCornerLast : *laserCloudSurfLast,
                          ki, kd, vi, vd, kret, vret, ring);
            }
            ind.resize(1); d2.resize(1);
            ind[0] = vi; d2[0] = vd;
            return vret;
        }

        kret = isCorner ? kdtreeCornerLast->nearestKSearch(q, 1, ind, d2)
                        : kdtreeSurfLast->nearestKSearch(q, 1, ind, d2);
        if (kret > 0) { ki = ind[0]; kd = d2[0]; }
        if (chk)
        {
            if (isCorner) vret = nnCornerLast.nn1(q, &vi, &vd, &ring);
            else          vret = nnSurfLast.nn1(q, &vi, &vd, &ring);
            a6Compare(isCorner ? *laserCloudCornerLast : *laserCloudSurfLast,
                      ki, kd, vi, vd, kret, vret, ring);
        }
        return kret;
    }
    // 对拍。★ 判定"是否有害"看的是**选中的点坐标**，不是索引：
    //   点到点云里重复点很多（0.4 m 网格平均 18 个点/格），d2 并列时两边可能各取一个，
    //   但坐标完全相同 ⇒ 下游 last_point_a/b/c 一模一样 ⇒ 结果逐位相同。
    void a6Compare(const pcl::PointCloud<PointType> &cloud,
                   int ki, float kd, int vi, float vd, int kret, int vret, int ring)
    {
        ++a6_q;
        if (ring >= 1 && ring <= 5) ++a6_ring[ring - 1]; else ++a6_ring[5];
        if (kret <= 0 || vret <= 0) { ++a6_empty; return; }

        const float sc  = (kd > 1.f) ? kd : 1.f;
        const float rel = std::fabs(kd - vd) / sc;
        if (rel > a6_dmax) a6_dmax = rel;

        if (ki == vi)
        {
            if (rel > 1e-6f) ++a6_d_diff;   // 同一个点，纯浮点结合顺序差异；>1e-6 才算异常
            return;
        }
        ++a6_idx_diff;
        const PointType &a = cloud.points[ki], &b = cloud.points[vi];
        if (a.x == b.x && a.y == b.y && a.z == b.z) { ++a6_dup; return; }
        if (rel <= 1e-6f) { ++a6_tie; return; }
        ++a6_bad;
        // 细分：这 2-10 个/帧 到底错在哪边
        if (kd > 25.f && vd > 25.f) { ++a6_gate_rej; return; }   // 两边都拒，无害
        if (rel > a6_gap) a6_gap = rel;
        if (vd < kd) ++a6_we_closer; else ++a6_kd_closer;
    }

    pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
    pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
    pcl::PointCloud<PointType>::Ptr surfPointsFlat;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
    pcl::PointCloud<PointType>::Ptr laserCloudFullRes;

    int laserCloudCornerLastNum;
    int laserCloudSurfLastNum;

    Eigen::Quaterniond q_w_curr = Eigen::Quaterniond(1, 0, 0, 0);
    Eigen::Vector3d t_w_curr = Eigen::Vector3d(0, 0, 0);

    double para_q[4];
    double para_t[3];
    Eigen::Map<Eigen::Quaterniond> q_last_curr{para_q};
    Eigen::Map<Eigen::Vector3d> t_last_curr{para_t};

    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> cornerSharpBuf;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> cornerLessSharpBuf;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> surfFlatBuf;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> surfLessFlatBuf;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> fullPointsBuf;

    std::mutex mBuf;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCornerPointsSharp;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCornerPointsLessSharp;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subSurfPointsFlat;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subSurfPointsLessFlat;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudFullRes;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudCornerLast;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurfLast;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubLaserPath;
    rclcpp::Publisher<lo_sam::msg::CloudInfo>::SharedPtr odom_cloud_pub;

    nav_msgs::msg::Path laserPath;

    int frameCount;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserOdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
