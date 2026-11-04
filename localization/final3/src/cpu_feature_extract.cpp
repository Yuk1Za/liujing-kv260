// ============================================================================
// cpu_feature_extract —— CPU 版特征提取（替代 pynq_feature_extraction.py + PLdataprocess）
//
// 逻辑严格对照 hardware/fpga_feature_extract_hw.cpp 的真实 dataflow 链：
//   allocate_rangeimage → compute_curvature → compute_ground → extract_features
//   （文件顶部的 *_to_output 系列是死代码，未参照）
//
// 与 FPGA 版的差异（有意为之，见 BOARDING-PREP.md §2）：
//   1. 直接发布 feature/cloud_{full,sharp,less_sharp,flat,less_flat}，
//      省去 FeatureIndicesRaw 索引重构（PLdataprocess 的 64 线硬编码不适用 16 线）；
//   2. 只发布有点的像元（FPGA 会把空像元置零发布）；
//   3. 曲率窗口要求 11 个像元全部有点，否则该点 picked=1（FPGA 的零填充会引入假曲率）。
//
// 订阅: feature/reordered_cloud (lo_sam/msg/LidarClouds)
// 发布: feature/cloud_full / cloud_sharp / cloud_less_sharp / cloud_flat / cloud_less_flat
// ============================================================================
#include <chrono>
#include <cmath>
#include <cstring>
#include <vector>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lo_sam/msg/lidar_clouds.hpp"

using LidarClouds = lo_sam::msg::LidarClouds;

static constexpr int   HORIZON        = 1800;
static constexpr int   FLAT_SIZE      = 4;
static constexpr int   SHARP_SIZE     = 2;
static constexpr int   LESS_SHARP     = 20;
static constexpr int   N_SEG          = 6;      // EXTRACT_SEGMENT
static constexpr float SCAN_PERIOD    = 0.1f;

// ---- registration_fpga.h 的硬件常量（原样搬运） ----
static constexpr float GROUND_MIN_ANGLE   = 10.0f;  // deg
static constexpr float VERTICAL_PLANE_MIN = 60.0f;  // deg
static constexpr float GROUND_H_THRESHOLD = 0.6f;   // m
// lidar_height 改为参数（KITTI=2.0，C16 上按实测改）

struct Cell {
    float x{0}, y{0}, z{0}, intensity{0}, range{0};
    bool  occupied{false};
};

struct FeatEntry {
    float   curvature;
    int16_t column;
    bool    valid;
};

class CpuFeatureExtractNode : public rclcpp::Node
{
public:
    CpuFeatureExtractNode() : Node("cpu_feature_extract")
    {
        n_scans_     = this->declare_parameter<int>("N_SCAN", 16);
        horizon_     = this->declare_parameter<int>("Horizon_SCAN", HORIZON);
        lidar_height_ = this->declare_parameter<double>("lidar_height", 2.0);
        ground_mode_ = this->declare_parameter<int>("ground_mode", 1);
        scan_lower_  = this->declare_parameter<double>("scan_lower_bound_deg", -15.0);
        scan_res_    = this->declare_parameter<double>("scan_resolution_deg", 2.0);
        min_range_   = this->declare_parameter<double>("minimum_range", 1.0);

        pub_full_      = create_publisher<sensor_msgs::msg::PointCloud2>("feature/cloud_full", 100);
        pub_sharp_     = create_publisher<sensor_msgs::msg::PointCloud2>("feature/cloud_sharp", 100);
        pub_less_sharp_ = create_publisher<sensor_msgs::msg::PointCloud2>("feature/cloud_less_sharp", 100);
        pub_flat_      = create_publisher<sensor_msgs::msg::PointCloud2>("feature/cloud_flat", 100);
        pub_less_flat_ = create_publisher<sensor_msgs::msg::PointCloud2>("feature/cloud_less_flat", 100);

        sub_ = create_subscription<LidarClouds>(
            "feature/reordered_cloud", 100,
            [this](LidarClouds::SharedPtr msg) { callback(std::move(msg)); });

        RCLCPP_INFO(get_logger(),
            "cpu_feature_extract ready: N_SCAN=%d Horizon=%d height=%.2f",
            n_scans_, horizon_, lidar_height_);
    }

private:
    // ---------------- 像元与角度工具 ----------------
    int findScanId(float x, float y, float z) const
    {
        const float horiz = std::sqrt(x * x + y * y);
        if (horiz < 1e-6f && std::fabs(z) < 1e-6f) return -1;
        const float ang = std::atan2(z, horiz) * 180.0f / M_PI;   // deg
        int id = static_cast<int>(std::round((ang - scan_lower_) / scan_res_));
        if (id < 0 || id >= n_scans_) return -1;
        return id;
    }

    static int findColumnId(float x, float y, int horizon)
    {
        float ang = std::atan2(y, x) + static_cast<float>(M_PI);
        int c = static_cast<int>(ang / (2.0f * static_cast<float>(M_PI)) * horizon);
        if (c >= horizon) c = horizon - 1;
        if (c < 0) c = 0;
        return c;
    }

    // ---------------- 主回调 ----------------
    void callback(LidarClouds::SharedPtr msg)
    {
        auto _t0 = std::chrono::steady_clock::now();
        const int H = horizon_;
        std::vector<Cell> img(static_cast<size_t>(n_scans_) * H);

        // ---- 1. 投影到 range image（列主序输入，与 FPGA 相同规则：先到先得）----
        sensor_msgs::PointCloud2ConstIterator<float> ix(msg->lidar_clouds, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iy(msg->lidar_clouds, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iz(msg->lidar_clouds, "z");
        sensor_msgs::PointCloud2ConstIterator<float> ii(msg->lidar_clouds, "intensity");
        for (; ix != ix.end(); ++ix, ++iy, ++iz, ++ii) {
            const float x = *ix, y = *iy, z = *iz;
            const float range = std::sqrt(x * x + y * y + z * z);
            if (range < min_range_) continue;
            const int c = findColumnId(x, y, H);
            const int s = findScanId(x, y, z);
            if (s < 0) continue;
            Cell &cell = img[static_cast<size_t>(c) * n_scans_ + s];
            if (cell.occupied) continue;               // overlay：先到先得
            cell.x = x; cell.y = y; cell.z = z;
            cell.range = range;
            cell.occupied = true;
        }

        // ---- 2. intensity = scanID + scanPeriod * relTime（LOAM 惯例，FPGA 同）----
        for (int c = 0; c < H; ++c)
            for (int s = 0; s < n_scans_; ++s) {
                Cell &cell = img[static_cast<size_t>(c) * n_scans_ + s];
                if (cell.occupied)
                    cell.intensity = static_cast<float>(s)
                                   + SCAN_PERIOD * (static_cast<float>(c) / H);
            }

        // ---- 3. 曲率（LOAM ±5 窗口）----
        std::vector<float> curv(img.size(), 0.0f);
        std::vector<uint8_t> picked(img.size(), 0);
        for (int c = 5; c < H - 5; ++c)
            for (int s = 0; s < n_scans_; ++s) {
                bool ok = true;
                float dx = 0, dy = 0, dz = 0;
                for (int k = -5; k <= 5; ++k) {
                    const Cell &p = img[static_cast<size_t>(c + k) * n_scans_ + s];
                    if (!p.occupied) { ok = false; break; }
                    dx += p.x; dy += p.y; dz += p.z;
                }
                if (!ok) { picked[idx(c, s)] = 1; continue; }   // 窗口不完整：排除
                const Cell &pc = img[idx(c, s)];
                dx -= 10.0f * pc.x; dy -= 10.0f * pc.y; dz -= 10.0f * pc.z;
                curv[idx(c, s)] = dx * dx + dy * dy + dz * dz;
            }

        // ---- 3b. 深度不连续 picked（FPGA compute_curvature 的 ±1% 规则，索引平移到批处理）----
        for (int c = 6; c < H - 5; ++c)
            for (int s = 0; s < n_scans_; ++s) {
                const float r1 = img[idx(c - 1, s)].range;   // FPGA: depth1 = r[c-6]
                const float r2 = img[idx(c, s)].range;       //        depth2 = r[c-5]
                if (!img[idx(c - 1, s)].occupied || !img[idx(c, s)].occupied) continue;
                if (r1 > 2.0f && r2 > 2.0f) {
                    if (r1 - r2 > r1 * 0.01f)
                        for (int k = -5; k <= 0; ++k) picked[idx(c - 1 + k, s)] = 1;
                    else if (r2 - r1 > r1 * 0.01f)
                        for (int k = 0; k <= 5; ++k) picked[idx(c - 1 + k, s)] = 1;
                }
            }

        // ---- 4. 地面判据（FPGA compute_ground：同 scan 相邻列坡度 + 垂直平面点）----
        // ground: 0=非地面 1=地面 2=垂直平面点
        std::vector<uint8_t> ground(img.size(), 0);
        std::vector<int> ground_count(n_scans_, 0);   // FPGA: 每帧重置
        for (int s = 0; s < n_scans_; ++s) {
            float avg_h = -static_cast<float>(lidar_height_);   // 运行均值，初始 = -height
            for (int c = 1; c < H; ++c) {
                const Cell &p0 = img[idx(c - 1, s)], &p1 = img[idx(c, s)];
                if (!p0.occupied || !p1.occupied) continue;
                const float dz = p1.z - p0.z;
                const float dh = std::sqrt((p1.x - p0.x) * (p1.x - p0.x)
                                         + (p1.y - p0.y) * (p1.y - p0.y));
                const float ang = (dh > 0.0f)
                    ? std::atan(dz / dh) * 180.0f / M_PI : 1e7f;

                if (p0.range >= 0.0f && std::fabs(ang) <= GROUND_MIN_ANGLE) {
                    bool is_ground = false;
                    if (c - 1 < 3 || s <= 4)
                        is_ground = std::fabs(p0.z + lidar_height_) < GROUND_H_THRESHOLD;
                    else
                        is_ground = std::fabs(p0.z - avg_h) < GROUND_H_THRESHOLD;
                    if (is_ground) {
                        ground[idx(c - 1, s)] = 1;
                        avg_h = (avg_h * ground_count[s] + p0.z) / (ground_count[s] + 1);
                        ground_count[s]++;
                    }
                }
                // 垂直平面点（大角度 + 低曲率），覆盖 ground=1
                if (s < n_scans_ - 1
                    && std::fabs(ang) >= VERTICAL_PLANE_MIN && curv[idx(c - 1, s)] <= 5.0f)
                    ground[idx(c - 1, s)] = 2;
            }
        }

        // ---- 5. 特征挑选（FPGA extract_features：6 段，每段每 scan Top-K）----
        // sharp 槽位 = 20 个角点（降序），其中前 2 个作为 sharp 发布、全部 20 个作为 less_sharp 发布；
        // flat 槽位 = 4 个平面点（升序）。插入语义与 FPGA 移位插入等价。
        const int seg_len = H / N_SEG;
        std::vector<uint8_t> is_corner(img.size(), 0), is_top_sharp(img.size(), 0), is_flat(img.size(), 0);

        const auto trySharp = [&](int s, int c, float cv, float range,
                                  std::vector<FeatEntry> &corner) {
            if (picked[idx(c, s)] || range <= 1.0f) return;
            float th = 1.0f;
            if (range >= 2 && range < 20)       th = 1.0f;
            else if (range >= 20 && range < 50)  th = 0.5f;
            else if (range >= 50 && range < 80)  th = 0.25f;
            else if (range >= 80 && range < 120) th = 0.125f;
            if (cv <= th) return;
            if (ground[idx(c, s)] == 1 || ground[idx(c, s)] == 2) return;

            // 近邻抑制：与已有条目 column 距离 <=5 → 原位替换（若更尖）
            bool near = false;
            for (auto &e : corner)
                if (std::abs(c - e.column) <= 5) { near = true; break; }
            if (near) {
                for (auto &e : corner)
                    if (std::abs(c - e.column) <= 5 && cv > e.curvature)
                        e = {cv, static_cast<int16_t>(c), true};
                return;
            }
            // 降序有序插入，超长截断（等价于 FPGA 的移位插入）
            for (auto it = corner.begin(); it != corner.end(); ++it) {
                if (cv > it->curvature) {
                    corner.insert(it, {cv, static_cast<int16_t>(c), true});
                    if (corner.size() > static_cast<size_t>(LESS_SHARP)) corner.pop_back();
                    return;
                }
            }
            if (corner.size() < static_cast<size_t>(LESS_SHARP))
                corner.push_back({cv, static_cast<int16_t>(c), true});
        };
        const auto tryFlat = [&](int s, int c, float cv, float range,
                                 std::vector<FeatEntry> &flatv) {
            if (picked[idx(c, s)] || range <= 1.0f) return;
            if (cv >= 0.1f) return;
            // ground_mode=1：flat 只从 ground==1/2 选（对应 PL 的 compute_ground IP）
            // ground_mode=0：无 ground IP，改用 LOAM 原版「低曲率即平面候选」
            if (ground_mode_ == 1 && ground[idx(c, s)] != 1 && ground[idx(c, s)] != 2) return;
            bool near = false;
            for (auto &e : flatv)
                if (std::abs(c - e.column) <= 5) { near = true; break; }
            if (near) {
                for (auto &e : flatv)
                    if (std::abs(c - e.column) <= 5 && cv < e.curvature)
                        e = {cv, static_cast<int16_t>(c), true};
                return;
            }
            for (auto it = flatv.begin(); it != flatv.end(); ++it) {
                if (cv < it->curvature) {
                    flatv.insert(it, {cv, static_cast<int16_t>(c), true});
                    if (flatv.size() > static_cast<size_t>(FLAT_SIZE)) flatv.pop_back();
                    return;
                }
            }
            if (flatv.size() < static_cast<size_t>(FLAT_SIZE))
                flatv.push_back({cv, static_cast<int16_t>(c), true});
        };

        for (int k = 0; k < N_SEG; ++k) {
            const int seg_begin = 5 + k * seg_len;
            const int seg_end   = (k == N_SEG - 1) ? (H - 7) : (4 + (k + 1) * seg_len);
            std::vector<std::vector<FeatEntry>> corner(n_scans_), flatv(n_scans_);
            for (int c = seg_begin; c < seg_end; ++c)
                for (int s = 0; s < n_scans_; ++s) {
                    const Cell &p = img[idx(c, s)];
                    if (!p.occupied) continue;
                    const float cv = curv[idx(c, s)];
                    trySharp(s, c, cv, p.range, corner[s]);
                    tryFlat(s, c, cv, p.range, flatv[s]);
                }
            // 段结束：标记特征像元（前 2 角点 = sharp；全部角点 = less_sharp）
            for (int s = 0; s < n_scans_; ++s) {
                for (size_t m = 0; m < corner[s].size(); ++m) {
                    is_corner[idx(corner[s][m].column, s)] = 1;
                    if (m < static_cast<size_t>(SHARP_SIZE))
                        is_top_sharp[idx(corner[s][m].column, s)] = 1;
                }
                for (const auto &e : flatv[s])
                    is_flat[idx(e.column, s)] = 1;
            }
        }

        // ---- 6. 发布五路特征点云 ----
        const auto &hdr = msg->header;
        pcl::PointCloud<pcl::PointXYZI> c_full, c_sharp, c_less_sharp, c_flat, c_less_flat;
        for (int c = 5; c < H - 6; ++c)
            for (int s = 0; s < n_scans_; ++s) {
                const Cell &p = img[idx(c, s)];
                if (!p.occupied) continue;
                const size_t i = idx(c, s);
                pcl::PointXYZI q;
                q.x = p.x; q.y = p.y; q.z = p.z; q.intensity = p.intensity;
                c_full.push_back(q);
                if (is_top_sharp[i]) c_sharp.push_back(q);
                if (is_corner[i])    c_less_sharp.push_back(q);
                if (is_flat[i])      c_flat.push_back(q);
                if (!is_corner[i] && !is_flat[i]) c_less_flat.push_back(q);
            }
        publish(pub_full_, hdr, c_full);
        publish(pub_sharp_, hdr, c_sharp);
        publish(pub_less_sharp_, hdr, c_less_sharp);
        publish(pub_flat_, hdr, c_flat);
        publish(pub_less_flat_, hdr, c_less_flat);

        {   // ★ CPU 版单帧耗时（与 PL 节点日志的“节点总耗时”同口径）
            auto _t1 = std::chrono::steady_clock::now();
            const double _ms = std::chrono::duration<double, std::milli>(_t1 - _t0).count();
            RCLCPP_INFO(get_logger(), [CPUFE] in=%u full=%zu flat=%zu sharp=%zu cost=%.1f ms,
                        msg->lidar_clouds.width, c_full.size(), c_flat.size(),
                        c_sharp.size(), _ms);
        }
    }

    static void publish(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub,
                        const std_msgs::msg::Header &hdr,
                        const pcl::PointCloud<pcl::PointXYZI> &cloud)
    {
        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(cloud, out);
        out.header = hdr;
        pub->publish(out);
    }

    size_t idx(int c, int s) const { return static_cast<size_t>(c) * n_scans_ + s; }

    int n_scans_, horizon_;
    int ground_mode_ = 1;   // 1=地面判据(现状) 0=LOAM原版低曲率判据
    double lidar_height_, scan_lower_, scan_res_, min_range_;
    rclcpp::Subscription<LidarClouds>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pub_full_, pub_sharp_, pub_less_sharp_, pub_flat_, pub_less_flat_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CpuFeatureExtractNode>());
    rclcpp::shutdown();
    return 0;
}
