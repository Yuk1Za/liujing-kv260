#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <lo_sam/msg/feature_indices_raw.hpp>

#include <unordered_set>
#include <vector>
#include <cstdint>
#include <memory>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <deque>
#include <string>

using std::placeholders::_1;
using FeatureIndicesRaw = lo_sam::msg::FeatureIndicesRaw;

struct RawIndex {
    int16_t scan_id;
    int16_t column_id;
};

class FeatureCloudExtractor : public rclcpp::Node
{
public:
    FeatureCloudExtractor()
    : Node("feature_cloud_extractor")
    {
        // 参数声明与获取
        this->declare_parameter<int>("n_scans", 64);
        this->declare_parameter<int>("horizon_scan", 1800);
        this->get_parameter("n_scans", N_SCANS_);
        this->get_parameter("horizon_scan", HORIZON_SCAN_);

        // 订阅
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/feature/fpga_output_cloud", 10,
            std::bind(&FeatureCloudExtractor::cloud_callback, this, _1));

        sub_flat_ = this->create_subscription<FeatureIndicesRaw>(
            "/feature/fpga_output_flat", 10,
            std::bind(&FeatureCloudExtractor::flat_callback, this, _1));
        
        sub_sharp_ = this->create_subscription<FeatureIndicesRaw>(
            "/feature/fpga_output_sharp", 10,
            std::bind(&FeatureCloudExtractor::sharp_callback, this, _1));

        sub_lesssharp_ = this->create_subscription<FeatureIndicesRaw>(
            "/feature/fpga_output_lesssharp", 10,
            std::bind(&FeatureCloudExtractor::lesssharp_callback, this, _1));

        // 发布
        pub_cloud_full_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("feature/cloud_full", 100);
        pub_cloud_flat_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("feature/cloud_flat", 100);
        pub_cloud_sharp_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("feature/cloud_sharp", 100);
        pub_cloud_lesssharp_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("feature/cloud_less_sharp", 100);
        pub_cloud_lessflat_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("feature/cloud_less_flat", 100);

        RCLCPP_INFO(this->get_logger(),
            "FeatureCloudExtractor node initialized. N_SCANS=%d, HORIZON_SCAN=%d", N_SCANS_, HORIZON_SCAN_);

        // ---- 四路同步模式（bug #2）----
        //   fix   （默认）：四路时间戳不一致就不发，丢掉落后的那几路  ← 生产用
        //   detect：保持原行为（凑齐四路就发），但统计"发出去时四路戳不一致"的次数 ← 只用于量化问题
        //   off   ：完全原版行为，不统计
        const char *m = std::getenv("PD_SYNC");
        sync_mode_ = m ? std::string(m) : std::string("fix");
        if (sync_mode_ != "fix" && sync_mode_ != "detect" && sync_mode_ != "off") {
            RCLCPP_WARN(this->get_logger(), "PD_SYNC=%s 不认识，回退到 fix", sync_mode_.c_str());
            sync_mode_ = "fix";
        }
        RCLCPP_INFO(this->get_logger(), "四路同步模式 PD_SYNC=%s", sync_mode_.c_str());
    }

private:
    int N_SCANS_;
    int HORIZON_SCAN_;

    // ---- 四路同步（bug #2 修复）----
    using CloudPtr = sensor_msgs::msg::PointCloud2::SharedPtr;
    using RawPtr = FeatureIndicesRaw::SharedPtr;
    struct StampedCloud { int64_t s; CloudPtr m; };
    struct StampedRaw { int64_t s; RawPtr m; };

    static constexpr size_t QCAP = 4;          // 每路只留最近 4 帧，够跨过一次丢帧
    std::deque<StampedCloud> q_cloud_;
    std::deque<StampedRaw> q_flat_, q_sharp_, q_less_;

    std::string sync_mode_;
    long n_pub_ = 0, n_mismatch_ = 0, n_pub_mismatch_ = 0, n_stall_ = 0;
    long n_skip_cloud_ = 0, n_skip_flat_ = 0, n_skip_sharp_ = 0, n_skip_less_ = 0;

    static int64_t stamp_ns(const builtin_interfaces::msg::Time &t)
    {
        return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
    }

    template <typename Q>
    static void trim(Q &q)
    {
        while (q.size() > QCAP) { q.pop_front(); }
    }

    // 订阅缓存（旧版单槽位已移除，统一走 q_* 队列）

    // 订阅器
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<FeatureIndicesRaw>::SharedPtr sub_flat_, sub_sharp_, sub_lesssharp_;

    // 发布器
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_full_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_flat_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_sharp_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_lesssharp_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_lessflat_;

    // 校验索引合法性
    bool isValidIndex(int16_t scan_id, int16_t column_id) const
    {
        return (scan_id >= 0 && scan_id < N_SCANS_) &&
               (column_id >= 0 && column_id < HORIZON_SCAN_) &&
               ((scan_id + column_id) > 0);
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        q_cloud_.push_back({stamp_ns(msg->header.stamp), msg});
        trim(q_cloud_);
        try_publish();
    }

    void flat_callback(const FeatureIndicesRaw::SharedPtr msg)
    {
        q_flat_.push_back({stamp_ns(msg->header.stamp), msg});
        trim(q_flat_);
        try_publish();
    }

    void sharp_callback(const FeatureIndicesRaw::SharedPtr msg)
    {
        q_sharp_.push_back({stamp_ns(msg->header.stamp), msg});
        trim(q_sharp_);
        try_publish();
    }

    void lesssharp_callback(const FeatureIndicesRaw::SharedPtr msg)
    {
        q_less_.push_back({stamp_ns(msg->header.stamp), msg});
        trim(q_less_);
        try_publish();
    }

    // 真正发货：四路各自带上自己的 header（都是从同一帧云 header 复制的，所以互相一致）
    void emit(const CloudPtr &c, const RawPtr &f, const RawPtr &s, const RawPtr &l)
    {
        pub_cloud_full_->publish(*c);

        auto flat_indices = parse_raw_indices(*f);
        auto sharp_indices = parse_raw_indices(*s);
        auto lesssharp_indices = parse_raw_indices(*l);

        auto flat_cloud = extract_feature_cloud(*c, flat_indices);
        auto sharp_cloud = extract_feature_cloud(*c, sharp_indices);
        auto lesssharp_cloud = extract_feature_cloud(*c, lesssharp_indices);

        pub_cloud_flat_->publish(flat_cloud);
        pub_cloud_sharp_->publish(sharp_cloud);
        pub_cloud_lesssharp_->publish(lesssharp_cloud);

        auto lessflat_cloud = generate_lessflat_cloud(*c, flat_indices, sharp_indices, lesssharp_indices);
        pub_cloud_lessflat_->publish(lessflat_cloud);

        ++n_pub_;
        if (n_pub_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "[SYNC] pub=%ld mismatch=%ld pub_mismatch=%ld stall=%ld skip=%ld/%ld/%ld/%ld",
                n_pub_, n_mismatch_, n_pub_mismatch_, n_stall_,
                n_skip_cloud_, n_skip_flat_, n_skip_sharp_, n_skip_less_);
        }
    }

    // 原版行为：凑齐四路就发（不比对时间戳）。同步模式下不调用，只用于量化问题。
    void try_publish_legacy()
    {
        if (q_cloud_.empty() || q_flat_.empty() || q_sharp_.empty() || q_less_.empty()) {
            return;
        }
        const int64_t sc = q_cloud_.back().s, sf = q_flat_.back().s;
        const int64_t ss = q_sharp_.back().s, sl = q_less_.back().s;
        if (!(sc == sf && sc == ss && sc == sl)) {
            ++n_mismatch_;
            ++n_pub_mismatch_;
            if (n_pub_mismatch_ <= 5 || n_pub_mismatch_ % 50 == 0) {
                RCLCPP_WARN(this->get_logger(),
                    "[SYNC] ★ 原版行为发出错配帧（第 %ld 次）：cloud=%ld flat=%ld sharp=%ld less=%ld ns",
                    n_pub_mismatch_, (long)sc, (long)sf, (long)ss, (long)sl);
            }
        }
        emit(q_cloud_.back().m, q_flat_.back().m, q_sharp_.back().m, q_less_.back().m);
        q_cloud_.clear(); q_flat_.clear(); q_sharp_.clear(); q_less_.clear();
    }

    // 同步模式：每路留最近 QCAP 帧，取"四路都存在的**最新**那个戳"发出去；
    // 找不到就不发（等下一帧），发完把 ≤ 该戳的全部清掉。丢一帧能自愈，越位不会串帧。
    void try_publish()
    {
        if (sync_mode_ != "fix") { try_publish_legacy(); return; }

        if (q_cloud_.empty() || q_flat_.empty() || q_sharp_.empty() || q_less_.empty()) {
            return;
        }

        bool hit = false;
        int64_t S = 0;
        size_t ic = 0, i_f = 0, i_s = 0, i_l = 0;
        for (size_t a = q_cloud_.size(); a-- > 0; ) {          // 从最新往回找
            const int64_t s = q_cloud_[a].s;
            bool ok = false;
            size_t bf = 0, bs = 0, bl = 0;
            for (size_t b = 0; b < q_flat_.size(); ++b) { if (q_flat_[b].s == s) { ok = true; bf = b; break; } }
            if (!ok) { continue; }
            ok = false;
            for (size_t b = 0; b < q_sharp_.size(); ++b) { if (q_sharp_[b].s == s) { ok = true; bs = b; break; } }
            if (!ok) { continue; }
            ok = false;
            for (size_t b = 0; b < q_less_.size(); ++b) { if (q_less_[b].s == s) { ok = true; bl = b; break; } }
            if (!ok) { continue; }
            hit = true; S = s; ic = a; i_f = bf; i_s = bs; i_l = bl;
            break;
        }

        if (!hit) {
            // 四路没有公共戳：说明有路丢了这一帧。若某路明显落后/超前，记账并丢它，避免永久卡死。
            ++n_stall_;
            const int64_t newest = std::max(std::max(q_cloud_.back().s, q_flat_.back().s),
                                            std::max(q_sharp_.back().s, q_less_.back().s));
            auto drop_old = [&](auto &q, long &cnt) {
                while (!q.empty() && q.front().s < newest) { q.pop_front(); ++cnt; }
            };
            drop_old(q_flat_, n_skip_flat_);
            drop_old(q_sharp_, n_skip_sharp_);
            drop_old(q_less_, n_skip_less_);
            while (!q_cloud_.empty() && q_cloud_.front().s < newest) { q_cloud_.pop_front(); ++n_skip_cloud_; }
            if (n_stall_ <= 5 || n_stall_ % 50 == 0) {
                RCLCPP_WARN(this->get_logger(),
                    "[SYNC] 四路无公共时间戳（第 %ld 次）：cloud=%zu flat=%zu sharp=%zu less=%zu 条缓存，已丢落后项 %ld/%ld/%ld/%ld",
                    n_stall_, q_cloud_.size(), q_flat_.size(), q_sharp_.size(), q_less_.size(),
                    n_skip_cloud_, n_skip_flat_, n_skip_sharp_, n_skip_less_);
            }
            return;
        }

        // 队列长度 >1 说明某一路来晚过（否则每次配完都会清空）⇒ 只在此时记一次"缓存曾失步"
        if (q_cloud_.size() > 1 || q_flat_.size() > 1 || q_sharp_.size() > 1 || q_less_.size() > 1) {
            ++n_mismatch_;
        }
        emit(q_cloud_[ic].m, q_flat_[i_f].m, q_sharp_[i_s].m, q_less_[i_l].m);
        while (!q_cloud_.empty() && q_cloud_.front().s <= S) { q_cloud_.pop_front(); }
        while (!q_flat_.empty() && q_flat_.front().s <= S) { q_flat_.pop_front(); }
        while (!q_sharp_.empty() && q_sharp_.front().s <= S) { q_sharp_.pop_front(); }
        while (!q_less_.empty() && q_less_.front().s <= S) { q_less_.pop_front(); }
    }

    // 解析二进制数据为索引列表
    std::vector<RawIndex> parse_raw_indices(const FeatureIndicesRaw &msg) const
    {
        std::vector<RawIndex> result;
        size_t count = msg.count;
        if (msg.data.size() < count * 4) {
            RCLCPP_WARN(this->get_logger(), "FeatureIndicesRaw data size too small for count");
            return result;
        }

        result.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            int16_t scan_id = 0;
            int16_t column_id = 0;
            // 低位字节序假设和系统一致，若有不同需改写
            std::memcpy(&scan_id, &msg.data[i * 4], 2);
            std::memcpy(&column_id, &msg.data[i * 4 + 2], 2);
            result.push_back({scan_id, column_id});
        }
        return result;
    }

    // 根据特征索引提取点云
    sensor_msgs::msg::PointCloud2 extract_feature_cloud(
        const sensor_msgs::msg::PointCloud2 &cloud_msg,
        const std::vector<RawIndex> &indices)
    {
        std::unordered_set<size_t> idx_set;
        for (const auto &feature : indices) {
            int16_t scan_id = feature.scan_id;
            int16_t column_id = feature.column_id;
            if (!isValidIndex(scan_id, column_id)) continue;
            size_t idx = static_cast<size_t>(column_id) * N_SCANS_ + scan_id;
            idx_set.insert(idx);
        }

        sensor_msgs::msg::PointCloud2 out_cloud;
        out_cloud.header = cloud_msg.header;
        out_cloud.height = 1;
        out_cloud.is_dense = cloud_msg.is_dense;
        out_cloud.is_bigendian = cloud_msg.is_bigendian;
        out_cloud.fields = cloud_msg.fields;
        out_cloud.point_step = cloud_msg.point_step;

        std::vector<uint8_t> data_out;
        data_out.reserve(idx_set.size() * cloud_msg.point_step);

        for (size_t idx : idx_set) {
            if (idx >= static_cast<size_t>(cloud_msg.width)) continue; // 安全检查
            const uint8_t *ptr = &cloud_msg.data[idx * cloud_msg.point_step];
            data_out.insert(data_out.end(), ptr, ptr + cloud_msg.point_step);
        }

        out_cloud.data = std::move(data_out);
        out_cloud.width = static_cast<uint32_t>(out_cloud.data.size() / out_cloud.point_step);
        out_cloud.row_step = out_cloud.width * out_cloud.point_step;

        return out_cloud;
    }

    // 生成 less_flat 点云，参考 FPGA 逻辑
    sensor_msgs::msg::PointCloud2 generate_lessflat_cloud(
        const sensor_msgs::msg::PointCloud2 &cloud_msg,
        const std::vector<RawIndex> &flat_indices,
        const std::vector<RawIndex> &sharp_indices,
        const std::vector<RawIndex> &lesssharp_indices)
    {
        std::unordered_set<size_t> corner_set;
        for (const auto &idx : sharp_indices) {
            if (isValidIndex(idx.scan_id, idx.column_id)) {
                corner_set.insert(static_cast<size_t>(idx.column_id) * N_SCANS_ + idx.scan_id);
            }
        }
        for (const auto &idx : lesssharp_indices) {
            if (isValidIndex(idx.scan_id, idx.column_id)) {
                corner_set.insert(static_cast<size_t>(idx.column_id) * N_SCANS_ + idx.scan_id);
            }
        }

        std::unordered_set<size_t> flat_set;
        for (const auto &idx : flat_indices) {
            if (isValidIndex(idx.scan_id, idx.column_id)) {
                flat_set.insert(static_cast<size_t>(idx.column_id) * N_SCANS_ + idx.scan_id);
            }
        }

        std::vector<uint8_t> data_out;
        data_out.reserve(cloud_msg.data.size());

        const uint32_t width = cloud_msg.width;
        for (size_t idx = 0; idx < static_cast<size_t>(width); ++idx) {
            int scan_id = static_cast<int>(idx % N_SCANS_);
            int column_id = static_cast<int>(idx / N_SCANS_);

            bool in_middle = (column_id >= 5) && (column_id < (HORIZON_SCAN_ - 6));
            if (!in_middle) continue;

            if (corner_set.count(idx) > 0) continue;
            if (flat_set.count(idx) > 0) continue;

            const uint8_t *ptr = &cloud_msg.data[idx * cloud_msg.point_step];
            data_out.insert(data_out.end(), ptr, ptr + cloud_msg.point_step);
        }

        sensor_msgs::msg::PointCloud2 out_cloud;
        out_cloud.header = cloud_msg.header;
        out_cloud.height = 1;
        out_cloud.width = static_cast<uint32_t>(data_out.size() / cloud_msg.point_step);
        out_cloud.row_step = out_cloud.width * cloud_msg.point_step;
        out_cloud.is_dense = cloud_msg.is_dense;
        out_cloud.is_bigendian = cloud_msg.is_bigendian;
        out_cloud.fields = cloud_msg.fields;
        out_cloud.point_step = cloud_msg.point_step;
        out_cloud.data = std::move(data_out);

        return out_cloud;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureCloudExtractor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
