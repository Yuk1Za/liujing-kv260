#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>
#include <string>
#include <iostream>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"

#include <nav_msgs/msg/odometry.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/impl/point_cloud2_iterator.hpp> 

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "lo_sam/msg/lidar_clouds.hpp"
using std::atan2;
using std::cos;
using std::sin;

const int N_SCANS = 64;
const int Horizon_SCAN = 1800;
const float MINIMUM_RANGE = 0.1f;
const int MAX_POINTS_PER_COLUMN = 150;

class FeatureExtractNode : public rclcpp::Node
{
public:
    FeatureExtractNode()
    : Node("scanRegistration")
    {
        // 声明参数，读取参数
        this->declare_parameter<int>("N_SCAN", 16);
        this->declare_parameter<std::string>("lidar_type", "VLP16");
        this->declare_parameter<std::string>("lidar_topic", "velodyne_points");
        this->declare_parameter<double>("minimum_range", 0.5);

        this->get_parameter("N_SCAN", N_SCANS);
        this->get_parameter("lidar_type", LIDAR_TYPE);
        this->get_parameter("lidar_topic", LIDAR_TOPIC);
        this->get_parameter("minimum_range", MINIMUM_RANGE);

        if (N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
        {
            RCLCPP_ERROR(this->get_logger(), "only support VLP16, HDL32, HDL64, OS1-32, OS1-64");
            rclcpp::shutdown();
            return;
        }

        // 发布话题
        reorderedPointCloudsPublisher = this->create_publisher<lo_sam::msg::LidarClouds>("feature/reordered_cloud", 2);

        // 订阅点云话题
        subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            LIDAR_TOPIC, rclcpp::QoS(2).reliable(),
            std::bind(&FeatureExtractNode::laserCloudHandler, this, std::placeholders::_1));

        systemInitCount = 0;
        systemInited = false;
    }

private:
    std::string LIDAR_TYPE;
    std::string LIDAR_TOPIC;

    const double scanPeriod = 0.1;
    const int systemDelay = 0;

    int systemInitCount;
    bool systemInited;
    int N_SCANS;
    float MINIMUM_RANGE;

    float cloudCurvature[400000];
    int cloudSortInd[400000];
    int cloudNeighborPicked[400000];
    int cloudLabel[400000];

    rclcpp::Publisher<lo_sam::msg::LidarClouds>::SharedPtr reorderedPointCloudsPublisher;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;

    bool PUB_EACH_LINE = false;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pubEachScan;

    template <typename PointT>
    void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                                pcl::PointCloud<PointT> &cloud_out, float thres)
    {
        if (&cloud_in != &cloud_out)
        {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }

        size_t j = 0;

        for (size_t i = 0; i < cloud_in.points.size(); ++i)
        {
            if (cloud_in.points[i].x * cloud_in.points[i].x +
                cloud_in.points[i].y * cloud_in.points[i].y +
                cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
                continue;
            cloud_out.points[j] = cloud_in.points[i];
            j++;
        }
        if (j != cloud_in.points.size())
        {
            cloud_out.points.resize(j);
        }

        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
    }

    static bool comp(int i, int j, float *cloudCurvature)
    {
        return (cloudCurvature[i] < cloudCurvature[j]);
    }

    int find_columnid(const pcl::PointXYZI& p) {
        // atan2(y, x) 范围是 -PI 到 PI
        float angle = atan2(p.y, p.x) + M_PI;
        // 计算 columnID
        // columnID = angle / (2 * PI) * Horizon_SCAN
        int columnIdn = static_cast<int>(angle / (2 * M_PI) * Horizon_SCAN);

        // 边界检查
        if (columnIdn >= Horizon_SCAN) columnIdn = Horizon_SCAN - 1;
        if (columnIdn < 0) columnIdn = 0;

        return columnIdn;
    }

    // vidx2: C16 仰角分档。实测 16 线 -14.93°~+15.01°，每 2° 一条。
    // 旧版 HLS 的 scanid_lut 是 ±0.0622 非均匀表，不是 C16 的（以前 scan 只落到 0~8）。
    int find_scanid_c16(const pcl::PointXYZI& p) {
        const float rr = sqrtf(p.x * p.x + p.y * p.y);
        if (rr < 1e-6f) return 0;
        const float elev_deg = atan2f(p.z, rr) * 57.29577951308232f;
        int line = (int)lroundf((elev_deg + 15.0f) * 0.5f);
        if (line < 0) line = 0;
        if (line > 15) line = 15;
        return line;
    }

    void laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
    {
        // 配置参数
        std::vector<std::vector<pcl::PointXYZI>> laserCloudInArray_temp(Horizon_SCAN);
        int laserCloudInArray_temp_count[Horizon_SCAN];

        for (int i = 0; i < Horizon_SCAN; ++i)
            laserCloudInArray_temp[i].reserve(MAX_POINTS_PER_COLUMN); 
        // 转换为PCL点云
        pcl::PointCloud<pcl::PointXYZI> laserCloudIn_pcl;
        pcl::fromROSMsg(*laserCloudMsg, laserCloudIn_pcl);

        if (laserCloudIn_pcl.empty())
            return;

        int cloudSize = laserCloudIn_pcl.size();
        bool capacityExceeded = false;

        for (int i = 0; i < cloudSize; i++)
        {
            pcl::PointXYZI p = laserCloudIn_pcl.points[i];

            // 距离过滤
            float range = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (!std::isfinite(range) || range < MINIMUM_RANGE || range >= 125 || std::abs(p.x)>=125 || std::abs(p.y)>=125 || std::abs(p.z)>=125)
                continue;

            // 计算 columnIdn
            int columnIdn = find_columnid(p);

            // 存储点到临时数组
            if (columnIdn >= 0 && columnIdn < Horizon_SCAN) {
                if (laserCloudInArray_temp[columnIdn].size() < MAX_POINTS_PER_COLUMN) {
                    laserCloudInArray_temp[columnIdn].push_back(p);
                } else { capacityExceeded = true; }
            }
        }

        cloudSize = 0;
        for (const auto &column : laserCloudInArray_temp) cloudSize += column.size();
        if (capacityExceeded || cloudSize < 2 || cloudSize > 115199) {
            RCLCPP_WARN(get_logger(), "Dropped complete frame: invalid count or column capacity (%d)",cloudSize); return;
        }
        // 1. 创建新的组合消息
        lo_sam::msg::LidarClouds combinedMsg;
        combinedMsg.header = laserCloudMsg->header;
        combinedMsg.cloud_size = cloudSize;

        sensor_msgs::msg::PointCloud2& cloud_msg = combinedMsg.lidar_clouds;
        
        cloud_msg.header = laserCloudMsg->header;
        cloud_msg.height = 1;
        cloud_msg.width = cloudSize;
        cloud_msg.is_dense = true;
        cloud_msg.is_bigendian = false;
        
        // 2. 设置 PointCloud2 的字段
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(4, 
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        
        modifier.resize(cloudSize);

        // 3. 填充数据 (直接操作内存，避免 PCL 的 Padding)
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_i(cloud_msg, "intensity");

        for (int i = 0; i < Horizon_SCAN; ++i)
        {
            for (size_t j = 0; j < laserCloudInArray_temp[i].size(); ++j)
            {
                const auto& p = laserCloudInArray_temp[i][j];
                
                *iter_x = p.x;
                *iter_y = p.y;
                *iter_z = p.z;
                // vidx2: intensity 在 IP 内部会被覆盖成 scanID+relTime，输入侧是死值。
                // 这里承载 (scan<<11)|column：HLS 直接拆位，省掉 find_scanid/find_columnid。
                // 0x3F800000 打底 ⇒ 规格化数，规避非规格化被 FTZ 的风险。
                const uint32_t _idx32 = 0x3F800000u
                    | ((uint32_t)find_scanid_c16(p) << 11)
                    | (uint32_t)(i & 0x7FF);
                float _fidx;
                std::memcpy(&_fidx, &_idx32, sizeof(_fidx));
                *iter_i = _fidx;

                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++iter_i;
            }
        }

        // 4. 复制 header，确保时间戳等信息准确
        combinedMsg.lidar_clouds.header = laserCloudMsg->header;

        float startOri = 0.0;
        float endOri = 0.0;

        if (cloudSize > 0) {
            // 第一个点的角度
            startOri = -atan2(laserCloudIn_pcl.points[0].y, laserCloudIn_pcl.points[0].x);

            // 最后一个点的角度
            endOri = -atan2(laserCloudIn_pcl.points.back().y, laserCloudIn_pcl.points.back().x) + 2*M_PI;

            // 确保 endOri 在 startOri 之后，处理跨越 PI/-PI 边界的情况
            if (endOri - startOri > 3 * M_PI) {
                endOri -= 2 * M_PI;
            } else if (endOri - startOri < M_PI) {
                endOri += 2 * M_PI;
            }
        }

        combinedMsg.start_ori = startOri;
        combinedMsg.end_ori = endOri;

        // 5. 发布消息
        reorderedPointCloudsPublisher->publish(combinedMsg);
    }  
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureExtractNode>();
    RCLCPP_INFO(rclcpp::get_logger("FeatureExtract"), "----> Feature Extraction node started.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
