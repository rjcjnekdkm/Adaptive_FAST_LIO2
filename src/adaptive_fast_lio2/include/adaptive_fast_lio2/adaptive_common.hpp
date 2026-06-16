#pragma once

#include <vector>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/msg/imu.hpp>

using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<PointType>;

// 支持的 LiDAR 类型编号，与 YAML 中 preprocess.lidar_type 对应。
enum LID_TYPE
{
    AVIA = 1,
    VELO16 = 2,
    OUST64 = 3,
    MID360 = 4
};

// 点内相对时间的原始单位，与 YAML 中 preprocess.timestamp_unit 对应。
enum TIME_UNIT
{
    SEC = 0,
    MS = 1,
    US = 2,
    NS = 3
};

struct MeasureGroup
{
    // 当前 LiDAR 帧的起止时间，单位：秒。
    double lidar_beg_time = 0.0;
    double lidar_end_time = 0.0;

    // 当前帧点云，以及覆盖该帧时间区间的 IMU 消息序列。
    PointCloudXYZI::Ptr lidar;
    std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu;
};
