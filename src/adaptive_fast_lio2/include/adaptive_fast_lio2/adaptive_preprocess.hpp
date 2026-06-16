#pragma once

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "adaptive_fast_lio2/adaptive_common.hpp"



/**
 * @brief FAST-LIO2 风格点云预处理模块
 *
 * 这个类对应 FAST-LIO2 中的 Preprocess。
 *
 * 它负责：
 * 1. 接收原始 LiDAR 消息；
 * 2. 按雷达类型转换成统一 PointCloudXYZI；
 * 3. 做最基础的 blind 盲区过滤；
 * 4. 做 point_filter_num 降采样；
 * 5. 将每个点在一帧内的相对时间写入 curvature。
 *
 * 注意：
 * - 这里不是特征提取；
 * - feature_enabled 参数暂时保留，但默认 false；
 * - 后续若严格复现 FAST-LIO2，可继续补充更多雷达 handler。
 */

 class Preprocess
{
public:
    Preprocess() = default;

    /**
     * @brief 处理 Livox CustomMsg 点云
     *
     * 对应 FAST-LIO2 中 Livox / Avia 分支。
     */
    void process(
        const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg,
        PointCloudXYZI::Ptr &pcl_out);

    /**
     * @brief 处理标准 PointCloud2 点云
     *
     * 对应 FAST-LIO2 中非 Livox 雷达分支。
     */
    void process(
        const sensor_msgs::msg::PointCloud2::UniquePtr &msg,
        PointCloudXYZI::Ptr &pcl_out,
        double *lidar_beg_time_offset_sec = nullptr);

public:
    // ===================== 参数 =====================
    //
    // 这些参数在 adaptive_laserMapping.cpp 中从 yaml 读取，
    // 然后写入 p_pre。
    int lidar_type = AVIA;       // 雷达类型，取值见 LID_TYPE。
    int N_SCANS = 16;            // 允许处理的扫描线数量。
    int point_filter_num = 2;    // 预处理阶段每隔多少个有效点保留一个。
    int time_unit = US;          // 输入点内相对时间的原始单位。
    int SCAN_RATE = 10;          // LiDAR 扫描频率，单位：Hz。

    double blind = 0.01;         // 近距离盲区半径，单位：米。

    // FAST-LIO2 默认不启用传统特征提取。
    // 这里保留这个参数，是为了和 FAST-LIO2 参数结构一致。
    bool feature_enabled = false;

private:
    /**
     * @brief 判断点是否在 LiDAR 盲区外
     */
    bool isPointValid(double x, double y, double z) const;

    /**
     * @brief Livox 点云具体处理函数
     */
    void livoxHandler(
        const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg,
        PointCloudXYZI::Ptr &pcl_out);

    /**
     * @brief 标准 PointCloud2 具体处理函数
     */
    void standardHandler(
        const sensor_msgs::msg::PointCloud2::UniquePtr &msg,
        PointCloudXYZI::Ptr &pcl_out,
        double *lidar_beg_time_offset_sec);
};
