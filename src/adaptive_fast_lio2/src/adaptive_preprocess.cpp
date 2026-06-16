#include "adaptive_fast_lio2/adaptive_preprocess.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <pcl_conversions/pcl_conversions.h>

/**
 * @brief 判断点是否有效
 *
 * FAST-LIO2 中 blind 表示 LiDAR 近距离盲区。
 * 过近的点通常噪声大，且对定位约束帮助有限，所以预处理时直接滤掉。
 */
bool Preprocess::isPointValid(double x, double y, double z) const
{
    const double range2 = x * x + y * y + z * z;
    return range2 > blind * blind;
}


/**
 * @brief 统一处理 Livox CustomMsg
 *
 * 当前只实现 Livox / Avia 分支。
 * 后续如果你使用 MID360，也可以在这里继续扩展 mid360Handler。
 */
void Preprocess::process(
    const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg,
    PointCloudXYZI::Ptr &pcl_out)
{
    if (pcl_out == nullptr)
    {
        pcl_out.reset(new PointCloudXYZI());
    }

    pcl_out->clear();

    if (msg == nullptr || msg->point_num == 0)
    {
        return;
    }

    livoxHandler(msg, pcl_out);
}


/**
 * @brief 统一处理标准 PointCloud2
 */
void Preprocess::process(
    const sensor_msgs::msg::PointCloud2::UniquePtr &msg,
    PointCloudXYZI::Ptr &pcl_out,
    double *lidar_beg_time_offset_sec)
{
    if (pcl_out == nullptr)
    {
        pcl_out.reset(new PointCloudXYZI());
    }

    pcl_out->clear();

    if (msg == nullptr)
    {
        return;
    }

    standardHandler(msg, pcl_out, lidar_beg_time_offset_sec);
}


/**
 * @brief Livox 点云预处理
 *
 * 这部分对应 FAST-LIO2 中 Livox / Avia 点云 handler 的核心逻辑。
 *
 * 当前实现步骤：
 * 1. 按扫描线 line 过滤；
 * 2. 按 tag 过滤回波类型；
 * 3. 使用 point_filter_num 做简单降采样；
 * 4. 使用 blind 过滤近距离点；
 * 5. 去掉连续重复点；
 * 6. 将 Livox offset_time 写入 curvature。
 *
 * 说明：
 * - 这里没有做 LOAM 式边缘/平面特征提取；
 * - curvature 在这里不是曲率，而是点在当前帧内的相对时间；
 * - sync_packages() 后面会用最后一个点的 curvature 估计 LiDAR 帧结束时间。
 */
void Preprocess::livoxHandler(
    const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg,
    PointCloudXYZI::Ptr &pcl_out)
{
    pcl_out->reserve(msg->point_num);

    unsigned int valid_num = 0;

    for (unsigned int i = 1; i < msg->point_num; ++i)
    {
        const auto &pt = msg->points[i];

        // 1. 扫描线过滤
        //
        // Livox CustomMsg 中每个点有 line 字段。
        // 如果 line 超过配置的 N_SCANS，则认为不是当前配置下需要处理的扫描线。
        if (pt.line >= N_SCANS)
        {
            continue;
        }

        // 2. 回波类型过滤
        //
        // FAST-LIO2 中常用 tag & 0x30 来过滤 Livox 的回波类型。
        // 这里保留两个常见有效类型：
        //   0x10
        //   0x00
        const bool return_valid =
            ((pt.tag & 0x30) == 0x10) ||
            ((pt.tag & 0x30) == 0x00);

        if (!return_valid)
        {
            continue;
        }

        valid_num++;

        // 3. 简单降采样
        //
        // 每 point_filter_num 个有效点取 1 个。
        // 这不是体素滤波，只是预处理阶段减少点数。
        if (valid_num % point_filter_num != 0)
        {
            continue;
        }

        // 4. 盲区过滤
        if (!isPointValid(pt.x, pt.y, pt.z))
        {
            continue;
        }

        // 5. 连续重复点过滤
        //
        // Livox 数据中可能存在连续重复点。
        // 这里只判断当前点和上一点是否几乎完全相同。
        const auto &last_pt = msg->points[i - 1];

        const bool same_as_last =
            std::abs(pt.x - last_pt.x) < 1e-7 &&
            std::abs(pt.y - last_pt.y) < 1e-7 &&
            std::abs(pt.z - last_pt.z) < 1e-7;

        if (same_as_last)
        {
            continue;
        }

        PointType added_pt;

        added_pt.x = pt.x;
        added_pt.y = pt.y;
        added_pt.z = pt.z;

        // Livox 点的反射强度字段叫 reflectivity
        added_pt.intensity = pt.reflectivity;

        // 6. 点在当前帧内的相对时间
        //
        // Livox offset_time 通常是 ns。
        // FAST-LIO2 中常把它转换成 ms 后放入 curvature。
        // 后续 sync_packages() 中 curvature / 1000.0 可得到秒。
        added_pt.curvature =
            static_cast<float>(pt.offset_time) / 1000000.0f;

        // normal 字段当前不使用，先置 0。
        added_pt.normal_x = 0.0f;
        added_pt.normal_y = 0.0f;
        added_pt.normal_z = 0.0f;

        pcl_out->push_back(added_pt);
    }
}


/**
 * @brief 标准 PointCloud2 点云预处理
 *
 * 当前标准点云分支只实现基础转换：
 * 1. sensor_msgs::msg::PointCloud2 -> pcl::PointXYZI；
 * 2. point_filter_num 简单降采样；
 * 3. blind 盲区过滤；
 * 4. 转换为 PointXYZINormal。
 *
 * 注意：
 * - 很多标准 PointCloud2 数据没有每点时间；
 * - 所以当前 curvature 先置 0；
 * - 后续如果数据里有 time / t / timestamp 字段，再补充解析。
 */
void Preprocess::standardHandler(
    const sensor_msgs::msg::PointCloud2::UniquePtr &msg,
    PointCloudXYZI::Ptr &pcl_out,
    double *lidar_beg_time_offset_sec)
{
    if (lidar_beg_time_offset_sec != nullptr)
    {
        *lidar_beg_time_offset_sec = 0.0;
    }

    bool has_intensity = false;
    for (const auto &field : msg->fields)
    {
        if (field.name == "intensity")
        {
            has_intensity = true;
            break;
        }
    }

    pcl::PointCloud<pcl::PointXYZI> cloud_raw;
    if (has_intensity)
    {
        pcl::fromROSMsg(*msg, cloud_raw);
    }
    else
    {
        // SubT-MRS 的部分 TartanAir 仿真点云只有 x/y/z。
        // 缺少 intensity 时先按 PointXYZ 读取，再显式补零，避免 PCL
        // 尝试匹配不存在的字段并保证后续 FAST-LIO2 数据结构完整。
        pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
        pcl::fromROSMsg(*msg, cloud_xyz);
        cloud_raw.resize(cloud_xyz.size());
        for (size_t i = 0; i < cloud_xyz.size(); ++i)
        {
            cloud_raw.points[i].x = cloud_xyz.points[i].x;
            cloud_raw.points[i].y = cloud_xyz.points[i].y;
            cloud_raw.points[i].z = cloud_xyz.points[i].z;
            cloud_raw.points[i].intensity = 0.0f;
        }
    }

    pcl_out->reserve(cloud_raw.size());

    // Ouster 等标准 PointCloud2 通常通过 t/time/timestamp 字段提供帧内相对时间。
    // 内部统一将相对时间转换为毫秒并保存到 curvature，供后续点云去畸变使用。
    const sensor_msgs::msg::PointField *time_field = nullptr;
    for (const auto &field : msg->fields)
    {
        if (field.name == "t" ||
            field.name == "time" ||
            field.name == "timestamp")
        {
            time_field = &field;
            break;
        }
    }

    double time_to_ms = 1.0;
    switch (time_unit)
    {
        case SEC:
            time_to_ms = 1000.0;
            break;
        case US:
            time_to_ms = 1e-3;
            break;
        case NS:
            time_to_ms = 1e-6;
            break;
        case MS:
        default:
            time_to_ms = 1.0;
            break;
    }

    for (size_t i = 0; i < cloud_raw.size(); ++i)
    {
        if (i % point_filter_num != 0)
        {
            continue;
        }

        const auto &pt = cloud_raw.points[i];

        if (!isPointValid(pt.x, pt.y, pt.z))
        {
            continue;
        }

        PointType added_pt;

        added_pt.x = pt.x;
        added_pt.y = pt.y;
        added_pt.z = pt.z;
        added_pt.intensity = pt.intensity;

        // 没有时间字段时保持为 0，此时同步模块会使用平均扫描周期估计帧结束时间。
        added_pt.curvature = 0.0f;
        if (time_field != nullptr && msg->point_step > 0)
        {
            const size_t row = i / msg->width;
            const size_t col = i % msg->width;
            const size_t byte_index =
                row * msg->row_step +
                col * msg->point_step +
                time_field->offset;

            size_t time_field_size = 0;
            if (time_field->datatype == sensor_msgs::msg::PointField::UINT32 ||
                time_field->datatype == sensor_msgs::msg::PointField::FLOAT32)
            {
                time_field_size = sizeof(uint32_t);
            }
            else if (time_field->datatype == sensor_msgs::msg::PointField::FLOAT64)
            {
                time_field_size = sizeof(double);
            }

            if (time_field_size > 0 &&
                byte_index + time_field_size <= msg->data.size())
            {
                double relative_time = 0.0;
                if (time_field->datatype == sensor_msgs::msg::PointField::UINT32)
                {
                    uint32_t value = 0;
                    std::memcpy(&value, msg->data.data() + byte_index, sizeof(value));
                    relative_time = static_cast<double>(value);
                }
                else if (time_field->datatype == sensor_msgs::msg::PointField::FLOAT32)
                {
                    float value = 0.0f;
                    std::memcpy(&value, msg->data.data() + byte_index, sizeof(value));
                    relative_time = static_cast<double>(value);
                }
                else if (time_field->datatype == sensor_msgs::msg::PointField::FLOAT64)
                {
                    std::memcpy(&relative_time, msg->data.data() + byte_index, sizeof(relative_time));
                }

                added_pt.curvature =
                    static_cast<float>(relative_time * time_to_ms);
            }
        }

        added_pt.normal_x = 0.0f;
        added_pt.normal_y = 0.0f;
        added_pt.normal_z = 0.0f;

        pcl_out->push_back(added_pt);
    }

    if (time_field == nullptr || pcl_out->empty())
    {
        return;
    }

    // 部分 Velodyne 数据集（例如 GEODE）使用帧结束时刻作为消息时间戳，
    // 每点 time 因而位于 [-scan_period, 0]。内部算法要求点时间从帧开始
    // 递增到帧结束，所以这里将最小相对时间平移到 0，并把同样的偏移量
    // 返回给回调函数，用于修正该帧真实的开始时间。
    float min_relative_time_ms = pcl_out->front().curvature;
    for (const auto &point : pcl_out->points)
    {
        min_relative_time_ms = std::min(min_relative_time_ms, point.curvature);
    }

    if (min_relative_time_ms < 0.0f)
    {
        for (auto &point : pcl_out->points)
        {
            point.curvature -= min_relative_time_ms;
        }

        if (lidar_beg_time_offset_sec != nullptr)
        {
            *lidar_beg_time_offset_sec =
                static_cast<double>(min_relative_time_ms) / 1000.0;
        }
    }
}
