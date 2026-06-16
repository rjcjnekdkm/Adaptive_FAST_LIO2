#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "adaptive_fast_lio2/adaptive_common.hpp"
#include "ikd-Tree/ikd_Tree.h"

class AdaptiveMapManager
{
public:
    AdaptiveMapManager();

    // 设置 ikd-tree 增量下采样使用的地图体素边长，单位：米。
    void setFilterSizeMap(double filter_size_map);
    // 清空地图并重建空的 ikd-tree。
    void reset();

    // 查询地图是否为空以及当前有效地图点数量。
    bool empty() const;
    size_t size() const;

    // 获取用于发布的地图点云缓存。
    PointCloudXYZI::Ptr getMapCloud();

    // 向 ikd-tree 增量插入点；need_downsample 控制是否执行体素冗余抑制。
    void addPoints(
        const PointCloudXYZI::Ptr &points_to_add,
        bool need_downsample);

    // 在世界坐标系地图中搜索 k 个近邻点，并返回平方距离。
    bool nearestSearch(
        const PointType &point_world,
        int k,
        std::vector<PointType> &nearest_points,
        std::vector<float> &squared_distances) const;

    // 删除给定轴对齐包围盒内的地图点，供局部地图滑窗管理使用。
    int deletePointBoxes(const std::vector<BoxPointType> &boxes);

    int Delete_Point_Boxes(const std::vector<BoxPointType> &boxes)
    {
        return deletePointBoxes(boxes);
    }

private:
    // 将 ikd-tree 中的有效点展开到 map_cloud_，供 ROS2 地图话题发布使用。
    void refreshMapCloudCache();

private:
    // ikd-tree 内部增量下采样使用的地图体素边长，单位：米。
    double filter_size_map_ = 0.5;

    // 地图发布缓存；实际最近邻查询和增删操作均由 ikdtree_ 完成。
    PointCloudXYZI::Ptr map_cloud_;
    // 地图增删后只标记缓存失效，发布地图时再展开 ikd-tree，避免每帧重复 flatten。
    bool map_cloud_dirty_ = true;
    // FAST-LIO2 增量地图使用的 ikd-tree 实例。
    std::unique_ptr<KD_TREE<PointType>> ikdtree_;
};
