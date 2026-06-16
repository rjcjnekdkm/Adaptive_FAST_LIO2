#include "adaptive_fast_lio2/adaptive_map_manager.hpp"

#include <iostream>

AdaptiveMapManager::AdaptiveMapManager()
{
    map_cloud_.reset(new PointCloudXYZI());
    ikdtree_ = std::make_unique<KD_TREE<PointType>>();
    ikdtree_->set_downsample_param(static_cast<float>(filter_size_map_));
}

void AdaptiveMapManager::setFilterSizeMap(double filter_size_map)
{
    filter_size_map_ = filter_size_map;
    if (ikdtree_ != nullptr)
    {
        ikdtree_->set_downsample_param(static_cast<float>(filter_size_map_));
    }
}

void AdaptiveMapManager::reset()
{
    map_cloud_.reset(new PointCloudXYZI());
    ikdtree_ = std::make_unique<KD_TREE<PointType>>();
    ikdtree_->set_downsample_param(static_cast<float>(filter_size_map_));
    map_cloud_dirty_ = true;
}

bool AdaptiveMapManager::empty() const
{
    return ikdtree_ == nullptr || ikdtree_->Root_Node == nullptr || ikdtree_->validnum() <= 0;
}

size_t AdaptiveMapManager::size() const
{
    if (ikdtree_ == nullptr)
    {
        return 0;
    }

    const int valid_num = ikdtree_->validnum();
    return valid_num > 0 ? static_cast<size_t>(valid_num) : 0;
}

PointCloudXYZI::Ptr AdaptiveMapManager::getMapCloud()
{
    if (map_cloud_dirty_)
    {
        refreshMapCloudCache();
    }
    return map_cloud_;
}

void AdaptiveMapManager::addPoints(
    const PointCloudXYZI::Ptr &points_to_add,
    bool need_downsample)
{
    if (points_to_add == nullptr || points_to_add->empty())
    {
        return;
    }

    if (ikdtree_ == nullptr)
    {
        ikdtree_ = std::make_unique<KD_TREE<PointType>>();
        ikdtree_->set_downsample_param(static_cast<float>(filter_size_map_));
    }

    KD_TREE<PointType>::PointVector points;
    points.reserve(points_to_add->size());

    for (const auto &pt : points_to_add->points)
    {
        points.push_back(pt);
    }

    if (ikdtree_->Root_Node == nullptr)
    {
        ikdtree_->Build(points);
    }
    else
    {
        ikdtree_->Add_Points(points, need_downsample);
    }

    map_cloud_dirty_ = true;
}

bool AdaptiveMapManager::nearestSearch(
    const PointType &point_world,
    int k,
    std::vector<PointType> &nearest_points,
    std::vector<float> &squared_distances) const
{
    nearest_points.clear();
    squared_distances.clear();

    if (ikdtree_ == nullptr || ikdtree_->Root_Node == nullptr || k <= 0)
    {
        return false;
    }

    KD_TREE<PointType>::PointVector points_near;
    std::vector<float> distances;

    ikdtree_->Nearest_Search(point_world, k, points_near, distances);

    if (points_near.empty())
    {
        return false;
    }

    nearest_points.assign(points_near.begin(), points_near.end());
    squared_distances = distances;

    return true;
}

int AdaptiveMapManager::deletePointBoxes(const std::vector<BoxPointType> &boxes)
{
    if (boxes.empty() || ikdtree_ == nullptr || ikdtree_->Root_Node == nullptr)
    {
        return 0;
    }

    std::vector<BoxPointType> boxes_copy = boxes;
    const int deleted_points = ikdtree_->Delete_Point_Boxes(boxes_copy);

    map_cloud_dirty_ = true;

    std::cout << "[MapManager] delete boxes=" << boxes.size()
              << ", delete_points=" << deleted_points
              << ", remain_points=" << size()
              << std::endl;

    return deleted_points;
}

void AdaptiveMapManager::refreshMapCloudCache()
{
    if (map_cloud_ == nullptr)
    {
        map_cloud_.reset(new PointCloudXYZI());
    }

    map_cloud_->clear();

    if (ikdtree_ == nullptr || ikdtree_->Root_Node == nullptr)
    {
        return;
    }

    KD_TREE<PointType>::PointVector storage;
    ikdtree_->flatten(ikdtree_->Root_Node, storage, NOT_RECORD);

    map_cloud_->points.assign(storage.begin(), storage.end());
    map_cloud_->width = static_cast<uint32_t>(map_cloud_->points.size());
    map_cloud_->height = 1;
    map_cloud_->is_dense = true;
    map_cloud_dirty_ = false;
}
