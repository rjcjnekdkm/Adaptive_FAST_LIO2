#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "adaptive_fast_lio2/adaptive_common.hpp"

enum class DegeneracyMode
{
    Normal = 0,
    Transient = 1,
    Persistent = 2
};

struct AdaptiveMapStrategyConfig
{
    bool enabled = false;
    double min_range = 0.5;
    double max_range = 80.0;
    double residual_scale = 2.0;
    double residual_robust_sigma = 2.5;
    double min_quality_score = 0.92;
    double normal_bin_angle_deg = 15.0;
    int max_points_per_normal_bin = 30;
    int max_novel_points_per_frame = 50;
    double persistent_direction_quota_scale = 0.5;
    double persistent_novel_quota_scale = 0.5;
    double persistent_insert_quota_scale = 1.0;
    int persistent_insert_quota_min = 0;
    int persistent_insert_quota_max = 0;
    double plane_residual_threshold = 0.1;
    bool transition_guard_enabled = false;
    double transition_guard_residual_sigma_scale = 0.75;
    double transition_guard_min_quality_score = 0.94;
};

struct AdaptiveMapFrameContext
{
    bool degenerate = false;
    DegeneracyMode mode = DegeneracyMode::Normal;
    bool transition_guard_active = false;
    int effective_points = 0;
    double residual_mean = 0.0;
    double residual_median = 0.0;
    double residual_mad = 0.0;
};

struct AdaptiveMapPointMetrics
{
    bool effective = false;
    bool has_local_neighbors = false;
    double residual_abs = 0.0;
    double quality_score = 0.0;
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    double weak_direction_score = 0.0;
};

struct AdaptiveMapFrameStats
{
    int quality_rejected = 0;
    int invalid_quality_rejected = 0;
    int direction_rejected = 0;
    int persistent_quota_rejected = 0;
    int novel_accepted = 0;
    int novel_rejected = 0;
    int persistent_insert_accepted = 0;
    int persistent_insert_quota = 0;
};

class AdaptiveMapStrategy
{
public:
    void configure(const AdaptiveMapStrategyConfig &config);
    void beginFrame(const AdaptiveMapFrameContext &context);

    std::vector<std::size_t> rankCandidates(
        const std::vector<AdaptiveMapPointMetrics> &metrics) const;

    bool allowInsert(
        const PointType &point_lidar,
        const AdaptiveMapPointMetrics &metrics);

    const AdaptiveMapFrameStats &stats() const;

private:
    int normalDirectionBin(const Eigen::Vector3d &normal) const;

private:
    AdaptiveMapStrategyConfig config_;
    AdaptiveMapFrameContext frame_;
    AdaptiveMapFrameStats stats_;
    std::unordered_map<int, int> normal_bin_counts_;
};
