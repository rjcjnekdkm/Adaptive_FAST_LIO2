#include "adaptive_fast_lio2/adaptive_map_strategy.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

void AdaptiveMapStrategy::configure(const AdaptiveMapStrategyConfig &config)
{
    config_ = config;
}

void AdaptiveMapStrategy::beginFrame(const AdaptiveMapFrameContext &context)
{
    frame_ = context;
    stats_ = AdaptiveMapFrameStats{};
    normal_bin_counts_.clear();

    if (frame_.mode == DegeneracyMode::Persistent &&
        config_.persistent_insert_quota_max > 0)
    {
        const int scaled_quota = static_cast<int>(std::round(
            config_.persistent_insert_quota_scale *
            static_cast<double>(std::max(0, frame_.effective_points))));
        stats_.persistent_insert_quota = std::min(
            config_.persistent_insert_quota_max,
            std::max(config_.persistent_insert_quota_min, scaled_quota));
    }
}

std::vector<std::size_t> AdaptiveMapStrategy::rankCandidates(
    const std::vector<AdaptiveMapPointMetrics> &metrics) const
{
    std::vector<std::size_t> indices(metrics.size());
    std::iota(indices.begin(), indices.end(), 0);
    if (!config_.enabled || !frame_.degenerate)
    {
        return indices;
    }

    const bool persistent = frame_.mode == DegeneracyMode::Persistent;
    std::stable_sort(
        indices.begin(), indices.end(),
        [&metrics, persistent](std::size_t lhs, std::size_t rhs)
        {
            if (metrics[lhs].effective != metrics[rhs].effective)
            {
                return metrics[lhs].effective;
            }
            if (!metrics[lhs].effective)
            {
                return false;
            }
            if (persistent &&
                std::abs(metrics[lhs].weak_direction_score -
                         metrics[rhs].weak_direction_score) > 1e-9)
            {
                return metrics[lhs].weak_direction_score >
                       metrics[rhs].weak_direction_score;
            }
            return metrics[lhs].quality_score > metrics[rhs].quality_score;
        });
    return indices;
}

bool AdaptiveMapStrategy::allowInsert(
    const PointType &point_lidar,
    const AdaptiveMapPointMetrics &metrics)
{
    if (!config_.enabled)
    {
        return true;
    }

    const double range = std::sqrt(
        point_lidar.x * point_lidar.x +
        point_lidar.y * point_lidar.y +
        point_lidar.z * point_lidar.z);
    if (range < config_.min_range || range > config_.max_range)
    {
        return false;
    }

    const bool persistent = frame_.mode == DegeneracyMode::Persistent;
    if (metrics.effective)
    {
        const bool innovation_unstable =
            config_.transition_guard_enabled &&
            frame_.transition_guard_active;
        const double robust_sigma =
            config_.residual_robust_sigma *
            (innovation_unstable
                 ? config_.transition_guard_residual_sigma_scale
                 : 1.0);
        const double quality_score_min = innovation_unstable
            ? std::max(
                  config_.min_quality_score,
                  config_.transition_guard_min_quality_score)
            : config_.min_quality_score;
        const double robust_residual_limit =
            frame_.residual_median +
            robust_sigma * 1.4826 * frame_.residual_mad;
        const double residual_limit = std::min(
            config_.plane_residual_threshold,
            std::max(
                std::min(
                    frame_.residual_mean * config_.residual_scale,
                    robust_residual_limit),
                0.03));

        if (metrics.residual_abs > residual_limit ||
            metrics.quality_score < quality_score_min)
        {
            ++stats_.quality_rejected;
            return false;
        }
    }
    else if (frame_.degenerate)
    {
        if (metrics.has_local_neighbors)
        {
            ++stats_.invalid_quality_rejected;
            return false;
        }

        const int novel_limit = persistent
            ? static_cast<int>(std::round(
                  config_.max_novel_points_per_frame *
                  config_.persistent_novel_quota_scale))
            : config_.max_novel_points_per_frame;
        if (stats_.novel_accepted >= std::max(0, novel_limit))
        {
            ++stats_.novel_rejected;
            return false;
        }
        ++stats_.novel_accepted;
    }

    if (frame_.degenerate && metrics.effective)
    {
        const int direction_limit = persistent
            ? static_cast<int>(std::round(
                  config_.max_points_per_normal_bin *
                  config_.persistent_direction_quota_scale))
            : config_.max_points_per_normal_bin;
        int &bin_count = normal_bin_counts_[normalDirectionBin(metrics.normal)];
        if (bin_count >= std::max(1, direction_limit))
        {
            ++stats_.direction_rejected;
            return false;
        }
        ++bin_count;
    }

    if (persistent && stats_.persistent_insert_quota > 0)
    {
        if (stats_.persistent_insert_accepted >=
            stats_.persistent_insert_quota)
        {
            ++stats_.persistent_quota_rejected;
            return false;
        }
        ++stats_.persistent_insert_accepted;
    }
    return true;
}

const AdaptiveMapFrameStats &AdaptiveMapStrategy::stats() const
{
    return stats_;
}

int AdaptiveMapStrategy::normalDirectionBin(
    const Eigen::Vector3d &input_normal) const
{
    Eigen::Vector3d normal = input_normal;
    if (normal.norm() < 1e-9)
    {
        return 0;
    }
    normal.normalize();
    if (normal.z() < 0.0 ||
        (std::abs(normal.z()) < 1e-9 && normal.y() < 0.0) ||
        (std::abs(normal.z()) < 1e-9 &&
         std::abs(normal.y()) < 1e-9 && normal.x() < 0.0))
    {
        normal = -normal;
    }

    constexpr double pi = 3.14159265358979323846;
    const double bin_angle =
        std::max(config_.normal_bin_angle_deg, 1.0) * pi / 180.0;
    const double azimuth = std::atan2(normal.y(), normal.x()) + pi;
    const double elevation = std::asin(std::clamp(normal.z(), -1.0, 1.0)) +
                             0.5 * pi;
    const int azimuth_bin = static_cast<int>(std::floor(azimuth / bin_angle));
    const int elevation_bin =
        static_cast<int>(std::floor(elevation / bin_angle));
    const int azimuth_bins =
        std::max(1, static_cast<int>(std::ceil(2.0 * pi / bin_angle)));
    return elevation_bin * azimuth_bins + azimuth_bin;
}
