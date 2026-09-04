#pragma once

#include <cstddef>
#include <deque>
#include <vector>

#include <Eigen/Dense>

#include "adaptive_fast_lio2/adaptive_common.hpp"
#include "adaptive_fast_lio2/adaptive_map_strategy.hpp"
#include "use-ikfom.hpp"

/**
 * @brief 自适应前端滑动窗口中的单帧观测。
 *
 * 该类型和下面的函数构成 FAST-LIO2 主流程与自适应算法之间的接口。
 * adaptive_laserMapping.cpp 只负责提供当前帧状态和匹配结果，算法实现位于
 * adaptive_frontend_module.cpp。
 */
struct DegeneracyWindowFrame
{
    bool static_degenerate = false;
    double effective_ratio = 0.0;
    double normal_eigen_ratio = 0.0;
    double residual_mean = 0.0;
    double condition_number = 1.0;
    double localizability_f0 = 0.0;
    double localizability_lambda0 = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double yaw = 0.0;
};

// 退化检测、双指标滑窗与观测性诊断。
void begin_adaptive_scan_match(std::size_t point_count);
void record_adaptive_neighbor(std::size_t point_index, bool has_neighbors);
void record_adaptive_effective_match(
    std::size_t point_index,
    double residual_abs,
    double quality_score,
    const Eigen::Vector3d &normal);
bool is_current_frame_degenerate();
const char *degeneracy_mode_name(DegeneracyMode mode);
double median_of_values(std::vector<double> values);
void update_degeneracy_window(bool current_static_degenerate);
void update_residual_diagnostics(
    double residual_sum,
    const std::vector<double> &effective_residuals);
void update_scan_observability_diagnostics(
    const Eigen::MatrixXd &pose_jacobian);
void update_localizability_diagnostics();
void update_pose_covariance_diagnostics();
void update_imu_angular_velocity_diagnostics(const MeasureGroup &meas);

// 掉头/状态突变保护。
void begin_transition_guard_frame();
bool detect_transition_guard_innovation(
    const state_ikfom &prior,
    const state_ikfom &nominal_posterior);
void update_transition_guard_history();

// 把逐点匹配诊断转换为 AdaptiveMapStrategy 的输入，不在主流程复制策略数据。
std::vector<AdaptiveMapPointMetrics> build_adaptive_map_point_metrics();

struct AdaptiveMapUpdatePlan
{
    bool frame_degenerate = false;
    std::vector<AdaptiveMapPointMetrics> point_metrics;
    std::vector<std::size_t> candidate_indices;
};

AdaptiveMapUpdatePlan prepare_adaptive_map_update();
bool allow_adaptive_map_insert(
    const AdaptiveMapUpdatePlan &plan,
    std::size_t point_index,
    const PointType &point_lidar);
void finish_adaptive_map_update(
    const AdaptiveMapUpdatePlan &plan,
    std::size_t add_num,
    std::size_t point_to_add_num,
    std::size_t point_no_need_downsample_num,
    int voxel_rejected_num,
    int total_rejected_num);

std::vector<double> build_adaptive_degeneracy_info(
    double lidar_end_time,
    double insert_ratio);

// 将模块产生的诊断量写入一行实验日志。
void write_runtime_log_row(
    bool frame_degenerate,
    size_t add_num,
    size_t point_to_add_num,
    size_t point_no_need_downsample_num,
    int quality_rejected_num,
    int invalid_quality_rejected_num,
    int direction_rejected_num,
    int persistent_quota_rejected_num,
    int novel_accepted_num,
    int novel_rejected_num,
    int voxel_rejected_num,
    int total_rejected_num);
