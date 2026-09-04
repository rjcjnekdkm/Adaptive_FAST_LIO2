#include "adaptive_fast_lio2/adaptive_frontend_module.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <unordered_map>
#include <utility>


#include "adaptive_fast_lio2/adaptive_map_manager.hpp"
#include "adaptive_fast_lio2/adaptive_runtime_logger.hpp"

// FAST-LIO2 主流程拥有传感器、滤波器和地图；本模块只读取这些接口数据，
// 并维护自适应算法状态。集中列出依赖，避免算法实现重新散落到主文件。
extern PointCloudXYZI::Ptr feats_down_body;
extern PointCloudXYZI::Ptr feats_down_world;
extern MeasureGroup Measures;
extern state_ikfom state_point;
extern esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
extern std::shared_ptr<AdaptiveMapManager> p_map;

extern int scan_count;
extern int scan_num;
extern int map_update_count;
extern int effct_feat_num;
extern double res_mean_last;
extern double filter_size_map;
extern bool adaptive_map_enable;
extern bool degeneracy_diagnostic_enable;
extern bool runtime_console_enable;
extern int runtime_log_interval_frames;
extern AdaptiveRuntimeLogger runtime_logger;
extern AdaptiveMapStrategy adaptive_map_strategy;
extern bool previous_frame_degenerate;
extern bool has_previous_degenerate_state;
extern int previous_degeneracy_mode;
extern bool has_previous_degeneracy_mode;
extern std::size_t last_map_add_num;

extern int adaptive_min_effective_points;
extern double adaptive_min_effective_ratio;
extern double adaptive_max_mean_residual;
extern double adaptive_min_normal_eigen_ratio;
extern bool adaptive_window_enable;
extern int adaptive_window_size;
extern double adaptive_window_persistent_ratio_threshold;
extern double adaptive_window_normal_ratio_scale;
extern double adaptive_window_residual_stability_ratio;
extern double adaptive_window_min_motion;
extern double adaptive_window_max_yaw_change;
extern int adaptive_window_min_recent_degenerate_streak;
extern double adaptive_window_min_condition_number;
extern int adaptive_window_enter_count_threshold;
extern int adaptive_window_exit_count_threshold;

extern std::deque<DegeneracyWindowFrame> degeneracy_window;
extern DegeneracyMode current_degeneracy_mode;
extern bool degeneracy_window_ready;
extern double window_degenerate_ratio;
extern double window_normal_eigen_ratio_mean;
extern double window_residual_cv;
extern double window_path_length;
extern double window_yaw_change;
extern double window_condition_number_mean;
extern int window_recent_degenerate_streak;
extern double window_localizability_f0_mean;
extern double window_localizability_lambda0_mean;
extern int persistent_enter_count;
extern int persistent_exit_count;

extern double frame_effective_ratio;
extern double frame_normal_eigen_ratio;
extern double frame_residual_median;
extern double frame_residual_mad;
extern double frame_condition_number;
extern Eigen::Matrix<double, 6, 1> frame_weak_direction;
extern std::size_t frame_localizability_observed_voxels;
extern double frame_localizability_planarity_mean;
extern Eigen::Vector3d frame_localizability_eigenvalues;
extern double frame_localizability_f0;
extern double frame_localizability_lambda0;

extern bool transition_guard_enable;
extern int transition_guard_history_size;
extern int transition_guard_min_history;
extern double transition_guard_mad_scale;
extern double transition_guard_translation_floor;
extern double transition_guard_rotation_floor;
extern double transition_guard_localizability_drop_ratio;
extern double transition_guard_turn_rate_threshold;
extern double transition_guard_residual_context_threshold;
extern int transition_guard_map_hold_frames;
extern bool frame_transition_guard_triggered;
extern bool frame_transition_guard_turn_residual_triggered;
extern bool frame_transition_guard_map_active;
extern int frame_transition_guard_map_hold_remaining;
extern int transition_guard_map_hold_remaining;
extern bool frame_transition_guard_history_ready;
extern double frame_lidar_nominal_correction_translation;
extern double frame_lidar_nominal_correction_rotation;
extern double frame_lidar_correction_translation_threshold;
extern double frame_lidar_correction_rotation_threshold;
extern double frame_transition_guard_localizability_drop;
extern double frame_transition_guard_nominal_residual;
extern std::deque<double> transition_guard_translation_history;
extern std::deque<double> transition_guard_rotation_history;
extern std::deque<double> transition_guard_f0_history;
extern std::deque<double> transition_guard_lambda0_history;

extern double frame_imu_angular_velocity_mean;
extern double frame_imu_angular_velocity_max;
extern Eigen::Vector3d frame_translation_cov_eigenvalues;
extern Eigen::Vector3d frame_translation_weak_direction;
extern Eigen::Vector3d frame_rotation_cov_eigenvalues;
extern Eigen::Vector3d frame_rotation_weak_direction;

extern std::vector<uint8_t> map_point_effective;
extern std::vector<double> map_point_residual_abs;
extern std::vector<double> map_point_quality_score;
extern std::vector<Eigen::Vector3d> map_point_normal;
extern std::vector<double> map_point_weak_direction_score;
extern std::vector<size_t> effective_point_indices;
extern std::vector<std::vector<PointType>> nearest_points_cache;
extern std::vector<uint8_t> map_point_has_local_neighbors;

extern std::uint64_t total_map_added;
extern std::uint64_t total_quality_rejected;
extern std::uint64_t total_direction_rejected;
extern std::uint64_t total_persistent_quota_rejected;
extern std::uint64_t total_voxel_rejected;
extern std::uint64_t total_scan_update_failures;

extern void pointBodyToWorld(const PointType &pi, PointType &po);

void begin_adaptive_scan_match(std::size_t point_count)
{
    map_point_effective.assign(point_count, 0);
    map_point_residual_abs.assign(point_count, 0.0);
    map_point_quality_score.assign(point_count, 0.0);
    map_point_normal.assign(point_count, Eigen::Vector3d::Zero());
    map_point_weak_direction_score.assign(point_count, 0.0);
    // IKFoM 非收敛迭代会复用上一轮近邻搜索结果；点数未变化时必须同步
    // 保留 has-neighbor 标记，这与拆分前行为一致。
    if (map_point_has_local_neighbors.size() != point_count)
    {
        map_point_has_local_neighbors.assign(point_count, 0);
    }
    effective_point_indices.clear();

    frame_residual_median = 0.0;
    frame_residual_mad = 0.0;
    frame_condition_number = 1.0;
    frame_weak_direction.setZero();
}

void record_adaptive_neighbor(std::size_t point_index, bool has_neighbors)
{
    if (point_index < map_point_has_local_neighbors.size())
    {
        map_point_has_local_neighbors[point_index] = has_neighbors ? 1 : 0;
    }
}

void record_adaptive_effective_match(
    std::size_t point_index,
    double residual_abs,
    double quality_score,
    const Eigen::Vector3d &normal)
{
    if (point_index >= map_point_effective.size())
    {
        return;
    }
    map_point_effective[point_index] = 1;
    map_point_residual_abs[point_index] = residual_abs;
    map_point_quality_score[point_index] = quality_score;
    map_point_normal[point_index] = normal;
    effective_point_indices.push_back(point_index);
}

std::vector<AdaptiveMapPointMetrics> build_adaptive_map_point_metrics()
{
    std::vector<AdaptiveMapPointMetrics> metrics(map_point_effective.size());
    for (std::size_t i = 0; i < metrics.size(); ++i)
    {
        metrics[i].effective = map_point_effective[i] != 0;
        metrics[i].has_local_neighbors =
            i < map_point_has_local_neighbors.size() &&
            map_point_has_local_neighbors[i] != 0;
        metrics[i].residual_abs = map_point_residual_abs[i];
        metrics[i].quality_score = map_point_quality_score[i];
        metrics[i].normal = map_point_normal[i];
        metrics[i].weak_direction_score = map_point_weak_direction_score[i];
    }
    return metrics;
}

AdaptiveMapUpdatePlan prepare_adaptive_map_update()
{
    AdaptiveMapUpdatePlan plan;
    plan.frame_degenerate = is_current_frame_degenerate();
    update_degeneracy_window(plan.frame_degenerate);

    AdaptiveMapFrameContext frame;
    frame.degenerate = plan.frame_degenerate;
    frame.mode = current_degeneracy_mode;
    frame.transition_guard_active = frame_transition_guard_map_active;
    frame.effective_points = effct_feat_num;
    frame.residual_mean = res_mean_last;
    frame.residual_median = frame_residual_median;
    frame.residual_mad = frame_residual_mad;

    plan.candidate_indices.resize(feats_down_body->size());
    std::iota(
        plan.candidate_indices.begin(),
        plan.candidate_indices.end(),
        0);

    if (adaptive_map_enable)
    {
        adaptive_map_strategy.beginFrame(frame);
        plan.point_metrics = build_adaptive_map_point_metrics();
        plan.candidate_indices =
            adaptive_map_strategy.rankCandidates(plan.point_metrics);
    }
    return plan;
}

bool allow_adaptive_map_insert(
    const AdaptiveMapUpdatePlan &plan,
    std::size_t point_index,
    const PointType &point_lidar)
{
    if (!adaptive_map_enable)
    {
        return true;
    }
    if (point_index >= plan.point_metrics.size())
    {
        return false;
    }
    return adaptive_map_strategy.allowInsert(
        point_lidar,
        plan.point_metrics[point_index]);
}

void finish_adaptive_map_update(
    const AdaptiveMapUpdatePlan &plan,
    std::size_t add_num,
    std::size_t point_to_add_num,
    std::size_t point_no_need_downsample_num,
    int voxel_rejected_num,
    int total_rejected_num)
{
    const AdaptiveMapFrameStats stats = adaptive_map_strategy.stats();
    const int quality_rejected_num = stats.quality_rejected;
    const int invalid_quality_rejected_num = stats.invalid_quality_rejected;
    const int direction_rejected_num = stats.direction_rejected;
    const int persistent_quota_rejected_num =
        stats.persistent_quota_rejected;
    const int novel_accepted_num = stats.novel_accepted;
    const int novel_rejected_num = stats.novel_rejected;

    ++map_update_count;
    last_map_add_num = add_num;
    total_map_added += add_num;
    total_quality_rejected +=
        quality_rejected_num + invalid_quality_rejected_num + novel_rejected_num;
    total_direction_rejected += direction_rejected_num;
    total_persistent_quota_rejected += persistent_quota_rejected_num;
    total_voxel_rejected += voxel_rejected_num;

    write_runtime_log_row(
        plan.frame_degenerate,
        add_num,
        point_to_add_num,
        point_no_need_downsample_num,
        quality_rejected_num,
        invalid_quality_rejected_num,
        direction_rejected_num,
        persistent_quota_rejected_num,
        novel_accepted_num,
        novel_rejected_num,
        voxel_rejected_num,
        total_rejected_num);

    const bool degenerate_state_changed =
        adaptive_map_enable &&
        (!has_previous_degenerate_state ||
         plan.frame_degenerate != previous_frame_degenerate);
    const bool degeneracy_mode_changed =
        adaptive_map_enable &&
        (!has_previous_degeneracy_mode ||
         static_cast<int>(current_degeneracy_mode) != previous_degeneracy_mode);
    const bool periodic_log =
        runtime_log_interval_frames > 0 &&
        map_update_count % runtime_log_interval_frames == 0;

    if (runtime_console_enable && degenerate_state_changed)
    {
        std::cout << "[Degeneracy] frame=" << map_update_count
                  << ", state="
                  << (plan.frame_degenerate ? "ENTER" : "EXIT")
                  << ", effective=" << effct_feat_num
                  << ", effective_ratio=" << frame_effective_ratio
                  << ", normal_eigen_ratio=" << frame_normal_eigen_ratio
                  << ", mean_residual=" << res_mean_last
                  << std::endl;
    }
    if (runtime_console_enable && degeneracy_mode_changed)
    {
        std::cout << "[DegeneracyMode] frame=" << map_update_count
                  << ", mode=" << degeneracy_mode_name(current_degeneracy_mode)
                  << ", window_ready=" << (degeneracy_window_ready ? "Y" : "N")
                  << ", deg_ratio=" << window_degenerate_ratio
                  << ", normal_mean=" << window_normal_eigen_ratio_mean
                  << ", residual_cv=" << window_residual_cv
                  << ", path=" << window_path_length
                  << ", yaw=" << window_yaw_change
                  << ", cond_mean=" << window_condition_number_mean
                  << ", streak=" << window_recent_degenerate_streak
                  << std::endl;
    }
    if (runtime_console_enable &&
        (periodic_log || degenerate_state_changed || degeneracy_mode_changed))
    {
        const double insert_ratio = feats_down_body->empty()
            ? 0.0
            : static_cast<double>(add_num) /
                  static_cast<double>(feats_down_body->size());
        std::ostringstream line;
        line << std::fixed << std::setprecision(3)
             << "[Runtime] frame=" << map_update_count
             << " pos=(" << state_point.pos.x() << ","
             << state_point.pos.y() << "," << state_point.pos.z() << ")"
             << " points=" << feats_down_body->size()
             << " effective=" << effct_feat_num
             << " eff_ratio=" << frame_effective_ratio
             << " residual(mean/median/mad)=" << res_mean_last << "/"
             << frame_residual_median << "/" << frame_residual_mad
             << " normal_ratio=" << frame_normal_eigen_ratio
             << " cond=" << frame_condition_number
             << " degenerate=" << (plan.frame_degenerate ? "Y" : "N")
             << " mode=" << degeneracy_mode_name(current_degeneracy_mode)
             << " win(deg/norm/cv/path/yaw/cond/streak)="
             << window_degenerate_ratio << "/"
             << window_normal_eigen_ratio_mean << "/"
             << window_residual_cv << "/"
             << window_path_length << "/"
             << window_yaw_change << "/"
             << window_condition_number_mean << "/"
             << window_recent_degenerate_streak
             << " add=" << add_num
             << " insert_ratio=" << insert_ratio
             << " reject(q/d/v)="
             << quality_rejected_num + invalid_quality_rejected_num +
                    novel_rejected_num
             << "/" << direction_rejected_num
             << "/" << voxel_rejected_num
             << " map=" << p_map->size()
             << " total(add/q/d/v)="
             << total_map_added << "/"
             << total_quality_rejected << "/"
             << total_direction_rejected << "/"
             << total_voxel_rejected;
        std::cout << line.str() << std::endl;
    }

    previous_frame_degenerate = plan.frame_degenerate;
    has_previous_degenerate_state = true;
    previous_degeneracy_mode = static_cast<int>(current_degeneracy_mode);
    has_previous_degeneracy_mode = true;
}

std::vector<double> build_adaptive_degeneracy_info(
    double lidar_end_time,
    double insert_ratio)
{
    std::vector<double> data(34, 0.0);
    data[0] = lidar_end_time;
    data[1] = static_cast<double>(current_degeneracy_mode);
    data[2] = previous_frame_degenerate ? 1.0 : 0.0;
    data[3] = frame_effective_ratio;
    data[4] = res_mean_last;
    data[5] = frame_normal_eigen_ratio;
    data[6] = frame_condition_number;
    data[7] = window_degenerate_ratio;
    data[8] = window_normal_eigen_ratio_mean;
    data[9] = window_residual_cv;
    data[10] = window_path_length;
    data[11] = window_yaw_change;
    data[12] = window_condition_number_mean;
    data[13] = static_cast<double>(window_recent_degenerate_streak);
    data[14] = insert_ratio;
    data[15] = static_cast<double>(frame_localizability_observed_voxels);
    data[16] = frame_localizability_planarity_mean;
    data[17] = frame_localizability_eigenvalues(0);
    data[18] = frame_localizability_eigenvalues(1);
    data[19] = frame_localizability_eigenvalues(2);
    data[20] = frame_localizability_f0;
    data[21] = frame_localizability_lambda0;
    data[22] = frame_translation_cov_eigenvalues(0);
    data[23] = frame_translation_cov_eigenvalues(1);
    data[24] = frame_translation_cov_eigenvalues(2);
    data[25] = frame_translation_weak_direction.x();
    data[26] = frame_translation_weak_direction.y();
    data[27] = frame_translation_weak_direction.z();
    data[28] = frame_rotation_cov_eigenvalues(0);
    data[29] = frame_rotation_cov_eigenvalues(1);
    data[30] = frame_rotation_cov_eigenvalues(2);
    data[31] = frame_rotation_weak_direction.x();
    data[32] = frame_rotation_weak_direction.y();
    data[33] = frame_rotation_weak_direction.z();
    return data;
}

void update_residual_diagnostics(
    double residual_sum,
    const std::vector<double> &effective_residuals)
{
    res_mean_last = effct_feat_num > 0
        ? residual_sum / static_cast<double>(effct_feat_num)
        : 0.0;
    frame_residual_median = median_of_values(effective_residuals);

    std::vector<double> residual_deviations;
    residual_deviations.reserve(effective_residuals.size());
    for (const double residual : effective_residuals)
    {
        residual_deviations.push_back(
            std::abs(residual - frame_residual_median));
    }
    frame_residual_mad = median_of_values(residual_deviations);
}

void update_scan_observability_diagnostics(
    const Eigen::MatrixXd &pose_jacobian)
{
    if (pose_jacobian.rows() < 1 || pose_jacobian.cols() != 6)
    {
        return;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        pose_jacobian,
        Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd singular_values = svd.singularValues();
    if (singular_values.size() == 0 || svd.matrixV().cols() < 1)
    {
        return;
    }

    const double sigma_max = singular_values(0);
    const double sigma_min = singular_values(singular_values.size() - 1);
    frame_condition_number =
        sigma_min > 1e-9 ? sigma_max / sigma_min : 1e9;

    const int weak_col = svd.matrixV().cols() - 1;
    frame_weak_direction =
        svd.matrixV().col(weak_col).template cast<double>();

    for (int row = 0; row < pose_jacobian.rows(); ++row)
    {
        if (row >= static_cast<int>(effective_point_indices.size()))
        {
            break;
        }
        const size_t point_index = effective_point_indices[row];
        if (point_index >= map_point_weak_direction_score.size())
        {
            continue;
        }
        const Eigen::VectorXd jacobian_row = pose_jacobian.row(row);
        const double row_norm = std::max(jacobian_row.norm(), 1e-9);
        map_point_weak_direction_score[point_index] =
            std::abs(jacobian_row.dot(frame_weak_direction)) / row_norm;
    }
}


/**
 * @brief 判断当前帧是否可能退化
 */
bool is_current_frame_degenerate()
{
    const int downsampled_points = static_cast<int>(feats_down_body->size());
    frame_effective_ratio =
        downsampled_points > 0
            ? static_cast<double>(effct_feat_num) / static_cast<double>(downsampled_points)
            : 0.0;

    // 计算法向量信息矩阵 A = (1/N) * Σ(n_i * n_i^T)。
    // 使用 rho = lambda_min / lambda_max 作为法向方向覆盖度。
    Eigen::Matrix3d normal_information = Eigen::Matrix3d::Zero();
    int normal_count = 0;
    for (size_t i = 0; i < map_point_effective.size(); ++i)
    {
        if (map_point_effective[i] == 0)
        {
            continue;
        }

        const Eigen::Vector3d &normal = map_point_normal[i];
        normal_information += normal * normal.transpose();
        normal_count++;
    }

    frame_normal_eigen_ratio = 0.0;
    if (normal_count > 0)
    {
        normal_information /= static_cast<double>(normal_count);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(normal_information);
        if (solver.info() == Eigen::Success)
        {
            const Eigen::Vector3d eigenvalues = solver.eigenvalues();
            frame_normal_eigen_ratio = eigenvalues(2) > 1e-9 ? eigenvalues(0) / eigenvalues(2) : 0.0;
        }
    }

    // 两个开关均关闭时只记录连续几何量，不计算静态退化标签；
    // diagnostic.enable 可在不修改地图插入的前提下启用标签和窗口状态机。
    if(!adaptive_map_enable && !degeneracy_diagnostic_enable)
    {
        return false;
    }

    return effct_feat_num < adaptive_min_effective_points ||
           frame_effective_ratio < adaptive_min_effective_ratio ||
           res_mean_last > adaptive_max_mean_residual ||
           frame_normal_eigen_ratio < adaptive_min_normal_eigen_ratio;
}

/**
 * @brief 将退化模式枚举转换为可读字符串
 *
 * 仅用于终端日志输出，不参与算法决策。
 */
const char *degeneracy_mode_name(DegeneracyMode mode)
{
    switch (mode)
    {
    case DegeneracyMode::Normal:
        return "NORMAL";
    case DegeneracyMode::Transient:
        return "TRANSIENT";
    case DegeneracyMode::Persistent:
        return "PERSISTENT";
    default:
        return "UNKNOWN";
    }
}

/**
 * @brief 将角度归一化到 [-pi, pi]
 *
 * 滑动窗口中累计 yaw 变化时需要处理角度跨越 ±pi 的情况，
 * 否则小转角可能被误算成接近 2*pi 的大转角。
 */
double normalize_angle(double angle)
{
    constexpr double pi = 3.14159265358979323846;
    while (angle > pi)
    {
        angle -= 2.0 * pi;
    }
    while (angle < -pi)
    {
        angle += 2.0 * pi;
    }
    return angle;
}

/**
 * @brief 从当前滤波状态姿态中提取 yaw 角
 */
double current_state_yaw()
{
    // 从当前 SO(3) 姿态中提取 yaw，用于窗口内累计转角。
    // 这里只需要判断“是否近似直行”，不参与滤波状态更新。
    const Eigen::Matrix3d rot = state_point.rot.toRotationMatrix();
    return std::atan2(rot(1, 0), rot(0, 0));
}

/**
 * @brief 更新滑动窗口退化状态
 *
 * 滑动窗口保存最近 K 帧：
 *   W_t = {t-K+1, ..., t}
 *
 * 并计算：
 *   1. 退化比例 gamma_t = count(D_j = true) / K
 *   2. 平均法向覆盖度 mean(rho_j)
 *   3. 残差变异系数 CV = std(mean_residual_j) / mean(mean_residual_j)
 *   4. 窗口内累计运动距离 L_t
 *   5. 窗口内累计 yaw 变化 Psi_t
 *   6. 最近连续退化长度 S_t
 *   7. 窗口平均 scan-to-map condition number
 */
void update_degeneracy_window(bool current_static_degenerate)
{
    // 每帧重新计算窗口统计量。若窗口未启用，或地图策略和纯诊断均未启用，
    // 这些值保持 0，并只保留 Normal/Transient 两种模式。
    window_degenerate_ratio = 0.0;
    window_normal_eigen_ratio_mean = 0.0;
    window_residual_cv = 0.0;
    window_path_length = 0.0;
    window_yaw_change = 0.0;
    window_condition_number_mean = 1.0;
    window_recent_degenerate_streak = 0;
    window_localizability_f0_mean = 0.0;
    window_localizability_lambda0_mean = 0.0;
    degeneracy_window_ready = false;

    if ((!adaptive_map_enable && !degeneracy_diagnostic_enable) || !adaptive_window_enable)
    {
        // 滑动窗口关闭时：当前帧退化就视为短时退化，否则 Normal。
        // 这样可以兼容原来的静态逐帧版本。
        current_degeneracy_mode = current_static_degenerate ? DegeneracyMode::Transient : DegeneracyMode::Normal;
        persistent_enter_count = 0;
        persistent_exit_count = current_static_degenerate ? 0 : persistent_exit_count + 1;
        return;
    }

    const bool window_frame_degenerate = current_static_degenerate;

    // 将当前帧的退化指标压入窗口。
    // 注意：这里记录的是 scan-to-map 和最终状态更新后的统计量，
    // 因此与本帧地图插入决策使用的是同一时刻的质量信息。
    DegeneracyWindowFrame frame;
    frame.static_degenerate = window_frame_degenerate;
    frame.effective_ratio = frame_effective_ratio;
    frame.normal_eigen_ratio = frame_normal_eigen_ratio;
    frame.residual_mean = res_mean_last;
    frame.condition_number = frame_condition_number;
    frame.localizability_f0 = frame_localizability_f0;
    frame.localizability_lambda0 = frame_localizability_lambda0;
    frame.position = state_point.pos;
    frame.yaw = current_state_yaw();

    degeneracy_window.push_back(frame);
    const int window_size = std::max(1, adaptive_window_size);
    // 保持固定长度 K：新帧进入，最旧帧弹出。
    while (static_cast<int>(degeneracy_window.size()) > window_size)
    {
        degeneracy_window.pop_front();
    }

    // 窗口未攒够 K 帧时，不允许进入 Persistent。
    // 这样避免刚启动或刚初始化时统计不稳定。
    degeneracy_window_ready = static_cast<int>(degeneracy_window.size()) >= window_size;

    // 统计窗口内退化帧比例 gamma_t 和平均法向覆盖度 mean(rho_t)。
    int degenerate_count = 0;
    double residual_sum = 0.0;
    double condition_number_sum = 0.0;
    for (const auto &item : degeneracy_window)
    {
        if (item.static_degenerate)
        {
            degenerate_count++;
        }
        window_normal_eigen_ratio_mean += item.normal_eigen_ratio;
        residual_sum += item.residual_mean;
        condition_number_sum += item.condition_number;
        window_localizability_f0_mean += item.localizability_f0;
        window_localizability_lambda0_mean += item.localizability_lambda0;
    }

    const double count = static_cast<double>(degeneracy_window.size());
    if (count > 0.0)
    {
        window_degenerate_ratio =
            static_cast<double>(degenerate_count) / count;
        window_normal_eigen_ratio_mean /= count;
        window_condition_number_mean = condition_number_sum / count;
        window_localizability_f0_mean /= count;
        window_localizability_lambda0_mean /= count;
    }

    // 从当前帧向前统计最近连续退化长度 S_t。
    // 它修正单纯比例判断的漏洞：退化-正常-退化虽然比例可能较高，
    // 但 recent streak 不足，不应被认为是严格持续退化。
    for (auto it = degeneracy_window.rbegin();
         it != degeneracy_window.rend();
         ++it)
    {
        if (!it->static_degenerate)
        {
            break;
        }
        window_recent_degenerate_streak++;
    }

    // 计算残差均值和标准差，用变异系数 CV 衡量残差是否稳定。
    const double residual_mean = count > 0.0 ? residual_sum / count : 0.0;
    double residual_variance = 0.0;
    for (const auto &item : degeneracy_window)
    {
        const double delta = item.residual_mean - residual_mean;
        residual_variance += delta * delta;
    }
    if (count > 0.0)
    {
        residual_variance /= count;
    }
    const double residual_std = std::sqrt(residual_variance);
    window_residual_cv =
        residual_mean > 1e-9 ? residual_std / residual_mean : residual_std;

    // 计算窗口内累计运动距离和累计 yaw 变化：
    //   L_t   = Σ ||p_i - p_{i-1}||
    //   Psi_t = Σ |wrap(yaw_i - yaw_{i-1})|
    //
    // 长走廊/隧道通常表现为 L_t 较大但 Psi_t 较小；
    // 拐角则 Psi_t 会明显增大。
    for (size_t i = 1; i < degeneracy_window.size(); ++i)
    {
        window_path_length +=
            (degeneracy_window[i].position -
             degeneracy_window[i - 1].position)
                .norm();
        window_yaw_change += std::abs(
            normalize_angle(
                degeneracy_window[i].yaw -
                degeneracy_window[i - 1].yaw));
    }

    // 持续退化候选判断。
    // 只有所有条件同时满足，才说明当前更像“长走廊/隧道式持续退化”，
    // 而不是短时拐角或单帧噪声。
    const bool persistent_candidate =
        degeneracy_window_ready &&
        window_degenerate_ratio > adaptive_window_persistent_ratio_threshold &&
        window_normal_eigen_ratio_mean <
            adaptive_window_normal_ratio_scale * adaptive_min_normal_eigen_ratio &&
        window_residual_cv < adaptive_window_residual_stability_ratio &&
        window_path_length > adaptive_window_min_motion &&
        window_yaw_change < adaptive_window_max_yaw_change &&
        window_recent_degenerate_streak >=
            std::max(1, adaptive_window_min_recent_degenerate_streak) &&
        window_condition_number_mean >= adaptive_window_min_condition_number;
    if (persistent_candidate)
    {
        // 连续满足 persistent_candidate 才进入 Persistent。
        persistent_enter_count++;
        persistent_exit_count = 0;
    }
    else
    {
        // 不满足则清空进入计数，同时累计退出计数。
        persistent_enter_count = 0;
        persistent_exit_count++;
    }

    if (!window_frame_degenerate)
    {
        // 当前帧已经不退化时，只有连续若干帧退出条件成立才回到 Normal。
        // 这样避免由于单帧指标抖动导致模式立即跳变。
        if (persistent_exit_count >= std::max(1, adaptive_window_exit_count_threshold))
        {
            current_degeneracy_mode = DegeneracyMode::Normal;
        }
        return;
    }

    if (current_degeneracy_mode == DegeneracyMode::Persistent)
    {
        // 已处于持续退化时，保持 Persistent，直到退出计数达到阈值。
        if (persistent_exit_count >= std::max(1, adaptive_window_exit_count_threshold))
        {
            current_degeneracy_mode = DegeneracyMode::Transient;
        }
    }
    else if (persistent_enter_count >= std::max(1, adaptive_window_enter_count_threshold))
    {
        // 连续多帧满足持续退化候选后，从 Transient 升级为 Persistent。
        current_degeneracy_mode = DegeneracyMode::Persistent;
    }
    else
    {
        // 当前帧退化但窗口还没证明持续退化，先作为短时退化处理。
        current_degeneracy_mode = DegeneracyMode::Transient;
    }
}

/**
 * @brief 计算一组数值的中位数
 *
 * 用于残差 median 和 MAD 计算。参数按值传入，允许函数内部重排数据，
 * 避免修改调用者持有的原始残差序列。
 */
double median_of_values(std::vector<double> values)
{
    if (values.empty())
    {
        return 0.0;
    }

    const size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    const double upper = values[middle];
    if (values.size() % 2 != 0)
    {
        return upper;
    }

    std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
    return 0.5 * (values[middle - 1] + upper);
}

/**
 * @brief 当前可见局部地图的体素索引
 *
 * 这里使用 scan-to-map 已经查询到的地图近邻中心进行分箱，不展开完整
 * iKD-Tree。这样每个局部地图体素只贡献一次，避免密集墙面重复点主导统计。
 */
struct ObservedMapVoxelKey
{
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;

    bool operator==(const ObservedMapVoxelKey &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct ObservedMapVoxelKeyHash
{
    std::size_t operator()(const ObservedMapVoxelKey &key) const
    {
        std::size_t seed = std::hash<std::int64_t>{}(key.x);
        seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

struct ObservedVoxelNormal
{
    Eigen::Matrix3d normal_outer_sum = Eigen::Matrix3d::Zero();
    double planarity_sum = 0.0;
    std::size_t observations = 0;
};

/**
 * @brief 计算当前 scan-to-map 可见体素的法向可定位性场
 *
 * 对每个具有有效局部地图近邻的当前点（不要求已经形成有效平面残差），
 * 复用其地图近邻并计算局部协方差。平面度定义为
 * rho = 1 - lambda_min(C) / lambda_mid(C)，随后构造：
 *
 *   M       = sum_v rho_v * n_v * n_v^T
 *   f0      = lambda_min(M) / trace(M)
 *   lambda0 = lambda_min(M) / |V|
 *
 * 同一地图体素被多个当前点观测时先平均法向外积与平面度，再作为一个体素
 * 贡献。低平面度/非平面体素仍计入 |V|，这是让 lambda0 区分信息缺失和
 * 信息稀释的关键。该函数只生成诊断量，不参与 ESIKF、状态机或地图更新。
 */
void update_localizability_diagnostics()
{
    frame_localizability_observed_voxels = 0;
    frame_localizability_planarity_mean = 0.0;
    frame_localizability_eigenvalues.setZero();
    frame_localizability_f0 = 0.0;
    frame_localizability_lambda0 = 0.0;

    if (filter_size_map <= 1e-6 || nearest_points_cache.empty())
    {
        return;
    }

    std::unordered_map<ObservedMapVoxelKey, ObservedVoxelNormal, ObservedMapVoxelKeyHash> voxels;
    voxels.reserve(static_cast<std::size_t>(std::max(0, effct_feat_num)));

    for (std::size_t i = 0; i < nearest_points_cache.size(); ++i)
    {
        if (i >= map_point_has_local_neighbors.size() ||
            map_point_has_local_neighbors[i] == 0 ||
            nearest_points_cache[i].size() < 3)
        {
            continue;
        }

        const auto &neighbors = nearest_points_cache[i];
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto &point : neighbors)
        {
            centroid += Eigen::Vector3d(point.x, point.y, point.z);
        }
        centroid /= static_cast<double>(neighbors.size());

        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (const auto &point : neighbors)
        {
            const Eigen::Vector3d delta =
                Eigen::Vector3d(point.x, point.y, point.z) - centroid;
            covariance += delta * delta.transpose();
        }
        covariance /= static_cast<double>(neighbors.size());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> covariance_solver(covariance);
        if (covariance_solver.info() != Eigen::Success)
        {
            continue;
        }

        const Eigen::Vector3d covariance_eigenvalues = covariance_solver.eigenvalues();
        if (covariance_eigenvalues(1) <= 1e-12)
        {
            continue;
        }

        const double planarity = std::clamp(
            1.0 - covariance_eigenvalues(0) / covariance_eigenvalues(1),
            0.0,
            1.0);
        const Eigen::Vector3d normal =
            covariance_solver.eigenvectors().col(0).normalized();
        if (!normal.allFinite())
        {
            continue;
        }

        const ObservedMapVoxelKey key{
            static_cast<std::int64_t>(std::floor(centroid.x() / filter_size_map)),
            static_cast<std::int64_t>(std::floor(centroid.y() / filter_size_map)),
            static_cast<std::int64_t>(std::floor(centroid.z() / filter_size_map))};

        ObservedVoxelNormal &voxel = voxels[key];
        voxel.normal_outer_sum += normal * normal.transpose();
        voxel.planarity_sum += planarity;
        voxel.observations++;
    }

    Eigen::Matrix3d localizability_field = Eigen::Matrix3d::Zero();
    double planarity_sum = 0.0;
    for (const auto &entry : voxels)
    {
        const ObservedVoxelNormal &sample = entry.second;
        if (sample.observations == 0)
        {
            continue;
        }
        const double inverse_observations =
            1.0 / static_cast<double>(sample.observations);
        const double mean_planarity =
            sample.planarity_sum * inverse_observations;
        localizability_field +=
            mean_planarity * sample.normal_outer_sum * inverse_observations;
        planarity_sum += mean_planarity;
    }

    frame_localizability_observed_voxels = voxels.size();
    if (voxels.empty())
    {
        return;
    }

    frame_localizability_planarity_mean =
        planarity_sum / static_cast<double>(voxels.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> field_solver(localizability_field);
    if (field_solver.info() != Eigen::Success)
    {
        return;
    }

    frame_localizability_eigenvalues =
        field_solver.eigenvalues().cwiseMax(0.0);
    const double trace = frame_localizability_eigenvalues.sum();
    if (trace > 1e-12)
    {
        frame_localizability_f0 =
            frame_localizability_eigenvalues(0) / trace;
    }
    frame_localizability_lambda0 =
        frame_localizability_eigenvalues(0) /
        static_cast<double>(voxels.size());
}

/**
 * @brief 从 IKFoM 后验协方差提取平移/旋转不确定性主方向
 *
 * state_ikfom 的误差状态前六维依次为 position[0:3]、rotation[3:6]。
 * 读取完整滤波协方差的两个边缘块，因此其数值已经包含其它状态及
 * 旋转-平移耦合经滤波传播后的影响。结果只写日志，不反馈到状态估计。
 */
void update_pose_covariance_diagnostics()
{
    frame_translation_cov_eigenvalues.setZero();
    frame_translation_weak_direction.setZero();
    frame_rotation_cov_eigenvalues.setZero();
    frame_rotation_weak_direction.setZero();

    const auto &posterior_covariance = kf.get_P();
    const Eigen::Matrix3d translation_covariance =
        0.5 * (posterior_covariance.block<3, 3>(0, 0) +
               posterior_covariance.block<3, 3>(0, 0).transpose());
    const Eigen::Matrix3d rotation_covariance =
        0.5 * (posterior_covariance.block<3, 3>(3, 3) +
               posterior_covariance.block<3, 3>(3, 3).transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> translation_solver(translation_covariance);
    if (translation_solver.info() == Eigen::Success)
    {
        frame_translation_cov_eigenvalues =
            translation_solver.eigenvalues().cwiseMax(0.0);
        frame_translation_weak_direction =
            translation_solver.eigenvectors().col(2).normalized();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> rotation_solver(rotation_covariance);
    if (rotation_solver.info() == Eigen::Success)
    {
        frame_rotation_cov_eigenvalues =
            rotation_solver.eigenvalues().cwiseMax(0.0);
        frame_rotation_weak_direction =
            rotation_solver.eigenvectors().col(2).normalized();
    }
}

/** @brief 记录当前 LiDAR 扫描覆盖的 IMU 角速度，用于掉头失锁诊断。 */
void update_imu_angular_velocity_diagnostics(const MeasureGroup &meas)
{
    frame_imu_angular_velocity_mean = 0.0;
    frame_imu_angular_velocity_max = 0.0;
    if (meas.imu.empty())
    {
        return;
    }

    double sum = 0.0;
    for (const auto &imu : meas.imu)
    {
        const auto &w = imu->angular_velocity;
        const double norm = std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
        sum += norm;
        frame_imu_angular_velocity_max = std::max(frame_imu_angular_velocity_max, norm);
    }
    frame_imu_angular_velocity_mean = sum / static_cast<double>(meas.imu.size());
}

/** @brief 计算滑窗的 median + scaled MAD 稳健上限。 */
double robust_innovation_threshold(
    const deque<double> &history,
    double absolute_floor)
{
    if (history.empty())
    {
        return absolute_floor;
    }

    std::vector<double> values(history.begin(), history.end());
    const double median = median_of_values(values);
    std::vector<double> deviations;
    deviations.reserve(values.size());
    for (const double value : values)
    {
        deviations.push_back(std::abs(value - median));
    }
    const double mad = median_of_values(deviations);
    return std::max(
        absolute_floor,
        median + transition_guard_mad_scale * 1.4826 * mad);
}

/** @brief 计算 IMU 传播先验到 LiDAR 更新后验的平移/旋转修正量。 */
std::pair<double, double> lidar_pose_correction(
    const state_ikfom &prior,
    const state_ikfom &posterior)
{
    const double translation = (posterior.pos - prior.pos).norm();
    const Eigen::Matrix3d delta_rotation =
        prior.rot.toRotationMatrix().transpose() *
        posterior.rot.toRotationMatrix();
    const double rotation = Eigen::AngleAxisd(delta_rotation).angle();
    return {translation, std::abs(rotation)};
}

/** @brief 在当前帧 ESIKF 更新前清空创新诊断量。 */
void begin_transition_guard_frame()
{
    frame_transition_guard_triggered = false;
    frame_transition_guard_turn_residual_triggered = false;
    frame_transition_guard_map_active = false;
    frame_transition_guard_history_ready = false;
    frame_lidar_nominal_correction_translation = 0.0;
    frame_lidar_nominal_correction_rotation = 0.0;
    frame_lidar_correction_translation_threshold =
        transition_guard_translation_floor;
    frame_lidar_correction_rotation_threshold =
        transition_guard_rotation_floor;
    frame_transition_guard_localizability_drop = 1.0;
    frame_transition_guard_nominal_residual = 0.0;

    if (!transition_guard_enable)
    {
        transition_guard_map_hold_remaining = 0;
        frame_transition_guard_map_hold_remaining = 0;
        transition_guard_translation_history.clear();
        transition_guard_rotation_history.clear();
        transition_guard_f0_history.clear();
        transition_guard_lambda0_history.clear();
        return;
    }

    // 触发后的有限保持期只延续严格入图质量门限，不暂停地图更新。
    if (transition_guard_map_hold_remaining > 0)
    {
        frame_transition_guard_map_active = true;
        --transition_guard_map_hold_remaining;
    }
    frame_transition_guard_map_hold_remaining =
        transition_guard_map_hold_remaining;
}

/**
 * @brief 使用名义 ESIKF 结果检测异常状态创新。
 *
 * 必须同时满足：历史窗口充分、平移/旋转修正量突增，以及可定位性下降、
 * 快速转动或残差升高三种上下文之一。结果只供地图插入策略使用。
 */
bool detect_transition_guard_innovation(
    const state_ikfom &prior,
    const state_ikfom &nominal_posterior)
{
    if (!transition_guard_enable)
    {
        return false;
    }

    const auto correction = lidar_pose_correction(prior, nominal_posterior);
    frame_lidar_nominal_correction_translation = correction.first;
    frame_lidar_nominal_correction_rotation = correction.second;
    frame_lidar_correction_translation_threshold = robust_innovation_threshold(
        transition_guard_translation_history,
        transition_guard_translation_floor);
    frame_lidar_correction_rotation_threshold = robust_innovation_threshold(
        transition_guard_rotation_history,
        transition_guard_rotation_floor);

    frame_transition_guard_history_ready =
        transition_guard_translation_history.size() >=
            static_cast<size_t>(transition_guard_min_history) &&
        transition_guard_rotation_history.size() >=
            static_cast<size_t>(transition_guard_min_history);

    const double f0_baseline = transition_guard_f0_history.empty()
        ? frame_localizability_f0
        : median_of_values(std::vector<double>(
              transition_guard_f0_history.begin(),
              transition_guard_f0_history.end()));
    const double lambda0_baseline = transition_guard_lambda0_history.empty()
        ? frame_localizability_lambda0
        : median_of_values(std::vector<double>(
              transition_guard_lambda0_history.begin(),
              transition_guard_lambda0_history.end()));
    const double f0_ratio = f0_baseline > 1e-9
        ? frame_localizability_f0 / f0_baseline
        : 1.0;
    const double lambda0_ratio = lambda0_baseline > 1e-9
        ? frame_localizability_lambda0 / lambda0_baseline
        : 1.0;
    frame_transition_guard_localizability_drop =
        std::min(f0_ratio, lambda0_ratio);

    const bool innovation_outlier =
        frame_lidar_nominal_correction_translation >
            frame_lidar_correction_translation_threshold ||
        frame_lidar_nominal_correction_rotation >
            frame_lidar_correction_rotation_threshold;
    const bool risky_context =
        frame_transition_guard_localizability_drop <
            transition_guard_localizability_drop_ratio ||
        frame_imu_angular_velocity_max >=
            transition_guard_turn_rate_threshold ||
        res_mean_last >= transition_guard_residual_context_threshold;

    const bool turn_residual_risk =
        frame_imu_angular_velocity_max >=
            transition_guard_turn_rate_threshold &&
        res_mean_last >= transition_guard_residual_context_threshold;

    frame_transition_guard_nominal_residual = res_mean_last;
    const bool innovation_context_trigger =
        frame_transition_guard_history_ready &&
        innovation_outlier && risky_context;
    frame_transition_guard_turn_residual_triggered =
        frame_transition_guard_history_ready && turn_residual_risk;
    frame_transition_guard_triggered =
        innovation_context_trigger ||
        frame_transition_guard_turn_residual_triggered;

    if (frame_transition_guard_triggered)
    {
        frame_transition_guard_map_active = true;
        transition_guard_map_hold_remaining =
            transition_guard_map_hold_frames;
        frame_transition_guard_map_hold_remaining =
            transition_guard_map_hold_remaining;

        if (runtime_console_enable)
        {
            std::cout << std::fixed << std::setprecision(3)
                      << "[InnovationMap] trigger frame=" << (map_update_count + 1)
                      << ", reason="
                      << (frame_transition_guard_turn_residual_triggered
                              ? "turn_residual"
                              : "innovation_context")
                      << ", correction(trans/rot)="
                      << frame_lidar_nominal_correction_translation << "/"
                      << frame_lidar_nominal_correction_rotation
                      << ", threshold="
                      << frame_lidar_correction_translation_threshold << "/"
                      << frame_lidar_correction_rotation_threshold
                      << ", angular_velocity_max="
                      << frame_imu_angular_velocity_max
                      << ", residual=" << res_mean_last
                      << ", localizability_ratio="
                      << frame_transition_guard_localizability_drop
                      << ", map_hold_frames="
                      << transition_guard_map_hold_frames
                      << std::endl;
        }
    }
    return frame_transition_guard_triggered;
}

/** @brief 将本帧最终创新和可定位性加入有界历史窗口。 */
void update_transition_guard_history()
{
    if (!transition_guard_enable || frame_transition_guard_map_active)
    {
        return;
    }

    transition_guard_translation_history.push_back(
        frame_lidar_nominal_correction_translation);
    transition_guard_rotation_history.push_back(
        frame_lidar_nominal_correction_rotation);
    transition_guard_f0_history.push_back(frame_localizability_f0);
    transition_guard_lambda0_history.push_back(frame_localizability_lambda0);

    while (transition_guard_translation_history.size() >
           static_cast<size_t>(transition_guard_history_size))
    {
        transition_guard_translation_history.pop_front();
        transition_guard_rotation_history.pop_front();
        transition_guard_f0_history.pop_front();
        transition_guard_lambda0_history.pop_front();
    }
}

/**
 * @brief 汇总当前帧实验指标并交给独立 CSV logger
 *
 * 本函数只负责从 SLAM 主流程读取已经计算完成的变量，并组装 RuntimeLogRow。
 * CSV 文件打开、表头和格式化写入由 AdaptiveRuntimeLogger 负责。
 *
 * 调用时机必须位于 map_incremental() 完成入图并更新累计计数之后，
 * 否则 map_size、total_map_added 等字段会落后一帧。
 */
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
    int total_rejected_num)
{
    const size_t downsampled_points = feats_down_body->size();
    RuntimeLogRow row;

    // 帧级状态与匹配质量指标。
    row.frame = map_update_count;
    row.lidar_begin_time = Measures.lidar_beg_time;
    row.lidar_end_time = Measures.lidar_end_time;
    row.received_lidar_messages = static_cast<std::uint64_t>(std::max(0, scan_count));
    row.synchronized_scans = static_cast<std::uint64_t>(std::max(0, scan_num));
    row.scan_update_failures = total_scan_update_failures;
    row.imu_angular_velocity_mean = frame_imu_angular_velocity_mean;
    row.imu_angular_velocity_max = frame_imu_angular_velocity_max;
    row.adaptive_map = adaptive_map_enable;
    row.degeneracy_diagnostic = degeneracy_diagnostic_enable;
    row.degenerate = frame_degenerate;
    row.degeneracy_mode = static_cast<int>(current_degeneracy_mode);
    row.window_ready = degeneracy_window_ready;
    row.pos_x = state_point.pos.x();
    row.pos_y = state_point.pos.y();
    row.pos_z = state_point.pos.z();
    const Eigen::Quaterniond orientation(state_point.rot.toRotationMatrix());
    const Eigen::Quaterniond normalized_orientation = orientation.normalized();
    row.quat_x = normalized_orientation.x();
    row.quat_y = normalized_orientation.y();
    row.quat_z = normalized_orientation.z();
    row.quat_w = normalized_orientation.w();
    row.downsampled_points = downsampled_points;
    row.effective_points = effct_feat_num;
    row.effective_ratio = frame_effective_ratio;
    row.residual_mean = res_mean_last;
    row.residual_median = frame_residual_median;
    row.residual_mad = frame_residual_mad;
    row.normal_eigen_ratio = frame_normal_eigen_ratio;
    row.condition_number = frame_condition_number;
    row.localizability_observed_voxels = frame_localizability_observed_voxels;
    row.localizability_planarity_mean = frame_localizability_planarity_mean;
    row.localizability_lambda_min = frame_localizability_eigenvalues(0);
    row.localizability_lambda_mid = frame_localizability_eigenvalues(1);
    row.localizability_lambda_max = frame_localizability_eigenvalues(2);
    row.localizability_f0 = frame_localizability_f0;
    row.localizability_lambda0 = frame_localizability_lambda0;
    row.transition_guard_triggered = frame_transition_guard_triggered;
    row.transition_guard_turn_residual_triggered =
        frame_transition_guard_turn_residual_triggered;
    row.transition_guard_map_active = frame_transition_guard_map_active;
    row.transition_guard_map_hold_remaining =
        frame_transition_guard_map_hold_remaining;
    row.transition_guard_history_ready = frame_transition_guard_history_ready;
    row.lidar_nominal_correction_translation =
        frame_lidar_nominal_correction_translation;
    row.lidar_nominal_correction_rotation =
        frame_lidar_nominal_correction_rotation;
    row.lidar_correction_translation_threshold =
        frame_lidar_correction_translation_threshold;
    row.lidar_correction_rotation_threshold =
        frame_lidar_correction_rotation_threshold;
    row.transition_guard_localizability_drop =
        frame_transition_guard_localizability_drop;
    row.transition_guard_nominal_residual =
        frame_transition_guard_nominal_residual;
    row.translation_cov_eigen_min = frame_translation_cov_eigenvalues(0);
    row.translation_cov_eigen_mid = frame_translation_cov_eigenvalues(1);
    row.translation_cov_eigen_max = frame_translation_cov_eigenvalues(2);
    row.translation_weak_dir_x = frame_translation_weak_direction.x();
    row.translation_weak_dir_y = frame_translation_weak_direction.y();
    row.translation_weak_dir_z = frame_translation_weak_direction.z();
    row.rotation_cov_eigen_min = frame_rotation_cov_eigenvalues(0);
    row.rotation_cov_eigen_mid = frame_rotation_cov_eigenvalues(1);
    row.rotation_cov_eigen_max = frame_rotation_cov_eigenvalues(2);
    row.rotation_weak_dir_x = frame_rotation_weak_direction.x();
    row.rotation_weak_dir_y = frame_rotation_weak_direction.y();
    row.rotation_weak_dir_z = frame_rotation_weak_direction.z();
    row.window_degenerate_ratio = window_degenerate_ratio;
    row.window_normal_eigen_ratio_mean = window_normal_eigen_ratio_mean;
    row.window_residual_cv = window_residual_cv;
    row.window_path_length = window_path_length;
    row.window_yaw_change = window_yaw_change;
    row.window_condition_number_mean = window_condition_number_mean;
    row.window_recent_degenerate_streak = window_recent_degenerate_streak;
    row.window_localizability_f0_mean = window_localizability_f0_mean;
    row.window_localizability_lambda0_mean = window_localizability_lambda0_mean;

    // 本帧地图更新结果。
    row.map_added = add_num;
    row.point_to_add = point_to_add_num;
    row.point_no_need_downsample = point_no_need_downsample_num;
    row.insert_ratio =
        downsampled_points > 0
            ? static_cast<double>(add_num) / static_cast<double>(downsampled_points)
            : 0.0;

    // 本帧不同筛选规则产生的拒绝/接纳数量。
    row.quality_rejected = quality_rejected_num;
    row.invalid_quality_rejected = invalid_quality_rejected_num;
    row.direction_rejected = direction_rejected_num;
    row.persistent_quota_rejected = persistent_quota_rejected_num;
    row.novel_accepted = novel_accepted_num;
    row.novel_rejected = novel_rejected_num;
    row.voxel_rejected = voxel_rejected_num;
    row.total_rejected = total_rejected_num;

    // 写入后的地图规模与程序启动以来的累计统计。
    row.map_size = p_map->size();
    row.total_map_added = total_map_added;
    row.total_quality_rejected = total_quality_rejected;
    row.total_direction_rejected = total_direction_rejected;
    row.total_persistent_quota_rejected = total_persistent_quota_rejected;
    row.total_voxel_rejected = total_voxel_rejected;

    runtime_logger.write(row);
}
