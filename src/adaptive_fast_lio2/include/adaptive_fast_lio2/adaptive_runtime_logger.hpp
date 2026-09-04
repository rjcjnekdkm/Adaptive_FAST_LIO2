#pragma once

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>

/**
 * @brief 单帧地图更新实验统计
 *
 * 该结构只保存已经计算完成的数据，不参与 SLAM 状态估计和地图筛选。
 * adaptive_laserMapping.cpp 在每次 map_incremental() 完成后填充一行，
 * AdaptiveRuntimeLogger 再按照固定列顺序写入 CSV。
 *
 * 后续增加实验指标时，需要同时修改：
 * 1. 本结构体；
 * 2. AdaptiveRuntimeLogger::write()；
 * 3. AdaptiveRuntimeLogger::writeHeader()；
 * 4. adaptive_laserMapping.cpp 中的 write_runtime_log_row()。
 */
struct RuntimeLogRow
{
    // 帧与时间信息。frame 是成功执行地图增量更新的帧编号。
    int frame = 0;
    double lidar_begin_time = 0.0;
    double lidar_end_time = 0.0;
    // 数据完整性计数：用于区分订阅丢帧、同步缺失和 scan-to-map 更新失败。
    std::uint64_t received_lidar_messages = 0;
    std::uint64_t synchronized_scans = 0;
    std::uint64_t scan_update_failures = 0;
    double imu_angular_velocity_mean = 0.0;
    double imu_angular_velocity_max = 0.0;

    // 当前实验是否启用 adaptive_map，以及本帧是否被判定为退化帧。
    bool adaptive_map = false;
    // 仅检测/记录状态机是否启用；启用时不代表会改变地图插入。
    bool degeneracy_diagnostic = false;
    bool degenerate = false;
    int degeneracy_mode = 0;
    bool window_ready = false;

    // 当前滤波状态在 camera_init/world 坐标系下的位置。
    double pos_x = 0.0;
    double pos_y = 0.0;
    double pos_z = 0.0;
    // 当前滤波状态姿态四元数，顺序为 x、y、z、w。
    double quat_x = 0.0;
    double quat_y = 0.0;
    double quat_z = 0.0;
    double quat_w = 1.0;

    // 当前帧下采样点数、有效点到面匹配数量及有效匹配比例。
    std::size_t downsampled_points = 0;
    int effective_points = 0;
    double effective_ratio = 0.0;

    // 当前帧有效点到面残差统计，以及法向量信息矩阵的最小/最大特征值比例。
    double residual_mean = 0.0;
    double residual_median = 0.0;
    double residual_mad = 0.0;
    double normal_eigen_ratio = 0.0;
    double condition_number = 1.0;

    // 当前 scan-to-map 实际观测到的局部地图体素可定位性场。
    // M = sum(rho_v * n_v * n_v^T)，其中每个地图体素只计一次。
    std::size_t localizability_observed_voxels = 0;
    double localizability_planarity_mean = 0.0;
    double localizability_lambda_min = 0.0;
    double localizability_lambda_mid = 0.0;
    double localizability_lambda_max = 0.0;
    double localizability_f0 = 0.0;
    double localizability_lambda0 = 0.0;

    // 异常创新检测：仅用于诊断和收紧地图写入质量门限。
    bool transition_guard_triggered = false;
    bool transition_guard_turn_residual_triggered = false;
    bool transition_guard_map_active = false;
    int transition_guard_map_hold_remaining = 0;
    bool transition_guard_history_ready = false;
    double lidar_nominal_correction_translation = 0.0;
    double lidar_nominal_correction_rotation = 0.0;
    double lidar_correction_translation_threshold = 0.0;
    double lidar_correction_rotation_threshold = 0.0;
    double transition_guard_localizability_drop = 1.0;
    double transition_guard_nominal_residual = 0.0;

    // IKFoM 后验位姿边缘协方差的主值和最大不确定方向。
    // 平移块单位为 m^2，旋转块单位为 rad^2；仅用于诊断，不参与滤波。
    double translation_cov_eigen_min = 0.0;
    double translation_cov_eigen_mid = 0.0;
    double translation_cov_eigen_max = 0.0;
    double translation_weak_dir_x = 0.0;
    double translation_weak_dir_y = 0.0;
    double translation_weak_dir_z = 0.0;
    double rotation_cov_eigen_min = 0.0;
    double rotation_cov_eigen_mid = 0.0;
    double rotation_cov_eigen_max = 0.0;
    double rotation_weak_dir_x = 0.0;
    double rotation_weak_dir_y = 0.0;
    double rotation_weak_dir_z = 0.0;

    double window_degenerate_ratio = 0.0;
    double window_normal_eigen_ratio_mean = 0.0;
    double window_residual_cv = 0.0;
    double window_path_length = 0.0;
    double window_yaw_change = 0.0;
    double window_condition_number_mean = 1.0;
    int window_recent_degenerate_streak = 0;
    double window_localizability_f0_mean = 0.0;
    double window_localizability_lambda0_mean = 0.0;

    // 本帧最终入图数量及其组成。insert_ratio = map_added / downsampled_points。
    std::size_t map_added = 0;
    std::size_t point_to_add = 0;
    std::size_t point_no_need_downsample = 0;
    double insert_ratio = 0.0;

    // 本帧各类拒绝/接纳统计，用于分析 adaptive_map 和原始体素筛选的作用。
    int quality_rejected = 0;
    int invalid_quality_rejected = 0;
    int direction_rejected = 0;
    // Persistent 总入图配额导致的拒绝数；用于验证持续退化策略是否真实生效。
    int persistent_quota_rejected = 0;
    int novel_accepted = 0;
    int novel_rejected = 0;
    int voxel_rejected = 0;
    int total_rejected = 0;

    // 地图规模和从程序启动到当前帧的累计统计。
    std::size_t map_size = 0;
    std::uint64_t total_map_added = 0;
    std::uint64_t total_quality_rejected = 0;
    std::uint64_t total_direction_rejected = 0;
    std::uint64_t total_persistent_quota_rejected = 0;
    std::uint64_t total_voxel_rejected = 0;
};

/**
 * @brief 自适应地图实验 CSV 记录器
 *
 * 该类仅负责 CSV 文件生命周期和格式化写入，与 ROS2、点云和 ikd-tree 解耦。
 * 记录器关闭或打开失败时，write() 会直接返回，不影响 SLAM 主流程。
 */
class AdaptiveRuntimeLogger
{
public:
    AdaptiveRuntimeLogger() = default;
    ~AdaptiveRuntimeLogger();

    /**
     * @brief 根据参数打开或关闭 CSV 记录
     * @param enable 是否启用记录
     * @param path CSV 输出路径，父目录需要提前存在
     * @param append true 表示追加，false 表示覆盖并重新写表头
     */
    void configure(bool enable, const std::string &path, bool append);

    // 将一帧实验统计按固定列顺序写入 CSV。
    void write(const RuntimeLogRow &row);

    // 刷新缓冲并关闭文件，可重复调用。
    void close();

    // 返回 CSV 文件是否已经成功打开并可写。
    bool isReady() const;

private:
    // 写入与 RuntimeLogRow/ write() 顺序一致的 CSV 表头。
    void writeHeader();

    // enable_ 表示配置要求启用；ready_ 表示文件实际已经成功打开。
    bool enable_ = false;
    bool ready_ = false;
    bool append_ = false;
    std::string path_;
    std::ofstream file_;
};
