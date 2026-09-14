#include "adaptive_fast_lio2/adaptive_runtime_logger.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>

namespace {
constexpr char kRuntimeCsvHeader[] =
    "frame,lidar_begin_time,lidar_end_time,adaptive_map,degenerate,"
    "degeneracy_mode,window_ready,"
    "pos_x,pos_y,pos_z,quat_x,quat_y,quat_z,quat_w,"
    "downsampled_points,effective_points,effective_ratio,"
    "residual_mean,residual_median,residual_mad,normal_eigen_ratio,"
    "condition_number,"
    "window_degenerate_ratio,window_normal_eigen_ratio_mean,"
    "window_residual_cv,window_path_length,window_yaw_change,"
    "window_condition_number_mean,window_recent_degenerate_streak,"
    "map_added,point_to_add,point_no_need_downsample,insert_ratio,"
    "quality_rejected,invalid_quality_rejected,direction_rejected,persistent_quota_rejected,"
    "novel_accepted,novel_rejected,voxel_rejected,total_rejected,"
    "map_size,total_map_added,total_quality_rejected,"
    "total_direction_rejected,total_persistent_quota_rejected,total_voxel_rejected,"
    "sync_imu_samples,sync_imu_first_time,sync_imu_last_time,frontend_core_revision,"
    "log_sequence,map_update_skipped,map_skip_reason,window_updated,"
    "range_near_rejected,range_far_rejected,map_min_range,map_max_range,"
    "map_min_effective_points,runtime_schema_revision,invalid_quality_filter_enabled,"
    "invalid_quality_low_effective_relax_enabled,invalid_quality_relax_active,"
    "invalid_quality_relax_effective_threshold,"
    "invalid_quality_relaxed,total_invalid_quality_relaxed,"
    "invalid_quality_turn_guard_enabled,invalid_quality_turn_guard_active,"
    "invalid_quality_turn_guard_yaw_threshold,invalid_quality_turn_guard_rejected,"
    "total_invalid_quality_turn_guard_rejected,"
    "directional_selection_enabled,directional_selection_active,"
    "equal_point_count_control_enabled,equal_point_count_control_active,"
    "equal_point_count_target,equal_point_count_rejected,"
    "total_equal_point_count_rejected";
}

AdaptiveRuntimeLogger::~AdaptiveRuntimeLogger()
{
    // 节点正常退出或对象销毁时，确保最后一批缓冲数据写入磁盘。
    close();
}

void AdaptiveRuntimeLogger::configure(bool enable, const std::string &path, bool append)
{
    // 允许重复配置。重新打开前先关闭旧文件，避免多个实验结果写入错误文件。
    close();

    enable_ = enable;
    path_ = path;
    append_ = append;
    ready_ = false;

    if (!enable_)
    {
        return;
    }

    if (path_.empty())
    {
        std::cerr << "[CSV] runtime_log.csv_enable=true but csv_path is empty. Disable CSV logging." << std::endl;
        return;
    }

    const std::filesystem::path csv_path(path_);
    const auto parent_path = csv_path.parent_path();
    if (!parent_path.empty())
    {
        std::error_code ec;
        std::filesystem::create_directories(parent_path, ec);
        if (ec)
        {
            std::cerr << "[CSV] failed to create directory: "
                      << parent_path.string()
                      << " (" << ec.message() << ")" << std::endl;
            return;
        }
    }

    // Do not append the new schema to a historical-core CSV.
    bool needs_header = !append_;
    if (append_)
    {
        std::ifstream existing(path_);
        std::string header;
        if (!std::getline(existing, header) || header.empty())
            needs_header = true;
        else if (header != kRuntimeCsvHeader)
        {
            std::cerr << "[CSV] legacy schema: choose a new shared-core output path." << std::endl;
            return;
        }
    }

    // append=false 用于独立实验：覆盖旧结果并写入新表头；
    // append=true 用于继续记录同一次实验，调用者需要保证已有文件表头一致。
    const std::ios_base::openmode mode =
        std::ios::out | (append_ ? std::ios::app : std::ios::trunc);
    file_.open(path_, mode);
    if (!file_.is_open())
    {
        std::cerr << "[CSV] failed to open: " << path_ << std::endl;
        return;
    }

    ready_ = true;
    if (needs_header)
    {
        // 覆盖模式下文件内容已清空，因此必须先写入列名。
        writeHeader();
    }

    std::cout << "[CSV] runtime statistics -> " << path_
              << (append_ ? " (append)" : " (overwrite)")
              << std::endl;
}

void AdaptiveRuntimeLogger::write(const RuntimeLogRow &row)
{
    // 日志属于实验辅助功能。文件未打开时静默跳过，不能中断建图主流程。
    if (!ready_ || !file_.is_open())
    {
        return;
    }

    // 时间、位置和比例保留 9 位小数，便于后续轨迹对齐和统计分析。
    // 列顺序必须与 writeHeader() 完全一致。
    file_ << std::fixed << std::setprecision(9)
          << row.frame << ","
          << row.lidar_begin_time << ","
          << row.lidar_end_time << ","
          << (row.adaptive_map ? 1 : 0) << ","
          << (row.degenerate ? 1 : 0) << ","
          << row.degeneracy_mode << ","
          << (row.window_ready ? 1 : 0) << ","
          << row.pos_x << ","
          << row.pos_y << ","
          << row.pos_z << ","
          << row.quat_x << ","
          << row.quat_y << ","
          << row.quat_z << ","
          << row.quat_w << ","
          << row.downsampled_points << ","
          << row.effective_points << ","
          << row.effective_ratio << ","
          << row.residual_mean << ","
          << row.residual_median << ","
          << row.residual_mad << ","
          << row.normal_eigen_ratio << ","
          << row.condition_number << ","
          << row.window_degenerate_ratio << ","
          << row.window_normal_eigen_ratio_mean << ","
          << row.window_residual_cv << ","
          << row.window_path_length << ","
          << row.window_yaw_change << ","
          << row.window_condition_number_mean << ","
          << row.window_recent_degenerate_streak << ","
          << row.map_added << ","
          << row.point_to_add << ","
          << row.point_no_need_downsample << ","
          << row.insert_ratio << ","
          << row.quality_rejected << ","
          << row.invalid_quality_rejected << ","
          << row.direction_rejected << ","
          << row.persistent_quota_rejected << ","
          << row.novel_accepted << ","
          << row.novel_rejected << ","
          << row.voxel_rejected << ","
          << row.total_rejected << ","
          << row.map_size << ","
          << row.total_map_added << ","
          << row.total_quality_rejected << ","
          << row.total_direction_rejected << ","
          << row.total_persistent_quota_rejected << ","
          << row.total_voxel_rejected << ","
          << row.sync_imu_samples << ","
          << row.sync_imu_first_time << ","
          << row.sync_imu_last_time << ","
          << row.frontend_core_revision << ","
          << row.log_sequence << ","
          << (row.map_update_skipped ? 1 : 0) << ","
          << row.map_skip_reason << ","
          << (row.window_updated ? 1 : 0) << ","
          << row.range_near_rejected << ","
          << row.range_far_rejected << ","
          << row.map_min_range << ","
          << row.map_max_range << ","
          << row.map_min_effective_points << ","
          << row.runtime_schema_revision << ","
          << (row.invalid_quality_filter_enabled ? 1 : 0) << ","
          << (row.invalid_quality_low_effective_relax_enabled ? 1 : 0) << ","
          << (row.invalid_quality_relax_active ? 1 : 0) << ","
          << row.invalid_quality_relax_effective_threshold << ","
          << row.invalid_quality_relaxed << ","
          << row.total_invalid_quality_relaxed << ","
          << (row.invalid_quality_turn_guard_enabled ? 1 : 0) << ","
          << (row.invalid_quality_turn_guard_active ? 1 : 0) << ","
          << row.invalid_quality_turn_guard_yaw_threshold << ","
          << row.invalid_quality_turn_guard_rejected << ","
          << row.total_invalid_quality_turn_guard_rejected << ","
          << (row.directional_selection_enabled ? 1 : 0) << ","
          << (row.directional_selection_active ? 1 : 0) << ","
          << (row.equal_point_count_control_enabled ? 1 : 0) << ","
          << (row.equal_point_count_control_active ? 1 : 0) << ","
          << row.equal_point_count_target << ","
          << row.equal_point_count_rejected << ","
          << row.total_equal_point_count_rejected
          << std::endl;
}

void AdaptiveRuntimeLogger::close()
{
    if (file_.is_open())
    {
        // 显式刷新，避免用户 Ctrl+C 结束实验时最后几帧仍停留在流缓冲区。
        file_.flush();
        file_.close();
    }
    ready_ = false;
}

bool AdaptiveRuntimeLogger::isReady() const
{
    return ready_;
}

void AdaptiveRuntimeLogger::writeHeader()
{
    file_ << kRuntimeCsvHeader << std::endl;
}
