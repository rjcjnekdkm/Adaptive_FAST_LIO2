#include "adaptive_fast_lio2/adaptive_runtime_logger.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>

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
    if (!append_)
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
          << row.received_lidar_messages << ","
          << row.synchronized_scans << ","
          << row.scan_update_failures << ","
          << row.imu_angular_velocity_mean << ","
          << row.imu_angular_velocity_max << ","
          << (row.adaptive_map ? 1 : 0) << ","
          << (row.degeneracy_diagnostic ? 1 : 0) << ","
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
          << row.localizability_observed_voxels << ","
          << row.localizability_planarity_mean << ","
          << row.localizability_lambda_min << ","
          << row.localizability_lambda_mid << ","
          << row.localizability_lambda_max << ","
          << row.localizability_f0 << ","
          << row.localizability_lambda0 << ","
          << (row.transition_guard_triggered ? 1 : 0) << ","
          << (row.transition_guard_turn_residual_triggered ? 1 : 0) << ","
          << (row.transition_guard_map_active ? 1 : 0) << ","
          << row.transition_guard_map_hold_remaining << ","
          << (row.transition_guard_history_ready ? 1 : 0) << ","
          << row.lidar_nominal_correction_translation << ","
          << row.lidar_nominal_correction_rotation << ","
          << row.lidar_correction_translation_threshold << ","
          << row.lidar_correction_rotation_threshold << ","
          << row.transition_guard_localizability_drop << ","
          << row.transition_guard_nominal_residual << ","
          << row.translation_cov_eigen_min << ","
          << row.translation_cov_eigen_mid << ","
          << row.translation_cov_eigen_max << ","
          << row.translation_weak_dir_x << ","
          << row.translation_weak_dir_y << ","
          << row.translation_weak_dir_z << ","
          << row.rotation_cov_eigen_min << ","
          << row.rotation_cov_eigen_mid << ","
          << row.rotation_cov_eigen_max << ","
          << row.rotation_weak_dir_x << ","
          << row.rotation_weak_dir_y << ","
          << row.rotation_weak_dir_z << ","
          << row.window_degenerate_ratio << ","
          << row.window_normal_eigen_ratio_mean << ","
          << row.window_residual_cv << ","
          << row.window_path_length << ","
          << row.window_yaw_change << ","
          << row.window_condition_number_mean << ","
          << row.window_recent_degenerate_streak << ","
          << row.window_localizability_f0_mean << ","
          << row.window_localizability_lambda0_mean << ","
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
          << row.total_voxel_rejected
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
    // 表头顺序需要与 write() 中 RuntimeLogRow 的写入顺序保持一致。
    file_ << "frame,lidar_begin_time,lidar_end_time,"
          << "received_lidar_messages,synchronized_scans,scan_update_failures,"
          << "imu_angular_velocity_mean,imu_angular_velocity_max,"
          << "adaptive_map,degeneracy_diagnostic,degenerate,"
          << "degeneracy_mode,window_ready,"
          << "pos_x,pos_y,pos_z,quat_x,quat_y,quat_z,quat_w,"
          << "downsampled_points,effective_points,effective_ratio,"
          << "residual_mean,residual_median,residual_mad,normal_eigen_ratio,"
          << "condition_number,"
          << "localizability_observed_voxels,localizability_planarity_mean,"
          << "localizability_lambda_min,localizability_lambda_mid,localizability_lambda_max,"
          << "localizability_f0,localizability_lambda0,"
          << "transition_guard_triggered,transition_guard_turn_residual_triggered,"
          << "transition_guard_map_active,transition_guard_map_hold_remaining,"
          << "transition_guard_history_ready,"
          << "lidar_nominal_correction_translation,"
          << "lidar_nominal_correction_rotation,lidar_correction_translation_threshold,"
          << "lidar_correction_rotation_threshold,transition_guard_localizability_drop,"
          << "transition_guard_nominal_residual,"
          << "translation_cov_eigen_min,translation_cov_eigen_mid,translation_cov_eigen_max,"
          << "translation_weak_dir_x,translation_weak_dir_y,translation_weak_dir_z,"
          << "rotation_cov_eigen_min,rotation_cov_eigen_mid,rotation_cov_eigen_max,"
          << "rotation_weak_dir_x,rotation_weak_dir_y,rotation_weak_dir_z,"
          << "window_degenerate_ratio,window_normal_eigen_ratio_mean,"
          << "window_residual_cv,window_path_length,window_yaw_change,"
          << "window_condition_number_mean,window_recent_degenerate_streak,"
          << "window_localizability_f0_mean,window_localizability_lambda0_mean,"
          << "map_added,point_to_add,point_no_need_downsample,insert_ratio,"
          << "quality_rejected,invalid_quality_rejected,direction_rejected,persistent_quota_rejected,"
          << "novel_accepted,novel_rejected,voxel_rejected,total_rejected,"
          << "map_size,total_map_added,total_quality_rejected,"
          << "total_direction_rejected,total_persistent_quota_rejected,total_voxel_rejected"
          << std::endl;
}
