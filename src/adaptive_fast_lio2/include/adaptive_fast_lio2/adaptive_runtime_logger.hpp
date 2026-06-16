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

    // 当前实验是否启用 adaptive_map，以及本帧是否被判定为退化帧。
    bool adaptive_map = false;
    bool degenerate = false;

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

    // 本帧最终入图数量及其组成。insert_ratio = map_added / downsampled_points。
    std::size_t map_added = 0;
    std::size_t point_to_add = 0;
    std::size_t point_no_need_downsample = 0;
    double insert_ratio = 0.0;

    // 本帧各类拒绝/接纳统计，用于分析 adaptive_map 和原始体素筛选的作用。
    int quality_rejected = 0;
    int invalid_quality_rejected = 0;
    int direction_rejected = 0;
    int novel_accepted = 0;
    int novel_rejected = 0;
    int voxel_rejected = 0;
    int total_rejected = 0;

    // 地图规模和从程序启动到当前帧的累计统计。
    std::size_t map_size = 0;
    std::uint64_t total_map_added = 0;
    std::uint64_t total_quality_rejected = 0;
    std::uint64_t total_direction_rejected = 0;
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
