#include <omp.h>
#include <algorithm>
#include <mutex>
#include <deque>
#include <vector>
#include <cstdint>
#include <cmath>
#include <iostream>
#include <condition_variable>
#include <unordered_map>
#include <numeric>
#include <iomanip>
#include <sstream>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "adaptive_fast_lio2/adaptive_common.hpp"
#include "adaptive_fast_lio2/adaptive_preprocess.hpp"
#include "adaptive_fast_lio2/adaptive_imu_process.hpp"
#include "adaptive_fast_lio2/adaptive_map_manager.hpp"
#include "adaptive_fast_lio2/adaptive_map_strategy.hpp"
#include "adaptive_fast_lio2/adaptive_frontend_module.hpp"
#include "adaptive_fast_lio2/adaptive_runtime_logger.hpp"
#include "use-ikfom.hpp"


using std::cout;
using std::endl;
using std::deque;
using std::mutex;
using std::condition_variable;
using std::shared_ptr;
using std::string;



// ===================== 全局 buffer =====================
// 三类传感器缓存共用同一把互斥锁，避免回调线程与主处理线程并发修改。
mutex mtx_buffer;
// 新数据到达后用于通知等待线程；当前定时器流程中保留该对象以兼容后续阻塞式处理。
condition_variable sig_buffer;

// 与 lidar_buffer 一一对应，保存每帧 LiDAR 的起始时间戳，单位：秒。
deque<double> time_buffer;
// 等待与 IMU 数据完成时间同步的 LiDAR 帧缓存。
deque<PointCloudXYZI::Ptr> lidar_buffer;
// 按时间顺序缓存的 IMU 消息。
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

// 当前已同步完成、等待里程计处理的一组 LiDAR-IMU 测量。
MeasureGroup Measures;

// ===================== 全局时间变量 =====================
// 最近一帧 LiDAR 和最近一条 IMU 的时间戳，单位：秒；用于检查时间回退。
double last_timestamp_lidar = 0.0;
double last_timestamp_imu = -1.0;
double time_diff_lidar_to_imu = 0.0;  // 手动配置的 LiDAR-IMU 时间偏移

double timediff_lidar_wrt_imu = 0.0; // 自同步估计出的时间差
bool timediff_set_flg = false;  // 是否已经估计过时间差

bool time_sync_en = false;  // 是否启用自同步
// 标记下一帧是否为程序收到的第一帧 LiDAR，第一帧不进行时间回退检查。
bool is_first_lidar = true;

// ===================== 全局计数变量 =====================
// 已进入 LiDAR 回调的点云帧数。
int scan_count = 0;
// 已进入 IMU 回调的消息数，用于控制日志打印频率。
int publish_count = 0;

// ===================== 运行日志参数 =====================
// 每隔多少个成功地图更新帧打印一行核心统计；设为 1 可逐帧观察。
int runtime_log_interval_frames = 10;
// 是否输出 LiDAR/IMU 回调和同步阶段的高频调试信息。
bool runtime_sensor_debug = false;
// 是否向终端输出前端运行日志。关闭后仍保留 CSV 记录和后端日志，便于观察后端回环。
bool runtime_console_enable = true;
// 是否将每帧实验统计写入 CSV，便于后续画曲线和做定量对比。
bool runtime_csv_enable = false;
// CSV 输出路径。建议每组实验单独设置文件名，避免 on/off 结果混在一起。
string runtime_csv_path;
// 是否追加写入已有 CSV。默认覆盖，保证每次实验文件干净。
bool runtime_csv_append = false;
AdaptiveRuntimeLogger runtime_logger;
// 用于在退化状态发生变化时立即输出提示。
bool previous_frame_degenerate = false;
bool has_previous_degenerate_state = false;
int previous_degeneracy_mode = 0;
bool has_previous_degeneracy_mode = false;
// 累计入图统计，便于观察长期地图更新趋势。
uint64_t total_map_added = 0;
uint64_t total_quality_rejected = 0;
uint64_t total_direction_rejected = 0;
uint64_t total_persistent_quota_rejected = 0;
uint64_t total_voxel_rejected = 0;
// scan-to-map 有效点不足而跳过地图更新的累计帧数；仅用于实验完整性诊断。
uint64_t total_scan_update_failures = 0;
size_t last_map_add_num = 0;

// ===================== 全局参数 =====================
// LiDAR 与 IMU 的 ROS2 订阅话题名称。
string lid_topic;
string imu_topic;
// 以下参数对应预处理模块中的雷达类型、扫描线数、抽点间隔、时间单位和扫描频率。
int lidar_type = AVIA;
int scan_line = 6;
int point_filter_num = 3;
int timestamp_unit = US;
int scan_rate = 10;
// IMU 初始化累计样本数量；按数据集 IMU 频率配置初始化时间窗口。
int imu_init_num = 200;

// LiDAR 近距离盲区半径，单位：米。
double blind = 4.0;
// 是否启用传统几何特征提取；FAST-LIO2 默认直接使用原始点，因此通常为 false。
bool feature_extract_enable = false;

// 点云发布控制：世界系点云、稠密点云、LiDAR/body 系点云。
bool scan_publish_en = true;
bool dense_publish_en = true;
bool scan_bodyframe_pub_en = true;
// 是否发布官方风格的累计显示地图 /Laser_map。
bool map_publish_en = false;

std::shared_ptr<Preprocess> p_pre(new Preprocess());                    //点云预处理模块
std::shared_ptr<AdaptiveImuProcess> p_imu(new AdaptiveImuProcess());    //IMU 处理模块
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());              //去畸变后的当前帧点云

// 是否在局部地图移动时删除旧地图点。
// FAST-LIO2 原版会删除移出局部地图 cube 的点。
bool local_map_delete_enable = true;


// ===================== LiDAR / IMU 同步相关变量 =====================
bool lidar_pushed = false; //  当前 lidar_buffer.front() 是否已经取出来等待 IMU
// 当前等待同步的 LiDAR 帧结束时间，单位：秒。
double lidar_end_time = 0.0;
double lidar_mean_scantime = 0.1;  //  如果当前点云没有每点时间，就用平均扫描时间估计帧结束时间

int scan_num = 0; //  用来统计已经处理的 LiDAR 帧数       


// ===================== LiDAR-IMU 外参 =====================
// extrinsic_T: LiDAR 坐标系到 IMU/body 坐标系的平移
// extrinsic_R: LiDAR 坐标系到 IMU/body 坐标系的旋转，按行优先展开为 9 个数
std::vector<double> extrinsic_T_vec = {0.04165, 0.02326, -0.0284};
std::vector<double> extrinsic_R_vec = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
};


// ===================== 当前帧与地图点云 =====================
// 当前帧在 LiDAR 坐标系下的体素下采样结果，用于 scan-to-map 和地图插入。
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
// feats_down_body 按最终估计位姿变换到世界坐标系后的点云。
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
// 当前帧下采样体素大小
double filter_size_surf = 0.5;

// 地图下采样体素大小
double filter_size_map = 0.5;

// 地图发布计数
int map_update_count = 0;
// /Laser_map 发布节流计数；独立于地图是否成功插入，避免更新失败时重复累计同一帧。
int map_publish_frame_count = 0;
// 与官方 FAST-LIO2 的 pcl_wait_pub 一致，仅用于累计发布 /Laser_map。
PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());


// ===================== 自适应地图插入参数 =====================
// 是否启用退化感知地图点筛选；关闭时保持原 FAST-LIO2 的体素插入行为。
bool adaptive_map_enable = false;
// 仅运行退化检测与窗口状态机、写入日志；绝不改变 ESIKF 或地图插入。
// 该开关用于第一阶段的纯诊断实验。
bool degeneracy_diagnostic_enable = false;

// 最终 scan-to-map 有效匹配点数量下限
int adaptive_min_effective_points = 200;

// 允许写入地图的 LiDAR 点量程范围，单位：米。
double adaptive_min_range = 0.5;
double adaptive_max_range = 80.0;

// 单点允许残差上限取“当前帧平均残差 × 倍数”与 plane_residual_threshold 的较小值。
double adaptive_residual_scale = 2.0;
// 稳健残差阈值系数：median + robust_sigma * 1.4826 * MAD。
double adaptive_residual_robust_sigma = 2.5;
// 单点 FAST-LIO2 风格质量分数下限，越接近 1 要求越严格。
double adaptive_min_quality_score = 0.92;
// 有效匹配点占当前帧下采样点的比例下限。
double adaptive_min_effective_ratio = 0.1;
// 当前帧平均点到面绝对残差上限，单位：米。
double adaptive_max_mean_residual = 0.15;
// 法向量信息矩阵最小/最大特征值比例下限，用于识别法向方向过于单一的场景。
double adaptive_min_normal_eigen_ratio = 0.02;
// 法向量方向分箱的角分辨率，单位：度。
double adaptive_normal_bin_angle_deg = 15.0;
// 退化帧中每个法向方向分箱最多允许写入地图的点数。
int adaptive_max_points_per_normal_bin = 30;
// 退化帧中允许写入的地图未知区域点上限，防止完全阻断地图向新区域生长。
int adaptive_max_novel_points_per_frame = 50;

// ===================== 滑动窗口退化判断参数 =====================
// 是否启用滑动窗口
bool adaptive_window_enable = true;
// 窗口长度 K，即最近 K 次 map_incremental() 的统计量参与动态退化判断。
int adaptive_window_size = 20;
// 最近 K 帧中，静态退化帧比例 gamma_t 需要超过该阈值，才可能认为是持续退化。
double adaptive_window_persistent_ratio_threshold = 0.6;
// 窗口平均法向特征值比例阈值放大系数：
// mean(normal_eigen_ratio) < normal_ratio_scale * adaptive_min_normal_eigen_ratio。
double adaptive_window_normal_ratio_scale = 1.5;
// 残差变异系数 CV = std(residual_mean) / mean(residual_mean) 的上限。
// 长走廊/隧道一般表现为“约束方向单一但残差相对稳定”，拐角/突变则波动更大。
double adaptive_window_residual_stability_ratio = 0.5;
// 窗口内累计位移下限，避免机器人静止或小幅抖动时被误判为长时间退化。
double adaptive_window_min_motion = 2.0;
// 窗口内累计 yaw 变化上限。长走廊通常 yaw 变化小；拐角则 yaw 变化大。
double adaptive_window_max_yaw_change = 0.25;
// 最近连续静态退化帧数量下限。用于避免“退化-正常-退化”仅靠比例被误判为持续退化。
int adaptive_window_min_recent_degenerate_streak = 5;
// scan-to-map 约束条件数下限。条件数越大，说明 H^T H 越病态，弱约束方向越明显。
// 默认 1.0 表示先记录和参与公式，但不额外阻断 Persistent 判定；实验后可调高。
double adaptive_window_min_condition_number = 1.0;
// 进入/退出 Persistent 模式的滞回计数，防止模式在边界附近频繁抖动。
int adaptive_window_enter_count_threshold = 3;
int adaptive_window_exit_count_threshold = 5;
// Persistent 模式下进一步收紧入图配额的比例。
// 例如 0.5 表示长走廊/隧道中每个法向方向最多写入的点数减半。
double adaptive_window_persistent_direction_quota_scale = 0.5;
double adaptive_window_persistent_novel_quota_scale = 0.5;
// Persistent 总入图配额：默认 max=0 表示不额外限制，保持既有正式实验行为。
// 启用后 Qp = clamp(round(scale * N_eff), min, max)，候选点已按弱方向贡献/质量排序。
double adaptive_window_persistent_insert_quota_scale = 1.0;
int adaptive_window_persistent_insert_quota_min = 0;
int adaptive_window_persistent_insert_quota_max = 0;

// 最近 K 帧的退化统计量缓存。每次插入新帧，超过 K 后弹出最旧帧。
std::deque<DegeneracyWindowFrame> degeneracy_window;
DegeneracyMode current_degeneracy_mode = DegeneracyMode::Normal;
// 窗口是否已经攒够 K 帧。未 ready 时不允许进入 Persistent。
bool degeneracy_window_ready = false;
// 以下变量是滑动窗口实时统计结果，同时会写入 CSV，便于画曲线验证模块行为。
double window_degenerate_ratio = 0.0;
double window_normal_eigen_ratio_mean = 0.0;
double window_residual_cv = 0.0;
double window_path_length = 0.0;
double window_yaw_change = 0.0;
double window_condition_number_mean = 1.0;
int window_recent_degenerate_streak = 0;
double window_localizability_f0_mean = 0.0;
double window_localizability_lambda0_mean = 0.0;
// Persistent 模式进入/退出计数器，用于状态机滞回。
int persistent_enter_count = 0;
int persistent_exit_count = 0;

// 当前帧实际计算得到的有效匹配比例与法向量特征值比例，用于退化判断和日志。
double frame_effective_ratio = 0.0;
double frame_normal_eigen_ratio = 0.0;
double frame_residual_median = 0.0;
double frame_residual_mad = 0.0;
double frame_condition_number = 1.0;
Eigen::Matrix<double, 6, 1> frame_weak_direction = Eigen::Matrix<double, 6, 1>::Zero();

// 当前 scan-to-map 实际观测到的地图体素法向可定位性场。
// 这些量用于日志，并作为异常创新上下文；不直接修改滤波状态。
size_t frame_localizability_observed_voxels = 0;
double frame_localizability_planarity_mean = 0.0;
Eigen::Vector3d frame_localizability_eigenvalues = Eigen::Vector3d::Zero();
double frame_localizability_f0 = 0.0;
double frame_localizability_lambda0 = 0.0;

// 创新感知地图策略：对 IMU 传播先验到 LiDAR 后验的位姿修正量做
// 滑窗稳健异常检测。该信号只收紧 Adaptive Map 的逐点入图质量门限，
// 不改变 ESIKF 状态、协方差或 LiDAR 测量噪声。
bool transition_guard_enable = false;
int transition_guard_history_size = 30;
int transition_guard_min_history = 15;
double transition_guard_mad_scale = 4.0;
double transition_guard_translation_floor = 0.10;
double transition_guard_rotation_floor = 0.03;
double transition_guard_localizability_drop_ratio = 0.80;
double transition_guard_turn_rate_threshold = 0.30;
double transition_guard_residual_context_threshold = 0.15;
double transition_guard_map_residual_sigma_scale = 0.75;
double transition_guard_map_min_quality_score = 0.94;
int transition_guard_map_hold_frames = 5;
bool frame_transition_guard_triggered = false;
bool frame_transition_guard_turn_residual_triggered = false;
bool frame_transition_guard_map_active = false;
int frame_transition_guard_map_hold_remaining = 0;
int transition_guard_map_hold_remaining = 0;
bool frame_transition_guard_history_ready = false;
double frame_lidar_nominal_correction_translation = 0.0;
double frame_lidar_nominal_correction_rotation = 0.0;
double frame_lidar_correction_translation_threshold = 0.0;
double frame_lidar_correction_rotation_threshold = 0.0;
double frame_transition_guard_localizability_drop = 1.0;
double frame_transition_guard_nominal_residual = 0.0;
deque<double> transition_guard_translation_history;
deque<double> transition_guard_rotation_history;
deque<double> transition_guard_f0_history;
deque<double> transition_guard_lambda0_history;

// 当前扫描对应的 IMU 角速度统计，仅用于验证掉头/快速旋转与失锁的时间关系。
double frame_imu_angular_velocity_mean = 0.0;
double frame_imu_angular_velocity_max = 0.0;

// IKFoM 更新后的位姿边缘协方差诊断量。最大特征值对应最不确定方向。
Eigen::Vector3d frame_translation_cov_eigenvalues = Eigen::Vector3d::Zero();
Eigen::Vector3d frame_translation_weak_direction = Eigen::Vector3d::Zero();
Eigen::Vector3d frame_rotation_cov_eigenvalues = Eigen::Vector3d::Zero();
Eigen::Vector3d frame_rotation_weak_direction = Eigen::Vector3d::Zero();

// h_share_model() 按 feats_down_body 索引生成的逐点质量信息。
// effective 表示该点是否形成有效点到面约束；其余数组分别保存绝对残差、质量分数和世界系平面法向量。
std::vector<uint8_t> map_point_effective;
std::vector<double> map_point_residual_abs;
std::vector<double> map_point_quality_score;
std::vector<Eigen::Vector3d> map_point_normal;
// 每个点对当前最弱 scan-to-map 约束方向的贡献度。Persistent 模式下优先保留贡献度高的点。
std::vector<double> map_point_weak_direction_score;
// 有效点在 feats_down_body 中的原始索引，用于把 h_x 行号映射回逐点质量数组。
std::vector<size_t> effective_point_indices;
// 表示该点在地图中是否存在满足距离阈值的局部近邻，用于区分坏匹配点和新区域点。
std::vector<uint8_t> map_point_has_local_neighbors;

// 与官方 FAST-LIO2 的 Nearest_Points 和 point_selected_surf 对应。
// IKFoM 未收敛时复用最近一次近邻搜索结果，仅重新计算平面残差和雅可比。
std::vector<std::vector<PointType>> nearest_points_cache;
std::vector<uint8_t> point_selected_surf;


// ===================== scan-to-map 残差相关变量 =====================
// 保存的是 body/LiDAR 坐标系下的点。
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI());
// 当前帧有效点对应的平面法向量和残差。 x/y/z 保存平面法向量 normal。intensity 保存点到平面的残差。
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI());
// 有效匹配点数量
int effct_feat_num = 0;
// 当前帧平均残差，用于判断 scan-to-map 匹配质量。
double res_mean_last = 0.0;
// 最近邻点数量。FAST-LIO2 常用 5 个近邻点拟合局部平面。
int nearest_search_num = 5;
// 最近邻最远点距离阈值。如果第 5 个近邻点距离太远，说明局部地图约束不可靠。
double nearest_sq_dist_threshold = 5.0;
// 地图近邻平面拟合阈值，官方 FAST-LIO2 默认使用 0.1 米；同时作为自适应插入残差上限。
double plane_residual_threshold = 0.1;
// 有效约束分数阈值。这个分数参考 FAST-LIO2 中 s 的思想，用于剔除不稳定约束。
double effective_score_threshold = 0.9;


// ===================== scan-to-map 状态更新参数 =====================
// 每帧最多迭代次数
int scan_match_max_iteration = 3;

// 有效残差点数量太少时，不做位姿更新
int scan_match_min_effective_points = 20;

// LiDAR 点到面观测噪声协方差，对应 FAST-LIO2 中的 LASER_POINT_COV。
double laser_point_cov = 0.001;
double gyr_cov = 0.1;
double acc_cov = 0.1;
double b_gyr_cov = 0.0001;
double b_acc_cov = 0.0001;

// ===================== IKFoM state update =====================
// 是否在线估计 LiDAR-IMU 外参。
bool extrinsic_est_en = false;
// IKFoM 各状态分量的迭代收敛阈值。
double ikfom_epsi[23] = {0.001};
// 完整 IKFoM 滤波器实例及其最新状态缓存。
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;

// ===================== 地图管理模块 =====================
// p_map 封装 ikd-tree 的增量插入、最近邻搜索、区域删除与发布缓存刷新。
std::shared_ptr<AdaptiveMapManager> p_map(new AdaptiveMapManager());
AdaptiveMapStrategy adaptive_map_strategy;


// ===================== 地图初始化与局部地图参数 =====================
//
// FAST-LIO2 中不会在地图为空时直接做 scan-to-map。
// 正确流程是：
//   1. 第一帧或前几帧先初始化地图；
//   2. 地图已有足够点之后，才进入 scan-to-map 残差计算；
//   3. 后续再做状态更新和 map_incremental()。

// 地图是否已由首个有效点云完成初始化。
bool flg_map_initialized = false;

// 地图初始化需要的最少当前帧下采样点数。
// 如果当前帧点太少，先不初始化地图。
// 官方 FAST-LIO2 在降采样当前帧点数大于 5 时构建初始 ikd-tree。
int map_init_min_points = 6;

// 局部地图立方体边长。
// FAST-LIO2 中使用局部地图 cube 管理 ikd-tree 中的点。
double cube_len = 1000.0;

// LiDAR 有效探测距离。
// 后续 lasermap_fov_segment() 会根据当前位置和探测距离移动局部地图。
double det_range = 450.0;

// 当 LiDAR 靠近局部地图边界到一定比例时，移动局部地图。
double move_threshold = 1.5;

// 是否启用局部地图范围检查和移动。
bool local_map_enable = true;

// 局部地图中心和边界。
// 使用 Eigen 保存连续坐标边界，删除时再转换为 ikd-tree 的 BoxPointType。
Eigen::Vector3d local_map_center = Eigen::Vector3d::Zero();
Eigen::Vector3d local_map_min = Eigen::Vector3d::Zero();
Eigen::Vector3d local_map_max = Eigen::Vector3d::Zero();



/**
 * @brief ROS2 时间戳转 double 秒
 */
inline double get_time_sec(const builtin_interfaces::msg::Time &stamp)
{
    return static_cast<double>(stamp.sec) +
           static_cast<double>(stamp.nanosec) * 1e-9;
}

/**
 * @brief double 秒转 ROS2 时间戳
 */
inline builtin_interfaces::msg::Time get_ros_time(double t)
{
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(std::floor(t));
    stamp.nanosec = static_cast<uint32_t>((t - stamp.sec) * 1e9);
    return stamp;
}



/**
 * @brief 标准 PointCloud2 回调
 * 1. 读取点云时间戳
 * 2. 调用 p_pre->process()
 * 3. 把处理后的 PointCloudXYZI 放入 lidar_buffer
 */
void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
    mtx_buffer.lock();

    scan_count++;
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();

    // 1. 检查 LiDAR 时间戳是否回退
    if(!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std:: endl;
        lidar_buffer.clear();
        time_buffer.clear();
    }

    if(is_first_lidar)
    {
        is_first_lidar = false;
    }

    last_timestamp_lidar = cur_time;

    // 2. 标准 PointCloud2 点云预处理
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    double lidar_beg_time_offset_sec = 0.0;
    p_pre->process(msg, cloud, &lidar_beg_time_offset_sec);
    cur_time += lidar_beg_time_offset_sec;

    // 3. 放入 LiDAR buffer
    lidar_buffer.push_back(cloud);
    time_buffer.push_back(cur_time);

    double preprocess_time = omp_get_wtime() - preprocess_start_time;

    if (runtime_console_enable && runtime_sensor_debug)
    {
        std::cout << "[Sensor][PointCloud2] frame=" << scan_count
                  << ", time=" << cur_time
                  << ", points=" << cloud->size()
                  << ", lidar_buffer=" << lidar_buffer.size()
                  << ", imu_buffer=" << imu_buffer.size()
                  << ", preprocess_ms=" << preprocess_time * 1000.0
                  << std::endl;
    }
    
    mtx_buffer.unlock();
    sig_buffer.notify_all();

}

/**
 * @brief Livox CustomMsg 回调
 * 1. 读取 Livox 时间戳
 * 2. 检查 LiDAR / IMU 时间差
 * 3. 调用 p_pre->process()
 * 4. 把处理后的 PointCloudXYZI 放入 lidar_buffer
 */
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
{
    mtx_buffer.lock();

    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();

    scan_count++;

    // 1. 检查 LiDAR 时间戳是否回退
    if(!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
        time_buffer.clear();
    }

    if(is_first_lidar)
    {
        is_first_lidar = false;
    }

    // 与官方 FAST-LIO2 一致，后续同步判断使用当前 LiDAR 帧时间。
    last_timestamp_lidar = cur_time;

    // 2. 如果未开启自动同步，但 LiDAR 和 IMU 时间差过大，打印警告
    if (!time_sync_en && std::abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        std::cout << "IMU and LiDAR not synced. "
                  << "IMU time: " << last_timestamp_imu
                  << ", LiDAR time: " << last_timestamp_lidar
                  << std::endl;
    }

    // 3. 如果开启自动同步，估计 LiDAR 相对于 IMU 的时间差
    if (time_sync_en && !timediff_set_flg && std::abs(last_timestamp_lidar - last_timestamp_imu) > 1.0 && !imu_buffer.empty())
    {
        timediff_set_flg = true;

        // 这里的 0.1 是 10Hz LiDAR 的一帧时间近似
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;

        std::cout << "Self sync IMU and LiDAR. time diff: "
                  << timediff_lidar_wrt_imu
                  << std::endl;
    }


    // 4、Livox 预处理
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    p_pre->process(msg,cloud);

    // 5. 放入 LiDAR buffer
    lidar_buffer.push_back(cloud);
    time_buffer.push_back(cur_time);
    

    double preprocess_time = omp_get_wtime() - preprocess_start_time;

    if (runtime_console_enable && runtime_sensor_debug)
    {
        std::cout << "[Sensor][Livox] frame=" << scan_count
                  << ", raw_points=" << msg->point_num
                  << ", points=" << cloud->size()
                  << ", lidar_buffer=" << lidar_buffer.size()
                  << ", imu_buffer=" << imu_buffer.size()
                  << ", preprocess_ms=" << preprocess_time * 1000.0
                  << std::endl;
    }

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}


/**
 * @brief IMU 回调
 * 1. 复制一份 IMU 消息
 * 2. 根据 time_diff_lidar_to_imu 做时间修正
 * 3. 放入 imu_buffer
 */
void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    publish_count++;

    // 1. 复制一份 IMU 消息
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    // 2. 先减去手动配置的 LiDAR-IMU 时间偏移
    double corrected_time = get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu;
    msg->header.stamp = get_ros_time(corrected_time);

    // 3. 如果开启 time_sync_en，并且已经估计出 timediff_lidar_wrt_imu，
    //    再根据自同步结果修正 IMU 时间
     if (std::abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        double synced_time = get_time_sec(msg_in->header.stamp) + timediff_lidar_wrt_imu;

        msg->header.stamp = get_ros_time(synced_time);
    }

    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    // 4. 检查 IMU 时间是否回退
    if (timestamp < last_timestamp_imu)
    {
        std::cerr << "imu loop back, clear buffer" << std::endl;
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;
    
    // 5. 放入 IMU buffer
    imu_buffer.push_back(msg);

    if (runtime_console_enable && runtime_sensor_debug && publish_count % 200 == 0)
    {
        std::cout << "[IMU] count=" << publish_count
                  << " time=" << timestamp
                  << " imu_buffer=" << imu_buffer.size()
                  << std::endl;
    }

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}



/**
 * @brief 同步一帧 LiDAR 和对应时间段内的 IMU
 * 输入来自全局 buffer：
 *   lidar_buffer
 *   time_buffer
 *   imu_buffer
 *
 * 输出：
 *   meas.lidar          当前帧点云
 *   meas.imu            当前帧 LiDAR 结束时刻之前的 IMU
 *   meas.lidar_beg_time 当前帧 LiDAR 开始时间
 *   meas.lidar_end_time 当前帧 LiDAR 结束时间
 */
bool sync_packages(MeasureGroup &meas)
{
    // 1、如果Lidar 或 imu buffer为空，不能同步
    if(lidar_buffer.empty() || imu_buffer.empty())
    {
        return false;
    }

    // 2、如果还没取出当前lidar帧，则取出lidar_buffer.front()
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();

        if(meas.lidar == nullptr || meas.lidar->empty())
        {
            lidar_buffer.pop_front();
            time_buffer.pop_front();
            return false;
        }


        // 3. 估计当前 LiDAR 帧结束时间
        //
        // Livox 点云中，每个点的 curvature 存的是该点相对帧起始时刻的时间，单位 ms。
        // 所以最后一个点 curvature / 1000.0 就是当前帧持续时间，单位秒。
        //
        // 如果 curvature 太小，说明没有有效每点时间，例如标准 PointCloud2 第一版。
        // 这时使用 lidar_mean_scantime 或默认 0.1 秒估计。
        // 标准 PointCloud2 的存储顺序不一定严格按点时间排列，因此不能
        // 假设最后一个点就是扫描结束点。取最大相对时间可兼容 Velodyne、
        // Ouster 与 Livox 点云，并避免低估当前帧持续时间。
        double last_point_time = 0.0;
        for (const auto &point : meas.lidar->points)
        {
            last_point_time = std::max(
                last_point_time,
                static_cast<double>(point.curvature) / 1000.0);
        }
        if(last_point_time < 0.5 * lidar_mean_scantime || last_point_time <= 0.0)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num++;
            lidar_end_time = meas.lidar_beg_time + last_point_time;

            // 更新平均扫描时间
            // 第一阶段可以简单平均
            lidar_mean_scantime += (last_point_time - lidar_mean_scantime) / static_cast<double>(scan_num);
        }

        meas.lidar_end_time = lidar_end_time;
        lidar_pushed = true;
    }

    // 4、如果最新imu时间还没有覆盖到当前lidar结束时间，暂时不能处理
    double newest_imu_time = get_time_sec(imu_buffer.back()->header.stamp);
    if(newest_imu_time < lidar_end_time)
    {
        return false;
    }

    // 5、取出当前lidar结束时间之前的所有imu
    meas.imu.clear();
    while (!imu_buffer.empty())
    {
        double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if(imu_time > lidar_end_time)
        {
            break;
        }

        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    // 6、当前lidar帧已经同步完成，从buffer中弹出
    lidar_buffer.pop_front();
    time_buffer.pop_front();

    lidar_pushed = false;

    return true;
}





/**
 * @brief 将 LiDAR/body 点转换到 world 坐标系
 * 当前输入点 pi 是 LiDAR 坐标系下的点。
 *   p_imu = offset_R_L_I * p_lidar + offset_T_L_I
 *   p_world = rot * p_imu + pos
 */
void pointBodyToWorld(const PointType &pi, PointType &po)
{
    Eigen::Vector3d p_lidar(pi.x, pi.y, pi.z);

    // LiDAR 坐标系 -> IMU/body 坐标系
    Eigen::Vector3d p_imu =
        state_point.offset_R_L_I.toRotationMatrix() * p_lidar +
        state_point.offset_T_L_I;

    // IMU/body 坐标系 -> world 坐标系
    Eigen::Vector3d p_world =
        state_point.rot.toRotationMatrix() * p_imu +
        state_point.pos;

    po = pi;
    po.x = static_cast<float>(p_world.x());
    po.y = static_cast<float>(p_world.y());
    po.z = static_cast<float>(p_world.z());
}

/**
 * @brief 将配置文件中的 LiDAR-IMU 外参写入 IKFoM 初始状态
 */
void setInitialExtrinsicToIkfom(
    const Eigen::Matrix3d &extrinsic_R,
    const Eigen::Vector3d &extrinsic_T)
{
    state_ikfom ikfom_state = kf.get_x();
    ikfom_state.offset_R_L_I = SO3(extrinsic_R);
    ikfom_state.offset_T_L_I = extrinsic_T;
    kf.change_x(ikfom_state);
    state_point = ikfom_state;
}

/**
 * @brief 从 IKFoM 滤波器读取最新状态到全局 state_point。
 */
void refreshStatePoint()
{
    state_point = kf.get_x();
}

/**
 * @brief 计算两个 PCL 点的欧氏距离平方
 */
float pointDistanceSquared(const PointType &a, const PointType &b)
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;

    return dx * dx + dy * dy + dz * dz;
}

/**
 * @brief 构造三维向量的反对称矩阵
 */
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d m;

    m << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;

    return m;
}


/**
 * @brief 根据地图近邻点拟合局部平面
 *
 * 输入：
 *   nearest_points:
 *     当前点在地图中的最近邻点，一般取 5 个。
 *
 * 输出：
 *   plane_normal:
 *     单位平面法向量。
 *
 *   plane_d:
 *     平面方程中的 d。
 *
 * 平面方程：
 *   n.x * X + n.y * Y + n.z * Z + d = 0
 *
 * 返回：
 *   true  表示平面拟合成功，并且近邻点确实近似共面；
 *   false 表示拟合失败或局部点不成平面。
 *
 * 注意：
 *   这不是特征提取。
 *   这里不是从当前帧中提边缘点/平面点。
 *   这是 FAST-LIO2 direct scan-to-map 中，为了构造点到局部地图平面的残差而做的局部平面估计。
 */
bool estimate_plane_from_neighbors(const std::vector<PointType> &nearest_points, Eigen::Vector3d &plane_normal, double &plane_d)
{
    if (nearest_points.size() < static_cast<size_t>(nearest_search_num))
    {
        return false;
    }

    // 与官方 esti_plane() 一致：求解 n_raw · p = -1，再归一化得到平面方程。
    Eigen::MatrixXd A(nearest_search_num, 3);
    Eigen::VectorXd b = Eigen::VectorXd::Constant(nearest_search_num, -1.0);
    for (int i = 0; i < nearest_search_num; ++i)
    {
        A(i, 0) = nearest_points[i].x;
        A(i, 1) = nearest_points[i].y;
        A(i, 2) = nearest_points[i].z;
    }

    const Eigen::Vector3d raw_normal = A.colPivHouseholderQr().solve(b);
    const double normal_norm = raw_normal.norm();
    if (!std::isfinite(normal_norm) || normal_norm < 1e-9)
    {
        return false;
    }

    plane_normal = raw_normal / normal_norm;
    plane_d = 1.0 / normal_norm;

    for (int i = 0; i < nearest_search_num; ++i)
    {
        const Eigen::Vector3d p(
            nearest_points[i].x,
            nearest_points[i].y,
            nearest_points[i].z);
        if (std::abs(plane_normal.dot(p) + plane_d) > plane_residual_threshold)
        {
            return false;
        }
    }
    return true;
}


/**
 * @brief 当前帧体素下采样
 *
 * 对应 FAST-LIO2 里对 feats_undistort 做 downSizeFilterSurf。
 */
void downsample_current_scan(const PointCloudXYZI::Ptr &cloud_in)
{
    feats_down_body->clear();

    if(cloud_in == nullptr || cloud_in->empty())
    {
        return;
    }

    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setInputCloud(cloud_in);
    voxel_filter.setLeafSize(
        static_cast<float>(filter_size_surf),
        static_cast<float>(filter_size_surf),
        static_cast<float>(filter_size_surf)
    );

    voxel_filter.filter(*feats_down_body);
}

/**
 * @brief FAST-LIO2 官方形式的 IKFoM 点到面观测模型
 *
 * 单个函数完成最近邻搜索、局部平面拟合、有效点筛选、逐点地图质量记录，
 * 以及 IKFoM 观测雅可比和残差向量构造。
 */
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    laserCloudOri->clear();
    corr_normvect->clear();

    effct_feat_num = 0;
    res_mean_last = 0.0;
    begin_adaptive_scan_match(
        feats_down_body == nullptr ? 0 : feats_down_body->size());

    if (feats_down_body == nullptr || feats_down_body->empty())
    {
        ekfom_data.valid = false;
        return;
    }

    feats_down_world->clear();
    feats_down_world->resize(feats_down_body->size());
    nearest_points_cache.resize(feats_down_body->size());
    point_selected_surf.resize(feats_down_body->size(), 0);

    if (p_map == nullptr || p_map->empty())
    {
        ekfom_data.valid = false;
        return;
    }

    // R/t：IMU 到世界坐标系的当前位姿；R_li/t_li：LiDAR 到 IMU 的外参。
    const Eigen::Matrix3d R = s.rot.toRotationMatrix();
    const Eigen::Matrix3d R_li = s.offset_R_L_I.toRotationMatrix();
    const Eigen::Vector3d t(
        s.pos[0],
        s.pos[1],
        s.pos[2]);
    const Eigen::Vector3d t_li(
        s.offset_T_L_I[0],
        s.offset_T_L_I[1],
        s.offset_T_L_I[2]);

    double residual_sum = 0.0;
    std::vector<double> effective_residuals;
    effective_residuals.reserve(feats_down_body->size());

    for (size_t i = 0; i < feats_down_body->size(); ++i)
    {
        const PointType &point_body = feats_down_body->points[i];

        // p_body、p_imu、p_world_eigen 分别表示同一点在 LiDAR、IMU、世界坐标系下的位置。
        Eigen::Vector3d p_body(
            point_body.x,
            point_body.y,
            point_body.z);

        Eigen::Vector3d p_imu = R_li * p_body + t_li;
        Eigen::Vector3d p_world_eigen = R * p_imu + t;

        PointType point_world = point_body;
        point_world.x = static_cast<float>(p_world_eigen.x());
        point_world.y = static_cast<float>(p_world_eigen.y());
        point_world.z = static_cast<float>(p_world_eigen.z());
        feats_down_world->points[i] = point_world;

        std::vector<PointType> &nearest_points = nearest_points_cache[i];

        if (ekfom_data.converge)
        {
            std::vector<float> squared_distances;
            const bool search_success =
                p_map->nearestSearch(
                    point_world,
                    nearest_search_num,
                    nearest_points,
                    squared_distances);

            point_selected_surf[i] =
                search_success &&
                nearest_points.size() >= static_cast<size_t>(nearest_search_num) &&
                !squared_distances.empty() &&
                squared_distances.back() <= nearest_sq_dist_threshold;
            record_adaptive_neighbor(i, point_selected_surf[i] != 0);
        }

        if (point_selected_surf[i] == 0)
        {
            continue;
        }

        Eigen::Vector3d plane_normal;
        double plane_d = 0.0;

        point_selected_surf[i] = 0;
        if (!estimate_plane_from_neighbors(nearest_points, plane_normal, plane_d))
        {
            continue;
        }

        const double residual =
            plane_normal.dot(p_world_eigen) + plane_d;

        const double point_range =
            std::max(p_body.norm(), 1e-6);

        // FAST-LIO2 风格有效性分数：相同残差下，远距离点获得略宽松的容忍度。
        const double score =
            1.0 - 0.9 * std::abs(residual) / std::sqrt(point_range);

        // 官方条件为 s > 0.9，因此等于阈值时也应判为无效点。
        if (score <= effective_score_threshold)
        {
            continue;
        }

        point_selected_surf[i] = 1;
        record_adaptive_effective_match(
            i, std::abs(residual), score, plane_normal);

        PointType normal_residual;
        normal_residual.x = static_cast<float>(plane_normal.x());
        normal_residual.y = static_cast<float>(plane_normal.y());
        normal_residual.z = static_cast<float>(plane_normal.z());
        normal_residual.intensity = static_cast<float>(residual);

        laserCloudOri->push_back(point_body);
        corr_normvect->push_back(normal_residual);
        residual_sum += std::abs(residual);
        effective_residuals.push_back(std::abs(residual));
        effct_feat_num++;
    }

    // 与官方 FAST-LIO2 一致：没有任何有效点时才将本次观测标记为无效。
    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        return;
    }

    update_residual_diagnostics(residual_sum, effective_residuals);

    ekfom_data.valid = true;
    // h_x 是点到面观测对状态的雅可比；h 是取负后的残差观测向量。
    ekfom_data.h_x =
        Eigen::MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; ++i)
    {
        const PointType &laser_p = laserCloudOri->points[i];
        const PointType &norm_p = corr_normvect->points[i];

        Eigen::Vector3d point_body(
            laser_p.x,
            laser_p.y,
            laser_p.z);

        Eigen::Matrix3d point_body_cross =
            skewSymmetric(point_body);

        Eigen::Vector3d point_imu =
            R_li * point_body + t_li;

        Eigen::Matrix3d point_imu_cross =
            skewSymmetric(point_imu);

        Eigen::Vector3d normal_world(
            norm_p.x,
            norm_p.y,
            norm_p.z);

        // C 为世界系法向量旋转到 IMU 系后的表达；A/B 分别对应姿态和外参旋转雅可比项。
        Eigen::Vector3d C =
            R.transpose() * normal_world;

        Eigen::Vector3d A =
            point_imu_cross * C;

        if (extrinsic_est_en)
        {
            Eigen::Vector3d B =
                point_body_cross * R_li.transpose() * C;

            ekfom_data.h_x.block<1, 12>(i, 0) <<
                norm_p.x, norm_p.y, norm_p.z,
                A.x(), A.y(), A.z(),
                B.x(), B.y(), B.z(),
                C.x(), C.y(), C.z();
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i, 0) <<
                norm_p.x, norm_p.y, norm_p.z,
                A.x(), A.y(), A.z(),
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0;
        }

        ekfom_data.h(i) = -norm_p.intensity;
    }

    update_scan_observability_diagnostics(
        ekfom_data.h_x.block(0, 0, effct_feat_num, 6));

}


/**
 * @brief 使用当前帧初始化地图
 *
 * 对应 FAST-LIO2 中地图未初始化时：
 *   1. 将当前帧点云从 body/LiDAR 坐标系转换到 world 坐标系；
 *   2. 加入地图；
 *   3. 初始化局部地图范围。
 *
 * p_map 内部通过 ikd-tree Build/Add_Points 完成首帧地图构建。
 */
bool init_map_with_current_scan()
{
    if (feats_down_body == nullptr || feats_down_body->empty())
    {
        std::cout << "[MapInit] feats_down_body is empty." << std::endl;
        return false;
    }

    if (static_cast<int>(feats_down_body->size()) < map_init_min_points)
    {
        std::cout << "[MapInit] not enough points. current="
                  << feats_down_body->size()
                  << ", required=" << map_init_min_points
                  << std::endl;
        return false;
    }

    feats_down_world->clear();
    feats_down_world->resize(feats_down_body->size());

    PointCloudXYZI::Ptr points_to_init(new PointCloudXYZI());
    points_to_init->reserve(feats_down_body->size());

    for (size_t i = 0; i < feats_down_body->size(); ++i)
    {
        PointType point_world;

        // pointBodyToWorld() 内部已经使用：
        //   LiDAR -> IMU/body 外参
        //   IMU/body -> world 当前状态
        pointBodyToWorld(feats_down_body->points[i], point_world);

        feats_down_world->points[i] = point_world;
        points_to_init->push_back(point_world);
    }

    // 初始化地图。
    // 当前 p_map->addPoints() 内部会累积点云、体素滤波、重建 KdTree。
    p_map->addPoints(points_to_init, true);

    // 官方 FAST-LIO2 使用 LiDAR 在世界坐标系下的位置初始化局部地图。
    local_map_center =
        state_point.pos +
        state_point.rot.toRotationMatrix() * state_point.offset_T_L_I;

    local_map_min = local_map_center - Eigen::Vector3d(cube_len / 2.0, cube_len / 2.0, cube_len / 2.0);

    local_map_max = local_map_center + Eigen::Vector3d(cube_len / 2.0, cube_len / 2.0, cube_len / 2.0);

    flg_map_initialized = true;

    std::cout << "[MapInit] initialized. "
              << "points=" << points_to_init->size()
              << ", map_size=" << p_map->size()
              << ", center=" << local_map_center.transpose()
              << std::endl;

    return true;
}

/**
 * @brief 局部地图/FOV 管理
 *
 * 作用：
 *   1. 维护一个局部地图 cube；
 *   2. 当 LiDAR 位置靠近 cube 边界时，移动 cube；
 *   3. 计算旧 cube 中需要删除的区域；
 *   4. 调用 p_map->Delete_Point_Boxes() 删除这些区域中的地图点。
 *
 * p_map 内部调用 ikd-tree 的 Delete_Point_Boxes() 完成增量区域删除。
 */
void lasermap_fov_segment()
{
    if (!local_map_enable)
    {
        return;
    }

    if (!flg_map_initialized)
    {
        return;
    }

    /**
     * FAST-LIO2 中用当前 LiDAR/IMU 的位置判断是否靠近局部地图边界。
     * 当前状态 pos 表示 IMU/body 在 world 下的位置。
     */
    Eigen::Vector3d pos_lidar =
        state_point.pos +
        state_point.rot.toRotationMatrix() * state_point.offset_T_L_I;

    /**
     * 局部地图尚未初始化时，根据当前位置初始化。
     */
    if ((local_map_max - local_map_min).norm() < 1e-6)
    {
        local_map_center = pos_lidar;

        local_map_min =
            local_map_center -
            Eigen::Vector3d(cube_len / 2.0, cube_len / 2.0, cube_len / 2.0);

        local_map_max =
            local_map_center +
            Eigen::Vector3d(cube_len / 2.0, cube_len / 2.0, cube_len / 2.0);
    }

    const Eigen::Vector3d old_center = local_map_center;
    const Eigen::Vector3d old_min = local_map_min;
    const Eigen::Vector3d old_max = local_map_max;

    Eigen::Vector3d new_center = old_center;

    bool need_move = false;

    // 与官方 FAST-LIO2 一致，根据 cube 和探测距离计算每次移动距离。
    const double move_dist =
        std::max(
            (cube_len - 2.0 * move_threshold * det_range) * 0.5 * 0.9,
            det_range * (move_threshold - 1.0));
    const double edge_threshold = move_threshold * det_range;

    for (int axis = 0; axis < 3; ++axis)
    {
        const double dist_to_min =
            pos_lidar(axis) - local_map_min(axis);

        const double dist_to_max =
            local_map_max(axis) - pos_lidar(axis);

        if (dist_to_min <= edge_threshold)
        {
            new_center(axis) -= move_dist;
            need_move = true;
        }
        else if (dist_to_max <= edge_threshold)
        {
            new_center(axis) += move_dist;
            need_move = true;
        }
    }

    if (!need_move)
    {
        return;
    }

    /**
     * 根据新中心计算新的局部地图 cube。
     */
    const Eigen::Vector3d new_min =
        new_center -
        Eigen::Vector3d(cube_len / 2.0, cube_len / 2.0, cube_len / 2.0);

    const Eigen::Vector3d new_max =
        new_center +
        Eigen::Vector3d(cube_len / 2.0, cube_len / 2.0, cube_len / 2.0);

    /**
     * 生成需要删除的 box。
     *
     * 思路：
     *   旧 cube 移动到新 cube 后，旧 cube 中不再属于新 cube 的区域应该删除。
     *
     * 对每个轴：
     *   - 如果新 cube 向正方向移动：
     *       删除旧 cube 负方向留下的 slab；
     *   - 如果新 cube 向负方向移动：
     *       删除旧 cube 正方向留下的 slab。
     *
     * 这些 slab 就是 FAST-LIO2 中需要传给 Delete_Point_Boxes() 的区域。
     */
    std::vector<BoxPointType> boxes_to_delete;

    for (int axis = 0; axis < 3; ++axis)
    {
        if (new_min(axis) > old_min(axis))
        {
            /**
             * 局部地图沿该轴正方向移动。
             * 旧 cube 的低端区域 [old_min, new_min] 不再需要。
             */
            BoxPointType box;

            for (int i = 0; i < 3; ++i)
            {
                box.vertex_min[i] = static_cast<float>(old_min(i));
                box.vertex_max[i] = static_cast<float>(old_max(i));
            }
            box.vertex_max[axis] = static_cast<float>(new_min(axis));

            boxes_to_delete.push_back(box);
        }
        else if (new_max(axis) < old_max(axis))
        {
            /**
             * 局部地图沿该轴负方向移动。
             * 旧 cube 的高端区域 [new_max, old_max] 不再需要。
             */
            BoxPointType box;

            for (int i = 0; i < 3; ++i)
            {
                box.vertex_min[i] = static_cast<float>(old_min(i));
                box.vertex_max[i] = static_cast<float>(old_max(i));
            }
            box.vertex_min[axis] = static_cast<float>(new_max(axis));

            boxes_to_delete.push_back(box);
        }
    }

    /**
     * 更新当前局部地图范围。
     */
    local_map_center = new_center;
    local_map_min = new_min;
    local_map_max = new_max;

    int deleted_points = 0;

    if (local_map_delete_enable && p_map != nullptr)
    {
        deleted_points =
            p_map->Delete_Point_Boxes(boxes_to_delete);
    }

    std::cout << "[LocalMap] moved. "
              << "old_center=" << old_center.transpose()
              << ", new_center=" << local_map_center.transpose()
              << ", delete_boxes=" << boxes_to_delete.size()
              << ", deleted_points=" << deleted_points
              << ", map_size=" << p_map->size()
              << std::endl;
}




/**
 * @brief 地图增量更新入口
 *
 * 当前实现流程：
 * 1. 当前帧点从 LiDAR/body 系转到 world 系；
 * 2. 根据 adaptive_map 判断是否允许插入；
 * 3. 保留 FAST-LIO2 原有的体素冗余判断；
 * 4. 将允许插入的点交给 p_map，并通过 ikd-tree Add_Points 增量更新地图。
 *
 * 注意：
 * adaptive_map_enable=false 时保持原始 FAST-LIO2 的地图点插入行为。
 */
void map_incremental()
{
    if(feats_down_body->empty())
    {
        return;
    }

    feats_down_world->clear();
    feats_down_world->resize(feats_down_body->size());

    PointCloudXYZI::Ptr point_to_add(new PointCloudXYZI());
    PointCloudXYZI::Ptr point_no_need_downsample(new PointCloudXYZI());

    point_to_add->reserve(feats_down_body->size());
    point_no_need_downsample->reserve(feats_down_body->size());

    const AdaptiveMapUpdatePlan adaptive_plan =
        prepare_adaptive_map_update();

    int rejected_num = 0;
    int voxel_rejected_num = 0;

    for(const size_t i : adaptive_plan.candidate_indices)
    {
        const PointType &point_body = feats_down_body->points[i];

        PointType point_world;
        pointBodyToWorld(point_body,point_world);
        // 保持与 feats_down_body 相同索引，便于逐点质量信息对齐。
        feats_down_world->points[i] = point_world;

        bool need_add = true;
        bool no_need_downsample = false;

        // 与官方 map_incremental() 一致，复用 h_share_model() 最终迭代得到的近邻。
        const std::vector<PointType> *nearest_points = nullptr;
        if (i < nearest_points_cache.size() &&
            !nearest_points_cache[i].empty())
        {
            nearest_points = &nearest_points_cache[i];
        }

        if (nearest_points != nullptr)
        {
            PointType mid_point;
            mid_point.x =
                static_cast<float>(std::floor(point_world.x / filter_size_map) * filter_size_map +
                                   0.5 * filter_size_map);
            mid_point.y =
                static_cast<float>(std::floor(point_world.y / filter_size_map) * filter_size_map +
                                   0.5 * filter_size_map);
            mid_point.z =
                static_cast<float>(std::floor(point_world.z / filter_size_map) * filter_size_map +
                                   0.5 * filter_size_map);

            const float point_dist_to_voxel_center =
                pointDistanceSquared(point_world, mid_point);

            const PointType &nearest_point = nearest_points->front();
            const double half_voxel = 0.5 * filter_size_map;

            if (std::fabs(nearest_point.x - mid_point.x) > half_voxel &&
                std::fabs(nearest_point.y - mid_point.y) > half_voxel &&
                std::fabs(nearest_point.z - mid_point.z) > half_voxel)
            {
                no_need_downsample = true;
            }
            else
            {
                for (int j = 0;
                     j < nearest_search_num &&
                     j < static_cast<int>(nearest_points->size());
                     ++j)
                {
                    if (pointDistanceSquared((*nearest_points)[j], mid_point) <
                        point_dist_to_voxel_center)
                    {
                        need_add = false;
                        break;
                    }
                }
            }
        }

        if (!need_add)
        {
            voxel_rejected_num++;
            continue;
        }

        const bool allow_insert =
            allow_adaptive_map_insert(adaptive_plan, i, point_body);

        if (!allow_insert)
        {
            rejected_num++;
            continue;
        }

        if (no_need_downsample)
        {
            point_no_need_downsample->push_back(point_world);
        }
        else
        {
            point_to_add->push_back(point_world);
        }
    }

    p_map->addPoints(point_to_add, true);
    p_map->addPoints(point_no_need_downsample, false);

    const size_t add_num =
        point_to_add->size() + point_no_need_downsample->size();
    finish_adaptive_map_update(
        adaptive_plan,
        add_num,
        point_to_add->size(),
        point_no_need_downsample->size(),
        voxel_rejected_num,
        rejected_num);

}


/**
 * @brief 节点
 *
 * 当前只负责：
 * 1. 声明参数
 * 2. 读取参数
 * 3. 创建 LiDAR / IMU 订阅器
 * 4. 定时打印 buffer 状态
 */
class AdaptiveLaserMappingNode:public rclcpp::Node
{
public:
    AdaptiveLaserMappingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()):Node("adaptive_laser_mapping",options)
    {
        // =========== 参数声明 =============
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);

        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 6);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<double>("preprocess.blind", 4.0);

        this->declare_parameter<int>("point_filter_num", 3);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter<bool>("publish.map_en", false);

        this->declare_parameter<int>("runtime_log.summary_interval_frames", 10);
        this->declare_parameter<bool>("runtime_log.sensor_debug", false);
        this->declare_parameter<bool>("runtime_log.console_enable", true);
        this->declare_parameter<bool>("runtime_log.csv_enable", false);
        this->declare_parameter<string>("runtime_log.csv_path", "");
        this->declare_parameter<bool>("runtime_log.csv_append", false);

        this->declare_parameter<double>("mapping.filter_size_surf", 0.5);
        this->declare_parameter<double>("mapping.filter_size_map", 0.5);

        this->declare_parameter<std::vector<double>>("mapping.extrinsic_T", std::vector<double>{0.04165, 0.02326, -0.0284});
        this->declare_parameter<std::vector<double>>(
            "mapping.extrinsic_R",
            std::vector<double>{
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0});

        this->declare_parameter<int>("mapping.nearest_search_num", 5);
        this->declare_parameter<double>("mapping.nearest_sq_dist_threshold", 5.0);
        this->declare_parameter<double>("mapping.plane_residual_threshold", 0.1);
        this->declare_parameter<double>("mapping.effective_score_threshold", 0.9);
        this->declare_parameter<int>("mapping.scan_match_max_iteration", 3);
        this->declare_parameter<int>("mapping.scan_match_min_effective_points", 20);
        this->declare_parameter<double>("mapping.laser_point_cov", 0.001);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<bool>("transition_guard.enable", false);
        this->declare_parameter<int>("transition_guard.history_size", 30);
        this->declare_parameter<int>("transition_guard.min_history", 15);
        this->declare_parameter<double>("transition_guard.mad_scale", 4.0);
        this->declare_parameter<double>("transition_guard.translation_floor", 0.10);
        this->declare_parameter<double>("transition_guard.rotation_floor", 0.03);
        this->declare_parameter<double>("transition_guard.localizability_drop_ratio", 0.80);
        this->declare_parameter<double>("transition_guard.turn_rate_threshold", 0.30);
        this->declare_parameter<double>("transition_guard.residual_context_threshold", 0.15);
        this->declare_parameter<double>("transition_guard.map_residual_sigma_scale", 0.75);
        this->declare_parameter<double>("transition_guard.map_min_quality_score", 0.94);
        this->declare_parameter<int>("transition_guard.map_hold_frames", 5);
        this->declare_parameter<int>("mapping.imu_init_num", 200);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", false);

        this->declare_parameter<int>("mapping.map_init_min_points", 6);
        this->declare_parameter<double>("mapping.cube_len", 1000.0);
        this->declare_parameter<double>("mapping.det_range", 450.0);
        this->declare_parameter<double>("mapping.move_threshold", 1.5);
        this->declare_parameter<bool>("mapping.local_map_enable", true);

        this->declare_parameter<bool>("mapping.local_map_delete_enable", true);

        // ===================== idea 模块参数声明 =====================
        //
        // adaptive_map：单帧退化感知地图更新。
        //   逐点插入策略位于独立 AdaptiveMapStrategy 模块。
        //   主要控制“当前帧哪些点允许写入 ikd-tree 地图”。
        //
        // adaptive_window：动态滑动窗口退化判断。
        //   作用位置：update_degeneracy_window()。
        //   主要控制 Normal / Transient / Persistent 模式切换。

        // 是否启用 idea 模块的退化感知地图更新；关闭时尽量保持原 FAST-LIO2 入图行为。
        this->declare_parameter<bool>("adaptive_map.enable", false);
        // 第一阶段纯诊断：运行退化标签和窗口状态机，但不改变地图插入或滤波更新。
        this->declare_parameter<bool>("degeneracy_diagnostic.enable", false);
        // 单帧有效点到面约束数量下限，低于该值认为当前帧几何约束不足。
        this->declare_parameter<int>("adaptive_map.min_effective_points", 200);
        // 允许入图的最小 LiDAR 量程，过滤盲区附近和过近的不稳定点。
        this->declare_parameter<double>("adaptive_map.min_range", 0.20);
        // 允许入图的最大 LiDAR 量程，过滤远距离噪声和稀疏弱约束点。
        this->declare_parameter<double>("adaptive_map.max_range", 80.0);
        // 单点残差阈值相对当前帧平均残差的放大系数。
        this->declare_parameter<double>("adaptive_map.residual_scale", 2.0);
        // 稳健残差阈值系数：median + sigma * 1.4826 * MAD。
        this->declare_parameter<double>("adaptive_map.residual_robust_sigma", 2.5);
        // 单点 FAST-LIO2 风格质量分数下限，低于该值拒绝入图。
        this->declare_parameter<double>("adaptive_map.min_quality_score", 0.92);
        // 有效约束点占当前帧下采样点数的比例下限，用于单帧退化判断。
        this->declare_parameter<double>("adaptive_map.min_effective_ratio", 0.1);
        // 当前帧平均点到面绝对残差上限，超过则认为匹配质量差。
        this->declare_parameter<double>("adaptive_map.max_mean_residual", 0.15);
        // 法向信息矩阵最小/最大特征值比例下限，过低表示法向方向覆盖不足。
        this->declare_parameter<double>("adaptive_map.min_normal_eigen_ratio", 0.02);
        // 法向方向分箱角度，退化帧中用于限制重复方向点写入地图。
        this->declare_parameter<double>("adaptive_map.normal_bin_angle_deg", 15.0);
        // 退化帧内每个法向方向分箱最多允许写入地图的点数。
        this->declare_parameter<int>("adaptive_map.max_points_per_normal_bin", 30);
        // 退化帧内地图未知区域 novel points 的帧级入图上限。
        this->declare_parameter<int>("adaptive_map.max_novel_points_per_frame", 50);

        // 是否启用动态滑动窗口判断；关闭时只使用单帧静态退化判断。
        this->declare_parameter<bool>("adaptive_window.enable", true);
        // 窗口长度 K，统计最近 K 次地图更新的退化状态和质量指标。
        this->declare_parameter<int>("adaptive_window.size", 20);
        // 窗口内静态退化帧比例阈值，超过才可能进入持续退化候选。
        this->declare_parameter<double>("adaptive_window.persistent_ratio_threshold", 0.6);
        // 窗口平均法向特征值比例阈值放大系数：mean(rho) < scale * rho_min。
        this->declare_parameter<double>("adaptive_window.normal_ratio_scale", 1.5);
        // 窗口残差变异系数上限，用于判断残差是否稳定。
        this->declare_parameter<double>("adaptive_window.residual_stability_ratio", 0.5);
        // 窗口内累计位移下限，避免静止或小幅抖动误判为持续退化。
        this->declare_parameter<double>("adaptive_window.min_motion", 2.0);
        // 窗口内累计 yaw 变化上限，用于区分直走廊/隧道和拐角。
        this->declare_parameter<double>("adaptive_window.max_yaw_change", 0.25);
        // 最近连续静态退化帧数量下限，避免退化-正常-退化被误判为持续退化。
        this->declare_parameter<int>("adaptive_window.min_recent_degenerate_streak", 5);
        // 窗口平均 scan-to-map 条件数下限；默认 1.0 表示先记录，不额外限制。
        this->declare_parameter<double>("adaptive_window.min_condition_number", 1.0);
        // 连续满足持续退化候选多少次后进入 Persistent 模式。
        this->declare_parameter<int>("adaptive_window.enter_count_threshold", 3);
        // 连续不满足持续退化候选多少次后退出 Persistent 模式。
        this->declare_parameter<int>("adaptive_window.exit_count_threshold", 5);
        // Persistent 模式下，每个法向方向入图配额的缩放比例。
        this->declare_parameter<double>("adaptive_window.persistent_direction_quota_scale", 0.5);
        // Persistent 模式下，novel points 入图配额的缩放比例。
        this->declare_parameter<double>("adaptive_window.persistent_novel_quota_scale", 0.5);
        // Persistent 总入图上限。max=0 时关闭，保持旧版行为；启用时按有效约束数
        // 计算候选配额，并通过 min/max 进行夹紧。
        this->declare_parameter<double>("adaptive_window.persistent_insert_quota_scale", 1.0);
        this->declare_parameter<int>("adaptive_window.persistent_insert_quota_min", 0);
        this->declare_parameter<int>("adaptive_window.persistent_insert_quota_max", 0);
        // ===================== 参数读取 =====================
        this->get_parameter("common.lid_topic", lid_topic);
        this->get_parameter("common.imu_topic", imu_topic);
        this->get_parameter("common.time_sync_en", time_sync_en);
        this->get_parameter("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu);

        this->get_parameter("preprocess.lidar_type", p_pre->lidar_type);
        this->get_parameter("preprocess.scan_line", p_pre->N_SCANS);
        this->get_parameter("preprocess.timestamp_unit", p_pre->time_unit);
        this->get_parameter("preprocess.scan_rate", p_pre->SCAN_RATE);
        this->get_parameter("preprocess.blind", p_pre->blind);

        this->get_parameter("point_filter_num", p_pre->point_filter_num);
        this->get_parameter("feature_extract_enable", p_pre->feature_enabled);
        this->get_parameter("publish.scan_publish_en", scan_publish_en);
        this->get_parameter("publish.dense_publish_en", dense_publish_en);
        this->get_parameter("publish.scan_bodyframe_pub_en", scan_bodyframe_pub_en);
        this->get_parameter("publish.map_en", map_publish_en);

        this->get_parameter("runtime_log.summary_interval_frames", runtime_log_interval_frames);
        this->get_parameter("runtime_log.sensor_debug", runtime_sensor_debug);
        this->get_parameter("runtime_log.console_enable", runtime_console_enable);
        this->get_parameter("runtime_log.csv_enable", runtime_csv_enable);
        this->get_parameter("runtime_log.csv_path", runtime_csv_path);
        this->get_parameter("runtime_log.csv_append", runtime_csv_append);
        runtime_log_interval_frames = std::max(runtime_log_interval_frames, 1);

        this->get_parameter("mapping.filter_size_surf", filter_size_surf);
        this->get_parameter("mapping.filter_size_map", filter_size_map);
        this->get_parameter("transition_guard.enable", transition_guard_enable);
        this->get_parameter("transition_guard.history_size", transition_guard_history_size);
        this->get_parameter("transition_guard.min_history", transition_guard_min_history);
        this->get_parameter("transition_guard.mad_scale", transition_guard_mad_scale);
        this->get_parameter("transition_guard.translation_floor", transition_guard_translation_floor);
        this->get_parameter("transition_guard.rotation_floor", transition_guard_rotation_floor);
        this->get_parameter(
            "transition_guard.localizability_drop_ratio",
            transition_guard_localizability_drop_ratio);
        this->get_parameter(
            "transition_guard.turn_rate_threshold",
            transition_guard_turn_rate_threshold);
        this->get_parameter(
            "transition_guard.residual_context_threshold",
            transition_guard_residual_context_threshold);
        this->get_parameter(
            "transition_guard.map_residual_sigma_scale",
            transition_guard_map_residual_sigma_scale);
        this->get_parameter(
            "transition_guard.map_min_quality_score",
            transition_guard_map_min_quality_score);
        this->get_parameter(
            "transition_guard.map_hold_frames",
            transition_guard_map_hold_frames);
        transition_guard_history_size = std::max(5, transition_guard_history_size);
        transition_guard_min_history = std::clamp(
            transition_guard_min_history, 3, transition_guard_history_size);
        transition_guard_mad_scale = std::max(0.0, transition_guard_mad_scale);
        transition_guard_translation_floor = std::max(
            0.0, transition_guard_translation_floor);
        transition_guard_rotation_floor = std::max(
            0.0, transition_guard_rotation_floor);
        transition_guard_localizability_drop_ratio = std::clamp(
            transition_guard_localizability_drop_ratio, 0.0, 1.0);
        transition_guard_turn_rate_threshold = std::max(
            0.0, transition_guard_turn_rate_threshold);
        transition_guard_residual_context_threshold = std::max(
            0.0, transition_guard_residual_context_threshold);
        transition_guard_map_residual_sigma_scale = std::clamp(
            transition_guard_map_residual_sigma_scale, 0.1, 1.0);
        transition_guard_map_min_quality_score = std::clamp(
            transition_guard_map_min_quality_score, 0.0, 1.0);
        transition_guard_map_hold_frames = std::max(
            0, transition_guard_map_hold_frames);

        // ===================== idea 模块参数读取：adaptive_map =====================
        //
        // 这些参数会写入全局变量，并在两个地方使用：
        //   1. is_current_frame_degenerate()：判断当前帧是否静态退化；
        //   2. AdaptiveMapStrategy：决定候选点是否允许进入 ikd-tree 地图。
        this->get_parameter("adaptive_map.enable", adaptive_map_enable);
        this->get_parameter("degeneracy_diagnostic.enable", degeneracy_diagnostic_enable);
        this->get_parameter("adaptive_map.min_effective_points", adaptive_min_effective_points);
        this->get_parameter("adaptive_map.min_range", adaptive_min_range);
        this->get_parameter("adaptive_map.max_range", adaptive_max_range);
        this->get_parameter("adaptive_map.residual_scale", adaptive_residual_scale);
        this->get_parameter("adaptive_map.residual_robust_sigma", adaptive_residual_robust_sigma);
        this->get_parameter("adaptive_map.min_quality_score", adaptive_min_quality_score);
        this->get_parameter("adaptive_map.min_effective_ratio", adaptive_min_effective_ratio);
        this->get_parameter("adaptive_map.max_mean_residual", adaptive_max_mean_residual);
        this->get_parameter("adaptive_map.min_normal_eigen_ratio", adaptive_min_normal_eigen_ratio);
        this->get_parameter("adaptive_map.normal_bin_angle_deg", adaptive_normal_bin_angle_deg);
        this->get_parameter("adaptive_map.max_points_per_normal_bin", adaptive_max_points_per_normal_bin);
        this->get_parameter("adaptive_map.max_novel_points_per_frame", adaptive_max_novel_points_per_frame);

        // ===================== idea 模块参数读取：adaptive_window =====================
        //
        // 这些参数控制滑动窗口动态退化判断：
        //   - size / persistent_ratio_threshold / normal_ratio_scale / residual_stability_ratio
        //     决定是否形成持续退化候选；
        //   - min_motion / max_yaw_change 用来区分长直走廊和拐角；
        //   - min_recent_degenerate_streak 修正单纯比例判断不连续的问题；
        //   - min_condition_number 用 scan-to-map 病态程度辅助确认退化；
        //   - enter_count_threshold / exit_count_threshold 用来做状态机滞回；
        //   - persistent_*_quota_scale 会在 Persistent 模式下收紧入图配额。
        this->get_parameter("adaptive_window.enable", adaptive_window_enable);
        this->get_parameter("adaptive_window.size", adaptive_window_size);
        this->get_parameter("adaptive_window.persistent_ratio_threshold", adaptive_window_persistent_ratio_threshold);
        this->get_parameter("adaptive_window.normal_ratio_scale", adaptive_window_normal_ratio_scale);
        this->get_parameter("adaptive_window.residual_stability_ratio", adaptive_window_residual_stability_ratio);
        this->get_parameter("adaptive_window.min_motion", adaptive_window_min_motion);
        this->get_parameter("adaptive_window.max_yaw_change", adaptive_window_max_yaw_change);
        this->get_parameter("adaptive_window.min_recent_degenerate_streak", adaptive_window_min_recent_degenerate_streak);
        this->get_parameter("adaptive_window.min_condition_number", adaptive_window_min_condition_number);
        this->get_parameter("adaptive_window.enter_count_threshold", adaptive_window_enter_count_threshold);
        this->get_parameter("adaptive_window.exit_count_threshold", adaptive_window_exit_count_threshold);
        this->get_parameter("adaptive_window.persistent_direction_quota_scale", adaptive_window_persistent_direction_quota_scale);
        this->get_parameter("adaptive_window.persistent_novel_quota_scale", adaptive_window_persistent_novel_quota_scale);
        this->get_parameter("adaptive_window.persistent_insert_quota_scale", adaptive_window_persistent_insert_quota_scale);
        this->get_parameter("adaptive_window.persistent_insert_quota_min", adaptive_window_persistent_insert_quota_min);
        this->get_parameter("adaptive_window.persistent_insert_quota_max", adaptive_window_persistent_insert_quota_max);

        // 防御性修正：窗口长度和滞回计数至少为 1，避免配置错误导致除零或状态机失效。
        adaptive_window_size = std::max(1, adaptive_window_size);
        adaptive_window_min_recent_degenerate_streak =
            std::max(1, adaptive_window_min_recent_degenerate_streak);
        adaptive_window_enter_count_threshold = std::max(1, adaptive_window_enter_count_threshold);
        adaptive_window_exit_count_threshold = std::max(1, adaptive_window_exit_count_threshold);

        AdaptiveMapStrategyConfig strategy_config;
        strategy_config.enabled = adaptive_map_enable;
        strategy_config.min_range = adaptive_min_range;
        strategy_config.max_range = adaptive_max_range;
        strategy_config.residual_scale = adaptive_residual_scale;
        strategy_config.residual_robust_sigma = adaptive_residual_robust_sigma;
        strategy_config.min_quality_score = adaptive_min_quality_score;
        strategy_config.normal_bin_angle_deg = adaptive_normal_bin_angle_deg;
        strategy_config.max_points_per_normal_bin =
            adaptive_max_points_per_normal_bin;
        strategy_config.max_novel_points_per_frame =
            adaptive_max_novel_points_per_frame;
        strategy_config.persistent_direction_quota_scale =
            adaptive_window_persistent_direction_quota_scale;
        strategy_config.persistent_novel_quota_scale =
            adaptive_window_persistent_novel_quota_scale;
        strategy_config.persistent_insert_quota_scale =
            adaptive_window_persistent_insert_quota_scale;
        strategy_config.persistent_insert_quota_min =
            adaptive_window_persistent_insert_quota_min;
        strategy_config.persistent_insert_quota_max =
            adaptive_window_persistent_insert_quota_max;
        strategy_config.transition_guard_enabled = transition_guard_enable;
        strategy_config.transition_guard_residual_sigma_scale =
            transition_guard_map_residual_sigma_scale;
        strategy_config.transition_guard_min_quality_score =
            transition_guard_map_min_quality_score;
        this->get_parameter("mapping.extrinsic_T", extrinsic_T_vec);
        this->get_parameter("mapping.extrinsic_R", extrinsic_R_vec);

        this->get_parameter("mapping.nearest_search_num", nearest_search_num);
        this->get_parameter("mapping.nearest_sq_dist_threshold", nearest_sq_dist_threshold);
        this->get_parameter("mapping.plane_residual_threshold", plane_residual_threshold);
        this->get_parameter("mapping.effective_score_threshold", effective_score_threshold);

        this->get_parameter("mapping.scan_match_max_iteration", scan_match_max_iteration);
        this->get_parameter("mapping.scan_match_min_effective_points", scan_match_min_effective_points);
        this->get_parameter("mapping.laser_point_cov", laser_point_cov);
        this->get_parameter("mapping.gyr_cov", gyr_cov);
        this->get_parameter("mapping.acc_cov", acc_cov);
        this->get_parameter("mapping.b_gyr_cov", b_gyr_cov);
        this->get_parameter("mapping.b_acc_cov", b_acc_cov);
        this->get_parameter("mapping.imu_init_num", imu_init_num);
        this->get_parameter("mapping.extrinsic_est_en", extrinsic_est_en);

        this->get_parameter("mapping.map_init_min_points", map_init_min_points);
        this->get_parameter("mapping.cube_len", cube_len);
        this->get_parameter("mapping.det_range", det_range);
        this->get_parameter("mapping.move_threshold", move_threshold);
        this->get_parameter("mapping.local_map_enable", local_map_enable);
        this->get_parameter("mapping.local_map_delete_enable", local_map_delete_enable);

        strategy_config.plane_residual_threshold = plane_residual_threshold;
        adaptive_map_strategy.configure(strategy_config);

        Eigen::Vector3d extrinsic_T = Eigen::Vector3d::Zero();
        Eigen::Matrix3d extrinsic_R = Eigen::Matrix3d::Identity();

        if (extrinsic_T_vec.size() == 3)
        {
            extrinsic_T << extrinsic_T_vec[0],
                   extrinsic_T_vec[1],
                   extrinsic_T_vec[2];
        }
        else
        {
            std::cerr << "[Param] mapping.extrinsic_T size is not 3. Use zero translation." << std::endl;
        }

        if (extrinsic_R_vec.size() == 9)
        {       
            extrinsic_R << extrinsic_R_vec[0], extrinsic_R_vec[1], extrinsic_R_vec[2],
                        extrinsic_R_vec[3], extrinsic_R_vec[4], extrinsic_R_vec[5],
                        extrinsic_R_vec[6], extrinsic_R_vec[7], extrinsic_R_vec[8];
        }
        else
        {
            std::cerr << "[Param] mapping.extrinsic_R size is not 9. Use identity rotation." << std::endl;
        }

        // 把外参写入 IMU 处理模块的状态中。
        // IMU 模块和 IKFoM 使用相同外参初值。
        p_imu->setExtrinsic(extrinsic_R, extrinsic_T);        
        p_imu->setNoiseCovariances(
            gyr_cov,
            acc_cov,
            b_gyr_cov,
            b_acc_cov);
        p_imu->setInitializationSampleCount(imu_init_num);
        setInitialExtrinsicToIkfom(extrinsic_R, extrinsic_T);
        kf.init_dyn_share(
            get_f,
            df_dx,
            df_dw,
            h_share_model,
            scan_match_max_iteration,
            ikfom_epsi);
        
        p_map->setFilterSizeMap(filter_size_map);

        std::cout << "[Config] topics lidar=" << lid_topic
                  << ", imu=" << imu_topic
                  << "; lidar_type=" << p_pre->lidar_type
                  << ", scan_line=" << p_pre->N_SCANS
                  << ", point_filter=" << p_pre->point_filter_num
                  << ", blind=" << p_pre->blind
                  << std::endl;
        std::cout << "[Config] voxel scan/map=" << filter_size_surf
                  << "/" << filter_size_map
                  << ", nearest=" << nearest_search_num
                  << ", imu_init_num=" << imu_init_num
                  << ", extrinsic_est=" << extrinsic_est_en
                  << ", local_map/delete=" << local_map_enable
                  << "/" << local_map_delete_enable
                  << std::endl;
        std::cout << "[Config] adaptive_map=" << adaptive_map_enable
                  << ", diagnostic_only=" << degeneracy_diagnostic_enable
                  << ", effective_min=" << adaptive_min_effective_points
                  << ", effective_ratio_min=" << adaptive_min_effective_ratio
                  << ", residual_max=" << adaptive_max_mean_residual
                  << ", normal_ratio_min=" << adaptive_min_normal_eigen_ratio
                  << ", quality_min=" << adaptive_min_quality_score
                  << std::endl;
        std::cout << "[Config] transition_guard=" << transition_guard_enable
                  << ", history=" << transition_guard_min_history
                  << "/" << transition_guard_history_size
                  << ", innovation_floor(trans/rot)="
                  << transition_guard_translation_floor << "/"
                  << transition_guard_rotation_floor
                  << ", mad_scale=" << transition_guard_mad_scale
                  << ", localizability_drop="
                  << transition_guard_localizability_drop_ratio
                  << ", map_residual_sigma_scale="
                  << transition_guard_map_residual_sigma_scale
                  << ", map_quality_min="
                  << transition_guard_map_min_quality_score
                  << ", map_hold_frames="
                  << transition_guard_map_hold_frames
                  << std::endl;
        // 打印 idea 动态窗口参数：
        //   persistent_ratio 表示窗口内退化帧比例阈值；
        //   normal_scale 表示窗口平均法向覆盖度阈值系数；
        //   residual_cv_max 表示残差稳定性要求；
        //   min_motion/max_yaw 用于识别长直持续退化而非拐角；
        //   min_streak/condition_min 用于增强持续退化确认；
        //   persistent_quota_scale 用于 Persistent 模式下收紧入图配额。
        std::cout << "[Config] adaptive_window=" << adaptive_window_enable
                  << ", size=" << adaptive_window_size
                  << ", persistent_ratio=" << adaptive_window_persistent_ratio_threshold
                  << ", normal_scale=" << adaptive_window_normal_ratio_scale
                  << ", residual_cv_max=" << adaptive_window_residual_stability_ratio
                  << ", min_motion=" << adaptive_window_min_motion
                  << ", max_yaw=" << adaptive_window_max_yaw_change
                  << ", min_streak=" << adaptive_window_min_recent_degenerate_streak
                  << ", condition_min=" << adaptive_window_min_condition_number
                  << ", enter/exit=" << adaptive_window_enter_count_threshold
                  << "/" << adaptive_window_exit_count_threshold
                  << ", persistent_quota_scale(dir/novel)="
                  << adaptive_window_persistent_direction_quota_scale
                  << "/"
                  << adaptive_window_persistent_novel_quota_scale
                  << std::endl;
        std::cout << "[Config] runtime summary_interval_frames="
                  << runtime_log_interval_frames
                  << ", sensor_debug=" << runtime_sensor_debug
                  << ", console_enable=" << runtime_console_enable
                  << ", csv_enable=" << runtime_csv_enable
                  << std::endl;
        std::cout << "[Config] extrinsic_T=" << extrinsic_T.transpose()
                  << std::endl;

        // CSV logger 是独立实验辅助模块；关闭或打开失败不会影响建图主流程。
        runtime_logger.configure(
            runtime_csv_enable,
            runtime_csv_path,
            runtime_csv_append);

         // ===================== 订阅初始化 =====================

        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                lid_topic, 20, livox_pcl_cbk);

            std::cout << "Subscribe Livox CustomMsg." << std::endl;        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);

            std::cout << "Subscribe standard PointCloud2." << std::endl;
        }

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10, imu_cbk);

       
        pub_cloud_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body",10);
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry",10);
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/path",10);
        path_.header.frame_id = "camera_init";
        pub_cloud_world_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered",10);
        pub_map_ =this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map",10);
        pub_ikdtree_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ikdtree_map", 10);
        pub_degeneracy_info_ =
            this->create_publisher<std_msgs::msg::Float64MultiArray>("/adaptive_frontend/degeneracy_info", 10);

        std::cout << "Publish topics: /cloud_registered_body, /cloud_registered, /Laser_map, /ikdtree_map, /Odometry, /path, /adaptive_frontend/degeneracy_info" << std::endl;

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);


        main_timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(10),
                std::bind(
                    &AdaptiveLaserMappingNode::timer_callback,
                    this));

        std::cout << "Adaptive FAST-LIO2 data input initialized." << std::endl;

    }

    ~AdaptiveLaserMappingNode()
    {
        runtime_logger.close();
    }


private:
    // 传感器订阅器：IMU、标准 PointCloud2、Livox CustomMsg。
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;

    // 里程计运行结果发布器：当前帧点云、里程计、轨迹、世界系点云和局部地图。
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_body_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_world_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
    // 实际用于最近邻匹配的 ikd-tree 有效地图，与累计显示用 /Laser_map 区分。
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ikdtree_map_;
    // 前端退化先验输出，供后端关键帧选择、回环验证和图优化加权使用。
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_degeneracy_info_;

    // 周期触发同步、状态传播、scan-to-map 和地图更新的主定时器。
    rclcpp::TimerBase::SharedPtr main_timer_;

    // 累积发布的位姿轨迹，坐标系固定为 camera_init。
    nav_msgs::msg::Path path_;

    // 发布 world/camera_init 到 body 的 TF 变换。
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void timer_callback()
    {
        bool synced = false;
        {
            std::lock_guard<std::mutex> lock(mtx_buffer);
            synced = sync_packages(Measures);
        }

        if(!synced)
        {
            return;
        }

        if (runtime_console_enable && runtime_sensor_debug)
        {
            std::cout << "[Sensor][Sync] lidar_points=" << Measures.lidar->size()
                      << ", imu_size=" << Measures.imu.size()
                      << ", lidar_beg=" << Measures.lidar_beg_time
                      << ", lidar_end=" << Measures.lidar_end_time
                      << std::endl;
        }

        
        // 1. 使用 IKFoM 完成 IMU 状态传播和点云去畸变。
        p_imu->Process(Measures, kf, feats_undistort);
        update_imu_angular_velocity_diagnostics(Measures);
        refreshStatePoint();

        if (!p_imu->isInitialized())
        {
            publish_current_cloud_body(Measures);
            publish_odometry(Measures);
            publish_path(Measures);
            publish_tf(Measures);
            return;
        }

        // 2、当前帧下采样
        downsample_current_scan(feats_undistort);

        // 3. 如果地图还没有初始化，先用当前帧初始化地图。
        //    地图初始化完成前，不做 scan-to-map 状态更新。
        if (!flg_map_initialized)
        {
            bool init_success = init_map_with_current_scan();

            publish_current_cloud_body(Measures);
            publish_current_cloud_world(Measures);
            publish_map(Measures);
            publish_odometry(Measures);
            publish_path(Measures);
            publish_tf(Measures);

            if (!init_success)
            {
                return;
            }

            return;
        }

        // 4、局部地图管理
        lasermap_fov_segment();

        // 5. 保存 IMU 传播先验，执行且仅执行一次原始 LiDAR 更新。
        // 先验到后验的异常创新只作为后续地图写入质量信号。
        const auto state_before_lidar_update = kf.get_x();
        begin_transition_guard_frame();
        double solve_H_time = 0.0;
        kf.update_iterated_dyn_share_modified(
            laser_point_cov,
            solve_H_time);

        update_localizability_diagnostics();
        detect_transition_guard_innovation(
            state_before_lidar_update,
            kf.get_x());

        update_transition_guard_history();

        refreshStatePoint();
        update_pose_covariance_diagnostics();

        // adaptive_map 关闭时保持官方 FAST-LIO2 行为：只要观测模型存在有效点，
        // 就使用滤波后的最终状态继续增量更新地图。额外的有效点数量门槛仅属于
        // 自适应地图质量控制，不能改变基础复现路径。
        const bool scan_update_success =
            !adaptive_map_enable ||
            effct_feat_num >= scan_match_min_effective_points;

        if(!scan_update_success)
        {
            total_scan_update_failures++;
            std::cout << "[Warning][ScanToMap] update failed: effective="
                      << effct_feat_num
                      << ", required=" << scan_match_min_effective_points
                      << ", solve_H_ms=" << solve_H_time * 1000.0
                      << ". Skip map insertion."
                      << std::endl;

            publish_current_cloud_body(Measures);
            publish_current_cloud_world(Measures);
            publish_map(Measures);
            publish_odometry(Measures);
            publish_path(Measures);
            publish_tf(Measures);
            return;
        }
        
        // 3、地图增量更新
        map_incremental();

        // 4、发布结果
        // 发布当前帧点云
        publish_current_cloud_body(Measures);
        // 发布当前帧全局点云
        publish_current_cloud_world(Measures);
        publish_map(Measures);
        // 发布 IMU 预测的 odometry 和 path        
        publish_odometry(Measures);
        publish_path(Measures);
        publish_tf(Measures);
        publish_degeneracy_info(Measures);
    }

    /**
     * @brief 发布当前帧前端退化统计，作为后端退化感知优化的输入。
     *
     * data 字段约定：
     * [0] lidar_begin_time
     * [1] degeneracy_mode，0=Normal，1=Transient，2=Persistent
     * [2] current frame static-degenerate flag
     * [3] effective_ratio
     * [4] residual_mean
     * [5] normal_eigen_ratio
     * [6] condition_number
     * [7] window_degenerate_ratio
     * [8] window_normal_eigen_ratio_mean
     * [9] window_residual_cv
     * [10] window_path_length
     * [11] window_yaw_change
     * [12] window_condition_number_mean
     * [13] window_recent_degenerate_streak
     * [14] insert_ratio，当前帧允许入图点占下采样点比例
     * [15] localizability_observed_voxels
     * [16] localizability_planarity_mean
     * [17:19] localizability field eigenvalues，升序
     * [20] localizability_f0 = lambda_min / trace(M)
     * [21] localizability_lambda0 = lambda_min / observed_voxels
     * [22:24] translation posterior covariance eigenvalues，升序，m^2
     * [25:27] translation maximum-uncertainty direction
     * [28:30] rotation posterior covariance eigenvalues，升序，rad^2
     * [31:33] rotation maximum-uncertainty direction
     */
    void publish_degeneracy_info(const MeasureGroup &meas)
    {
        if (!pub_degeneracy_info_)
        {
            return;
        }

        std_msgs::msg::Float64MultiArray msg;
        // 后端关键帧点云 /cloud_registered_body 使用 lidar_end_time。
        // 退化先验必须使用同一帧末时刻，避免后端将其错配到相邻关键帧。
        const double insert_ratio =
            feats_down_body->empty()
                ? 0.0
                : static_cast<double>(last_map_add_num) /
                      static_cast<double>(feats_down_body->size());
        msg.data = build_adaptive_degeneracy_info(
            meas.lidar_end_time, insert_ratio);
        pub_degeneracy_info_->publish(msg);
    }


    // 发布当前帧点云
    void publish_current_cloud_body(const MeasureGroup &meas)
    {
        if(!scan_bodyframe_pub_en || pub_cloud_body_->get_subscription_count() == 0)
        {
            return;
        }

        if(feats_undistort == nullptr || feats_undistort->empty())
        {
            return;
        }

        PointCloudXYZI cloud_body;
        cloud_body.reserve(feats_undistort->size());

        const Eigen::Matrix3d R_li =
            state_point.offset_R_L_I.toRotationMatrix();
        const Eigen::Vector3d t_li =
            state_point.offset_T_L_I;

        // feats_undistort 仍位于 LiDAR 坐标系；官方发布 body 话题前会应用 LiDAR->IMU 外参。
        for (const auto &point_lidar : feats_undistort->points)
        {
            PointType point_body = point_lidar;
            const Eigen::Vector3d p_lidar(
                point_lidar.x,
                point_lidar.y,
                point_lidar.z);
            const Eigen::Vector3d p_body = R_li * p_lidar + t_li;
            point_body.x = static_cast<float>(p_body.x());
            point_body.y = static_cast<float>(p_body.y());
            point_body.z = static_cast<float>(p_body.z());
            cloud_body.push_back(point_body);
        }

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud_body, cloud_msg);

        cloud_msg.header.stamp = get_ros_time(meas.lidar_end_time);
        cloud_msg.header.frame_id = "body";

        pub_cloud_body_->publish(cloud_msg);
    }

    /**
    * @brief 发布当前 IMU 预测状态
    *
    * 第一版只是 IMU 积分结果，不代表最终 SLAM 位姿。
    * 后面 scan-to-map 和 ESKF 更新后，这里发布的才是优化后的状态。
    */
    void publish_odometry(const MeasureGroup &meas)
    {
        nav_msgs::msg::Odometry odom;

        // 与 /cloud_registered_body 的时间戳严格对齐。
        // 点云在整帧去畸变完成后发布，代表 lidar_end_time 的状态；若 Odometry
        // 仍标为 lidar_beg_time，后端会把上一帧 body 点云匹配给当前帧位姿。
        odom.header.stamp = get_ros_time(meas.lidar_end_time);
        odom.header.frame_id = "camera_init";
        odom.child_frame_id = "body";

        odom.pose.pose.position.x = state_point.pos.x();
        odom.pose.pose.position.y = state_point.pos.y();
        odom.pose.pose.position.z = state_point.pos.z();

        Eigen::Quaterniond q(state_point.rot.toRotationMatrix());
        q.normalize();

        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        pub_odom_->publish(odom);
    }

    //发布Path
    void publish_path(const MeasureGroup &meas)
    {
        geometry_msgs::msg::PoseStamped pose;
        // Path 与 Odometry/关键帧点云采用同一帧末时刻，便于轨迹记录和后端同步。
        pose.header.stamp = get_ros_time(meas.lidar_end_time);
        pose.header.frame_id = "camera_init";

        pose.pose.position.x = state_point.pos.x();
        pose.pose.position.y = state_point.pos.y();
        pose.pose.position.z = state_point.pos.z();

        Eigen::Quaterniond q(state_point.rot.toRotationMatrix());
        q.normalize();
        
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        path_.header.stamp = pose.header.stamp;
        path_.header.frame_id = "camera_init";
        path_.poses.push_back(pose);

        pub_path_->publish(path_);
    }

    /**
    * @brief 发布当前帧 world 坐标系点云
    *
    * 对应 FAST-LIO2 中 /cloud_registered。
    *
    * 注意：
    * 1. feats_down_world 是 map_incremental() 中由 body 点转换得到的；
    * 2. frame_id 使用 camera_init，表示世界坐标系；
    * 3. 这里必须调用 pcl::toROSMsg()，否则 cloud_msg 里没有点云数据。
    */
    void publish_current_cloud_world(const MeasureGroup &meas)
    {
        if(!scan_publish_en || pub_cloud_world_->get_subscription_count() == 0)
        {
            return;
        }

        PointCloudXYZI::Ptr cloud_body =
            dense_publish_en ? feats_undistort : feats_down_body;

        if(cloud_body == nullptr || cloud_body->empty())
        {
            return;
        }

        PointCloudXYZI::Ptr cloud_world(new PointCloudXYZI());
        cloud_world->reserve(cloud_body->size());

        for (const auto &point_body : cloud_body->points)
        {
            PointType point_world;
            pointBodyToWorld(point_body, point_world);
            cloud_world->push_back(point_world);
        }

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_world,cloud_msg);

        cloud_msg.header.stamp = get_ros_time(meas.lidar_end_time);
        cloud_msg.header.frame_id = "camera_init";

        pub_cloud_world_->publish(cloud_msg);

    }

    // 发布地图
    void publish_map(const MeasureGroup &meas)
    {
        // 约每秒发布一次地图；/Laser_map 服从官方 map_en，/ikdtree_map 始终用于诊断。
        constexpr int map_publish_interval_frames = 10;
        map_publish_frame_count++;
        if (map_publish_frame_count % map_publish_interval_frames != 0)
        {
            return;
        }

        if (map_publish_en && pub_map_->get_subscription_count() > 0)
        {
            PointCloudXYZI::Ptr cloud_body =
                dense_publish_en ? feats_undistort : feats_down_body;
            if (cloud_body != nullptr && !cloud_body->empty())
            {
                PointCloudXYZI cloud_world;
                cloud_world.reserve(cloud_body->size());
                for (const auto &point_body : cloud_body->points)
                {
                    PointType point_world;
                    pointBodyToWorld(point_body, point_world);
                    cloud_world.push_back(point_world);
                }
                *pcl_wait_pub += cloud_world;

                sensor_msgs::msg::PointCloud2 map_msg;
                pcl::toROSMsg(*pcl_wait_pub, map_msg);

                map_msg.header.stamp = get_ros_time(meas.lidar_end_time);
                map_msg.header.frame_id = "camera_init";

                pub_map_->publish(map_msg);
            }
        }

        if (pub_ikdtree_map_->get_subscription_count() > 0)
        {
            PointCloudXYZI::Ptr ikdtree_map = p_map->getMapCloud();
            if (ikdtree_map != nullptr && !ikdtree_map->empty())
            {
                sensor_msgs::msg::PointCloud2 ikdtree_map_msg;
                pcl::toROSMsg(*ikdtree_map, ikdtree_map_msg);
                ikdtree_map_msg.header.stamp = get_ros_time(meas.lidar_end_time);
                ikdtree_map_msg.header.frame_id = "camera_init";
                pub_ikdtree_map_->publish(ikdtree_map_msg);
            }
        }
    }

    void publish_tf(const MeasureGroup &meas)
    {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = get_ros_time(meas.lidar_end_time);
        transform.header.frame_id = "camera_init";
        transform.child_frame_id = "body";

        transform.transform.translation.x = state_point.pos.x();
        transform.transform.translation.y = state_point.pos.y();
        transform.transform.translation.z = state_point.pos.z();

        Eigen::Quaterniond q(state_point.rot.toRotationMatrix());
        q.normalize();

        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);

    auto node = std::make_shared<AdaptiveLaserMappingNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
