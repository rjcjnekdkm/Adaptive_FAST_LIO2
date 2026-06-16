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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

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
// 累计入图统计，便于观察长期地图更新趋势。
uint64_t total_map_added = 0;
uint64_t total_quality_rejected = 0;
uint64_t total_direction_rejected = 0;
uint64_t total_voxel_rejected = 0;

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

// 当前帧实际计算得到的有效匹配比例与法向量特征值比例，用于退化判断和日志。
double frame_effective_ratio = 0.0;
double frame_normal_eigen_ratio = 0.0;
double frame_residual_median = 0.0;
double frame_residual_mad = 0.0;

// h_share_model() 按 feats_down_body 索引生成的逐点质量信息。
// effective 表示该点是否形成有效点到面约束；其余数组分别保存绝对残差、质量分数和世界系平面法向量。
std::vector<uint8_t> map_point_effective;
std::vector<double> map_point_residual_abs;
std::vector<double> map_point_quality_score;
std::vector<Eigen::Vector3d> map_point_normal;
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

    if (runtime_sensor_debug)
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

    if (runtime_sensor_debug)
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

    if (runtime_sensor_debug && publish_count % 200 == 0)
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

void refreshStatePoint()
{
    state_point = kf.get_x();
}

float pointDistanceSquared(const PointType &a, const PointType &b)
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;

    return dx * dx + dy * dy + dz * dz;
}

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
 * @brief 判断当前帧是否可能退化
 *
 * 联合使用最终 scan-to-map 结果判断：
 *   1. 有效匹配点数量
 *   2. 有效匹配点比例
 *   3. 平均残差
 *   4. 法向量信息矩阵的最小/最大特征值比例
 */
bool is_current_frame_degenerate()
{
    if(!adaptive_map_enable)
    {
        return false;
    }

    const int downsampled_points = static_cast<int>(feats_down_body->size());
    frame_effective_ratio =
        downsampled_points > 0
            ? static_cast<double>(effct_feat_num) / static_cast<double>(downsampled_points)
            : 0.0;

    // Σ(n·n^T) 描述有效平面法向量的方向覆盖程度：
    // 最小特征值接近 0 表示至少有一个方向几乎没有几何约束。
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
            frame_normal_eigen_ratio =
                eigenvalues(2) > 1e-9 ? eigenvalues(0) / eigenvalues(2) : 0.0;
        }
    }

    return effct_feat_num < adaptive_min_effective_points ||
           frame_effective_ratio < adaptive_min_effective_ratio ||
           res_mean_last > adaptive_max_mean_residual ||
           frame_normal_eigen_ratio < adaptive_min_normal_eigen_ratio;
}

int normal_direction_bin(const Eigen::Vector3d &input_normal)
{
    constexpr double pi = 3.14159265358979323846;
    Eigen::Vector3d normal = input_normal.normalized();
    // 平面法向量 n 与 -n 表示同一平面方向，统一到同一半球后再分箱。
    if (normal.z() < 0.0 ||
        (std::abs(normal.z()) < 1e-9 && normal.y() < 0.0) ||
        (std::abs(normal.z()) < 1e-9 && std::abs(normal.y()) < 1e-9 && normal.x() < 0.0))
    {
        normal = -normal;
    }

    const double bin_angle =
        std::max(adaptive_normal_bin_angle_deg, 1.0) * pi / 180.0;
    const double azimuth = std::atan2(normal.y(), normal.x()) + pi;
    const double elevation = std::asin(std::clamp(normal.z(), -1.0, 1.0)) + 0.5 * pi;
    // 使用方位角和俯仰角构造稳定的一维分箱编号。
    const int azimuth_bin = static_cast<int>(std::floor(azimuth / bin_angle));
    const int elevation_bin = static_cast<int>(std::floor(elevation / bin_angle));
    const int azimuth_bin_count = static_cast<int>(std::ceil(2.0 * pi / bin_angle));

    return elevation_bin * azimuth_bin_count + azimuth_bin;
}

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
    row.adaptive_map = adaptive_map_enable;
    row.degenerate = frame_degenerate;
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
    row.novel_accepted = novel_accepted_num;
    row.novel_rejected = novel_rejected_num;
    row.voxel_rejected = voxel_rejected_num;
    row.total_rejected = total_rejected_num;

    // 写入后的地图规模与程序启动以来的累计统计。
    row.map_size = p_map->size();
    row.total_map_added = total_map_added;
    row.total_quality_rejected = total_quality_rejected;
    row.total_direction_rejected = total_direction_rejected;
    row.total_voxel_rejected = total_voxel_rejected;

    runtime_logger.write(row);
}


/**
 * @brief 判断一个点是否允许加入地图
 *
 * 插入规则：
 * 1. 如果 adaptive_map 未开启，所有点都允许加入
 * 2. 距离太近，不加入
 * 3. 距离太远，不加入
 * 4. 使用稳健残差阈值剔除异常或质量分数过低的匹配点
 * 5. 退化帧中拒绝地图附近的匹配失败点，并限制重复法向方向
 * 6. 对地图未知区域的新点设置帧级配额，避免地图完全停止生长
 */
bool allow_map_insert_point(
    const PointType &point_body,
    size_t point_index,
    bool frame_degenerate,
    std::unordered_map<int, int> &normal_bin_counts,
    int &quality_reject_num,
    int &invalid_quality_num,
    int &direction_reject_num,
    int &novel_accept_num,
    int &novel_reject_num)
{
    if(!adaptive_map_enable)
    {
        return true;
    }

    double x = point_body.x;
    double y = point_body.y;
    double z = point_body.z;

    double range = std::sqrt(x*x + y*y + z*z);

    if(range < adaptive_min_range)
    {
        return false;
    }

    if(range > adaptive_max_range)
    {
        return false;
    }

    // 只有在最终位姿下成功形成点到面约束的点，才具有可用于筛选的质量信息。
    const bool has_quality =
        point_index < map_point_effective.size() &&
        map_point_effective[point_index] != 0;

    if (has_quality)
    {
        // 使用中位数和 MAD 构造稳健阈值，避免少量异常残差抬高整帧门限。
        const double robust_residual_limit =
            frame_residual_median +
            adaptive_residual_robust_sigma * 1.4826 * frame_residual_mad;
        const double residual_limit =
            std::min(
                plane_residual_threshold,
                std::max(
                    std::min(
                        res_mean_last * adaptive_residual_scale,
                        robust_residual_limit),
                    0.03));

        const double residual_abs =
            map_point_residual_abs[point_index];

        const double quality_score =
            map_point_quality_score[point_index];

        if (residual_abs > residual_limit ||
            quality_score < adaptive_min_quality_score)
        {
            quality_reject_num++;
            return false;
        }
    }
    else if (frame_degenerate)
    {
        const bool has_local_neighbors =
            point_index < map_point_has_local_neighbors.size() &&
            map_point_has_local_neighbors[point_index] != 0;

        if (has_local_neighbors)
        {
            // 地图附近已有结构但当前点无法形成有效约束，视为坏匹配或动态/异常点。
            invalid_quality_num++;
            return false;
        }

        // 地图未知区域没有可用近邻。完全拒绝会导致退化阶段地图无法向新区域生长，
        // 因此按帧限额保留少量新信息。
        if (novel_accept_num >= std::max(0, adaptive_max_novel_points_per_frame))
        {
            novel_reject_num++;
            return false;
        }
        novel_accept_num++;
    }

    if(frame_degenerate && has_quality)
    {
        const int bin = normal_direction_bin(map_point_normal[point_index]);
        int &bin_count = normal_bin_counts[bin];
        if (bin_count >= std::max(1, adaptive_max_points_per_normal_bin))
        {
            direction_reject_num++;
            return false;
        }
        bin_count++;
    }

    return true;

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

    map_point_effective.clear();
    map_point_residual_abs.clear();
    map_point_quality_score.clear();
    map_point_normal.clear();
    frame_residual_median = 0.0;
    frame_residual_mad = 0.0;

    if (feats_down_body == nullptr || feats_down_body->empty())
    {
        ekfom_data.valid = false;
        return;
    }

    map_point_effective.assign(feats_down_body->size(), 0);
    map_point_residual_abs.assign(feats_down_body->size(), 0.0);
    map_point_quality_score.assign(feats_down_body->size(), 0.0);
    map_point_normal.assign(feats_down_body->size(), Eigen::Vector3d::Zero());
    if (map_point_has_local_neighbors.size() != feats_down_body->size())
    {
        map_point_has_local_neighbors.assign(feats_down_body->size(), 0);
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
            map_point_has_local_neighbors[i] =
                point_selected_surf[i] != 0;
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
        map_point_effective[i] = 1;
        map_point_residual_abs[i] = std::abs(residual);
        map_point_quality_score[i] = score;
        map_point_normal[i] = plane_normal;

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

    res_mean_last =
        residual_sum / static_cast<double>(effct_feat_num);
    frame_residual_median = median_of_values(effective_residuals);

    std::vector<double> residual_deviations;
    residual_deviations.reserve(effective_residuals.size());
    for (const double residual : effective_residuals)
    {
        residual_deviations.push_back(
            std::abs(residual - frame_residual_median));
    }
    frame_residual_mad = median_of_values(residual_deviations);

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

    bool frame_degenerate = is_current_frame_degenerate();

    int rejected_num = 0;
    int voxel_rejected_num = 0;
    int quality_rejected_num = 0;
    int invalid_quality_rejected_num = 0;
    int direction_rejected_num = 0;
    int novel_accepted_num = 0;
    int novel_rejected_num = 0;
    // 仅在当前帧内统计各法向方向已接纳的点数，防止单一方向约束大量写入地图。
    std::unordered_map<int, int> normal_bin_counts;

    std::vector<size_t> candidate_indices(feats_down_body->size());
    std::iota(candidate_indices.begin(), candidate_indices.end(), 0);
    if (adaptive_map_enable && frame_degenerate)
    {
        // 退化帧存在方向配额时优先保留高质量点，避免结果依赖 VoxelGrid 输出顺序。
        std::stable_sort(
            candidate_indices.begin(),
            candidate_indices.end(),
            [](size_t lhs, size_t rhs)
            {
                const bool lhs_effective =
                    lhs < map_point_effective.size() && map_point_effective[lhs] != 0;
                const bool rhs_effective =
                    rhs < map_point_effective.size() && map_point_effective[rhs] != 0;
                if (lhs_effective != rhs_effective)
                {
                    return lhs_effective;
                }
                if (!lhs_effective)
                {
                    return false;
                }
                return map_point_quality_score[lhs] > map_point_quality_score[rhs];
            });
    }

    for(const size_t i : candidate_indices)
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

        bool allow_insert =
            allow_map_insert_point(
                point_body,
                i,
                frame_degenerate,
                normal_bin_counts,
                quality_rejected_num,
                invalid_quality_rejected_num,
                direction_rejected_num,
                novel_accepted_num,
                novel_rejected_num);

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
        point_to_add->size() +
        point_no_need_downsample->size();

    map_update_count++;
    total_map_added += add_num;
    total_quality_rejected +=
        quality_rejected_num + invalid_quality_rejected_num + novel_rejected_num;
    total_direction_rejected += direction_rejected_num;
    total_voxel_rejected += voxel_rejected_num;

    // 所有本帧和累计统计更新完成后再记录，保证 CSV 中各字段属于同一帧状态。
    write_runtime_log_row(
        frame_degenerate,
        add_num,
        point_to_add->size(),
        point_no_need_downsample->size(),
        quality_rejected_num,
        invalid_quality_rejected_num,
        direction_rejected_num,
        novel_accepted_num,
        novel_rejected_num,
        voxel_rejected_num,
        rejected_num);

    const bool degenerate_state_changed =
        adaptive_map_enable &&
        (!has_previous_degenerate_state ||
         frame_degenerate != previous_frame_degenerate);
    const bool periodic_log =
        runtime_log_interval_frames > 0 &&
        map_update_count % runtime_log_interval_frames == 0;

    if (degenerate_state_changed)
    {
        std::cout << "[Degeneracy] frame=" << map_update_count
                  << ", state=" << (frame_degenerate ? "ENTER" : "EXIT")
                  << ", effective=" << effct_feat_num
                  << ", effective_ratio=" << frame_effective_ratio
                  << ", normal_eigen_ratio=" << frame_normal_eigen_ratio
                  << ", mean_residual=" << res_mean_last
                  << std::endl;
    }

    if (periodic_log || degenerate_state_changed)
    {
        const double insert_ratio =
            feats_down_body->empty()
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
             << " degenerate=" << (frame_degenerate ? "Y" : "N")
             << " add=" << add_num
             << " insert_ratio=" << insert_ratio
             << " reject(q/d/v)="
             << quality_rejected_num + invalid_quality_rejected_num + novel_rejected_num
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

    previous_frame_degenerate = frame_degenerate;
    has_previous_degenerate_state = true;
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
        this->declare_parameter<int>("mapping.imu_init_num", 200);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", false);

        this->declare_parameter<int>("mapping.map_init_min_points", 6);
        this->declare_parameter<double>("mapping.cube_len", 1000.0);
        this->declare_parameter<double>("mapping.det_range", 450.0);
        this->declare_parameter<double>("mapping.move_threshold", 1.5);
        this->declare_parameter<bool>("mapping.local_map_enable", true);

        this->declare_parameter<bool>("mapping.local_map_delete_enable", true);

        // ===================== idea 参数预留 =====================
        this->declare_parameter<bool>("adaptive_map.enable", false);
        this->declare_parameter<int>("adaptive_map.min_effective_points", 200);
        this->declare_parameter<double>("adaptive_map.min_range", 0.20);
        this->declare_parameter<double>("adaptive_map.max_range", 80.0);
        this->declare_parameter<double>("adaptive_map.residual_scale", 2.0);
        this->declare_parameter<double>("adaptive_map.residual_robust_sigma", 2.5);
        this->declare_parameter<double>("adaptive_map.min_quality_score", 0.92);
        this->declare_parameter<double>("adaptive_map.min_effective_ratio", 0.1);
        this->declare_parameter<double>("adaptive_map.max_mean_residual", 0.15);
        this->declare_parameter<double>("adaptive_map.min_normal_eigen_ratio", 0.02);
        this->declare_parameter<double>("adaptive_map.normal_bin_angle_deg", 15.0);
        this->declare_parameter<int>("adaptive_map.max_points_per_normal_bin", 30);
        this->declare_parameter<int>("adaptive_map.max_novel_points_per_frame", 50);

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
        this->get_parameter("runtime_log.csv_enable", runtime_csv_enable);
        this->get_parameter("runtime_log.csv_path", runtime_csv_path);
        this->get_parameter("runtime_log.csv_append", runtime_csv_append);
        runtime_log_interval_frames = std::max(runtime_log_interval_frames, 1);

        this->get_parameter("mapping.filter_size_surf", filter_size_surf);
        this->get_parameter("mapping.filter_size_map", filter_size_map);

        this->get_parameter("adaptive_map.enable", adaptive_map_enable);
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
                  << ", effective_min=" << adaptive_min_effective_points
                  << ", effective_ratio_min=" << adaptive_min_effective_ratio
                  << ", residual_max=" << adaptive_max_mean_residual
                  << ", normal_ratio_min=" << adaptive_min_normal_eigen_ratio
                  << ", quality_min=" << adaptive_min_quality_score
                  << std::endl;
        std::cout << "[Config] runtime summary_interval_frames="
                  << runtime_log_interval_frames
                  << ", sensor_debug=" << runtime_sensor_debug
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
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic,20,livox_pcl_cbk);

            std::cout << "Subscribe Livox CustomMsg." << std::endl;        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic,rclcpp::SensorDataQoS(),standard_pcl_cbk);

            std::cout << "Subscribe standard PointCloud2." << std::endl;
        }

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic,10,imu_cbk);

       
        pub_cloud_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body",10);
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry",10);
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/path",10);
        path_.header.frame_id = "camera_init";
        pub_cloud_world_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered",10);
        pub_map_ =this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map",10);
        pub_ikdtree_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ikdtree_map", 10);

        std::cout << "Publish topics: /cloud_registered_body, /cloud_registered, /Laser_map, /ikdtree_map, /Odometry, /path" << std::endl;

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

        if (runtime_sensor_debug)
        {
            std::cout << "[Sensor][Sync] lidar_points=" << Measures.lidar->size()
                      << ", imu_size=" << Measures.imu.size()
                      << ", lidar_beg=" << Measures.lidar_beg_time
                      << ", lidar_end=" << Measures.lidar_end_time
                      << std::endl;
        }

        
        // 1. 使用 IKFoM 完成 IMU 状态传播和点云去畸变。
        p_imu->Process(Measures, kf, feats_undistort);
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

        // 5. 与官方 FAST-LIO2 一致，通过唯一的 h_share_model() 完成 IKFoM 迭代更新。
        double solve_H_time = 0.0;
        kf.update_iterated_dyn_share_modified(
            laser_point_cov,
            solve_H_time);

        refreshStatePoint();

        // adaptive_map 关闭时保持官方 FAST-LIO2 行为：只要观测模型存在有效点，
        // 就使用滤波后的最终状态继续增量更新地图。额外的有效点数量门槛仅属于
        // 自适应地图质量控制，不能改变基础复现路径。
        const bool scan_update_success =
            !adaptive_map_enable ||
            effct_feat_num >= scan_match_min_effective_points;

        if(!scan_update_success)
        {
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
    }


    // 发布当前帧点云
    void publish_current_cloud_body(const MeasureGroup &meas)
    {
        if(!scan_bodyframe_pub_en)
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

        odom.header.stamp = get_ros_time(meas.lidar_beg_time);
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
        pose.header.stamp = get_ros_time(meas.lidar_beg_time);
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
        if(!scan_publish_en)
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

        if (map_publish_en)
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
