#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace
{
// 后端内部统一使用带 intensity 的 PCL 点类型。
// 当前节点订阅的是前端发布的 /cloud_registered_body，保存关键帧时再下采样。
using PointType = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointType>;

/**
 * @brief 将角度归一化到 [-pi, pi]。
 *
 * 关键帧选择需要判断两帧 yaw 角变化，如果不做归一化，
 * 在 pi/-pi 附近会把很小的转角误认为接近 2pi 的大转角。
 */
double wrapAngle(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

/**
 * @brief 从 ROS pose 四元数中提取 yaw。
 *
 * 后端关键帧选择只用平面航向角判断是否发生明显转向：
 * - 位移大于 keyframe_distance_：保存关键帧；
 * - 或 yaw 变化大于 keyframe_yaw_：保存关键帧。
 */
double yawFromPose(const geometry_msgs::msg::Pose &pose)
{
    const auto &q = pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

/**
 * @brief 将前端 /Odometry 转为 GTSAM Pose3。
 *
 * 前端 odometry 是创新点 1 间接影响后端的主要通道：
 * adaptive 地图更新会影响后续 scan-to-map，从而影响这里的位姿初值。
 */
gtsam::Pose3 odomToPose3(const nav_msgs::msg::Odometry &odom)
{
    const auto &p = odom.pose.pose.position;
    const auto &q = odom.pose.pose.orientation;
    return gtsam::Pose3(
        gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
        gtsam::Point3(p.x, p.y, p.z));
}

/**
 * @brief 将 GTSAM Pose3 转回 ROS Pose，用于发布优化后的 path。
 */
geometry_msgs::msg::Pose pose3ToMsg(const gtsam::Pose3 &pose)
{
    geometry_msgs::msg::Pose msg;
    const auto t = pose.translation();
    const auto q = pose.rotation().toQuaternion();
    msg.position.x = t.x();
    msg.position.y = t.y();
    msg.position.z = t.z();
    msg.orientation.w = q.w();
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    return msg;
}

/**
 * @brief 将 PCL ICP 输出的 4x4 变换矩阵转为 GTSAM Pose3。
 *
 * ICP 的 final transformation 表示 source keyframe 到 target keyframe 的相对变换，
 * 这里转成 loop factor 的测量值，加入 pose graph。
 */
gtsam::Pose3 eigenToPose3(const Eigen::Matrix4f &tf)
{
    Eigen::Matrix3d rot = tf.block<3, 3>(0, 0).cast<double>();
    Eigen::Vector3d trans = tf.block<3, 1>(0, 3).cast<double>();
    return gtsam::Pose3(gtsam::Rot3(rot), gtsam::Point3(trans.x(), trans.y(), trans.z()));
}

/**
 * @brief 回环 ICP 的附加质量指标。
 *
 * 前向 ICP fitness 只能说明“当前帧能否贴到候选子地图”，在长直隧道中
 * 沿走廊方向仍可能缺少约束。因此额外记录：
 * - reverse_fitness：反向 ICP 的 fitness；
 * - reciprocal_*：正反 ICP 变换是否互为逆；
 * - observability_*：点到点 ICP 线性化 Hessian 的 SVD 可观测性。
 */
struct LoopValidationMetrics
{
    double reverse_fitness = std::numeric_limits<double>::quiet_NaN();
    double reciprocal_translation_error = std::numeric_limits<double>::quiet_NaN();
    double reciprocal_yaw_error = std::numeric_limits<double>::quiet_NaN();
    size_t observability_correspondences = 0;
    double observability_condition_number = std::numeric_limits<double>::quiet_NaN();
    double observability_min_singular_value = std::numeric_limits<double>::quiet_NaN();
    double path_separation = std::numeric_limits<double>::quiet_NaN();
};

/**
 * @brief 评估 ICP 约束的局部可观测性。
 *
 * 对前向 ICP 的最终 source->target 变换，将 source 点变换到 target 系，
 * 与 target 最近邻构造点到点残差。其微分雅可比近似为
 *
 *     J = [ I  -[p]_x ],   H = sum(J^T J) / N .
 *
 * H 的最小奇异值越小、条件数越大，说明存在弱约束方向。它不替代 ICP，
 * 而是用于拒绝“fitness 很小但几何上不可观测”的回环因子。
 */
LoopValidationMetrics evaluateLoopObservability(
    const Cloud::ConstPtr &source,
    const Cloud::ConstPtr &target,
    const Eigen::Matrix4f &source_to_target,
    double max_correspondence_distance)
{
    LoopValidationMetrics metrics;
    if (!source || !target || source->empty() || target->empty())
    {
        return metrics;
    }

    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(target);
    const float max_sq_distance = static_cast<float>(
        max_correspondence_distance * max_correspondence_distance);
    Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
    std::vector<int> indices(1);
    std::vector<float> squared_distances(1);

    for (const auto &point : source->points)
    {
        if (!pcl::isFinite(point))
        {
            continue;
        }
        const Eigen::Vector4f transformed = source_to_target * point.getVector4fMap();
        PointType query;
        query.x = transformed.x();
        query.y = transformed.y();
        query.z = transformed.z();
        if (kdtree.nearestKSearch(query, 1, indices, squared_distances) != 1 ||
            squared_distances.front() > max_sq_distance)
        {
            continue;
        }

        const Eigen::Vector3d p = transformed.head<3>().cast<double>();
        Eigen::Matrix<double, 3, 6> jacobian = Eigen::Matrix<double, 3, 6>::Zero();
        jacobian.block<3, 3>(0, 0).setIdentity();
        jacobian.block<3, 3>(0, 3) <<
             0.0,  p.z(), -p.y(),
            -p.z(), 0.0,  p.x(),
             p.y(), -p.x(), 0.0;
        hessian.noalias() += jacobian.transpose() * jacobian;
        ++metrics.observability_correspondences;
    }

    if (metrics.observability_correspondences < 6)
    {
        return metrics;
    }

    hessian /= static_cast<double>(metrics.observability_correspondences);
    const Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(
        hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix<double, 6, 1> singular = svd.singularValues();
    metrics.observability_min_singular_value = singular(5);
    metrics.observability_condition_number =
        singular(0) / std::max(singular(5), 1e-12);
    return metrics;
}

/**
 * @brief 从 GTSAM Pose3 中提取 yaw。
 */
double yawFromPose3(const gtsam::Pose3 &pose)
{
    const Eigen::Matrix3d rot = pose.rotation().matrix();
    return std::atan2(rot(1, 0), rot(0, 0));
}

/**
 * @brief 前端退化信息缓存。
 *
 * 这些量来自 /adaptive_frontend/degeneracy_info，由 adaptive_laserMapping.cpp 发布。
 * 它们把创新点 1 和创新点 2 串起来：
 *
 * 创新点 1：在前端判断退化并控制 ikd-tree 地图插入；
 * 创新点 2：后端读取这里的退化先验，调整回环验证阈值和 pose graph 边权重。
 */
struct DegeneracyInfo
{
    // 当前 LiDAR 帧起始时间，单位秒；用于和前端日志对齐。
    double time = 0.0;
    // 退化模式：0=Normal，1=Transient，2=Persistent。
    int mode = 0;
    // 当前帧是否被单帧静态规则判为退化。
    bool static_degenerate = false;
    // 当前帧有效匹配点比例：effective_points / downsampled_points。
    double effective_ratio = 1.0;
    // 当前帧有效点到面残差均值。
    double residual_mean = 0.0;
    // 当前帧法向信息矩阵最小/最大特征值比例，越小说明法向分布越单一。
    double normal_eigen_ratio = 1.0;
    // 当前帧 scan-to-map 条件数，越大说明约束越病态。
    double condition_number = 1.0;
    // 滑动窗口内单帧退化比例。
    double window_degenerate_ratio = 0.0;
    // 滑动窗口平均条件数。
    double window_condition_number_mean = 1.0;
    // 当前帧实际进入 ikd-tree 的点占下采样点比例，用于衡量前端筛选强度。
    double insert_ratio = 1.0;
    // 第一阶段新增诊断量。后端仅缓存，暂不参与 severity、回环或图优化。
    std::size_t localizability_observed_voxels = 0;
    double localizability_planarity_mean = 0.0;
    Eigen::Vector3d localizability_eigenvalues = Eigen::Vector3d::Zero();
    double localizability_f0 = 0.0;
    double localizability_lambda0 = 0.0;
    Eigen::Vector3d translation_cov_eigenvalues = Eigen::Vector3d::Zero();
    Eigen::Vector3d translation_weak_direction = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotation_cov_eigenvalues = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotation_weak_direction = Eigen::Vector3d::Zero();
    /**
     * @brief 计算后端使用的连续退化强度 D_i ∈ [0,1]。
     *
     * D_i 不是新的前端判据，而是把前端离散状态和窗口统计转成后端权重：
     *
     * - Normal：主要相信 window_degenerate_ratio；
     * - Transient：至少给中等退化强度；
     * - Persistent：至少给高退化强度；
     * - 条件数很大时再略微提高退化强度。
     *
     * 后端使用方式：
     * - D_i 越大，odometry factor 的 sigma 越大；
     * - sigma 越大，信息矩阵越小；
     * - pose graph 优化时越不盲目相信这段前端里程计。
     */
    double severity() const
    {
        // 退化程度 D_i ∈ [0,1]，后端用它调节 odometry 边权重。
        // Persistent 直接给更高先验；窗口退化比例和条件数作为连续补充。
        double d = std::clamp(window_degenerate_ratio, 0.0, 1.0);
        if (mode == 1)
        {
            d = std::max(d, 0.45);
        }
        else if (mode >= 2)
        {
            d = std::max(d, 0.8);
        }
        if (condition_number > 100.0 || window_condition_number_mean > 100.0)
        {
            d = std::min(1.0, d + 0.1);
        }
        return d;
    }
};

/**
 * @brief 后端关键帧结构。
 *
 * 后端不直接使用前端 ikd-tree 地图，而是保存关键帧级数据：
 * - odom_pose：前端给出的初始位姿；
 * - optimized_pose：GTSAM 优化后的位姿；
 * - cloud：当前关键帧点云；
 * - degeneracy：该关键帧对应的前端退化状态；
 * - scan_context：用于快速回环候选检索的全局描述子。
 */
struct KeyFrame
{
    int id = -1;
    rclcpp::Time stamp;
    gtsam::Pose3 odom_pose;
    gtsam::Pose3 optimized_pose;
    Cloud::Ptr cloud;
    DegeneracyInfo degeneracy;
    // Scan Context 极坐标网格（ring × sector），每格保存最大高度。
    std::vector<float> scan_context;
    // ring key：每一环对所有扇区求均值，旋转不变，用于快速粗检索。
    std::vector<float> ring_key;
    // sector key：每一扇区对所有环求均值，用于估计两个描述子的航向循环偏移。
    std::vector<float> sector_key;
};

/**
 * @brief 前端全频里程计样本。
 *
 * Pose graph 只优化关键帧；实验评测则需要与真值同时间密度的轨迹。该结构保存
 * 每一帧前端位姿，供后端在优化后将最近关键帧的图优化校正传播到全频轨迹。
 */
struct OdomSample
{
    rclcpp::Time stamp;
    gtsam::Pose3 odom_pose;
};

}  // namespace

class AdaptiveDegenerateBackend : public rclcpp::Node
{
public:
    /**
     * @brief 构造退化感知后端节点。
     *
     * 输入：
     * - /Odometry：前端实时里程计；
     * - /cloud_registered_body：当前帧 body/lidar 系去畸变点云；
     * - /adaptive_frontend/degeneracy_info：创新点 1 输出的退化先验。
     *
     * 输出：
     * - /adaptive_backend/optimized_path：GTSAM 优化后的关键帧轨迹。
     *
     * 当前版本是松耦合后端：
     * - 不回写 FAST-LIO2 的 state_point；
     * - 不重构前端 ikd-tree；
     * - 只额外维护关键帧、回环约束和优化轨迹。
     */
    AdaptiveDegenerateBackend()
        : Node("adaptive_degenerate_backend")
    {
        // 关键帧选择参数：位移或 yaw 变化超过阈值才保存关键帧，避免每帧都进后端。
        keyframe_distance_ = declare_parameter<double>("keyframe_distance", 1.0);
        keyframe_yaw_ = declare_parameter<double>("keyframe_yaw", 0.25);
        // odometry 与点云时间戳允许的最大差值。超过该阈值说明当前缓存不同步，跳过。
        cloud_time_tolerance_ = declare_parameter<double>("cloud_time_tolerance", 0.20);
        // 点云和退化消息的时间同步队列长度，过小可能找不到对应消息。
        const auto sync_queue_param =
            declare_parameter<int>("sync_queue_size", 200);
        sync_queue_size_ = static_cast<size_t>(std::max<int64_t>(
            20, sync_queue_param));
        // 回环检索时排除最近 N 个关键帧，避免把相邻帧误当成回环。
        recent_exclusion_num_ = declare_parameter<int>("recent_exclusion_num", 30);
        // 用累计行驶距离补充关键帧编号排除。关键帧密度会随速度/转向改变，
        // 因此在长直隧道中“相隔 30 帧”并不总是代表足够远的历史重访。
        // 默认 0 表示保持旧行为；启用后，近路径候选会在 SC 排序后、ICP 前剔除。
        loop_min_path_separation_ =
            declare_parameter<double>("loop_min_path_separation", 0.0);
        // 每个当前关键帧最多尝试多少个 ScanContext 候选。
        // 默认保持 top-1：当前 SubT 长走廊实验中 top-K 虽能增加候选召回，
        // 但会引入更多通过局部门控却拉坏全局/局部轨迹的回环。
        // 若后续换数据集或进一步增强候选验证，可通过参数改大该值重新实验。
        loop_candidate_top_k_ = declare_parameter<int>("loop_candidate_top_k", 1);
        // 回环冷却：同一当前关键帧附近不重复添加回环。
        // 原逻辑只禁止完全相同的 current_id；当前默认扩大到相邻关键帧。
        loop_current_cooldown_keyframes_ =
            declare_parameter<int>("loop_current_cooldown_keyframes", 10);
        // 回环冷却：同一候选关键帧附近不重复添加回环，抑制长走廊中的重复匹配。
        loop_candidate_cooldown_keyframes_ =
            declare_parameter<int>("loop_candidate_cooldown_keyframes", 20);
        // 回环对冷却：当前帧和候选帧都相近时，视为同一局部回环事件。
        loop_pair_cooldown_keyframes_ =
            declare_parameter<int>("loop_pair_cooldown_keyframes", 30);
        // 完整 Scan Context 距离阈值，越小表示经扇区循环对齐后两个描述子越相似。
        scan_context_distance_threshold_ =
            declare_parameter<double>("scan_context_distance_threshold", 0.18);
        // ring key 粗检索保留的历史候选数。它只缩小检索范围，最终仍以完整 SC 距离排序。
        scan_context_ringkey_candidate_num_ =
            declare_parameter<int>("scan_context_ringkey_candidate_num", 10);
        // Normal 状态下 ICP 回环验证阈值。
        icp_fitness_threshold_normal_ =
            declare_parameter<double>("icp_fitness_threshold_normal", 0.30);
        // Persistent 退化相关关键帧使用更严格 ICP 阈值，抑制长走廊相似结构假回环。
        icp_fitness_threshold_degenerate_ =
            declare_parameter<double>("icp_fitness_threshold_degenerate", 0.10);
        // 双向 ICP：额外从候选子地图配准回当前帧，检查正反变换是否互为逆。
        // 默认关闭以保持既有正式实验行为；启用后可抑制重复隧道中的单向伪配准。
        loop_bidirectional_icp_enable_ =
            declare_parameter<bool>("loop_bidirectional_icp_enable", false);
        loop_bidirectional_icp_fitness_threshold_ =
            declare_parameter<double>("loop_bidirectional_icp_fitness_threshold", 0.30);
        loop_reciprocal_translation_threshold_ =
            declare_parameter<double>("loop_reciprocal_translation_threshold", 0.30);
        loop_reciprocal_yaw_threshold_ =
            declare_parameter<double>("loop_reciprocal_yaw_threshold", 0.10);
        // ICP Hessian-SVD 可观测性门控：默认关闭，避免未经标定的阈值改变旧实验。
        // 启用后要求有效对应数足够、条件数不过大且最小奇异值不接近零。
        loop_observability_enable_ =
            declare_parameter<bool>("loop_observability_enable", false);
        // 分离“记录指标”和“按指标拒绝”：首次在新数据集上仅记录 SVD 分布，
        // 标定后才打开 reject，避免凭经验阈值把所有真实回环一并滤掉。
        loop_observability_reject_enable_ =
            declare_parameter<bool>("loop_observability_reject_enable", false);
        loop_observability_min_correspondences_ =
            declare_parameter<int>("loop_observability_min_correspondences", 100);
        loop_observability_max_condition_number_ =
            declare_parameter<double>("loop_observability_max_condition_number", 1e5);
        loop_observability_min_singular_value_ =
            declare_parameter<double>("loop_observability_min_singular_value", 1e-4);
        // ICP 最大对应点距离，影响配准可收敛范围和误匹配风险。
        icp_max_correspondence_distance_ =
            declare_parameter<double>("icp_max_correspondence_distance", 2.0);
        // ICP 相对位姿一致性阈值：ICP 结果不能和前端 odometry 预测相差过大。
        // 这一步用于过滤“fitness 较小但几何位置不合理”的假回环。
        loop_consistency_trans_normal_ =
            declare_parameter<double>("loop_consistency_trans_normal", 2.0);
        loop_consistency_trans_degenerate_ =
            declare_parameter<double>("loop_consistency_trans_degenerate", 1.0);
        loop_consistency_yaw_normal_ =
            declare_parameter<double>("loop_consistency_yaw_normal", 0.50);
        loop_consistency_yaw_degenerate_ =
            declare_parameter<double>("loop_consistency_yaw_degenerate", 0.25);
        // 回环边权重。数值越大表示回环约束越弱，避免少量回环过度拉坏局部 RPE。
        loop_noise_sigma_normal_ =
            declare_parameter<double>("loop_noise_sigma_normal", 0.30);
        loop_noise_sigma_degenerate_ =
            declare_parameter<double>("loop_noise_sigma_degenerate", 0.50);
        // ICP target 子地图参数：候选关键帧附近拼接 ±N 帧，提高匹配点数量和几何稳定性。
        loop_submap_search_num_ = declare_parameter<int>("loop_submap_search_num", 10);
        loop_submap_leaf_size_ = declare_parameter<double>("loop_submap_leaf_size", 0.5);
        // 关键帧点云体素下采样大小。越大越快但几何细节越少。
        keyframe_leaf_size_ = declare_parameter<double>("keyframe_leaf_size", 0.4);
        // 是否发布后端优化地图。只发布 ROS topic，不保存 PCD 文件。
        // 综合最优实验默认关闭该可视化输出，避免周期性拼接全局地图影响在线优化节奏；
        // 需要 RViz 查看后端地图时，可手动改为 true 或通过参数开启。
        publish_optimized_map_ = declare_parameter<bool>("publish_optimized_map", false);
        // 优化地图发布间隔。拼接全部关键帧点云有开销，因此默认每 10 个关键帧发布一次。
        optimized_map_publish_interval_ =
            declare_parameter<int>("optimized_map_publish_interval", 10);
        // 后端优化地图体素下采样大小。仅影响发布出来的可视化/对比地图，不影响优化本身。
        optimized_map_leaf_size_ =
            declare_parameter<double>("optimized_map_leaf_size", 0.5);
        // Scan Context 参数：rings × sectors 的极坐标最大高度网格。
        scan_context_rings_ = declare_parameter<int>("scan_context_rings", 20);
        scan_context_sectors_ = declare_parameter<int>("scan_context_sectors", 60);
        scan_context_max_radius_ = declare_parameter<double>("scan_context_max_radius", 80.0);
        // sector key 的最优循环偏移附近再搜索的扇区数量，补偿粗对齐的离散误差。
        scan_context_alignment_search_radius_ =
            declare_parameter<int>("scan_context_alignment_search_radius", 2);
        // 后端文件记录。TUM 用于 evo 评估，CSV 用于分析回环候选与退化感知验证行为。
        backend_log_enable_ = declare_parameter<bool>("backend_log_enable", true);
        backend_tum_path_ = declare_parameter<std::string>(
            "backend_tum_path",
            "/home/romi/Adaptive_FAST_LIO2/experiments/subt_mrs_hawkins_long_corridor/results/adaptive_backend_optimized.tum");
        // 全频优化轨迹：将关键帧图优化校正传播到每一条前端 odometry。
        // 它是与 FAST-LIO2、前端消融进行公平 ATE 对比时应使用的结果；
        // backend_tum_path 仍保留关键帧轨迹，便于检查 pose graph 本身。
        backend_full_tum_path_ = declare_parameter<std::string>(
            "backend_full_tum_path",
            "/home/romi/Adaptive_FAST_LIO2/experiments/subt_mrs_hawkins_long_corridor/results/adaptive_backend_optimized_full.tum");
        backend_loop_csv_path_ = declare_parameter<std::string>(
            "backend_loop_csv_path",
            "/home/romi/Adaptive_FAST_LIO2/experiments/subt_mrs_hawkins_long_corridor/results/adaptive_backend_loops.csv");

        // 前端位姿输入：用于关键帧位姿初值和 odometry factor。
        // 回环 ICP 与 iSAM2 优化会暂时占用回调线程。GEODE Gamma 的 odometry
        // 约为 10 Hz；保留至少 300 条（约 30 s）可覆盖实测约 15 s 的积压，
        // 又避免使用过大的离线缓存。避免尾段位姿被 DDS 丢弃而使后端 ATE 少匹配真值。
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", rclcpp::QoS(std::max<size_t>(300, sync_queue_size_)),
            std::bind(&AdaptiveDegenerateBackend::odomHandler, this, std::placeholders::_1));
        // 当前帧点云输入：用于保存关键帧、生成 ScanContext、ICP 回环验证。
        sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered_body", rclcpp::SensorDataQoS(),
            std::bind(&AdaptiveDegenerateBackend::cloudHandler, this, std::placeholders::_1));
        // 前端退化信息输入：后端退化感知优化的核心先验。
        sub_degeneracy_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/adaptive_frontend/degeneracy_info", 100,
            std::bind(&AdaptiveDegenerateBackend::degeneracyHandler, this, std::placeholders::_1));

        // 后端优化轨迹。
        pub_optimized_path_ =
            create_publisher<nav_msgs::msg::Path>("/adaptive_backend/optimized_path", 10);
        // 后端优化地图：使用 optimized_pose_i × keyframe_cloud_i 拼接得到。
        // 使用 transient_local，让 RViz/ros2 topic echo 在地图发布之后再订阅时，
        // 也能收到最近一次发布的地图，避免“错过低频地图发布后看起来没有话题输出”。
        pub_optimized_map_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/adaptive_backend/optimized_map",
            rclcpp::QoS(1).reliable().transient_local());

        // iSAM2 是增量式 pose graph 优化器，适合在线逐关键帧加入因子。
        gtsam::ISAM2Params params;
        params.relinearizeThreshold = 0.1;
        params.relinearizeSkip = 1;
        isam_ = std::make_unique<gtsam::ISAM2>(params);
        configureFileLogging();

        RCLCPP_INFO(
            get_logger(),
            "Adaptive degenerate backend started. Input: /Odometry, /cloud_registered_body, /adaptive_frontend/degeneracy_info");
    }

private:
    /**
     * @brief 初始化后端文件记录器。
     *
     * - loop CSV 采用逐事件写入；
     * - optimized TUM 每次优化后整体覆盖写入，因为回环会改变历史关键帧位姿。
     */
    void configureFileLogging()
    {
        if (!backend_log_enable_)
        {
            return;
        }

        // launch 参数经常指向一次新建的实验子目录。前端 CSV 会在运行时创建其
        // 父目录，但后端在构造阶段已经开始打开日志；这里主动创建三类输出的
        // 父目录，避免因目录尚不存在而静默关闭全部后端记录。
        std::error_code mkdir_error;
        const auto ensure_parent_directory = [&](const std::string &path)
        {
            const std::filesystem::path parent = std::filesystem::path(path).parent_path();
            if (!parent.empty())
            {
                std::filesystem::create_directories(parent, mkdir_error);
            }
        };
        ensure_parent_directory(backend_loop_csv_path_);
        ensure_parent_directory(backend_tum_path_);
        ensure_parent_directory(backend_full_tum_path_);
        if (mkdir_error)
        {
            RCLCPP_WARN(
                get_logger(), "Cannot create backend log directory: %s",
                mkdir_error.message().c_str());
        }

        loop_csv_.open(backend_loop_csv_path_, std::ios::out | std::ios::trunc);
        if (!loop_csv_.is_open())
        {
            RCLCPP_WARN(
                get_logger(),
                "Cannot open backend loop CSV: %s",
                backend_loop_csv_path_.c_str());
            backend_log_enable_ = false;
            return;
        }

        loop_csv_
            << "time,current_id,candidate_id,scan_context_distance,icp_fitness,"
            << "threshold,trans_error,yaw_error,degenerate_pair,accepted,"
            << "target_points,source_points,reverse_icp_fitness,"
            << "reciprocal_translation_error,reciprocal_yaw_error,"
            << "observability_correspondences,observability_condition_number,"
            << "observability_min_singular_value,path_separation,reason\n";
        loop_csv_.flush();

        std::ofstream tum(backend_tum_path_, std::ios::out | std::ios::trunc);
        if (!tum.is_open())
        {
            RCLCPP_WARN(
                get_logger(),
                "Cannot open backend TUM file: %s",
                backend_tum_path_.c_str());
        }

        std::ofstream full_tum(backend_full_tum_path_, std::ios::out | std::ios::trunc);
        if (!full_tum.is_open())
        {
            RCLCPP_WARN(
                get_logger(),
                "Cannot open backend full-rate TUM file: %s",
                backend_full_tum_path_.c_str());
        }

        RCLCPP_INFO(
            get_logger(),
            "Backend logs -> keyframe TUM: %s, full-rate TUM: %s, loops: %s",
            backend_tum_path_.c_str(),
            backend_full_tum_path_.c_str(),
            backend_loop_csv_path_.c_str());
    }

    /**
     * @brief 写入一条回环候选记录。
     */
    void writeLoopCsv(
        int current_id,
        int candidate_id,
        double sc_distance,
        double icp_fitness,
        double threshold,
        double trans_error,
        double yaw_error,
        bool degenerate_pair,
        bool accepted,
        size_t target_points,
        size_t source_points,
        const std::string &reason,
        const LoopValidationMetrics &metrics = LoopValidationMetrics{})
    {
        if (!backend_log_enable_ || !loop_csv_.is_open())
        {
            return;
        }

        const double time =
            (current_id >= 0 && current_id < static_cast<int>(keyframes_.size()))
                ? keyframes_[current_id].stamp.seconds()
                : now().seconds();

        loop_csv_ << std::fixed << std::setprecision(9)
                  << time << ","
                  << current_id << ","
                  << candidate_id << ","
                  << sc_distance << ","
                  << icp_fitness << ","
                  << threshold << ","
                  << trans_error << ","
                  << yaw_error << ","
                  << (degenerate_pair ? 1 : 0) << ","
                  << (accepted ? 1 : 0) << ","
                  << target_points << ","
                  << source_points << ","
                  << metrics.reverse_fitness << ","
                  << metrics.reciprocal_translation_error << ","
                  << metrics.reciprocal_yaw_error << ","
                  << metrics.observability_correspondences << ","
                  << metrics.observability_condition_number << ","
                  << metrics.observability_min_singular_value << ","
                  << metrics.path_separation << ","
                  << reason << "\n";
        loop_csv_.flush();
    }

    /**
     * @brief 将当前优化后的关键帧轨迹写成 TUM 格式。
     *
     * TUM 格式：
     *
     *     timestamp tx ty tz qx qy qz qw
     */
    void writeOptimizedTum() const
    {
        if (!backend_log_enable_ || backend_tum_path_.empty())
        {
            return;
        }

        std::ofstream tum(backend_tum_path_, std::ios::out | std::ios::trunc);
        if (!tum.is_open())
        {
            return;
        }

        tum << std::fixed << std::setprecision(9);
        for (const auto &kf : keyframes_)
        {
            const auto t = kf.optimized_pose.translation();
            const auto q = kf.optimized_pose.rotation().toQuaternion().normalized();
            tum << kf.stamp.seconds() << " "
                << t.x() << " " << t.y() << " " << t.z() << " "
                << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
                << "\n";
        }
    }

    /**
     * @brief 将 pose graph 的关键帧校正传播到全频前端轨迹并写为 TUM。
     *
     * 对每一帧前端位姿 T_front(t)，查找时间上最近的关键帧 k，并构造
     *
     *     Delta_k = T_opt_k * inv(T_odom_k),
     *     T_full_opt(t) = Delta_k * T_front(t).
     *
     * 这保证关键帧处严格等于 GTSAM 优化结果，同时为全部前端时间戳提供后端
     * 校正后的位姿。相比只输出稀疏关键帧，官方 ATE 可与前端方法公平同步。
     */
    void writeOptimizedFullTum() const
    {
        if (!backend_log_enable_ || backend_full_tum_path_.empty() ||
            odom_samples_.empty() || keyframes_.empty())
        {
            return;
        }

        std::ofstream tum(backend_full_tum_path_, std::ios::out | std::ios::trunc);
        if (!tum.is_open())
        {
            return;
        }

        tum << std::fixed << std::setprecision(9);
        size_t keyframe_index = 0;
        for (const auto &sample : odom_samples_)
        {
            const double sample_time = sample.stamp.seconds();
            while (keyframe_index + 1 < keyframes_.size() &&
                   keyframes_[keyframe_index + 1].stamp.seconds() <= sample_time)
            {
                ++keyframe_index;
            }

            // 在相邻两关键帧之间选择时间更近的一个，减小分段校正造成的误差。
            size_t nearest_index = keyframe_index;
            if (keyframe_index + 1 < keyframes_.size())
            {
                const double left_dt = std::abs(sample_time - keyframes_[keyframe_index].stamp.seconds());
                const double right_dt = std::abs(keyframes_[keyframe_index + 1].stamp.seconds() - sample_time);
                if (right_dt < left_dt)
                {
                    nearest_index = keyframe_index + 1;
                }
            }

            const auto &anchor = keyframes_[nearest_index];
            const gtsam::Pose3 correction =
                anchor.optimized_pose.compose(anchor.odom_pose.inverse());
            const gtsam::Pose3 optimized_pose = correction.compose(sample.odom_pose);
            const auto t = optimized_pose.translation();
            const auto q = optimized_pose.rotation().toQuaternion().normalized();
            tum << sample_time << " "
                << t.x() << " " << t.y() << " " << t.z() << " "
                << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
                << "\n";
        }
    }

    /**
     * @brief 缓存点云消息，供 odometry 按时间戳寻找最近帧。
     *
     * 不能只保存 latest cloud：当回放速率或回调调度变化时，最新点云可能
     * 已经跨过当前 odometry，导致关键帧点云和位姿错配。
     */
    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        cloud_queue_.push_back(msg);
        while (cloud_queue_.size() > sync_queue_size_)
        {
            cloud_queue_.pop_front();
        }
    }

    /**
     * @brief 解析前端发布的退化信息。
     *
     * data 字段来自 adaptive_laserMapping.cpp::publish_degeneracy_info()：
     * [0] time
     * [1] degeneracy_mode
     * [2] static_degenerate
     * [3] effective_ratio
     * [4] residual_mean
     * [5] normal_eigen_ratio
     * [6] condition_number
     * [7] window_degenerate_ratio
     * [12] window_condition_number_mean
     * [14] insert_ratio
     * [15:33] 第一阶段局部可定位性场和后验位姿协方差诊断量
     *
     * 后端暂时只取和“退化感知验证/加权”直接相关的字段。
     */
    void degeneracyHandler(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 15)
        {
            return;
        }

        DegeneracyInfo info;
        info.time = msg->data[0];
        info.mode = static_cast<int>(std::round(msg->data[1]));
        info.static_degenerate = msg->data[2] > 0.5;
        info.effective_ratio = msg->data[3];
        info.residual_mean = msg->data[4];
        info.normal_eigen_ratio = msg->data[5];
        info.condition_number = msg->data[6];
        info.window_degenerate_ratio = msg->data[7];
        info.window_condition_number_mean = msg->data[12];
        info.insert_ratio = msg->data[14];
        // 保持对旧版 15 元消息的兼容；新增字段存在时才解析。
        if (msg->data.size() >= 34)
        {
            info.localizability_observed_voxels =
                static_cast<std::size_t>(std::max(0.0, msg->data[15]));
            info.localizability_planarity_mean = msg->data[16];
            info.localizability_eigenvalues =
                Eigen::Vector3d(msg->data[17], msg->data[18], msg->data[19]);
            info.localizability_f0 = msg->data[20];
            info.localizability_lambda0 = msg->data[21];
            info.translation_cov_eigenvalues =
                Eigen::Vector3d(msg->data[22], msg->data[23], msg->data[24]);
            info.translation_weak_direction =
                Eigen::Vector3d(msg->data[25], msg->data[26], msg->data[27]);
            info.rotation_cov_eigenvalues =
                Eigen::Vector3d(msg->data[28], msg->data[29], msg->data[30]);
            info.rotation_weak_direction =
                Eigen::Vector3d(msg->data[31], msg->data[32], msg->data[33]);
        }
        std::lock_guard<std::mutex> lock(mtx_);
        degeneracy_queue_.push_back(info);
        while (degeneracy_queue_.size() > sync_queue_size_)
        {
            degeneracy_queue_.pop_front();
        }
    }

    /**
     * @brief 里程计回调，按时间戳同步点云与退化信息，并尝试创建关键帧。
     *
     * 后端以 odometry 为主触发，是因为关键帧必须首先有前端位姿。
     * 如果最近点云和 odometry 时间差太大，说明缓存未同步，直接跳过，避免错配。
     */
    void odomHandler(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 无论本帧能否与点云同步成关键帧，都保留其前端位姿，供全频后端
        // 优化轨迹导出使用。重复时间戳不重复记录。
        const rclcpp::Time odom_stamp(msg->header.stamp);
        if (odom_samples_.empty() || odom_stamp > odom_samples_.back().stamp)
        {
            odom_samples_.push_back({odom_stamp, odomToPose3(*msg)});
        }

        Cloud::Ptr cloud(new Cloud());
        DegeneracyInfo degeneracy;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (cloud_queue_.empty())
            {
                return;
            }

            const double odom_time = rclcpp::Time(msg->header.stamp).seconds();
            auto matched_cloud = cloud_queue_.end();
            double matched_cloud_time = -std::numeric_limits<double>::infinity();
            for (auto it = cloud_queue_.begin(); it != cloud_queue_.end(); ++it)
            {
                const double cloud_time = rclcpp::Time((*it)->header.stamp).seconds();
                // 因果匹配：不允许使用 odometry 之后的未来点云，也不重复使用
                // 上一次已经匹配过的点云。
                if (cloud_time <= odom_time &&
                    cloud_time > last_matched_cloud_time_ &&
                    cloud_time > matched_cloud_time)
                {
                    matched_cloud = it;
                    matched_cloud_time = cloud_time;
                }
            }
            if (matched_cloud == cloud_queue_.end() ||
                odom_time - matched_cloud_time > cloud_time_tolerance_)
            {
                return;
            }

            pcl::fromROSMsg(**matched_cloud, *cloud);
            if (cloud->empty())
            {
                return;
            }
            last_matched_cloud_time_ = matched_cloud_time;
            // 已经使用过的点云不再参与后续匹配，保证点云时间单调递增且单次使用。
            cloud_queue_.erase(cloud_queue_.begin(), std::next(matched_cloud));

            if (!degeneracy_queue_.empty())
            {
                auto matched_degeneracy = degeneracy_queue_.end();
                double matched_degeneracy_time =
                    -std::numeric_limits<double>::infinity();
                for (auto it = degeneracy_queue_.begin(); it != degeneracy_queue_.end(); ++it)
                {
                    // 退化状态同样只允许使用当前 odometry 之前的状态，
                    // 但允许多个关键帧复用最近一条状态。
                    if (it->time <= odom_time && it->time > matched_degeneracy_time)
                    {
                        matched_degeneracy = it;
                        matched_degeneracy_time = it->time;
                    }
                }
                if (matched_degeneracy != degeneracy_queue_.end() &&
                    odom_time - matched_degeneracy_time <= cloud_time_tolerance_)
                {
                    degeneracy = *matched_degeneracy;
                }
            }
        }

        maybeAddKeyFrame(*msg, cloud, degeneracy);
    }

    /**
     * @brief 根据位移/yaw 阈值决定是否保存关键帧。
     *
     * 保存关键帧后会立即执行四件事：
     * 1. 对点云下采样；
     * 2. 生成 ScanContext 描述子；
     * 3. 加入 pose graph 的 prior/odometry factor；
     * 4. 检测回环并执行一次增量优化。
     */
    void maybeAddKeyFrame(
        const nav_msgs::msg::Odometry &odom,
        const Cloud::Ptr &cloud,
        const DegeneracyInfo &degeneracy)
    {
        const gtsam::Pose3 pose = odomToPose3(odom);
        const double yaw = yawFromPose(odom.pose.pose);

        if (!keyframes_.empty())
        {
            const auto last_t = keyframes_.back().odom_pose.translation();
            const auto cur_t = pose.translation();
            const double distance =
                std::hypot(cur_t.x() - last_t.x(), cur_t.y() - last_t.y());
            const double yaw_delta = std::abs(wrapAngle(yaw - last_keyframe_yaw_));
            if (distance < keyframe_distance_ && yaw_delta < keyframe_yaw_)
            {
                return;
            }
        }

        // 关键帧点云不直接使用进入 ikd-tree 的点。
        // 原因：ScanContext/ICP 需要相对完整的几何结构；前端筛选太强可能使描述子变稀疏。
        // 但该关键帧仍携带 degeneracy 信息，因此创新点 1 会影响后端验证和加权。
        Cloud::Ptr downsampled(new Cloud());
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(keyframe_leaf_size_, keyframe_leaf_size_, keyframe_leaf_size_);
        voxel.setInputCloud(cloud);
        voxel.filter(*downsampled);

        KeyFrame kf;
        kf.id = static_cast<int>(keyframes_.size());
        kf.stamp = odom.header.stamp;
        kf.odom_pose = pose;
        kf.optimized_pose = pose;
        kf.cloud = downsampled;
        kf.degeneracy = degeneracy;
        kf.scan_context = makeScanContext(*downsampled);
        kf.ring_key = makeRingKey(kf.scan_context);
        kf.sector_key = makeSectorKey(kf.scan_context);

        addPoseGraphFactors(kf);
        keyframes_.push_back(kf);
        last_keyframe_yaw_ = yaw;

        detectAndAddLoop(kf.id);
        optimizeAndPublish();

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "backend keyframes=%zu, latest_mode=%d, D=%.2f",
            keyframes_.size(), degeneracy.mode, degeneracy.severity());
    }

    /**
     * @brief 构造 Scan Context 的 ring × sector 最大高度描述子。
     *
     * 以关键帧雷达坐标系为原点，将 xy 平面划分为极坐标网格。第 r 环、第 s 扇区的
     * 数值为该格内点的最大 z 值。与官方 Scan Context 一致，旋转只会造成列（sector）
     * 的循环移位，因此后续可通过 sector shift 对齐两个描述子。
     */
    std::vector<float> makeScanContext(const Cloud &cloud) const
    {
        const int rows = std::max(1, scan_context_rings_);
        const int cols = std::max(1, scan_context_sectors_);
        std::vector<float> desc(rows * cols, -std::numeric_limits<float>::infinity());
        const double ring_step = scan_context_max_radius_ / static_cast<double>(rows);
        const double sector_step = 2.0 * M_PI / static_cast<double>(cols);

        for (const auto &pt : cloud.points)
        {
            const double radius = std::hypot(pt.x, pt.y);
            if (radius < 1e-3 || radius >= scan_context_max_radius_)
            {
                continue;
            }
            double angle = std::atan2(pt.y, pt.x);
            if (angle < 0.0)
            {
                angle += 2.0 * M_PI;
            }

            const int ring = std::min(rows - 1, static_cast<int>(radius / ring_step));
            const int sector = std::min(cols - 1, static_cast<int>(angle / sector_step));
            float &cell = desc[ring * cols + sector];
            cell = std::max(cell, pt.z);
        }

        for (auto &value : desc)
        {
            if (!std::isfinite(value))
            {
                value = 0.0f;
            }
        }
        return desc;
    }

    /**
     * @brief 从 Scan Context 网格生成旋转不变的 ring key。
     *
     * 第 r 维是第 r 个环内所有扇区的均值：
     *
     *     RK(r) = (1 / S) sum_s SC(r, s)
     *
     * 车辆/传感器绕 z 轴转动只会交换扇区列，不改变环内均值；因此 ring key 适合
     * 在全部历史关键帧中快速预筛候选，但不能单独用作回环判据。
     */
    std::vector<float> makeRingKey(const std::vector<float> &descriptor) const
    {
        const int rows = std::max(1, scan_context_rings_);
        const int cols = std::max(1, scan_context_sectors_);
        std::vector<float> key(rows, 0.0f);
        if (descriptor.size() != static_cast<size_t>(rows * cols))
        {
            return key;
        }
        for (int r = 0; r < rows; ++r)
        {
            double sum = 0.0;
            for (int s = 0; s < cols; ++s)
            {
                sum += descriptor[r * cols + s];
            }
            key[r] = static_cast<float>(sum / static_cast<double>(cols));
        }
        return key;
    }

    /**
     * @brief 从 Scan Context 网格生成 sector key，用于估计航向循环偏移。
     *
     * 第 s 维是第 s 个扇区内所有环的均值。比较两个 sector key 的所有循环移位，
     * 可得到描述子对齐所需的粗略 yaw 偏移。
     */
    std::vector<float> makeSectorKey(const std::vector<float> &descriptor) const
    {
        const int rows = std::max(1, scan_context_rings_);
        const int cols = std::max(1, scan_context_sectors_);
        std::vector<float> key(cols, 0.0f);
        if (descriptor.size() != static_cast<size_t>(rows * cols))
        {
            return key;
        }
        for (int s = 0; s < cols; ++s)
        {
            double sum = 0.0;
            for (int r = 0; r < rows; ++r)
            {
                sum += descriptor[r * cols + s];
            }
            key[s] = static_cast<float>(sum / static_cast<double>(rows));
        }
        return key;
    }

    /** @brief 计算等长向量的均方根距离，用于 ring key 粗检索。 */
    double vectorDistance(
        const std::vector<float> &a,
        const std::vector<float> &b) const
    {
        if (a.size() != b.size() || a.empty())
        {
            return std::numeric_limits<double>::infinity();
        }
        double squared_sum = 0.0;
        for (size_t i = 0; i < a.size(); ++i)
        {
            const double d = static_cast<double>(a[i]) - static_cast<double>(b[i]);
            squared_sum += d * d;
        }
        return std::sqrt(squared_sum / static_cast<double>(a.size()));
    }

    /**
     * @brief 计算两个 sector key 的最优循环移位。
     *
     * 返回的 shift 满足：candidate 的第 (s + shift) 列与 current 的第 s 列对应。
     */
    int estimateSectorShift(
        const std::vector<float> &current,
        const std::vector<float> &candidate) const
    {
        if (current.size() != candidate.size() || current.empty())
        {
            return 0;
        }
        const int cols = static_cast<int>(current.size());
        int best_shift = 0;
        double best_distance = std::numeric_limits<double>::infinity();
        for (int shift = 0; shift < cols; ++shift)
        {
            double squared_sum = 0.0;
            for (int s = 0; s < cols; ++s)
            {
                const double d = static_cast<double>(current[s]) -
                    static_cast<double>(candidate[(s + shift) % cols]);
                squared_sum += d * d;
            }
            if (squared_sum < best_distance)
            {
                best_distance = squared_sum;
                best_shift = shift;
            }
        }
        return best_shift;
    }

    /**
     * @brief 计算指定列循环移位下的正式 Scan Context 距离。
     *
     * 每个非空扇区单独计算余弦距离并取平均：
     *
     *     d = mean_s [1 - (c_s dot h_s) / (||c_s|| ||h_s||)]
     *
     * 其中 h_s 是候选描述子循环移位后的第 s 列。该列级距离不会因不同扇区的点数
     * 或绝对高度整体放大而被单个大值主导。
     */
    double scanContextDistanceAtShift(
        const std::vector<float> &current,
        const std::vector<float> &candidate,
        int shift) const
    {
        const int rows = std::max(1, scan_context_rings_);
        const int cols = std::max(1, scan_context_sectors_);
        if (current.size() != static_cast<size_t>(rows * cols) ||
            candidate.size() != current.size())
        {
            return 1.0;
        }

        double distance_sum = 0.0;
        int valid_columns = 0;
        for (int s = 0; s < cols; ++s)
        {
            const int candidate_s = (s + shift + cols) % cols;
            double dot = 0.0;
            double current_norm = 0.0;
            double candidate_norm = 0.0;
            for (int r = 0; r < rows; ++r)
            {
                const double a = current[r * cols + s];
                const double b = candidate[r * cols + candidate_s];
                dot += a * b;
                current_norm += a * a;
                candidate_norm += b * b;
            }
            if (current_norm < 1e-9 || candidate_norm < 1e-9)
            {
                continue;
            }
            distance_sum += 1.0 - dot / std::sqrt(current_norm * candidate_norm);
            ++valid_columns;
        }
        return valid_columns > 0 ? distance_sum / static_cast<double>(valid_columns) : 1.0;
    }

    /**
     * @brief 使用 sector key 粗对齐并在其附近细搜索，返回最小 SC 距离。
     */
    std::pair<double, int> alignedScanContextDistance(const KeyFrame &current,
                                                      const KeyFrame &candidate) const
    {
        const int cols = std::max(1, scan_context_sectors_);
        const int coarse_shift = estimateSectorShift(current.sector_key, candidate.sector_key);
        const int radius = std::min(std::max(0, scan_context_alignment_search_radius_), cols / 2);
        double best_distance = std::numeric_limits<double>::infinity();
        int best_shift = coarse_shift;
        for (int delta = -radius; delta <= radius; ++delta)
        {
            const int shift = (coarse_shift + delta + cols) % cols;
            const double distance =
                scanContextDistanceAtShift(current.scan_context, candidate.scan_context, shift);
            if (distance < best_distance)
            {
                best_distance = distance;
                best_shift = shift;
            }
        }
        return {best_distance, best_shift};
    }

    /**
     * @brief 向 pose graph 中加入 prior 或 odometry factor。
     *
     * 第一帧：
     * - 加 prior factor 固定全局坐标系。
     *
     * 后续帧：
     * - 使用前后关键帧的前端 odometry 相对位姿作为 BetweenFactor 测量；
     * - 使用退化强度 D_i 调整噪声 sigma。
     *
     * 数学形式：
     *
     *     min_X Σ || Log( Z_ij^{-1} (X_i^{-1} X_j) ) ||^2_{Ω_ij}
     *
     * 其中 Ω_ij = Σ_ij^{-1}。
     * 当 Persistent 退化更强时，sigma 增大，Ω 变小，
     * 后端优化时会降低对该段前端里程计的信任。
     */
    void addPoseGraphFactors(const KeyFrame &kf)
    {
        if (kf.id == 0)
        {
            const auto prior_noise =
                gtsam::noiseModel::Diagonal::Sigmas(
                    (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.05, 0.05, 0.05).finished());
            graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, kf.odom_pose, prior_noise));
            values_.insert(0, kf.odom_pose);
            return;
        }

        const KeyFrame &prev = keyframes_.back();
        const gtsam::Pose3 relative = prev.odom_pose.between(kf.odom_pose);
        const double d = std::max(prev.degeneracy.severity(), kf.degeneracy.severity());
        // 退化感知 odometry 边权重：
        // d 越大，trans_sigma / rot_sigma 越大，表示这段前端里程计越不可靠。
        const double trans_sigma = 0.10 + 0.40 * d;
        const double rot_sigma = 0.05 + 0.25 * d;
        const auto odom_noise =
            gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << rot_sigma, rot_sigma, rot_sigma,
                 trans_sigma, trans_sigma, trans_sigma)
                    .finished());

        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            prev.id, kf.id, relative, odom_noise));
        values_.insert(kf.id, kf.odom_pose);
    }

    /**
     * @brief 使用 ScanContext 为当前关键帧寻找历史回环候选。
     *
     * 流程：
     * 1. 排除最近 recent_exclusion_num_ 个关键帧；
     * 2. 用旋转不变 ring key 从全部历史帧中取最近的少量候选；
     * 3. 对粗候选用 sector key 循环对齐，计算正式 Scan Context 距离；
     * 4. 按正式距离排序，取前 loop_candidate_top_k_ 个；
     * 5. 依次进入 ICP 几何验证，第一条通过验证的候选被加入 pose graph。
     *
     * ScanContext 只负责“召回候选”，不会直接接受回环；
     * 真正是否加入 loop factor 由 addIcpLoopFactor() 决定。
     *
     * 为什么从 top-1 改成 top-K：
     * - 长走廊/隧道场景中很多位置描述子相似；
     * - top-1 可能是局部几何最像但 ICP 不可靠的错误候选；
     * - top-K 提高召回率，但质量门控不放松，因此不会直接降低回环质量。
     */
    void detectAndAddLoop(int current_id)
    {
        if (current_id <= recent_exclusion_num_)
        {
            return;
        }

        const auto &cur = keyframes_[current_id];
        // 第一阶段：ring key 不受 yaw 影响，代价低，适合作为全历史库的粗检索键。
        std::vector<std::pair<double, int>> ringkey_candidates;

        for (int i = 0; i < current_id - recent_exclusion_num_; ++i)
        {
            ringkey_candidates.emplace_back(
                vectorDistance(cur.ring_key, keyframes_[i].ring_key), i);
        }

        if (ringkey_candidates.empty())
        {
            return;
        }
        if (loop_index_container_.count(current_id) > 0)
        {
            return;
        }

        std::sort(
            ringkey_candidates.begin(),
            ringkey_candidates.end(),
            [](const auto &lhs, const auto &rhs)
            {
                return lhs.first < rhs.first;
            });

        // 第二阶段：正式 Scan Context 距离包含 sector 循环对齐，保存 shift 便于日志/调试扩展。
        struct ScanContextCandidate
        {
            double distance;
            int id;
            int sector_shift;
        };
        std::vector<ScanContextCandidate> candidates;
        const int ringkey_limit = std::min(
            static_cast<int>(ringkey_candidates.size()),
            std::max(1, scan_context_ringkey_candidate_num_));
        for (int idx = 0; idx < ringkey_limit; ++idx)
        {
            const int candidate_id = ringkey_candidates[idx].second;
            const auto [distance, sector_shift] =
                alignedScanContextDistance(cur, keyframes_[candidate_id]);
            if (distance <= scan_context_distance_threshold_)
            {
                candidates.push_back({distance, candidate_id, sector_shift});
            }
        }
        if (candidates.empty())
        {
            return;
        }
        std::sort(
            candidates.begin(),
            candidates.end(),
            [](const ScanContextCandidate &lhs, const ScanContextCandidate &rhs)
            {
                return lhs.distance < rhs.distance;
            });

        // 先按累计行驶距离去除近路径候选，再截取 top-K。否则 top-1 很容易被
        // 隧道中仅相隔十几米、描述子却最相似的伪回环占据，真正远时序候选
        // 根本没有机会进入 ICP 验证。
        std::vector<ScanContextCandidate> path_separated_candidates;
        path_separated_candidates.reserve(candidates.size());
        for (const auto &candidate : candidates)
        {
            const double path_separation =
                keyframePathSeparation(candidate.id, current_id);
            if (loop_min_path_separation_ > 0.0 &&
                path_separation < loop_min_path_separation_)
            {
                LoopValidationMetrics metrics;
                metrics.path_separation = path_separation;
                writeLoopCsv(
                    current_id, candidate.id, candidate.distance,
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(), false, false, 0,
                    cur.cloud ? cur.cloud->size() : 0,
                    "path_separation_reject", metrics);
                continue;
            }
            path_separated_candidates.push_back(candidate);
        }
        if (path_separated_candidates.empty())
        {
            return;
        }

        const int max_try = std::min(
            static_cast<int>(path_separated_candidates.size()),
            std::max(1, loop_candidate_top_k_));
        for (int idx = 0; idx < max_try; ++idx)
        {
            const double sc_distance = path_separated_candidates[idx].distance;
            const int candidate_id = path_separated_candidates[idx].id;
            const double path_separation =
                keyframePathSeparation(candidate_id, current_id);
            std::string cooldown_reason;
            if (isLoopCoolingDown(current_id, candidate_id, cooldown_reason))
            {
                writeLoopCsv(
                    current_id,
                    candidate_id,
                    sc_distance,
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(),
                    false,
                    false,
                    0,
                    cur.cloud ? cur.cloud->size() : 0,
                    cooldown_reason);
                RCLCPP_INFO(
                    get_logger(),
                    "Reject loop cur=%d cand=%d: %s",
                    current_id, candidate_id, cooldown_reason.c_str());
                continue;
            }
            if (addIcpLoopFactor(
                    current_id, candidate_id, sc_distance, path_separation))
            {
                return;
            }
        }
    }

    /**
     * @brief 判断候选回环是否处于冷却期。
     *
     * 冷却的对象不是单个 current_id，而是已经接受的回环事件：
     * - current 关键帧相近：避免连续帧反复加回环；
     * - candidate 关键帧相近：避免多个当前帧重复连接同一历史区域；
     * - 两端都相近：即使两者不是完全相同的 ID，也视为同一回环事件。
     */
    bool isLoopCoolingDown(
        int current_id,
        int candidate_id,
        std::string &reason) const
    {
        const int current_gap = std::max(0, loop_current_cooldown_keyframes_);
        const int candidate_gap = std::max(0, loop_candidate_cooldown_keyframes_);
        const int pair_gap = std::max(0, loop_pair_cooldown_keyframes_);

        for (const auto &accepted : accepted_loop_pairs_)
        {
            const int current_delta = std::abs(current_id - accepted.first);
            const int candidate_delta = std::abs(candidate_id - accepted.second);

            if (current_delta < current_gap)
            {
                reason = "current_cooldown_reject";
                return true;
            }
            if (candidate_delta < candidate_gap)
            {
                reason = "candidate_cooldown_reject";
                return true;
            }
            if (current_delta < pair_gap && candidate_delta < pair_gap)
            {
                reason = "loop_pair_cooldown_reject";
                return true;
            }
        }
        return false;
    }

    /**
     * @brief 计算两个关键帧之间沿前端轨迹的累计行驶距离。
     *
     * 它使用相邻关键帧 odometry 位姿的平移增量求和，而不是两端欧氏直线距离。
     * 因而适合区分“机器人已经绕回原处”和“仍在同一条隧道中向前走了十几米”。
     */
    double keyframePathSeparation(int first_id, int second_id) const
    {
        if (keyframes_.empty())
        {
            return 0.0;
        }
        const int start = std::max(0, std::min(first_id, second_id));
        const int end = std::min(
            static_cast<int>(keyframes_.size()) - 1,
            std::max(first_id, second_id));
        double distance = 0.0;
        for (int i = start + 1; i <= end; ++i)
        {
            distance += (keyframes_[i].odom_pose.translation() -
                         keyframes_[i - 1].odom_pose.translation()).norm();
        }
        return distance;
    }

    /**
     * @brief 构建候选关键帧附近的局部子地图，并统一变换到 center_id 坐标系。
     *
     * 为什么要做子地图：
     * - 单帧 VLP-16 点云较稀疏，当前帧 vs 历史单帧 ICP 容易对应点不足；
     * - 拼接候选帧前后若干关键帧后，target 具有更完整的墙面、地面和结构细节；
     * - 这样更接近 LIO-SAM 中 loopFindNearKeyframes() 的做法。
     *
     * 坐标变换：
     * - 每个关键帧 cloud_i 原本在第 i 帧 LiDAR/body 坐标系；
     * - T_w_i 为第 i 帧前端 odometry 位姿；
     * - 若要把 cloud_i 转到 center 帧坐标系：
     *
     *       p_center = T_center^{-1} T_i p_i
     *
     * - GTSAM 中 keyframes_[center].odom_pose.between(keyframes_[i].odom_pose)
     *   正好就是 T_center^{-1} T_i。
     */
    Cloud::Ptr buildLocalSubmapInCenterFrame(int center_id) const
    {
        Cloud::Ptr submap(new Cloud());
        if (keyframes_.empty() || center_id < 0 || center_id >= static_cast<int>(keyframes_.size()))
        {
            return submap;
        }

        const int search_num = std::max(0, loop_submap_search_num_);
        const int start_id = std::max(0, center_id - search_num);
        const int end_id =
            std::min(static_cast<int>(keyframes_.size()) - 1, center_id + search_num);

        for (int i = start_id; i <= end_id; ++i)
        {
            if (!keyframes_[i].cloud || keyframes_[i].cloud->empty())
            {
                continue;
            }

            const gtsam::Pose3 tf_center_i =
                keyframes_[center_id].odom_pose.between(keyframes_[i].odom_pose);
            Cloud transformed;
            pcl::transformPointCloud(
                *keyframes_[i].cloud,
                transformed,
                tf_center_i.matrix().cast<float>());
            *submap += transformed;
        }

        if (submap->empty())
        {
            return submap;
        }

        Cloud::Ptr downsampled(new Cloud());
        pcl::VoxelGrid<PointType> voxel;
        const float leaf = static_cast<float>(std::max(0.05, loop_submap_leaf_size_));
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.setInputCloud(submap);
        voxel.filter(*downsampled);
        return downsampled;
    }

    /**
     * @brief 使用 ICP 验证 ScanContext 回环候选，并在可靠时加入 loop factor。
     *
     * 退化感知体现在：
     * - target 不再是历史单帧，而是候选附近 ±N 帧拼接得到的局部子地图；
     * - 如果当前关键帧或候选关键帧处于 Persistent，说明结构重复/约束弱风险更高；
     * - 因此使用更严格的 icp_fitness_threshold_degenerate_；
     * - 只有 ICP 收敛且 fitness 足够小，才把该回环加入 pose graph。
     *
     * 这样避免长走廊、隧道中“看起来相似但位置不同”的假回环。
     */
    bool addIcpLoopFactor(
        int current_id,
        int candidate_id,
        double sc_distance,
        double path_separation)
    {
        Cloud::Ptr target_submap = buildLocalSubmapInCenterFrame(candidate_id);
        if (!target_submap || target_submap->empty())
        {
            writeLoopCsv(
                current_id,
                candidate_id,
                sc_distance,
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                false,
                false,
                0,
                keyframes_[current_id].cloud ? keyframes_[current_id].cloud->size() : 0,
                "empty_target_submap");
            RCLCPP_INFO(
                get_logger(),
                "Reject loop cur=%d cand=%d: empty target submap",
                current_id, candidate_id);
            return false;
        }

        Cloud::Ptr aligned(new Cloud());
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
        icp.setMaximumIterations(50);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setInputSource(keyframes_[current_id].cloud);
        icp.setInputTarget(target_submap);

        // ICP 的 source 是当前关键帧点云，target 是历史候选关键帧点云。
        // PCL 需要的初值是：source -> target。
        // 对世界系位姿 T_w_cur、T_w_cand，有：
        //     p_w = T_w_cur * p_cur
        //     p_cand = T_cand_w * p_w
        // 因此 source->target 初值应为 T_cand_w * T_w_cur，
        // 即 candidate_pose.between(current_pose)。
        const gtsam::Pose3 initial_relative =
            keyframes_[candidate_id].odom_pose.between(keyframes_[current_id].odom_pose);
        Eigen::Matrix4f initial_guess =
            initial_relative.matrix().cast<float>();
        icp.align(*aligned, initial_guess);

        if (!icp.hasConverged())
        {
            writeLoopCsv(
                current_id,
                candidate_id,
                sc_distance,
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                false,
                false,
                target_submap->size(),
                keyframes_[current_id].cloud ? keyframes_[current_id].cloud->size() : 0,
                "icp_not_converged");
            RCLCPP_INFO(
                get_logger(),
                "Reject loop cur=%d cand=%d: ICP not converged, source=%zu target=%zu",
                current_id, candidate_id,
                keyframes_[current_id].cloud ? keyframes_[current_id].cloud->size() : 0,
                target_submap->size());
            return false;
        }

        const double fitness = icp.getFitnessScore();
        // 只要回环两端任意一端处于 Persistent，就按退化回环处理。
        const bool degenerate_pair =
            keyframes_[current_id].degeneracy.mode >= 2 ||
            keyframes_[candidate_id].degeneracy.mode >= 2;
        const double fitness_threshold =
            degenerate_pair ? icp_fitness_threshold_degenerate_
                            : icp_fitness_threshold_normal_;
        if (fitness > fitness_threshold)
        {
            writeLoopCsv(
                current_id,
                candidate_id,
                sc_distance,
                fitness,
                fitness_threshold,
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                degenerate_pair,
                false,
                target_submap->size(),
                keyframes_[current_id].cloud ? keyframes_[current_id].cloud->size() : 0,
                "fitness_reject");
            RCLCPP_INFO(
                get_logger(),
                "Reject loop cur=%d cand=%d: SC=%.3f ICP=%.3f threshold=%.3f deg_pair=%d target=%zu",
                current_id, candidate_id, sc_distance, fitness,
                fitness_threshold, degenerate_pair ? 1 : 0, target_submap->size());
            return false;
        }

        const Eigen::Matrix4f forward_transformation = icp.getFinalTransformation();
        LoopValidationMetrics validation_metrics;
        validation_metrics.path_separation = path_separation;

        // 第三层：双向 ICP 验证。单向 ICP 在重复走廊内可能把两段平行墙面
        // 贴合；若反向配准不能收敛、fitness 很差或正反变换不互逆，则拒绝。
        if (loop_bidirectional_icp_enable_)
        {
            Cloud::Ptr reverse_aligned(new Cloud());
            pcl::IterativeClosestPoint<PointType, PointType> reverse_icp;
            reverse_icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
            reverse_icp.setMaximumIterations(50);
            reverse_icp.setTransformationEpsilon(1e-6);
            reverse_icp.setEuclideanFitnessEpsilon(1e-6);
            reverse_icp.setInputSource(target_submap);
            reverse_icp.setInputTarget(keyframes_[current_id].cloud);
            reverse_icp.align(*reverse_aligned, initial_guess.inverse());

            if (!reverse_icp.hasConverged())
            {
                writeLoopCsv(
                    current_id, candidate_id, sc_distance, fitness, fitness_threshold,
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(), degenerate_pair, false,
                    target_submap->size(),
                    keyframes_[current_id].cloud ? keyframes_[current_id].cloud->size() : 0,
                    "bidirectional_icp_not_converged", validation_metrics);
                RCLCPP_INFO(
                    get_logger(),
                    "Reject loop cur=%d cand=%d: reverse ICP not converged",
                    current_id, candidate_id);
                return false;
            }

            validation_metrics.reverse_fitness = reverse_icp.getFitnessScore();
            const Eigen::Matrix4f reverse_transformation =
                reverse_icp.getFinalTransformation();
            const Eigen::Matrix4f reciprocal =
                forward_transformation * reverse_transformation;
            validation_metrics.reciprocal_translation_error =
                reciprocal.block<3, 1>(0, 3).cast<double>().norm();
            validation_metrics.reciprocal_yaw_error = std::abs(wrapAngle(
                std::atan2(static_cast<double>(reciprocal(1, 0)),
                           static_cast<double>(reciprocal(0, 0)))));

            if (validation_metrics.reverse_fitness > loop_bidirectional_icp_fitness_threshold_ ||
                validation_metrics.reciprocal_translation_error >
                    loop_reciprocal_translation_threshold_ ||
                validation_metrics.reciprocal_yaw_error > loop_reciprocal_yaw_threshold_)
            {
                writeLoopCsv(
                    current_id, candidate_id, sc_distance, fitness, fitness_threshold,
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(), degenerate_pair, false,
                    target_submap->size(),
                    keyframes_[current_id].cloud ? keyframes_[current_id].cloud->size() : 0,
                    "bidirectional_consistency_reject", validation_metrics);
                RCLCPP_INFO(
                    get_logger(),
                    "Reject loop cur=%d cand=%d: reverse ICP=%.3f reciprocal trans=%.3f yaw=%.3f",
                    current_id, candidate_id, validation_metrics.reverse_fitness,
                    validation_metrics.reciprocal_translation_error,
                    validation_metrics.reciprocal_yaw_error);
                return false;
            }
        }

        // 第四层：Hessian-SVD 可观测性验证。特别针对“墙面拟合很好，
        // 但沿隧道方向没有约束”的情况；所有指标也写入 CSV 便于标定阈值。
        if (loop_observability_enable_)
        {
            const LoopValidationMetrics observability_metrics = evaluateLoopObservability(
                keyframes_[current_id].cloud, target_submap, forward_transformation,
                icp_max_correspondence_distance_);
            validation_metrics.observability_correspondences =
                observability_metrics.observability_correspondences;
            validation_metrics.observability_condition_number =
                observability_metrics.observability_condition_number;
            validation_metrics.observability_min_singular_value =
                observability_metrics.observability_min_singular_value;

            const bool observable =
                validation_metrics.observability_correspondences >=
                    static_cast<size_t>(std::max(6, loop_observability_min_correspondences_)) &&
                std::isfinite(validation_metrics.observability_condition_number) &&
                validation_metrics.observability_condition_number <=
                    loop_observability_max_condition_number_ &&
                std::isfinite(validation_metrics.observability_min_singular_value) &&
                validation_metrics.observability_min_singular_value >=
                    loop_observability_min_singular_value_;
            if (loop_observability_reject_enable_ && !observable)
            {
                writeLoopCsv(
                    current_id, candidate_id, sc_distance, fitness, fitness_threshold,
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(), degenerate_pair, false,
                    target_submap->size(),
                    keyframes_[current_id].cloud ? keyframes_[current_id].cloud->size() : 0,
                    "observability_reject", validation_metrics);
                RCLCPP_INFO(
                    get_logger(),
                    "Reject loop cur=%d cand=%d: observability corr=%zu cond=%.2e min_sv=%.2e",
                    current_id, candidate_id,
                    validation_metrics.observability_correspondences,
                    validation_metrics.observability_condition_number,
                    validation_metrics.observability_min_singular_value);
                return false;
            }
        }

        // ICP 输出的是 source(current) -> target(candidate) 的变换：
        //     T_candidate^{-1} T_current
        // 对 GTSAM BetweenFactor(candidate_id, current_id) 来说，
        // 这正好是 candidate -> current 的测量 Z_candidate_current。
        const gtsam::Pose3 measurement = eigenToPose3(forward_transformation);
        // 第二层回环质量门控：ICP 的相对位姿不能和前端里程计预测差太多。
        //
        // 仅靠 fitness 会有风险：长走廊/隧道里局部几何重复，错误位置也可能配准出较小残差。
        // 因此这里比较两种相对位姿：
        //
        //     Z_odom = T_candidate^{-1} T_current
        //     Z_icp  = ICP(source=current -> target=candidate)
        //
        // 若二者平移差或 yaw 差超过阈值，说明这个回环虽然“形状像”，但“位置关系不可信”。
        const Eigen::Vector3d odom_t = initial_relative.translation();
        const Eigen::Vector3d icp_t = measurement.translation();
        const double trans_error = (odom_t - icp_t).norm();
        const double yaw_error =
            std::abs(wrapAngle(yawFromPose3(initial_relative) - yawFromPose3(measurement)));
        const double trans_threshold =
            degenerate_pair ? loop_consistency_trans_degenerate_
                            : loop_consistency_trans_normal_;
        const double yaw_threshold =
            degenerate_pair ? loop_consistency_yaw_degenerate_
                            : loop_consistency_yaw_normal_;
        if (trans_error > trans_threshold || yaw_error > yaw_threshold)
        {
            writeLoopCsv(
                current_id,
                candidate_id,
                sc_distance,
                fitness,
                fitness_threshold,
                trans_error,
                yaw_error,
                degenerate_pair,
                false,
                target_submap->size(),
                keyframes_[current_id].cloud ? keyframes_[current_id].cloud->size() : 0,
                "consistency_reject",
                validation_metrics);
            RCLCPP_INFO(
                get_logger(),
                "Reject loop cur=%d cand=%d: consistency trans=%.3f/%.3f yaw=%.3f/%.3f SC=%.3f ICP=%.3f deg_pair=%d target=%zu",
                current_id, candidate_id,
                trans_error, trans_threshold,
                yaw_error, yaw_threshold,
                sc_distance, fitness,
                degenerate_pair ? 1 : 0,
                target_submap->size());
            return false;
        }

        // 退化相关回环虽然通过了更严格验证，但仍给稍大的基础 sigma，避免
        // 单条回环过度拉扯轨迹。
        const double sigma =
            degenerate_pair ? loop_noise_sigma_degenerate_ : loop_noise_sigma_normal_;
        const auto loop_noise =
            gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << sigma, sigma, sigma,
                 2.0 * sigma, 2.0 * sigma, 2.0 * sigma)
                    .finished());

        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            candidate_id, current_id, measurement, loop_noise));
        loop_index_container_[current_id] = candidate_id;
        accepted_loop_pairs_.emplace_back(current_id, candidate_id);
        writeLoopCsv(
            current_id,
            candidate_id,
            sc_distance,
            fitness,
            fitness_threshold,
            trans_error,
            yaw_error,
            degenerate_pair,
            true,
            target_submap->size(),
            keyframes_[current_id].cloud ? keyframes_[current_id].cloud->size() : 0,
            "accepted",
            validation_metrics);

        RCLCPP_WARN(
            get_logger(),
            "Accept loop cur=%d cand=%d: SC=%.3f ICP=%.3f deg_pair=%d target=%zu",
            current_id, candidate_id, sc_distance, fitness,
            degenerate_pair ? 1 : 0, target_submap->size());
        return true;
    }

    /**
     * @brief 执行 iSAM2 增量优化，并发布优化后的关键帧轨迹。
     *
     * 当前版本发布两类结果：
     * - optimized path：优化后的关键帧轨迹；
     * - optimized map：用优化后位姿重新拼接关键帧点云得到的后端地图。
     */
    void optimizeAndPublish()
    {
        if (values_.empty() && graph_.empty())
        {
            return;
        }

        isam_->update(graph_, values_);
        isam_->update();
        graph_.resize(0);
        values_.clear();

        const gtsam::Values result = isam_->calculateEstimate();
        nav_msgs::msg::Path path;
        path.header.stamp = now();
        path.header.frame_id = "camera_init";
        path.poses.reserve(keyframes_.size());

        for (auto &kf : keyframes_)
        {
            if (!result.exists(kf.id))
            {
                continue;
            }
            kf.optimized_pose = result.at<gtsam::Pose3>(kf.id);
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = kf.stamp;
            pose_msg.header.frame_id = "camera_init";
            pose_msg.pose = pose3ToMsg(kf.optimized_pose);
            path.poses.push_back(pose_msg);
        }

        pub_optimized_path_->publish(path);
        publishOptimizedMapIfNeeded();
        writeOptimizedTum();
        writeOptimizedFullTum();
    }

    /**
     * @brief 按优化后的关键帧位姿发布后端点云地图。
     *
     * 对每个关键帧 i：
     *
     *     p_world = T_world_i^opt * p_i
     *
     * 其中 p_i 是保存在关键帧局部坐标系下的点云，
     * T_world_i^opt 是 pose graph 优化后的关键帧位姿。
     *
     * 注意：
     * - 这里仅发布 /adaptive_backend/optimized_map；
     * - 不保存 PCD；
     * - 为降低开销，默认每 optimized_map_publish_interval_ 个关键帧发布一次。
     */
    void publishOptimizedMapIfNeeded()
    {
        if (!publish_optimized_map_ || !pub_optimized_map_)
        {
            return;
        }
        if (keyframes_.empty())
        {
            return;
        }

        const int interval = std::max(1, optimized_map_publish_interval_);
        const bool should_publish =
            keyframes_.size() <= 2 ||
            keyframes_.size() % static_cast<size_t>(interval) == 0 ||
            loop_index_container_.size() != last_published_loop_count_;
        if (!should_publish)
        {
            return;
        }

        Cloud::Ptr global_map(new Cloud());
        for (const auto &kf : keyframes_)
        {
            if (!kf.cloud || kf.cloud->empty())
            {
                continue;
            }

            Cloud transformed;
            pcl::transformPointCloud(
                *kf.cloud,
                transformed,
                kf.optimized_pose.matrix().cast<float>());
            *global_map += transformed;
        }

        if (global_map->empty())
        {
            return;
        }

        Cloud::Ptr downsampled(new Cloud());
        pcl::VoxelGrid<PointType> voxel;
        const float leaf = static_cast<float>(std::max(0.05, optimized_map_leaf_size_));
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.setInputCloud(global_map);
        voxel.filter(*downsampled);

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*downsampled, msg);
        msg.header.stamp = now();
        msg.header.frame_id = "camera_init";
        pub_optimized_map_->publish(msg);
        last_published_map_keyframes_ = keyframes_.size();
        last_published_loop_count_ = loop_index_container_.size();

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "optimized map published: keyframes=%zu, points=%zu, loops=%zu",
            keyframes_.size(), downsampled->size(), loop_index_container_.size());
    }

private:
    // 互斥锁保护点云/退化消息队列，避免 ROS 回调并发读写。
    std::mutex mtx_;
    // 按时间戳保存最近若干点云，odomHandler() 从中选择最近消息。
    std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> cloud_queue_;
    // 按时间戳保存最近若干退化状态，避免使用过期的 latest 状态。
    std::deque<DegeneracyInfo> degeneracy_queue_;
    // 最近一次已匹配点云的时间戳，保证点云单调且不重复使用。
    double last_matched_cloud_time_ = -std::numeric_limits<double>::infinity();

    // 三个输入订阅器：位姿、点云、前端退化先验。
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_degeneracy_;
    // 后端优化轨迹发布器。
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_optimized_path_;
    // 后端优化地图发布器。
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_optimized_map_;

    // GTSAM 增量优化器与待加入的新因子/新初值。
    std::unique_ptr<gtsam::ISAM2> isam_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values values_;
    // 后端维护的所有关键帧。
    std::vector<KeyFrame> keyframes_;
    // 全部前端 odometry，用于输出全频后端校正轨迹，不参与 pose graph 建图。
    std::vector<OdomSample> odom_samples_;
    // 已经接受的回环约束，用于避免同一 current_id 重复加回环。
    std::unordered_map<int, int> loop_index_container_;
    // 已接受回环对，用于执行 current/candidate/pair 三种冷却规则。
    std::vector<std::pair<int, int>> accepted_loop_pairs_;

    // 以下参数均可通过 ROS2 parameter 覆盖。
    double keyframe_distance_ = 1.0;
    double keyframe_yaw_ = 0.25;
    double cloud_time_tolerance_ = 0.20;
    size_t sync_queue_size_ = 200;
    int recent_exclusion_num_ = 30;
    double loop_min_path_separation_ = 0.0;
    int loop_candidate_top_k_ = 1;
    int loop_current_cooldown_keyframes_ = 10;
    int loop_candidate_cooldown_keyframes_ = 20;
    int loop_pair_cooldown_keyframes_ = 30;
    double scan_context_distance_threshold_ = 0.18;
    int scan_context_ringkey_candidate_num_ = 10;
    double icp_fitness_threshold_normal_ = 0.30;
    double icp_fitness_threshold_degenerate_ = 0.10;
    bool loop_bidirectional_icp_enable_ = false;
    double loop_bidirectional_icp_fitness_threshold_ = 0.30;
    double loop_reciprocal_translation_threshold_ = 0.30;
    double loop_reciprocal_yaw_threshold_ = 0.10;
    bool loop_observability_enable_ = false;
    bool loop_observability_reject_enable_ = false;
    int loop_observability_min_correspondences_ = 100;
    double loop_observability_max_condition_number_ = 1e5;
    double loop_observability_min_singular_value_ = 1e-4;
    double icp_max_correspondence_distance_ = 2.0;
    double loop_consistency_trans_normal_ = 2.0;
    double loop_consistency_trans_degenerate_ = 1.0;
    double loop_consistency_yaw_normal_ = 0.50;
    double loop_consistency_yaw_degenerate_ = 0.25;
    double loop_noise_sigma_normal_ = 0.30;
    double loop_noise_sigma_degenerate_ = 0.50;
    int loop_submap_search_num_ = 10;
    double loop_submap_leaf_size_ = 0.5;
    double keyframe_leaf_size_ = 0.4;
    bool publish_optimized_map_ = false;
    int optimized_map_publish_interval_ = 10;
    double optimized_map_leaf_size_ = 0.5;
    int scan_context_rings_ = 20;
    int scan_context_sectors_ = 60;
    double scan_context_max_radius_ = 80.0;
    int scan_context_alignment_search_radius_ = 2;
    double last_keyframe_yaw_ = 0.0;
    bool backend_log_enable_ = true;
    std::string backend_tum_path_;
    std::string backend_full_tum_path_;
    std::string backend_loop_csv_path_;
    std::ofstream loop_csv_;
    size_t last_published_map_keyframes_ = 0;
    size_t last_published_loop_count_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdaptiveDegenerateBackend>());
    rclcpp::shutdown();
    return 0;
}
