#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sensor_msgs/msg/imu.hpp>

#include "adaptive_fast_lio2/adaptive_common.hpp"
#include "adaptive_fast_lio2/adaptive_state.hpp"
#include "use-ikfom.hpp"

/**
 * @brief IMU 前向传播过程中保存的一帧状态
 *
 * FAST-LIO2 在点云去畸变时，不是只用最终位姿，
 * 而是保存一段 IMU 传播过程中的位姿序列。
 *
 * 每个 LiDAR 点有自己的相对时间 curvature，
 * 去畸变时需要找到这个点对应时刻附近的 IMU 位姿，
 * 再把该点补偿到 LiDAR 帧结束时刻。
 */
struct ImuPose
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 该 IMU 状态对应的绝对时间，单位：秒
    double timestamp = 0.0;

    // 相对于当前 LiDAR 帧起始时刻的时间，单位：秒
    double offset_time = 0.0;

    // 当前时刻 body/IMU 在 world 坐标系下的姿态
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();

    // 当前时刻 body/IMU 在 world 坐标系下的位置
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();

    // 当前时刻 body/IMU 在 world 坐标系下的速度
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();

    // 去零偏后的角速度
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();

    // 去零偏后的加速度
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
};


/**
 * @brief FAST-LIO2 风格 IMU 处理模块
 *
 * 对应 FAST-LIO2 的 ImuProcess。
 *
 * 当前阶段实现：
 * 1. IMU 初始化框架；
 * 2. 陀螺仪零偏初值估计；
 * 3. IMU 前向传播；
 * 4. 保存传播过程中的位姿序列；
 * 5. 根据点云 curvature 做一版基础去畸变。
 *
 * 后续阶段再继续补：
 * 1. 更严格的重力初始化；
 * 2. 加速度计零偏估计；
 * 3. 与 ESKF / IKFoM 状态结合；
 * 4. 更严格的反向去畸变。
 */
class AdaptiveImuProcess
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AdaptiveImuProcess();

    /**
     * @brief 处理一组 LiDAR + IMU 测量
     *
     * 输入：
     *   meas:
     *     sync_packages() 同步出的一帧 LiDAR 和对应 IMU
     *
     * 输出：
     *   pcl_undistort:
     *     去畸变后的点云，对应 FAST-LIO2 中 feats_undistort
     */
    void Process(
        const MeasureGroup &meas,
        PointCloudXYZI::Ptr &pcl_undistort);


    void Process(
        const MeasureGroup &meas,
        esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
        PointCloudXYZI::Ptr &pcl_undistort);

    bool isInitialized() const;

    void reset();

    /**
    * @brief 设置 LiDAR 到 IMU/body 的外参
    *   offset_R_L_I: LiDAR 坐标系到 IMU 坐标系的旋转
    *   offset_T_L_I: LiDAR 坐标系到 IMU 坐标系的平移
    *
     * 如果外参为单位阵和零平移，表示默认 LiDAR 坐标系和 IMU/body 坐标系重合。
    */
    void setExtrinsic(
        const Eigen::Matrix3d &rot_lidar_to_imu,
        const Eigen::Vector3d &trans_lidar_to_imu);

    void setNoiseCovariances(
        double gyr_cov,
        double acc_cov,
        double gyr_bias_cov,
        double acc_bias_cov);

    /**
     * @brief 设置 IMU 初始化阶段需要累计的样本数量
     *
     * 初始化窗口过长时，车辆启动加速度可能被误认为重力分量，进而产生
     * 初始俯仰误差和长期 Z 轴漂移。该参数允许针对数据集采样率调整窗口。
     */
    void setInitializationSampleCount(int sample_count);

private:
    /**
     * @brief IMU 初始化
     *
     * 这一步对应 FAST-LIO2 中 IMU_init() 的初始框架。
     *
     * 当前版本主要估计：
     *   gyro mean -> bg 初值
     *
     * 后续会继续扩展：
     *   acc mean -> gravity / ba 初始化
     */
    void imuInit(const MeasureGroup &meas);

    /**
     * @brief 前向传播 IMU 状态
     *
     * 根据当前 LiDAR 帧内的 IMU 序列，将状态传播到 LiDAR 帧结束时刻。
     */
    void forwardPropagate(const MeasureGroup &meas);


    void forwardPropagateIkfom(
        const MeasureGroup &meas,
        esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state);
        

    /**
     * @brief 使用平均角速度和平均加速度传播一步
     */
    void propagateState(
        const Eigen::Vector3d &gyro,
        const Eigen::Vector3d &acc,
        double dt,
        double target_time);

    /**
     * @brief 保存当前状态到 imu_pose_seq_
     */
    void pushCurrentPose(
        double timestamp,
        double lidar_beg_time,
        const Eigen::Vector3d &gyro,
        const Eigen::Vector3d &acc);

    /**
     * @brief 根据点云中每个点的相对时间进行去畸变
     *
     * 当前版本使用最近的 IMU 位姿做补偿。
     * 后续可以升级为插值位姿。
     */
    void undistortPcl(
        const MeasureGroup &meas,
        PointCloudXYZI::Ptr &pcl_undistort) const;

    void syncAdaptiveStateToIkfom(
        esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
        bool reset_covariance);

    void syncAdaptiveStateFromIkfom(const state_ikfom &ikfom_state);


    Eigen::Matrix3d so3Exp(const Eigen::Vector3d &w) const;
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &v) const;

    double stampToSec(const builtin_interfaces::msg::Time &stamp) const;

private:
    AdaptiveState state_;

    // 是否还处在 IMU 初始化阶段
    bool imu_need_init_ = true;

    // 初始化需要累计的 IMU 数量，可通过 mapping.imu_init_num 配置。
    int init_imu_num_ = 200;

    // 当前已经累计的 IMU 数量
    int init_count_ = 0;

    // 初始化阶段累计平均角速度
    Eigen::Vector3d mean_gyr_ = Eigen::Vector3d::Zero();

    // 初始化阶段累计平均加速度
    Eigen::Vector3d mean_acc_ = Eigen::Vector3d::Zero();

    // 将原始加速度计模长缩放到标准重力模长的比例系数。
    double acc_scale_ = 1.0;

    // IMU 过程噪声：加速度、角速度、陀螺仪零偏随机游走、加速度计零偏随机游走。
    Eigen::Vector3d cov_acc_ = Eigen::Vector3d(0.1, 0.1, 0.1);
    Eigen::Vector3d cov_gyr_ = Eigen::Vector3d(0.1, 0.1, 0.1);
    Eigen::Vector3d cov_bias_gyr_ = Eigen::Vector3d(0.0001, 0.0001, 0.0001);
    Eigen::Vector3d cov_bias_acc_ = Eigen::Vector3d(0.0001, 0.0001, 0.0001);

    // 上一帧最后一个 IMU，用于跨 LiDAR 帧连续传播
    sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;

    // 上一帧 LiDAR 的结束时间，单位：秒，用于维持跨帧传播连续性。
    double last_lidar_end_time_ = -1.0;

    // 当前 LiDAR 帧内的 IMU 位姿序列，用于点云去畸变
    std::vector<ImuPose, Eigen::aligned_allocator<ImuPose>> imu_pose_seq_;
};
