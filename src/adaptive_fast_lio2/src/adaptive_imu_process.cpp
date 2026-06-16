#include "adaptive_fast_lio2/adaptive_imu_process.hpp"
#include "use-ikfom.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

AdaptiveImuProcess::AdaptiveImuProcess()
{
    reset();
}


void AdaptiveImuProcess::reset()
{
    state_ = AdaptiveState();

    imu_need_init_ = true;
    init_count_ = 0;

    mean_gyr_.setZero();
    mean_acc_.setZero();
    acc_scale_ = 1.0;

    cov_acc_ = Eigen::Vector3d(0.1, 0.1, 0.1);
    cov_gyr_ = Eigen::Vector3d(0.1, 0.1, 0.1);
    cov_bias_gyr_ = Eigen::Vector3d(0.0001, 0.0001, 0.0001);
    cov_bias_acc_ = Eigen::Vector3d(0.0001, 0.0001, 0.0001);

    last_imu_.reset();
    last_lidar_end_time_ = -1.0;
    imu_pose_seq_.clear();
}



bool AdaptiveImuProcess::isInitialized() const
{
    return !imu_need_init_ && state_.inited;
}

void AdaptiveImuProcess::setExtrinsic(const Eigen::Matrix3d &rot_lidar_to_imu, const Eigen::Vector3d &trans_lidar_to_imu)
{
    state_.offset_R_L_I = rot_lidar_to_imu;
    state_.offset_T_L_I = trans_lidar_to_imu;
}

void AdaptiveImuProcess::setNoiseCovariances(
    double gyr_cov,
    double acc_cov,
    double gyr_bias_cov,
    double acc_bias_cov)
{
    cov_gyr_.setConstant(gyr_cov);
    cov_acc_.setConstant(acc_cov);
    cov_bias_gyr_.setConstant(gyr_bias_cov);
    cov_bias_acc_.setConstant(acc_bias_cov);
}

void AdaptiveImuProcess::setInitializationSampleCount(int sample_count)
{
    // 至少使用一条 IMU，避免无效配置导致初始化永远无法完成。
    init_imu_num_ = std::max(1, sample_count);
}


void AdaptiveImuProcess::syncAdaptiveStateToIkfom(
    esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
    bool reset_covariance)
{
    state_ikfom ikfom_state = kf_state.get_x();

    ikfom_state.pos = state_.pos;
    ikfom_state.rot = SO3(state_.rot);
    ikfom_state.vel = state_.vel;
    ikfom_state.bg = state_.bg;
    ikfom_state.ba = state_.ba;
    ikfom_state.grav = S2(state_.grav);
    ikfom_state.offset_R_L_I = SO3(state_.offset_R_L_I);
    ikfom_state.offset_T_L_I = state_.offset_T_L_I;

    kf_state.change_x(ikfom_state);

    if (reset_covariance)
    {
        esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
        init_P.setIdentity();
        init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
        init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
        init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
        init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
        init_P(21, 21) = init_P(22, 22) = 0.00001;
        kf_state.change_P(init_P);
    }
}


void AdaptiveImuProcess::syncAdaptiveStateFromIkfom(const state_ikfom &ikfom_state)
{
    state_.rot = ikfom_state.rot.toRotationMatrix();
    state_.pos = Eigen::Vector3d(
        ikfom_state.pos[0],
        ikfom_state.pos[1],
        ikfom_state.pos[2]);
    state_.vel = Eigen::Vector3d(
        ikfom_state.vel[0],
        ikfom_state.vel[1],
        ikfom_state.vel[2]);
    state_.bg = Eigen::Vector3d(
        ikfom_state.bg[0],
        ikfom_state.bg[1],
        ikfom_state.bg[2]);
    state_.ba = Eigen::Vector3d(
        ikfom_state.ba[0],
        ikfom_state.ba[1],
        ikfom_state.ba[2]);
    state_.grav = Eigen::Vector3d(
        ikfom_state.grav[0],
        ikfom_state.grav[1],
        ikfom_state.grav[2]);
    state_.offset_R_L_I = ikfom_state.offset_R_L_I.toRotationMatrix();
    state_.offset_T_L_I = Eigen::Vector3d(
        ikfom_state.offset_T_L_I[0],
        ikfom_state.offset_T_L_I[1],
        ikfom_state.offset_T_L_I[2]);
    state_.inited = true;
}



double AdaptiveImuProcess::stampToSec(
    const builtin_interfaces::msg::Time &stamp) const
{
    return static_cast<double>(stamp.sec) +
           static_cast<double>(stamp.nanosec) * 1e-9;
}


Eigen::Matrix3d AdaptiveImuProcess::skewSymmetric(
    const Eigen::Vector3d &v) const
{
    Eigen::Matrix3d m;

    m << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;

    return m;
}


Eigen::Matrix3d AdaptiveImuProcess::so3Exp(
    const Eigen::Vector3d &w) const
{
    const double theta = w.norm();

    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    if (theta < 1e-8)
    {
        return I + skewSymmetric(w);
    }

    const Eigen::Matrix3d K = skewSymmetric(w / theta);

    return I +
           std::sin(theta) * K +
           (1.0 - std::cos(theta)) * K * K;
}


/**
 * @brief IMU 初始化
 *
 * FAST-LIO2 会使用前面一段静止或近似静止 IMU 数据初始化：
 *   1. 平均角速度 -> 陀螺仪零偏 bg；
 *   2. 平均加速度 -> 重力方向和加速度计零偏相关初值。
 *
 * 当前版本为了先保持流程正确：
 *   1. 估计 bg；
 *   2. gravity 暂时使用默认 (0, 0, -9.81)；
 *   3. ba 暂时保持 0。
 *
 * 后续阶段我们再把重力初始化补得更接近 FAST-LIO2。
 */
void AdaptiveImuProcess::imuInit(const MeasureGroup &meas)
{
    if (meas.imu.empty())
    {
        return;
    }

    for (size_t i = 0; i < meas.imu.size(); ++i)
    {
        const auto &imu = meas.imu[i];

        Eigen::Vector3d gyr(
            imu->angular_velocity.x,
            imu->angular_velocity.y,
            imu->angular_velocity.z);

        Eigen::Vector3d acc(
            imu->linear_acceleration.x,
            imu->linear_acceleration.y,
            imu->linear_acceleration.z);

        init_count_++;

        // 在线均值更新：
        // mean_new = mean_old + (x - mean_old) / n
        mean_gyr_ += (gyr - mean_gyr_) / static_cast<double>(init_count_);
        mean_acc_ += (acc - mean_acc_) / static_cast<double>(init_count_);
    }

    if (init_count_ < init_imu_num_)
    {
        // 每累计约 50 条 IMU 输出一次进度，避免初始化阶段逐帧刷屏。
        if (init_count_ % 50 < static_cast<int>(meas.imu.size()))
        {
            std::cout << "[IMU Init] collecting: "
                      << init_count_ << " / " << init_imu_num_
                      << std::endl;
        }
        return;
    }

    // 陀螺仪零偏初值
    state_.bg = mean_gyr_;

    // 当前阶段暂时不估计 ba
    state_.ba = Eigen::Vector3d::Zero();

    const double acc_norm = mean_acc_.norm();
    if (acc_norm > 1e-3)
    {
        state_.grav = -mean_acc_ / acc_norm * 9.81;
        acc_scale_ = 9.81 / acc_norm;
    }
    else
    {
        state_.grav = Eigen::Vector3d(0.0, 0.0, -9.81);
        acc_scale_ = 1.0;
    }

    // 初始化传播时间。
    // 从当前这组测量的最后一个 IMU 开始，下一帧再正式传播。
    last_imu_ = meas.imu.back();
    state_.last_imu_time = stampToSec(last_imu_->header.stamp);
    state_.inited = true;

    imu_need_init_ = false;

    std::cout << "[IMU Init] finished. "
              << "bg=" << state_.bg.transpose()
              << ", mean_acc=" << mean_acc_.transpose()
              << ", mean_acc_norm=" << mean_acc_.norm()
              << ", grav=" << state_.grav.transpose()
              << ", acc_scale=" << acc_scale_
              << std::endl;
}


/**
 * @brief 将当前状态保存到 imu_pose_seq_
 *
 * imu_pose_seq_ 用于后面对点云去畸变。
 * 每个点根据自己的点时间，查找对应的 IMU pose。
 */
void AdaptiveImuProcess::pushCurrentPose(double timestamp, double lidar_beg_time, const Eigen::Vector3d &gyro, const Eigen::Vector3d &acc)
{
    ImuPose pose;

    pose.timestamp = timestamp;
    pose.offset_time = timestamp - lidar_beg_time;

    pose.rot = state_.rot;
    pose.pos = state_.pos;
    pose.vel = state_.vel;

    pose.gyro = gyro;
    pose.acc = acc;

    imu_pose_seq_.push_back(pose);
}


/**
 * @brief 状态传播一步
 *
 * 输入 gyro/acc 已经是当前时间段的平均测量。
 */
void AdaptiveImuProcess::propagateState(const Eigen::Vector3d &gyro_raw, const Eigen::Vector3d &acc_raw, double dt, double target_time)
{
    if (dt <= 0.0 || dt > 0.2)
    {
        state_.last_imu_time = target_time;
        return;
    }

    // 去零偏
    const Eigen::Vector3d gyro = gyro_raw - state_.bg;
    const Eigen::Vector3d acc = acc_raw * acc_scale_ - state_.ba;

    // 姿态传播：
    // R_{k+1} = R_k * Exp((gyro - bg) * dt)
    const Eigen::Matrix3d dR = so3Exp(gyro * dt);
    state_.rot = state_.rot * dR;

    // 加速度转到 world 系：
    // a_world = R * (acc - ba) + gravity
    const Eigen::Vector3d acc_world =
        state_.rot * acc + state_.grav;

    // 位置、速度传播
    state_.pos =
        state_.pos +
        state_.vel * dt +
        0.5 * acc_world * dt * dt;

    state_.vel =
        state_.vel +
        acc_world * dt;

    state_.last_imu_time = target_time;
}


/**
 * @brief 前向传播 IMU
 *
 * FAST-LIO2 中会把上一帧最后一个 IMU 与当前帧 IMU 拼接起来，
 * 这样相邻 LiDAR 帧之间的 IMU 积分是连续的。
 *
 * 当前实现同样保留 last_imu_。
 */
void AdaptiveImuProcess::forwardPropagate(const MeasureGroup &meas)
{
    imu_pose_seq_.clear();

    std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu_vec;

    if (last_imu_ != nullptr)
    {
        imu_vec.push_back(last_imu_);
    }

    for (size_t i = 0; i < meas.imu.size(); ++i)
    {
        imu_vec.push_back(meas.imu[i]);
    }

    if (imu_vec.size() < 2)
    {
        if (!meas.imu.empty())
        {
            last_imu_ = meas.imu.back();
        }
        return;
    }

    // 保存传播起点状态。
    // 如果点时间早于第一个传播 pose，就会使用这个起点 pose。
    pushCurrentPose(
        state_.last_imu_time,
        meas.lidar_beg_time,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    for (size_t i = 0; i + 1 < imu_vec.size(); ++i)
    {
        const auto &head = imu_vec[i];
        const auto &tail = imu_vec[i + 1];

        const double head_time = stampToSec(head->header.stamp);
        const double tail_time = stampToSec(tail->header.stamp);

        if (tail_time <= state_.last_imu_time)
        {
            continue;
        }

        // 使用前后两帧 IMU 的平均作为当前小时间段输入。
        // 这和 FAST-LIO2 中使用相邻 IMU 平均值的思想一致。
        Eigen::Vector3d gyro_avg(
            0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
            0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
            0.5 * (head->angular_velocity.z + tail->angular_velocity.z));

        Eigen::Vector3d acc_avg(
            0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
            0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
            0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z));

        // 如果 state_.last_imu_time 已经落在 head 和 tail 之间，
        // dt 应该只积分剩余部分。
        const double dt = tail_time - state_.last_imu_time;

        propagateState(
            gyro_avg,
            acc_avg,
            dt,
            tail_time);

        pushCurrentPose(
            tail_time,
            meas.lidar_beg_time,
            gyro_avg - state_.bg,
            acc_avg - state_.ba);

        if (tail_time >= meas.lidar_end_time)
        {
            break;
        }
    }

    // 如果 IMU 最后一条时间仍早于 LiDAR 结束时间，
    // 理论上 sync_packages() 不应该让这种情况发生。
    // 这里保留一个保护逻辑。
    if (state_.last_imu_time < meas.lidar_end_time &&
        !imu_vec.empty())
    {
        const auto &imu = imu_vec.back();

        Eigen::Vector3d gyro(
            imu->angular_velocity.x,
            imu->angular_velocity.y,
            imu->angular_velocity.z);

        Eigen::Vector3d acc(
            imu->linear_acceleration.x,
            imu->linear_acceleration.y,
            imu->linear_acceleration.z);

        const double dt = meas.lidar_end_time - state_.last_imu_time;

        propagateState(
            gyro,
            acc,
            dt,
            meas.lidar_end_time);

        pushCurrentPose(
            meas.lidar_end_time,
            meas.lidar_beg_time,
            gyro - state_.bg,
            acc - state_.ba);
    }

    if (!meas.imu.empty())
    {
        last_imu_ = meas.imu.back();
    }
}



void AdaptiveImuProcess::forwardPropagateIkfom(
    const MeasureGroup &meas,
    esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state)
{
    imu_pose_seq_.clear();

    std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu_vec;
    if (last_imu_ != nullptr)
    {
        imu_vec.push_back(last_imu_);
    }

    for (size_t i = 0; i < meas.imu.size(); ++i)
    {
        imu_vec.push_back(meas.imu[i]);
    }

    syncAdaptiveStateFromIkfom(kf_state.get_x());

    const double propagation_start_time =
        state_.last_imu_time > 0.0 ? state_.last_imu_time : meas.lidar_beg_time;

    pushCurrentPose(
        propagation_start_time,
        meas.lidar_beg_time,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (imu_vec.empty())
    {
        return;
    }

    auto Q = process_noise_cov();
    input_ikfom in;
    bool has_input = false;
    double last_predict_time = propagation_start_time;

    for (size_t i = 0; i + 1 < imu_vec.size(); ++i)
    {
        const auto &head = imu_vec[i];
        const auto &tail = imu_vec[i + 1];

        const double head_time = stampToSec(head->header.stamp);
        const double tail_time = stampToSec(tail->header.stamp);

        if (tail_time <= last_predict_time)
        {
            continue;
        }

        Eigen::Vector3d gyro_avg(
            0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
            0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
            0.5 * (head->angular_velocity.z + tail->angular_velocity.z));

        Eigen::Vector3d acc_avg(
            0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
            0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
            0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z));

        acc_avg *= acc_scale_;

        double dt = 0.0;
        if (head_time < last_predict_time)
        {
            dt = tail_time - last_predict_time;
        }
        else
        {
            dt = tail_time - head_time;
        }

        if (dt <= 0.0 || std::abs(dt) > 0.2)
        {
            last_predict_time = tail_time;
            continue;
        }

        in.acc = acc_avg;
        in.gyro = gyro_avg;
        has_input = true;

        Q.block<3, 3>(0, 0).diagonal() = cov_gyr_;
        Q.block<3, 3>(3, 3).diagonal() = cov_acc_;
        Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
        Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;

        kf_state.predict(dt, Q, in);
        last_predict_time = tail_time;

        syncAdaptiveStateFromIkfom(kf_state.get_x());

        const Eigen::Vector3d gyro_unbias = gyro_avg - state_.bg;
        const Eigen::Vector3d acc_world =
            state_.rot * (acc_avg - state_.ba) + state_.grav;

        pushCurrentPose(
            tail_time,
            meas.lidar_beg_time,
            gyro_unbias,
            acc_world);
    }


    if (has_input)
    {
        double dt_end = meas.lidar_end_time - last_predict_time;
        if (std::abs(dt_end) > 1e-9 && std::abs(dt_end) < 0.2)
        {
            kf_state.predict(dt_end, Q, in);
            last_predict_time = meas.lidar_end_time;
            syncAdaptiveStateFromIkfom(kf_state.get_x());
        }
    }

    state_.last_imu_time = meas.lidar_end_time;
    last_lidar_end_time_ = meas.lidar_end_time;

    if (!meas.imu.empty())
    {
        last_imu_ = meas.imu.back();
    }

}






/**
 * @brief 点云去畸变
 *
 * FAST-LIO2 的去畸变思想：
 * 1. 每个点有自己的采样时间；
 * 2. LiDAR 扫描一帧期间，IMU/body 在运动；
 * 3. 要把所有点补偿到同一个参考时刻，通常是 LiDAR 帧结束时刻；
 * 4. 后续 scan-to-map 使用这个去畸变后的点云。
 *
 * 当前版本假设 LiDAR 外参为单位：
 *   LiDAR frame == body frame
 *
 * 后续接入外参后，需要加入 offset_R_L_I / offset_T_L_I。
 */
void AdaptiveImuProcess::undistortPcl(const MeasureGroup &meas, PointCloudXYZI::Ptr &pcl_undistort) const
{
    if (pcl_undistort == nullptr)
    {
        pcl_undistort.reset(new PointCloudXYZI());
    }

    pcl_undistort->clear();

    if (meas.lidar == nullptr || meas.lidar->empty())
    {
        return;
    }

    *pcl_undistort = *meas.lidar;
    std::sort(
        pcl_undistort->points.begin(),
        pcl_undistort->points.end(),
        [](const PointType &a, const PointType &b)
        {
            return a.curvature < b.curvature;
        });

    if (imu_pose_seq_.size() < 2)
    {
        return;
    }

    // 判断点云是否有有效每点时间。
    // Livox 分支 curvature 为 ms；
    // 标准 PointCloud2 当前 curvature 为 0，所以不做去畸变。
    double max_rel_time = 0.0;

    for (size_t i = 0; i < pcl_undistort->size(); ++i)
    {
        max_rel_time = std::max(max_rel_time,static_cast<double>(pcl_undistort->points[i].curvature));
    }

    if (max_rel_time <= 0.0)
    {
        return;
    }

    // ===================== LiDAR-IMU 外参 =====================
    //   offset_R_L_I：LiDAR 坐标系到 IMU/body 坐标系的旋转
    //   offset_T_L_I：LiDAR 坐标系到 IMU/body 坐标系的平移
    // 含义是：
    //   p_imu = R_L_I * p_lidar + T_L_I
    // 也就是说，原始点云中的点首先是在 LiDAR 坐标系下，
    // 去畸变时不能直接把它当作 IMU/body 坐标系下的点。
    const Eigen::Matrix3d R_L_I = state_.offset_R_L_I;
    const Eigen::Vector3d T_L_I = state_.offset_T_L_I;

    // ===================== LiDAR 帧结束时刻的状态 =====================
    // forwardPropagate() 执行后，state_ 已经传播到当前 LiDAR 帧结束时刻附近。
    // R_end：LiDAR 帧结束时刻，IMU/body 到 world 的旋转
    // P_end：LiDAR 帧结束时刻，IMU/body 在 world 中的位置
    // 去畸变的目标是：
    //   把一帧 LiDAR 中不同采样时刻的点，统一补偿到帧结束时刻。
    const Eigen::Matrix3d R_end = state_.rot;
    const Eigen::Vector3d P_end = state_.pos;


    // 与官方 FAST-LIO2 一致，从最后一个点向前逐段反向补偿。
    auto point_it = pcl_undistort->points.end() - 1;
    for (auto pose_it = imu_pose_seq_.end() - 1;
         pose_it != imu_pose_seq_.begin();
         --pose_it)
    {
        const ImuPose &head = *(pose_it - 1);
        const ImuPose &tail = *pose_it;

        while (true)
        {
            const double point_offset =
                static_cast<double>(point_it->curvature) / 1000.0;
            if (point_offset <= head.offset_time)
            {
                break;
            }

            const double dt = point_offset - head.offset_time;
            const Eigen::Matrix3d R_i =
                head.rot * so3Exp(tail.gyro * dt);
            const Eigen::Vector3d T_ei =
                head.pos +
                head.vel * dt +
                0.5 * tail.acc * dt * dt -
                P_end;

            const Eigen::Vector3d p_lidar(
                point_it->x,
                point_it->y,
                point_it->z);

            const Eigen::Vector3d p_compensated =
                R_L_I.transpose() *
                (R_end.transpose() *
                    (R_i * (R_L_I * p_lidar + T_L_I) + T_ei) -
                 T_L_I);

            point_it->x = static_cast<float>(p_compensated.x());
            point_it->y = static_cast<float>(p_compensated.y());
            point_it->z = static_cast<float>(p_compensated.z());

            if (point_it == pcl_undistort->points.begin())
            {
                return;
            }
            --point_it;
        }
    }
}


void AdaptiveImuProcess::Process(
    const MeasureGroup &meas,
    PointCloudXYZI::Ptr &pcl_undistort)
{
    if (meas.lidar == nullptr || meas.lidar->empty())
    {
        if (pcl_undistort != nullptr)
        {
            pcl_undistort->clear();
        }
        return;
    }

    // 1. IMU 初始化阶段
    //
    // 初始化完成前，不做正式状态传播。
    // 先直接输出原始点云，保证主流程不中断。
    if (imu_need_init_)
    {
        imuInit(meas);

        if (pcl_undistort == nullptr)
        {
            pcl_undistort.reset(new PointCloudXYZI());
        }

        *pcl_undistort = *meas.lidar;

        return;
    }

    // 2. IMU 前向传播
    forwardPropagate(meas);

    // 3. 点云去畸变
    undistortPcl(meas, pcl_undistort);
}


void AdaptiveImuProcess::Process(
    const MeasureGroup &meas,
    esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
    PointCloudXYZI::Ptr &pcl_undistort)
{
    if (meas.lidar == nullptr || meas.lidar->empty())
    {
        if (pcl_undistort != nullptr)
        {
            pcl_undistort->clear();
        }
        return;
    }

    if (meas.imu.empty())
    {
        if (pcl_undistort == nullptr)
        {
            pcl_undistort.reset(new PointCloudXYZI());
        }
        *pcl_undistort = *meas.lidar;
        return;
    }

    if (imu_need_init_)
    {
        imuInit(meas);

        if (pcl_undistort == nullptr)
        {
            pcl_undistort.reset(new PointCloudXYZI());
        }
        *pcl_undistort = *meas.lidar;

        if (!imu_need_init_)
        {
            syncAdaptiveStateToIkfom(kf_state, true);
        }

        return;
    }

    forwardPropagateIkfom(meas, kf_state);
    undistortPcl(meas, pcl_undistort);
}
