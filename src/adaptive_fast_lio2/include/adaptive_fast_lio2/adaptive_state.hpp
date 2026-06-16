#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

/**
 * @brief FAST-LIO2 风格的状态结构雏形
 *
 * FAST-LIO2 原始状态不是简单的 pos/vel/rot，
 * 它还包括：
 *   1. IMU 陀螺仪零偏 bg
 *   2. IMU 加速度计零偏 ba
 *   3. 重力向量 grav
 *   4. LiDAR 到 IMU 的外参 offset_R_L_I / offset_T_L_I
 *
 * 当前阶段先建立这个状态结构。
 * 后续接入 IKFoM / ESKF 时，会继续扩展为真正的 state_ikfom。
 */
struct AdaptiveState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ===================== 位姿状态 =====================

    // body/IMU 在 world 坐标系下的旋转
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();

    // body/IMU 在 world 坐标系下的位置
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();

    // body/IMU 在 world 坐标系下的速度
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();


    // ===================== IMU 零偏 =====================

    // 陀螺仪零偏，后续 IMU 初始化和 ESKF 会估计它
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();

    // 加速度计零偏，后续 IMU 初始化和 ESKF 会估计它
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();


    // ===================== 重力 =====================

    // 当前先使用默认重力方向。
    // 后续严格实现 FAST-LIO2 时，需要在 IMU 初始化阶段估计重力方向。
    Eigen::Vector3d grav = Eigen::Vector3d(0.0, 0.0, -9.81);


    // ===================== LiDAR-IMU 外参 =====================

    // LiDAR 到 IMU 的旋转外参
    Eigen::Matrix3d offset_R_L_I = Eigen::Matrix3d::Identity();

    // LiDAR 到 IMU 的平移外参
    Eigen::Vector3d offset_T_L_I = Eigen::Vector3d::Zero();


    // ===================== 时间状态 =====================

    // 上一条已经用于传播的 IMU 时间
    double last_imu_time = -1.0;

    // 是否已经收到第一条 IMU
    bool inited = false;
};