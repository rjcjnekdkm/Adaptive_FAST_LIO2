#pragma once
// Minimal internal types for the FAST-LIO2 IMU port (source: FAST_LIO/include/common_lib.h).
// Pose6D is private working storage, not a dependency on fast_lio's ROS messages.
#include <array>
#include "adaptive_fast_lio2/adaptive_common.hpp"
using namespace std;
using namespace Eigen;
using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;
#define MD(a,b) Eigen::Matrix<double, (a), (b)>
#define G_m_s2 (9.81)
#define VEC_FROM_ARRAY(v) v[0],v[1],v[2]
#define MAT_FROM_ARRAY(v) v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
inline const M3D Eye3d(M3D::Identity());
inline const V3D Zero3d(0, 0, 0);
struct Pose6D {
    double offset_time = 0.0;
    std::array<double, 3> acc{}, gyr{}, vel{}, pos{};
    std::array<double, 9> rot{};
};
template<typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &g, \
                const Matrix<T, 3, 1> &v, const Matrix<T, 3, 1> &p, const Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++)  rot_kp.rot[i*3+j] = R(i,j);
    }
    return move(rot_kp);
}
