#pragma once
#include <Eigen/Geometry>

namespace af {

inline Eigen::Matrix3d rpy_to_matrix(double roll, double pitch, double yaw)
{
    return (Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX())).toRotationMatrix();
}

} // namespace af
