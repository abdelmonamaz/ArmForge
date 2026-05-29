#pragma once
#include <numbers>
#include <string>

namespace af {

enum class JointType { Revolute, Prismatic };

struct JointLimits {
    double min     = -std::numbers::pi;
    double max     =  std::numbers::pi;
    double vel_max =  2.0;    // rad/s  (or m/s for prismatic)
    double acc_max =  5.0;    // rad/s²
};

struct Joint {
    std::string name;
    JointType   type     = JointType::Revolute;
    JointLimits limits;
    double      position = 0.0;  // current q (rad or m)
};

} // namespace af
