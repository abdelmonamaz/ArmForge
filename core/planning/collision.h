#pragma once
#include "obstacle.h"
#include <vector>

namespace af {

// True if every sample point clears every obstacle by at least `margin`.
// Sample points are typically the robot's frame origins for a given
// configuration (a coarse "stick figure" approximation of the arm).
inline bool isClearOfObstacles(const std::vector<Eigen::Vector3d>& points,
                               const std::vector<SphereObstacle>& obstacles,
                               double margin = 0.0)
{
    for (const auto& p : points) {
        for (const auto& obs : obstacles) {
            if ((p - obs.center).norm() < obs.radius + margin) {
                return false;
            }
        }
    }
    return true;
}

} // namespace af
