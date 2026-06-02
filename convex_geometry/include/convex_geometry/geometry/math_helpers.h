#ifndef CONVEX_GEOMETRY_GEOMETRY_MATH_HELPERS_H
#define CONVEX_GEOMETRY_GEOMETRY_MATH_HELPERS_H

#include <Eigen/Dense>

#include <algorithm>

namespace convex_geometry {
namespace math_helpers {

constexpr double kDirectionEpsilon = 1e-12;

inline double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(value, max_value));
}

inline Eigen::Vector3d normalizedOrZero(const Eigen::Vector3d& value,
                                        double direction_epsilon = kDirectionEpsilon) {
    if (!value.array().isFinite().all() || value.squaredNorm() <= direction_epsilon) {
        return Eigen::Vector3d::Zero();
    }
    return value.normalized();
}

inline bool isFiniteVector3(const Eigen::Vector3d& value) {
    return value.array().isFinite().all();
}

}  // namespace math_helpers
}  // namespace convex_geometry

#endif  // CONVEX_GEOMETRY_GEOMETRY_MATH_HELPERS_H
