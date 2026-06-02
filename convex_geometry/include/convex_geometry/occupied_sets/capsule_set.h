#ifndef CONVEX_GEOMETRY_OCCUPIED_SETS_CAPSULE_SET_H
#define CONVEX_GEOMETRY_OCCUPIED_SETS_CAPSULE_SET_H

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include "convex_geometry/occupied_sets/convex_set_base.h"

namespace convex_geometry {

class CapsuleSet final : public ConvexSet3D {
public:
    CapsuleSet() = default;

    CapsuleSet(const Eigen::Vector3d& center_value,
               double radius,
               double length,
               const Eigen::Quaterniond& orientation)
        : center_value_(center_value),
          radius_(std::max(0.0, radius)),
          length_(std::max(0.0, length)),
          orientation_(orientation.normalized()) {}

    Eigen::Vector3d center() const override {
        return center_value_;
    }

    SupportQueryResult support(const Eigen::Vector3d& direction) const override {
        if (direction.squaredNorm() <= 1e-18) {
            return makeSupportQueryResult(center_value_, direction);
        }

        const Eigen::Vector3d local_direction = orientation_.conjugate() * direction;
        const Eigen::Vector3d local_unit = local_direction.normalized();

        const double segment_half_length = std::max(0.0, 0.5 * length_ - radius_);
        Eigen::Vector3d local_support = radius_ * local_unit;
        local_support.z() += local_direction.z() >= 0.0
            ? segment_half_length
            : -segment_half_length;

        return makeSupportQueryResult(center_value_ + orientation_ * local_support, direction);
    }

private:
    Eigen::Vector3d center_value_ = Eigen::Vector3d::Zero();
    double radius_ = 0.0;
    double length_ = 0.0;
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
};

}  // namespace convex_geometry

#endif  // CONVEX_GEOMETRY_OCCUPIED_SETS_CAPSULE_SET_H
