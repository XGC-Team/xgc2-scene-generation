#ifndef CONVEX_GEOMETRY_OCCUPIED_SETS_BOX_SET_H
#define CONVEX_GEOMETRY_OCCUPIED_SETS_BOX_SET_H

#include <Eigen/Dense>
#include "convex_geometry/occupied_sets/convex_set_base.h"

namespace convex_geometry {

class BoxSet final : public ConvexSet3D {
public:
    BoxSet() = default;

    BoxSet(const Eigen::Vector3d& center_value,
           const Eigen::Vector3d& size,
           const Eigen::Quaterniond& orientation)
        : center_value_(center_value), size_(size), orientation_(orientation.normalized()) {}

    Eigen::Vector3d center() const override {
        return center_value_;
    }

    SupportQueryResult support(const Eigen::Vector3d& direction) const override {
        const Eigen::Vector3d local_direction = orientation_.conjugate() * direction;
        const Eigen::Vector3d local_support(
            local_direction.x() >= 0.0 ? 0.5 * size_.x() : -0.5 * size_.x(),
            local_direction.y() >= 0.0 ? 0.5 * size_.y() : -0.5 * size_.y(),
            local_direction.z() >= 0.0 ? 0.5 * size_.z() : -0.5 * size_.z());
        return makeSupportQueryResult(center_value_ + orientation_ * local_support, direction);
    }

private:
    Eigen::Vector3d center_value_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d size_ = Eigen::Vector3d::Ones();
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
};

}  // namespace convex_geometry

#endif  // CONVEX_GEOMETRY_OCCUPIED_SETS_BOX_SET_H
