#ifndef CONVEX_GEOMETRY_OCCUPIED_SETS_CYLINDER_SET_H
#define CONVEX_GEOMETRY_OCCUPIED_SETS_CYLINDER_SET_H

#include <Eigen/Dense>
#include <cmath>
#include "convex_geometry/occupied_sets/convex_set_base.h"

namespace convex_geometry {

class CylinderSet final : public ConvexSet3D {
public:
    CylinderSet() = default;

    CylinderSet(const Eigen::Vector3d& center_value,
                double radius,
                double height,
                const Eigen::Quaterniond& orientation)
        : center_value_(center_value),
          radius_(radius),
          height_(height),
          orientation_(orientation.normalized()) {}

    Eigen::Vector3d center() const override {
        return center_value_;
    }

    SupportQueryResult support(const Eigen::Vector3d& direction) const override {
        const Eigen::Vector3d local_direction = orientation_.conjugate() * direction;
        Eigen::Vector3d local_support = Eigen::Vector3d::Zero();
        const double radial_norm = std::hypot(local_direction.x(), local_direction.y());
        if (radial_norm > 1e-9) {
            local_support.x() = radius_ * local_direction.x() / radial_norm;
            local_support.y() = radius_ * local_direction.y() / radial_norm;
        }
        local_support.z() = local_direction.z() >= 0.0 ? 0.5 * height_ : -0.5 * height_;
        return makeSupportQueryResult(center_value_ + orientation_ * local_support, direction);
    }

private:
    Eigen::Vector3d center_value_ = Eigen::Vector3d::Zero();
    double radius_ = 0.0;
    double height_ = 0.0;
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
};

}  // namespace convex_geometry

#endif  // CONVEX_GEOMETRY_OCCUPIED_SETS_CYLINDER_SET_H
