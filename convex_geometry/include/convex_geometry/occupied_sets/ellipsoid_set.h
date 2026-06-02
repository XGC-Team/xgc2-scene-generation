#ifndef CONVEX_GEOMETRY_OCCUPIED_SETS_ELLIPSOID_SET_H
#define CONVEX_GEOMETRY_OCCUPIED_SETS_ELLIPSOID_SET_H

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

#include "convex_geometry/occupied_sets/convex_set_base.h"

namespace convex_geometry {

class EllipsoidSet final : public ConvexSet3D {
public:
    EllipsoidSet() = default;

    EllipsoidSet(const Eigen::Vector3d& center_value,
                 const Eigen::Vector3d& radii,
                 const Eigen::Quaterniond& orientation)
        : center_value_(center_value),
          radii_(radii.cwiseAbs()),
          orientation_(orientation.normalized()) {}

    Eigen::Vector3d center() const override {
        return center_value_;
    }

    SupportQueryResult support(const Eigen::Vector3d& direction) const override {
        if (direction.squaredNorm() <= 1e-18) {
            return makeSupportQueryResult(center_value_, direction);
        }

        const Eigen::Vector3d local_direction = orientation_.conjugate() * direction;
        const Eigen::Vector3d squared_radii = radii_.cwiseProduct(radii_);
        const Eigen::Vector3d numerator = squared_radii.cwiseProduct(local_direction);
        const double denominator = std::sqrt(std::max(0.0, local_direction.dot(numerator)));
        if (denominator <= 1e-18) {
            return makeSupportQueryResult(center_value_, direction);
        }

        const Eigen::Vector3d local_support = numerator / denominator;
        return makeSupportQueryResult(center_value_ + orientation_ * local_support, direction);
    }

private:
    Eigen::Vector3d center_value_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d radii_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
};

}  // namespace convex_geometry

#endif  // CONVEX_GEOMETRY_OCCUPIED_SETS_ELLIPSOID_SET_H
