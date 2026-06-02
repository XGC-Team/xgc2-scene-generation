#ifndef CONVEX_GEOMETRY_OCCUPIED_SETS_SPHERE_SET_H
#define CONVEX_GEOMETRY_OCCUPIED_SETS_SPHERE_SET_H

#include <Eigen/Dense>
#include "convex_geometry/occupied_sets/convex_set_base.h"

namespace convex_geometry {

class SphereSet final : public ConvexSet3D {
public:
    SphereSet() = default;

    SphereSet(const Eigen::Vector3d& center_value, double radius)
        : center_value_(center_value), radius_(radius) {}

    Eigen::Vector3d center() const override {
        return center_value_;
    }

    SupportQueryResult support(const Eigen::Vector3d& direction) const override {
        if (direction.squaredNorm() <= 1e-18) {
            return makeSupportQueryResult(center_value_, direction);
        }
        return makeSupportQueryResult(center_value_ + radius_ * direction.normalized(), direction);
    }

    double radius() const {
        return radius_;
    }

private:
    Eigen::Vector3d center_value_ = Eigen::Vector3d::Zero();
    double radius_ = 0.0;
};

}  // namespace convex_geometry

#endif  // CONVEX_GEOMETRY_OCCUPIED_SETS_SPHERE_SET_H
