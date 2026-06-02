#ifndef CONVEX_GEOMETRY_OCCUPIED_SETS_BALL_INFLATED_SET_H
#define CONVEX_GEOMETRY_OCCUPIED_SETS_BALL_INFLATED_SET_H

#include <algorithm>
#include <memory>
#include <utility>

#include <Eigen/Dense>

#include "convex_geometry/occupied_sets/convex_set_base.h"

namespace convex_geometry {

class BallInflatedSet final : public ConvexSet3D {
public:
    BallInflatedSet(std::shared_ptr<const ConvexSet3D> base_set, double radius)
        : base_set_(std::move(base_set)), radius_(std::max(0.0, radius)) {}

    Eigen::Vector3d center() const override {
        return base_set_ ? base_set_->center() : Eigen::Vector3d::Zero();
    }

    SupportQueryResult support(const Eigen::Vector3d& direction) const override {
        SupportQueryResult result = base_set_
            ? base_set_->support(direction)
            : makeSupportQueryResult(Eigen::Vector3d::Zero(), direction);

        const double direction_norm = direction.norm();
        if (radius_ <= 0.0 || direction_norm <= 1e-18) {
            return result;
        }

        result.support_point += radius_ * direction / direction_norm;
        result.support_value += radius_ * direction_norm;
        return result;
    }

private:
    std::shared_ptr<const ConvexSet3D> base_set_;
    double radius_ = 0.0;
};

}  // namespace convex_geometry

#endif  // CONVEX_GEOMETRY_OCCUPIED_SETS_BALL_INFLATED_SET_H
