#ifndef CONVEX_GEOMETRY_OCCUPIED_SETS_CONVEX_BODY_H
#define CONVEX_GEOMETRY_OCCUPIED_SETS_CONVEX_BODY_H

#include <memory>
#include <string>
#include <Eigen/Dense>
#include "convex_geometry/occupied_sets/convex_set_base.h"

namespace convex_geometry {

struct ConvexBody {
    int id = -1;
    std::string name;
    std::string geometry_type;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d scale = Eigen::Vector3d::Ones();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    std::shared_ptr<const ConvexSet3D> shape;

    bool isRegistered() const {
        return static_cast<bool>(shape);
    }

    Eigen::Vector3d center() const {
        return shape ? shape->center() : position;
    }

    SupportQueryResult support(const Eigen::Vector3d& direction) const {
        return shape ? shape->support(direction) : makeSupportQueryResult(position, direction);
    }
};

}  // namespace convex_geometry

#endif  // CONVEX_GEOMETRY_OCCUPIED_SETS_CONVEX_BODY_H
