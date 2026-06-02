#ifndef CONVEX_GEOMETRY_OCCUPIED_SETS_CONVEX_SET_BASE_H
#define CONVEX_GEOMETRY_OCCUPIED_SETS_CONVEX_SET_BASE_H

#include <Eigen/Dense>

namespace convex_geometry {

struct SupportQueryResult {
    Eigen::Vector3d support_point = Eigen::Vector3d::Zero();
    double support_value = 0.0;
};

inline SupportQueryResult makeSupportQueryResult(const Eigen::Vector3d& support_point,
                                                 const Eigen::Vector3d& direction) {
    SupportQueryResult result;
    result.support_point = support_point;
    result.support_value = support_point.dot(direction);
    return result;
}

class ConvexSet3D {
public:
    virtual ~ConvexSet3D() = default;

    virtual Eigen::Vector3d center() const = 0;
    virtual SupportQueryResult support(const Eigen::Vector3d& direction) const = 0;
};

}  // namespace convex_geometry

#endif  // CONVEX_GEOMETRY_OCCUPIED_SETS_CONVEX_SET_BASE_H
