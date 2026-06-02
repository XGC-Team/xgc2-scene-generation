#ifndef CONVEX_GEOMETRY_OCCUPIED_SETS_POINT_SET_H
#define CONVEX_GEOMETRY_OCCUPIED_SETS_POINT_SET_H

#include <Eigen/Dense>
#include <limits>
#include <utility>
#include <vector>
#include "convex_geometry/occupied_sets/convex_set_base.h"

namespace convex_geometry {

template <typename PointContainer = std::vector<Eigen::Vector3d>>
class PointSet final : public ConvexSet3D {
public:
    PointSet() = default;

    PointSet(const Eigen::Vector3d& center_value, PointContainer support_point_offsets)
        : center_value_(center_value), support_point_offsets_(std::move(support_point_offsets)) {}

    Eigen::Vector3d center() const override {
        return center_value_;
    }

    SupportQueryResult support(const Eigen::Vector3d& direction) const override {
        if (support_point_offsets_.empty()) {
            return makeSupportQueryResult(center_value_, direction);
        }

        double best_value = -std::numeric_limits<double>::infinity();
        Eigen::Vector3d best_offset = Eigen::Vector3d::Zero();
        for (const auto& offset : support_point_offsets_) {
            const double candidate_value = offset.dot(direction);
            if (candidate_value > best_value) {
                best_value = candidate_value;
                best_offset = offset;
            }
        }

        return makeSupportQueryResult(center_value_ + best_offset, direction);
    }

    const PointContainer& supportPointOffsets() const {
        return support_point_offsets_;
    }

private:
    Eigen::Vector3d center_value_ = Eigen::Vector3d::Zero();
    PointContainer support_point_offsets_;
};

using SupportPointSet = PointSet<std::vector<Eigen::Vector3d>>;

}  // namespace convex_geometry

#endif  // CONVEX_GEOMETRY_OCCUPIED_SETS_POINT_SET_H
