#ifndef CONVEX_GEOMETRY_OCCUPIED_SETS_SWEPT_HULL_SET_H
#define CONVEX_GEOMETRY_OCCUPIED_SETS_SWEPT_HULL_SET_H

#include <Eigen/Dense>
#include "convex_geometry/occupied_sets/convex_set_base.h"

namespace convex_geometry {

template <typename StartSet, typename EndSet>
class SweptHullSet {
public:
    SweptHullSet(const StartSet& start_set, const EndSet& end_set)
        : start_set_(&start_set), end_set_(&end_set) {}

    Eigen::Vector3d center() const {
        return 0.5 * (start_set_->center() + end_set_->center());
    }

    SupportQueryResult support(const Eigen::Vector3d& direction) const {
        const SupportQueryResult start_support = start_set_->support(direction);
        const SupportQueryResult end_support = end_set_->support(direction);
        return end_support.support_value > start_support.support_value ? end_support : start_support;
    }

private:
    const StartSet* start_set_ = nullptr;
    const EndSet* end_set_ = nullptr;
};

}  // namespace convex_geometry

#endif  // CONVEX_GEOMETRY_OCCUPIED_SETS_SWEPT_HULL_SET_H
