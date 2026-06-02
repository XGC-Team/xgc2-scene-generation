#ifndef CONVEX_GEOMETRY_OCCUPIED_SETS_TRANSLATED_SET_VIEW_H
#define CONVEX_GEOMETRY_OCCUPIED_SETS_TRANSLATED_SET_VIEW_H

#include <Eigen/Dense>
#include "convex_geometry/occupied_sets/convex_set_base.h"

namespace convex_geometry {

template <typename BaseSet>
class TranslatedSetView {
public:
    TranslatedSetView(const BaseSet& base_set, const Eigen::Vector3d& translation)
        : base_set_(&base_set), translation_(translation) {}

    Eigen::Vector3d center() const {
        return base_set_->center() + translation_;
    }

    SupportQueryResult support(const Eigen::Vector3d& direction) const {
        SupportQueryResult result = base_set_->support(direction);
        result.support_point += translation_;
        result.support_value += translation_.dot(direction);
        return result;
    }

private:
    const BaseSet* base_set_ = nullptr;
    Eigen::Vector3d translation_ = Eigen::Vector3d::Zero();
};

}  // namespace convex_geometry

#endif  // CONVEX_GEOMETRY_OCCUPIED_SETS_TRANSLATED_SET_VIEW_H
