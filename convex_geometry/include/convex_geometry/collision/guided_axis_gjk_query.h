#ifndef CONVEX_GEOMETRY_COLLISION_GUIDED_AXIS_GJK_QUERY_H
#define CONVEX_GEOMETRY_COLLISION_GUIDED_AXIS_GJK_QUERY_H

#include <Eigen/Dense>
#include <chrono>
#include <cmath>

#include "convex_geometry/collision/distance_gjk_query.h"
#include "convex_geometry/geometry/math_helpers.h"

namespace convex_geometry {
namespace gjk {

struct GuidedAxisGjkQuery {
    template <typename SetA, typename SetB>
    static Result query(const SetA& set_a,
                        const SetB& set_b,
                        const WarmStart* warm_start,
                        double query_minimum_distance,
                        double guided_minimum_margin,
                        int max_iterations,
                        double tolerance) {
        Result result = DistanceGjkQuery::query(
            set_a,
            set_b,
            warm_start,
            query_minimum_distance,
            max_iterations,
            tolerance);

        if (!isAdjustableResult(result) ||
            result.status == Result::Status::kOverlap) {
            return result;
        }

        const Eigen::Vector3d distance_normal =
            math_helpers::normalizedOrZero(result.separator.normal, kDirectionEps);
        const Eigen::Vector3d center_normal =
            math_helpers::normalizedOrZero(set_b.center() - set_a.center(), kDirectionEps);
        if (!canOptimizeSphericalAxis(distance_normal, center_normal)) {
            return result;
        }

        SeparationQuadruple guided_separator;
        result.guide_attempted = true;
        const auto guide_start = std::chrono::steady_clock::now();
        if (optimizeSphericalAxisSeparator(
                set_a,
                set_b,
                distance_normal,
                center_normal,
                guided_minimum_margin,
                guided_separator)) {
            result.separator = guided_separator;
            result.status = Result::Status::kSuccess;
            result.guide_success = true;
        }
        const auto guide_end = std::chrono::steady_clock::now();
        result.guide_correction_time_us =
            std::chrono::duration<double, std::micro>(guide_end - guide_start).count();
        return result;
    }

private:
    static constexpr double kDirectionEps = math_helpers::kDirectionEpsilon;
    static constexpr double kDotEps = 1e-12;
    static constexpr double kAlphaEps = 1e-12;
    static constexpr int kGuidedAxisSamples = 16;
    static constexpr int kGuidedAxisRefinementIterations = 12;

    static bool canOptimizeSphericalAxis(const Eigen::Vector3d& from_normal,
                                         const Eigen::Vector3d& to_normal) {
        return from_normal.squaredNorm() > kDirectionEps &&
               to_normal.squaredNorm() > kDirectionEps &&
               from_normal.dot(to_normal) > -1.0 + kDotEps;
    }

    struct SphericalAxisInterpolator {
        SphericalAxisInterpolator(const Eigen::Vector3d& from_normal,
                                  const Eigen::Vector3d& to_normal)
            : from(from_normal), to(to_normal) {
            const double dot = math_helpers::clamp(from.dot(to), -1.0, 1.0);
            if (dot > 1.0 - kDotEps) {
                return;
            }

            theta = std::acos(dot);
            const double sin_theta = std::sin(theta);
            if (sin_theta <= kDirectionEps) {
                return;
            }

            inv_sin_theta = 1.0 / sin_theta;
            use_interpolation = true;
        }

        Eigen::Vector3d interpolate(double alpha) const {
            if (!use_interpolation) {
                return from;
            }

            const double from_weight =
                std::sin((1.0 - alpha) * theta) * inv_sin_theta;
            const double to_weight = std::sin(alpha * theta) * inv_sin_theta;
            return math_helpers::normalizedOrZero(
                from_weight * from + to_weight * to,
                kDirectionEps);
        }

        Eigen::Vector3d from = Eigen::Vector3d::Zero();
        Eigen::Vector3d to = Eigen::Vector3d::Zero();
        double theta = 0.0;
        double inv_sin_theta = 0.0;
        bool use_interpolation = false;
    };

    template <typename SetA, typename SetB>
    static SeparationQuadruple makeUnitAxisSeparator(
        const SetA& set_a,
        const SetB& set_b,
        const Eigen::Vector3d& unit_axis) {
        SeparationQuadruple separator;
        separator.normal = unit_axis;
        const auto support_a = set_a.support(unit_axis);
        const auto support_b = set_b.support(-unit_axis);
        separator.point_a = support_a.support_point;
        separator.point_b = support_b.support_point;
        separator.margin = -support_b.support_value - support_a.support_value;
        return separator;
    }

    template <typename SetA, typename SetB>
    static bool optimizeSphericalAxisSeparator(
        const SetA& set_a,
        const SetB& set_b,
        const Eigen::Vector3d& from_normal,
        const Eigen::Vector3d& to_normal,
        double minimum_margin,
        SeparationQuadruple& best_separator) {
        const SphericalAxisInterpolator interpolator(from_normal, to_normal);
        const double alpha_step =
            1.0 / static_cast<double>(kGuidedAxisSamples);
        double best_alpha = 0.0;
        bool found_separator = false;

        for (int sample = kGuidedAxisSamples; sample >= 1; --sample) {
            const double alpha = static_cast<double>(sample) * alpha_step;
            const Eigen::Vector3d axis = interpolator.interpolate(alpha);
            const SeparationQuadruple candidate =
                makeUnitAxisSeparator(set_a, set_b, axis);

            if (!isVerifiedSeparator(candidate, minimum_margin)) {
                continue;
            }

            best_separator = candidate;
            best_alpha = alpha;
            found_separator = true;
            break;
        }

        if (!found_separator) {
            return false;
        }

        if (best_alpha < 1.0 - kAlphaEps) {
            double low = best_alpha;
            double high = best_alpha + alpha_step;
            if (high > 1.0) {
                high = 1.0;
            }

            for (int iter = 0; iter < kGuidedAxisRefinementIterations; ++iter) {
                const double alpha = 0.5 * (low + high);
                const Eigen::Vector3d axis = interpolator.interpolate(alpha);
                const SeparationQuadruple candidate =
                    makeUnitAxisSeparator(set_a, set_b, axis);

                if (isVerifiedSeparator(candidate, minimum_margin)) {
                    best_separator = candidate;
                    low = alpha;
                } else {
                    high = alpha;
                }
            }
        }

        return true;
    }

    static bool isUsableSeparator(const SeparationQuadruple& separator) {
        return std::isfinite(separator.margin) &&
               separator.margin > 0.0 &&
               math_helpers::isFiniteVector3(separator.normal) &&
               separator.normal.squaredNorm() > kDirectionEps &&
               math_helpers::isFiniteVector3(separator.point_a) &&
               math_helpers::isFiniteVector3(separator.point_b);
    }

    static bool isVerifiedSeparator(const SeparationQuadruple& separator,
                                    double minimum_margin) {
        return isUsableSeparator(separator) &&
               separator.margin >= minimum_margin;
    }

    static bool isAdjustableResult(const Result& result) {
        return result.status != Result::Status::kInvalid &&
               isUsableSeparator(result.separator);
    }
};

}  // namespace gjk
}  // namespace convex_geometry

#endif  // CONVEX_GEOMETRY_COLLISION_GUIDED_AXIS_GJK_QUERY_H
