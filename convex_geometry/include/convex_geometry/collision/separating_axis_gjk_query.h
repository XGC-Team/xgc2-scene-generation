#ifndef CONVEX_GEOMETRY_COLLISION_SEPARATING_AXIS_GJK_QUERY_H
#define CONVEX_GEOMETRY_COLLISION_SEPARATING_AXIS_GJK_QUERY_H

#include <Eigen/Dense>
#include <cmath>

#include "convex_geometry/collision/separation_query.h"

namespace convex_geometry {
namespace gjk {

struct SeparatingAxisGjkQuery {
    template <typename SetA, typename SetB>
    static Result query(const SetA& set_a,
                        const SetB& set_b,
                        const WarmStart* warm_start,
                        double minimum_margin,
                        int max_iterations,
                        double tolerance) {
        Result result;
        if (minimum_margin <= 0.0 || max_iterations <= 0 || tolerance <= 0.0) {
            result.status = Result::Status::kInvalid;
            return result;
        }

        detail::Simplex simplex;

        const auto hasWarmStart = [&]() {
            return warm_start != nullptr && warm_start->valid;
        };

        const auto hasDirectionalWarmStart = [&]() {
            return hasWarmStart() &&
                   warm_start->separator.normal.array().isFinite().all() &&
                   warm_start->separator.normal.squaredNorm() > 1e-12;
        };

        const auto hasReusableWarmStart = [&]() {
#if CONVEX_GEOMETRY_GJK_ALWAYS_TRUST_WARM_START
            return hasWarmStart();
#else
            return hasWarmStart() &&
                   std::isfinite(warm_start->separator.margin) &&
                   warm_start->separator.margin > 0.0 &&
                   warm_start->separator.normal.array().isFinite().all() &&
                   warm_start->separator.normal.squaredNorm() > 1e-12 &&
                   warm_start->separator.point_a.array().isFinite().all() &&
                   warm_start->separator.point_b.array().isFinite().all();
#endif
        };

        const auto fallbackDirection = [&](const Eigen::Vector3d& preferred_direction) -> Eigen::Vector3d {
            if (preferred_direction.squaredNorm() > 1e-12) {
                return preferred_direction.normalized();
            }
            if (hasDirectionalWarmStart()) {
                return warm_start->separator.normal.normalized();
            }
            return Eigen::Vector3d::UnitX();
        };

        const auto isUsableSeparator = [&](const SeparationQuadruple& separator) {
            return std::isfinite(separator.margin) &&
                   separator.margin > 0.0 &&
                   separator.normal.array().isFinite().all() &&
                   separator.normal.squaredNorm() > 1e-12 &&
                   separator.point_a.array().isFinite().all() &&
                   separator.point_b.array().isFinite().all();
        };

        const auto setResult = [&](Result::Status status, const SeparationQuadruple& separator, int iterations) {
            result.status = status;
            result.separator = separator;
            result.iterations = iterations;
            return result;
        };

        const auto makeVertex = [&](const Eigen::Vector3d& query_direction) {
            detail::Vertex vertex;
            const Eigen::Vector3d normal = fallbackDirection(query_direction);
            const auto support_a = set_a.support(normal);
            const auto support_b = set_b.support(-normal);
            vertex.a = support_a.support_point;
            vertex.b = support_b.support_point;
            vertex.w = vertex.b - vertex.a;
            return vertex;
        };

        const auto makeAxisSeparator = [&](const Eigen::Vector3d& query_direction,
                                           const detail::Vertex& vertex) {
            SeparationQuadruple separator;
            separator.normal = fallbackDirection(query_direction);
            separator.point_a = vertex.a;
            separator.point_b = vertex.b;
            separator.margin = separator.normal.dot(vertex.w);
            return separator;
        };

        const auto belowMinimumMargin = [&](double margin) {
            return margin < minimum_margin;
        };

        const auto makeSeparatorFromSimplex = [&](const Eigen::Vector3d& preferred_direction) {
            SeparationQuadruple separator;
            separator.point_a = simplex.point_a;
            separator.point_b = simplex.point_b;
            separator.margin = simplex.closest.norm();
            separator.normal =
                separator.margin > 1e-12 ? simplex.closest / separator.margin
                                         : fallbackDirection(preferred_direction);
            return separator;
        };

        const auto fallbackToWarmStartOrCurrent =
            [&](Result::Status status, const Eigen::Vector3d& preferred_direction, int iterations) {
                if (hasReusableWarmStart()) {
                    return setResult(status, warm_start->separator, iterations);
                }
                if (simplex.count > 0) {
                    return setResult(status, makeSeparatorFromSimplex(preferred_direction), iterations);
                }
                const detail::Vertex vertex = makeVertex(preferred_direction);
                return setResult(status, makeAxisSeparator(preferred_direction, vertex), iterations);
            };

        Eigen::Vector3d direction = set_b.center() - set_a.center();
        if (direction.squaredNorm() <= 1e-12) {
            direction = fallbackDirection(direction);
        }

        for (int iter = 0; iter < max_iterations; ++iter) {
            const Eigen::Vector3d query_direction = fallbackDirection(direction);
            const detail::Vertex vertex = makeVertex(query_direction);
            const SeparationQuadruple axis_separator =
                makeAxisSeparator(query_direction, vertex);

            if (isUsableSeparator(axis_separator)) {
                if (belowMinimumMargin(axis_separator.margin)) {
                    return fallbackToWarmStartOrCurrent(
                        Result::Status::kOverlap,
                        query_direction,
                        iter + 1);
                }
                return setResult(Result::Status::kSuccess, axis_separator, iter + 1);
            }

            if (simplex.count > 0 && detail::hasEquivalentVertex(simplex, vertex)) {
                return fallbackToWarmStartOrCurrent(
                    Result::Status::kMaxIterations,
                    query_direction,
                    iter + 1);
            }

            if (simplex.count == 0) {
                simplex.assignSingle(vertex);
            } else {
                simplex.append(vertex);
            }

            const bool contains_origin = detail::solveSimplex(simplex);
            if (contains_origin) {
                simplex.closest.setZero();
                simplex.point_b = simplex.point_a;
                return fallbackToWarmStartOrCurrent(
                    Result::Status::kOverlap,
                    query_direction,
                    iter + 1);
            }

            if (simplex.closest.squaredNorm() <= 1e-18) {
                return fallbackToWarmStartOrCurrent(
                    Result::Status::kOverlap,
                    query_direction,
                    iter + 1);
            }

            direction = simplex.closest;
        }

        return fallbackToWarmStartOrCurrent(
            Result::Status::kMaxIterations,
            direction,
            max_iterations);
    }
};

}  // namespace gjk
}  // namespace convex_geometry

#endif  // CONVEX_GEOMETRY_COLLISION_SEPARATING_AXIS_GJK_QUERY_H
