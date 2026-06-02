#ifndef CONVEX_GEOMETRY_COLLISION_DISTANCE_GJK_QUERY_H
#define CONVEX_GEOMETRY_COLLISION_DISTANCE_GJK_QUERY_H

#include <Eigen/Dense>
#include <chrono>
#include <cmath>

#include "convex_geometry/collision/separation_query.h"

namespace convex_geometry {
namespace gjk {

struct DistanceGjkQuery {
    template <typename SetA, typename SetB>
    static Result query(const SetA& set_a,
                        const SetB& set_b,
                        const WarmStart* warm_start,
                        double minimum_distance,
                        int max_iterations,
                        double tolerance) {
        Result result;
        const auto query_start = std::chrono::steady_clock::now();
        const auto stampTime = [&]() {
            const auto query_end = std::chrono::steady_clock::now();
            result.distance_gjk_time_us =
                std::chrono::duration<double, std::micro>(query_end - query_start).count();
        };
        if (minimum_distance <= 0.0 || max_iterations <= 0 || tolerance <= 0.0) {
            result.status = Result::Status::kInvalid;
            stampTime();
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

        const auto setResult = [&](Result::Status status, const SeparationQuadruple& separator, int iterations) {
            result.status = status;
            result.separator = separator;
            result.iterations = iterations;
            stampTime();
            return result;
        };

        Eigen::Vector3d direction = detail::chooseInitialDirection(set_b.center() - set_a.center(), warm_start);
        const double minimum_distance_sq = minimum_distance * minimum_distance;

        const auto fallbackToWarmStartOrCurrent =
            [&](Result::Status status, const Eigen::Vector3d& preferred_direction, int iterations) {
                if (hasReusableWarmStart()) {
                    return setResult(status, warm_start->separator, iterations);
                }
                return setResult(status, makeSeparatorFromSimplex(preferred_direction), iterations);
            };

        const auto makeVertex = [&](const Eigen::Vector3d& query_direction) {
            detail::Vertex vertex;
            const auto support_a = set_a.support(query_direction);
            const auto support_b = set_b.support(-query_direction);
            vertex.a = support_a.support_point;
            vertex.b = support_b.support_point;
            vertex.w = vertex.b - vertex.a;
            return vertex;
        };

        const auto currentDistanceSquared = [&]() {
            return simplex.closest.squaredNorm();
        };

        const auto belowMinimumDistance = [&](double distance_sq) {
            return distance_sq < minimum_distance_sq;
        };

        const auto nextDirection = [&]() {
            return simplex.closest.normalized();
        };

        const auto progressMetric = [&](const detail::Vertex& vertex, double distance_sq) {
            return distance_sq - simplex.closest.dot(vertex.w);
        };

        const auto reachedTolerance = [&](double progress) {
            return progress <= tolerance;
        };

        simplex.assignSingle(makeVertex(direction));

        for (int iter = 0; iter < max_iterations; ++iter) {
            const int completed_iterations = iter;
            const double distance_sq = currentDistanceSquared();
            const bool overlap_warning = belowMinimumDistance(distance_sq);
            if (overlap_warning) {
                return fallbackToWarmStartOrCurrent(
                    Result::Status::kOverlap,
                    direction,
                    completed_iterations);
            }

            const int query_iteration = iter + 1;
            direction = nextDirection();
            const detail::Vertex vertex = makeVertex(direction);
            const bool duplicate_support = detail::hasEquivalentVertex(simplex, vertex);
            if (duplicate_support) {
                return setResult(Result::Status::kSuccess, makeSeparatorFromSimplex(direction), query_iteration);
            }

            const double progress = progressMetric(vertex, distance_sq);
            const bool converged = reachedTolerance(progress);
            if (converged) {
                return setResult(Result::Status::kSuccess, makeSeparatorFromSimplex(direction), query_iteration);
            }

            simplex.append(vertex);
            const bool contains_origin = detail::solveSimplex(simplex);
            if (contains_origin) {
                simplex.closest.setZero();
                simplex.point_b = simplex.point_a;
            }
        }

        const SeparationQuadruple current_separator = makeSeparatorFromSimplex(direction);
        if (isUsableSeparator(current_separator)) {
            return setResult(Result::Status::kMaxIterations, current_separator, max_iterations);
        }
        return fallbackToWarmStartOrCurrent(Result::Status::kMaxIterations, direction, max_iterations);
    }
};

}  // namespace gjk
}  // namespace convex_geometry

#endif  // CONVEX_GEOMETRY_COLLISION_DISTANCE_GJK_QUERY_H
