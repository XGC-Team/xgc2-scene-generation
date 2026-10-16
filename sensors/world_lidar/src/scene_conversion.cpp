#include "xgc2_world_lidar/scene_conversion.h"

#include <cmath>
#include <stdexcept>
#include <xgc2_math/scene/scene_conversion.hpp>

namespace xgc2_world_lidar {

namespace {

Eigen::Quaterniond normalized(const Eigen::Quaterniond& q) {
    const double n = q.norm();
    if (!std::isfinite(n) || n < 1e-9)
        throw std::invalid_argument("invalid orientation");
    return Eigen::Quaterniond(q.coeffs() / n);
}

Obstacle toObstacle(const ScenePartDescription& part,
                    const Eigen::Vector3d& position,
                    const Eigen::Quaterniond& orientation) {
    if (!part.geometry)
        throw std::invalid_argument("missing geometry definition");
    Obstacle result = *part.geometry;
    result.position = position;
    result.orientation = orientation;
    return result;
}

} // namespace

std::shared_ptr<const Obstacle>
prepareGeometry(const xgc2_math::geometry::GeometryPayload& geometry) {
    auto result = xgc2_math::scene::obstacleFromGeometry(geometry);
    xgc2_world_lidar::sampleSurface(result, 1e9); // validates once, never per state
    return std::make_shared<const Obstacle>(std::move(result));
}

std::vector<Obstacle> toObstacles(const std::vector<SceneObstacleDescription>& scene) {
    std::vector<Obstacle> out;
    for (const auto& obstacle : scene) {
        for (std::size_t i = 0; i < obstacle.parts.size(); ++i) {
            const auto& part = obstacle.parts[i];
            try {
                const Eigen::Quaterniond parent = normalized(obstacle.pose.orientation);
                const Eigen::Vector3d position =
                    obstacle.pose.position + parent * part.pose.position;
                const Eigen::Quaterniond orientation = parent * normalized(part.pose.orientation);
                Obstacle o = toObstacle(part, position, orientation);
                out.push_back(std::move(o));
            } catch (const std::invalid_argument& e) {
                throw std::invalid_argument("obstacle '" + obstacle.id + "' part " +
                                            std::to_string(i) + ": " + e.what());
            }
        }
    }
    return out;
}

} // namespace xgc2_world_lidar
