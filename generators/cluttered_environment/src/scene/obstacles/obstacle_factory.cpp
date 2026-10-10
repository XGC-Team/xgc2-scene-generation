/**
 * @file obstacle_factory.cpp
 * @brief Implementation of obstacle factory
 */

#include "cluttered_environment/scene/obstacles/obstacle_factory.h"

#include "cluttered_environment/scene/obstacles/polytope_obstacles.h"
#include "cluttered_environment/scene/obstacles/primitive_obstacles.h"

namespace cluttered_environment {

ObstaclePtr ObstacleFactory::createObstacle(ros::NodeHandle& nh,
                                            const ObstacleConfig& config,
                                            double tf_rate,
                                            double velocity_update_rate,
                                            double velocity_timeout) {
    try {
        ObstaclePtr obstacle;

        if (config.type == "sphere") {
            obstacle = std::make_shared<SphereObstacle>(
                nh, config, tf_rate, velocity_update_rate, velocity_timeout);
        } else if (config.type == "cylinder") {
            obstacle = std::make_shared<CylinderObstacle>(
                nh, config, tf_rate, velocity_update_rate, velocity_timeout);
        } else if (config.type == "capsule") {
            obstacle = std::make_shared<CapsuleObstacle>(
                nh, config, tf_rate, velocity_update_rate, velocity_timeout);
        } else if (config.type == "cube") {
            obstacle = std::make_shared<CubeObstacle>(
                nh, config, tf_rate, velocity_update_rate, velocity_timeout);
        } else if (config.type == "h_polytope") {
            obstacle = std::make_shared<HPolytopeObstacle>(
                nh, config, tf_rate, velocity_update_rate, velocity_timeout);
        } else if (config.type == "v_polytope") {
            obstacle = std::make_shared<VPolytopeObstacle>(
                nh, config, tf_rate, velocity_update_rate, velocity_timeout);
        } else {
            ROS_ERROR("Unknown obstacle type: %s", config.type.c_str());
            return nullptr;
        }

        obstacle->initialize();
        return obstacle;
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to create obstacle: %s", e.what());
        return nullptr;
    }
}

} // namespace cluttered_environment
