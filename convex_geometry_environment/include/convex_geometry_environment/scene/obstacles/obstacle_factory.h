/**
 * @file obstacle_factory.h
 * @brief Factory for creating obstacle objects
 */

#ifndef CONVEX_GEOMETRY_ENVIRONMENT_SCENE_OBSTACLES_OBSTACLE_FACTORY_H
#define CONVEX_GEOMETRY_ENVIRONMENT_SCENE_OBSTACLES_OBSTACLE_FACTORY_H

#include "convex_geometry_environment/scene/obstacles/obstacle_base.h"

namespace convex_geometry_environment {

class ObstacleFactory {
public:
    static ObstaclePtr createObstacle(ros::NodeHandle& nh,
                                      const ObstacleConfig& config,
                                      double tf_rate,
                                      double velocity_update_rate,
                                      double velocity_timeout);
};

}  // namespace convex_geometry_environment

#endif  // CONVEX_GEOMETRY_ENVIRONMENT_SCENE_OBSTACLES_OBSTACLE_FACTORY_H
