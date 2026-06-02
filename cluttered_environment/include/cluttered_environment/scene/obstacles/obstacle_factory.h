/**
 * @file obstacle_factory.h
 * @brief Factory for creating obstacle objects
 */

#ifndef CLUTTERED_ENVIRONMENT_SCENE_OBSTACLES_OBSTACLE_FACTORY_H
#define CLUTTERED_ENVIRONMENT_SCENE_OBSTACLES_OBSTACLE_FACTORY_H

#include "cluttered_environment/scene/obstacles/obstacle_base.h"

namespace cluttered_environment {

class ObstacleFactory {
public:
    static ObstaclePtr createObstacle(ros::NodeHandle& nh,
                                      const ObstacleConfig& config,
                                      double tf_rate,
                                      double velocity_update_rate,
                                      double velocity_timeout);
};

}  // namespace cluttered_environment

#endif  // CLUTTERED_ENVIRONMENT_SCENE_OBSTACLES_OBSTACLE_FACTORY_H
