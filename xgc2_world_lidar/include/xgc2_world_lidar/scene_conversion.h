#ifndef XGC2_WORLD_LIDAR_SCENE_CONVERSION_H
#define XGC2_WORLD_LIDAR_SCENE_CONVERSION_H

// xgc2_geometry_msgs/SceneSnapshot parts -> Obstacle, without ROS.
//
// The ROS node copies each message field into ScenePartDescription; the rules
// below are the whole conversion and are unit-tested without ROS. A part's
// world pose is
// obstacle_pose * part_pose; box `size` is the full side length; cylinder
// `height` is the full height along local z; capsule `height` is the straight
// part; convex `vertices` are part-frame points (triangles are not needed, the
// solid is their convex hull).

#include <string>
#include <vector>

#include "xgc2_world_lidar/world_lidar.h"

namespace xgc2_world_lidar {

struct Pose {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

struct ScenePartDescription {
    std::string type; // box | sphere | cylinder | capsule | convex
    Pose pose;        // part frame in the obstacle frame
    Eigen::Vector3d size = Eigen::Vector3d::Zero();
    double radius = 0.0;
    double height = 0.0;
    std::vector<Eigen::Vector3d> vertices;
};

struct SceneObstacleDescription {
    std::string id;
    Pose pose; // obstacle frame in the world (already the current state pose for a mover)
    std::vector<ScenePartDescription> parts;
};

// Throws std::invalid_argument naming the obstacle and part on an unknown type
// or invalid geometry.
std::vector<Obstacle> toObstacles(const std::vector<SceneObstacleDescription>& scene);

} // namespace xgc2_world_lidar

#endif // XGC2_WORLD_LIDAR_SCENE_CONVERSION_H
