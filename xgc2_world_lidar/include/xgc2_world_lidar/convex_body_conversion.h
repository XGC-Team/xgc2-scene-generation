#ifndef XGC2_WORLD_LIDAR_CONVEX_BODY_CONVERSION_H
#define XGC2_WORLD_LIDAR_CONVEX_BODY_CONVERSION_H

// xgc2_geometry_msgs/GeometryLibrary + ConvexBodyArray -> Obstacle, without ROS.
//
// This is the Gazebo simulator's obstacle truth
// (/xgc2/simulation/obstacles/geometry_library and .../instances, published by
// the xgc2_gazebo_scene system plugin from the actual collision geometry of
// every managed obstacle). Each instance is one convex part in the world frame:
//
//   geometry_type "cube"      box, `scale` = full side lengths
//   geometry_type "sphere"    sphere, radius = scale.x
//   geometry_type "cylinder"  cylinder along local z, radius = scale.x,
//                             full height = scale.z
//   any other type            the library template of that type; its
//                             support points, multiplied component-wise by
//                             `scale`, are local-frame vertices whose convex
//                             hull is the solid (collision meshes arrive this
//                             way, already reduced to their convex vertices)
//
// `pose` places the part: local -> world.

#include <string>
#include <vector>

#include "xgc2_world_lidar/scene_conversion.h"
#include "xgc2_world_lidar/world_lidar.h"

namespace xgc2_world_lidar {

struct GeometryTemplateDescription {
    std::string type;
    std::vector<Eigen::Vector3d> support_points;
};

struct ConvexBodyDescription {
    std::string name;
    std::string geometry_type;
    Pose pose;  // world
    Eigen::Vector3d scale = Eigen::Vector3d::Ones();
};

// Throws std::invalid_argument naming the instance on a missing template or
// invalid geometry.
std::vector<Obstacle> fromConvexBodies(const std::vector<GeometryTemplateDescription>& library,
                                       const std::vector<ConvexBodyDescription>& bodies);

}  // namespace xgc2_world_lidar

#endif  // XGC2_WORLD_LIDAR_CONVEX_BODY_CONVERSION_H
