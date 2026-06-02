/**
 * @file primitive_obstacles.h
 * @brief Primitive obstacle implementations
 */

#ifndef CONVEX_GEOMETRY_ENVIRONMENT_SCENE_OBSTACLES_PRIMITIVE_OBSTACLES_H
#define CONVEX_GEOMETRY_ENVIRONMENT_SCENE_OBSTACLES_PRIMITIVE_OBSTACLES_H

#include "convex_geometry_environment/scene/obstacles/obstacle_base.h"

namespace convex_geometry_environment {

class SphereObstacle : public ObstacleBase {
public:
    SphereObstacle(ros::NodeHandle& nh,
                   const ObstacleConfig& config,
                   double tf_rate,
                   double velocity_update_rate,
                   double velocity_timeout);

protected:
    void generateGeometry() override;
    convex_geometry::GeometryTemplate buildGeometryTemplate(int resolution) const override;
    convex_geometry::ConvexBody buildOccupiedBody() const override;
    visualization_msgs::Marker createMarkerMessage() override;

private:
    double radius_;
    int resolution_;
};

class CylinderObstacle : public ObstacleBase {
public:
    CylinderObstacle(ros::NodeHandle& nh,
                     const ObstacleConfig& config,
                     double tf_rate,
                     double velocity_update_rate,
                     double velocity_timeout);

protected:
    void generateGeometry() override;
    convex_geometry::GeometryTemplate buildGeometryTemplate(int resolution) const override;
    convex_geometry::ConvexBody buildOccupiedBody() const override;
    visualization_msgs::Marker createMarkerMessage() override;

private:
    double radius_;
    double height_;
    int resolution_;
};

class CapsuleObstacle : public ObstacleBase {
public:
    CapsuleObstacle(ros::NodeHandle& nh,
                    const ObstacleConfig& config,
                    double tf_rate,
                    double velocity_update_rate,
                    double velocity_timeout);

protected:
    void generateGeometry() override;
    convex_geometry::GeometryTemplate buildGeometryTemplate(int resolution) const override;
    convex_geometry::ConvexBody buildOccupiedBody() const override;
    visualization_msgs::Marker createMarkerMessage() override;
    std::vector<visualization_msgs::Marker> getMarkers() override;

private:
    visualization_msgs::Marker createCapMarker(int cap_index, double z_offset) const;

    double radius_;
    double length_;
};

class CubeObstacle : public ObstacleBase {
public:
    CubeObstacle(ros::NodeHandle& nh,
                 const ObstacleConfig& config,
                 double tf_rate,
                 double velocity_update_rate,
                 double velocity_timeout);

protected:
    void generateGeometry() override;
    convex_geometry::GeometryTemplate buildGeometryTemplate(int resolution) const override;
    convex_geometry::ConvexBody buildOccupiedBody() const override;
    visualization_msgs::Marker createMarkerMessage() override;

private:
    Eigen::Vector3d size_;
};

}  // namespace convex_geometry_environment

#endif  // CONVEX_GEOMETRY_ENVIRONMENT_SCENE_OBSTACLES_PRIMITIVE_OBSTACLES_H
