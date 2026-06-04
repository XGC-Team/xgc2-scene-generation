/**
 * @file primitive_obstacles.h
 * @brief Primitive obstacle implementations
 */

#ifndef CLUTTERED_ENVIRONMENT_SCENE_OBSTACLES_PRIMITIVE_OBSTACLES_H
#define CLUTTERED_ENVIRONMENT_SCENE_OBSTACLES_PRIMITIVE_OBSTACLES_H

#include "cluttered_environment/scene/obstacles/obstacle_base.h"

namespace cluttered_environment {

class SphereObstacle : public ObstacleBase {
public:
    SphereObstacle(ros::NodeHandle& nh,
                   const ObstacleConfig& config,
                   double tf_rate,
                   double velocity_update_rate,
                   double velocity_timeout);

protected:
    void generateGeometry() override;
    xgc2_geometry_msgs::GeometryTemplate buildGeometryTemplate(int resolution) const override;
    xgc2_geometry::ConvexBody buildOccupiedBody() const override;
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
    xgc2_geometry_msgs::GeometryTemplate buildGeometryTemplate(int resolution) const override;
    xgc2_geometry::ConvexBody buildOccupiedBody() const override;
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
    xgc2_geometry_msgs::GeometryTemplate buildGeometryTemplate(int resolution) const override;
    xgc2_geometry::ConvexBody buildOccupiedBody() const override;
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
    xgc2_geometry_msgs::GeometryTemplate buildGeometryTemplate(int resolution) const override;
    xgc2_geometry::ConvexBody buildOccupiedBody() const override;
    visualization_msgs::Marker createMarkerMessage() override;

private:
    Eigen::Vector3d size_;
};

} // namespace cluttered_environment

#endif // CLUTTERED_ENVIRONMENT_SCENE_OBSTACLES_PRIMITIVE_OBSTACLES_H
