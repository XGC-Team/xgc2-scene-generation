/**
 * @file polytope_obstacles.h
 * @brief Polytope obstacle implementations
 */

#ifndef CONVEX_GEOMETRY_ENVIRONMENT_SCENE_OBSTACLES_POLYTOPE_OBSTACLES_H
#define CONVEX_GEOMETRY_ENVIRONMENT_SCENE_OBSTACLES_POLYTOPE_OBSTACLES_H

#include "convex_geometry_environment/scene/obstacles/obstacle_base.h"

namespace convex_geometry_environment {

class HPolytopeObstacle : public ObstacleBase {
public:
    HPolytopeObstacle(ros::NodeHandle& nh,
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
    std::string getType() const override;

private:
    void computeVerticesFromHMatrix();
    visualization_msgs::Marker createEdgeMarker();

    Eigen::MatrixXd h_matrix_;
    double epsilon_;
    std::vector<Eigen::Vector3d> support_vertices_unit_;
};

class VPolytopeObstacle : public ObstacleBase {
public:
    VPolytopeObstacle(ros::NodeHandle& nh,
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
    std::string getType() const override;

private:
    void generateConvexHull();
    visualization_msgs::Marker createEdgeMarker();

    std::vector<Eigen::Vector3d> vertices_original_;
    double epsilon_;
};

}  // namespace convex_geometry_environment

#endif  // CONVEX_GEOMETRY_ENVIRONMENT_SCENE_OBSTACLES_POLYTOPE_OBSTACLES_H
