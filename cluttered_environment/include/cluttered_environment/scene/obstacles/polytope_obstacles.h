/**
 * @file polytope_obstacles.h
 * @brief Polytope obstacle implementations
 */

#ifndef CLUTTERED_ENVIRONMENT_SCENE_OBSTACLES_POLYTOPE_OBSTACLES_H
#define CLUTTERED_ENVIRONMENT_SCENE_OBSTACLES_POLYTOPE_OBSTACLES_H

#include "cluttered_environment/scene/obstacles/obstacle_base.h"

namespace cluttered_environment {

class HPolytopeObstacle : public ObstacleBase {
public:
    HPolytopeObstacle(ros::NodeHandle& nh,
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
    xgc2_geometry_msgs::GeometryTemplate buildGeometryTemplate(int resolution) const override;
    xgc2_geometry::ConvexBody buildOccupiedBody() const override;
    visualization_msgs::Marker createMarkerMessage() override;
    std::vector<visualization_msgs::Marker> getMarkers() override;
    std::string getType() const override;

private:
    void generateConvexHull();
    visualization_msgs::Marker createEdgeMarker();

    std::vector<Eigen::Vector3d> vertices_original_;
    double epsilon_;
};

} // namespace cluttered_environment

#endif // CLUTTERED_ENVIRONMENT_SCENE_OBSTACLES_POLYTOPE_OBSTACLES_H
