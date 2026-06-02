/**
 * @file scenario_manager.h
 * @brief Scenario manager for convex-geometry-environment loading and publishing
 */

#ifndef CONVEX_GEOMETRY_ENVIRONMENT_SCENARIO_MANAGER_H
#define CONVEX_GEOMETRY_ENVIRONMENT_SCENARIO_MANAGER_H

#include <ros/ros.h>
#include <XmlRpcValue.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "convex_geometry/GeometryLibrary.h"
#include "convex_geometry/ConvexBodyArray.h"
#include "convex_geometry_environment/scene/obstacles/obstacle_base.h"

namespace convex_geometry_environment {

class ScenarioManager {
public:
    explicit ScenarioManager(ros::NodeHandle& nh);

    bool loadAndInitialize();
    void printSummary() const;

private:
    void initializeMarkerPublishing();
    void initializeDynamicObstaclePublishing();
    void publishMarkers(const ros::TimerEvent&);

    bool loadGlobalConfig();
    bool loadHPolytopeLibrary();
    bool loadVPolytopeLibrary();
    bool loadObstacles();

    ObstacleConfig parseObstacleConfig(XmlRpc::XmlRpcValue& obs_xml);
    std::vector<ObstacleConfig> expandObstacleGroup(XmlRpc::XmlRpcValue& group_xml);
    std::vector<ObstacleConfig> expandCubeFrameGroup(XmlRpc::XmlRpcValue& group_xml);
    void expandHPolytopeTemplate(const std::string& template_name, ObstacleConfig& config);
    void parseInlineHMatrix(XmlRpc::XmlRpcValue& h_matrix_xml, ObstacleConfig& config);
    void expandVPolytopeTemplate(const std::string& template_name, ObstacleConfig& config);
    void parseInlineVertices(XmlRpc::XmlRpcValue& vertices_xml, ObstacleConfig& config);

    int countDynamicObstacles() const;
    convex_geometry::GeometryLibrary buildGeometryLibrary();
    convex_geometry::ConvexBodyArray buildConvexBodyArray();
    void publishGeometryOnce();
    void publishDynamicBodyInstances(const ros::TimerEvent&);

    ros::NodeHandle nh_;

    ros::Publisher marker_array_pub_;
    ros::Timer marker_timer_;

    ros::Publisher geometry_library_pub_;
    ros::Publisher static_body_instances_pub_;

    ros::Publisher dynamic_body_instances_pub_;
    ros::Timer dynamic_instance_timer_;

    std::string world_frame_id_;
    double publish_rate_ = 0.0;
    double tf_publish_rate_ = 0.0;
    double velocity_update_rate_ = 0.0;
    double velocity_timeout_ = 0.0;
    std::string h_polytope_library_path_;
    int global_resolution_ = 0;

    std::map<std::string, XmlRpc::XmlRpcValue> h_polytope_templates_;
    std::map<std::string, XmlRpc::XmlRpcValue> v_polytope_templates_;
    std::vector<ObstaclePtr> obstacles_;
};

}  // namespace convex_geometry_environment

#endif  // CONVEX_GEOMETRY_ENVIRONMENT_SCENARIO_MANAGER_H
