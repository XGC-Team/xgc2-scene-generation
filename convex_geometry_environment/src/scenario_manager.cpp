/**
 * @file scenario_manager.cpp
 * @brief Convex geometry environment loading, obstacle registration, and publishing
 */

#include "convex_geometry_environment/scenario_manager.h"

#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "convex_geometry/GeometryLibrary.h"
#include "convex_geometry/GeometryTemplate.h"
#include "convex_geometry/ConvexBodyInstance.h"
#include "convex_geometry/ConvexBodyArray.h"
#include "convex_geometry_environment/scene/obstacles/obstacle_factory.h"

namespace convex_geometry_environment {

namespace {

double xmlToDouble(const XmlRpc::XmlRpcValue& value) {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        return static_cast<int>(value);
    }
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        return static_cast<double>(value);
    }
    throw std::runtime_error("Expected numeric XML-RPC value");
}

int xmlToInt(const XmlRpc::XmlRpcValue& value) {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        return static_cast<int>(value);
    }
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        return static_cast<int>(static_cast<double>(value));
    }
    throw std::runtime_error("Expected integer XML-RPC value");
}

std::string memberString(XmlRpc::XmlRpcValue& xml,
                         const std::string& key,
                         const std::string& default_value) {
    if (!xml.hasMember(key)) {
        return default_value;
    }
    if (xml[key].getType() != XmlRpc::XmlRpcValue::TypeString) {
        throw std::runtime_error("Field '" + key + "' must be a string");
    }
    return static_cast<std::string>(xml[key]);
}

double memberDouble(XmlRpc::XmlRpcValue& xml,
                    const std::string& key,
                    double default_value) {
    if (!xml.hasMember(key)) {
        return default_value;
    }
    return xmlToDouble(xml[key]);
}

int memberInt(XmlRpc::XmlRpcValue& xml,
              const std::string& key,
              int default_value) {
    if (!xml.hasMember(key)) {
        return default_value;
    }
    return xmlToInt(xml[key]);
}

Eigen::Vector3d memberVector3(XmlRpc::XmlRpcValue& xml,
                              const std::string& key) {
    if (!xml.hasMember(key) ||
        xml[key].getType() != XmlRpc::XmlRpcValue::TypeArray ||
        xml[key].size() != 3) {
        throw std::runtime_error("Field '" + key + "' must be [x, y, z]");
    }
    return Eigen::Vector3d(
        xmlToDouble(xml[key][0]),
        xmlToDouble(xml[key][1]),
        xmlToDouble(xml[key][2]));
}

Eigen::Vector4d memberColor(XmlRpc::XmlRpcValue& xml,
                            const std::string& key,
                            const Eigen::Vector4d& default_value) {
    if (!xml.hasMember(key)) {
        return default_value;
    }
    if (xml[key].getType() != XmlRpc::XmlRpcValue::TypeArray ||
        xml[key].size() != 4) {
        throw std::runtime_error("Field '" + key + "' must be [r, g, b, a]");
    }
    return Eigen::Vector4d(
        xmlToDouble(xml[key][0]),
        xmlToDouble(xml[key][1]),
        xmlToDouble(xml[key][2]),
        xmlToDouble(xml[key][3]));
}

Eigen::Quaterniond yawQuaternion(double yaw) {
    const double half_yaw = 0.5 * yaw;
    return Eigen::Quaterniond(
        std::cos(half_yaw), 0.0, 0.0, std::sin(half_yaw));
}

std::string makeFrameId(int id) {
    return "obstacle_" + std::to_string(id);
}

}  // namespace

ScenarioManager::ScenarioManager(ros::NodeHandle& nh) : nh_(nh) {}

bool ScenarioManager::loadAndInitialize() {
    try {
        ROS_INFO("Loading scenario configuration from ROS parameter server");

        if (!loadGlobalConfig()) {
            return false;
        }

        if (!loadHPolytopeLibrary()) {
            ROS_WARN("No H-polytope templates found in parameter server");
        }

        if (!loadVPolytopeLibrary()) {
            ROS_WARN("No V-polytope templates found in parameter server");
        }

        if (!loadObstacles()) {
            return false;
        }

        initializeMarkerPublishing();

        ROS_INFO("Successfully loaded and instantiated %lu obstacles", obstacles_.size());

        publishGeometryOnce();
        initializeDynamicObstaclePublishing();
        return true;

    } catch (const std::exception& e) {
        ROS_ERROR("Error loading configuration: %s", e.what());
        return false;
    }
}

void ScenarioManager::printSummary() const {
    ROS_INFO("=== Scenario Summary ===");
    ROS_INFO("World frame: %s", world_frame_id_.c_str());
    ROS_INFO("Publish rate: %.1f Hz", publish_rate_);
    ROS_INFO("TF publish rate: %.1f Hz", tf_publish_rate_);
    ROS_INFO("Total obstacles: %lu", obstacles_.size());

    for (const auto& obs : obstacles_) {
        ROS_INFO("  - [%d] %s (Type: %s)",
                 obs->getId(),
                 obs->getName().c_str(),
                 obs->getType().c_str());
    }
    ROS_INFO("========================\n");
}

void ScenarioManager::initializeMarkerPublishing() {
    marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/convex_geometry_environment/markers", 10);

    if (publish_rate_ > 0) {
        marker_timer_ = nh_.createTimer(
            ros::Duration(1.0 / publish_rate_),
            &ScenarioManager::publishMarkers, this);
    }
}

void ScenarioManager::initializeDynamicObstaclePublishing() {
    dynamic_body_instances_pub_ = nh_.advertise<convex_geometry::ConvexBodyArray>(
        "/convex_geometry/dynamic_body_instances", 10, true);

    const bool has_dynamic = countDynamicObstacles() > 0;

    ros::Duration(0.5).sleep();
    publishDynamicBodyInstances(ros::TimerEvent());
    ROS_INFO("Published initial dynamic body instances (dynamic obstacle count: %d)",
             has_dynamic ? countDynamicObstacles() : 0);

    if (has_dynamic) {
        dynamic_instance_timer_ = nh_.createTimer(
            ros::Duration(1.0 / 50.0),
            &ScenarioManager::publishDynamicBodyInstances, this);
        ROS_INFO("Dynamic body instance publisher started at 50 Hz");
    } else {
        ROS_INFO("No dynamic obstacles found, published empty array (latched)");
    }
}

void ScenarioManager::publishMarkers(const ros::TimerEvent&) {
    visualization_msgs::MarkerArray marker_array;

    for (const auto& obstacle : obstacles_) {
        auto markers = obstacle->getMarkers();
        marker_array.markers.insert(
            marker_array.markers.end(),
            markers.begin(),
            markers.end());
    }

    marker_array_pub_.publish(marker_array);
}

bool ScenarioManager::loadGlobalConfig() {
    nh_.param<std::string>("global/world_frame_id", world_frame_id_, "world");
    nh_.param<double>("global/publish_rate", publish_rate_, 20.0);
    nh_.param<double>("global/tf_publish_rate", tf_publish_rate_, 50.0);
    nh_.param<double>("global/velocity_update_rate", velocity_update_rate_, 100.0);
    nh_.param<double>("global/velocity_timeout", velocity_timeout_, 1.0);
    nh_.param<std::string>("global/h_polytope_library", h_polytope_library_path_, "");
    nh_.param<int>("global/resolution", global_resolution_, 50);

    return true;
}

bool ScenarioManager::loadHPolytopeLibrary() {
    if (!nh_.hasParam("templates")) {
        ROS_WARN("No H-polytope library found in parameter server");
        return false;
    }

    XmlRpc::XmlRpcValue templates_xml;
    if (!nh_.getParam("templates", templates_xml)) {
        return false;
    }

    if (templates_xml.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("H-polytope library templates must be a struct/dict");
        return false;
    }

    for (auto it = templates_xml.begin(); it != templates_xml.end(); ++it) {
        h_polytope_templates_[it->first] = it->second;
    }

    ROS_INFO("Loaded %lu H-polytope templates", h_polytope_templates_.size());
    return true;
}

bool ScenarioManager::loadVPolytopeLibrary() {
    if (!nh_.hasParam("v_templates")) {
        ROS_WARN("No V-polytope library found in parameter server");
        return false;
    }

    XmlRpc::XmlRpcValue templates_xml;
    if (!nh_.getParam("v_templates", templates_xml)) {
        return false;
    }

    if (templates_xml.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("V-polytope library templates must be a struct/dict");
        return false;
    }

    for (auto it = templates_xml.begin(); it != templates_xml.end(); ++it) {
        v_polytope_templates_[it->first] = it->second;
    }

    ROS_INFO("Loaded %lu V-polytope templates", v_polytope_templates_.size());
    return true;
}

bool ScenarioManager::loadObstacles() {
    if (!nh_.hasParam("obstacles")) {
        ROS_ERROR("No 'obstacles' parameter found");
        return false;
    }

    XmlRpc::XmlRpcValue obstacles_xml;
    if (!nh_.getParam("obstacles", obstacles_xml)) {
        return false;
    }

    if (obstacles_xml.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("'obstacles' parameter must be an array");
        return false;
    }

    std::set<int> obstacle_ids;
    auto append_obstacle = [&](const ObstacleConfig& config,
                               const std::string& source_name) -> bool {
        if (!obstacle_ids.insert(config.id).second) {
            ROS_ERROR("Duplicate obstacle id %d while loading %s",
                      config.id,
                      source_name.c_str());
            return false;
        }

        ObstaclePtr obstacle = ObstacleFactory::createObstacle(
            nh_, config, tf_publish_rate_, velocity_update_rate_, velocity_timeout_);

        if (!obstacle) {
            ROS_ERROR("Failed to create obstacle id=%d from %s",
                      config.id,
                      source_name.c_str());
            return false;
        }

        obstacles_.push_back(obstacle);
        return true;
    };

    for (int i = 0; i < obstacles_xml.size(); ++i) {
        try {
            ObstacleConfig config = parseObstacleConfig(obstacles_xml[i]);
            if (!append_obstacle(config, "obstacles[" + std::to_string(i) + "]")) {
                return false;
            }

        } catch (const std::exception& e) {
            ROS_ERROR("Failed to load obstacle %d: %s", i, e.what());
            return false;
        }
    }

    if (nh_.hasParam("obstacle_groups")) {
        XmlRpc::XmlRpcValue groups_xml;
        if (!nh_.getParam("obstacle_groups", groups_xml)) {
            return false;
        }
        if (groups_xml.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("'obstacle_groups' parameter must be an array");
            return false;
        }

        for (int i = 0; i < groups_xml.size(); ++i) {
            try {
                const std::vector<ObstacleConfig> expanded_configs =
                    expandObstacleGroup(groups_xml[i]);
                for (const auto& config : expanded_configs) {
                    if (!append_obstacle(config,
                                         "obstacle_groups[" + std::to_string(i) + "]")) {
                        return false;
                    }
                }
                ROS_INFO("Expanded obstacle group %d into %lu primitive obstacles",
                         i,
                         expanded_configs.size());
            } catch (const std::exception& e) {
                ROS_ERROR("Failed to expand obstacle group %d: %s", i, e.what());
                return false;
            }
        }
    }

    return true;
}

ObstacleConfig ScenarioManager::parseObstacleConfig(XmlRpc::XmlRpcValue& obs_xml) {
    ObstacleConfig config;

    config.id = static_cast<int>(obs_xml["id"]);
    config.name = static_cast<std::string>(obs_xml["name"]);
    config.type = static_cast<std::string>(obs_xml["type"]);
    config.motion_type = static_cast<std::string>(obs_xml["motion_type"]);

    config.frame_id = static_cast<std::string>(obs_xml["frame"]["id"]);
    config.parent_frame_id = world_frame_id_;

    XmlRpc::XmlRpcValue& pos = obs_xml["pose"]["position"];
    config.position = Eigen::Vector3d(
        static_cast<double>(pos[0]),
        static_cast<double>(pos[1]),
        static_cast<double>(pos[2]));

    XmlRpc::XmlRpcValue& ori = obs_xml["pose"]["orientation"];
    config.orientation = Eigen::Quaterniond(
        static_cast<double>(ori[3]),
        static_cast<double>(ori[0]),
        static_cast<double>(ori[1]),
        static_cast<double>(ori[2]));

    if (obs_xml.hasMember("geometry") && obs_xml["geometry"].hasMember("scale")) {
        XmlRpc::XmlRpcValue& scale = obs_xml["geometry"]["scale"];
        config.scale = Eigen::Vector3d(
            static_cast<double>(scale[0]),
            static_cast<double>(scale[1]),
            static_cast<double>(scale[2]));
    } else {
        config.scale = Eigen::Vector3d(1.0, 1.0, 1.0);
    }

    config.velocity_obstacle_enabled = static_cast<bool>(obs_xml["velocity_obstacle"]["enabled"]);
    if (config.velocity_obstacle_enabled && obs_xml["velocity_obstacle"].hasMember("topic")) {
        config.velocity_topic = static_cast<std::string>(obs_xml["velocity_obstacle"]["topic"]);
    }

    if (config.velocity_obstacle_enabled && obs_xml["velocity_obstacle"].hasMember("initial_velocity")) {
        XmlRpc::XmlRpcValue& init_vel = obs_xml["velocity_obstacle"]["initial_velocity"];
        if (init_vel.getType() == XmlRpc::XmlRpcValue::TypeArray && init_vel.size() == 3) {
            config.initial_velocity = Eigen::Vector3d(
                static_cast<double>(init_vel[0]),
                static_cast<double>(init_vel[1]),
                static_cast<double>(init_vel[2]));
        } else {
            ROS_WARN("Invalid initial_velocity format for obstacle %d, using zero", config.id);
            config.initial_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
        }
    } else {
        config.initial_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
    }

    if (obs_xml.hasMember("visualization") && obs_xml["visualization"].hasMember("color")) {
        XmlRpc::XmlRpcValue& color = obs_xml["visualization"]["color"];
        if (color.getType() == XmlRpc::XmlRpcValue::TypeArray && color.size() == 4) {
            config.color = Eigen::Vector4d(
                static_cast<double>(color[0]),
                static_cast<double>(color[1]),
                static_cast<double>(color[2]),
                static_cast<double>(color[3]));
        } else {
            ROS_WARN("Invalid color format for obstacle %d, using default green", config.id);
            config.color = Eigen::Vector4d(0.2, 0.8, 0.2, 0.8);
        }
    } else {
        config.color = Eigen::Vector4d(0.2, 0.8, 0.2, 0.8);
    }

    if (config.type == "h_polytope") {
        if (obs_xml["geometry"].hasMember("h_polytope_type")) {
            expandHPolytopeTemplate(static_cast<std::string>(obs_xml["geometry"]["h_polytope_type"]), config);
        } else if (obs_xml["geometry"].hasMember("h_matrix")) {
            parseInlineHMatrix(obs_xml["geometry"]["h_matrix"], config);
        } else {
            throw std::runtime_error("H-polytope obstacle must have either 'h_polytope_type' or 'h_matrix'");
        }
    }

    if (config.type == "v_polytope") {
        if (obs_xml["geometry"].hasMember("v_polytope_type")) {
            expandVPolytopeTemplate(static_cast<std::string>(obs_xml["geometry"]["v_polytope_type"]), config);
        } else if (obs_xml["geometry"].hasMember("vertices")) {
            parseInlineVertices(obs_xml["geometry"]["vertices"], config);
        } else {
            throw std::runtime_error("V-polytope obstacle must have either 'v_polytope_type' or 'vertices'");
        }
    }

    return config;
}

std::vector<ObstacleConfig> ScenarioManager::expandObstacleGroup(
    XmlRpc::XmlRpcValue& group_xml) {
    if (group_xml.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::runtime_error("obstacle group must be a dictionary");
    }
    if (!group_xml.hasMember("template")) {
        throw std::runtime_error("obstacle group missing 'template'");
    }

    const std::string template_name = static_cast<std::string>(group_xml["template"]);
    if (template_name == "cube_frame") {
        return expandCubeFrameGroup(group_xml);
    }

    throw std::runtime_error("Unsupported obstacle group template: " + template_name);
}

std::vector<ObstacleConfig> ScenarioManager::expandCubeFrameGroup(
    XmlRpc::XmlRpcValue& group_xml) {
    const int start_id = memberInt(group_xml, "start_id", -1);
    if (start_id < 0) {
        throw std::runtime_error("cube_frame group requires nonnegative start_id");
    }

    const std::string name_prefix =
        memberString(group_xml, "name_prefix", "cube_frame");
    const std::string motion_type =
        memberString(group_xml, "motion_type", "static");
    const Eigen::Vector3d center = memberVector3(group_xml, "center");
    const double yaw = memberDouble(group_xml, "yaw", 0.0);
    const double width = memberDouble(group_xml, "width", 2.0);
    const double height = memberDouble(group_xml, "height", 2.0);
    const double rod_thickness =
        memberDouble(group_xml, "rod_thickness", 0.25);
    const Eigen::Vector4d color = memberColor(
        group_xml, "color", Eigen::Vector4d(0.50, 0.50, 0.50, 1.00));

    if (width <= 0.0 || height <= 0.0 || rod_thickness <= 0.0) {
        throw std::runtime_error(
            "cube_frame width, height, and rod_thickness must be positive");
    }

    const Eigen::Quaterniond orientation = yawQuaternion(yaw);
    const double half_width = 0.5 * width;
    const double half_height = 0.5 * height;
    const Eigen::Vector3d lateral(std::cos(yaw), std::sin(yaw), 0.0);

    auto make_cube = [&](int id,
                         const std::string& suffix,
                         const Eigen::Vector3d& position,
                         const Eigen::Vector3d& scale) {
        ObstacleConfig config;
        config.id = id;
        config.name = name_prefix + "_" + suffix;
        config.type = "cube";
        config.motion_type = motion_type;
        config.frame_id = makeFrameId(id);
        config.parent_frame_id = world_frame_id_;
        config.position = position;
        config.orientation = orientation;
        config.scale = scale;
        config.color = color;
        config.velocity_obstacle_enabled = false;
        config.initial_velocity = Eigen::Vector3d::Zero();
        return config;
    };

    std::vector<ObstacleConfig> configs;
    configs.reserve(4);
    configs.push_back(make_cube(
        start_id,
        "left_side",
        center - half_width * lateral,
        Eigen::Vector3d(rod_thickness, rod_thickness, height)));
    configs.push_back(make_cube(
        start_id + 1,
        "right_side",
        center + half_width * lateral,
        Eigen::Vector3d(rod_thickness, rod_thickness, height)));
    configs.push_back(make_cube(
        start_id + 2,
        "top_edge",
        center + Eigen::Vector3d(0.0, 0.0, half_height),
        Eigen::Vector3d(width + rod_thickness, rod_thickness, rod_thickness)));
    configs.push_back(make_cube(
        start_id + 3,
        "bottom_edge",
        center - Eigen::Vector3d(0.0, 0.0, half_height),
        Eigen::Vector3d(width + rod_thickness, rod_thickness, rod_thickness)));
    return configs;
}

void ScenarioManager::expandHPolytopeTemplate(const std::string& template_name, ObstacleConfig& config) {
    if (h_polytope_templates_.find(template_name) == h_polytope_templates_.end()) {
        throw std::runtime_error("H-polytope template not found: " + template_name);
    }

    XmlRpc::XmlRpcValue& tmpl = h_polytope_templates_[template_name];
    config.geometry_params_string["h_polytope_template"] = template_name;

    if (tmpl.hasMember("epsilon")) {
        config.geometry_params_double["epsilon"] = static_cast<double>(tmpl["epsilon"]);
    }

    parseInlineHMatrix(tmpl["h_matrix"], config);
    ROS_INFO("Expanded H-polytope template: %s", template_name.c_str());
}

void ScenarioManager::parseInlineHMatrix(XmlRpc::XmlRpcValue& h_matrix_xml, ObstacleConfig& config) {
    if (h_matrix_xml.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        throw std::runtime_error("h_matrix must be an array");
    }

    const int rows = h_matrix_xml.size();
    std::vector<double> h_matrix_flat;

    for (int i = 0; i < rows; ++i) {
        XmlRpc::XmlRpcValue& row = h_matrix_xml[i];
        if (row.getType() != XmlRpc::XmlRpcValue::TypeArray || row.size() != 4) {
            throw std::runtime_error("Each h_matrix row must be an array of 4 elements");
        }

        for (int j = 0; j < 4; ++j) {
            h_matrix_flat.push_back(static_cast<double>(row[j]));
        }
    }

    config.geometry_params_vec["h_matrix_flat"] = h_matrix_flat;
    config.geometry_params_int["h_matrix_rows"] = rows;
}

void ScenarioManager::expandVPolytopeTemplate(const std::string& template_name, ObstacleConfig& config) {
    if (v_polytope_templates_.find(template_name) == v_polytope_templates_.end()) {
        throw std::runtime_error("V-polytope template not found: " + template_name);
    }

    XmlRpc::XmlRpcValue& tmpl = v_polytope_templates_[template_name];
    config.geometry_params_string["v_polytope_template"] = template_name;

    if (tmpl.hasMember("epsilon")) {
        config.geometry_params_double["epsilon"] = static_cast<double>(tmpl["epsilon"]);
    }

    parseInlineVertices(tmpl["vertices"], config);
    ROS_INFO("Expanded V-polytope template: %s", template_name.c_str());
}

void ScenarioManager::parseInlineVertices(XmlRpc::XmlRpcValue& vertices_xml, ObstacleConfig& config) {
    if (vertices_xml.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        throw std::runtime_error("vertices must be an array");
    }

    const int num_vertices = vertices_xml.size();
    std::vector<double> vertices_flat;

    for (int i = 0; i < num_vertices; ++i) {
        XmlRpc::XmlRpcValue& vertex = vertices_xml[i];
        if (vertex.getType() != XmlRpc::XmlRpcValue::TypeArray || vertex.size() != 3) {
            throw std::runtime_error("Each vertex must be an array of 3 elements [x, y, z]");
        }

        vertices_flat.push_back(static_cast<double>(vertex[0]));
        vertices_flat.push_back(static_cast<double>(vertex[1]));
        vertices_flat.push_back(static_cast<double>(vertex[2]));
    }

    config.geometry_params_vec["vertices_flat"] = vertices_flat;
    ROS_INFO("Parsed %d vertices for V-polytope", num_vertices);
}

int ScenarioManager::countDynamicObstacles() const {
    int count = 0;
    for (const auto& obs : obstacles_) {
        if (obs->getConfig().motion_type == "dynamic") {
            ++count;
        }
    }
    return count;
}

convex_geometry::GeometryLibrary ScenarioManager::buildGeometryLibrary() {
    convex_geometry::GeometryLibrary library;
    library.header.stamp = ros::Time::now();
    library.header.frame_id = world_frame_id_;

    std::set<std::string> unique_types;
    for (const auto& obstacle : obstacles_) {
        const std::string geometry_type = obstacle->getType();
        if (!unique_types.insert(geometry_type).second) {
            continue;
        }

        convex_geometry::GeometryTemplate tmpl = obstacle->buildGeometryTemplate(global_resolution_);
        library.templates.push_back(tmpl);

        ROS_INFO("Generated geometry template: type=%s, resolution=%d, support_points=%lu",
                 geometry_type.c_str(), tmpl.resolution, tmpl.support_points.size());
    }

    ROS_INFO("Built geometry library with %lu templates", library.templates.size());
    return library;
}

convex_geometry::ConvexBodyArray ScenarioManager::buildConvexBodyArray() {
    convex_geometry::ConvexBodyArray array;
    array.header.stamp = ros::Time::now();
    array.header.frame_id = world_frame_id_;

    for (const auto& obstacle : obstacles_) {
        const auto& config = obstacle->getConfig();
        if (config.motion_type == "dynamic") {
            ROS_INFO("Skipping dynamic obstacle: id=%d, name=%s, motion_type=%s",
                     config.id, config.name.c_str(), config.motion_type.c_str());
            continue;
        }

        convex_geometry::ConvexBodyInstance instance = obstacle->buildConvexBodyInstance(true);
        array.instances.push_back(instance);

        ROS_INFO("Added static instance: id=%d, name=%s, type=%s, motion_type=%s, scale=[%.2f,%.2f,%.2f]",
                 instance.id, instance.name.c_str(), instance.geometry_type.c_str(),
                 config.motion_type.c_str(),
                 instance.scale.x, instance.scale.y, instance.scale.z);
    }

    ROS_INFO("========== Static Obstacle Summary ==========");
    ROS_INFO("Total obstacles processed: %lu", obstacles_.size());
    ROS_INFO("Static obstacles in array: %lu", array.instances.size());
    ROS_INFO("Dynamic obstacles skipped: %lu", obstacles_.size() - array.instances.size());
    ROS_INFO("============================================");
    return array;
}

void ScenarioManager::publishGeometryOnce() {
    ROS_INFO("Publishing geometry library and static body instances...");

    geometry_library_pub_ = nh_.advertise<convex_geometry::GeometryLibrary>(
        "/convex_geometry/geometry_library", 1, true);

    static_body_instances_pub_ = nh_.advertise<convex_geometry::ConvexBodyArray>(
        "/convex_geometry/static_body_instances", 1, true);

    ros::Duration(0.5).sleep();

    convex_geometry::GeometryLibrary library = buildGeometryLibrary();
    geometry_library_pub_.publish(library);

    convex_geometry::ConvexBodyArray instances = buildConvexBodyArray();
    static_body_instances_pub_.publish(instances);

    ROS_INFO("Geometry information published successfully (latched)");
    ROS_INFO("  - Geometry templates: %lu types", library.templates.size());
    ROS_INFO("  - Static body instances: %lu obstacles", instances.instances.size());
}

void ScenarioManager::publishDynamicBodyInstances(const ros::TimerEvent&) {
    convex_geometry::ConvexBodyArray array;
    array.header.stamp = ros::Time::now();
    array.header.frame_id = world_frame_id_;

    for (const auto& obstacle : obstacles_) {
        const auto& config = obstacle->getConfig();
        if (config.motion_type != "dynamic") {
            continue;
        }

        array.instances.push_back(obstacle->buildConvexBodyInstance(false));
    }

    dynamic_body_instances_pub_.publish(array);
}

}  // namespace convex_geometry_environment
