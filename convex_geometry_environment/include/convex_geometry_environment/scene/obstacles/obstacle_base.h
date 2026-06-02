/**
 * @file obstacle_base.h
 * @brief Base class for all obstacle types
 */

#ifndef CONVEX_GEOMETRY_ENVIRONMENT_SCENE_OBSTACLES_OBSTACLE_BASE_H
#define CONVEX_GEOMETRY_ENVIRONMENT_SCENE_OBSTACLES_OBSTACLE_BASE_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <Eigen/Dense>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "convex_geometry/GeometryTemplate.h"
#include "convex_geometry/ConvexBodyInstance.h"
#include "convex_geometry/occupied_sets/convex_body.h"

namespace convex_geometry_environment {

struct ObstacleConfig {
    int id;
    std::string name;
    std::string type;
    std::string motion_type;
    std::string frame_id;
    std::string parent_frame_id;

    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d scale;

    // Visualization color (RGBA, range [0.0, 1.0])
    Eigen::Vector4d color;

    bool velocity_obstacle_enabled;
    std::string velocity_topic;
    Eigen::Vector3d initial_velocity;

    // Geometry parameters (type-specific, stored as generic map)
    std::map<std::string, double> geometry_params_double;
    std::map<std::string, int> geometry_params_int;
    std::map<std::string, std::string> geometry_params_string;
    std::map<std::string, std::vector<double>> geometry_params_vec;
};

class ObstacleBase {
public:
    ObstacleBase(ros::NodeHandle& nh,
                 const ObstacleConfig& config,
                 double tf_rate,
                 double velocity_update_rate,
                 double velocity_timeout);

    virtual ~ObstacleBase() = default;

    virtual void initialize();

    int getId() const { return config_.id; }
    std::string getName() const { return config_.name; }
    virtual std::string getType() const { return config_.type; }

    const ObstacleConfig& getConfig() const {
        return config_;
    }

    const geometry_msgs::Pose& getCurrentPose() const {
        return current_pose_;
    }

    const geometry_msgs::Twist& getCurrentVelocity() const {
        return current_velocity_;
    }

    const std::vector<Eigen::Vector3d>& getVerticesLocal() const {
        return vertices_local_;
    }

    const convex_geometry::ConvexBody& occupiedBody() const {
        return occupied_body_;
    }

    virtual convex_geometry::GeometryTemplate buildGeometryTemplate(int resolution) const = 0;
    convex_geometry::ConvexBodyInstance buildConvexBodyInstance(bool is_static) const;
    virtual std::vector<visualization_msgs::Marker> getMarkers();

protected:
    virtual void generateGeometry() = 0;
    virtual convex_geometry::ConvexBody buildOccupiedBody() const = 0;

    virtual void publishTF();
    virtual void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
    virtual void updatePosition();

    void refreshOccupiedBody();
    bool isStaticObstacle() const;
    geometry_msgs::TransformStamped buildTransformStamped() const;

    Eigen::Vector3d currentPositionVector() const;
    Eigen::Quaterniond currentOrientationQuaternion() const;
    Eigen::Vector3d currentVelocityVector() const;

    virtual visualization_msgs::Marker createMarkerMessage() = 0;
    virtual visualization_msgs::Marker createVelocityMarker();

    // Configuration
    ObstacleConfig config_;

    // ROS handles
    ros::NodeHandle nh_;

    // Publishers and subscribers
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    ros::Subscriber velocity_sub_;

    // Timers
    ros::Timer tf_timer_;
    ros::Timer velocity_update_timer_;

    // Canonical occupied-set representation used by external consumers.
    convex_geometry::ConvexBody occupied_body_;

    // Visualization data (in local coordinate frame). These are RViz-side caches only.
    std::vector<Eigen::Vector3d> vertices_local_;
    std::vector<uint32_t> triangle_indices_;
    Eigen::Vector3d bbox_min_local_;
    Eigen::Vector3d bbox_max_local_;

    // Current velocity (for dynamic obstacles)
    geometry_msgs::Twist current_velocity_;

    // Velocity obstacle parameters
    ros::Time last_velocity_time_;
    double velocity_timeout_;
    double velocity_update_rate_;

    // Current pose (updated by velocity integration)
    geometry_msgs::Pose current_pose_;
};

using ObstaclePtr = std::shared_ptr<ObstacleBase>;

}  // namespace convex_geometry_environment

#endif  // CONVEX_GEOMETRY_ENVIRONMENT_SCENE_OBSTACLES_OBSTACLE_BASE_H
