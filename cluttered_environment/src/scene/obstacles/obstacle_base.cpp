/**
 * @file obstacle_base.cpp
 * @brief Implementation of ObstacleBase class
 */

#include "cluttered_environment/scene/obstacles/obstacle_base.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

namespace cluttered_environment {

ObstacleBase::ObstacleBase(ros::NodeHandle& nh,
                           const ObstacleConfig& config,
                           double tf_rate,
                           double velocity_update_rate,
                           double velocity_timeout)
    : nh_(nh), config_(config), velocity_timeout_(velocity_timeout),
      velocity_update_rate_(velocity_update_rate) {
    if (isStaticObstacle()) {
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();
    } else {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    }

    if (config_.velocity_obstacle_enabled && !config_.velocity_topic.empty()) {
        velocity_sub_ =
            nh_.subscribe(config_.velocity_topic, 1, &ObstacleBase::velocityCallback, this);

        ROS_INFO("Obstacle [%s]: Subscribed to velocity topic: %s",
                 config_.name.c_str(),
                 config_.velocity_topic.c_str());
    }

    current_velocity_ = geometry_msgs::Twist();
    if (config_.velocity_obstacle_enabled) {
        current_velocity_.linear.x = config_.initial_velocity.x();
        current_velocity_.linear.y = config_.initial_velocity.y();
        current_velocity_.linear.z = config_.initial_velocity.z();
        ROS_INFO("Obstacle [%s]: Initial velocity set to [%.2f, %.2f, %.2f] m/s",
                 config_.name.c_str(),
                 config_.initial_velocity.x(),
                 config_.initial_velocity.y(),
                 config_.initial_velocity.z());
    }

    current_pose_.position.x = config_.position.x();
    current_pose_.position.y = config_.position.y();
    current_pose_.position.z = config_.position.z();
    current_pose_.orientation.x = config_.orientation.x();
    current_pose_.orientation.y = config_.orientation.y();
    current_pose_.orientation.z = config_.orientation.z();
    current_pose_.orientation.w = config_.orientation.w();

    if (tf_rate > 0 && !isStaticObstacle()) {
        tf_timer_ = nh_.createTimer(ros::Duration(1.0 / tf_rate),
                                    [this](const ros::TimerEvent&) { this->publishTF(); });
    }

    if (velocity_update_rate > 0 && config_.velocity_obstacle_enabled) {
        velocity_update_timer_ =
            nh_.createTimer(ros::Duration(1.0 / velocity_update_rate),
                            [this](const ros::TimerEvent&) { this->updatePosition(); });
    }

    ROS_INFO("Obstacle [%s] (ID: %d, Type: %s) initialized",
             config_.name.c_str(),
             config_.id,
             config_.type.c_str());

    if (isStaticObstacle()) {
        publishTF();
    }
}

void ObstacleBase::initialize() {
    generateGeometry();
    refreshOccupiedBody();

    ROS_INFO("Obstacle [%s]: Geometry initialized (type: %s)",
             config_.name.c_str(),
             config_.type.c_str());
}

void ObstacleBase::publishTF() {
    const geometry_msgs::TransformStamped transform_stamped = buildTransformStamped();

    if (isStaticObstacle()) {
        tf_static_broadcaster_->sendTransform(transform_stamped);
        return;
    }

    tf_broadcaster_->sendTransform(transform_stamped);
}

bool ObstacleBase::isStaticObstacle() const {
    return config_.motion_type != "dynamic";
}

geometry_msgs::TransformStamped ObstacleBase::buildTransformStamped() const {
    geometry_msgs::TransformStamped transform_stamped;

    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = config_.parent_frame_id;
    transform_stamped.child_frame_id = config_.frame_id;

    transform_stamped.transform.translation.x = current_pose_.position.x;
    transform_stamped.transform.translation.y = current_pose_.position.y;
    transform_stamped.transform.translation.z = current_pose_.position.z;
    transform_stamped.transform.rotation.x = current_pose_.orientation.x;
    transform_stamped.transform.rotation.y = current_pose_.orientation.y;
    transform_stamped.transform.rotation.z = current_pose_.orientation.z;
    transform_stamped.transform.rotation.w = current_pose_.orientation.w;

    return transform_stamped;
}

void ObstacleBase::velocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    current_velocity_ = *msg;
    last_velocity_time_ = ros::Time::now();
    occupied_body_.velocity = currentVelocityVector();
}

void ObstacleBase::updatePosition() {
    if (!config_.velocity_obstacle_enabled) {
        return;
    }

    const double dt = 1.0 / velocity_update_rate_;
    current_pose_.position.x += current_velocity_.linear.x * dt;
    current_pose_.position.y += current_velocity_.linear.y * dt;
    current_pose_.position.z += current_velocity_.linear.z * dt;

    refreshOccupiedBody();
}

void ObstacleBase::refreshOccupiedBody() {
    occupied_body_ = buildOccupiedBody();
}

std::vector<visualization_msgs::Marker> ObstacleBase::getMarkers() {
    std::vector<visualization_msgs::Marker> markers;
    markers.push_back(createMarkerMessage());

    if (config_.velocity_obstacle_enabled) {
        markers.push_back(createVelocityMarker());
    }

    return markers;
}

xgc2_geometry_msgs::ConvexBodyInstance ObstacleBase::buildConvexBodyInstance(bool is_static) const {
    xgc2_geometry_msgs::ConvexBodyInstance instance;
    instance.id = occupied_body_.id;
    instance.name = occupied_body_.name;
    instance.geometry_type = occupied_body_.geometry_type;
    instance.is_static = is_static;

    instance.pose.position.x = occupied_body_.position.x();
    instance.pose.position.y = occupied_body_.position.y();
    instance.pose.position.z = occupied_body_.position.z();

    const Eigen::Quaterniond orientation = occupied_body_.orientation.normalized();
    instance.pose.orientation.x = orientation.x();
    instance.pose.orientation.y = orientation.y();
    instance.pose.orientation.z = orientation.z();
    instance.pose.orientation.w = orientation.w();

    instance.scale.x = occupied_body_.scale.x();
    instance.scale.y = occupied_body_.scale.y();
    instance.scale.z = occupied_body_.scale.z();

    instance.velocity.linear.x = occupied_body_.velocity.x();
    instance.velocity.linear.y = occupied_body_.velocity.y();
    instance.velocity.linear.z = occupied_body_.velocity.z();

    return instance;
}

Eigen::Vector3d ObstacleBase::currentPositionVector() const {
    return Eigen::Vector3d(
        current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
}

Eigen::Quaterniond ObstacleBase::currentOrientationQuaternion() const {
    return Eigen::Quaterniond(current_pose_.orientation.w,
                              current_pose_.orientation.x,
                              current_pose_.orientation.y,
                              current_pose_.orientation.z)
        .normalized();
}

Eigen::Vector3d ObstacleBase::currentVelocityVector() const {
    return Eigen::Vector3d(
        current_velocity_.linear.x, current_velocity_.linear.y, current_velocity_.linear.z);
}

visualization_msgs::Marker ObstacleBase::createVelocityMarker() {
    visualization_msgs::Marker arrow_marker;

    arrow_marker.header.stamp = ros::Time::now();
    arrow_marker.header.frame_id = config_.frame_id;
    arrow_marker.ns = "velocity_arrows";
    arrow_marker.id = config_.id + 1000;
    arrow_marker.type = visualization_msgs::Marker::ARROW;

    const double velocity_magnitude =
        std::sqrt(current_velocity_.linear.x * current_velocity_.linear.x +
                  current_velocity_.linear.y * current_velocity_.linear.y +
                  current_velocity_.linear.z * current_velocity_.linear.z);

    const double min_velocity = 0.01;
    if (velocity_magnitude < min_velocity) {
        arrow_marker.action = visualization_msgs::Marker::DELETE;
        return arrow_marker;
    }

    arrow_marker.action = visualization_msgs::Marker::ADD;
    arrow_marker.pose.orientation.w = 1.0;
    arrow_marker.scale.x = 0.10;
    arrow_marker.scale.y = 0.18;
    arrow_marker.scale.z = 0.30;
    arrow_marker.color.r = 0.0f;
    arrow_marker.color.g = 1.0f;
    arrow_marker.color.b = 0.0f;
    arrow_marker.color.a = 1.0f;
    arrow_marker.lifetime = ros::Duration(0);

    geometry_msgs::Point arrow_start;
    geometry_msgs::Point arrow_end;
    arrow_start.x = 0.0;
    arrow_start.y = 0.0;
    arrow_start.z = 0.0;

    const double arrow_length = 1.5;
    const double norm_factor = arrow_length / velocity_magnitude;
    arrow_end.x = current_velocity_.linear.x * norm_factor;
    arrow_end.y = current_velocity_.linear.y * norm_factor;
    arrow_end.z = current_velocity_.linear.z * norm_factor;

    arrow_marker.points.push_back(arrow_start);
    arrow_marker.points.push_back(arrow_end);

    return arrow_marker;
}

} // namespace cluttered_environment
