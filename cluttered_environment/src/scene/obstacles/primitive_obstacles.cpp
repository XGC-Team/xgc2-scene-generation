/**
 * @file primitive_obstacles.cpp
 * @brief Implementation of primitive obstacle types
 */

#include "cluttered_environment/scene/obstacles/primitive_obstacles.h"

#include "geometry/occupied_sets/box_set.h"
#include "geometry/occupied_sets/capsule_set.h"
#include "geometry/occupied_sets/cylinder_set.h"
#include "geometry/occupied_sets/sphere_set.h"

#include <algorithm>
#include <cmath>

namespace cluttered_environment {

namespace {

geometry_msgs::Point toGeometryPoint(const Eigen::Vector3d& point) {
    geometry_msgs::Point ros_point;
    ros_point.x = point.x();
    ros_point.y = point.y();
    ros_point.z = point.z();
    return ros_point;
}

xgc2_math::ConvexBody makeBodyMetadata(const ObstacleConfig& config,
                                           const std::string& geometry_type,
                                           const Eigen::Vector3d& position,
                                           const Eigen::Quaterniond& orientation,
                                           const Eigen::Vector3d& scale,
                                           const Eigen::Vector3d& velocity) {
    xgc2_math::ConvexBody body;
    body.id = config.id;
    body.name = config.name;
    body.geometry_type = geometry_type;
    body.position = position;
    body.orientation = orientation.normalized();
    body.scale = scale;
    body.velocity = velocity;
    return body;
}

} // namespace

SphereObstacle::SphereObstacle(ros::NodeHandle& nh,
                               const ObstacleConfig& config,
                               double tf_rate,
                               double velocity_update_rate,
                               double velocity_timeout)
    : ObstacleBase(nh, config, tf_rate, velocity_update_rate, velocity_timeout) {
    radius_ = 1.0;
}

void SphereObstacle::generateGeometry() {
    bbox_min_local_ = Eigen::Vector3d(
        -radius_ * config_.scale.x(), -radius_ * config_.scale.y(), -radius_ * config_.scale.z());
    bbox_max_local_ = Eigen::Vector3d(
        radius_ * config_.scale.x(), radius_ * config_.scale.y(), radius_ * config_.scale.z());
}

xgc2_geometry_msgs::GeometryTemplate
SphereObstacle::buildGeometryTemplate(int /*resolution*/) const {
    xgc2_geometry_msgs::GeometryTemplate tmpl;
    tmpl.type = getType();
    tmpl.resolution = 0;
    return tmpl;
}

xgc2_math::ConvexBody SphereObstacle::buildOccupiedBody() const {
    xgc2_math::ConvexBody body = makeBodyMetadata(config_,
                                                      getType(),
                                                      currentPositionVector(),
                                                      currentOrientationQuaternion(),
                                                      config_.scale,
                                                      currentVelocityVector());
    body.shape = std::make_shared<xgc2_math::SphereSet>(body.position, body.scale.x());
    return body;
}

visualization_msgs::Marker SphereObstacle::createMarkerMessage() {
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = config_.frame_id;
    marker.ns = "obstacles";
    marker.id = config_.id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = radius_ * 2.0 * config_.scale.x();
    marker.scale.y = radius_ * 2.0 * config_.scale.y();
    marker.scale.z = radius_ * 2.0 * config_.scale.z();
    marker.color.r = static_cast<float>(config_.color[0]);
    marker.color.g = static_cast<float>(config_.color[1]);
    marker.color.b = static_cast<float>(config_.color[2]);
    marker.color.a = static_cast<float>(config_.color[3]);
    marker.lifetime = ros::Duration(0);

    return marker;
}

CylinderObstacle::CylinderObstacle(ros::NodeHandle& nh,
                                   const ObstacleConfig& config,
                                   double tf_rate,
                                   double velocity_update_rate,
                                   double velocity_timeout)
    : ObstacleBase(nh, config, tf_rate, velocity_update_rate, velocity_timeout) {
    radius_ = 1.0;
    height_ = 1.0;
}

void CylinderObstacle::generateGeometry() {
    const double half_height = height_ / 2.0;

    bbox_min_local_ = Eigen::Vector3d(-radius_ * config_.scale.x(),
                                      -radius_ * config_.scale.y(),
                                      -half_height * config_.scale.z());
    bbox_max_local_ = Eigen::Vector3d(
        radius_ * config_.scale.x(), radius_ * config_.scale.y(), half_height * config_.scale.z());
}

xgc2_geometry_msgs::GeometryTemplate CylinderObstacle::buildGeometryTemplate(int resolution) const {
    xgc2_geometry_msgs::GeometryTemplate tmpl;
    tmpl.type = getType();
    tmpl.resolution = std::max(0, resolution);

    if (tmpl.resolution == 0) {
        return tmpl;
    }

    for (int i = 0; i < tmpl.resolution; ++i) {
        const double angle = 2.0 * M_PI * i / tmpl.resolution;
        tmpl.support_points.push_back(
            toGeometryPoint(Eigen::Vector3d(std::cos(angle), std::sin(angle), -0.5)));
        tmpl.support_points.push_back(
            toGeometryPoint(Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.5)));
    }

    return tmpl;
}

xgc2_math::ConvexBody CylinderObstacle::buildOccupiedBody() const {
    xgc2_math::ConvexBody body = makeBodyMetadata(config_,
                                                      getType(),
                                                      currentPositionVector(),
                                                      currentOrientationQuaternion(),
                                                      config_.scale,
                                                      currentVelocityVector());
    body.shape = std::make_shared<xgc2_math::CylinderSet>(
        body.position, body.scale.x(), body.scale.z(), body.orientation);
    return body;
}

visualization_msgs::Marker CylinderObstacle::createMarkerMessage() {
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = config_.frame_id;
    marker.ns = "obstacles";
    marker.id = config_.id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = radius_ * 2.0 * config_.scale.x();
    marker.scale.y = radius_ * 2.0 * config_.scale.y();
    marker.scale.z = height_ * config_.scale.z();
    marker.color.r = static_cast<float>(config_.color[0]);
    marker.color.g = static_cast<float>(config_.color[1]);
    marker.color.b = static_cast<float>(config_.color[2]);
    marker.color.a = static_cast<float>(config_.color[3]);
    marker.lifetime = ros::Duration(0);

    return marker;
}

CapsuleObstacle::CapsuleObstacle(ros::NodeHandle& nh,
                                 const ObstacleConfig& config,
                                 double tf_rate,
                                 double velocity_update_rate,
                                 double velocity_timeout)
    : ObstacleBase(nh, config, tf_rate, velocity_update_rate, velocity_timeout) {
    radius_ = 1.0;
    length_ = 2.0;
}

void CapsuleObstacle::generateGeometry() {
    const double radius = std::max(0.0, config_.scale.x());
    const double length = std::max(2.0 * radius, config_.scale.z());
    const double half_length = 0.5 * length;

    bbox_min_local_ = Eigen::Vector3d(-radius, -radius, -half_length);
    bbox_max_local_ = Eigen::Vector3d(radius, radius, half_length);
}

xgc2_geometry_msgs::GeometryTemplate
CapsuleObstacle::buildGeometryTemplate(int /*resolution*/) const {
    xgc2_geometry_msgs::GeometryTemplate tmpl;
    tmpl.type = getType();
    tmpl.resolution = 0;
    return tmpl;
}

xgc2_math::ConvexBody CapsuleObstacle::buildOccupiedBody() const {
    xgc2_math::ConvexBody body = makeBodyMetadata(config_,
                                                      getType(),
                                                      currentPositionVector(),
                                                      currentOrientationQuaternion(),
                                                      config_.scale,
                                                      currentVelocityVector());
    body.shape = std::make_shared<xgc2_math::CapsuleSet>(
        body.position, body.scale.x(), body.scale.z(), body.orientation);
    return body;
}

visualization_msgs::Marker CapsuleObstacle::createMarkerMessage() {
    visualization_msgs::Marker marker;

    const double radius = std::max(0.0, config_.scale.x());
    const double length = std::max(2.0 * radius, config_.scale.z());
    const double cylinder_length = std::max(0.0, length - 2.0 * radius);

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = config_.frame_id;
    marker.ns = "obstacles";
    marker.id = config_.id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = cylinder_length;
    marker.color.r = static_cast<float>(config_.color[0]);
    marker.color.g = static_cast<float>(config_.color[1]);
    marker.color.b = static_cast<float>(config_.color[2]);
    marker.color.a = static_cast<float>(config_.color[3]);
    marker.lifetime = ros::Duration(0);

    return marker;
}

visualization_msgs::Marker CapsuleObstacle::createCapMarker(int cap_index, double z_offset) const {
    visualization_msgs::Marker marker;

    const double radius = std::max(0.0, config_.scale.x());

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = config_.frame_id;
    marker.ns = "capsule_caps";
    marker.id = config_.id * 2 + cap_index;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = z_offset;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = 2.0 * radius;
    marker.color.r = static_cast<float>(config_.color[0]);
    marker.color.g = static_cast<float>(config_.color[1]);
    marker.color.b = static_cast<float>(config_.color[2]);
    marker.color.a = static_cast<float>(config_.color[3]);
    marker.lifetime = ros::Duration(0);

    return marker;
}

std::vector<visualization_msgs::Marker> CapsuleObstacle::getMarkers() {
    std::vector<visualization_msgs::Marker> markers;

    const double radius = std::max(0.0, config_.scale.x());
    const double length = std::max(2.0 * radius, config_.scale.z());
    const double cap_offset = 0.5 * std::max(0.0, length - 2.0 * radius);

    markers.push_back(createMarkerMessage());
    markers.push_back(createCapMarker(0, -cap_offset));
    markers.push_back(createCapMarker(1, cap_offset));

    visualization_msgs::Marker delete_edge_marker;
    delete_edge_marker.header.stamp = ros::Time::now();
    delete_edge_marker.header.frame_id = config_.frame_id;
    delete_edge_marker.ns = "edges";
    delete_edge_marker.id = config_.id + 10000;
    delete_edge_marker.action = visualization_msgs::Marker::DELETE;
    delete_edge_marker.pose.orientation.w = 1.0;
    markers.push_back(delete_edge_marker);

    if (config_.velocity_obstacle_enabled) {
        markers.push_back(createVelocityMarker());
    }

    return markers;
}

CubeObstacle::CubeObstacle(ros::NodeHandle& nh,
                           const ObstacleConfig& config,
                           double tf_rate,
                           double velocity_update_rate,
                           double velocity_timeout)
    : ObstacleBase(nh, config, tf_rate, velocity_update_rate, velocity_timeout) {
    size_ = Eigen::Vector3d(1.0, 1.0, 1.0);
}

void CubeObstacle::generateGeometry() {
    const Eigen::Vector3d half_size = size_ / 2.0;
    const Eigen::Vector3d hs(half_size.x() * config_.scale.x(),
                             half_size.y() * config_.scale.y(),
                             half_size.z() * config_.scale.z());

    bbox_min_local_ = -hs;
    bbox_max_local_ = hs;
}

xgc2_geometry_msgs::GeometryTemplate CubeObstacle::buildGeometryTemplate(int /*resolution*/) const {
    xgc2_geometry_msgs::GeometryTemplate tmpl;
    tmpl.type = getType();
    tmpl.resolution = 0;

    const std::vector<Eigen::Vector3d> corners = {Eigen::Vector3d(-0.5, -0.5, -0.5),
                                                  Eigen::Vector3d(0.5, -0.5, -0.5),
                                                  Eigen::Vector3d(0.5, 0.5, -0.5),
                                                  Eigen::Vector3d(-0.5, 0.5, -0.5),
                                                  Eigen::Vector3d(-0.5, -0.5, 0.5),
                                                  Eigen::Vector3d(0.5, -0.5, 0.5),
                                                  Eigen::Vector3d(0.5, 0.5, 0.5),
                                                  Eigen::Vector3d(-0.5, 0.5, 0.5)};

    for (const auto& corner : corners) {
        tmpl.support_points.push_back(toGeometryPoint(corner));
    }

    return tmpl;
}

xgc2_math::ConvexBody CubeObstacle::buildOccupiedBody() const {
    xgc2_math::ConvexBody body = makeBodyMetadata(config_,
                                                      getType(),
                                                      currentPositionVector(),
                                                      currentOrientationQuaternion(),
                                                      config_.scale,
                                                      currentVelocityVector());
    body.shape =
        std::make_shared<xgc2_math::BoxSet>(body.position, body.scale, body.orientation);
    return body;
}

visualization_msgs::Marker CubeObstacle::createMarkerMessage() {
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = config_.frame_id;
    marker.ns = "obstacles";
    marker.id = config_.id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = size_.x() * config_.scale.x();
    marker.scale.y = size_.y() * config_.scale.y();
    marker.scale.z = size_.z() * config_.scale.z();
    marker.color.r = static_cast<float>(config_.color[0]);
    marker.color.g = static_cast<float>(config_.color[1]);
    marker.color.b = static_cast<float>(config_.color[2]);
    marker.color.a = static_cast<float>(config_.color[3]);
    marker.lifetime = ros::Duration(0);

    return marker;
}

} // namespace cluttered_environment
