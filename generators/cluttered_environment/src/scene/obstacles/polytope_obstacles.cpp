/**
 * @file polytope_obstacles.cpp
 * @brief Implementation of polytope obstacle types
 */

#include "cluttered_environment/scene/obstacles/polytope_obstacles.h"

#include "xgc2_math/geometry/geo_utils.hpp"
#include "xgc2_math/geometry/occupied_sets/point_set.h"
#include "xgc2_math/geometry/quickhull.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace cluttered_environment {

namespace {

constexpr std::size_t kMinQuickHullVertices = 4;
constexpr int kHMatrixCols = 4;

Eigen::Index toEigenIndex(std::size_t value) {
    if (value > static_cast<std::size_t>(std::numeric_limits<Eigen::Index>::max())) {
        throw std::runtime_error("point count exceeds Eigen::Index range");
    }
    return static_cast<Eigen::Index>(value);
}

uint32_t toMarkerIndex(std::size_t value) {
    if (value > static_cast<std::size_t>(std::numeric_limits<uint32_t>::max())) {
        throw std::runtime_error("marker index exceeds uint32 range");
    }
    return static_cast<uint32_t>(value);
}

} // namespace

HPolytopeObstacle::HPolytopeObstacle(ros::NodeHandle& nh,
                                     const ObstacleConfig& config,
                                     double tf_rate,
                                     double velocity_update_rate,
                                     double velocity_timeout)
    : ObstacleBase(nh, config, tf_rate, velocity_update_rate, velocity_timeout) {
    epsilon_ = config.geometry_params_double.count("epsilon")
                   ? config.geometry_params_double.at("epsilon")
                   : 1e-6;

    if (config.geometry_params_vec.count("h_matrix_flat")) {
        const auto flat = config.geometry_params_vec.at("h_matrix_flat");
        const int rows = config.geometry_params_int.at("h_matrix_rows");
        if (rows <= 0) {
            ROS_ERROR("H-polytope obstacle %d: h_matrix_rows must be positive", config.id);
            return;
        }

        const std::size_t expected_values =
            static_cast<std::size_t>(rows) * static_cast<std::size_t>(kHMatrixCols);
        if (flat.size() != expected_values) {
            ROS_ERROR("H-polytope obstacle %d: h_matrix_flat size does not match h_matrix_rows",
                      config.id);
            return;
        }

        h_matrix_.resize(rows, kHMatrixCols);
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < kHMatrixCols; ++j) {
                const std::size_t flat_index =
                    static_cast<std::size_t>(i) * static_cast<std::size_t>(kHMatrixCols) +
                    static_cast<std::size_t>(j);
                h_matrix_(i, j) = flat[flat_index];
            }
        }
    } else {
        ROS_ERROR("H-polytope obstacle must have 'h_matrix_flat' in config");
    }
}

void HPolytopeObstacle::generateGeometry() {
    computeVerticesFromHMatrix();

    for (auto& vertex : vertices_local_) {
        vertex.x() *= config_.scale.x();
        vertex.y() *= config_.scale.y();
        vertex.z() *= config_.scale.z();
    }

    if (!vertices_local_.empty()) {
        bbox_min_local_ = vertices_local_[0];
        bbox_max_local_ = vertices_local_[0];
        for (const auto& v : vertices_local_) {
            bbox_min_local_ = bbox_min_local_.cwiseMin(v);
            bbox_max_local_ = bbox_max_local_.cwiseMax(v);
        }
    }
}

void HPolytopeObstacle::computeVerticesFromHMatrix() {
    if (h_matrix_.cols() != 4 ||
        h_matrix_.rows() < static_cast<Eigen::Index>(kMinQuickHullVertices)) {
        ROS_ERROR("H-polytope obstacle %d: Need at least %zu valid H-rows, got %ld",
                  config_.id,
                  kMinQuickHullVertices,
                  h_matrix_.rows());
        return;
    }

    Eigen::Matrix3Xd vPoly;

    if (!geo_utils::enumerateVs(h_matrix_, vPoly, epsilon_)) {
        ROS_ERROR("Failed to enumerate vertices from H-representation for obstacle %d", config_.id);
        return;
    }

    ROS_INFO("H-polytope obstacle %d: Enumerated %ld vertices", config_.id, vPoly.cols());

    support_vertices_unit_.clear();
    support_vertices_unit_.reserve(static_cast<std::size_t>(vPoly.cols()));
    for (Eigen::Index i = 0; i < vPoly.cols(); ++i) {
        support_vertices_unit_.push_back(vPoly.col(i));
    }

    if (support_vertices_unit_.size() < kMinQuickHullVertices) {
        ROS_ERROR("H-polytope obstacle %d: Need at least %zu vertices for 3D hull, got %zu",
                  config_.id,
                  kMinQuickHullVertices,
                  support_vertices_unit_.size());
        return;
    }

    quickhull::QuickHull<double> qh;
    const auto hull = qh.getConvexHull(vPoly.data(), vPoly.cols(), false, true, epsilon_);

    const auto& idx_buffer = hull.getIndexBuffer();
    const std::size_t num_triangles = idx_buffer.size() / 3;
    ROS_INFO("H-polytope obstacle %d: Generated %zu triangles", config_.id, num_triangles);

    vertices_local_.clear();
    vertices_local_.reserve(idx_buffer.size());
    for (const size_t idx : idx_buffer) {
        vertices_local_.push_back(vPoly.col(idx));
    }

    triangle_indices_.clear();
    triangle_indices_.reserve(idx_buffer.size());
    for (size_t i = 0; i < idx_buffer.size(); ++i) {
        triangle_indices_.push_back(toMarkerIndex(i));
    }

    if (!vertices_local_.empty()) {
        bbox_min_local_ = vertices_local_[0];
        bbox_max_local_ = vertices_local_[0];
        for (const auto& v : vertices_local_) {
            bbox_min_local_ = bbox_min_local_.cwiseMin(v);
            bbox_max_local_ = bbox_max_local_.cwiseMax(v);
        }
    }
}

visualization_msgs::Marker HPolytopeObstacle::createMarkerMessage() {
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = config_.frame_id;
    marker.ns = "obstacles";
    marker.id = config_.id;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = static_cast<float>(config_.color[0]);
    marker.color.g = static_cast<float>(config_.color[1]);
    marker.color.b = static_cast<float>(config_.color[2]);
    marker.color.a = static_cast<float>(config_.color[3]);
    marker.lifetime = ros::Duration(0);

    for (size_t i = 0; i < triangle_indices_.size(); ++i) {
        const auto& vertex = vertices_local_[triangle_indices_[i]];
        geometry_msgs::Point p;
        p.x = vertex.x();
        p.y = vertex.y();
        p.z = vertex.z();
        marker.points.push_back(p);
    }

    return marker;
}

visualization_msgs::Marker HPolytopeObstacle::createEdgeMarker() {
    visualization_msgs::Marker edge_marker;

    edge_marker.header.stamp = ros::Time::now();
    edge_marker.header.frame_id = config_.frame_id;
    edge_marker.ns = "edges";
    edge_marker.id = config_.id + 10000;
    edge_marker.action = visualization_msgs::Marker::DELETE;
    edge_marker.pose.orientation.w = 1.0;

    return edge_marker;
}

std::vector<visualization_msgs::Marker> HPolytopeObstacle::getMarkers() {
    std::vector<visualization_msgs::Marker> markers;
    markers.push_back(createMarkerMessage());
    markers.push_back(createEdgeMarker());

    if (config_.velocity_obstacle_enabled) {
        const bool has_valid_velocity =
            !last_velocity_time_.isZero() &&
            (ros::Time::now() - last_velocity_time_).toSec() < velocity_timeout_;

        if (has_valid_velocity) {
            markers.push_back(createVelocityMarker());
        } else if (!last_velocity_time_.isZero()) {
            visualization_msgs::Marker delete_marker;
            delete_marker.header.stamp = ros::Time::now();
            delete_marker.header.frame_id = config_.frame_id;
            delete_marker.ns = "velocity_arrows";
            delete_marker.id = config_.id + 1000;
            delete_marker.action = visualization_msgs::Marker::DELETE;
            markers.push_back(delete_marker);
        }
    }

    return markers;
}

std::string HPolytopeObstacle::getType() const {
    if (config_.geometry_params_string.count("h_polytope_template")) {
        return "h_polytope:" + config_.geometry_params_string.at("h_polytope_template");
    }
    return "h_polytope:inline:" + std::to_string(config_.id);
}

VPolytopeObstacle::VPolytopeObstacle(ros::NodeHandle& nh,
                                     const ObstacleConfig& config,
                                     double tf_rate,
                                     double velocity_update_rate,
                                     double velocity_timeout)
    : ObstacleBase(nh, config, tf_rate, velocity_update_rate, velocity_timeout) {
    epsilon_ = config.geometry_params_double.count("epsilon")
                   ? config.geometry_params_double.at("epsilon")
                   : 1e-6;

    if (config.geometry_params_vec.count("vertices_flat")) {
        const auto flat = config.geometry_params_vec.at("vertices_flat");
        if (flat.size() % 3 != 0) {
            ROS_ERROR("V-polytope obstacle %d: vertices_flat size must be divisible by 3",
                      config.id);
            return;
        }

        const std::size_t num_vertices = flat.size() / 3;

        vertices_original_.clear();
        vertices_original_.reserve(num_vertices);
        for (std::size_t i = 0; i < num_vertices; ++i) {
            vertices_original_.push_back(
                Eigen::Vector3d(flat[3 * i], flat[3 * i + 1], flat[3 * i + 2]));
        }

        ROS_INFO(
            "V-polytope obstacle %d: Loaded %zu vertices from template", config_.id, num_vertices);
    } else {
        ROS_ERROR("V-polytope obstacle must have 'vertices_flat' in config");
    }
}

void VPolytopeObstacle::generateGeometry() {
    generateConvexHull();

    for (auto& vertex : vertices_local_) {
        vertex.x() *= config_.scale.x();
        vertex.y() *= config_.scale.y();
        vertex.z() *= config_.scale.z();
    }

    if (!vertices_local_.empty()) {
        bbox_min_local_ = vertices_local_[0];
        bbox_max_local_ = vertices_local_[0];
        for (const auto& v : vertices_local_) {
            bbox_min_local_ = bbox_min_local_.cwiseMin(v);
            bbox_max_local_ = bbox_max_local_.cwiseMax(v);
        }
    }
}

void VPolytopeObstacle::generateConvexHull() {
    if (vertices_original_.size() < kMinQuickHullVertices) {
        ROS_ERROR("V-polytope obstacle %d: Need at least %zu vertices for 3D hull, got %zu",
                  config_.id,
                  kMinQuickHullVertices,
                  vertices_original_.size());
        return;
    }

    const Eigen::Index vertex_count = toEigenIndex(vertices_original_.size());
    Eigen::Matrix3Xd vPoly(3, vertex_count);
    for (Eigen::Index i = 0; i < vertex_count; ++i) {
        vPoly.col(i) = vertices_original_[static_cast<std::size_t>(i)];
    }

    quickhull::QuickHull<double> qh;
    const auto hull = qh.getConvexHull(vPoly.data(), vPoly.cols(), false, true, epsilon_);

    const auto& idx_buffer = hull.getIndexBuffer();
    const std::size_t num_triangles = idx_buffer.size() / 3;
    ROS_INFO("V-polytope obstacle %d: Generated %zu triangles from %zu vertices",
             config_.id,
             num_triangles,
             vertices_original_.size());

    vertices_local_.clear();
    vertices_local_.reserve(idx_buffer.size());
    for (const size_t idx : idx_buffer) {
        vertices_local_.push_back(vPoly.col(idx));
    }

    triangle_indices_.clear();
    triangle_indices_.reserve(idx_buffer.size());
    for (size_t i = 0; i < idx_buffer.size(); ++i) {
        triangle_indices_.push_back(toMarkerIndex(i));
    }
}

visualization_msgs::Marker VPolytopeObstacle::createMarkerMessage() {
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = config_.frame_id;
    marker.ns = "obstacles";
    marker.id = config_.id;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = static_cast<float>(config_.color[0]);
    marker.color.g = static_cast<float>(config_.color[1]);
    marker.color.b = static_cast<float>(config_.color[2]);
    marker.color.a = static_cast<float>(config_.color[3]);
    marker.lifetime = ros::Duration(0);

    for (size_t i = 0; i < triangle_indices_.size(); ++i) {
        const auto& vertex = vertices_local_[triangle_indices_[i]];
        geometry_msgs::Point p;
        p.x = vertex.x();
        p.y = vertex.y();
        p.z = vertex.z();
        marker.points.push_back(p);
    }

    return marker;
}

visualization_msgs::Marker VPolytopeObstacle::createEdgeMarker() {
    visualization_msgs::Marker edge_marker;

    edge_marker.header.stamp = ros::Time::now();
    edge_marker.header.frame_id = config_.frame_id;
    edge_marker.ns = "edges";
    edge_marker.id = config_.id + 10000;
    edge_marker.action = visualization_msgs::Marker::DELETE;
    edge_marker.pose.orientation.w = 1.0;

    return edge_marker;
}

std::vector<visualization_msgs::Marker> VPolytopeObstacle::getMarkers() {
    std::vector<visualization_msgs::Marker> markers;
    markers.push_back(createMarkerMessage());
    markers.push_back(createEdgeMarker());

    if (config_.velocity_obstacle_enabled) {
        const bool has_valid_velocity =
            !last_velocity_time_.isZero() &&
            (ros::Time::now() - last_velocity_time_).toSec() < velocity_timeout_;

        if (has_valid_velocity) {
            markers.push_back(createVelocityMarker());
        } else if (!last_velocity_time_.isZero()) {
            visualization_msgs::Marker delete_marker;
            delete_marker.header.stamp = ros::Time::now();
            delete_marker.header.frame_id = config_.frame_id;
            delete_marker.ns = "velocity_arrows";
            delete_marker.id = config_.id + 1000;
            delete_marker.action = visualization_msgs::Marker::DELETE;
            markers.push_back(delete_marker);
        }
    }

    return markers;
}

std::string VPolytopeObstacle::getType() const {
    if (config_.geometry_params_string.count("v_polytope_template")) {
        return "v_polytope:" + config_.geometry_params_string.at("v_polytope_template");
    }
    return "v_polytope:inline:" + std::to_string(config_.id);
}

} // namespace cluttered_environment
