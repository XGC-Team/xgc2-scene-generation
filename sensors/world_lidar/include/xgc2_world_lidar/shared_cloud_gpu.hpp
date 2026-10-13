// SPDX-License-Identifier: GPL-2.0-only
#pragma once
#include "xgc2_world_lidar/sensor_metadata.hpp"
#include <Eigen/Geometry>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
namespace xgc2_world_lidar {
// Non-ROS call adapter only; the existing node owns subscriptions and lifecycle.
// Exactly one instance/context on its ros::spin owner thread, shared by all poses.
class SharedCloudGpu {
public:
    SharedCloudGpu();
    ~SharedCloudGpu();
    void load(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
              const SensorMetadata& metadata);
    const pcl::PointCloud<pcl::PointXYZI>&
    scan(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, double stamp);
    // Switch per-sensor projection on the same uploaded map and GL context.
    const pcl::PointCloud<pcl::PointXYZI>&
    scan(const Eigen::Vector3d&, const Eigen::Quaterniond&, double stamp, const SensorMetadata&);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace xgc2_world_lidar
