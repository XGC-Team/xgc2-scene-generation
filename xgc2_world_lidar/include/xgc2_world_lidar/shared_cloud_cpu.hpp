#pragma once
#include "xgc2_world_lidar/sensor_metadata.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <vector>
namespace xgc2_world_lidar {
struct CropResult {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::vector<int> indices;
    std::vector<float> squared_distances;
    std::size_t radius_candidates = 0;
};
class SharedCloudCpu {
public:
    void load(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input,
              const SensorMetadata& metadata);
    void scanInto(const Eigen::Vector3d& position,
                  const Eigen::Quaterniond& orientation,
                  CropResult* result) const;
    std::size_t input_count() const { return inputs_; }
    std::size_t voxel_count() const { return voxels_->size(); }
    std::size_t load_count() const { return loads_; }

private:
    SensorMetadata metadata_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxels_{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::search::KdTree<pcl::PointXYZ> tree_;
    std::size_t inputs_ = 0, loads_ = 0;
};
} // namespace xgc2_world_lidar
