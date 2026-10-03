#include "xgc2_world_lidar/shared_cloud_cpu.hpp"
#include <pcl/filters/voxel_grid.h>
#include <stdexcept>
namespace xgc2_world_lidar {
void SharedCloudCpu::load(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input,
                          const SensorMetadata& metadata) {
    validateSensorMetadata(metadata);
    if (metadata.backend != "cpu" || !input)
        throw std::invalid_argument("actual CPU cloud required");
    metadata_ = metadata;
    inputs_ = input->size();
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(
        metadata.prevoxel_leaf_m[0], metadata.prevoxel_leaf_m[1], metadata.prevoxel_leaf_m[2]);
    voxel.setInputCloud(input); // use the caller-owned decoded cloud; no extra full input copy
    voxel.filter(*voxels_);
    if (!voxels_->empty())
        tree_.setInputCloud(voxels_);
    ++loads_;
}
void SharedCloudCpu::scanInto(const Eigen::Vector3d& position,
                              const Eigen::Quaterniond& orientation,
                              CropResult* result) const {
    if (!result)
        throw std::invalid_argument("sensor-owned reusable scratch required");
    result->cloud.points.clear();
    result->indices.clear();
    result->squared_distances.clear();
    result->radius_candidates = 0;
    result->cloud.width = 0;
    result->cloud.height = 1;
    result->cloud.is_dense = true;
    if (!loads_ || voxels_->empty())
        return;
    const auto& m = metadata_;
    const bool body_dot = m.heading_cos_min.has_value(),
               world_slab = m.vertical_slab_tan.has_value();
    const Eigen::Matrix3d rotation = orientation.toRotationMatrix();
    const Eigen::Vector3d body_x = rotation.col(0);
    pcl::PointXYZ search(position.x(), position.y(), position.z());
    tree_.radiusSearch(search, m.range_m, result->indices, result->squared_distances);
    result->radius_candidates = result->indices.size();
    for (auto index : result->indices) {
        const auto& p = voxels_->points[index];
        // Preserve the fixed original division/order, including floating boundaries.
        if (world_slab && std::abs(p.z - position.z()) / m.range_m > *m.vertical_slab_tan)
            continue;
        if (body_dot) {
            const Eigen::Vector3d delta(p.x - position.x(), p.y - position.y(), p.z - position.z());
            if (delta.normalized().dot(body_x) < *m.heading_cos_min)
                continue;
        }
        result->cloud.points.push_back(p);
    }
    result->cloud.width = result->cloud.points.size();
}
} // namespace xgc2_world_lidar
