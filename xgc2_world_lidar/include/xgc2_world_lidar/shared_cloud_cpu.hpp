#pragma once
#include "xgc2_world_lidar/scan_pool.h"
#include "xgc2_world_lidar/sensor_metadata.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>
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
// One sensor pose of a scan batch (world frame, the body pose the crop reads).
struct ScanPose {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};
// Receives the scan of poses[index] on the thread `worker` (a ScanPool worker index) that
// computed it. `scan` lives in that worker's scratch and is valid only until the sink returns;
// anything else the sink reuses across calls can be indexed by `worker` without locking.
using ScanSink = std::function<void(std::size_t index, std::size_t worker, const CropResult& scan)>;
class SharedCloudCpu {
public:
    void load(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input,
              const SensorMetadata& metadata);
    void scanInto(const Eigen::Vector3d& position,
                  const Eigen::Quaterniond& orientation,
                  CropResult* result) const;
    // Scans poses[0..n) on `pool`, each job being one scan followed by its sink (the node
    // serializes and publishes there). Every worker thread owns one reusable CropResult, so
    // jobs share nothing but the read-only map index. Each scan equals scanInto() on the same
    // pose, so the sink sees the same points at every pool size; only the order in which
    // different poses reach their sinks depends on scheduling. A pool of one thread runs the
    // sinks in index order on the calling thread. Not reentrant (one batch at a time).
    void scanBatch(ScanPool& pool, const std::vector<ScanPose>& poses, const ScanSink& sink);
    std::size_t input_count() const { return inputs_; }
    std::size_t voxel_count() const { return voxels_->size(); }
    std::size_t load_count() const { return loads_; }

private:
    SensorMetadata metadata_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxels_{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::search::KdTree<pcl::PointXYZ> tree_;
    std::size_t inputs_ = 0, loads_ = 0;
    std::vector<CropResult> worker_scratch_; // one per pool thread, grown by scanBatch
};
} // namespace xgc2_world_lidar
