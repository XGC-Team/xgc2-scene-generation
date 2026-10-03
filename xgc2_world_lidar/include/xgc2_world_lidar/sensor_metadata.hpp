#pragma once
#include <array>
#include <optional>
#include <string>
namespace xgc2_world_lidar {
// Existing sensor's owned caller configuration, not a wire schema/paper preset.
struct SensorMetadata {
    std::string observation_model; // crop_through, pinhole_depth, lidar_scan
    std::string backend;           // cpu or gpu, independent execution choice
    std::array<float, 3> prevoxel_leaf_m{{0, 0, 0}};
    double range_m = 0;
    std::optional<double> heading_cos_min;   // absent: no body-dot predicate
    std::optional<double> vertical_slab_tan; // absent: no fixed world-Z slab
    double publish_rate_hz = 0;
    std::string frame_id, stamp_policy; // explicit caller map/world and zero/pose
    std::string input_cloud_topic, pose_topic, output_topic, pose_type;
};
// First slice: CPU crop_through. Projection/occlusion follow that model;
// camera declarations remain caller provenance and do not enter this crop.
void validateSensorMetadata(const SensorMetadata& m);
} // namespace xgc2_world_lidar
