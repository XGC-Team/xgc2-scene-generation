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
    // One field of view and range spec for every model. 0 means "not set".
    //   crop_through/cpu: min_range_m (drop closer returns; with range_m it is the radial
    //     interval), h_fov_deg in (0, 360] and v_fov_deg in (0, 180] (the azimuth and elevation
    //     extent around body +X, as in WorldLidar::insideFov). Unset or full (360 / 180) keeps
    //     every point of the radius, which is the crop before these fields existed. They add to
    //     heading_cos_min and vertical_slab_tan; they do not replace them.
    //   lidar_scan/gpu: all of them are the original spherical grid and must be set (near,
    //     both extents, both resolutions, point-cover spacing).
    // crop_through returns stored map points, so it has no resolution: h_res, v_res and
    // point_cover_spacing_m are refused there rather than ignored, and prevoxel_leaf_m is
    // refused for the GPU, so one spec never silently means two models.
    double min_range_m = 0, h_fov_deg = 0, v_fov_deg = 0;
    int h_res = 0, v_res = 0;
    double point_cover_spacing_m = 0;
};
// CPU crop_through. Projection/occlusion follow that model; camera declarations remain caller
// provenance and do not enter this crop.
void validateSensorMetadata(const SensorMetadata& m);
void validateGpuSensorMetadata(const SensorMetadata& m);
} // namespace xgc2_world_lidar
