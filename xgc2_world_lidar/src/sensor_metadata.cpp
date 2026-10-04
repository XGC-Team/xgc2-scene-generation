#include "xgc2_world_lidar/sensor_metadata.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
namespace xgc2_world_lidar {
void validateSensorMetadata(const SensorMetadata& m) {
    if (m.backend != "cpu" || m.observation_model != "crop_through")
        throw std::invalid_argument("first shared CPU slice supports crop_through/cpu; other "
                                    "combinations are not silently substituted");
    if (!(m.range_m > 0 && std::isfinite(m.range_m) && m.publish_rate_hz > 0 &&
          std::isfinite(m.publish_rate_hz)) ||
        m.frame_id.empty() || (m.stamp_policy != "zero" && m.stamp_policy != "pose"))
        throw std::invalid_argument("actual radius/rate/frame/stamp required");
    for (auto leaf : m.prevoxel_leaf_m)
        if (!(leaf > 0 && std::isfinite(leaf)))
            throw std::invalid_argument("actual PCL voxel XYZ required");
    if (m.heading_cos_min &&
        (!std::isfinite(*m.heading_cos_min) || *m.heading_cos_min < -1 || *m.heading_cos_min > 1))
        throw std::invalid_argument("invalid body-dot predicate");
    if (m.vertical_slab_tan && (!std::isfinite(*m.vertical_slab_tan) || *m.vertical_slab_tan < 0))
        throw std::invalid_argument("invalid world-Z predicate");
}
void validateGpuSensorMetadata(const SensorMetadata& m) {
    if (m.observation_model != "lidar_scan" || m.backend != "gpu")
        throw std::invalid_argument("GPU requires explicit lidar_scan/gpu (spherical nearest)");
    // Validate the original native float parameter domain, not the demo values.
    const float near = static_cast<float>(m.min_range_m), far = static_cast<float>(m.range_m);
    const float spacing = static_cast<float>(m.point_cover_spacing_m);
    if (!std::isfinite(near) || !std::isfinite(far) || near <= 0 || far <= near ||
        !std::isfinite(spacing) || spacing <= 0 || !std::isfinite(m.publish_rate_hz) ||
        m.publish_rate_hz <= 0 || m.publish_rate_hz > std::numeric_limits<int>::max() ||
        m.h_res <= 0 || m.v_res <= 0 || !std::isfinite(m.h_fov_deg) ||
        !std::isfinite(m.v_fov_deg) || m.h_fov_deg <= 0 || m.h_fov_deg > 360 || m.v_fov_deg <= 0 ||
        m.v_fov_deg > 180 || m.h_res > std::numeric_limits<int>::max() / m.v_res)
        throw std::invalid_argument(
            "finite positive GPU range/spacing/rate and spherical grid required");
    const float h_step = static_cast<float>(m.h_fov_deg) / m.h_res;
    const float v_step = static_cast<float>(m.v_fov_deg) / m.v_res;
    if (!std::isfinite(h_step) || h_step <= 0 || h_step != v_step)
        throw std::invalid_argument(
            "original GPU kernel requires equal horizontal/vertical angular steps");
    // Original shader asin(cover_dis/depth), with depth >= near.
    const float shader_cover = 0.55f * 1.7321f * spacing;
    const float host_cover = static_cast<float>(0.55 * 1.7321 * spacing);
    if (!std::isfinite(shader_cover) || !std::isfinite(host_cover) || shader_cover > near ||
        host_cover > near)
        throw std::invalid_argument("original GPU point-cover/near ratio must be <=1 for asin");
    if (m.frame_id.empty() || (m.stamp_policy != "zero" && m.stamp_policy != "pose") ||
        (m.pose_type != "nav_msgs/Odometry" && m.pose_type != "geometry_msgs/PoseStamped"))
        throw std::invalid_argument("explicit GPU frame/pose-type/stamp required");
    if (m.heading_cos_min || m.vertical_slab_tan)
        throw std::invalid_argument("GPU does not implement CPU crop predicates");
    for (auto leaf : m.prevoxel_leaf_m)
        if (leaf != 0)
            throw std::invalid_argument("GPU does not implement CPU prevoxel");
}
} // namespace xgc2_world_lidar
