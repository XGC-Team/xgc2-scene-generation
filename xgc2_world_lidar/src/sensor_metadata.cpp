#include "xgc2_world_lidar/sensor_metadata.hpp"
#include <cmath>
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
} // namespace xgc2_world_lidar
