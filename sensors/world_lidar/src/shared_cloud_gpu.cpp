// SPDX-License-Identifier: GPL-2.0-only
#include "xgc2_world_lidar/shared_cloud_gpu.hpp"
#include <cstring>
#include <stdexcept>
#include <vector>
#include <xgc2_sensing/loader.hpp>
namespace xgc2_world_lidar {
struct SharedCloudGpu::Impl {
    const xgc2_sensing_gpu_api* api = nullptr;
    void* renderer = nullptr;
    SensorMetadata metadata;
    std::vector<unsigned char> bytes;
    pcl::PointCloud<pcl::PointXYZI> points;
    ~Impl() {
        if (renderer)
            api->destroy(renderer);
    }
};
static xgc2_sensing_gpu_projection projection(const SensorMetadata& m) {
    return {m.range_m,
            m.min_range_m,
            m.h_fov_deg,
            m.v_fov_deg,
            m.h_res,
            m.v_res,
            m.point_cover_spacing_m,
            m.publish_rate_hz,
            uint32_t(m.observation_model == "crop_through"),
            uint32_t(m.heading_cos_min.has_value() || m.vertical_slab_tan.has_value()),
            m.heading_cos_min.value_or(-1),
            m.vertical_slab_tan.value_or(0)};
}
SharedCloudGpu::SharedCloudGpu() : impl_(new Impl) {}
SharedCloudGpu::~SharedCloudGpu() = default;
void SharedCloudGpu::load(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                          const SensorMetadata& m) {
    validateGpuSensorMetadata(m);
    if (impl_->renderer || !cloud || cloud->empty())
        throw std::invalid_argument("GPU requires one nonempty static cloud");
    impl_->api = &xgc2_sensing::gpuLibrary();
    impl_->metadata = m;
    std::vector<float> xyz;
    xyz.reserve(cloud->size() * 3);
    for (const auto& p : *cloud) {
        xyz.push_back(p.x);
        xyz.push_back(p.y);
        xyz.push_back(p.z);
    }
    const auto config = projection(m);
    char error[512] = {};
    impl_->renderer = impl_->api->create(xyz.data(), cloud->size(), &config, error, sizeof(error));
    if (!impl_->renderer)
        throw std::runtime_error(error);
    impl_->points.points.reserve(cloud->size());
    impl_->bytes.reserve(cloud->size() * 16);
}
const pcl::PointCloud<pcl::PointXYZI>&
SharedCloudGpu::scan(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double stamp) {
    return scan(p, q, stamp, impl_->metadata);
}
const pcl::PointCloud<pcl::PointXYZI>& SharedCloudGpu::scan(const Eigen::Vector3d& p,
                                                            const Eigen::Quaterniond& q,
                                                            double stamp,
                                                            const SensorMetadata& m) {
    const auto config = projection(m);
    const double pos[3] = {p.x(), p.y(), p.z()}, rot[4] = {q.w(), q.x(), q.y(), q.z()};
    char error[512] = {};
    auto grow = [](void* context, size_t size) -> void* {
        auto& data = *static_cast<std::vector<unsigned char>*>(context);
        data.resize(size);
        return data.data();
    };
    if (impl_->api->scan(
            impl_->renderer, pos, rot, stamp, &config, grow, &impl_->bytes, error, sizeof(error)))
        throw std::runtime_error(error);
    impl_->points.resize(impl_->bytes.size() / 16);
    for (size_t i = 0; i < impl_->points.size(); ++i) {
        float xyzi[4];
        std::memcpy(xyzi, impl_->bytes.data() + i * 16, 16);
        auto& point = impl_->points[i];
        point.x = xyzi[0];
        point.y = xyzi[1];
        point.z = xyzi[2];
        point.intensity = xyzi[3];
    }
    return impl_->points;
}
} // namespace xgc2_world_lidar
