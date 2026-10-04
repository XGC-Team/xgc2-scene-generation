// SPDX-License-Identifier: GPL-2.0-only
// Original fixed renderer is included once in this non-ROS translation unit.
#ifdef XGC_WORLD_LIDAR_GPU
#include "xgc2_world_lidar/shared_cloud_gpu.hpp"
#include <algorithm>
#include <cctype>
#include <opengl_sim.hpp>
#include <stdexcept>
namespace xgc2_world_lidar {
SharedCloudGpu::SharedCloudGpu()
    : renderer_(new opengl_pointcloud_render), rendered_(new pcl::PointCloud<pcl::PointXYZI>) {}
SharedCloudGpu::~SharedCloudGpu() = default; // fixed cleanup_once, on the owner thread
void SharedCloudGpu::load(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                          const SensorMetadata& metadata) {
    if (loaded_ || !cloud || cloud->empty())
        throw std::invalid_argument("GPU requires one nonempty static cloud");
    const float polar_res = static_cast<float>(metadata.h_fov_deg) / metadata.h_res;
    renderer_->setParameters(metadata.h_res,
                             metadata.v_res,
                             250.0f,
                             250.0f,
                             static_cast<float>(metadata.point_cover_spacing_m),
                             polar_res,
                             static_cast<float>(metadata.h_fov_deg),
                             static_cast<float>(metadata.v_fov_deg),
                             static_cast<float>(metadata.min_range_m),
                             static_cast<float>(metadata.range_m),
                             static_cast<int>(metadata.publish_rate_hz),
                             0,
                             0,
                             0);
    pcl::PointCloud<PointType> typed;
    pcl::copyPointCloud(*cloud, typed); // one XYZ->original PointXYZI conversion; no voxel/filter
    if (!renderer_->read_pointcloud_frommemory(typed))
        throw std::runtime_error("original GPU context/static upload initialization failed");
    const char* vendor = reinterpret_cast<const char*>(glGetString(GL_VENDOR));
    const char* name = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
    const char* version = reinterpret_cast<const char*>(glGetString(GL_VERSION));
    if (!vendor || !name || !version)
        throw std::runtime_error("actual GL identity missing");
    std::string check(name);
    std::transform(
        check.begin(), check.end(), check.begin(), [](unsigned char c) { return std::tolower(c); });
    for (const auto* software : {"llvmpipe", "softpipe", "swrast", "software"})
        if (check.find(software) != std::string::npos)
            throw std::runtime_error("software GL is not admitted as GPU; no CPU fallback");
    std::cout << "GL_VENDOR=" << vendor << "\nGL_RENDERER=" << name << "\nGL_VERSION=" << version
              << std::endl;
    loaded_ = true;
}
const pcl::PointCloud<pcl::PointXYZI>& SharedCloudGpu::scan(const Eigen::Vector3d& position,
                                                            const Eigen::Quaterniond& orientation,
                                                            double stamp) {
    if (!loaded_)
        throw std::logic_error("GPU static world is not loaded");
    renderer_->render_pointcloud(
        rendered_, position.cast<float>(), orientation.cast<float>(), stamp);
    return *rendered_; // original points/intensity, including a legitimate empty scan
}
} // namespace xgc2_world_lidar

#endif // XGC_WORLD_LIDAR_GPU
