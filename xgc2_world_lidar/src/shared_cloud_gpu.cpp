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
SharedCloudGpu::~SharedCloudGpu() {
    // Release projection buffers while the sole owner context is still alive.
    if (framebuffer_) glDeleteFramebuffers(1, &framebuffer_);
    if (color_buffer_) glDeleteRenderbuffers(1, &color_buffer_);
    if (depth_buffer_) glDeleteRenderbuffers(1, &depth_buffer_);
} // renderer_ then executes its original cleanup_once on this owner thread
void SharedCloudGpu::load(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                          const SensorMetadata& metadata) {
    validateGpuSensorMetadata(metadata);
    if (loaded_ || !cloud || cloud->empty())
        throw std::invalid_argument("GPU requires one nonempty static cloud");
    const float polar_res =
        static_cast<float>(metadata.h_fov_deg) / static_cast<float>(metadata.h_res);
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
const pcl::PointCloud<pcl::PointXYZI>& SharedCloudGpu::scan(
    const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double stamp,
    const SensorMetadata& m) {
    validateGpuSensorMetadata(m);
    if (!loaded_) throw std::logic_error("GPU static world is not loaded");
    renderer_->setParameters(m.h_res, m.v_res, 250.0f, 250.0f,
        static_cast<float>(m.point_cover_spacing_m),
        static_cast<float>(m.h_fov_deg) / static_cast<float>(m.h_res),
        static_cast<float>(m.h_fov_deg), static_cast<float>(m.v_fov_deg),
        static_cast<float>(m.min_range_m), static_cast<float>(m.range_m),
        static_cast<int>(m.publish_rate_hz), 0, 0, 0);
    if (!glfwGetCurrentContext()) throw std::runtime_error("GPU owner context missing");
    // The X11 window resize is asynchronous. A per-scan window resize can read
    // pixels from the previous sensor's drawable dimensions. Keep one reusable
    // offscreen framebuffer in the same context; preserve the native depth
    // precision and the original shader/render/readback math unchanged.
    if (!framebuffer_) {
        GLint bits=0, component=0;
        // Query the actual default framebuffer attachment in the core profile.
        glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_DEPTH,
            GL_FRAMEBUFFER_ATTACHMENT_DEPTH_SIZE, &bits);
        glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_DEPTH,
            GL_FRAMEBUFFER_ATTACHMENT_COMPONENT_TYPE, &component);
        depth_format_ = bits==16 ? GL_DEPTH_COMPONENT16 : bits==24 ? GL_DEPTH_COMPONENT24 :
            component==GL_FLOAT ? GL_DEPTH_COMPONENT32F : GL_DEPTH_COMPONENT32;
        if (bits!=16 && bits!=24 && bits!=32)
            throw std::runtime_error("unsupported native depth precision: " + std::to_string(bits));
        glGenFramebuffers(1, &framebuffer_);
        glGenRenderbuffers(1, &color_buffer_);
        glGenRenderbuffers(1, &depth_buffer_);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
    if (buffer_width_!=m.h_res || buffer_height_!=m.v_res) {
        const auto require_size = [&] {
            GLint width=0, height=0;
            glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_WIDTH, &width);
            glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_HEIGHT, &height);
            if (width!=m.h_res || height!=m.v_res)
                throw std::runtime_error("GPU projection buffer allocation unavailable for requested resolution");
        };
        glBindRenderbuffer(GL_RENDERBUFFER, color_buffer_);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, m.h_res, m.v_res);
        require_size();
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color_buffer_);
        glBindRenderbuffer(GL_RENDERBUFFER, depth_buffer_);
        glRenderbufferStorage(GL_RENDERBUFFER, depth_format_, m.h_res, m.v_res);
        require_size();
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_buffer_);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER)!=GL_FRAMEBUFFER_COMPLETE)
            throw std::runtime_error("GPU projection framebuffer allocation failed");
        buffer_width_=m.h_res;buffer_height_=m.v_res;
    }
    glDrawBuffer(GL_COLOR_ATTACHMENT0); glReadBuffer(GL_COLOR_ATTACHMENT0);
    glViewport(0, 0, m.h_res, m.v_res);
    return scan(p, q, stamp);
}
} // namespace xgc2_world_lidar

#endif // XGC_WORLD_LIDAR_GPU
