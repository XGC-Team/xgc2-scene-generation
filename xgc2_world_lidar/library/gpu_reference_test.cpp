// Original fixed-window entry vs shared-context projection switching.
// Requires real hardware; no software GL or CPU fallback is admitted by load().
#include <cassert>
#include <cmath>
#include <iostream>
#include <omp.h>
#include <xgc2_world_lidar/shared_cloud_gpu.hpp>
#include <xgc2_world_lidar/world_lidar.h>
using namespace xgc2_world_lidar;
int main() {
    omp_set_dynamic(0);
    omp_set_num_threads(1);
    SensorConfig scene_config;
    WorldLidar source(scene_config);
    source.setScene({Obstacle::box({5, 0, 0}, {1, 6, 4})});
    pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& p : source.globalMap(.1))
        map->push_back(pcl::PointXYZ(
            static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())));
    SensorMetadata configs[2];
    pcl::PointCloud<pcl::PointXYZI> references[2];
    for (int i = 0; i < 2; ++i) {
        auto& m = configs[i];
        m.backend = "gpu";
        m.observation_model = "lidar_scan";
        m.min_range_m = .2;
        m.range_m = 20;
        m.publish_rate_hz = 10;
        m.h_res = i ? 240 : 120;
        m.v_res = i ? 80 : 40;
        m.h_fov_deg = 90;
        m.v_fov_deg = 30;
        m.point_cover_spacing_m = .1;
        m.frame_id = "world";
        m.stamp_policy = "pose";
        m.pose_type = "geometry_msgs/PoseStamped";
        SharedCloudGpu original;
        original.load(map, m);
        references[i] =
            original.scan({0, double(i), 0}, Eigen::Quaterniond::Identity(), 1700000000);
        assert(!references[i].empty());
    }
    double maximum = 0;
    {
        SharedCloudGpu shared;
        shared.load(map, configs[0]);
        for (int k = 0; k < 12; ++k) {
            const int i = k % 2;
            const auto& got = shared.scan(
                {0, double(i), 0}, Eigen::Quaterniond::Identity(), 1700000000 + k * .1, configs[i]);
            assert(got.size() == references[i].size());
            for (size_t n = 0; n < got.size(); ++n) {
                const auto& a = got[n];
                const auto& b = references[i][n];
                for (auto d : {a.x - b.x, a.y - b.y, a.z - b.z, a.intensity - b.intensity}) {
                    assert(std::isfinite(d));
                    maximum = std::max(maximum, double(std::abs(d)));
                }
            }
        }
    }
    assert(maximum <= 1e-6);
    std::cout << "{\"result\":\"PASS\",\"switches\":12,\"original_points\":["
              << references[0].size() << ',' << references[1].size()
              << "],\"max_abs_xyzi_error\":" << maximum << ",\"tolerance\":1e-6}\n";
}
