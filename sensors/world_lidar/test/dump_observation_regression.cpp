// Compare this output byte for byte between the original core and PR6. This
// invokes both implementations' public API; it contains no sensor model.
#include "xgc2_world_lidar/world_lidar.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>

using namespace xgc2_world_lidar;

namespace {
template <class T>
void write(const T& value) {
    if (std::fwrite(&value, sizeof value, 1, stdout) != 1)
        std::abort();
}
void point(const Eigen::Vector3d& p) {
    for (int i = 0; i < 3; ++i)
        write(p[i]);
}
void cloud(const std::vector<Eigen::Vector3d>& values) {
    write(static_cast<uint64_t>(values.size()));
    for (const auto& p : values)
        point(p);
}
} // namespace

int main() {
    const Eigen::Quaterniond rotation(
        Eigen::AngleAxisd(0.43, Eigen::Vector3d(1, 2, 3).normalized()));
    std::vector<Obstacle> initial{
        Obstacle::box({3, 0, 1}, {1, 4, 2}, rotation),
        Obstacle::sphere({3.2, 0, 1}, 0.8),
        Obstacle::cylinder({-2, 3, 1}, 0.6, 2.5, rotation),
        Obstacle::capsule({0, -3, 1}, 0.3, 1.5, rotation),
        Obstacle::convexHull({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {1, 2, 0}, rotation)};
    std::mt19937 rng(936);
    std::uniform_real_distribution<double> position(-6.0, 6.0);
    for (int i = 0; i < 25; ++i)
        initial.push_back(Obstacle::sphere({position(rng), position(rng), position(rng)}, 0.25));
    for (int mode = 0; mode < 3; ++mode) {
        for (bool keep : {false, true}) {
            for (bool noisy : {false, true}) {
                for (bool crop : {false, true}) {
                    SensorConfig config;
                    config.mode = static_cast<SensorConfig::Mode>(mode);
                    config.range = 7;
                    config.min_range = 0.2;
                    config.h_fov_deg = mode == 2 ? 100 : 270;
                    config.v_fov_deg = 90;
                    config.h_res = 48;
                    config.v_res = 9;
                    config.width = 20;
                    config.height = 12;
                    config.surface_spacing = 0.25;
                    config.penetrating_keep_buried = keep;
                    config.penetrating_heading_crop = crop;
                    config.heading_cos_min = 0.3;
                    config.noise_std = noisy ? 0.015 : 0;
                    config.seed = 77;
                    WorldLidar lidar(config);
                    auto obstacles = initial;
                    for (int revision = 0; revision < 10; ++revision) {
                        if (revision == 1 || revision == 2)
                            obstacles[0].position.y() += 0.25;
                        if (revision == 3)
                            obstacles[1].radius *= 1.3;
                        if (revision == 4)
                            obstacles.push_back(Obstacle::box({-1, -1, 1}, {1, 2, 1}));
                        if (revision == 5)
                            obstacles.erase(obstacles.begin() + 2);
                        if (revision == 6)
                            std::reverse(obstacles.begin(), obstacles.end());
                        if (revision == 7)
                            obstacles.clear();
                        if (revision == 8)
                            obstacles = initial;
#ifdef PR6_INCREMENTAL
                        auto scene =
                            std::make_shared<LidarScene>(obstacles, 0.25, keep, *lidar.scene());
                        lidar.setScene(scene);
#else
                        lidar.setScene(obstacles);
#endif
                        cloud(lidar.globalMap(0.25));
                        for (int scan = 0; scan < 3; ++scan) {
                            const Eigen::Vector3d origin(scan - 1.0, 0.5 * revision, 0.2);
                            const std::vector<VehicleBody> others{{4, {2, 1, 1}, 0.4},
                                                                  {9, {100, 100, 0}, 0.5}};
                            const auto tagged = lidar.scanTagged(origin, rotation, others);
                            write(static_cast<uint64_t>(tagged.size()));
                            for (const auto& p : tagged) {
                                point(p.point);
                                write(p.vehicle_id);
                            }
                            if (mode != 1) {
                                const auto beams = lidar.scanWithBeams(origin, rotation, others);
                                write(static_cast<uint64_t>(beams.size()));
                                for (const auto& b : beams) {
                                    point(b.origin);
                                    point(b.direction);
                                    write(b.hit);
                                    write(b.range);
                                    write(b.vehicle_id);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
