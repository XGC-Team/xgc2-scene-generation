// Fleet scan benchmark of the shared static-cloud CPU crop (shared_cloud_cpu_node's query path),
// without ROS: what one node tick costs for N robots, measured the way the node does the work
// (scan, then the PointCloud2 data copy and one more copy standing in for serialization).
// Not a test; run it by hand:
//
//   bench_shared_cloud_cpu [--robots N] [--ticks T] [--threads W] [--scale S]
//                          [--reference-sorted] [--rate HZ]
//
// The map is a replica of the native Swarm-Formation forest (60 columns of 0.1 m cells and 20
// rings over 34 m x 15 m), --scale S multiplies the side lengths and the counts per area. The
// sensor is the Swarm-Formation CPU crop (5 m, body-X cone 0.5, z slab). --threads W scans the
// robots of a tick on a ScanPool of W threads through SharedCloudCpu::scanBatch (1: serial).
// --reference-sorted runs the previous query instead (sorted radius search over all candidates,
// then the crop), one scan per job on the same pool. The checksum covers every output byte, so
// two runs that print the same checksum produced identical clouds point for point and in order.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <random>
#include <string>
#include <vector>

#include "xgc2_world_lidar/scan_pool.h"
#include "xgc2_world_lidar/shared_cloud_cpu.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using xgc2_world_lidar::CropResult;
using xgc2_world_lidar::ScanPool;
using xgc2_world_lidar::ScanPose;
using xgc2_world_lidar::SensorMetadata;
using xgc2_world_lidar::SharedCloudCpu;

double millis(Clock::duration d) {
    return std::chrono::duration<double, std::milli>(d).count();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr forest(double scale) {
    std::minstd_rand0 eng(7);
    std::uniform_real_distribution<double> rx(-17 * scale, 17 * scale),
        ry(-7.5 * scale, 7.5 * scale), rh(2, 3), inflation(0.5, 1.5), rz(0.7, 3), theta(-0.5, 0.5),
        r12(1, 1.2);
    const double res = 0.1;
    const int count = static_cast<int>(60 * scale * scale);
    std::vector<std::array<double, 2>> accepted;
    accepted.reserve(static_cast<std::size_t>(count));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < count; ++i) {
        double x = rx(eng), y = ry(eng);
        const double f = inflation(eng);
        bool close = false;
        for (const auto& p : accepted)
            if (std::hypot(x - p[0], y - p[1]) < 0.8) {
                close = true;
                break;
            }
        if (close) {
            --i;
            continue;
        }
        accepted.push_back({x, y});
        x = std::floor(x / res) * res + res / 2;
        y = std::floor(y / res) * res + res / 2;
        const int width = static_cast<int>(std::ceil(0.5 * f / res));
        const double radius = 0.5 * f / 2;
        for (int r = static_cast<int>(-width / 2.0); r < width / 2.0; ++r)
            for (int s = static_cast<int>(-width / 2.0); s < width / 2.0; ++s) {
                const int height = static_cast<int>(std::ceil(rh(eng) / res));
                const double px = x + (r + 0.5) * res + 0.01, py = y + (s + 0.5) * res + 0.01;
                if (std::hypot(px - x, py - y) > radius)
                    continue;
                for (int t = -10; t < height; ++t)
                    cloud->push_back({static_cast<float>(px),
                                      static_cast<float>(py),
                                      static_cast<float>((t + 0.5) * res + 0.01)});
            }
    }
    for (int i = 0; i < static_cast<int>(20 * scale * scale); ++i) {
        double x = rx(eng), y = ry(eng), z = rz(eng);
        x = std::floor(x / res) * res + res / 2;
        y = std::floor(y / res) * res + res / 2;
        z = std::floor(z / res) * res + res / 2;
        const double a0 = theta(eng), r1 = r12(eng), r2 = r12(eng);
        for (int k = 0; k * (res / 2) < 6.282; ++k) {
            const double a = k * (res / 2);
            cloud->push_back({static_cast<float>(x - std::sin(a0) * r1 * std::cos(a)),
                              static_cast<float>(y + std::cos(a0) * r1 * std::cos(a)),
                              static_cast<float>(z + r2 * std::sin(a))});
        }
    }
    return cloud;
}

// The previous query: PCL's sorted radius search, then the crop in candidate order.
class SortedReference {
public:
    SortedReference(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& world, const SensorMetadata& m)
        : m_(m), tree_(true) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setLeafSize(m.prevoxel_leaf_m[0], m.prevoxel_leaf_m[1], m.prevoxel_leaf_m[2]);
        voxel.setInputCloud(world);
        voxel.filter(*voxels_);
        tree_.setInputCloud(voxels_);
    }
    void scanInto(const ScanPose& pose, CropResult* result) const {
        result->cloud.points.clear();
        result->indices.clear();
        result->squared_distances.clear();
        const Eigen::Vector3d body_x = pose.orientation.toRotationMatrix().col(0);
        const pcl::PointXYZ search(static_cast<float>(pose.position.x()),
                                   static_cast<float>(pose.position.y()),
                                   static_cast<float>(pose.position.z()));
        tree_.radiusSearch(search, m_.range_m, result->indices, result->squared_distances);
        result->radius_candidates = result->indices.size();
        for (auto index : result->indices) {
            const auto& p = voxels_->points[index];
            if (std::abs(p.z - pose.position.z()) / m_.range_m > *m_.vertical_slab_tan)
                continue;
            const Eigen::Vector3d delta(
                p.x - pose.position.x(), p.y - pose.position.y(), p.z - pose.position.z());
            if (delta.normalized().dot(body_x) < *m_.heading_cos_min)
                continue;
            result->cloud.points.push_back(p);
        }
        result->cloud.width = result->cloud.points.size();
    }

private:
    SensorMetadata m_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxels_{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::search::KdTree<pcl::PointXYZ> tree_;
};

struct Slot { // per worker: scan scratch for the reference path, message and wire copies
    CropResult scratch;
    std::vector<uint8_t> message, wire;
};

} // namespace

int main(int argc, char** argv) {
    int robots = 20, ticks = 200, threads = 1;
    double scale = 1.0, rate = 12.0;
    bool reference = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto value = [&] { return i + 1 < argc ? std::atof(argv[++i]) : 0.0; };
        if (a == "--robots")
            robots = static_cast<int>(value());
        else if (a == "--ticks")
            ticks = static_cast<int>(value());
        else if (a == "--threads")
            threads = static_cast<int>(value());
        else if (a == "--scale")
            scale = value();
        else if (a == "--rate")
            rate = value();
        else if (a == "--reference-sorted")
            reference = true;
        else {
            std::fprintf(stderr, "unknown option %s\n", a.c_str());
            return 2;
        }
    }
    SensorMetadata m;
    m.observation_model = "crop_through";
    m.backend = "cpu";
    m.prevoxel_leaf_m = {0.1f, 0.1f, 0.1f};
    m.range_m = 5.0;
    m.heading_cos_min = 0.5;
    m.vertical_slab_tan = 0.5773502691896257;
    m.publish_rate_hz = rate;
    m.frame_id = "map";
    m.stamp_policy = "zero";
    const auto world = forest(scale);
    SharedCloudCpu cpu;
    const auto t0 = Clock::now();
    cpu.load(world, m);
    const double load_ms = millis(Clock::now() - t0);
    const SortedReference old_path(world, m);

    std::mt19937_64 g(3);
    std::uniform_real_distribution<double> ux(-17 * scale, 17 * scale),
        uy(-7.5 * scale, 7.5 * scale), uz(0.5, 2), yaw(-3.14, 3.14);
    std::vector<ScanPose> poses(robots);
    for (auto& p : poses) {
        p.position = {ux(g), uy(g), uz(g)};
        p.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw(g), Eigen::Vector3d::UnitZ()));
    }
    ScanPool pool(static_cast<std::size_t>(threads));
    std::vector<Slot> slots(pool.threads());
    std::vector<std::vector<uint8_t>> wire_of(robots);
    unsigned long long checksum = 1469598103934665603ULL;
    std::size_t candidates = 0, kept = 0;
    std::vector<double> tick_ms;
    auto publish = [&](std::size_t index, std::size_t worker, const CropResult& scan) {
        Slot& slot = slots[worker];
        const std::size_t bytes = scan.cloud.size() * sizeof(pcl::PointXYZ);
        slot.message.resize(bytes); // PointCloud2.data fill
        if (bytes)
            std::memcpy(slot.message.data(), scan.cloud.points.data(), bytes);
        slot.wire.resize(bytes); // stands in for the serialization copy inside publish()
        if (bytes)
            std::memcpy(slot.wire.data(), slot.message.data(), bytes);
        wire_of[index] = slot.wire;
    };
    for (int t = 0; t < ticks + 20; ++t) {
        for (auto& p : poses) {
            p.position.x() += 0.125; // 1.5 m/s at 12 Hz
            if (p.position.x() > 17 * scale)
                p.position.x() = -17 * scale;
        }
        const auto a = Clock::now();
        std::vector<std::size_t> radius(robots), count(robots);
        if (reference) {
            pool.runWithWorker(poses.size(), [&](std::size_t i, std::size_t worker) {
                old_path.scanInto(poses[i], &slots[worker].scratch);
                radius[i] = slots[worker].scratch.radius_candidates;
                count[i] = slots[worker].scratch.cloud.size();
                publish(i, worker, slots[worker].scratch);
            });
        } else {
            cpu.scanBatch(pool, poses, [&](std::size_t i, std::size_t worker, const CropResult& s) {
                radius[i] = s.radius_candidates;
                count[i] = s.cloud.size();
                publish(i, worker, s);
            });
        }
        const double d = millis(Clock::now() - a);
        if (t < 20)
            continue; // warm-up
        tick_ms.push_back(d);
        for (int i = 0; i < robots; ++i) {
            candidates += radius[i];
            kept += count[i];
            for (uint8_t byte : wire_of[i])
                checksum = (checksum ^ byte) * 1099511628211ULL;
            checksum ^= wire_of[i].size();
        }
    }
    std::sort(tick_ms.begin(), tick_ms.end());
    double sum = 0;
    for (double d : tick_ms)
        sum += d;
    const double mean = sum / static_cast<double>(tick_ms.size());
    const double scans = static_cast<double>(ticks) * robots;
    std::printf("%s: map %zu points, %zu voxels, load %.1f ms | %d robots, %d threads, %.0f Hz | "
                "per scan %.0f candidates, %.0f kept (%.0f KB at point_step 16) | tick mean %.3f "
                "p50 %.3f p99 %.3f ms = %.1f us per scan, %.1f%% of the %.1f ms period | "
                "checksum %016llx\n",
                reference ? "reference-sorted" : "scanBatch",
                world->size(),
                cpu.voxel_count(),
                load_ms,
                robots,
                threads,
                rate,
                static_cast<double>(candidates) / scans,
                static_cast<double>(kept) / scans,
                static_cast<double>(kept) / scans * 16.0 / 1024.0,
                mean,
                tick_ms[tick_ms.size() / 2],
                tick_ms[tick_ms.size() * 99 / 100],
                mean * 1000.0 / robots,
                mean * rate / 10.0,
                1000.0 / rate,
                checksum);
    return 0;
}
