// Plain-assert tests for the ROS-free shared static-cloud CPU crop. Exit status 0 iff every
// check holds.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <vector>

#include "xgc2_world_lidar/scan_pool.h"
#include "xgc2_world_lidar/shared_cloud_cpu.hpp"

namespace {

int g_failures = 0;
int g_checks = 0;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        ++g_checks;                                                                                \
        if (!(cond)) {                                                                             \
            ++g_failures;                                                                          \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                          \
        }                                                                                          \
    } while (0)

using xgc2_world_lidar::CropResult;
using xgc2_world_lidar::ScanPool;
using xgc2_world_lidar::ScanPose;
using xgc2_world_lidar::SensorMetadata;
using xgc2_world_lidar::SharedCloudCpu;

// A forest of dense columns on a 0.1 m grid plus a few thin rings: the structure of the native
// Swarm-Formation world, at a size a test can scan many times.
pcl::PointCloud<pcl::PointXYZ>::Ptr forest(unsigned seed, int columns) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> ux(-17.0f, 17.0f), uy(-7.5f, 7.5f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int c = 0; c < columns; ++c) {
        const float x = std::floor(ux(rng) * 10.0f) / 10.0f;
        const float y = std::floor(uy(rng) * 10.0f) / 10.0f;
        for (int i = -2; i < 3; ++i)
            for (int j = -2; j < 3; ++j)
                for (int k = 0; k < 35; ++k)
                    cloud->push_back({x + 0.1f * i, y + 0.1f * j, -1.0f + 0.1f * k});
    }
    for (int r = 0; r < 6; ++r) {
        const float x = ux(rng), y = uy(rng), z = 1.0f + 0.2f * r;
        for (float a = 0.0f; a < 6.282f; a += 0.05f)
            cloud->push_back({x + std::cos(a), y + 0.1f * std::sin(a), z + std::sin(a)});
    }
    return cloud;
}

SensorMetadata swarmMetadata() {
    SensorMetadata m; // Swarm-Formation / FLIP CPU crop as in shared_cpu_through.yaml
    m.observation_model = "crop_through";
    m.backend = "cpu";
    m.prevoxel_leaf_m = {0.1f, 0.1f, 0.1f};
    m.range_m = 5.0;
    m.heading_cos_min = 0.5;
    m.vertical_slab_tan = 0.5773502691896257;
    m.publish_rate_hz = 12.0;
    m.frame_id = "map";
    m.stamp_policy = "zero";
    return m;
}

std::vector<ScanPose> randomPoses(unsigned seed, int count) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> x(-19.0, 19.0), y(-9.0, 9.0), z(0.2, 2.5);
    std::normal_distribution<double> n(0.0, 1.0);
    std::vector<ScanPose> poses;
    for (int i = 0; i < count; ++i)
        poses.push_back({{x(rng), y(rng), z(rng)},
                         Eigen::Quaterniond(n(rng), n(rng), n(rng), n(rng)).normalized()});
    return poses;
}

// The bytes a node would publish for one scan: the kept points, 12 bytes each, in order.
std::vector<uint8_t> bytesOf(const CropResult& scan) {
    std::vector<uint8_t> out(scan.cloud.size() * sizeof(pcl::PointXYZ));
    if (!out.empty())
        std::memcpy(out.data(), scan.cloud.points.data(), out.size());
    return out;
}

// Reference: every pose through scanInto() on one thread with one scratch, the pre-batch path.
std::vector<std::vector<uint8_t>> serialReference(const SharedCloudCpu& cpu,
                                                  const std::vector<ScanPose>& poses,
                                                  std::vector<std::size_t>* candidates) {
    std::vector<std::vector<uint8_t>> out;
    CropResult scratch;
    for (const auto& pose : poses) {
        cpu.scanInto(pose.position, pose.orientation, &scratch);
        out.push_back(bytesOf(scratch));
        if (candidates)
            candidates->push_back(scratch.radius_candidates);
    }
    return out;
}

void testThreadCount() {
    using xgc2_world_lidar::defaultCloudScanThreads;
    CHECK(defaultCloudScanThreads(100, 32) == 8);
    CHECK(defaultCloudScanThreads(100, 16) == 4);
    CHECK(defaultCloudScanThreads(100, 8) == 2); // the 8-core development box
    CHECK(defaultCloudScanThreads(100, 4) == 1);
    CHECK(defaultCloudScanThreads(100, 2) == 1);
    CHECK(defaultCloudScanThreads(100, 0) == 1); // unknown hardware: serial
    CHECK(defaultCloudScanThreads(3, 64) == 3);  // never more threads than sensors
    CHECK(defaultCloudScanThreads(7, 8) == 2);   // the seven-robot Swarm profile
    CHECK(defaultCloudScanThreads(0, 16) == 1);
}

void testPoolWorkerIndex() {
    for (std::size_t threads : {1, 2, 5}) {
        ScanPool pool(threads);
        CHECK(pool.threads() == threads);
        std::vector<std::atomic<int>> runs(64);
        std::vector<std::size_t> worker_of(64, 99);
        for (int batch = 0; batch < 20; ++batch) {
            for (auto& r : runs)
                r = 0;
            pool.runWithWorker(runs.size(), [&](std::size_t i, std::size_t worker) {
                ++runs[i];
                worker_of[i] = worker;
            });
            bool once = true, in_range = true;
            for (std::size_t i = 0; i < runs.size(); ++i) {
                once = once && runs[i] == 1;
                in_range = in_range && worker_of[i] < pool.threads();
            }
            CHECK(once);
            CHECK(in_range);
        }
        if (threads == 1) {
            bool inline_zero = true;
            for (std::size_t w : worker_of)
                inline_zero = inline_zero && w == 0;
            CHECK(inline_zero);
        }
    }
    // run() keeps its contract on top of runWithWorker().
    ScanPool pool(3);
    std::atomic<int> sum{0};
    pool.run(10, [&](std::size_t i) { sum += static_cast<int>(i); });
    CHECK(sum == 45);
}

// scanBatch() must hand every sink the same points as scanInto(), at every pool size.
void testBatchEqualsSerial() {
    SharedCloudCpu cpu;
    cpu.load(forest(3, 60), swarmMetadata());
    CHECK(cpu.voxel_count() > 40000);
    const auto poses = randomPoses(11, 100);
    std::vector<std::size_t> candidates;
    const auto reference = serialReference(cpu, poses, &candidates);
    std::size_t nonempty = 0, zero_neighbour = 0;
    for (std::size_t i = 0; i < poses.size(); ++i) {
        nonempty += reference[i].empty() ? 0 : 1;
        zero_neighbour += candidates[i] == 0 ? 1 : 0;
    }
    CHECK(nonempty > 20); // the poses really hit the map
    for (std::size_t threads : {1, 2, 3, 8}) {
        ScanPool pool(threads);
        for (int repeat = 0; repeat < 3; ++repeat) { // scratch reuse across batches
            std::vector<std::vector<uint8_t>> got(poses.size());
            std::vector<std::size_t> got_candidates(poses.size(), 12345);
            std::vector<int> calls(poses.size(), 0);
            std::vector<std::size_t> order;
            std::mutex order_mutex;
            std::atomic<bool> worker_ok{true};
            cpu.scanBatch(
                pool, poses, [&](std::size_t i, std::size_t worker, const CropResult& scan) {
                    if (worker >= pool.threads())
                        worker_ok = false;
                    got[i] = bytesOf(scan);
                    got_candidates[i] = scan.radius_candidates;
                    ++calls[i];
                    std::lock_guard<std::mutex> lock(order_mutex);
                    order.push_back(i);
                });
            CHECK(worker_ok);
            CHECK(got == reference);
            CHECK(got_candidates == candidates);
            CHECK(std::all_of(calls.begin(), calls.end(), [](int c) { return c == 1; }));
            if (threads == 1) { // exactly the serial order
                bool ascending = order.size() == poses.size();
                for (std::size_t k = 0; k < order.size(); ++k)
                    ascending = ascending && order[k] == k;
                CHECK(ascending);
            }
        }
    }
    (void)zero_neighbour;
    // An empty batch calls no sink; a pose outside the map reports zero candidates.
    ScanPool pool(2);
    int calls = 0;
    cpu.scanBatch(pool, {}, [&](std::size_t, std::size_t, const CropResult&) { ++calls; });
    CHECK(calls == 0);
    const std::vector<ScanPose> far = {{{500.0, 500.0, 500.0}, Eigen::Quaterniond::Identity()}};
    cpu.scanBatch(pool, far, [&](std::size_t, std::size_t, const CropResult& scan) {
        ++calls;
        CHECK(scan.radius_candidates == 0);
        CHECK(scan.cloud.empty());
    });
    CHECK(calls == 1);
}

// A batch before load() is a defined no-publication state, as scanInto() before load().
void testBatchBeforeLoad() {
    SharedCloudCpu cpu;
    ScanPool pool(2);
    std::atomic<int> calls{0}, with_candidates{0};
    cpu.scanBatch(pool, randomPoses(1, 4), [&](std::size_t, std::size_t, const CropResult& scan) {
        ++calls;
        with_candidates += scan.radius_candidates ? 1 : 0;
    });
    CHECK(calls == 4);
    CHECK(with_candidates == 0);
}

} // namespace

int main() try {
    testThreadCount();
    testPoolWorkerIndex();
    testBatchEqualsSerial();
    testBatchBeforeLoad();
    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
} catch (const std::exception& error) {
    std::fprintf(stderr, "shared cloud cpu test exception: %s\n", error.what());
    return 1;
}
