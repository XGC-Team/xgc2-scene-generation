// Plain-assert tests for the ROS-free shared static-cloud CPU crop. Exit status 0 iff every
// check holds.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>

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
                    cloud->push_back({x + 0.1f * static_cast<float>(i),
                                      y + 0.1f * static_cast<float>(j),
                                      -1.0f + 0.1f * static_cast<float>(k)});
    }
    for (int r = 0; r < 6; ++r) {
        const float x = ux(rng), y = uy(rng), z = 1.0f + 0.2f * static_cast<float>(r);
        for (int k = 0; k < 126; ++k) {
            const float a = 0.05f * static_cast<float>(k);
            cloud->push_back({x + std::cos(a), y + 0.1f * std::sin(a), z + std::sin(a)});
        }
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
    poses.reserve(static_cast<std::size_t>(count));
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
    out.reserve(poses.size());
    CropResult scratch;
    for (const auto& pose : poses) {
        cpu.scanInto(pose.position, pose.orientation, &scratch);
        out.push_back(bytesOf(scratch));
        if (candidates)
            candidates->push_back(scratch.radius_candidates);
    }
    return out;
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
    std::size_t nonempty = 0;
    for (std::size_t i = 0; i < poses.size(); ++i) {
        nonempty += reference[i].empty() ? 0 : 1;
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

// The crop as it was before the unsorted search: PCL's sorted radius search over the voxelized
// map, then the crop predicates in candidate order. Independent of SharedCloudCpu's code.
class SortedSearchReference {
public:
    SortedSearchReference(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& world,
                          const SensorMetadata& metadata)
        : m_(metadata), tree_(true) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setLeafSize(m_.prevoxel_leaf_m[0], m_.prevoxel_leaf_m[1], m_.prevoxel_leaf_m[2]);
        voxel.setInputCloud(world);
        voxel.filter(*voxels_);
        tree_.setInputCloud(voxels_);
    }
    std::vector<uint8_t> scan(const ScanPose& pose, std::size_t* candidates, bool* ties) const {
        std::vector<int> indices;
        std::vector<float> squared;
        const Eigen::Vector3d body_x = pose.orientation.toRotationMatrix().col(0);
        const pcl::PointXYZ search(static_cast<float>(pose.position.x()),
                                   static_cast<float>(pose.position.y()),
                                   static_cast<float>(pose.position.z()));
        tree_.radiusSearch(search, m_.range_m, indices, squared);
        *candidates = indices.size();
        for (std::size_t n = 1; n < squared.size(); ++n)
            *ties = *ties || squared[n] == squared[n - 1];
        std::vector<uint8_t> out;
        for (int index : indices) {
            const auto& p = voxels_->points[index];
            if (m_.vertical_slab_tan &&
                std::abs(p.z - pose.position.z()) / m_.range_m > *m_.vertical_slab_tan)
                continue;
            if (m_.heading_cos_min) {
                const Eigen::Vector3d delta(
                    p.x - pose.position.x(), p.y - pose.position.y(), p.z - pose.position.z());
                if (delta.normalized().dot(body_x) < *m_.heading_cos_min)
                    continue;
            }
            const auto* bytes = reinterpret_cast<const uint8_t*>(&p);
            out.insert(out.end(), bytes, bytes + sizeof(pcl::PointXYZ));
        }
        return out;
    }

private:
    SensorMetadata m_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxels_{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::search::KdTree<pcl::PointXYZ> tree_;
};

// Sorting only the kept points must give the sequence the sorted search gave, byte for byte,
// including among points at exactly equal distance (the index breaks those ties).
void testOrderEqualsSortedSearch() {
    const SensorMetadata swarm = swarmMetadata();
    SensorMetadata ego = swarmMetadata();
    ego.heading_cos_min = 0.0; // EGO-Planner-v2 half-space
    SensorMetadata ball = swarmMetadata();
    ball.heading_cos_min.reset(); // Primitive: ball only
    ball.vertical_slab_tan.reset();
    ball.range_m = 5.0;
    for (const SensorMetadata& metadata : {swarm, ego, ball}) {
        const auto world = forest(5, 60);
        SharedCloudCpu cpu;
        cpu.load(world, metadata);
        const SortedSearchReference reference(world, metadata);
        CropResult scratch;
        bool ties = false;
        std::size_t compared = 0, nonempty = 0;
        for (const auto& pose : randomPoses(23, 1500)) {
            std::size_t candidates = 0;
            const auto expected = reference.scan(pose, &candidates, &ties);
            cpu.scanInto(pose.position, pose.orientation, &scratch);
            CHECK(bytesOf(scratch) == expected);
            CHECK(scratch.radius_candidates == candidates);
            ++compared;
            nonempty += expected.empty() ? 0 : 1;
        }
        CHECK(compared == 1500);
        CHECK(nonempty > 300);
        CHECK(ties); // the forest's columns are symmetric, so equal distances do occur
    }
    // A lattice with integer coordinates and sensors on lattice points: the equal squared
    // distances are exact, so most of the radius is tied and only the index orders it.
    pcl::PointCloud<pcl::PointXYZ>::Ptr lattice(new pcl::PointCloud<pcl::PointXYZ>);
    for (int x = -8; x <= 8; ++x)
        for (int y = -8; y <= 8; ++y)
            for (int z = -3; z <= 3; ++z)
                lattice->push_back(
                    {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
    SensorMetadata grid = swarmMetadata();
    grid.prevoxel_leaf_m = {0.5f, 0.5f, 0.5f};
    grid.heading_cos_min = -1.0; // keep every direction: the crop must not hide a tie
    grid.vertical_slab_tan = 10.0;
    SharedCloudCpu cpu;
    cpu.load(lattice, grid);
    const SortedSearchReference reference(lattice, grid);
    CropResult scratch;
    bool ties = false;
    std::mt19937 rng(7);
    for (int q = 0; q < 200; ++q) {
        const ScanPose pose = {{static_cast<double>(static_cast<int>(rng() % 9) - 4),
                                static_cast<double>(static_cast<int>(rng() % 9) - 4),
                                static_cast<double>(static_cast<int>(rng() % 3) - 1)},
                               Eigen::Quaterniond::Identity()};
        std::size_t candidates = 0;
        const auto expected = reference.scan(pose, &candidates, &ties);
        cpu.scanInto(pose.position, pose.orientation, &scratch);
        CHECK(!expected.empty());
        CHECK(bytesOf(scratch) == expected);
    }
    CHECK(ties);
}

std::vector<pcl::PointXYZ> pointsOf(const CropResult& scan) {
    return std::vector<pcl::PointXYZ>(scan.cloud.points.begin(), scan.cloud.points.end());
}

struct Window {
    double min_range, h_fov_deg, v_fov_deg;
};

// Independent window test for one point. `slack` shrinks (+) or grows (-) every limit so a
// point within `slack` of one is neither clearly in nor clearly out.
bool insideWindow(const pcl::PointXYZ& p, const ScanPose& pose, const Window& w, double slack) {
    const Eigen::Vector3d delta(
        p.x - pose.position.x(), p.y - pose.position.y(), p.z - pose.position.z());
    const Eigen::Vector3d local = pose.orientation.toRotationMatrix().transpose() * delta;
    const double kPi = std::acos(-1.0);
    if (w.min_range > 0 && delta.norm() < w.min_range + slack)
        return false;
    if (w.h_fov_deg > 0 && w.h_fov_deg < 360 &&
        std::abs(std::atan2(local.y(), local.x())) > 0.5 * w.h_fov_deg * kPi / 180.0 - slack)
        return false;
    if (w.v_fov_deg > 0 && w.v_fov_deg < 180 &&
        std::abs(std::atan2(local.z(), std::hypot(local.x(), local.y()))) >
            0.5 * w.v_fov_deg * kPi / 180.0 - slack)
        return false;
    return true;
}

// The window keeps exactly the points of the default crop that lie inside it, in the default
// order. Points within `guard` of a limit are left out of both sides (float against double).
void checkWindow(const SharedCloudCpu& windowed,
                 const SharedCloudCpu& unlimited,
                 const std::vector<ScanPose>& poses,
                 const Window& w,
                 double guard,
                 std::size_t* kept_total,
                 std::size_t* dropped_total) {
    CropResult a, b;
    for (const auto& pose : poses) {
        unlimited.scanInto(pose.position, pose.orientation, &a);
        windowed.scanInto(pose.position, pose.orientation, &b);
        std::vector<pcl::PointXYZ> expected, got;
        for (const auto& p : pointsOf(a)) {
            const bool clearly_in = insideWindow(p, pose, w, guard);
            const bool clearly_out = !insideWindow(p, pose, w, -guard);
            if (clearly_in)
                expected.push_back(p);
            *dropped_total += clearly_out ? 1 : 0;
        }
        for (const auto& p : pointsOf(b))
            if (insideWindow(p, pose, w, guard))
                got.push_back(p);
            else
                CHECK(insideWindow(p, pose, w, -guard)); // nothing clearly outside is kept
        CHECK(expected.size() == got.size());
        bool same = expected.size() == got.size();
        for (std::size_t k = 0; same && k < expected.size(); ++k)
            same =
                expected[k].x == got[k].x && expected[k].y == got[k].y && expected[k].z == got[k].z;
        CHECK(same);
        *kept_total += got.size();
    }
}

void testFovAndRange() {
    const auto world = forest(5, 60);
    const SensorMetadata swarm = swarmMetadata();
    SharedCloudCpu unlimited;
    unlimited.load(world, swarm);
    // Unset and explicitly full are the crop before these fields existed, byte for byte.
    SensorMetadata full = swarmMetadata();
    full.h_fov_deg = 360.0;
    full.v_fov_deg = 180.0;
    full.min_range_m = 0.0;
    SharedCloudCpu explicit_full;
    explicit_full.load(world, full);
    const SortedSearchReference reference(world, swarm);
    CropResult a, b;
    std::size_t nonempty = 0;
    for (const auto& pose : randomPoses(31, 600)) {
        std::size_t candidates = 0;
        bool ties = false;
        unlimited.scanInto(pose.position, pose.orientation, &a);
        explicit_full.scanInto(pose.position, pose.orientation, &b);
        CHECK(bytesOf(a) == bytesOf(b));
        CHECK(bytesOf(a) == reference.scan(pose, &candidates, &ties));
        nonempty += a.cloud.empty() ? 0 : 1;
    }
    CHECK(nonempty > 100);
    // Windows on the forest, random poses.
    const auto poses = randomPoses(37, 600);
    for (const Window& w : {Window{0.0, 60.0, 0.0},
                            Window{0.0, 0.0, 40.0},
                            Window{1.5, 0.0, 0.0},
                            Window{1.0, 90.0, 60.0},
                            Window{2.0, 360.0, 30.0}}) {
        SensorMetadata m = swarmMetadata();
        m.min_range_m = w.min_range;
        m.h_fov_deg = w.h_fov_deg;
        m.v_fov_deg = w.v_fov_deg;
        SharedCloudCpu windowed;
        windowed.load(world, m);
        std::size_t kept = 0, dropped = 0;
        checkWindow(windowed, unlimited, poses, w, 1e-5, &kept, &dropped);
        CHECK(kept > 100);    // the window keeps points
        CHECK(dropped > 100); // and it removes points the default crop keeps
    }
    // A lattice with integer coordinates, sensors on lattice points, identity attitude: limits
    // sit exactly on points (inclusive at the near range and at the half-extent angles).
    pcl::PointCloud<pcl::PointXYZ>::Ptr lattice(new pcl::PointCloud<pcl::PointXYZ>);
    for (int x = -8; x <= 8; ++x)
        for (int y = -8; y <= 8; ++y)
            for (int z = -4; z <= 4; ++z)
                lattice->push_back(
                    {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
    SensorMetadata every = swarmMetadata();
    every.prevoxel_leaf_m = {0.5f, 0.5f, 0.5f};
    every.heading_cos_min = -1.0;
    every.vertical_slab_tan = 10.0;
    SharedCloudCpu lattice_all;
    lattice_all.load(lattice, every);
    for (const Window& w : {Window{3.0, 0.0, 0.0},
                            Window{0.0, 90.0, 0.0},
                            Window{0.0, 0.0, 90.0},
                            Window{2.0, 180.0, 90.0}}) {
        SensorMetadata m = every;
        m.min_range_m = w.min_range;
        m.h_fov_deg = w.h_fov_deg;
        m.v_fov_deg = w.v_fov_deg;
        SharedCloudCpu windowed;
        windowed.load(lattice, m);
        std::vector<ScanPose> on_lattice;
        on_lattice.reserve(12);
        for (int q = 0; q < 12; ++q)
            on_lattice.push_back({{static_cast<double>(q % 5 - 2),
                                   static_cast<double>(q % 3 - 1),
                                   static_cast<double>(q % 2)},
                                  Eigen::Quaterniond::Identity()});
        std::size_t kept = 0, dropped = 0;
        checkWindow(windowed, lattice_all, on_lattice, w, 1e-12, &kept, &dropped);
        CHECK(kept > 0);
        // Exactly on a limit counts as inside: compare the counts of the inclusive window.
        CHECK(dropped > 0);
        CHECK(kept + dropped >= 12);
    }
    // The inclusive near range and half-extent: a point exactly 3 m away and one exactly 45
    // degrees off the heading are kept by min_range 3 and h_fov 90, a point just inside is not.
    pcl::PointCloud<pcl::PointXYZ>::Ptr two(new pcl::PointCloud<pcl::PointXYZ>);
    two->push_back({3.0f, 0.0f, 0.0f});
    two->push_back({2.9f, 0.0f, 0.0f});
    two->push_back({3.0f, 3.0f, 0.0f});
    two->push_back({3.0f, 3.3f, 0.0f});
    SensorMetadata edge = swarmMetadata();
    edge.prevoxel_leaf_m = {0.05f, 0.05f, 0.05f};
    edge.heading_cos_min.reset();
    edge.vertical_slab_tan.reset();
    edge.range_m = 10.0;
    edge.min_range_m = 3.0;
    edge.h_fov_deg = 90.0;
    SharedCloudCpu inclusive;
    inclusive.load(two, edge);
    CropResult scan;
    inclusive.scanInto({0, 0, 0}, Eigen::Quaterniond::Identity(), &scan);
    CHECK(scan.radius_candidates == 4);
    CHECK(scan.cloud.size() == 2);
    CHECK(scan.cloud.size() == 2 && scan.cloud.points[0].x == 3.0f &&
          scan.cloud.points[0].y == 0.0f);
    CHECK(scan.cloud.size() == 2 && scan.cloud.points[1].x == 3.0f &&
          scan.cloud.points[1].y == 3.0f);
}

void testSpecValidation() {
    using xgc2_world_lidar::validateSensorMetadata;
    auto rejected = [](const SensorMetadata& m) {
        try {
            validateSensorMetadata(m);
        } catch (const std::invalid_argument&) {
            return true;
        }
        return false;
    };
    CHECK(!rejected(swarmMetadata()));
    SensorMetadata m = swarmMetadata();
    m.h_fov_deg = 120;
    m.v_fov_deg = 60;
    m.min_range_m = 0.5;
    CHECK(!rejected(m));
    m = swarmMetadata();
    m.h_fov_deg = 360;
    m.v_fov_deg = 180;
    CHECK(!rejected(m));
    for (double bad : {-1.0, 361.0, std::nan(""), std::numeric_limits<double>::infinity()}) {
        m = swarmMetadata();
        m.h_fov_deg = bad;
        CHECK(rejected(m));
    }
    for (double bad : {-1.0, 181.0, std::nan("")}) {
        m = swarmMetadata();
        m.v_fov_deg = bad;
        CHECK(rejected(m));
    }
    for (double bad : {-0.1, 5.0, 6.0, std::nan("")}) { // range_m is 5
        m = swarmMetadata();
        m.min_range_m = bad;
        CHECK(rejected(m));
    }
    // lidar_scan's grid is refused here instead of being ignored.
    m = swarmMetadata();
    m.h_res = 360;
    CHECK(rejected(m));
    m = swarmMetadata();
    m.v_res = 16;
    CHECK(rejected(m));
    m = swarmMetadata();
    m.point_cover_spacing_m = 0.1;
    CHECK(rejected(m));
    // The orthogonal model and backend fields still select one implementation.
    m = swarmMetadata();
    m.backend = "gpu";
    CHECK(rejected(m));
    m = swarmMetadata();
    m.observation_model = "lidar_scan";
    CHECK(rejected(m));
}

} // namespace

int main() try {
    testPoolWorkerIndex();
    testBatchEqualsSerial();
    testBatchBeforeLoad();
    testOrderEqualsSortedSearch();
    testFovAndRange();
    testSpecValidation();
    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
} catch (const std::exception& error) {
    std::fprintf(stderr, "shared cloud cpu test exception: %s\n", error.what());
    return 1;
}
