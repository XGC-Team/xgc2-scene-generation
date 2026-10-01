// Fleet scan benchmark of the ROS-free core: what one world_lidar fleet tick
// costs for N robots, measured the way the node does the work (scan, then the
// PointCloud2 float32 x y z records). Not a test; run it by hand:
//
//   bench_world_lidar [--parts FILE] [--robots N] [--ticks T]
//                     [--mode penetrating|raycast] [--path tagged|into]
//                     [--threads W] [--seed S]
//
// Without --parts the scene is a synthetic stand-in for the native
// Swarm-Formation forest (60 pillars, 20 rings of 126 thin capsules: 2580
// parts). --parts reads one part per line, as written by
//
//   import json, sys
//   for o in json.load(open(sys.argv[1]))['obstacles']:
//       op = o['pose']['position']; oq = o['pose'].get('orientation', [0, 0, 0, 1])
//       for p in o['parts']:
//           g = p['geometry']; pp = p.get('pose', {}).get('position', [0, 0, 0])
//           pq = p.get('pose', {}).get('orientation', [0, 0, 0, 1]); v = g.get('vertices', [])
//           print(*[g['type'], *op, *oq, *pp, *pq, g.get('radius', 0), g.get('height', 0),
//                   *g.get('size', [0, 0, 0]), len(v), *[c for x in v for c in x]])
//
// (quaternions x y z w). Robots start on a grid over the scene and move along
// +x by 0.075 m per tick (1.5 m/s at 20 Hz). The sensor is the lightweight
// default (bridge_equivalent: penetrating, 8 m, full sphere, buried samples
// kept) or a 360 x 32 raycast over the same range. --path tagged scans with
// scanTagged() and packs the records the way the node used to; --path into
// writes them in place with scanInto() into a buffer reused across ticks.
// --threads W scans the robots of a tick on a ScanPool of W threads, as the
// node does (0: the node's default for this many robots on this host).
// The checksum covers every output byte, so two builds or paths that print
// the same checksum produced identical clouds point for point.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "xgc2_world_lidar/scan_pool.h"
#include "xgc2_world_lidar/scene_conversion.h"
#include "xgc2_world_lidar/world_lidar.h"

namespace {

using xgc2_world_lidar::LidarScene;
using xgc2_world_lidar::Obstacle;
using xgc2_world_lidar::SceneObstacleDescription;
using xgc2_world_lidar::ScenePartDescription;
using xgc2_world_lidar::SensorConfig;
using xgc2_world_lidar::TaggedPoint;
using xgc2_world_lidar::WorldLidar;
using Clock = std::chrono::steady_clock;

double msSince(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

// Process CPU time (every thread) in ms: steadier than wall time on a busy host.
double cpuMs() {
    return 1e3 * static_cast<double>(std::clock()) / CLOCKS_PER_SEC;
}

double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
}

std::vector<SceneObstacleDescription> readParts(const std::string& path) {
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error("cannot read " + path);
    std::vector<SceneObstacleDescription> scene;
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream s(line);
        std::string type;
        double op[3], oq[4], pp[3], pq[4], radius, height, size[3];
        int vertices = 0;
        s >> type >> op[0] >> op[1] >> op[2] >> oq[0] >> oq[1] >> oq[2] >> oq[3] >> pp[0] >>
            pp[1] >> pp[2] >> pq[0] >> pq[1] >> pq[2] >> pq[3] >> radius >> height >> size[0] >>
            size[1] >> size[2] >> vertices;
        if (!s)
            continue;
        ScenePartDescription part;
        part.type = type;
        part.pose.position = Eigen::Vector3d(pp[0], pp[1], pp[2]);
        part.pose.orientation = Eigen::Quaterniond(pq[3], pq[0], pq[1], pq[2]);
        part.radius = radius;
        part.height = height;
        part.size = Eigen::Vector3d(size[0], size[1], size[2]);
        for (int k = 0; k < vertices; ++k) {
            Eigen::Vector3d v;
            s >> v.x() >> v.y() >> v.z();
            part.vertices.push_back(v);
        }
        SceneObstacleDescription obstacle;
        obstacle.id = "part" + std::to_string(scene.size());
        obstacle.pose.position = Eigen::Vector3d(op[0], op[1], op[2]);
        obstacle.pose.orientation = Eigen::Quaterniond(oq[3], oq[0], oq[1], oq[2]);
        obstacle.parts.push_back(part);
        scene.push_back(obstacle);
    }
    return scene;
}

// 60 pillars (r 0.15..0.4 m, 4 m tall) and 20 tilted rings of 126 capsules
// (r 0.05 m, about 1 m ring radius) over x [-16, 16], y [-8, 8].
std::vector<Obstacle> syntheticForest(unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> x(-16.0, 16.0), y(-8.0, 8.0), u(0.0, 1.0);
    std::vector<Obstacle> out;
    out.reserve(60 + 20 * 126); // pillars, then ring segments
    for (int i = 0; i < 60; ++i)
        out.push_back(
            Obstacle::cylinder(Eigen::Vector3d(x(rng), y(rng), 1.0), 0.15 + 0.25 * u(rng), 4.0));
    const double pi = 3.14159265358979323846;
    const int segments = 126;
    for (int r = 0; r < 20; ++r) {
        const Eigen::Vector3d center(x(rng), y(rng), 0.8 + 1.6 * u(rng));
        const Eigen::Quaterniond tilt =
            Eigen::Quaterniond(Eigen::AngleAxisd(2.0 * pi * u(rng), Eigen::Vector3d::UnitZ())) *
            Eigen::Quaterniond(Eigen::AngleAxisd(0.6 * u(rng), Eigen::Vector3d::UnitX()));
        for (int k = 0; k < segments; ++k) {
            const double a0 = 2.0 * pi * k / segments, a1 = 2.0 * pi * (k + 1) / segments;
            const Eigen::Vector3d p0 =
                center + tilt * Eigen::Vector3d(std::cos(a0), std::sin(a0), 0);
            const Eigen::Vector3d p1 =
                center + tilt * Eigen::Vector3d(std::cos(a1), std::sin(a1), 0);
            const Eigen::Quaterniond along =
                Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), p1 - p0);
            out.push_back(Obstacle::capsule(0.5 * (p0 + p1), 0.05, (p1 - p0).norm(), along));
        }
    }
    return out;
}

struct Fnv {
    uint64_t h = 1469598103934665603ull;
    void add(const void* data, std::size_t n) {
        const auto* p = static_cast<const unsigned char*>(data);
        for (std::size_t i = 0; i < n; ++i)
            h = (h ^ p[i]) * 1099511628211ull;
    }
};

// The node's cloud records: float32 x y z per point.
void pack(const std::vector<TaggedPoint>& points, std::vector<uint8_t>* data) {
    data->resize(12 * points.size());
    uint8_t* out = data->data();
    for (const auto& p : points) {
        const float f[3] = {static_cast<float>(p.point.x()),
                            static_cast<float>(p.point.y()),
                            static_cast<float>(p.point.z())};
        std::memcpy(out, f, sizeof f);
        out += sizeof f;
    }
}

int run(int argc, char** argv) {
    std::string parts, mode = "penetrating", path = "tagged";
    int robots = 7, ticks = 40, threads = 1;
    unsigned seed = 7;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string key = argv[i], value = argv[i + 1];
        if (key == "--parts")
            parts = value;
        else if (key == "--robots")
            robots = std::max(1, std::atoi(value.c_str()));
        else if (key == "--ticks")
            ticks = std::max(1, std::atoi(value.c_str()));
        else if (key == "--mode")
            mode = value;
        else if (key == "--threads")
            threads = std::max(0, std::atoi(value.c_str()));
        else if (key == "--path")
            path = value;
        else if (key == "--seed")
            seed = static_cast<unsigned>(std::atoi(value.c_str()));
        else {
            std::fprintf(stderr, "unknown option %s\n", key.c_str());
            return 2;
        }
    }
    const std::vector<Obstacle> obstacles =
        parts.empty() ? syntheticForest(seed) : xgc2_world_lidar::toObstacles(readParts(parts));
    for (const bool keep : {true, false}) {
        std::vector<double> wall, cpu;
        std::size_t samples = 0;
        for (int k = 0; k < 5; ++k) {
            const auto start = Clock::now();
            const double cpu_start = cpuMs();
            const LidarScene scene(obstacles, 0.1, keep);
            cpu.push_back(cpuMs() - cpu_start);
            wall.push_back(msSince(start));
            samples = scene.sampleCount();
        }
        std::printf("scene build keep_buried=%d: %zu parts, %zu samples, median of 5: "
                    "%.2f ms wall, %.2f ms cpu\n",
                    keep ? 1 : 0,
                    obstacles.size(),
                    samples,
                    median(wall),
                    median(cpu));
    }

    SensorConfig config;
    config.range = 8.0;
    config.h_fov_deg = 360.0;
    config.v_fov_deg = 180.0;
    config.surface_spacing = 0.1;
    if (mode == "penetrating") {
        config.mode = SensorConfig::kPenetrating;
        config.penetrating_keep_buried = true;
    } else if (mode == "raycast") {
        config.mode = SensorConfig::kRaycast;
        config.h_res = 360;
        config.v_res = 32;
    } else {
        std::fprintf(stderr, "--mode must be penetrating or raycast\n");
        return 2;
    }
    if (path != "tagged" && path != "into") {
        std::fprintf(stderr, "--path must be tagged or into\n");
        return 2;
    }
    auto scene = std::make_shared<const LidarScene>(
        obstacles, config.surface_spacing, config.penetrating_keep_buried);
    std::vector<std::unique_ptr<WorldLidar>> fleet;
    std::vector<Eigen::Vector3d> start;
    const int side = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(robots))));
    for (int i = 0; i < robots; ++i) {
        fleet.push_back(std::make_unique<WorldLidar>(config));
        fleet.back()->setScene(scene);
        const int column = i % side, row = i / side;
        start.emplace_back(
            -16.0 + 30.0 * (column + 0.5) / side, -7.0 + 14.0 * (row + 0.5) / side, 0.5);
    }
    std::vector<std::vector<uint8_t>> clouds(robots);
    if (threads == 0)
        threads = static_cast<int>(xgc2_world_lidar::defaultScanThreads(
            static_cast<std::size_t>(robots), std::thread::hardware_concurrency()));
    xgc2_world_lidar::ScanPool pool(static_cast<std::size_t>(threads));
    std::vector<double> tick_ms, tick_cpu;
    double points = 0.0;
    Fnv checksum;
    for (int t = 0; t < ticks; ++t) {
        const auto tick_start = Clock::now();
        const double cpu_start = cpuMs();
        pool.run(static_cast<std::size_t>(robots), [&](std::size_t i) {
            const Eigen::Vector3d p = start[i] + Eigen::Vector3d(0.075 * t, 0.0, 0.0);
            if (path == "into")
                fleet[i]->scanInto(p, Eigen::Quaterniond::Identity(), {}, false, &clouds[i]);
            else
                pack(fleet[i]->scanTagged(p, Eigen::Quaterniond::Identity(), {}), &clouds[i]);
        });
        tick_cpu.push_back(cpuMs() - cpu_start);
        tick_ms.push_back(msSince(tick_start));
        for (const auto& cloud : clouds) {
            checksum.add(cloud.data(), cloud.size());
            points += static_cast<double>(cloud.size()) / 12.0;
        }
    }
    std::vector<double> sorted = tick_ms;
    std::sort(sorted.begin(), sorted.end());
    double mean = 0.0;
    for (double v : tick_ms)
        mean += v;
    mean /= static_cast<double>(tick_ms.size());
    std::printf("%s via %s, %d robots, %d ticks, %d threads: tick wall mean %.2f p50 %.2f p95 "
                "%.2f ms, cpu p50 %.2f ms (%.3f ms/scan), %.0f points/scan, checksum %016llx\n",
                mode.c_str(),
                path.c_str(),
                robots,
                ticks,
                threads,
                mean,
                sorted[sorted.size() / 2],
                sorted[std::min(sorted.size() - 1, sorted.size() * 95 / 100)],
                median(tick_cpu),
                median(tick_cpu) / robots,
                points / (static_cast<double>(ticks) * robots),
                static_cast<unsigned long long>(checksum.h));
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "bench_world_lidar: %s\n", e.what());
    } catch (...) {
        std::fprintf(stderr, "bench_world_lidar: unknown error\n");
    }
    return 1;
}
