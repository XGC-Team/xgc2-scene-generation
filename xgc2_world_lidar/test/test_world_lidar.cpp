// Plain-assert tests for the ROS-free core. Exit status 0 iff every check holds.

#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>

#include "xgc2_world_lidar/scene_conversion.h"
#include "xgc2_world_lidar/world_lidar.h"

using xgc2_world_lidar::Beam;
using xgc2_world_lidar::Obstacle;
using xgc2_world_lidar::VehicleBody;
using xgc2_world_lidar::SensorConfig;
using xgc2_world_lidar::WorldLidar;

namespace {

int g_failures = 0;
int g_checks = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        ++g_checks;                                                          \
        if (!(cond)) {                                                       \
            ++g_failures;                                                    \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                    \
    } while (0)

const Eigen::Quaterniond kI = Eigen::Quaterniond::Identity();

Eigen::Quaterniond randomQuat(std::mt19937& rng) {
    std::normal_distribution<double> n(0.0, 1.0);
    return Eigen::Quaterniond(n(rng), n(rng), n(rng), n(rng)).normalized();
}

// A mixed random scene: every primitive type, rotated.
std::vector<Obstacle> randomScene(std::mt19937& rng, int count, double half_extent) {
    std::uniform_real_distribution<double> pos(-half_extent, half_extent);
    std::uniform_real_distribution<double> z(-2.0, 2.0);
    std::uniform_real_distribution<double> sz(0.3, 2.0);
    std::vector<Obstacle> out;
    for (int i = 0; i < count; ++i) {
        const Eigen::Vector3d c(pos(rng), pos(rng), z(rng));
        switch (i % 5) {
            case 0: out.push_back(Obstacle::box(c, Eigen::Vector3d(sz(rng), sz(rng), sz(rng)), randomQuat(rng))); break;
            case 1: out.push_back(Obstacle::sphere(c, 0.5 * sz(rng))); break;
            case 2: out.push_back(Obstacle::cylinder(c, 0.5 * sz(rng), 2.0 * sz(rng), randomQuat(rng))); break;
            case 3: out.push_back(Obstacle::capsule(c, 0.4 * sz(rng), sz(rng), randomQuat(rng))); break;
            case 4: {
                // irregular_rock_1 from cluttered_environment/config/v_polytope_library.yaml
                std::vector<Eigen::Vector3d> v{{-0.45, -0.40, -0.50}, {0.50, -0.35, -0.50}, {0.45, 0.48, -0.50},
                                               {-0.38, 0.42, -0.50},  {-0.25, -0.15, 0.80}, {0.30, -0.20, 0.75},
                                               {0.28, 0.25, 0.85},    {-0.22, 0.30, 0.70}};
                const double s = sz(rng);
                for (auto& p : v) p *= s;
                out.push_back(Obstacle::convexHull(v, c, randomQuat(rng)));
                break;
            }
        }
    }
    return out;
}

// True when the open segment (a, b) passes through the interior of the scene.
bool segmentCrossesInterior(const WorldLidar& lidar, const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    const int n = 400;
    for (int k = 1; k < n; ++k) {
        const Eigen::Vector3d p = a + (b - a) * (static_cast<double>(k) / n);
        if ((p - b).norm() < 1e-4) continue;
        if (lidar.surfaceResidual(p) < -1e-6) return true;
    }
    return false;
}

SensorConfig lidar32(SensorConfig::Mode mode) {
    SensorConfig c;
    c.mode = mode;
    c.range = 20.0;
    c.h_fov_deg = 360.0;
    c.v_fov_deg = 30.0;
    c.h_res = 360;
    c.v_res = 32;
    c.surface_spacing = 0.1;
    return c;
}

void testRaycastOcclusion() {
    std::printf("raycast: no return behind an occluder\n");
    const Obstacle front = Obstacle::box(Eigen::Vector3d(5, 0, 0), Eigen::Vector3d(1, 4, 4));
    const Obstacle back = Obstacle::box(Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(1, 12, 8));
    WorldLidar lidar(lidar32(SensorConfig::kRaycast));
    lidar.setScene({front, back});
    WorldLidar front_only(lidar32(SensorConfig::kRaycast));
    front_only.setScene({front});
    const Eigen::Vector3d o = Eigen::Vector3d::Zero();
    const auto pts = lidar.scan(o, kI);
    int on_back = 0, bad = 0;
    for (const auto& p : pts) {
        if (p.x() > 9.0) {
            ++on_back;
            // The beam to a back-wall point must not meet the front box first.
            if (front_only.castRay(o, p - o, 20.0) < (p - o).norm() - 1e-9) ++bad;
        }
        CHECK(p.x() <= 5.5 + 1e-9 || p.x() >= 9.5 - 1e-9);  // front face or back wall only
        CHECK(p.x() <= 4.5 + 1e-6 || p.x() >= 9.5 - 1e-6);  // never the far face of the front box
    }
    CHECK(on_back > 0);
    CHECK(bad == 0);
    std::printf("  %zu points, %d on the back wall, %d occluded returns\n", pts.size(), on_back, bad);

    // Random clutter: the segment sensor -> return never crosses a solid.
    std::mt19937 rng(7);
    WorldLidar cluttered(lidar32(SensorConfig::kRaycast));
    cluttered.setScene(randomScene(rng, 100, 12.0));
    int crossings = 0;
    std::size_t total = 0;
    for (int trial = 0; trial < 3; ++trial) {
        const Eigen::Vector3d s(3.0 * trial - 3.0, 0.5, 4.0);  // above most clutter (z in [-2, 2])
        const auto cloud = cluttered.scan(s, randomQuat(rng));
        total += cloud.size();
        for (std::size_t i = 0; i < cloud.size(); i += 7) crossings += segmentCrossesInterior(cluttered, s, cloud[i]);
    }
    CHECK(total > 0);
    CHECK(crossings == 0);
    std::printf("  clutter: %zu points, %d segments crossing a solid\n", total, crossings);
}

void testPenetratingBackFaces() {
    std::printf("penetrating: back faces and hidden obstacles are returned\n");
    const Obstacle front = Obstacle::box(Eigen::Vector3d(5, 0, 0), Eigen::Vector3d(2, 2, 2));
    const Obstacle hidden = Obstacle::sphere(Eigen::Vector3d(9, 0, 0), 0.5);  // fully in the box's shadow
    WorldLidar pen(lidar32(SensorConfig::kPenetrating));
    pen.setScene({front, hidden});
    WorldLidar ray(lidar32(SensorConfig::kRaycast));
    ray.setScene({front, hidden});
    const auto p = pen.scan(Eigen::Vector3d::Zero(), kI);
    const auto r = ray.scan(Eigen::Vector3d::Zero(), kI);
    int back_face = 0, hidden_pts = 0, ray_back = 0, ray_hidden = 0;
    for (const auto& q : p) back_face += std::abs(q.x() - 6.0) < 1e-9, hidden_pts += q.x() > 8.0;
    for (const auto& q : r) ray_back += std::abs(q.x() - 6.0) < 1e-9, ray_hidden += q.x() > 8.0;
    CHECK(back_face > 0);
    CHECK(hidden_pts > 0);
    CHECK(ray_back == 0);
    CHECK(ray_hidden == 0);
    std::printf("  penetrating %zu pts (back face %d, hidden %d); raycast %zu pts (back face %d, hidden %d)\n",
                p.size(), back_face, hidden_pts, r.size(), ray_back, ray_hidden);
}

void testOnSurfaceAndInRange() {
    std::printf("both modes: points on surfaces (within noise) and within range\n");
    for (int mode = 0; mode < 2; ++mode) {
        for (double noise : {0.0, 0.02}) {
            SensorConfig c = lidar32(mode == 0 ? SensorConfig::kRaycast : SensorConfig::kPenetrating);
            c.range = 8.0;
            c.noise_std = noise;
            c.seed = 3;
            std::mt19937 rng(11);
            WorldLidar lidar(c);
            lidar.setScene(randomScene(rng, 100, 10.0));
            const double tol = noise == 0.0 ? 1e-6 : 6.0 * noise;
            double worst = 0.0, farthest = 0.0;
            std::size_t n = 0;
            for (int trial = 0; trial < 4; ++trial) {
                const Eigen::Vector3d s(-6.0 + 4.0 * trial, 1.0, 3.5);
                for (const auto& q : lidar.scan(s, randomQuat(rng))) {
                    worst = std::max(worst, std::abs(lidar.surfaceResidual(q)));
                    farthest = std::max(farthest, (q - s).norm());
                    ++n;
                }
            }
            CHECK(n > 0);
            CHECK(worst <= tol);
            CHECK(farthest <= c.range + 1e-9);
            std::printf("  %s noise=%.2f: %zu pts, max |residual| %.2e (tol %.1e), max range %.3f / %.1f\n",
                        mode == 0 ? "raycast    " : "penetrating", noise, n, worst, tol, farthest, c.range);
        }
    }
}

void testBvhMatchesBruteForce() {
    std::printf("raycast: BVH closest hit equals brute force over single obstacles\n");
    std::mt19937 rng(5);
    const auto scene = randomScene(rng, 100, 15.0);
    SensorConfig c = lidar32(SensorConfig::kRaycast);
    WorldLidar lidar(c);
    lidar.setScene(scene);
    std::vector<std::unique_ptr<WorldLidar>> singles;
    for (const auto& o : scene) {
        singles.emplace_back(new WorldLidar(c));
        singles.back()->setScene({o});
    }
    std::uniform_real_distribution<double> u(-15.0, 15.0);
    std::normal_distribution<double> n(0.0, 1.0);
    int mismatches = 0, hits = 0;
    for (int i = 0; i < 3000; ++i) {
        const Eigen::Vector3d o(u(rng), u(rng), 0.2 * u(rng));
        const Eigen::Vector3d d = Eigen::Vector3d(n(rng), n(rng), n(rng)).normalized();
        double brute = std::numeric_limits<double>::infinity();
        for (const auto& s : singles) brute = std::min(brute, s->castRay(o, d, 25.0));
        const double t = lidar.castRay(o, d, 25.0);
        hits += std::isfinite(t);
        if (!(t == brute || std::abs(t - brute) < 1e-9)) ++mismatches;
    }
    CHECK(mismatches == 0);
    CHECK(hits > 100);
    std::printf("  3000 rays, %d hits, %d mismatches\n", hits, mismatches);
}

void testFieldOfView() {
    std::printf("field of view: 90 deg sector follows the attitude\n");
    const Eigen::Quaterniond yaw90(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
    for (int mode = 0; mode < 2; ++mode) {
        SensorConfig c = lidar32(mode == 0 ? SensorConfig::kRaycast : SensorConfig::kPenetrating);
        c.h_fov_deg = 90.0;
        c.h_res = 90;
        WorldLidar lidar(c);
        // A ring of pillars all around the sensor.
        std::vector<Obstacle> ring;
        for (int k = 0; k < 16; ++k) {
            const double a = 2 * M_PI * k / 16;
            ring.push_back(Obstacle::cylinder(Eigen::Vector3d(6 * std::cos(a), 6 * std::sin(a), 0), 0.8, 6.0));
        }
        lidar.setScene(ring);
        const auto pts = lidar.scan(Eigen::Vector3d::Zero(), yaw90);  // looking along +y
        int outside = 0;
        for (const auto& p : pts) {
            const double az = std::atan2(p.y(), p.x()) - M_PI / 2;
            outside += std::abs(az) > M_PI / 4 + 1e-9;
            const double el = std::atan2(p.z(), std::hypot(p.x(), p.y()));
            outside += std::abs(el) > 15.0 * M_PI / 180 + 1e-9;
        }
        CHECK(!pts.empty());
        CHECK(outside == 0);
        std::printf("  %s: %zu pts, %d outside the sector\n", mode == 0 ? "raycast" : "penetrating", pts.size(),
                    outside);
    }
}

void testDeterminismAndGlobalMap() {
    std::printf("seeded noise is reproducible; global map lies on surfaces\n");
    SensorConfig c = lidar32(SensorConfig::kRaycast);
    c.noise_std = 0.05;
    c.seed = 42;
    std::mt19937 r1(9), r2(9);
    WorldLidar a(c), b(c);
    a.setScene(randomScene(r1, 30, 8.0));
    b.setScene(randomScene(r2, 30, 8.0));
    bool same = true;
    for (int k = 0; k < 3; ++k) {
        const auto pa = a.scan(Eigen::Vector3d(0, 0, 3), kI);
        const auto pb = b.scan(Eigen::Vector3d(0, 0, 3), kI);
        same = same && pa.size() == pb.size();
        for (std::size_t i = 0; same && i < pa.size(); ++i) same = pa[i] == pb[i];
    }
    CHECK(same);
    const auto map = a.globalMap(0.2);
    double worst = 0.0;
    for (const auto& p : map) worst = std::max(worst, std::abs(a.surfaceResidual(p)));
    CHECK(map.size() > 1000);
    CHECK(worst < 1e-9);
    std::printf("  global map %zu pts, max |residual| %.1e\n", map.size(), worst);
}

void testSceneConversionAndValidation() {
    std::printf("scene conversion: obstacle pose * part pose; invalid input refused\n");
    using namespace xgc2_world_lidar;
    SceneObstacleDescription o;
    o.id = "wall";
    o.pose.position = Eigen::Vector3d(0, 0, 1);
    o.pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
    ScenePartDescription part;
    part.type = "box";
    part.pose.position = Eigen::Vector3d(4, 0, 0);  // -> world (0, 4, 1)
    part.size = Eigen::Vector3d(1, 2, 2);           // rotated: 2 wide in x, 1 deep in y
    o.parts.push_back(part);
    ScenePartDescription rock;
    rock.type = "convex";
    rock.pose.position = Eigen::Vector3d(0, 0, 5);
    rock.vertices = {{-0.45, -0.40, -0.50}, {0.50, -0.35, -0.50}, {0.45, 0.48, -0.50}, {-0.38, 0.42, -0.50},
                     {-0.25, -0.15, 0.80},  {0.30, -0.20, 0.75},  {0.28, 0.25, 0.85},   {-0.22, 0.30, 0.70},
                     {0.0, 0.0, 0.0}};  // an interior point is ignored
    o.parts.push_back(rock);
    const auto obstacles = toObstacles({o});
    CHECK(obstacles.size() == 2);
    WorldLidar lidar(lidar32(SensorConfig::kRaycast));
    lidar.setScene(obstacles);
    CHECK(std::abs(lidar.castRay(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d::UnitY(), 20) - 3.5) < 1e-9);
    CHECK(std::abs(lidar.castRay(Eigen::Vector3d(0.9, 0, 1), Eigen::Vector3d::UnitY(), 20) - 3.5) < 1e-9);
    CHECK(std::isinf(lidar.castRay(Eigen::Vector3d(1.1, 0, 1), Eigen::Vector3d::UnitY(), 20)));
    // Rock centre at world z = 1 + 5; its bottom face (part-local z = -0.5) at z = 5.5.
    CHECK(std::abs(lidar.castRay(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d::UnitZ(), 20) - 5.5) < 1e-9);

    auto throws = [](const std::vector<SceneObstacleDescription>& s) {
        try {
            toObstacles(s);
        } catch (const std::invalid_argument&) {
            return true;
        }
        return false;
    };
    SceneObstacleDescription bad = o;
    bad.parts = {part};
    bad.parts[0].type = "mesh";
    CHECK(throws({bad}));
    bad.parts[0].type = "box";
    bad.parts[0].size.x() = 0.0;
    CHECK(throws({bad}));
    bad.parts[0] = rock;
    bad.parts[0].vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}};  // flat
    CHECK(throws({bad}));
    bool config_refused = false;
    try {
        SensorConfig c;
        c.h_res = 0;
        WorldLidar l(c);
    } catch (const std::invalid_argument&) {
        config_refused = true;
    }
    CHECK(config_refused);
}

// ---- S1 beams, S2 depth frustum, S0 heading crop, vehicle bodies ----------

void testBeams() {
    std::printf("beams (S1): hits equal scan(); free segments are free; no-return beams are empty\n");
    for (double noise : {0.0, 0.02}) {
        SensorConfig c = lidar32(SensorConfig::kRaycast);
        c.range = 10.0;
        c.noise_std = noise;
        c.seed = 17;
        std::mt19937 r1(21), r2(21);
        WorldLidar a(c), b(c);
        a.setScene(randomScene(r1, 80, 10.0));
        b.setScene(randomScene(r2, 80, 10.0));
        const Eigen::Vector3d o(0.5, -0.5, 3.0);
        const Eigen::Quaterniond q(Eigen::AngleAxisd(0.4, Eigen::Vector3d(1, 2, 3).normalized()));
        const auto pts = a.scan(o, q);
        const auto beams = b.scanWithBeams(o, q);
        CHECK(beams.size() == b.beamCount());
        std::vector<Eigen::Vector3d> from_beams;
        int crossing = 0, bad_miss = 0, bad_record = 0;
        for (std::size_t i = 0; i < beams.size(); ++i) {
            const Beam& be = beams[i];
            bad_record += !(be.origin == o) || std::abs(be.direction.norm() - 1.0) > 1e-12 || be.vehicle_id != -1;
            if (be.hit) {
                from_beams.push_back(be.origin + be.range * be.direction);
                bad_record += !(be.range > 0.0 && be.range <= c.range);
                // [origin, return) shrunk by the noise bound is free.
                const double shrink = noise == 0.0 ? 1e-6 : 6.0 * noise;
                if (i % 5 == 0 && be.range > shrink)
                    crossing += segmentCrossesInterior(b, o, o + (be.range - shrink) * be.direction);
            } else {
                bad_record += be.range != c.range;
                if (noise == 0.0) bad_miss += std::isfinite(b.castRay(o, be.direction, c.range));
            }
        }
        bool same = from_beams.size() == pts.size();
        for (std::size_t i = 0; same && i < pts.size(); ++i) same = from_beams[i] == pts[i];
        CHECK(same);
        CHECK(crossing == 0);
        CHECK(bad_miss == 0);
        CHECK(bad_record == 0);
        std::printf("  noise=%.2f: %zu beams, %zu hits (== scan(): %s), %d free segments crossing a solid, "
                    "%d no-return beams with a surface in range\n",
                    noise, beams.size(), from_beams.size(), same ? "yes" : "NO", crossing, bad_miss);
    }
    SensorConfig pen = lidar32(SensorConfig::kPenetrating);
    WorldLidar p(pen);
    bool threw = false;
    try {
        p.scanWithBeams(Eigen::Vector3d::Zero(), kI);
    } catch (const std::logic_error&) {
        threw = true;
    }
    CHECK(threw);
}

void testDepthFrustum() {
    std::printf("depth frustum (S2): occlusion, surfaces, range limits, frustum\n");
    SensorConfig c;
    c.mode = SensorConfig::kDepthFrustum;
    c.width = 160;
    c.height = 120;
    c.h_fov_deg = 90.0;
    c.range = 20.0;
    c.min_range = 0.3;
    WorldLidar lidar(c);
    CHECK(lidar.beamCount() == 160u * 120u);
    const Eigen::Quaterniond yaw90(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));  // looks along +y
    const Obstacle front = Obstacle::box(Eigen::Vector3d(0, 5, 0), Eigen::Vector3d(4, 1, 4));
    const Obstacle back = Obstacle::box(Eigen::Vector3d(0, 10, 0), Eigen::Vector3d(30, 1, 30));
    const Obstacle behind = Obstacle::box(Eigen::Vector3d(0, -5, 0), Eigen::Vector3d(4, 1, 4));
    lidar.setScene({front, back, behind});
    WorldLidar front_only(c);
    front_only.setScene({front});
    const Eigen::Vector3d o = Eigen::Vector3d::Zero();
    const auto pts = lidar.scan(o, yaw90);
    int occluded = 0, off_surface = 0, out_of_frustum = 0, far_face = 0, on_back = 0, on_behind = 0;
    const double tan_h = 0.5 * c.width / (0.5 * c.width / std::tan(M_PI / 4));  // = 1
    const double tan_v = tan_h * c.height / c.width;
    for (const auto& p : pts) {
        on_back += p.y() > 9.0;
        on_behind += p.y() < 0.0;
        far_face += std::abs(p.y() - 5.5) < 1e-6;
        if (p.y() > 9.0 && front_only.castRay(o, p - o, 20.0) < p.norm() - 1e-9) ++occluded;
        off_surface += std::abs(lidar.surfaceResidual(p)) > 1e-6;
        const Eigen::Vector3d s = yaw90.inverse() * p;  // sensor frame
        out_of_frustum += !(s.x() > 0 && std::abs(s.y() / s.x()) <= tan_h + 1e-9 &&
                            std::abs(s.z() / s.x()) <= tan_v + 1e-9);
        out_of_frustum += p.norm() > c.range + 1e-9 || p.norm() < c.min_range - 1e-9;
    }
    CHECK(!pts.empty());
    CHECK(on_back > 0);
    CHECK(occluded == 0);
    CHECK(far_face == 0);
    CHECK(on_behind == 0);
    CHECK(off_surface == 0);
    CHECK(out_of_frustum == 0);
    std::printf("  %zu pts, %d on the back wall, %d occluded, %d off-surface, %d outside frustum/range\n", pts.size(),
                on_back, occluded, off_surface, out_of_frustum);
    // Clutter: no return behind an occluder, exactly as raycast.
    std::mt19937 rng(8);
    WorldLidar cluttered(c);
    cluttered.setScene(randomScene(rng, 100, 12.0));
    int crossings = 0;
    std::size_t total = 0;
    for (int k = 0; k < 3; ++k) {
        const Eigen::Vector3d s(-3.0 + 3.0 * k, 0.0, 3.5);
        const Eigen::Quaterniond q(Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(2.0 * k, Eigen::Vector3d::UnitZ()));
        const auto cloud = cluttered.scan(s, q);
        total += cloud.size();
        for (std::size_t i = 0; i < cloud.size(); i += 5) crossings += segmentCrossesInterior(cluttered, s, cloud[i]);
    }
    CHECK(total > 0);
    CHECK(crossings == 0);
    std::printf("  clutter: %zu pts, %d segments crossing a solid\n", total, crossings);
    // min_range: a wall at 0.1 m (<= 0.16 m along the corner pixels) blocks
    // the view and gives no return (unknown).
    lidar.setScene({Obstacle::box(Eigen::Vector3d(0.6, 0, 0), Eigen::Vector3d(1.0, 10, 10))});
    const auto beams = lidar.scanWithBeams(o, kI);
    int hits = 0;
    for (const auto& b : beams) hits += b.hit;
    CHECK(hits == 0);
    bool refused = false;
    try {
        SensorConfig bad = c;
        bad.h_fov_deg = 200.0;
        WorldLidar l(bad);
    } catch (const std::invalid_argument&) {
        refused = true;
    }
    CHECK(refused);
}

void testHeadingCrop() {
    std::printf("penetrating heading crop (S0, ZJU local_sensing)\n");
    std::vector<Obstacle> ring;
    for (int k = 0; k < 12; ++k) {
        const double a = 2 * M_PI * k / 12;
        ring.push_back(Obstacle::cylinder(Eigen::Vector3d(3 * std::cos(a), 3 * std::sin(a), 0), 0.4, 10.0));
    }
    const Eigen::Quaterniond att(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()));
    const Eigen::Vector3d x_body = att * Eigen::Vector3d::UnitX();
    for (double cos_min : {0.0, 0.5}) {
        SensorConfig c = lidar32(SensorConfig::kPenetrating);
        c.range = 5.0;
        c.v_fov_deg = 180.0;
        c.penetrating_heading_crop = true;
        c.heading_cos_min = cos_min;
        WorldLidar cropped(c);
        cropped.setScene(ring);
        c.penetrating_heading_crop = false;
        WorldLidar full(c);
        full.setScene(ring);
        const auto a = cropped.scan(Eigen::Vector3d::Zero(), att);
        const auto b = full.scan(Eigen::Vector3d::Zero(), att);
        int violations = 0, expected = 0;
        for (const auto& p : a)
            violations += p.normalized().dot(x_body) < cos_min || std::abs(p.z()) > std::tan(M_PI / 6) * 5.0 + 1e-12;
        for (const auto& p : b)
            expected += p.normalized().dot(x_body) >= cos_min && std::abs(p.z()) <= std::tan(M_PI / 6) * 5.0;
        CHECK(!a.empty());
        CHECK(violations == 0);
        CHECK(static_cast<int>(a.size()) == expected);
        std::printf("  cos_min=%.1f: %zu of %zu pts kept, %d violations\n", cos_min, a.size(), b.size(), violations);
    }
}

void testVehicleBodies() {
    std::printf("vehicle bodies: neighbours occlude, are tagged, default off\n");
    const Obstacle wall = Obstacle::box(Eigen::Vector3d(8, 0, 0), Eigen::Vector3d(1, 20, 20));
    const std::vector<VehicleBody> others{{3, Eigen::Vector3d(4, 0, 0), 0.5}, {7, Eigen::Vector3d(-4, 0, 0), 0.3}};
    for (int mode = 0; mode < 3; ++mode) {
        SensorConfig c = lidar32(mode == 0 ? SensorConfig::kRaycast
                                           : (mode == 1 ? SensorConfig::kPenetrating : SensorConfig::kDepthFrustum));
        c.v_fov_deg = 60.0;
        c.h_fov_deg = mode == 2 ? 90.0 : 360.0;
        WorldLidar lidar(c);
        lidar.setScene({wall});
        const auto tagged = lidar.scanTagged(Eigen::Vector3d::Zero(), kI, others);
        const auto plain = lidar.scanTagged(Eigen::Vector3d::Zero(), kI);
        int on3 = 0, on7 = 0, bad_tag = 0, static_pts = 0;
        for (const auto& t : tagged) {
            if (t.vehicle_id == 3) ++on3, bad_tag += std::abs((t.point - others[0].position).norm() - 0.5) > 1e-9;
            else if (t.vehicle_id == 7) ++on7, bad_tag += std::abs((t.point - others[1].position).norm() - 0.3) > 1e-9;
            else ++static_pts, bad_tag += t.vehicle_id != -1 || std::abs(lidar.surfaceResidual(t.point)) > 1e-6;
        }
        int plain_tagged = 0;
        for (const auto& t : plain) plain_tagged += t.vehicle_id != -1;
        CHECK(on3 > 0);
        CHECK(mode == 2 ? on7 == 0 : on7 > 0);  // the camera looks along +x only
        CHECK(bad_tag == 0);
        CHECK(plain_tagged == 0);
        if (mode != 1) {
            // Non-penetrating: the neighbour at x=4 hides part of the wall.
            CHECK(static_pts < static_cast<int>(plain.size()));
            const auto beams = lidar.scanWithBeams(Eigen::Vector3d::Zero(), kI, others);
            int tagged_beams = 0, bad_beam = 0;
            for (const auto& b : beams) {
                if (b.vehicle_id < 0) continue;
                ++tagged_beams;
                const Eigen::Vector3d p = b.origin + b.range * b.direction;
                const bool on_body = std::abs((p - others[0].position).norm() - 0.5) <= 1e-9 ||
                                     std::abs((p - others[1].position).norm() - 0.3) <= 1e-9;
                bad_beam += !b.hit || !on_body;
            }
            CHECK(tagged_beams == on3 + on7);
            CHECK(bad_beam == 0);
        } else {
            CHECK(static_pts == static_cast<int>(plain.size()));  // see-through: nothing hidden
        }
        std::printf("  %-11s: %d pts on vehicle 3, %d on vehicle 7, %d static (%zu without bodies)\n",
                    mode == 0 ? "raycast" : (mode == 1 ? "penetrating" : "depth"), on3, on7, static_pts, plain.size());
    }
}

}  // namespace

int main() {
    testRaycastOcclusion();
    testPenetratingBackFaces();
    testOnSurfaceAndInRange();
    testBvhMatchesBruteForce();
    testFieldOfView();
    testDeterminismAndGlobalMap();
    testSceneConversionAndValidation();
    testBeams();
    testDepthFrustum();
    testHeadingCrop();
    testVehicleBodies();
    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
