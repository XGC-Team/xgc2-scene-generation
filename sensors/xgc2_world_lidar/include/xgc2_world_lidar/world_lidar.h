#ifndef XGC2_WORLD_LIDAR_WORLD_LIDAR_H
#define XGC2_WORLD_LIDAR_WORLD_LIDAR_H

// World-frame LiDAR / point-cloud sensor, without ROS.
//
// One scene, one sensor model and one output frame for every simulated robot
// that enables it, on every simulator. The scene is a set of closed convex
// solids; a scan returns WORLD-frame points.
//
//   kRaycast      first return per beam (non-penetrating). Beam pattern:
//                 h_res x v_res beams over h_fov_deg x v_fov_deg, sensor frame
//                 x forward, y left, z up, elevation centred on the x-y plane.
//                 A beam that hits nothing inside `range` yields no point.
//   kPenetrating  every surface sample of every obstacle (spacing
//                 `surface_spacing`) that lies within `range` and inside the
//                 angular field of view; no occlusion, so back faces and
//                 hidden obstacles are returned (depth-map / global-map style).
//                 Samples strictly inside another obstacle are dropped, so the
//                 cloud lies on the boundary of the union of the solids, unless
//                 `penetrating_keep_buried` (preset bridge_equivalent).
//                 Optional `penetrating_heading_crop` reproduces the CPU
//                 ZJU local_sensing crop (pointcloud_render_node.cpp:127-136):
//                 keep dot(normalized(p - x), body x axis) >= heading_cos_min
//                 and |p_z - x_z| <= vertical_slab_tan * range.
//   kDepthFrustum first return per pixel of a pinhole camera (width x height,
//                 intrinsics fx fy cx cy; 0 derives them from h_fov_deg with
//                 square pixels and a centred principal point). Camera looks
//                 along sensor +x; pixel u grows to sensor -y, v to sensor -z.
//                 Same ray caster, occlusion and range limits as kRaycast.
//
// Range limits: a return is kept when min_range <= distance <= range, the
// distance measured along the beam (Euclidean, also for kDepthFrustum).
//
// Noise: zero-mean Gaussian of standard deviation `noise_std` along the
// sensor-to-point direction (range noise). Scans draw from a generator seeded
// by `seed` and the scan index, so a run is reproducible.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace xgc2_world_lidar {

struct SensorConfig {
    enum Mode { kRaycast, kPenetrating, kDepthFrustum };
    Mode mode = kRaycast;
    double range = 20.0;          // metres, max range
    double min_range = 0.0;       // metres, returns closer than this are dropped
    double h_fov_deg = 360.0;     // (0, 360]
    double v_fov_deg = 30.0;      // [0, 180]
    int h_res = 360;              // beams per row
    int v_res = 32;               // rows
    double surface_spacing = 0.1; // metres, kPenetrating sample spacing
    double noise_std = 0.0;       // metres, range noise
    unsigned seed = 0;
    // kPenetrating: ZJU local_sensing heading half-space / cone and z slab.
    bool penetrating_heading_crop = false;
    double heading_cos_min = 0.0;                   // 0.0 half-space, 0.5 +-60 deg cone
    double vertical_slab_tan = 0.57735026918962576; // tan(pi/6)
    // kPenetrating: keep samples that lie strictly inside another obstacle
    // (every part is sampled on its own).
    bool penetrating_keep_buried = false;
    // kDepthFrustum pinhole camera (ZJU camera.yaml is 640 x 480).
    int width = 160;
    int height = 120;
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0; // pixels; 0 = derive
};

// One beam of a kRaycast / kDepthFrustum scan (the free
// segment of a beam is [origin, origin + range * direction) only when `hit`).
struct Beam {
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();     // world
    Eigen::Vector3d direction = Eigen::Vector3d::UnitX(); // world, unit
    bool hit = false;
    double range = 0.0;  // return distance (with noise) if hit, else SensorConfig::range
    int vehicle_id = -1; // >= 0: the return is on that vehicle's body, -1: static geometry
};

// A returned point with its source (-1 static geometry, else a vehicle id).
struct TaggedPoint {
    Eigen::Vector3d point;
    int vehicle_id = -1;
};

// Another vehicle's body, a sphere, seen by the sensor (off unless passed).
struct VehicleBody {
    int id = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double radius = 0.3;
};

// A closed convex solid. Every shape is defined in its local frame and placed
// by (position, orientation), local -> world. Conventions follow
// xgc2_geometry_msgs/SceneGeometry: box `size` is the full side length,
// cylinder `height` is the full height along local z, capsule `height` is the
// straight cylindrical part along local z, convex `vertices` are local-frame
// points whose convex hull is the solid.
struct Obstacle {
    enum Type { kBox, kSphere, kCylinder, kCapsule, kConvex };
    Type type = kBox;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d size = Eigen::Vector3d::Zero();
    double radius = 0.0;
    double height = 0.0;
    std::vector<Eigen::Vector3d> vertices;

    static Obstacle box(const Eigen::Vector3d& center,
                        const Eigen::Vector3d& size,
                        const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity());
    static Obstacle alignedBox(const Eigen::Vector3d& min_corner,
                               const Eigen::Vector3d& max_corner);
    static Obstacle sphere(const Eigen::Vector3d& center, double radius);
    static Obstacle
    cylinder(const Eigen::Vector3d& center,
             double radius,
             double height,
             const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity());
    static Obstacle capsule(const Eigen::Vector3d& center,
                            double radius,
                            double height,
                            const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity());
    // World-frame vertices (pose = identity).
    static Obstacle convexHull(const std::vector<Eigen::Vector3d>& world_vertices);
    // Local-frame vertices placed by a pose.
    static Obstacle convexHull(const std::vector<Eigen::Vector3d>& local_vertices,
                               const Eigen::Vector3d& position,
                               const Eigen::Quaterniond& orientation);
};

// Surface samples of one obstacle in the world frame, at roughly `spacing`
// metres (faces, edges and caps included). Throws std::invalid_argument on a
// degenerate obstacle.
std::vector<Eigen::Vector3d> sampleSurface(const Obstacle& obstacle, double spacing);

namespace detail {
struct Shape;
struct Bvh;
struct Scene;
} // namespace detail

// Immutable scene geometry and sampled-map index, shared by a fleet. Rebuild
// once when scene contents change (from the previous scene when only some
// obstacles moved); per-robot scans never resample the map.
class LidarScene {
public:
    LidarScene(const std::vector<Obstacle>& obstacles,
               double spacing = 0.1,
               bool keep_buried = false);
    // The scene LidarScene(obstacles, spacing, keep_buried) builds (the same
    // samples in the same order), built from `previous`: the shapes, samples
    // and sampled-map index of obstacles that are bitwise unchanged at the same
    // position in the list are reused, so after some obstacles moved only they
    // (and, without buried samples, the obstacles they touch) are compiled,
    // sampled and indexed again. Builds from scratch when the spacing, the
    // buried-sample policy or the number of obstacles differ.
    LidarScene(const std::vector<Obstacle>& obstacles,
               double spacing,
               bool keep_buried,
               const LidarScene& previous);
    ~LidarScene();
    LidarScene(const LidarScene&) = delete;
    LidarScene& operator=(const LidarScene&) = delete;
    std::size_t sampleCount() const;

private:
    friend class WorldLidar;
    std::unique_ptr<detail::Scene> data_;
};

class WorldLidar {
public:
    // Throws std::invalid_argument on an invalid configuration.
    explicit WorldLidar(SensorConfig config);
    ~WorldLidar();
    WorldLidar(const WorldLidar&) = delete;
    WorldLidar& operator=(const WorldLidar&) = delete;

    // Replaces the scene (rebuilds the BVH, shared surface samples and index). Throws
    // std::invalid_argument on a degenerate obstacle and then leaves the previous scene installed.
    void setScene(const std::vector<Obstacle>& obstacles);
    // Installs an already compiled fleet scene. Penetrating scans require its
    // sampling spacing and buried-surface policy to match their configuration.
    void setScene(std::shared_ptr<const LidarScene> scene);
    std::shared_ptr<const LidarScene> scene() const { return scene_; }

    // One scan from a sensor at `position` with attitude `attitude`
    // (sensor -> world). Returns WORLD-frame points. Thread-safe with respect
    // to other scan() calls; not with respect to setScene().
    std::vector<Eigen::Vector3d> scan(const Eigen::Vector3d& position,
                                      const Eigen::Quaterniond& attitude) const;

    // ---- additions beyond the fixed contract -------------------------------
    // Per-beam records of one kRaycast / kDepthFrustum scan, in beam order
    // (throws std::logic_error in kPenetrating, which has no beams). The hits
    // of this call are exactly the points scan() would return from the same
    // scan index, plus returns on `others`.
    std::vector<Beam> scanWithBeams(const Eigen::Vector3d& position,
                                    const Eigen::Quaterniond& attitude,
                                    const std::vector<VehicleBody>& others = {}) const;
    // Points of one scan in any mode, tagged with their source; `others` are
    // added as spheres (kPenetrating: sampled at surface_spacing).
    std::vector<TaggedPoint> scanTagged(const Eigen::Vector3d& position,
                                        const Eigen::Quaterniond& attitude,
                                        const std::vector<VehicleBody>& others = {}) const;
    // The points scanTagged() returns, written straight into `data` as the
    // records of a sensor_msgs/PointCloud2: float32 x y z in host byte order,
    // then int32 vehicle_id (-1 static geometry) when `with_id` (point_step 12
    // or 16). `data` is resized to exactly the records; its capacity is kept,
    // so a buffer reused across scans does not reallocate. Returns the count.
    std::size_t scanInto(const Eigen::Vector3d& position,
                         const Eigen::Quaterniond& attitude,
                         const std::vector<VehicleBody>& others,
                         bool with_id,
                         std::vector<uint8_t>* data) const;
    // Distance along unit `direction` to the first surface within max_range,
    // or +infinity. A ray starting inside a solid returns its exit surface.
    double castRay(const Eigen::Vector3d& origin,
                   const Eigen::Vector3d& direction,
                   double max_range) const;
    // Surface samples of the whole scene at `spacing`, without samples buried
    // inside another obstacle unless penetrating_keep_buried (the latched
    // global map). No range/FOV filter.
    std::vector<Eigen::Vector3d> globalMap(double spacing) const;
    // Scene residual: min over obstacles of a per-obstacle function that is
    // 0 on the surface, < 0 inside and > 0 outside (|value| <= distance to that
    // surface). Used by the tests; also a cheap occupancy query.
    double surfaceResidual(const Eigen::Vector3d& point) const;

    const SensorConfig& config() const { return config_; }
    std::size_t obstacleCount() const;
    std::size_t beamCount() const { return beams_.size(); }

private:
    template <class Sink>
    void traceBeams(const Eigen::Vector3d& position,
                    const Eigen::Matrix3d& rotation,
                    uint64_t scan_index,
                    const std::vector<VehicleBody>& others,
                    Sink&& sink) const;
    // Out: reserve(upper bound of the points to come), push(point, vehicle id).
    template <class Out>
    void samplePenetrating(const Eigen::Vector3d& position,
                           const Eigen::Matrix3d& rotation,
                           uint64_t scan_index,
                           const std::vector<VehicleBody>& others,
                           Out& out) const;
    template <class Out>
    void scanPoints(const Eigen::Vector3d& position,
                    const Eigen::Quaterniond& attitude,
                    const std::vector<VehicleBody>& others,
                    Out& out) const;
    bool insideFov(const Eigen::Vector3d& sensor_frame_vector) const;
    bool insideCrop(const Eigen::Vector3d& world_offset, const Eigen::Matrix3d& rotation) const;

    SensorConfig config_;
    std::vector<Eigen::Vector3d> beams_; // unit, sensor frame
    std::shared_ptr<const LidarScene> scene_;
    mutable std::atomic<uint64_t> scan_counter_{0};
};

} // namespace xgc2_world_lidar

#endif // XGC2_WORLD_LIDAR_WORLD_LIDAR_H
