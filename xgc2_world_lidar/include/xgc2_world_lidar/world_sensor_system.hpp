#pragma once

#include "xgc2_world_lidar/scene_conversion.h"
#include "xgc2_world_lidar/sensor_metadata.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace xgc2_world_lidar {

// Cold, frozen equipment records. Indices refer to the original World source
// roster, not a new robot table or an external ROS interface.
struct NormalSensorEquipment {
    std::size_t source_index = 0;
    SensorConfig sensor;
    Pose mount;
    double rate_hz = 10.0, pose_timeout_sec = 0.5;
    int body_id = -1;
    bool with_bodies = false, publish_beams = false, yaw_only = false;
    std::string frame_id = "world";
};

struct SharedSensorEquipment {
    SensorMetadata metadata;
    // One original static-cloud/index/context shared by these sensors. Their
    // order is the caller's frozen pose_topics/output_topics order. These index
    // acquisition.sources (p/q/stamp), NOT PCD/voxel/filtered point indices.
    std::vector<std::size_t> source_indices;
    // Original effective CPU width, including caller: min(requested, sensors).
    // Original absent value is 1; GPU has no CPU scan-pool width.
    std::size_t worker_threads = 1;
};

struct SensorBodyBinding {
    int id = 0;
    double radius = 0.3;
};

struct NormalMapEquipment {
    SensorConfig sensor;
    double period_sec = 1.0;
    std::string frame_id = "world";
};

struct WorldSensorConfiguration {
    std::size_t source_count = 0;
    std::vector<NormalSensorEquipment> normal;
    std::vector<SharedSensorEquipment> shared;
    std::vector<SensorBodyBinding> bodies;
    std::optional<NormalMapEquipment> normal_map;
    // Original normal node's max-rate timer; per-sensor next_scan stays separate.
    double normal_poll_rate_hz = 10.0;
    bool require_complete_body_roster = false;
    // Original normal effective width, resolved once by its frozen caller:
    // explicit >0 clamps to sensed count; <=0 uses defaultScanThreads.
    std::size_t normal_worker_threads = 1;
};

// World owns ingestion/latest state. The sensor caller captures ONLY these
// values when an original due opportunity actually begins, never each tick
// while busy. No pointers/references into mutable model, ROS or Host state.
struct SensorAcquisition {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    ros::Time source_stamp;
    std::uint64_t source_version = 0, generation = 0, world_commit = 0,
                  geometry_version = 0;
    bool ready = false;
};

struct SensorBodyAcquisition {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    ros::Time source_stamp;
    bool ready = false;
};

struct SensorOutputDemand {
    // Original normal scan admission only: points OR configured beams demand.
    // Once admitted, publish points AND configured beams, never filter each
    // output separately. Shared sensors keep their original unconditional pub.
    bool points = true, beams = false;
};

// Build geometry outside physics. Freeze all pointees before sharing this pin.
// normal[i] is the matching normal equipment's original LidarScene policy;
// shared[i] is the original decoded first cloud for shared group i. A missing
// pin is ordinary not-ready. A valid empty CPU input keeps original semantics.
struct WorldSensorGeometry {
    std::uint64_t version = 0;
    std::vector<std::shared_ptr<const LidarScene>> normal;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> shared;
    std::shared_ptr<const LidarScene> normal_map;
    bool normal_has_dynamic = false;
};

struct WorldSensorAcquisition {
    // configure preallocates fixed cardinalities; capture must not resize.
    std::vector<SensorAcquisition> sources;
    std::vector<SensorBodyAcquisition> bodies;
    // Output order: normal first, then each shared group's source_indices.
    std::vector<SensorOutputDemand> demand;
    std::shared_ptr<const WorldSensorGeometry> geometry;
    bool enabled = true; // original NORMAL set_enabled only; shared has no such gate
    // World scene owner supplies its existing ready/state-age decision. A new
    // refused/pending epoch uses false; its previous latched map must clear.
    bool normal_geometry_ready = true;
    bool normal_state_fresh = true;
    bool normal_map_requested = false;
    // The existing SceneSnapshot clear event, distinct from dynamic geometry
    // updates (which dirty the map but do not clear it). World owns this token.
    std::uint64_t normal_map_clear_version = 0, world_commit = 0;
};

enum class SensorOutputKind { Points, Beams, Map };
constexpr std::size_t kWorldSensorMapOutput = static_cast<std::size_t>(-1);

struct WorldSensorCallbacks {
    // Original ROS clock domain. Finish samples now(), NOT rounded Grid.target.
    std::function<ros::Time()> now;
    // Optional original geometry callback queue, drained only on this sensor
    // caller between batches/before service. Deserialize/fromROSMsg/geometry
    // preparation belong here, not in physics/control or the short capture.
    // No subscription, second latest source, raw/results queue or new worker.
    std::function<void()> process_inputs;
    // Short capture into caller-owned preallocated AoS values; the World holds
    // its source lock only here. No scan/pack/publish, Host API or physics join.
    std::function<void(WorldSensorAcquisition&)> capture;
    // Synchronous consumption, on the ONE sensor execution caller, outside the
    // compute pool. The message and acquisition reference expire on return;
    // ROS publish(const&) serializes before return. No deferred borrow/queue.
    std::function<void(std::size_t, SensorOutputKind,
                       const sensor_msgs::PointCloud2&, const SensorAcquisition&)> publish;
    // Original unrecoverable error -> existing World fatal/fence. This runs on
    // the sensor caller: arrange lifecycle stop elsewhere; do not self-join.
    std::function<void(std::exception_ptr)> fatal;
};

// One World owns this lifecycle. configure/constructor allocates the frozen
// roster. start creates ONE sensor caller; that caller creates/uses/destroys
// every GPU resource and invokes/joins the existing ScanPool. physics may only
// notify(). Library owns active acquisitions, not another mutable latest source.
class WorldSensorSystem {
public:
    WorldSensorSystem(WorldSensorConfiguration configuration, WorldSensorCallbacks callbacks);
    ~WorldSensorSystem();
    WorldSensorSystem(const WorldSensorSystem&) = delete;
    WorldSensorSystem& operator=(const WorldSensorSystem&) = delete;

    // Cold lifecycle only: returns after the caller's pool/layout/clock-anchor
    // initialization, or rethrows its failure after join. No map/pose/step wait;
    // cold failure does not invoke runtime fatal while the host holds a lock.
    void start();
    void notify() noexcept;
    // Nonblocking World Stop/fatal fence: admit no new work or output. Active
    // acquisition/geometry/output pins live through the existing completion.
    void fence() noexcept;
    // Lifecycle thread only. Fence, join caller, then release source/ROS owners.
    // GPU destruction occurs on its legal caller before join returns.
    void stop();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Exact existing due state, exposed only so affected original targets can
// exercise TimerManager1.17.4 boundaries without a ROS node or another timer.
struct SharedSensorDue {
    ros::Time last_expected, next_expected;
    ros::Duration period;
    void start(const ros::Time& now, const ros::Duration& duration);
    void clockRegression(const ros::Time& now);
    bool due(const ros::Time& now) const;
    void complete(const ros::Time& finish);
};

} // namespace xgc2_world_lidar
