#include "xgc2_world_lidar/scan_pool.h"
#include "xgc2_world_lidar/world_sensor_system.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <gtest/gtest.h>
#include <mutex>
#include <set>
#include <stdexcept>

namespace xgc2_world_lidar {
namespace {
ros::Time at(double seconds) {
    ros::Time t;
    return t.fromSec(seconds);
}

struct Observed {
    std::size_t index;
    SensorOutputKind kind;
    sensor_msgs::PointCloud2 cloud;
    SensorAcquisition source;
};

// Fake only the World source/output boundary. Every observation below runs
// the real existing WorldLidar or SharedCloudCpu, not a replacement sensor.
struct Rig {
    std::atomic<double> time{0};
    std::mutex mutex;
    std::condition_variable changed;
    SensorAcquisition source;
    std::shared_ptr<WorldSensorGeometry> geometry;
    std::vector<Observed> outputs;
    bool clock_started = false, ready = true, block_output = false, output_entered = false,
         release_output = false, throw_capture = false, failed = false, points_demand = false,
         beams_demand = false, map_demand = false, normal_ready = true;
    std::size_t captures = 0;
    std::uint64_t clear_version = 0;
    std::unique_ptr<WorldSensorSystem> system;

    Rig() : geometry(std::make_shared<WorldSensorGeometry>()) {
        source.ready = true;
        source.source_stamp = at(0);
        source.source_version = 7;
        source.generation = 1;
        source.world_commit = 100;
        geometry->version = 11;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->header.frame_id = "map";
        cloud->push_back(pcl::PointXYZ(1, 0, 0));
        cloud->push_back(pcl::PointXYZ(2, 0, 0));
        geometry->shared.push_back(cloud);
    }
    ~Rig() {
        {
            std::lock_guard<std::mutex> lock(mutex);
            release_output = true;
        }
        changed.notify_all();
        if (system)
            system->stop();
    }
    WorldSensorCallbacks callbacks() {
        WorldSensorCallbacks c;
        c.now = [this] {
            const auto result = at(time.load());
            {
                std::lock_guard<std::mutex> lock(mutex);
                clock_started = true;
            }
            changed.notify_all();
            return result;
        };
        c.capture = [this](WorldSensorAcquisition& out) {
            std::lock_guard<std::mutex> lock(mutex);
            ++captures;
            if (throw_capture)
                throw std::runtime_error("actual source-owner failure");
            for (auto& value : out.sources) {
                value = source;
                value.ready = ready;
            }
            out.geometry = geometry;
            out.world_commit = source.world_commit;
            out.normal_map_requested = map_demand;
            out.normal_geometry_ready = normal_ready;
            out.normal_map_clear_version = clear_version;
            for (auto& demand : out.demand) {
                demand.points = points_demand;
                demand.beams = beams_demand;
            }
            changed.notify_all();
        };
        c.publish = [this](std::size_t index,
                           SensorOutputKind kind,
                           const sensor_msgs::PointCloud2& cloud,
                           const SensorAcquisition& sample) {
            std::unique_lock<std::mutex> lock(mutex);
            output_entered = true;
            changed.notify_all();
            if (block_output)
                changed.wait(lock, [this] { return release_output; });
            outputs.push_back({index, kind, cloud, sample}); // caller consumes owned copy
            changed.notify_all();
        };
        c.fatal = [this](const std::exception_ptr&) {
            std::lock_guard<std::mutex> lock(mutex);
            failed = true;
            changed.notify_all();
        };
        return c;
    }
    bool wait(const std::function<bool()>& predicate) {
        std::unique_lock<std::mutex> lock(mutex);
        return changed.wait_for(lock, std::chrono::seconds(3), predicate);
    }
    void start(WorldSensorConfiguration config) {
        system = std::make_unique<WorldSensorSystem>(std::move(config), callbacks());
        system->start();
        EXPECT_TRUE(wait([this] { return clock_started; }));
    }
    void advance(double now) {
        time.store(now);
        system->notify();
    }
};

WorldSensorConfiguration sharedConfig() {
    WorldSensorConfiguration config;
    config.source_count = 1;
    SharedSensorEquipment shared;
    shared.source_indices = {0};
    auto& m = shared.metadata;
    m.observation_model = "crop_through";
    m.backend = "cpu";
    m.prevoxel_leaf_m = {{0.1f, 0.1f, 0.1f}};
    m.range_m = 5;
    m.publish_rate_hz = 12;
    m.frame_id = "map";
    m.stamp_policy = "zero";
    config.shared.push_back(std::move(shared));
    return config;
}

TEST(WorldSensorDue, ExactCompletionAndBackwardClock) {
    SharedSensorDue due;
    due.start(at(10), ros::Duration(1));
    EXPECT_EQ(due.last_expected, at(10));
    EXPECT_EQ(due.next_expected, at(11));
    EXPECT_FALSE(due.due(at(10.5)));
    EXPECT_TRUE(due.due(at(11)));
    due.complete(at(13)); // next+period == finish: strict < must not re-anchor
    EXPECT_EQ(due.last_expected, at(11));
    EXPECT_EQ(due.next_expected, at(12));
    due.complete(at(15));
    EXPECT_EQ(due.last_expected, at(12));
    EXPECT_EQ(due.next_expected, at(15));
    due.clockRegression(at(11));
    EXPECT_EQ(due.last_expected, at(11));
    EXPECT_EQ(due.next_expected, at(12));
    due.clockRegression(at(11.5));
    EXPECT_EQ(due.next_expected, at(12));
}

TEST(WorldSensorExecution, SharedNoInputDoesNotRestartPhaseAndHasNoSubscriberGate) {
    Rig rig;
    rig.ready = false;
    rig.start(sharedConfig());
    rig.advance(0.1);
    ASSERT_TRUE(rig.wait([&] { return rig.captures >= 1; }));
    {
        std::lock_guard<std::mutex> lock(rig.mutex);
        rig.ready = true;
    }
    rig.advance(0.2);
    ASSERT_TRUE(rig.wait([&] { return !rig.outputs.empty(); }));
    rig.system->stop();
    EXPECT_FALSE(rig.failed);
    ASSERT_FALSE(rig.outputs.empty());
    const auto& result = rig.outputs.front();
    EXPECT_EQ(result.cloud.header.frame_id, "map");
    EXPECT_TRUE(result.cloud.header.stamp.isZero());
    EXPECT_EQ(result.cloud.point_step, sizeof(pcl::PointXYZ));
    EXPECT_EQ(result.source.generation, 1U);
    EXPECT_EQ(result.source.source_version, 7U);
    EXPECT_EQ(result.source.world_commit, 100U);
    EXPECT_EQ(result.source.geometry_version, 11U);
    EXPECT_GT(result.cloud.width, 0U); // demand false did NOT suppress shared output
}

TEST(WorldSensorExecution, BusyOwnsSampleThroughPublishAndFenceThenJoin) {
    Rig rig;
    rig.block_output = true;
    rig.start(sharedConfig());
    rig.advance(0.1);
    ASSERT_TRUE(rig.wait([&] { return rig.output_entered; }));
    std::size_t captured;
    {
        std::lock_guard<std::mutex> lock(rig.mutex);
        captured = rig.captures;
        rig.source.generation = 2;
        rig.source.source_version = 8;
        rig.source.position.x() = 1;
        rig.source.world_commit = 101;
    }
    rig.advance(0.4);
    rig.system->notify();
    rig.system->notify();
    {
        std::lock_guard<std::mutex> lock(rig.mutex);
        EXPECT_EQ(rig.captures, captured);
    }
    rig.system->fence(); // already-entered synchronous publication owns its old values
    {
        std::lock_guard<std::mutex> lock(rig.mutex);
        rig.release_output = true;
    }
    rig.changed.notify_all();
    rig.system->stop();
    ASSERT_EQ(rig.outputs.size(), 1U);
    EXPECT_EQ(rig.outputs.front().source.generation, 1U);
    EXPECT_EQ(rig.outputs.front().source.source_version, 7U);
    EXPECT_DOUBLE_EQ(rig.outputs.front().source.position.x(), 0);
    EXPECT_EQ(rig.captures, captured); // notifications did not enqueue payloads
}

TEST(WorldSensorExecution, NormalBeamAdmissionStillPublishesBothOriginalOutputs) {
    Rig rig;
    WorldSensorConfiguration config;
    config.source_count = 1;
    NormalSensorEquipment sensor;
    sensor.sensor.h_res = 1;
    sensor.sensor.v_res = 1;
    sensor.publish_beams = true;
    config.normal.push_back(sensor);
    rig.geometry->normal.push_back(std::make_shared<LidarScene>(
        std::vector<Obstacle>{Obstacle::box(Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(1, 2, 2))}));
    rig.start(std::move(config));
    rig.advance(0.1);
    ASSERT_TRUE(rig.wait([&] { return rig.captures >= 1; }));
    {
        std::lock_guard<std::mutex> lock(rig.mutex);
        EXPECT_TRUE(rig.outputs.empty());
        rig.beams_demand = true;
    }
    rig.advance(0.2);
    ASSERT_TRUE(rig.wait([&] { return rig.outputs.size() >= 2; }));
    rig.system->stop();
    ASSERT_EQ(rig.outputs.size(), 2U);
    EXPECT_EQ(rig.outputs[0].kind, SensorOutputKind::Points);
    EXPECT_EQ(rig.outputs[1].kind, SensorOutputKind::Beams);
    EXPECT_EQ(rig.outputs[0].cloud.point_step, 12U);
    EXPECT_EQ(rig.outputs[1].cloud.point_step, 32U);
    EXPECT_EQ(rig.outputs[0].source.source_stamp, at(0));
    EXPECT_EQ(rig.outputs[1].source.source_stamp, at(0));
}

TEST(WorldSensorExecution, OriginalMapClearAndLatchedMapAreOneOutputOwner) {
    Rig rig;
    WorldSensorConfiguration config;
    NormalMapEquipment map;
    map.sensor.mode = SensorConfig::kPenetrating;
    config.normal_map = map;
    rig.geometry->normal_map = std::make_shared<LidarScene>(
        std::vector<Obstacle>{Obstacle::box(Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(1, 2, 2))});
    rig.clear_version = 1;
    rig.map_demand = true;
    rig.start(std::move(config));
    rig.advance(0.1);
    ASSERT_TRUE(rig.wait([&] { return rig.outputs.size() >= 2; }));
    const auto same_pin = rig.geometry->normal_map;
    {
        std::lock_guard<std::mutex> lock(rig.mutex);
        rig.clear_version = 2;
        rig.normal_ready = false;
    }
    rig.advance(0.2);
    ASSERT_TRUE(rig.wait([&] { return rig.outputs.size() >= 3; }));
    {
        std::lock_guard<std::mutex> lock(rig.mutex);
        rig.normal_ready = true;
    }
    rig.advance(0.3);
    ASSERT_TRUE(rig.wait([&] { return rig.outputs.size() >= 4; }));
    rig.system->stop();
    ASSERT_EQ(rig.outputs.size(), 4U);
    EXPECT_EQ(rig.outputs[0].index, kWorldSensorMapOutput);
    EXPECT_EQ(rig.outputs[0].kind, SensorOutputKind::Map);
    EXPECT_EQ(rig.outputs[0].cloud.width, 0U);
    EXPECT_GT(rig.outputs[1].cloud.width, 0U);
    EXPECT_EQ(rig.outputs[1].cloud.header.frame_id, "world");
    EXPECT_EQ(rig.outputs[1].cloud.header.stamp, at(0.1));
    EXPECT_EQ(rig.geometry->normal_map, same_pin);
    EXPECT_EQ(rig.outputs[2].cloud.width, 0U);
    EXPECT_EQ(rig.outputs[3].cloud.data, rig.outputs[1].cloud.data);
    EXPECT_EQ(rig.outputs[3].cloud.header.stamp, at(0.3));
}

TEST(WorldSensorExecution, OriginalBackendErrorFencesWorldAndReportsOnce) {
    Rig rig;
    rig.throw_capture = true;
    rig.start(sharedConfig());
    rig.advance(0.1);
    ASSERT_TRUE(rig.wait([&] { return rig.failed; }));
    rig.system->stop();
    EXPECT_TRUE(rig.outputs.empty());
    EXPECT_EQ(rig.captures, 1U);
}

TEST(WorldSensorPool, EffectiveBatchWidthsAndExceptionReuse) {
    ScanPool pool(4); // one capacity reused for original serial/parallel groups
    const auto caller = std::this_thread::get_id();
    for (std::size_t limit : {1U, 2U, 1U, 3U, 1U}) {
        std::vector<std::atomic<unsigned>> calls(17);
        for (auto& count : calls)
            count.store(0);
        std::mutex mutex;
        std::set<std::thread::id> participants;
        pool.run(calls.size(), limit, [&](std::size_t i) {
            ++calls[i];
            std::lock_guard<std::mutex> lock(mutex);
            participants.insert(std::this_thread::get_id());
        });
        for (const auto& count : calls)
            EXPECT_EQ(count.load(), 1U);
        EXPECT_LE(participants.size(), limit);
        if (limit == 1) {
            ASSERT_EQ(participants.size(), 1U);
            EXPECT_EQ(*participants.begin(), caller);
        }
    }
    std::atomic<unsigned> attempted{0};
    EXPECT_THROW(pool.run(8,
                          2,
                          [&](std::size_t i) {
                              ++attempted;
                              if (i == 2)
                                  throw std::runtime_error("original job error");
                          }),
                 std::runtime_error);
    EXPECT_EQ(attempted.load(), 8U);
    attempted = 0;
    pool.run(8, 1, [&](std::size_t) {
        EXPECT_EQ(std::this_thread::get_id(), caller);
        ++attempted;
    });
    EXPECT_EQ(attempted.load(), 8U);
}

TEST(WorldSensorExecution, GeometryQueueHookPrecedesShortCaptureOnSensorCaller) {
    Rig rig;
    auto callbacks = rig.callbacks();
    const auto world_thread = std::this_thread::get_id();
    std::thread::id geometry_thread, capture_thread, output_thread;
    bool processed = false, capture_after_processing = false;
    callbacks.process_inputs = [&] {
        std::lock_guard<std::mutex> lock(rig.mutex);
        geometry_thread = std::this_thread::get_id();
        processed = true;
    };
    const auto original_capture = callbacks.capture;
    callbacks.capture = [&](WorldSensorAcquisition& frame) {
        {
            std::lock_guard<std::mutex> lock(rig.mutex);
            capture_thread = std::this_thread::get_id();
            capture_after_processing = processed;
        }
        original_capture(frame);
    };
    const auto original_publish = callbacks.publish;
    callbacks.publish = [&](std::size_t i,
                            SensorOutputKind kind,
                            const sensor_msgs::PointCloud2& cloud,
                            const SensorAcquisition& sample) {
        output_thread = std::this_thread::get_id();
        original_publish(i, kind, cloud, sample);
    };
    rig.system = std::make_unique<WorldSensorSystem>(sharedConfig(), std::move(callbacks));
    rig.system->start();
    ASSERT_TRUE(rig.wait([&] { return rig.clock_started; }));
    rig.advance(0.1);
    ASSERT_TRUE(rig.wait([&] { return !rig.outputs.empty(); }));
    rig.system->stop();
    EXPECT_TRUE(capture_after_processing);
    EXPECT_NE(geometry_thread, world_thread);
    EXPECT_EQ(capture_thread, geometry_thread);
    EXPECT_EQ(output_thread, geometry_thread);
}

TEST(WorldSensorExecution, ColdStartFailureReturnsWithoutRuntimeFatalOrInputWait) {
    Rig rig;
    auto callbacks = rig.callbacks();
    std::atomic<bool> runtime_fatal{false};
    callbacks.now = []() -> ros::Time {
        throw std::runtime_error("cold clock initialization failed");
    };
    callbacks.fatal = [&](const std::exception_ptr&) { runtime_fatal = true; };
    rig.system = std::make_unique<WorldSensorSystem>(sharedConfig(), std::move(callbacks));
    EXPECT_THROW(rig.system->start(), std::runtime_error);
    rig.system->stop();
    EXPECT_FALSE(runtime_fatal.load());
    EXPECT_EQ(rig.captures, 0U);
    EXPECT_TRUE(rig.outputs.empty());
}
} // namespace
} // namespace xgc2_world_lidar
