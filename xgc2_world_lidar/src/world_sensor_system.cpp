#include "xgc2_world_lidar/world_sensor_system.hpp"
#include "xgc2_world_lidar/observation_contract.h"
#include "xgc2_world_lidar/scan_pool.h"
#include "xgc2_world_lidar/shared_cloud_cpu.hpp"
#ifdef XGC_WORLD_LIDAR_GPU
#include "xgc2_world_lidar/shared_cloud_gpu.hpp"
#endif
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <pcl_conversions/pcl_conversions.h>
#include <stdexcept>
#include <thread>

namespace xgc2_world_lidar {
namespace {
// The layout of a PointCloud2 of `float_fields` float32 fields, then an
// int32 `vehicle_id` when `with_id`; no records yet.
sensor_msgs::PointCloud2 cloudLayout(const std::vector<std::string>& float_fields,
                                     bool with_id,
                                     const std::string& frame_id) {
    sensor_msgs::PointCloud2 msg;
    msg.header.frame_id = frame_id;
    msg.height = 1;
    uint32_t offset = 0;
    for (const auto& name : float_fields) {
        sensor_msgs::PointField f;
        f.name = name;
        f.offset = offset;
        f.datatype = sensor_msgs::PointField::FLOAT32;
        f.count = 1;
        msg.fields.push_back(f);
        offset += 4;
    }
    if (with_id) {
        sensor_msgs::PointField f;
        f.name = "vehicle_id";
        f.offset = offset;
        f.datatype = sensor_msgs::PointField::INT32;
        f.count = 1;
        msg.fields.push_back(f);
        offset += 4;
    }
    msg.is_bigendian = false;
    msg.point_step = offset;
    msg.is_dense = true;
    return msg;
}

// Sets the record count and stamp of a cloud whose `data` holds the records.
void finishCloud(sensor_msgs::PointCloud2* msg, std::size_t count, const ros::Time& stamp) {
    msg->header.stamp = stamp;
    msg->width = static_cast<uint32_t>(count);
    msg->row_step = msg->point_step * msg->width;
}

// A PointCloud2 of `count` records: `float_fields` float32 fields, then an
// int32 `vehicle_id` when `with_id`. `write(i, floats, id)` fills record i.
template <class Write>
sensor_msgs::PointCloud2 makeCloud(std::size_t count,
                                   const std::vector<std::string>& float_fields,
                                   bool with_id,
                                   const ros::Time& stamp,
                                   const std::string& frame_id,
                                   Write&& write) {
    sensor_msgs::PointCloud2 msg = cloudLayout(float_fields, with_id, frame_id);
    finishCloud(&msg, count, stamp);
    msg.data.resize(msg.row_step);
    std::vector<float> floats(float_fields.size());
    uint8_t* out = msg.data.data();
    for (std::size_t i = 0; i < count; ++i) {
        int32_t id = -1;
        write(i, floats.data(), &id);
        std::memcpy(out, floats.data(), 4 * floats.size());
        if (with_id)
            std::memcpy(out + 4 * floats.size(), &id, 4);
        out += msg.point_step;
    }
    return msg;
}

// x y z float32 (the /points and global-map layout), + vehicle_id if tagged.
sensor_msgs::PointCloud2 toCloud(const std::vector<Eigen::Vector3d>& points,
                                 const ros::Time& stamp,
                                 const std::string& frame_id) {
    return makeCloud(points.size(),
                     {"x", "y", "z"},
                     false,
                     stamp,
                     frame_id,
                     [&](std::size_t i, float* f, int32_t*) {
                         f[0] = static_cast<float>(points[i].x());
                         f[1] = static_cast<float>(points[i].y());
                         f[2] = static_cast<float>(points[i].z());
                     });
}

// The beam hits as /points records (x y z float32, + vehicle_id), in beam
// order: one scan feeds both topics, so /points are exactly the beam hits.
std::size_t packHits(const std::vector<Beam>& beams, bool with_id, std::vector<uint8_t>* data) {
    const std::size_t stride = with_id ? 16 : 12;
    std::size_t count = 0;
    for (const auto& b : beams)
        count += b.hit ? 1 : 0;
    data->resize(count * stride);
    uint8_t* out = data->data();
    for (const auto& b : beams) {
        if (!b.hit)
            continue;
        const Eigen::Vector3d p = b.origin + b.range * b.direction;
        const float xyz[3] = {
            static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())};
        std::memcpy(out, xyz, sizeof xyz);
        if (with_id) {
            const int32_t id = b.vehicle_id;
            std::memcpy(out + sizeof xyz, &id, sizeof id);
        }
        out += stride;
    }
    return count;
}

// Free-segment records: origin, unit direction, range, hit (1/0).
sensor_msgs::PointCloud2 toBeamCloud(const std::vector<Beam>& beams,
                                     bool with_id,
                                     const ros::Time& stamp,
                                     const std::string& frame_id) {
    return makeCloud(beams.size(),
                     {"x", "y", "z", "dx", "dy", "dz", "range", "hit"},
                     with_id,
                     stamp,
                     frame_id,
                     [&](std::size_t i, float* f, int32_t* id) {
                         const Beam& b = beams[i];
                         for (int k = 0; k < 3; ++k) {
                             f[k] = static_cast<float>(b.origin[k]);
                             f[3 + k] = static_cast<float>(b.direction[k]);
                         }
                         f[6] = static_cast<float>(b.range);
                         f[7] = b.hit ? 1.0f : 0.0f;
                         *id = b.vehicle_id;
                     });
}

} // namespace

void SharedSensorDue::start(const ros::Time& now, const ros::Duration& duration) {
    period = duration;
    last_expected = now;
    next_expected = now + period;
}
void SharedSensorDue::clockRegression(const ros::Time& now) {
    if (now < last_expected) {
        last_expected = now;
        next_expected = now + period;
    }
}
bool SharedSensorDue::due(const ros::Time& now) const {
    return next_expected <= now;
}
void SharedSensorDue::complete(const ros::Time& finish) {
    // ros_comm1.17.4 TimerManager::updateNext: strict <, not finish+period.
    if (next_expected <= finish) {
        last_expected = next_expected;
        next_expected += period;
    }
    if (next_expected + period < finish)
        next_expected = finish;
}

struct WorldSensorSystem::Impl {
    struct NormalRuntime {
        std::unique_ptr<WorldLidar> lidar;
        ros::Time next_scan;
        std::vector<VehicleBody> others;
        std::vector<Beam> beams;
        sensor_msgs::PointCloud2 output;
        std::size_t count = 0;
    };
    struct SharedSensorRuntime {
        CropResult scratch;
        sensor_msgs::PointCloud2 output;
    };
    struct SharedRuntime {
        SharedSensorDue due;
        std::unique_ptr<SharedCloudCpu> cpu;
        std::vector<SharedSensorRuntime> sensors;
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr loaded_cloud;
        std::size_t output_begin = 0;
    };

    WorldSensorConfiguration configuration;
    WorldSensorCallbacks callbacks;
    WorldSensorAcquisition acquisition;
    std::vector<NormalRuntime> normal;
    std::vector<SharedRuntime> shared;
    std::vector<std::size_t> active;
    std::vector<VehicleBody> bodies;
    std::unique_ptr<ScanPool> pool;
    // The normal node also had a max-rate ROS timer. Preserve that opportunity
    // clock; its PER-SENSOR next_scan below remains the independent skip policy.
    SharedSensorDue normal_tick;
    ros::Time last_normal_tick;
    std::unique_ptr<WorldLidar> map_lidar;
    std::shared_ptr<const LidarScene> map_scene;
    ros::Time map_stamp;
    std::uint64_t map_clear_version = 0;
    bool map_dirty = false, map_clear_pending = false;
#ifdef XGC_WORLD_LIDAR_GPU
    std::unique_ptr<SharedCloudGpu> gpu;
    std::size_t resident_gpu_group = static_cast<std::size_t>(-1);
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr resident_gpu_cloud;
#endif
    std::atomic<bool> fenced{false};
    std::mutex mutex;
    std::condition_variable wake;
    std::uint64_t notifications = 0;
    bool started = false, initialized = false;
    std::exception_ptr initialization_error;
    std::thread caller;

    Impl(WorldSensorConfiguration c, WorldSensorCallbacks cb)
        : configuration(std::move(c)), callbacks(std::move(cb)) {
        if (!callbacks.now || !callbacks.capture || !callbacks.publish || !callbacks.fatal)
            throw std::invalid_argument("World sensor clock/capture/output/fatal owners required");
        if (configuration.normal_worker_threads < 1)
            throw std::invalid_argument("sensor caller parallelism must be positive");
        if (!std::isfinite(configuration.normal_poll_rate_hz) ||
            configuration.normal_poll_rate_hz <= 0)
            throw std::invalid_argument("original normal poll rate must be finite and positive");
        acquisition.sources.resize(configuration.source_count);
        acquisition.bodies.resize(configuration.bodies.size());
        normal.resize(configuration.normal.size());
        active.reserve(normal.size());
        bodies.reserve(configuration.bodies.size());
        std::size_t output_count = normal.size();
        for (std::size_t i = 0; i < normal.size(); ++i) {
            const auto& equipment = configuration.normal[i];
            if (equipment.source_index >= configuration.source_count ||
                !std::isfinite(equipment.rate_hz) || equipment.rate_hz <= 0 ||
                !std::isfinite(equipment.pose_timeout_sec) || equipment.pose_timeout_sec <= 0)
                throw std::invalid_argument("invalid frozen normal source/rate/age");
            auto& runtime = normal[i];
            runtime.lidar = std::make_unique<WorldLidar>(equipment.sensor);
            runtime.others.reserve(configuration.bodies.size());
            runtime.output =
                cloudLayout({"x", "y", "z"}, equipment.with_bodies, equipment.frame_id);
        }
        shared.resize(configuration.shared.size());
        for (std::size_t i = 0; i < shared.size(); ++i) {
            const auto& equipment = configuration.shared[i];
            const auto& metadata = equipment.metadata;
            if (equipment.source_indices.empty())
                throw std::invalid_argument("original shared source/output bindings required");
            if (metadata.backend == "cpu") {
                validateSensorMetadata(metadata);
                if (equipment.worker_threads < 1)
                    throw std::invalid_argument("original shared CPU width must be positive");
            } else if (metadata.backend == "gpu") {
                validateGpuSensorMetadata(metadata);
#ifndef XGC_WORLD_LIDAR_GPU
                throw std::invalid_argument("GPU unsupported by this CPU-only build; no fallback");
#endif
            } else {
                throw std::invalid_argument("unsupported explicit sensor backend; no fallback");
            }
            auto& runtime = shared[i];
            runtime.output_begin = output_count;
            runtime.sensors.resize(equipment.source_indices.size());
            output_count += runtime.sensors.size();
            for (auto index : equipment.source_indices)
                if (index >= configuration.source_count)
                    throw std::invalid_argument("shared source is outside frozen source roster");
        }
        acquisition.demand.resize(output_count);
        if (configuration.normal_map) {
            if (!std::isfinite(configuration.normal_map->period_sec) ||
                configuration.normal_map->period_sec <= 0)
                throw std::invalid_argument("original normal map period required");
            map_lidar = std::make_unique<WorldLidar>(configuration.normal_map->sensor);
        }
    }

    void capture() {
        for (auto& source : acquisition.sources)
            source.ready = false;
        for (auto& body : acquisition.bodies)
            body.ready = false;
        acquisition.geometry.reset();
        callbacks.capture(acquisition);
        if (acquisition.sources.size() != configuration.source_count ||
            acquisition.bodies.size() != configuration.bodies.size())
            throw std::invalid_argument("capture changed frozen acquisition cardinality");
        std::size_t count = normal.size();
        for (const auto& group : shared)
            count += group.sensors.size();
        if (acquisition.demand.size() != count)
            throw std::invalid_argument("capture changed frozen output demand cardinality");
        for (auto& source : acquisition.sources)
            source.geometry_version = acquisition.geometry ? acquisition.geometry->version : 0;
    }

    void publish(std::size_t index,
                 SensorOutputKind kind,
                 const sensor_msgs::PointCloud2& cloud,
                 const SensorAcquisition& sample) {
        if (!fenced.load(std::memory_order_acquire))
            callbacks.publish(index, kind, cloud, sample);
    }

    void publishMap(const ros::Time& now) {
        if (!map_lidar)
            return;
        if (acquisition.normal_map_clear_version != map_clear_version) {
            map_clear_version = acquisition.normal_map_clear_version;
            map_clear_pending = true;
            // A successfully reinstalled scene may retain the same immutable
            // pin. Refill the cleared latch through its original ready/demand
            // gates, just as the old installObstacles always marked it dirty.
            map_dirty = true;
            map_stamp = ros::Time();
        }
        if (!acquisition.enabled)
            return;
        SensorAcquisition stamp;
        stamp.source_stamp = now;
        stamp.world_commit = acquisition.world_commit;
        stamp.geometry_version = acquisition.geometry ? acquisition.geometry->version : 0;
        if (map_clear_pending) {
            publish(kWorldSensorMapOutput,
                    SensorOutputKind::Map,
                    toCloud({}, now, configuration.normal_map->frame_id),
                    stamp);
            map_clear_pending = false;
        }
        if (!acquisition.normal_geometry_ready || !acquisition.normal_state_fresh ||
            !acquisition.geometry || !acquisition.geometry->normal_map)
            return;
        const auto& geometry = acquisition.geometry;
        if (map_scene != geometry->normal_map) {
            map_scene = geometry->normal_map;
            map_lidar->setScene(map_scene);
            map_dirty = true;
        }
        if (!map_dirty || !acquisition.normal_map_requested)
            return;
        if (!geometry->normal_has_dynamic || map_stamp.isZero() ||
            (now - map_stamp).toSec() >= configuration.normal_map->period_sec) {
            const auto points = map_lidar->globalMap(map_lidar->config().surface_spacing);
            publish(kWorldSensorMapOutput,
                    SensorOutputKind::Map,
                    toCloud(points, now, configuration.normal_map->frame_id),
                    stamp);
            map_dirty = false;
            map_stamp = now;
        }
    }

    void normalScan(std::size_t i) {
        auto& runtime = normal[i];
        const auto& equipment = configuration.normal[i];
        const auto& sample = acquisition.sources[equipment.source_index];
        Pose body;
        body.position = sample.position;
        body.orientation = sample.orientation;
        const Pose p = sensorWorldPose(body, equipment.mount);
        Eigen::Quaterniond q = p.orientation.normalized();
        if (equipment.yaw_only) {
            const Eigen::Vector3d x = q * Eigen::Vector3d::UnitX();
            q = Eigen::AngleAxisd(std::atan2(x.y(), x.x()), Eigen::Vector3d::UnitZ());
        }
        runtime.others.clear();
        for (const auto& other : bodies)
            if (equipment.with_bodies && other.id != equipment.body_id)
                runtime.others.push_back(other);
        if (equipment.publish_beams) {
            runtime.beams = runtime.lidar->scanWithBeams(p.position, q, runtime.others);
        } else {
            runtime.count = runtime.lidar->scanInto(
                p.position, q, runtime.others, equipment.with_bodies, &runtime.output.data);
        }
    }

    void serviceNormal(const ros::Time& scheduled, const ros::Time& now) {
        capture();
        if (!last_normal_tick.isZero() && now < last_normal_tick) {
            for (auto& runtime : normal)
                runtime.next_scan = ros::Time();
            map_stamp = ros::Time();
        }
        last_normal_tick = now;
        publishMap(now); // existing latched clear is independent of sensor demand
        if (!acquisition.enabled || !acquisition.normal_geometry_ready ||
            !acquisition.normal_state_fresh || !acquisition.geometry)
            return;
        active.clear();
        bodies.clear();
        bool body_truth_ready = true;
        for (std::size_t i = 0; i < acquisition.bodies.size(); ++i) {
            const auto& sample = acquisition.bodies[i];
            // The World source owner has applied the original body's pose age.
            if (!sample.ready) {
                body_truth_ready = false;
                continue;
            }
            const auto& binding = configuration.bodies[i];
            bodies.push_back({binding.id, sample.position, binding.radius});
        }
        for (std::size_t i = 0; i < normal.size(); ++i) {
            const auto& equipment = configuration.normal[i];
            const auto& sample = acquisition.sources[equipment.source_index];
            const auto& demand = acquisition.demand[i];
            if (!sample.ready ||
                !freshObservationTime(
                    sample.source_stamp.toSec(), now.toSec(), equipment.pose_timeout_sec) ||
                (!demand.points && !(equipment.publish_beams && demand.beams)) ||
                (equipment.with_bodies && configuration.require_complete_body_roster &&
                 !body_truth_ready))
                continue;
            if (i >= acquisition.geometry->normal.size() || !acquisition.geometry->normal[i])
                continue;
            auto& runtime = normal[i];
            if (runtime.lidar->scene() != acquisition.geometry->normal[i])
                runtime.lidar->setScene(acquisition.geometry->normal[i]);
            if (!runtime.next_scan.isZero() && scheduled < runtime.next_scan - ros::Duration(1e-6))
                continue;
            // Original normal next_scan skip-expired policy, NOT shared completion.
            if (runtime.next_scan.isZero())
                runtime.next_scan = scheduled;
            runtime.next_scan += ros::Duration(1.0 / equipment.rate_hz);
            if (runtime.next_scan <= scheduled)
                runtime.next_scan = scheduled + ros::Duration(1.0 / equipment.rate_hz);
            active.push_back(i);
        }
        pool->run(active.size(), configuration.normal_worker_threads, [&](std::size_t k) {
            normalScan(active[k]);
        });
        // Packing/ROS publication never occupies a ScanPool compute worker.
        for (auto i : active) {
            auto& runtime = normal[i];
            const auto& equipment = configuration.normal[i];
            const auto& sample = acquisition.sources[equipment.source_index];
            const auto count =
                equipment.publish_beams
                    ? packHits(runtime.beams, equipment.with_bodies, &runtime.output.data)
                    : runtime.count;
            finishCloud(&runtime.output, count, sample.source_stamp);
            // Configured beams imply both outputs, including beams-only admission.
            publish(i, SensorOutputKind::Points, runtime.output, sample);
            if (equipment.publish_beams)
                publish(i,
                        SensorOutputKind::Beams,
                        toBeamCloud(runtime.beams,
                                    equipment.with_bodies,
                                    sample.source_stamp,
                                    equipment.frame_id),
                        sample);
        }
    }

    void serviceShared(std::size_t group_index) {
        capture(); // at actual service start, never while the previous tick is busy
        auto& runtime = shared[group_index];
        const auto& equipment = configuration.shared[group_index];
        const auto& metadata = equipment.metadata;
        if (!acquisition.geometry || group_index >= acquisition.geometry->shared.size() ||
            !acquisition.geometry->shared[group_index])
            return; // ordinary not-ready
        const auto& cloud = acquisition.geometry->shared[group_index];
        if (metadata.backend == "cpu" && runtime.loaded_cloud != cloud) {
            auto next = std::make_unique<SharedCloudCpu>();
            next->load(cloud, metadata);
            runtime.cpu = std::move(next);
            runtime.loaded_cloud = cloud;
        }
#ifdef XGC_WORLD_LIDAR_GPU
        if (metadata.backend == "gpu" &&
            (resident_gpu_group != group_index || resident_gpu_cloud != cloud)) {
            // One actual resident model/context. Per-group CPU loaded_cloud is
            // NOT evidence that this renderer still contains that GPU model.
            // Previous borrowed scan buffer was packed/consumed before here.
            gpu.reset();
            resident_gpu_cloud.reset();
            resident_gpu_group = static_cast<std::size_t>(-1);
            gpu = std::make_unique<SharedCloudGpu>();
            gpu->load(cloud, metadata);
            resident_gpu_cloud = cloud;
            resident_gpu_group = group_index;
        }
#endif
        if (metadata.backend == "cpu") {
            pool->run(runtime.sensors.size(), equipment.worker_threads, [&](std::size_t i) {
                const auto& source = acquisition.sources[equipment.source_indices[i]];
                if (source.ready) {
                    runtime.cpu->scanInto(
                        source.position, source.orientation, &runtime.sensors[i].scratch);
                }
            });
        }
        for (std::size_t i = 0; i < runtime.sensors.size(); ++i) {
            const auto& source = acquisition.sources[equipment.source_indices[i]];
            if (!source.ready)
                continue;
            auto& sensor = runtime.sensors[i];
            const void* points = nullptr;
            if (metadata.backend == "cpu") {
                if (!sensor.scratch.radius_candidates)
                    continue;
                sensor.output.width = sensor.scratch.cloud.size();
                sensor.output.is_dense = true;
                points = sensor.scratch.cloud.points.data();
            } else {
#ifdef XGC_WORLD_LIDAR_GPU
                const auto& scan =
                    gpu->scan(source.position, source.orientation, source.source_stamp.toSec());
                sensor.output.width = scan.size();
                sensor.output.is_dense = scan.is_dense;
                points = scan.points.data();
#endif
            }
            sensor.output.height = 1;
            sensor.output.row_step = sensor.output.width * sensor.output.point_step;
            if (metadata.backend == "gpu") {
                sensor.output.data.resize(sensor.output.row_step);
                if (!sensor.output.data.empty())
                    std::memcpy(sensor.output.data.data(), points, sensor.output.data.size());
            } else if (sensor.output.row_step == 0) {
                sensor.output.data.clear();
            } else {
                const auto* bytes = static_cast<const std::uint8_t*>(points);
                sensor.output.data.assign(bytes, bytes + sensor.output.row_step);
            }
            sensor.output.header.frame_id = metadata.frame_id;
            sensor.output.header.stamp =
                metadata.stamp_policy == "zero" ? ros::Time(0) : source.source_stamp;
            // Original shared tick has NO subscriber gate. GPU's borrowed scan
            // buffer is packed/consumed before its next scan invocation.
            publish(runtime.output_begin + i, SensorOutputKind::Points, sensor.output, source);
        }
    }

    void run() {
        std::exception_ptr error;
        bool cold_complete = false;
        try {
            std::size_t capacity = normal.empty() ? 1 : configuration.normal_worker_threads;
            for (const auto& equipment : configuration.shared)
                if (equipment.metadata.backend == "cpu")
                    capacity = std::max(
                        capacity,
                        std::min(equipment.worker_threads, equipment.source_indices.size()));
            pool = std::make_unique<ScanPool>(capacity);
            const auto start_time = callbacks.now();
            normal_tick.start(start_time, ros::Duration(1.0 / configuration.normal_poll_rate_hz));
            for (std::size_t i = 0; i < shared.size(); ++i) {
                const auto& metadata = configuration.shared[i].metadata;
                shared[i].due.start(start_time, ros::Duration(1.0 / metadata.publish_rate_hz));
                for (auto& sensor : shared[i].sensors) {
                    if (metadata.backend == "cpu") {
                        pcl::PointCloud<pcl::PointXYZ> empty;
                        pcl::toROSMsg(empty, sensor.output);
                        if (sensor.output.point_step != sizeof(pcl::PointXYZ))
                            throw std::runtime_error("actual PCL XYZ layout required");
                    } else {
#ifdef XGC_WORLD_LIDAR_GPU
                        pcl::PointCloud<pcl::PointXYZI> empty;
                        pcl::toROSMsg(empty, sensor.output);
                        if (sensor.output.point_step != sizeof(pcl::PointXYZI))
                            throw std::runtime_error(
                                "actual original GPU XYZ/intensity layout required");
#endif
                    }
                }
            }
            // Only fixed pool/layout/original clock-anchor initialization.
            // Never wait for geometry, a pose, a timer opportunity or physics.
            {
                std::lock_guard<std::mutex> lock(mutex);
                initialized = true;
            }
            cold_complete = true;
            wake.notify_all();
            while (!fenced.load(std::memory_order_acquire)) {
                std::uint64_t seen;
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    seen = notifications;
                }
                if (callbacks.process_inputs)
                    callbacks.process_inputs();
                if (fenced.load())
                    break;
                auto now = callbacks.now();
                if (!normal.empty() || map_lidar) {
                    normal_tick.clockRegression(now);
                    if (normal_tick.due(now)) {
                        serviceNormal(normal_tick.next_expected, now);
                        normal_tick.complete(callbacks.now());
                    }
                }
                for (std::size_t i = 0; i < shared.size() && !fenced.load(); ++i) {
                    if (callbacks.process_inputs)
                        callbacks.process_inputs();
                    if (fenced.load())
                        break;
                    now = callbacks.now();
                    shared[i].due.clockRegression(now);
                    if (shared[i].due.due(now)) {
                        serviceShared(i);
                        // Busy ends only after scan, pack AND synchronous output return.
                        shared[i].due.complete(callbacks.now());
                    }
                }
                if (fenced.load())
                    break;
                now = callbacks.now();
                ros::Time next;
                bool have_due = false;
                if (!normal.empty() || map_lidar) {
                    next = normal_tick.next_expected;
                    have_due = true;
                }
                for (const auto& group : shared)
                    if (!have_due || group.due.next_expected < next) {
                        next = group.due.next_expected;
                        have_due = true;
                    }
                std::unique_lock<std::mutex> lock(mutex);
                const auto notified = [&] { return fenced.load() || notifications != seen; };
                if (!have_due)
                    wake.wait(lock, notified);
                else if (next > now)
                    wake.wait_for(
                        lock, std::chrono::duration<double>((next - now).toSec()), notified);
            }
        } catch (...) {
            fenced.store(true, std::memory_order_release);
            error = std::current_exception();
        }
#ifdef XGC_WORLD_LIDAR_GPU
        gpu.reset(); // also on exception/fence, before the caller exits
        resident_gpu_cloud.reset();
#endif
        pool.reset();
        acquisition.geometry.reset();
        for (auto& group : shared) {
            group.cpu.reset();
            group.loaded_cloud.reset();
        }
        for (auto& sensor : normal)
            sensor.lidar.reset();
        map_lidar.reset();
        map_scene.reset();
        if (error && !cold_complete) {
            // start may be holding the host's cold lifecycle lock. Do NOT call
            // fatal here: it can take that host lock and deadlock start/join.
            {
                std::lock_guard<std::mutex> lock(mutex);
                initialization_error = error;
                initialized = true;
            }
            wake.notify_all();
        } else if (error) {
            callbacks.fatal(error); // resources gone, host lock free, no self-join
        }
    }
};

WorldSensorSystem::WorldSensorSystem(WorldSensorConfiguration c, WorldSensorCallbacks cb)
    : impl_(std::make_unique<Impl>(std::move(c), std::move(cb))) {}
WorldSensorSystem::~WorldSensorSystem() {
    stop();
}
void WorldSensorSystem::start() {
    std::unique_lock<std::mutex> lock(impl_->mutex);
    if (impl_->started)
        throw std::logic_error("World sensor lifecycle already started");
    impl_->started = true;
    impl_->caller = std::thread([this] { impl_->run(); });
    impl_->wake.wait(lock, [this] { return impl_->initialized; });
    const auto error = impl_->initialization_error;
    lock.unlock();
    if (error) {
        impl_->caller.join(); // caller already cleaned resources; no host fatal callback
        std::rethrow_exception(error);
    }
}
void WorldSensorSystem::notify() noexcept {
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        ++impl_->notifications;
    }
    impl_->wake.notify_one();
}
void WorldSensorSystem::fence() noexcept {
    impl_->fenced.store(true, std::memory_order_release);
    impl_->wake.notify_one();
}
void WorldSensorSystem::stop() {
    fence();
    if (impl_->caller.joinable()) {
        if (impl_->caller.get_id() == std::this_thread::get_id())
            throw std::logic_error("World sensor lifecycle cannot self-join");
        impl_->caller.join();
    }
}

} // namespace xgc2_world_lidar
