/**
 * @file world_lidar_node.cpp
 * @brief ROS1 edge of the xgc2_world_lidar core.
 *
 * One node serves the vehicles listed in ~vehicle_ids (XGC2 starts one node
 * per robot that enables the sensor).
 *
 * Scene (~scene_source)
 *   snapshot  <scene_namespace>/snapshot  xgc2_geometry_msgs/SceneSnapshot
 *             <scene_namespace>/state     xgc2_geometry_msgs/SceneState (dynamic obstacle poses)
 *             The common scene document, used with the lightweight simulator.
 *   gazebo    ~geometry_library_topic     xgc2_geometry_msgs/GeometryLibrary (latched)
 *             ~instances_topic            xgc2_geometry_msgs/ConvexBodyArray
 *             The obstacles Gazebo actually simulates, published by the
 *             xgc2_gazebo_scene system plugin.
 * Inputs
 *   pose_topic_pattern          geometry_msgs/PoseStamped per vehicle
 * Outputs
 *   output_topic_pattern        sensor_msgs/PointCloud2 per vehicle (default
 *                               <ns>/simple_lidar/points), frame "world",
 *                               x y z float32 (+ int32 vehicle_id when ~vehicle_bodies is on)
 *   beams_topic_pattern         raycast / depth_frustum only (~publish_beams; default
 *                               <ns>/simple_lidar/beams): one record per beam, frame "world", float32 x y z (origin)
 *                               dx dy dz (unit) range hit(1/0) (+ int32 vehicle_id)
 *   global_map_topic            sensor_msgs/PointCloud2, latched, penetrating full-scene
 *                               cloud (~publish_global_map)
 * Model
 *   ~mode, ~preset and the sensor parameters; see prepareParameters().
 * Switches
 *   ~enabled                    initial state (default true). While disabled the node
 *                               scans nothing and publishes nothing (no empty clouds:
 *                               an empty cloud would read as observed-free space).
 *   ~set_enabled                std_srvs/SetBool: runtime toggle of ~enabled.
 *   ~enabled_vehicles           list of vehicle ids that get a sensor; empty = all
 *                               served vehicles. The others get no publisher.
 *
 * Snapshot rules: an older revision of the current epoch is ignored; a
 * snapshot with dynamic obstacles waits for a state of the same epoch and
 * revision; a state that lacks a dynamic obstacle is refused. Gazebo rules:
 * instances wait for the library; the scene is rebuilt only when an instance
 * changes. The sensor is rebuilt at most once per scan tick.
 */

#include <ros/package.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/SetBool.h>
#include <xgc2_geometry_msgs/ConvexBodyArray.h>
#include <xgc2_geometry_msgs/GeometryLibrary.h>
#include <xgc2_geometry_msgs/SceneSnapshot.h>
#include <xgc2_geometry_msgs/SceneState.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <regex>
#include <future>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "xgc2_world_lidar/convex_body_conversion.h"
#include "xgc2_world_lidar/scene_conversion.h"
#include "xgc2_world_lidar/observation_contract.h"
#include "xgc2_world_lidar/world_lidar.h"

namespace xgc2_world_lidar {
namespace {

std::string substituteId(std::string pattern, int id) {
    const std::string key = "{id}";
    for (std::size_t at = pattern.find(key); at != std::string::npos; at = pattern.find(key))
        pattern.replace(at, key.size(), std::to_string(id));
    return pattern;
}

Pose poseOf(const geometry_msgs::Pose& p) {
    Pose out;
    out.position = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
    out.orientation = Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    return out;
}

// A PointCloud2 of `count` records: `float_fields` float32 fields, then an
// int32 `vehicle_id` when `with_id`. `write(i, floats, id)` fills record i.
template <class Write>
sensor_msgs::PointCloud2 makeCloud(std::size_t count, const std::vector<std::string>& float_fields, bool with_id,
                                   const ros::Time& stamp, const std::string& frame_id, Write&& write) {
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(count);
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
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = true;
    msg.data.resize(msg.row_step);
    std::vector<float> floats(float_fields.size());
    uint8_t* out = msg.data.data();
    for (std::size_t i = 0; i < count; ++i) {
        int32_t id = -1;
        write(i, floats.data(), &id);
        std::memcpy(out, floats.data(), 4 * floats.size());
        if (with_id) std::memcpy(out + 4 * floats.size(), &id, 4);
        out += msg.point_step;
    }
    return msg;
}

// x y z float32 (the /points and global-map layout), + vehicle_id if tagged.
sensor_msgs::PointCloud2 toCloud(const std::vector<Eigen::Vector3d>& points, const ros::Time& stamp,
                                 const std::string& frame_id) {
    return makeCloud(points.size(), {"x", "y", "z"}, false, stamp, frame_id,
                     [&](std::size_t i, float* f, int32_t*) {
                         f[0] = static_cast<float>(points[i].x());
                         f[1] = static_cast<float>(points[i].y());
                         f[2] = static_cast<float>(points[i].z());
                     });
}

sensor_msgs::PointCloud2 toTaggedCloud(const std::vector<TaggedPoint>& points, bool with_id, const ros::Time& stamp,
                                       const std::string& frame_id) {
    return makeCloud(points.size(), {"x", "y", "z"}, with_id, stamp, frame_id,
                     [&](std::size_t i, float* f, int32_t* id) {
                         f[0] = static_cast<float>(points[i].point.x());
                         f[1] = static_cast<float>(points[i].point.y());
                         f[2] = static_cast<float>(points[i].point.z());
                         *id = points[i].vehicle_id;
                     });
}

// Free-segment records: origin, unit direction, range, hit (1/0).
sensor_msgs::PointCloud2 toBeamCloud(const std::vector<Beam>& beams, bool with_id, const ros::Time& stamp,
                                     const std::string& frame_id) {
    return makeCloud(beams.size(), {"x", "y", "z", "dx", "dy", "dz", "range", "hit"}, with_id, stamp, frame_id,
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

// Private parameters of a process launch: an empty mode / preset or a zero
// numeric override means "not set", so the preset or default applies. A
// preset (config/presets/<name>.yaml, flat "key: scalar" lines) then fills
// every key that is still unset; explicit parameters always win.
void prepareParameters(ros::NodeHandle& pnh) {
    for (const char* key : {"mode", "preset"}) {
        std::string value;
        if (pnh.getParam(key, value) && value.empty()) pnh.deleteParam(key);
    }
    for (const char* key : {"rate", "range", "h_fov_deg", "v_fov_deg", "h_res", "v_res"}) {
        double value = 0.0;
        int integer = 0;
        if ((pnh.getParam(key, value) && value == 0.0) || (pnh.getParam(key, integer) && integer == 0))
            pnh.deleteParam(key);
    }
    // depth_frustum: the image is h_res x v_res when given, and a camera
    // without an explicit field of view looks 90 deg wide (the 360 deg lidar
    // default is not a pinhole camera).
    std::string mode;
    if (pnh.getParam("mode", mode) && mode == "depth_frustum") {
        int value = 0;
        if (!pnh.hasParam("width") && pnh.getParam("h_res", value)) pnh.setParam("width", value);
        if (!pnh.hasParam("height") && pnh.getParam("v_res", value)) pnh.setParam("height", value);
        if (!pnh.hasParam("h_fov_deg")) pnh.setParam("h_fov_deg", 90.0);
    }
    std::string preset;
    if (!pnh.getParam("preset", preset)) return;
    if (!std::regex_match(preset, std::regex("[a-z0-9_]{1,64}")))
        throw std::invalid_argument("~preset must be a preset name, got '" + preset + "'");
    const std::string path = ros::package::getPath("xgc2_world_lidar") + "/config/presets/" + preset + ".yaml";
    std::ifstream file(path);
    if (!file) throw std::invalid_argument("unknown ~preset '" + preset + "' (" + path + ")");
    const std::regex line_pattern(R"(^\s*([A-Za-z_][A-Za-z0-9_]*)\s*:\s*([^#]*?)\s*(#.*)?$)");
    const std::regex integer_pattern(R"(^[-+]?[0-9]+$)");
    const std::regex real_pattern(R"(^[-+]?([0-9]+\.?[0-9]*|\.[0-9]+)([eE][-+]?[0-9]+)?$)");
    std::string line;
    while (std::getline(file, line)) {
        std::smatch match;
        if (!std::regex_match(line, match, line_pattern) || match[2].str().empty()) continue;
        const std::string key = match[1].str();
        std::string value = match[2].str();
        if (pnh.hasParam(key)) continue;
        if (value == "true" || value == "false") pnh.setParam(key, value == "true");
        else if (std::regex_match(value, integer_pattern)) pnh.setParam(key, std::stoi(value));
        else if (std::regex_match(value, real_pattern)) pnh.setParam(key, std::stod(value));
        else {
            if (value.size() >= 2 && (value.front() == '"' || value.front() == '\'') && value.back() == value.front())
                value = value.substr(1, value.size() - 2);
            pnh.setParam(key, value);
        }
    }
}

SensorConfig loadConfig(const ros::NodeHandle& pnh) {
    SensorConfig c;
    const std::string mode = pnh.param<std::string>("mode", "raycast");
    if (mode == "raycast") c.mode = SensorConfig::kRaycast;
    else if (mode == "penetrating") c.mode = SensorConfig::kPenetrating;
    else if (mode == "depth_frustum") c.mode = SensorConfig::kDepthFrustum;
    else throw std::invalid_argument("~mode must be raycast, penetrating or depth_frustum, got '" + mode + "'");
    c.range = pnh.param("range", c.range);
    c.min_range = pnh.param("min_range", c.min_range);
    c.penetrating_heading_crop = pnh.param("penetrating_heading_crop", c.penetrating_heading_crop);
    c.heading_cos_min = pnh.param("heading_cos_min", c.heading_cos_min);
    c.vertical_slab_tan = pnh.param("vertical_slab_tan", c.vertical_slab_tan);
    c.penetrating_keep_buried = pnh.param("penetrating_keep_buried", c.penetrating_keep_buried);
    c.width = pnh.param("width", c.width);
    c.height = pnh.param("height", c.height);
    c.fx = pnh.param("fx", c.fx);
    c.fy = pnh.param("fy", c.fy);
    c.cx = pnh.param("cx", c.cx);
    c.cy = pnh.param("cy", c.cy);
    c.h_fov_deg = pnh.param("h_fov_deg", c.h_fov_deg);
    c.v_fov_deg = pnh.param("v_fov_deg", c.v_fov_deg);
    c.h_res = pnh.param("h_res", c.h_res);
    c.v_res = pnh.param("v_res", c.v_res);
    c.surface_spacing = pnh.param("surface_spacing", c.surface_spacing);
    c.noise_std = pnh.param("noise_std", c.noise_std);
    c.seed = static_cast<unsigned>(pnh.param("seed", 0));
    return c;
}

class WorldLidarNode {
public:
    WorldLidarNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh) {
        frame_id_ = pnh.param<std::string>("frame_id", "world");
        yaw_only_ = pnh.param("yaw_only", false);
        pose_timeout_ = pnh.param("pose_timeout", 0.5);
        worker_threads_ = std::max(1, pnh.param("worker_threads", 1));
        const double rate = pnh.param("rate", 10.0);
        if (!std::isfinite(rate) || !(rate > 0.0))
            throw std::invalid_argument("~rate must be finite and > 0");
        if (frame_id_ != "world" || !std::isfinite(pose_timeout_) || pose_timeout_ <= 0.0)
            throw std::invalid_argument("world lidar needs frame_id=world and a positive finite pose_timeout");
        std::vector<double> mount(6, 0.0);
        if (pnh.hasParam("sensor_pose") && !pnh.getParam("sensor_pose", mount))
            throw std::invalid_argument("~sensor_pose must be a six-number array");
        sensor_mount_ = mountingPose(mount);

        lidar_ = std::make_unique<WorldLidar>(loadConfig(pnh));
        has_beams_ = lidar_->config().mode != SensorConfig::kPenetrating;
        publish_beams_ = has_beams_ && pnh.param("publish_beams", false);
        vehicle_bodies_ = pnh.param("vehicle_bodies", false);
        vehicle_body_radius_ = pnh.param("vehicle_body_radius", 0.3);
        if (vehicle_bodies_ && !(vehicle_body_radius_ > 0.0))
            throw std::invalid_argument("~vehicle_body_radius must be > 0");

        if (pnh.param("publish_global_map", false)) {
            SensorConfig map_config;
            map_config.mode = SensorConfig::kPenetrating;
            map_config.surface_spacing = pnh.param("global_map_spacing", 0.1);
            map_config.penetrating_keep_buried = lidar_->config().penetrating_keep_buried;
            map_lidar_ = std::make_unique<WorldLidar>(map_config);
            map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
                pnh.param<std::string>("global_map_topic", "/world_lidar/global_map"), 1, true);
            map_period_ = 1.0 / std::max(1e-3, pnh.param("global_map_rate", 1.0));
        }

        std::vector<int> ids;
        if (!pnh.getParam("vehicle_ids", ids)) {
            const int n = pnh.param("num_vehicles", 1);
            for (int i = 1; i <= n; ++i) ids.push_back(i);
        }
        std::vector<int> enabled_ids;
        pnh.getParam("enabled_vehicles", enabled_ids);
        for (int id : enabled_ids)
            if (std::find(ids.begin(), ids.end(), id) == ids.end())
                throw std::invalid_argument("~enabled_vehicles names vehicle " + std::to_string(id) +
                                            ", which this node does not serve");
        enabled_ = pnh.param("enabled", true);
        const std::string pose_pattern =
            pnh.param<std::string>("pose_topic_pattern", "/uav{id}/mavros/local_position/pose");
        // Relative names resolve in the node namespace: <ns>/simple_lidar/{points,beams}.
        const std::string out_pattern = pnh.param<std::string>("output_topic_pattern", "simple_lidar/points");
        const std::string beams_pattern = pnh.param<std::string>("beams_topic_pattern", "simple_lidar/beams");
        vehicles_.resize(ids.size());
        for (std::size_t k = 0; k < ids.size(); ++k) {
            Vehicle& v = vehicles_[k];
            v.id = ids[k];
            v.sensed = enabled_ids.empty() ||
                       std::find(enabled_ids.begin(), enabled_ids.end(), v.id) != enabled_ids.end();
            if (v.sensed) {
                v.pub = nh_.advertise<sensor_msgs::PointCloud2>(substituteId(out_pattern, v.id), 1);
                if (publish_beams_)
                    v.beams_pub = nh_.advertise<sensor_msgs::PointCloud2>(substituteId(beams_pattern, v.id), 1);
            }
            // Poses of every served vehicle (a vehicle without a sensor is still a body for the others).
            v.sub = nh_.subscribe<geometry_msgs::PoseStamped>(
                substituteId(pose_pattern, v.id), 1,
                [this, k](const geometry_msgs::PoseStamped::ConstPtr& msg) {
                    if (!validObservationPose(poseOf(msg->pose), msg->header.frame_id, frame_id_)) {
                        vehicles_[k].pose.reset();
                        ROS_WARN_THROTTLE(1.0, "world_lidar: invalid or non-world body pose; no scan");
                        return;
                    }
                    vehicles_[k].pose = msg;
                });
            ROS_INFO("world_lidar: vehicle %d  %s -> %s", v.id, substituteId(pose_pattern, v.id).c_str(),
                     v.sensed ? substituteId(out_pattern, v.id).c_str() : "(no sensor: not in ~enabled_vehicles)");
        }
        enable_srv_ = pnh.advertiseService("set_enabled", &WorldLidarNode::setEnabled, this);

        const std::string scene_source = pnh.param<std::string>("scene_source", "snapshot");
        if (scene_source == "snapshot") {
            const std::string scene_ns = pnh.param<std::string>("scene_namespace", "/xgc/scene");
            snapshot_sub_ = nh_.subscribe(scene_ns + "/snapshot", 1, &WorldLidarNode::snapshotCallback, this);
            state_sub_ = nh_.subscribe(scene_ns + "/state", 1, &WorldLidarNode::stateCallback, this);
            ROS_INFO("world_lidar: scene from %s/{snapshot,state}", scene_ns.c_str());
        } else if (scene_source == "gazebo") {
            gazebo_source_ = true;
            const std::string library_topic = pnh.param<std::string>(
                "geometry_library_topic", "/xgc2/simulation/obstacles/geometry_library");
            const std::string instances_topic =
                pnh.param<std::string>("instances_topic", "/xgc2/simulation/obstacles/instances");
            library_sub_ = nh_.subscribe(library_topic, 1, &WorldLidarNode::libraryCallback, this);
            instances_sub_ = nh_.subscribe(instances_topic, 1, &WorldLidarNode::instancesCallback, this);
            ROS_INFO("world_lidar: scene from Gazebo obstacle truth %s + %s", library_topic.c_str(),
                     instances_topic.c_str());
        } else {
            throw std::invalid_argument("~scene_source must be snapshot or gazebo, got '" + scene_source + "'");
        }
        timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &WorldLidarNode::tick, this);
        const SensorConfig& c = lidar_->config();
        const char* mode = c.mode == SensorConfig::kRaycast
                               ? "raycast"
                               : (c.mode == SensorConfig::kPenetrating ? "penetrating" : "depth_frustum");
        ROS_INFO("world_lidar: mode=%s range=[%.2f, %.1f] m fov=%.0fx%.0f deg beams=%zu spacing=%.2f noise=%.3f "
                 "rate=%.1f Hz beams_topic=%s vehicle_bodies=%s",
                 mode, c.min_range, c.range, c.h_fov_deg, c.v_fov_deg, lidar_->beamCount(), c.surface_spacing,
                 c.noise_std, rate, publish_beams_ ? "on" : "off", vehicle_bodies_ ? "on" : "off");
        ROS_INFO("world_lidar: %s (toggle with %s)", enabled_ ? "enabled" : "disabled: publishing nothing",
                 enable_srv_.getService().c_str());
    }

private:
    struct Vehicle {
        int id = 0;
        ros::Subscriber sub;
        ros::Publisher pub;
        ros::Publisher beams_pub;
        geometry_msgs::PoseStamped::ConstPtr pose;
        bool sensed = true;  // in ~enabled_vehicles
    };

    bool setEnabled(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
        enabled_ = req.data;
        res.success = true;
        res.message = enabled_ ? "world_lidar enabled" : "world_lidar disabled: publishing nothing";
        ROS_INFO("world_lidar: %s", res.message.c_str());
        return true;
    }

    void snapshotCallback(const xgc2_geometry_msgs::SceneSnapshot::ConstPtr& msg) {
        if (snapshot_ && msg->epoch == snapshot_->epoch && msg->revision <= snapshot_->revision) return;
        // A new document invalidates the installed scene immediately. In particular,
        // do not scan the previous epoch while waiting for a new dynamic state.
        scene_ready_ = false;
        pending_ = false;
        map_dirty_ = false;
        state_.reset();
        if (msg->header.frame_id != frame_id_) {
            snapshot_.reset();
            ROS_ERROR("world_lidar: snapshot frame is not world");
            return;
        }
        snapshot_ = msg;
        const bool has_dynamic = std::any_of(msg->obstacles.begin(), msg->obstacles.end(),
                                             [](const auto& o) { return o.dynamic; });
        if (!has_dynamic) {
            pending_ = true;
        } else {
            ROS_INFO("world_lidar: snapshot %s/%lu has dynamic obstacles; awaiting matching state",
                     msg->epoch.c_str(), static_cast<unsigned long>(msg->revision));
        }
    }

    void stateCallback(const xgc2_geometry_msgs::SceneState::ConstPtr& msg) {
        if (!snapshot_ || msg->epoch != snapshot_->epoch || msg->revision != snapshot_->revision) return;
        if (msg->header.frame_id != frame_id_) {
            state_.reset();
            pending_ = false;
            scene_ready_ = false;
            return;
        }
        state_ = msg;
        pending_ = true;
    }

    void libraryCallback(const xgc2_geometry_msgs::GeometryLibrary::ConstPtr& msg) {
        library_ = msg;
        if (instances_) pending_ = true;
    }

    // Gazebo republishes instances at a fixed rate; rebuild only on change.
    void instancesCallback(const xgc2_geometry_msgs::ConvexBodyArray::ConstPtr& msg) {
        const bool changed = !instances_ || !sameBodies(*instances_, *msg);
        instances_ = msg;
        if (changed && library_) pending_ = true;
    }

    static bool sameBodies(const xgc2_geometry_msgs::ConvexBodyArray& a, const xgc2_geometry_msgs::ConvexBodyArray& b) {
        if (a.instances.size() != b.instances.size()) return false;
        for (std::size_t i = 0; i < a.instances.size(); ++i) {
            const auto& x = a.instances[i];
            const auto& y = b.instances[i];
            if (x.name != y.name || x.geometry_type != y.geometry_type || x.pose != y.pose || x.scale != y.scale)
                return false;
        }
        return true;
    }

    std::vector<Obstacle> gazeboObstacles(bool* has_dynamic) const {
        std::vector<GeometryTemplateDescription> library;
        for (const auto& t : library_->templates) {
            GeometryTemplateDescription d;
            d.type = t.type;
            for (const auto& p : t.support_points) d.support_points.emplace_back(p.x, p.y, p.z);
            library.push_back(std::move(d));
        }
        std::vector<ConvexBodyDescription> bodies;
        *has_dynamic = false;
        for (const auto& i : instances_->instances) {
            ConvexBodyDescription d;
            d.name = i.name;
            d.geometry_type = i.geometry_type;
            d.pose = poseOf(i.pose);
            d.scale = Eigen::Vector3d(i.scale.x, i.scale.y, i.scale.z);
            *has_dynamic = *has_dynamic || !i.is_static;
            bodies.push_back(std::move(d));
        }
        return fromConvexBodies(library, bodies);
    }

    // Builds the obstacle list from the selected scene source and installs
    // it into both sensors. Throws std::invalid_argument on a refused scene.
    void install(const ros::Time& now) {
        if (gazebo_source_) {
            bool has_dynamic = false;
            const std::vector<Obstacle> obstacles = gazeboObstacles(&has_dynamic);
            installObstacles(obstacles, has_dynamic, now);
            return;
        }
        std::map<std::string, const geometry_msgs::Pose*> state_pose;
        if (state_)
            for (const auto& s : state_->obstacles) state_pose[s.id] = &s.pose;
        std::vector<SceneObstacleDescription> scene;
        bool has_dynamic = false;
        for (const auto& o : snapshot_->obstacles) {
            SceneObstacleDescription d;
            d.id = o.id;
            const auto it = state_pose.find(o.id);
            if (o.dynamic && it == state_pose.end())
                throw std::invalid_argument("state lacks dynamic obstacle '" + o.id + "'");
            has_dynamic = has_dynamic || o.dynamic;
            d.pose = poseOf(it != state_pose.end() ? *it->second : o.pose);
            for (const auto& p : o.parts) {
                ScenePartDescription part;
                part.type = p.geometry.type;
                part.pose = poseOf(p.pose);
                part.size = Eigen::Vector3d(p.geometry.size.x, p.geometry.size.y, p.geometry.size.z);
                part.radius = p.geometry.radius;
                part.height = p.geometry.height;
                for (const auto& v : p.geometry.vertices) part.vertices.emplace_back(v.x, v.y, v.z);
                d.parts.push_back(std::move(part));
            }
            scene.push_back(std::move(d));
        }
        installObstacles(toObstacles(scene), has_dynamic, now);
    }

    void installObstacles(const std::vector<Obstacle>& obstacles, bool has_dynamic, const ros::Time& now) {
        lidar_->setScene(obstacles);
        scene_ready_ = true;
        map_obstacles_ = obstacles;
        map_has_dynamic_ = has_dynamic;
        map_dirty_ = true;
        publishMap(now);
    }

    // Publishes the global map of the installed scene when it changed (rate
    // limited while obstacles move). Never while disabled: the map waits.
    void publishMap(const ros::Time& now) {
        if (!enabled_ || !scene_ready_ || !map_dirty_ || !map_lidar_ || !map_pub_.getNumSubscribers()) return;
        const bool has_dynamic = map_has_dynamic_;
        if (map_lidar_ && (!has_dynamic || map_stamp_.isZero() || (now - map_stamp_).toSec() >= map_period_)) {
            map_dirty_ = false;
            const std::vector<Obstacle>& obstacles = map_obstacles_;
            map_lidar_->setScene(obstacles);
            const auto cloud = map_lidar_->globalMap(map_lidar_->config().surface_spacing);
            map_pub_.publish(toCloud(cloud, now, frame_id_));
            map_stamp_ = now;
            if (!has_dynamic)
                ROS_INFO("world_lidar: scene installed, %zu solids, global map %zu points", obstacles.size(),
                         cloud.size());
        }
    }

    void tick(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        if (!enabled_) return;
        const bool subscribed = std::any_of(vehicles_.begin(), vehicles_.end(), [this](const Vehicle& v) {
            return v.sensed && (v.pub.getNumSubscribers() ||
                               (publish_beams_ && v.beams_pub.getNumSubscribers()));
        });
        if (!subscribed && !(map_lidar_ && map_pub_.getNumSubscribers())) return;
        if (pending_) {
            pending_ = false;
            try {
                install(now);
            } catch (const std::exception& e) {
                scene_ready_ = false;
                ROS_ERROR_THROTTLE(1.0, "world_lidar: scene refused: %s", e.what());
            }
        }
        if (!enabled_) return;  // publishes nothing, not even an empty cloud
        if (!scene_ready_) {
            ROS_WARN_THROTTLE(5.0, "world_lidar: no scene installed yet; not publishing");
            return;
        }
        if (map_has_dynamic_ && !gazebo_source_ &&
            (!state_ || !freshObservationTime(state_->header.stamp.toSec(), now.toSec(), pose_timeout_))) {
            ROS_WARN_THROTTLE(1.0, "world_lidar: dynamic scene state is stale or from a future clock epoch");
            return;
        }
        publishMap(now);
        std::vector<std::size_t> fresh;
        for (std::size_t k = 0; k < vehicles_.size(); ++k) {
            const auto& pose = vehicles_[k].pose;
            if (!pose) continue;
            if (!freshObservationTime(pose->header.stamp.toSec(), now.toSec(), pose_timeout_)) {
                if (vehicles_[k].sensed)
                    ROS_WARN_THROTTLE(5.0, "world_lidar: pose of vehicle %d is stale", vehicles_[k].id);
                continue;
            }
            fresh.push_back(k);
        }
        std::vector<std::size_t> active;
        for (std::size_t k : fresh)
            if (vehicles_[k].sensed && (vehicles_[k].pub.getNumSubscribers() ||
                (publish_beams_ && vehicles_[k].beams_pub.getNumSubscribers()))) active.push_back(k);
        // Bodies of the other served vehicles with a fresh pose (tagged returns).
        std::vector<VehicleBody> bodies;
        if (vehicle_bodies_)
            for (std::size_t k : fresh)
                bodies.push_back({vehicles_[k].id, poseOf(vehicles_[k].pose->pose).position, vehicle_body_radius_});
        std::vector<std::vector<TaggedPoint>> clouds(active.size());
        std::vector<std::vector<Beam>> beams(active.size());
        auto scanOne = [&](std::size_t i) {
            const Vehicle& v = vehicles_[active[i]];
            const Pose p = sensorWorldPose(poseOf(v.pose->pose), sensor_mount_);
            Eigen::Quaterniond q = p.orientation.normalized();
            if (yaw_only_) {
                const Eigen::Vector3d x = q * Eigen::Vector3d::UnitX();
                q = Eigen::AngleAxisd(std::atan2(x.y(), x.x()), Eigen::Vector3d::UnitZ());
            }
            std::vector<VehicleBody> others;
            for (const auto& b : bodies)
                if (b.id != v.id) others.push_back(b);
            if (!has_beams_) {
                clouds[i] = lidar_->scanTagged(p.position, q, others);
                return;
            }
            // One scan feeds both topics, so /points are exactly the beam hits.
            beams[i] = lidar_->scanWithBeams(p.position, q, others);
            for (const auto& b : beams[i])
                if (b.hit) clouds[i].push_back({b.origin + b.range * b.direction, b.vehicle_id});
        };
        if (worker_threads_ <= 1 || active.size() <= 1) {
            for (std::size_t i = 0; i < active.size(); ++i) scanOne(i);
        } else {
            std::vector<std::future<void>> jobs;
            const std::size_t workers = std::min<std::size_t>(worker_threads_, active.size());
            for (std::size_t w = 0; w < workers; ++w)
                jobs.push_back(std::async(std::launch::async, [&, w] {
                    for (std::size_t i = w; i < active.size(); i += workers) scanOne(i);
                }));
            for (auto& j : jobs) j.get();
        }
        for (std::size_t i = 0; i < active.size(); ++i) {
            const Vehicle& v = vehicles_[active[i]];
            const ros::Time stamp = v.pose->header.stamp;
            v.pub.publish(toTaggedCloud(clouds[i], vehicle_bodies_, stamp, frame_id_));
            if (publish_beams_) v.beams_pub.publish(toBeamCloud(beams[i], vehicle_bodies_, stamp, frame_id_));
        }
    }

    ros::NodeHandle nh_;
    std::string frame_id_;
    Pose sensor_mount_;
    bool yaw_only_ = false;
    double pose_timeout_ = 0.5;
    int worker_threads_ = 1;
    bool has_beams_ = true;
    bool publish_beams_ = true;
    bool vehicle_bodies_ = false;
    double vehicle_body_radius_ = 0.3;
    std::unique_ptr<WorldLidar> lidar_;
    std::unique_ptr<WorldLidar> map_lidar_;
    ros::Publisher map_pub_;
    std::vector<Obstacle> map_obstacles_;
    bool map_has_dynamic_ = false;
    bool map_dirty_ = false;
    bool enabled_ = true;
    ros::ServiceServer enable_srv_;
    double map_period_ = 1.0;
    ros::Time map_stamp_;
    std::vector<Vehicle> vehicles_;
    ros::Subscriber snapshot_sub_, state_sub_, library_sub_, instances_sub_;
    bool gazebo_source_ = false;
    xgc2_geometry_msgs::GeometryLibrary::ConstPtr library_;
    xgc2_geometry_msgs::ConvexBodyArray::ConstPtr instances_;
    ros::Timer timer_;
    xgc2_geometry_msgs::SceneSnapshot::ConstPtr snapshot_;
    xgc2_geometry_msgs::SceneState::ConstPtr state_;
    bool pending_ = false;
    bool scene_ready_ = false;
};

}  // namespace
}  // namespace xgc2_world_lidar

int main(int argc, char** argv) {
    ros::init(argc, argv, "world_lidar");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    try {
        xgc2_world_lidar::prepareParameters(pnh);
        xgc2_world_lidar::WorldLidarNode node(nh, pnh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("world_lidar: %s", e.what());
        return 1;
    }
    return 0;
}
