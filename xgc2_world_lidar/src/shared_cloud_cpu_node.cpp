#include "xgc2_world_lidar/world_sensor_system.hpp"
#include <algorithm>
#include <atomic>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdexcept>
#include <string>
#include <vector>

// This ROS entry remains only for a real external (e.g. Gazebo) source. The
// lightweight World attaches the same library directly; its recipe retires
// this process, not the per-robot topics or their scientific source semantics.
class SharedCloudNode {
    struct Sensor {
        ros::Publisher pub;
        ros::Subscriber pose;
        xgc2_world_lidar::SensorAcquisition source;
    };
    ros::NodeHandle nh_{"~"};
    ros::CallbackQueue geometry_callbacks_;
    ros::NodeHandle geometry_nh_{nh_};
    ros::Subscriber input_;
    xgc2_world_lidar::SensorMetadata metadata_;
    std::vector<Sensor> sensors_;
    std::mutex source_mutex_;
    std::shared_ptr<const xgc2_world_lidar::WorldSensorGeometry> geometry_;
    std::unique_ptr<xgc2_world_lidar::WorldSensorSystem> system_;
    bool loaded_ = false, gpu_ = false;
    std::atomic<bool> failed_{false};

public:
    SharedCloudNode() {
        geometry_nh_.setCallbackQueue(&geometry_callbacks_);
        auto& m = metadata_;
        nh_.getParam("observation_model", m.observation_model);
        nh_.getParam("backend", m.backend);
        if (m.backend != "cpu" && m.backend != "gpu")
            throw std::invalid_argument("unsupported explicit sensor backend; no fallback");
        gpu_ = m.backend == "gpu";
        if (!gpu_) {
            std::vector<double> leaf;
            if (!nh_.getParam("prevoxel_leaf_m", leaf) || leaf.size() != 3)
                throw std::runtime_error("prevoxel XYZ required");
            for (int i = 0; i < 3; ++i)
                m.prevoxel_leaf_m[i] = leaf[i];
        }
        nh_.getParam("range_m", m.range_m);
        double value;
        if (nh_.getParam("heading_cos_min", value))
            m.heading_cos_min = value;
        if (nh_.getParam("vertical_slab_tan", value))
            m.vertical_slab_tan = value;
        nh_.getParam("publish_rate_hz", m.publish_rate_hz);
        nh_.getParam("frame_id", m.frame_id);
        nh_.getParam("stamp_policy", m.stamp_policy);
        nh_.getParam("input_cloud_topic", m.input_cloud_topic);
        nh_.getParam("pose_type", m.pose_type);
        if (gpu_) {
#ifdef XGC_WORLD_LIDAR_GPU
            XmlRpc::XmlRpcValue gpu_prevoxel;
            if (nh_.getParam("prevoxel_leaf_m", gpu_prevoxel))
                throw std::invalid_argument("GPU does not implement CPU prevoxel_leaf_m");
            if (!nh_.getParam("min_range_m", m.min_range_m) ||
                !nh_.getParam("h_fov_deg", m.h_fov_deg) ||
                !nh_.getParam("v_fov_deg", m.v_fov_deg) || !nh_.getParam("h_res", m.h_res) ||
                !nh_.getParam("v_res", m.v_res) ||
                !nh_.getParam("point_cover_spacing_m", m.point_cover_spacing_m))
                throw std::invalid_argument(
                    "explicit GPU spherical grid/near/point-cover required");
            xgc2_world_lidar::validateGpuSensorMetadata(m);
#else
            throw std::invalid_argument("GPU is unsupported by this CPU-only build; no fallback");
#endif
        } else {
            xgc2_world_lidar::validateSensorMetadata(m);
        }

        std::vector<std::string> poses, outputs;
        if (!nh_.getParam("pose_topics", poses) || !nh_.getParam("output_topics", outputs) ||
            poses.empty() || poses.size() != outputs.size() || m.input_cloud_topic.empty())
            throw std::runtime_error("actual cloud/pose/output bindings required");
        if (m.pose_type != "nav_msgs/Odometry" && m.pose_type != "geometry_msgs/PoseStamped")
            throw std::runtime_error("unsupported explicit pose type");
        sensors_.resize(poses.size());
        xgc2_world_lidar::WorldSensorConfiguration config;
        config.source_count = poses.size();
        std::size_t effective_threads = 1;
        if (!gpu_) {
            const int worker_threads = nh_.param("worker_threads", 1);
            if (worker_threads < 1)
                throw std::invalid_argument("CPU worker_threads must be positive");
            effective_threads = std::min<std::size_t>(worker_threads, poses.size());
        }
        xgc2_world_lidar::SharedSensorEquipment equipment;
        equipment.metadata = m;
        equipment.worker_threads = effective_threads;
        for (std::size_t i = 0; i < poses.size(); ++i) {
            equipment.source_indices.push_back(i);
            sensors_[i].pub = nh_.advertise<sensor_msgs::PointCloud2>(outputs[i], 10);
            if (m.pose_type == "nav_msgs/Odometry")
                sensors_[i].pose = nh_.subscribe<nav_msgs::Odometry>(
                    poses[i], 50, [this, i](const nav_msgs::Odometry::ConstPtr& p) {
                        setPose(i, p->pose.pose, p->header.stamp);
                    });
            else
                sensors_[i].pose = nh_.subscribe<geometry_msgs::PoseStamped>(
                    poses[i], 50, [this, i](const geometry_msgs::PoseStamped::ConstPtr& p) {
                        setPose(i, p->pose, p->header.stamp);
                    });
        }
        config.shared.push_back(std::move(equipment));
        xgc2_world_lidar::WorldSensorCallbacks callbacks;
        callbacks.now = [] { return ros::Time::now(); };
        callbacks.process_inputs = [this] {
            geometry_callbacks_.callAvailable(ros::WallDuration(0));
        };
        callbacks.capture = [this](xgc2_world_lidar::WorldSensorAcquisition& out) {
            std::lock_guard<std::mutex> lock(source_mutex_);
            for (std::size_t i = 0; i < sensors_.size(); ++i)
                out.sources[i] = sensors_[i].source;
            out.geometry = geometry_;
        };
        callbacks.publish = [this](std::size_t i,
                                   xgc2_world_lidar::SensorOutputKind,
                                   const sensor_msgs::PointCloud2& cloud,
                                   const xgc2_world_lidar::SensorAcquisition&) {
            sensors_[i].pub.publish(cloud);
        };
        callbacks.fatal = [this](std::exception_ptr error) {
            failed_.store(true, std::memory_order_release);
            try {
                std::rethrow_exception(std::move(error));
            } catch (const std::exception& e) {
                ROS_FATAL("%s", e.what());
            } catch (...) {
                ROS_FATAL("unknown shared sensor backend failure");
            }
            ros::shutdown();
        };
        system_ = std::make_unique<xgc2_world_lidar::WorldSensorSystem>(std::move(config),
                                                                        std::move(callbacks));
        input_ = geometry_nh_.subscribe<sensor_msgs::PointCloud2>(
            m.input_cloud_topic, 1, [this](const sensor_msgs::PointCloud2::ConstPtr& msg) {
                {
                    std::lock_guard<std::mutex> lock(source_mutex_);
                    if (loaded_)
                        return;
                }
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(*msg, *cloud);
                if (gpu_ && msg->header.frame_id != metadata_.frame_id)
                    throw std::invalid_argument(
                        "GPU cloud must already use the declared world frame");
                auto geometry = std::make_shared<xgc2_world_lidar::WorldSensorGeometry>();
                geometry->version = 1;
                geometry->shared.push_back(cloud);
                {
                    std::lock_guard<std::mutex> lock(source_mutex_);
                    geometry_ = std::move(geometry);
                    loaded_ = true;
                }
                input_
                    .shutdown(); // exact original first-cloud lifecycle, including empty CPU input
                system_->notify();
            });
        system_->start(); // one sensor caller replaces this entry's old scan/timer owner
    }
    ~SharedCloudNode() {
        system_->fence();
        input_.shutdown();
        geometry_callbacks_.disable();
        for (auto& sensor : sensors_)
            sensor.pose.shutdown();
        system_->stop(); // input snapshots, publisher and geometry owners still alive
    }
    bool failed() const { return failed_.load(std::memory_order_acquire); }
    void setPose(std::size_t i, const geometry_msgs::Pose& p, const ros::Time& stamp) {
        {
            std::lock_guard<std::mutex> lock(source_mutex_);
            auto& s = sensors_[i].source;
            s.position = {p.position.x, p.position.y, p.position.z};
            s.orientation = {p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z};
            s.source_stamp = stamp;
            s.ready = true;
            ++s.source_version;
        }
        system_->notify();
    }
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "shared_cloud_cpu_sensor");
    try {
        SharedCloudNode node;
        ros::spin();
        return node.failed() ? 1 : 0;
    } catch (const std::exception& e) {
        ROS_FATAL("%s", e.what());
        return 1;
    }
    return 0;
}
