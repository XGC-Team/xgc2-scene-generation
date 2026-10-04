#include "xgc2_world_lidar/scan_pool.h"
#include "xgc2_world_lidar/shared_cloud_cpu.hpp"
#ifdef XGC_WORLD_LIDAR_GPU
#include "xgc2_world_lidar/shared_cloud_gpu.hpp"
#include <cstring>
#endif
#include <algorithm>
#include <cstdint>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>
class SharedCloudNode {
    struct Sensor {
        ros::Publisher pub;
        ros::Subscriber pose;
        bool ready = false;
        Eigen::Vector3d p;
        Eigen::Quaterniond q;
        ros::Time stamp;
        xgc2_world_lidar::CropResult scratch;
        sensor_msgs::PointCloud2 output;
    };
    ros::NodeHandle nh_{"~"};
    ros::Subscriber input_;
    ros::Timer timer_;
    xgc2_world_lidar::SensorMetadata metadata_;
    xgc2_world_lidar::SharedCloudCpu world_;
#ifdef XGC_WORLD_LIDAR_GPU
    std::unique_ptr<xgc2_world_lidar::SharedCloudGpu> gpu_world_;
#endif
    std::vector<Sensor> sensors_;
    std::unique_ptr<xgc2_world_lidar::ScanPool> cpu_pool_;
    bool loaded_ = false, zero_stamp_ = false, gpu_ = false;

public:
    SharedCloudNode() {
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
            gpu_world_ = std::make_unique<xgc2_world_lidar::SharedCloudGpu>();
#else
            throw std::invalid_argument("GPU is unsupported by this CPU-only build; no fallback");
#endif
        } else {
            xgc2_world_lidar::validateSensorMetadata(m);
        }
        zero_stamp_ = m.stamp_policy == "zero";
        std::vector<std::string> poses, outputs;
        if (!nh_.getParam("pose_topics", poses) || !nh_.getParam("output_topics", outputs) ||
            poses.empty() || poses.size() != outputs.size() || m.input_cloud_topic.empty())
            throw std::runtime_error("actual cloud/pose/output bindings required");
        if (m.pose_type != "nav_msgs/Odometry" && m.pose_type != "geometry_msgs/PoseStamped")
            throw std::runtime_error("unsupported explicit pose type");
        sensors_.resize(poses.size());
        if (!gpu_) {
            // Total parallelism includes the caller: 1 is serial; 0 is invalid, not auto.
            const int worker_threads = nh_.param("worker_threads", 1);
            if (worker_threads < 1)
                throw std::invalid_argument("CPU worker_threads must be positive");
            const std::size_t threads = std::min<std::size_t>(
                static_cast<std::size_t>(worker_threads), sensors_.size());
            if (threads > 1)
                cpu_pool_ = std::make_unique<xgc2_world_lidar::ScanPool>(threads);
        }
        for (std::size_t i = 0; i < poses.size(); ++i) {
            sensors_[i].pub = nh_.advertise<sensor_msgs::PointCloud2>(outputs[i], 10);
#ifdef XGC_WORLD_LIDAR_GPU
            if (gpu_) {
                pcl::PointCloud<pcl::PointXYZI> empty;
                pcl::toROSMsg(empty, sensors_[i].output);
                if (sensors_[i].output.point_step != sizeof(pcl::PointXYZI))
                    throw std::runtime_error("actual original GPU XYZ/intensity layout required");
            } else
#endif
            {
                pcl::PointCloud<pcl::PointXYZ> empty;
                pcl::toROSMsg(empty, sensors_[i].output);
                if (sensors_[i].output.point_step != sizeof(pcl::PointXYZ))
                    throw std::runtime_error("actual PCL XYZ layout required");
            }
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
        input_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            m.input_cloud_topic, 1, [this](const sensor_msgs::PointCloud2::ConstPtr& msg) {
                if (loaded_)
                    return;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(*msg, *cloud);
#ifdef XGC_WORLD_LIDAR_GPU
                if (gpu_) {
                    if (msg->header.frame_id != metadata_.frame_id)
                        throw std::invalid_argument(
                            "GPU cloud must already use the declared world frame");
                    gpu_world_->load(cloud, metadata_);
                } else
#endif
                {
                    world_.load(cloud, metadata_);
                }
                loaded_ = true;
                input_.shutdown(); // original static first-cloud contract
            });
        timer_ = nh_.createTimer(ros::Duration(1.0 / m.publish_rate_hz),
                                 [this](const ros::TimerEvent&) { tick(); });
    }
    void setPose(std::size_t i, const geometry_msgs::Pose& p, const ros::Time& stamp) {
        auto& s = sensors_[i];
        s.p = {p.position.x, p.position.y, p.position.z};
        s.q = {p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z};
        s.stamp = stamp;
        s.ready = true;
    }
    void tick() {
        if (!loaded_)
            return;
        if (cpu_pool_) {
            // Single ROS spin keeps poses and the installed index unchanged until join.
            // One complete query per sensor; only its reusable scratch is written.
            cpu_pool_->run(sensors_.size(), [this](std::size_t i) {
                auto& s = sensors_[i];
                if (s.ready)
                    world_.scanInto(s.p, s.q, &s.scratch);
            });
        }
        for (auto& s : sensors_) {
            if (!s.ready)
                continue;
            const void* points = nullptr;
#ifdef XGC_WORLD_LIDAR_GPU
            if (gpu_) {
                const auto& scan = gpu_world_->scan(s.p, s.q, s.stamp.toSec());
                s.output.width = scan.size();
                s.output.is_dense = scan.is_dense;
                points = scan.points.data(); // one renderer buffer, serialize before the next pose
            } else
#endif
            {
                if (!cpu_pool_)
                    world_.scanInto(s.p, s.q, &s.scratch);
                if (!s.scratch.radius_candidates)
                    continue; // original CPU zero-neighbour no-publication behavior
                s.output.width = s.scratch.cloud.size();
                s.output.is_dense = true;
                points = s.scratch.cloud.points.data();
            }
            s.output.height = 1;
            s.output.row_step = s.output.width * s.output.point_step;
#ifdef XGC_WORLD_LIDAR_GPU
            if (gpu_) {
                s.output.data.resize(s.output.row_step);
                if (!s.output.data.empty())
                    std::memcpy(s.output.data.data(), points, s.output.data.size());
            } else
#endif
            {
                // Keep one owned payload copy; avoid resize's redundant tail initialization.
                if (s.output.row_step == 0) {
                    s.output.data.clear();
                } else {
                    const auto* bytes = static_cast<const std::uint8_t*>(points);
                    s.output.data.assign(bytes, bytes + s.output.row_step);
                }
            }
            s.output.header.frame_id = metadata_.frame_id;
            s.output.header.stamp = zero_stamp_ ? ros::Time(0) : s.stamp;
            s.pub.publish(s.output);
        }
    }
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "shared_cloud_cpu_sensor");
    try {
        SharedCloudNode node;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("%s", e.what());
        return 1;
    }
    return 0;
}
