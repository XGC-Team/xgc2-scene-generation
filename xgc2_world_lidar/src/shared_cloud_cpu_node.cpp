#include "xgc2_world_lidar/shared_cloud_cpu.hpp"
#ifdef XGC_WORLD_LIDAR_GPU
#include "xgc2_world_lidar/shared_cloud_gpu.hpp"
#endif
#include <algorithm>
#include <cstdint>
#include <cstring>
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
    };
    // State owned by one scan thread (index = ScanPool worker): the serialization buffer is
    // reused by every sensor that thread scans, because publish() serializes before it returns.
    struct Worker {
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
    std::vector<Worker> workers_;
    std::unique_ptr<xgc2_world_lidar::ScanPool> pool_;
    std::vector<std::size_t> active_; // sensors with a pose in this tick, in sensor order
    std::vector<xgc2_world_lidar::ScanPose> poses_;
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
            // Optional extent and near range (0 / absent: the whole radius, as before). The
            // resolution and point-cover parameters are lidar_scan's and are refused here.
            nh_.getParam("min_range_m", m.min_range_m);
            nh_.getParam("h_fov_deg", m.h_fov_deg);
            nh_.getParam("v_fov_deg", m.v_fov_deg);
            for (const char* name : {"h_res", "v_res", "point_cover_spacing_m"})
                if (nh_.hasParam(name))
                    throw std::invalid_argument(std::string(name) +
                                                " is a lidar_scan parameter; crop_through has "
                                                "no resolution");
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
        // The af994ea contract: total parallelism including the caller. 1 (default) is serial
        // in sensor order on the spin thread; n is n threads up to one per sensor; 0 and
        // negative values are invalid. The GL context belongs to the spin thread, so the GPU
        // backend always scans there and does not read worker_threads.
        std::size_t threads = 1;
        if (!gpu_) {
            const int worker_threads = nh_.param("worker_threads", 1);
            if (worker_threads < 1)
                throw std::invalid_argument("CPU worker_threads must be positive");
            threads = std::min<std::size_t>(static_cast<std::size_t>(worker_threads),
                                            std::max<std::size_t>(1, poses.size()));
        }
        pool_ = std::make_unique<xgc2_world_lidar::ScanPool>(threads);
        workers_.resize(pool_->threads());
        for (auto& worker : workers_) {
#ifdef XGC_WORLD_LIDAR_GPU
            if (gpu_) {
                pcl::PointCloud<pcl::PointXYZI> empty;
                pcl::toROSMsg(empty, worker.output);
                if (worker.output.point_step != sizeof(pcl::PointXYZI))
                    throw std::runtime_error("actual original GPU XYZ/intensity layout required");
            } else
#endif
            {
                pcl::PointCloud<pcl::PointXYZ> empty;
                pcl::toROSMsg(empty, worker.output);
                if (worker.output.point_step != sizeof(pcl::PointXYZ))
                    throw std::runtime_error("actual PCL XYZ layout required");
            }
        }
        sensors_.resize(poses.size());
        for (std::size_t i = 0; i < poses.size(); ++i) {
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
    // Fills the worker's reused message with `count` records at `points` and publishes it for
    // sensor `s`. publish() serializes before it returns, so the buffer is free again after.
    void publishCloud(const Sensor& s, Worker& worker, const void* points, std::size_t count) {
        auto& output = worker.output;
        output.width = count;
        output.is_dense = true;
        output.height = 1;
        output.row_step = output.width * output.point_step;
        // One owned payload copy into the per-thread buffer (capacity retained); assign avoids
        // resize's initialization of a grown tail that the copy then overwrites.
        const auto* bytes = static_cast<const std::uint8_t*>(points);
        output.data.assign(bytes, bytes + output.row_step);
        output.header.frame_id = metadata_.frame_id;
        output.header.stamp = zero_stamp_ ? ros::Time(0) : s.stamp;
        s.pub.publish(output);
    }
    void tick() {
        if (!loaded_)
            return;
        active_.clear();
        poses_.clear();
        for (std::size_t i = 0; i < sensors_.size(); ++i) {
            if (!sensors_[i].ready)
                continue;
            active_.push_back(i);
            poses_.push_back({sensors_[i].p, sensors_[i].q});
        }
#ifdef XGC_WORLD_LIDAR_GPU
        if (gpu_) {
            for (std::size_t k = 0; k < active_.size(); ++k) {
                const auto& s = sensors_[active_[k]];
                const auto& scan = gpu_world_->scan(s.p, s.q, s.stamp.toSec());
                auto& output = workers_[0].output;
                output.width = scan.size();
                output.is_dense = scan.is_dense;
                output.height = 1;
                output.row_step = output.width * output.point_step;
                output.data.resize(output.row_step);
                // one renderer buffer, serialize before the next pose
                if (!output.data.empty())
                    std::memcpy(output.data.data(), scan.points.data(), output.data.size());
                output.header.frame_id = metadata_.frame_id;
                output.header.stamp = zero_stamp_ ? ros::Time(0) : s.stamp;
                s.pub.publish(output);
            }
            return;
        }
#endif
        world_.scanBatch(
            *pool_,
            poses_,
            [this](std::size_t k, std::size_t worker, const xgc2_world_lidar::CropResult& scan) {
                if (!scan.radius_candidates)
                    return; // original CPU zero-neighbour no-publication behavior
                publishCloud(sensors_[active_[k]],
                             workers_[worker],
                             scan.cloud.points.data(),
                             scan.cloud.size());
            });
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
