#!/usr/bin/env python3
"""Temporary integration edit, limited to this branch; removed before final PR."""
from pathlib import Path

p = Path('xgc2_world_lidar/src/world_lidar_node.cpp')
s = p.read_text()
changes = [
('#include "xgc2_world_lidar/scene_conversion.h"', '#include "xgc2_world_lidar/scene_conversion.h"\n#include "xgc2_world_lidar/observation_contract.h"'),
('        if (!(rate > 0.0)) throw std::invalid_argument("~rate must be > 0");', '''        if (!std::isfinite(rate) || !(rate > 0.0))
            throw std::invalid_argument("~rate must be finite and > 0");
        if (frame_id_ != "world" || !std::isfinite(pose_timeout_) || pose_timeout_ <= 0.0)
            throw std::invalid_argument("world lidar needs frame_id=world and a positive finite pose_timeout");
        std::vector<double> mount(6, 0.0);
        if (pnh.hasParam("sensor_pose") && !pnh.getParam("sensor_pose", mount))
            throw std::invalid_argument("~sensor_pose must be a six-number array");
        sensor_mount_ = mountingPose(mount);'''),
('                [this, k](const geometry_msgs::PoseStamped::ConstPtr& msg) { vehicles_[k].pose = msg; });', '''                [this, k](const geometry_msgs::PoseStamped::ConstPtr& msg) {
                    if (!validObservationPose(poseOf(msg->pose), msg->header.frame_id, frame_id_)) {
                        vehicles_[k].pose.reset();
                        ROS_WARN_THROTTLE(1.0, "world_lidar: invalid or non-world body pose; no scan");
                        return;
                    }
                    vehicles_[k].pose = msg;
                });'''),
('''        if (snapshot_ && msg->epoch == snapshot_->epoch && msg->revision < snapshot_->revision) return;
        snapshot_ = msg;
        state_.reset();''', '''        if (snapshot_ && msg->epoch == snapshot_->epoch && msg->revision <= snapshot_->revision) return;
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
        snapshot_ = msg;'''),
('''        state_ = msg;
        pending_ = true;''', '''        if (msg->header.frame_id != frame_id_) {
            state_.reset();
            pending_ = false;
            scene_ready_ = false;
            return;
        }
        state_ = msg;
        pending_ = true;'''),
('        if (!enabled_ || !map_dirty_) return;', '        if (!enabled_ || !scene_ready_ || !map_dirty_ || !map_lidar_ || !map_pub_.getNumSubscribers()) return;'),
('''        const ros::Time now = ros::Time::now();
        if (pending_)''', '''        const ros::Time now = ros::Time::now();
        if (!enabled_) return;
        const bool subscribed = std::any_of(vehicles_.begin(), vehicles_.end(), [this](const Vehicle& v) {
            return v.sensed && (v.pub.getNumSubscribers() ||
                               (publish_beams_ && v.beams_pub.getNumSubscribers()));
        });
        if (!subscribed && !(map_lidar_ && map_pub_.getNumSubscribers())) return;
        if (pending_)'''),
('''        publishMap(now);
        std::vector<std::size_t> fresh;''', '''        if (map_has_dynamic_ && !gazebo_source_ &&
            (!state_ || !freshObservationTime(state_->header.stamp.toSec(), now.toSec(), pose_timeout_))) {
            ROS_WARN_THROTTLE(1.0, "world_lidar: dynamic scene state is stale or from a future clock epoch");
            return;
        }
        publishMap(now);
        std::vector<std::size_t> fresh;'''),
('            if (!pose->header.stamp.isZero() && (now - pose->header.stamp).toSec() > pose_timeout_) {', '            if (!freshObservationTime(pose->header.stamp.toSec(), now.toSec(), pose_timeout_)) {'),
('            if (vehicles_[k].sensed) active.push_back(k);', '''            if (vehicles_[k].sensed && (vehicles_[k].pub.getNumSubscribers() ||
                (publish_beams_ && vehicles_[k].beams_pub.getNumSubscribers()))) active.push_back(k);'''),
('            const Pose p = poseOf(v.pose->pose);', '            const Pose p = sensorWorldPose(poseOf(v.pose->pose), sensor_mount_);'),
('            const ros::Time stamp = v.pose->header.stamp.isZero() ? now : v.pose->header.stamp;', '            const ros::Time stamp = v.pose->header.stamp;'),
('    std::string frame_id_;\n    bool yaw_only_', '    std::string frame_id_;\n    Pose sensor_mount_;\n    bool yaw_only_'),
]
for old, new in changes:
    if s.count(old) != 1:
        raise SystemExit('source changed; refusing an ambiguous replacement: ' + old[:100])
    s = s.replace(old, new)
p.write_text(s)
