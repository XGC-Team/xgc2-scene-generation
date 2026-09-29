#ifndef XGC2_WORLD_LIDAR_OBSERVATION_CONTRACT_H
#define XGC2_WORLD_LIDAR_OBSERVATION_CONTRACT_H

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "xgc2_world_lidar/scene_conversion.h"

namespace xgc2_world_lidar {

// The node receives world-frame body poses. A frame label is not a transform.
inline bool validObservationPose(const Pose& pose, const std::string& frame,
                                 const std::string& expected_frame) {
    return frame == expected_frame && pose.position.allFinite() &&
           pose.orientation.coeffs().allFinite() &&
           std::isfinite(pose.orientation.squaredNorm()) &&
           pose.orientation.squaredNorm() > 1e-12;
}

// Zero is a legitimate simulation time, not permission to stamp stale data now.
inline bool freshObservationTime(double stamp, double now, double timeout) {
    return std::isfinite(stamp) && std::isfinite(now) && std::isfinite(timeout) &&
           stamp >= 0.0 && now >= stamp && timeout > 0.0 && now - stamp <= timeout;
}

inline Pose mountingPose(const std::vector<double>& xyz_rpy) {
    if (xyz_rpy.size() != 6)
        throw std::invalid_argument("sensor_pose needs x y z roll pitch yaw (metres/radians)");
    for (double value : xyz_rpy)
        if (!std::isfinite(value)) throw std::invalid_argument("sensor_pose must be finite");
    Pose pose;
    pose.position = Eigen::Vector3d(xyz_rpy[0], xyz_rpy[1], xyz_rpy[2]);
    pose.orientation = Eigen::AngleAxisd(xyz_rpy[5], Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(xyz_rpy[4], Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(xyz_rpy[3], Eigen::Vector3d::UnitX());
    return pose;
}

inline Pose sensorWorldPose(const Pose& body, const Pose& mount) {
    if (!validObservationPose(body, "world", "world") ||
        !validObservationPose(mount, "world", "world"))
        throw std::invalid_argument("invalid body or sensor mounting pose");
    const Eigen::Quaterniond rotation = body.orientation.normalized();
    Pose result;
    result.position = body.position + rotation * mount.position;
    result.orientation = rotation * mount.orientation.normalized();
    return result;
}

}  // namespace xgc2_world_lidar
#endif
