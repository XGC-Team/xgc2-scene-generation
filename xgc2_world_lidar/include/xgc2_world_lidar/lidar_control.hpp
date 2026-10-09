#pragma once
#include <ros/callback_queue_interface.h>
#include <functional>
#include <memory>
#include <string>

namespace xgc2_world_lidar {
// One control host for the whole source/fleet. The ROS callback queue is used
// solely as the source's event-driven thread dispatcher, never as RPC transport.
class LidarControl {
 public:
  LidarControl(std::string socket, std::string target, bool initial_enabled,
               ros::CallbackQueueInterface* queue, std::function<void(bool)> apply);
  ~LidarControl();
  void Start();
  void Stop();
 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
