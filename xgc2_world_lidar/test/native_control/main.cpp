#include "xgc2_world_lidar/lidar_control.hpp"
#include <ros/callback_queue.h>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  ros::CallbackQueue queue;
  bool enabled = true;
  unsigned calls = 0;
  xgc2_world_lidar::LidarControl control(argv[1], "fixture", enabled, &queue,
      [&](bool value) { enabled = value; ++calls; });
  control.Start();
  std::cout << "ready" << std::endl;
  std::string command;
  while (std::getline(std::cin, command)) {
    if (command == "dispatch") {
      queue.callAvailable(ros::WallDuration(0));
      std::cout << (enabled ? "true" : "false") << " " << calls << std::endl;
    } else if (command == "stop") break;
    else return 3;
  }
  control.Stop();
  return 0;
}
