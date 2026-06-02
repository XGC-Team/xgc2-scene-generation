/**
 * @file scenario_node.cpp
 * @brief Cluttered-environment node entrypoint
 */

#include <ros/ros.h>

#include "cluttered_environment/scenario_manager.h"

/**
 * @brief Main function
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "cluttered_environment_node");
    ros::NodeHandle nh("~");

    ROS_INFO("=================================================");
    ROS_INFO("  Cluttered Environment Generator Node");
    ROS_INFO("  杂乱环境生成器节点");
    ROS_INFO("=================================================");

    cluttered_environment::ScenarioManager scenario_mgr(nh);

    // Load configuration and instantiate obstacles
    if (!scenario_mgr.loadAndInitialize()) {
        ROS_ERROR("Failed to load and initialize scenario!");
        return -1;
    }

    // Print summary
    scenario_mgr.printSummary();

    ROS_INFO("Scenario is running. All obstacles are publishing TF and markers.");
    ROS_INFO("Press Ctrl+C to exit.\n");

    // Spin - obstacle timers will handle TF and marker publishing
    ros::spin();

    return 0;
}
