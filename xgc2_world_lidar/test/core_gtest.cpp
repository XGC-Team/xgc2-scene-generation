// Runs the plain-assert core suites under catkin's test runner.
#include <gtest/gtest.h>

#include <atomic>
#include <mutex>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <string>
#include <vector>

#include "xgc2_world_lidar/convex_body_conversion.h"
#include "xgc2_world_lidar/scan_pool.h"
#include "xgc2_world_lidar/shared_cloud_cpu.hpp"

#define main world_lidar_tests_main
#include "test_world_lidar.cpp"
#undef main

#undef CHECK
namespace scene_sources {
#define main scene_sources_tests_main
#include "test_scene_sources.cpp"
#undef main
} // namespace scene_sources

#undef CHECK
namespace shared_cloud_cpu {
#define main shared_cloud_cpu_tests_main
#include "test_shared_cloud_cpu.cpp"
#undef main
} // namespace shared_cloud_cpu

TEST(WorldLidarCore, EveryCheckHolds) {
    EXPECT_EQ(world_lidar_tests_main(), 0);
}
TEST(SceneSources, EveryCheckHolds) {
    EXPECT_EQ(scene_sources::scene_sources_tests_main(), 0);
}

TEST(SharedCloudCpu, EveryCheckHolds) {
    EXPECT_EQ(shared_cloud_cpu::shared_cloud_cpu_tests_main(), 0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
