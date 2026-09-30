// Runs the plain-assert core suites under catkin's test runner.
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "xgc2_world_lidar/convex_body_conversion.h"

#define main world_lidar_tests_main
#include "test_world_lidar.cpp"
#undef main

#undef CHECK
namespace scene_sources {
#define main scene_sources_tests_main
#include "test_scene_sources.cpp"
#undef main
} // namespace scene_sources

TEST(WorldLidarCore, EveryCheckHolds) {
    EXPECT_EQ(world_lidar_tests_main(), 0);
}
TEST(SceneSources, EveryCheckHolds) {
    EXPECT_EQ(scene_sources::scene_sources_tests_main(), 0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
