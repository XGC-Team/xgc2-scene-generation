#include <gtest/gtest.h>

#include "cluttered_environment/scene/obstacles/obstacle_factory.h"

namespace cluttered_environment {
namespace {

ObstacleConfig makeBaseConfig() {
    ObstacleConfig config;
    config.id = 1;
    config.name = "unsupported_obstacle";
    config.type = "unsupported";
    config.motion_type = "static";
    config.frame_id = "obstacle_1";
    config.parent_frame_id = "world";
    config.position = Eigen::Vector3d::Zero();
    config.orientation = Eigen::Quaterniond::Identity();
    config.scale = Eigen::Vector3d::Ones();
    config.color = Eigen::Vector4d(1.0, 0.0, 0.0, 1.0);
    config.velocity_obstacle_enabled = false;
    config.initial_velocity = Eigen::Vector3d::Zero();
    return config;
}

TEST(ObstacleConfigTest, BuildsStaticUnsupportedConfig) {
    const ObstacleConfig config = makeBaseConfig();

    EXPECT_EQ(1, config.id);
    EXPECT_EQ("unsupported", config.type);
    EXPECT_EQ("static", config.motion_type);
    EXPECT_FALSE(config.velocity_obstacle_enabled);
    EXPECT_TRUE(config.position.isZero());
    EXPECT_TRUE(config.initial_velocity.isZero());
    EXPECT_DOUBLE_EQ(1.0, config.orientation.norm());
}

} // namespace
} // namespace cluttered_environment

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
