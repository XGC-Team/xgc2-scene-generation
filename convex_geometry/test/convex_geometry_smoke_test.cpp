#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "convex_geometry/collision/distance_gjk_query.h"
#include "convex_geometry/occupied_sets/convex_body_factory.h"

TEST(ConvexGeometrySmoke, FactoryBodyCanBeUsedInCollisionQuery) {
    const convex_geometry::ConvexBody a =
        convex_geometry::ConvexBodyFactory::makeSphereBody(Eigen::Vector3d::Zero(), 0.5);
    const convex_geometry::ConvexBody b =
        convex_geometry::ConvexBodyFactory::makeSphereBody(Eigen::Vector3d(2.0, 0.0, 0.0), 0.5);

    ASSERT_NE(a.shape, nullptr);
    ASSERT_NE(b.shape, nullptr);
    const convex_geometry::gjk::Result result =
        convex_geometry::gjk::DistanceGjkQuery::query(*a.shape, *b.shape, nullptr, 0.1, 32, 1e-9);

    EXPECT_EQ(result.status, convex_geometry::gjk::Result::Status::kSuccess);
    EXPECT_TRUE(result.separator.normal.allFinite());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
