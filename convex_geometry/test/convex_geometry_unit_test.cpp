#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include <Eigen/Dense>

#include "convex_geometry/collision/distance_gjk_query.h"
#include "convex_geometry/collision/gjk_warm_start_cache.h"
#include "convex_geometry/collision/guided_axis_gjk_query.h"
#include "convex_geometry/collision/separating_axis_gjk_query.h"
#include "convex_geometry/occupied_sets/ball_inflated_set.h"
#include "convex_geometry/occupied_sets/box_set.h"
#include "convex_geometry/occupied_sets/capsule_set.h"
#include "convex_geometry/occupied_sets/cylinder_set.h"
#include "convex_geometry/occupied_sets/ellipsoid_set.h"
#include "convex_geometry/occupied_sets/point_set.h"
#include "convex_geometry/occupied_sets/sphere_set.h"
#include "convex_geometry/occupied_sets/swept_hull_set.h"
#include "convex_geometry/occupied_sets/translated_set_view.h"

namespace {

bool isFiniteSeparator(const convex_geometry::gjk::SeparationQuadruple& separator) {
    return std::isfinite(separator.margin) &&
           separator.normal.allFinite() &&
           separator.point_a.allFinite() &&
           separator.point_b.allFinite();
}

void expectUsableSeparator(const convex_geometry::gjk::Result& result) {
    EXPECT_TRUE(isFiniteSeparator(result.separator));
    EXPECT_GT(result.separator.normal.squaredNorm(), 1e-12);
}

void expectVerifiedSeparator(const convex_geometry::gjk::Result& result,
                             double minimum_margin) {
    expectUsableSeparator(result);
    EXPECT_GE(result.separator.margin, minimum_margin);
}

Eigen::Quaterniond yawQuaternion(double yaw) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

TEST(ConvexGeometryUnit, SphereSupportUsesRadiusAlongDirection) {
    const convex_geometry::SphereSet sphere(Eigen::Vector3d(1.0, 2.0, 3.0), 2.0);

    const convex_geometry::SupportQueryResult support =
        sphere.support(Eigen::Vector3d::UnitX());
    EXPECT_TRUE(support.support_point.isApprox(Eigen::Vector3d(3.0, 2.0, 3.0), 1e-12));
    EXPECT_DOUBLE_EQ(support.support_value, 3.0);
}

TEST(ConvexGeometryUnit, BoxSupportSelectsSignedExtents) {
    const convex_geometry::BoxSet box(
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d(2.0, 4.0, 6.0),
        Eigen::Quaterniond::Identity());

    const convex_geometry::SupportQueryResult support =
        box.support(Eigen::Vector3d(-1.0, 1.0, -1.0));
    EXPECT_TRUE(support.support_point.isApprox(Eigen::Vector3d(-1.0, 2.0, -3.0), 1e-12));
}

TEST(ConvexGeometryUnit, PrimitiveSupportFunctionsRemainFinite) {
    const Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();
    const convex_geometry::CylinderSet cylinder(Eigen::Vector3d::Zero(), 1.0, 2.0, identity);
    const convex_geometry::CapsuleSet capsule(Eigen::Vector3d::Zero(), 0.5, 3.0, identity);
    const convex_geometry::EllipsoidSet ellipsoid(
        Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 2.0, 3.0), identity);
    const convex_geometry::SupportPointSet point_set(
        Eigen::Vector3d(1.0, 0.0, 0.0),
        std::vector<Eigen::Vector3d>{
            Eigen::Vector3d(-1.0, 0.0, 0.0),
            Eigen::Vector3d(2.0, 0.0, 0.0)});

    const Eigen::Vector3d direction(1.0, 2.0, 3.0);
    EXPECT_TRUE(cylinder.support(direction).support_point.allFinite());
    EXPECT_TRUE(capsule.support(direction).support_point.allFinite());
    EXPECT_TRUE(ellipsoid.support(direction).support_point.allFinite());
    EXPECT_TRUE(point_set.support(direction).support_point.allFinite());
    EXPECT_TRUE(capsule.support(Eigen::Vector3d::Zero()).support_point.isZero(0.0));
}

TEST(ConvexGeometryUnit, WrapperSetsPreserveSupportSemantics) {
    auto base = std::make_shared<convex_geometry::SphereSet>(Eigen::Vector3d::Zero(), 1.0);
    const convex_geometry::BallInflatedSet inflated(base, 0.5);
    const convex_geometry::TranslatedSetView<convex_geometry::SphereSet> translated(
        *base, Eigen::Vector3d(2.0, 0.0, 0.0));
    const convex_geometry::SphereSet end(Eigen::Vector3d(3.0, 0.0, 0.0), 1.0);
    const convex_geometry::SweptHullSet<convex_geometry::SphereSet, convex_geometry::SphereSet>
        swept(*base, end);

    EXPECT_NEAR(inflated.support(Eigen::Vector3d::UnitX()).support_point.x(), 1.5, 1e-12);
    EXPECT_NEAR(translated.center().x(), 2.0, 1e-12);
    EXPECT_NEAR(translated.support(Eigen::Vector3d::UnitX()).support_point.x(), 3.0, 1e-12);
    EXPECT_NEAR(swept.center().x(), 1.5, 1e-12);
    EXPECT_NEAR(swept.support(Eigen::Vector3d::UnitX()).support_point.x(), 4.0, 1e-12);
}

TEST(ConvexGeometryUnit, DistanceGjkReportsSeparatedSpheres) {
    const convex_geometry::SphereSet a(Eigen::Vector3d::Zero(), 1.0);
    const convex_geometry::SphereSet b(Eigen::Vector3d(3.0, 0.0, 0.0), 1.0);

    const convex_geometry::gjk::Result result =
        convex_geometry::gjk::DistanceGjkQuery::query(a, b, nullptr, 0.1, 32, 1e-9);

    EXPECT_EQ(result.status, convex_geometry::gjk::Result::Status::kSuccess);
    EXPECT_NEAR(result.separator.margin, 1.0, 1e-9);
    EXPECT_TRUE(result.separator.normal.isApprox(Eigen::Vector3d::UnitX(), 1e-9));
}

TEST(ConvexGeometryUnit, DistanceGjkFlagsDistanceBelowMinimum) {
    const convex_geometry::SphereSet a(Eigen::Vector3d::Zero(), 1.0);
    const convex_geometry::SphereSet b(Eigen::Vector3d(2.5, 0.0, 0.0), 1.0);

    const convex_geometry::gjk::Result result =
        convex_geometry::gjk::DistanceGjkQuery::query(a, b, nullptr, 0.75, 32, 1e-9);

    EXPECT_EQ(result.status, convex_geometry::gjk::Result::Status::kOverlap);
    EXPECT_NEAR(result.separator.margin, 0.5, 1e-9);
}

TEST(ConvexGeometryUnit, DistanceGjkFlagsTouchingSetsAsBelowMinimum) {
    const convex_geometry::SphereSet a(Eigen::Vector3d::Zero(), 1.0);
    const convex_geometry::SphereSet b(Eigen::Vector3d(2.0, 0.0, 0.0), 1.0);

    const convex_geometry::gjk::Result result =
        convex_geometry::gjk::DistanceGjkQuery::query(a, b, nullptr, 0.01, 32, 1e-9);

    EXPECT_EQ(result.status, convex_geometry::gjk::Result::Status::kOverlap);
    EXPECT_NEAR(result.separator.margin, 0.0, 1e-12);
    expectUsableSeparator(result);
}

TEST(ConvexGeometryUnit, SeparatingAxisGjkReportsUsableSeparator) {
    const convex_geometry::SphereSet a(Eigen::Vector3d::Zero(), 1.0);
    const convex_geometry::SphereSet b(Eigen::Vector3d(3.0, 0.0, 0.0), 1.0);

    const convex_geometry::gjk::Result result =
        convex_geometry::gjk::SeparatingAxisGjkQuery::query(a, b, nullptr, 0.5, 32, 1e-9);

    EXPECT_EQ(result.status, convex_geometry::gjk::Result::Status::kSuccess);
    EXPECT_GT(result.separator.margin, 0.5);
    EXPECT_TRUE(result.separator.normal.allFinite());
    EXPECT_TRUE(result.separator.point_a.allFinite());
    EXPECT_TRUE(result.separator.point_b.allFinite());
}

TEST(ConvexGeometryUnit, SeparatingAxisGjkTreatsMarginBelowMinimumAsOverlap) {
    const convex_geometry::SphereSet a(Eigen::Vector3d::Zero(), 1.0);
    const convex_geometry::SphereSet b(Eigen::Vector3d(2.4, 0.0, 0.0), 1.0);

    const convex_geometry::gjk::Result result =
        convex_geometry::gjk::SeparatingAxisGjkQuery::query(a, b, nullptr, 0.5, 32, 1e-9);

    EXPECT_EQ(result.status, convex_geometry::gjk::Result::Status::kOverlap);
    EXPECT_LT(result.separator.margin, 0.5);
    expectUsableSeparator(result);
}

TEST(ConvexGeometryUnit, GuidedAxisGjkRunsCorrectionAndPreservesValidSeparator) {
    const convex_geometry::SphereSet a(Eigen::Vector3d::Zero(), 1.0);
    const convex_geometry::SphereSet b(Eigen::Vector3d(3.0, 1.0, 0.0), 1.0);

    const convex_geometry::gjk::Result result =
        convex_geometry::gjk::GuidedAxisGjkQuery::query(a, b, nullptr, 0.1, 0.1, 32, 1e-9);

    EXPECT_EQ(result.status, convex_geometry::gjk::Result::Status::kSuccess);
    expectVerifiedSeparator(result, 0.1);
}

TEST(ConvexGeometryUnit, GjkWarmStartCanSeedSubsequentQuery) {
    const convex_geometry::SphereSet a(Eigen::Vector3d::Zero(), 1.0);
    const convex_geometry::SphereSet b(Eigen::Vector3d(3.0, 0.0, 0.0), 1.0);

    const convex_geometry::gjk::Result first =
        convex_geometry::gjk::DistanceGjkQuery::query(a, b, nullptr, 0.1, 32, 1e-9);
    ASSERT_EQ(first.status, convex_geometry::gjk::Result::Status::kSuccess);

    convex_geometry::gjk::WarmStart warm_start;
    warm_start.separator = first.separator;
    warm_start.valid = true;
    const convex_geometry::SphereSet moved_b(Eigen::Vector3d(3.1, 0.0, 0.0), 1.0);
    const convex_geometry::gjk::Result second =
        convex_geometry::gjk::DistanceGjkQuery::query(a, moved_b, &warm_start, 0.1, 32, 1e-9);

    EXPECT_EQ(second.status, convex_geometry::gjk::Result::Status::kSuccess);
    EXPECT_NEAR(second.separator.margin, 1.1, 1e-9);
    expectUsableSeparator(second);
}

TEST(ConvexGeometryUnit, StaleWarmStartDoesNotCorruptSuccessfulQuery) {
    const convex_geometry::SphereSet a(Eigen::Vector3d::Zero(), 1.0);
    const convex_geometry::SphereSet b(Eigen::Vector3d(4.0, 0.0, 0.0), 1.0);

    convex_geometry::gjk::WarmStart stale;
    stale.valid = true;
    stale.separator.normal = Eigen::Vector3d::UnitY();
    stale.separator.margin = 100.0;
    stale.separator.point_a = Eigen::Vector3d(0.0, 100.0, 0.0);
    stale.separator.point_b = Eigen::Vector3d(0.0, 200.0, 0.0);

    const convex_geometry::gjk::Result result =
        convex_geometry::gjk::DistanceGjkQuery::query(a, b, &stale, 0.1, 32, 1e-9);

    EXPECT_EQ(result.status, convex_geometry::gjk::Result::Status::kSuccess);
    EXPECT_NEAR(result.separator.margin, 2.0, 1e-9);
    EXPECT_TRUE(result.separator.normal.isApprox(Eigen::Vector3d::UnitX(), 1e-9));
}

TEST(ConvexGeometryUnit, WarmStartCacheSizesFollowConfiguredLayout) {
    convex_geometry::GjkWarmStartCache cache;
    cache.initialize(5, 3, 2);

    EXPECT_EQ(cache.expectedStaticObstacles(), 3);
    EXPECT_EQ(cache.expectedNeighbors(), 2);
    EXPECT_EQ(cache.stageStaticWarmStarts().size(), 18u);
    EXPECT_EQ(cache.terminalStaticGammaWarmStarts().size(), 3u);
    EXPECT_EQ(cache.stageNeighborWarmStarts().size(), 12u);
    EXPECT_EQ(cache.terminalNeighborGammaWarmStarts().size(), 2u);
}

TEST(ConvexGeometryUnit, GjkHandlesMixedPrimitivePairs) {
    const Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();
    const convex_geometry::SphereSet sphere(Eigen::Vector3d::Zero(), 0.5);
    const convex_geometry::BoxSet box(
        Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Vector3d(0.6, 0.8, 1.0), identity);
    const convex_geometry::CylinderSet cylinder(Eigen::Vector3d(0.0, 2.0, 0.0), 0.3, 1.0, identity);
    const convex_geometry::CapsuleSet capsule(Eigen::Vector3d(2.0, 2.0, 0.0), 0.25, 1.2, identity);
    const convex_geometry::EllipsoidSet ellipsoid(
        Eigen::Vector3d(0.0, -2.0, 0.0), Eigen::Vector3d(0.3, 0.4, 0.5), identity);
    const convex_geometry::SupportPointSet polytope(
        Eigen::Vector3d(2.0, -2.0, 0.0),
        std::vector<Eigen::Vector3d>{
            Eigen::Vector3d(-0.2, -0.2, -0.2),
            Eigen::Vector3d(0.2, -0.2, -0.2),
            Eigen::Vector3d(-0.2, 0.2, -0.2),
            Eigen::Vector3d(0.0, 0.0, 0.3)});

    const std::vector<const convex_geometry::ConvexSet3D*> sets = {
        &sphere, &box, &cylinder, &capsule, &ellipsoid, &polytope};

    for (std::size_t i = 0; i + 1 < sets.size(); ++i) {
        const convex_geometry::gjk::Result result =
            convex_geometry::gjk::DistanceGjkQuery::query(*sets[i], *sets[i + 1], nullptr, 0.05, 64, 1e-9);
        EXPECT_EQ(result.status, convex_geometry::gjk::Result::Status::kSuccess) << i;
        expectUsableSeparator(result);
    }
}

TEST(ConvexGeometryUnit, DeterministicSphereStressMatchesAnalyticDistance) {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> radius_dist(0.05, 0.35);
    std::uniform_real_distribution<double> gap_dist(0.05, 2.0);
    std::uniform_real_distribution<double> direction_dist(-1.0, 1.0);

    for (int sample = 0; sample < 200; ++sample) {
        Eigen::Vector3d direction(
            direction_dist(rng), direction_dist(rng), direction_dist(rng));
        if (direction.squaredNorm() < 1e-12) {
            direction = Eigen::Vector3d::UnitX();
        }
        direction.normalize();

        const double radius_a = radius_dist(rng);
        const double radius_b = radius_dist(rng);
        const double gap = gap_dist(rng);
        const Eigen::Vector3d center_b = direction * (radius_a + radius_b + gap);
        const convex_geometry::SphereSet a(Eigen::Vector3d::Zero(), radius_a);
        const convex_geometry::SphereSet b(center_b, radius_b);

        const convex_geometry::gjk::Result distance_result =
            convex_geometry::gjk::DistanceGjkQuery::query(a, b, nullptr, 0.01, 64, 1e-10);
        EXPECT_EQ(distance_result.status, convex_geometry::gjk::Result::Status::kSuccess) << sample;
        EXPECT_NEAR(distance_result.separator.margin, gap, 1e-8) << sample;
        expectUsableSeparator(distance_result);

        const convex_geometry::gjk::Result axis_result =
            convex_geometry::gjk::SeparatingAxisGjkQuery::query(a, b, nullptr, 0.01, 64, 1e-10);
        EXPECT_EQ(axis_result.status, convex_geometry::gjk::Result::Status::kSuccess) << sample;
        EXPECT_NEAR(axis_result.separator.margin, gap, 1e-8) << sample;
        expectUsableSeparator(axis_result);
    }
}

TEST(ConvexGeometryUnit, SphereBoundaryStressCoversTinyGapAndTouchingCases) {
    const std::vector<double> gaps = {0.0, 1e-6, 1e-4, 0.01, 0.1};
    const Eigen::Vector3d direction = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();

    for (double gap : gaps) {
        const convex_geometry::SphereSet a(Eigen::Vector3d::Zero(), 0.4);
        const convex_geometry::SphereSet b(direction * (0.7 + gap), 0.3);
        const double minimum_distance = 1e-5;

        const convex_geometry::gjk::Result result =
            convex_geometry::gjk::DistanceGjkQuery::query(a, b, nullptr, minimum_distance, 64, 1e-12);

        if (gap < minimum_distance) {
            EXPECT_EQ(result.status, convex_geometry::gjk::Result::Status::kOverlap) << gap;
        } else {
            EXPECT_EQ(result.status, convex_geometry::gjk::Result::Status::kSuccess) << gap;
        }
        EXPECT_NEAR(result.separator.margin, gap, 1e-8) << gap;
        expectUsableSeparator(result);
    }
}

TEST(ConvexGeometryUnit, DeterministicAxisAlignedBoxStressRemainsSeparated) {
    std::mt19937 rng(13);
    std::uniform_real_distribution<double> size_dist(0.2, 1.0);
    std::uniform_real_distribution<double> gap_dist(0.05, 1.5);

    for (int sample = 0; sample < 100; ++sample) {
        const Eigen::Vector3d size_a(size_dist(rng), size_dist(rng), size_dist(rng));
        const Eigen::Vector3d size_b(size_dist(rng), size_dist(rng), size_dist(rng));
        const double gap = gap_dist(rng);
        const double center_distance = 0.5 * size_a.x() + 0.5 * size_b.x() + gap;

        const convex_geometry::BoxSet a(
            Eigen::Vector3d::Zero(), size_a, Eigen::Quaterniond::Identity());
        const convex_geometry::BoxSet b(
            Eigen::Vector3d(center_distance, 0.0, 0.0),
            size_b,
            Eigen::Quaterniond::Identity());

        const convex_geometry::gjk::Result distance_result =
            convex_geometry::gjk::DistanceGjkQuery::query(a, b, nullptr, 0.01, 64, 1e-10);
        EXPECT_EQ(distance_result.status, convex_geometry::gjk::Result::Status::kSuccess) << sample;
        EXPECT_NEAR(distance_result.separator.margin, gap, 1e-8) << sample;
        expectUsableSeparator(distance_result);

        const convex_geometry::gjk::Result guided_result =
            convex_geometry::gjk::GuidedAxisGjkQuery::query(a, b, nullptr, 0.01, 0.01, 64, 1e-10);
        EXPECT_EQ(guided_result.status, convex_geometry::gjk::Result::Status::kSuccess) << sample;
        EXPECT_NEAR(guided_result.separator.margin, gap, 1e-8) << sample;
        expectVerifiedSeparator(guided_result, 0.01);
    }
}

TEST(ConvexGeometryUnit, RotatedBoxStressMatchesAnalyticGapAlongSharedAxis) {
    const Eigen::Quaterniond orientation = yawQuaternion(0.37);
    const Eigen::Vector3d axis = orientation * Eigen::Vector3d::UnitX();
    std::mt19937 rng(29);
    std::uniform_real_distribution<double> size_dist(0.2, 0.9);
    std::uniform_real_distribution<double> gap_dist(1e-4, 1.0);

    for (int sample = 0; sample < 80; ++sample) {
        const Eigen::Vector3d size_a(size_dist(rng), size_dist(rng), size_dist(rng));
        const Eigen::Vector3d size_b(size_dist(rng), size_dist(rng), size_dist(rng));
        const double gap = gap_dist(rng);
        const double center_distance = 0.5 * size_a.x() + 0.5 * size_b.x() + gap;

        const convex_geometry::BoxSet a(Eigen::Vector3d::Zero(), size_a, orientation);
        const convex_geometry::BoxSet b(axis * center_distance, size_b, orientation);

        const convex_geometry::gjk::Result distance_result =
            convex_geometry::gjk::DistanceGjkQuery::query(a, b, nullptr, 1e-5, 64, 1e-10);
        EXPECT_EQ(distance_result.status, convex_geometry::gjk::Result::Status::kSuccess) << sample;
        EXPECT_NEAR(distance_result.separator.margin, gap, 1e-8) << sample;
        expectUsableSeparator(distance_result);

        const convex_geometry::gjk::Result axis_result =
            convex_geometry::gjk::SeparatingAxisGjkQuery::query(a, b, nullptr, 1e-5, 64, 1e-10);
        EXPECT_EQ(axis_result.status, convex_geometry::gjk::Result::Status::kSuccess) << sample;
        EXPECT_NEAR(axis_result.separator.margin, gap, 1e-8) << sample;
        expectUsableSeparator(axis_result);
    }
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
