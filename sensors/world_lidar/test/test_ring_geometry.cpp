#include "xgc2_world_lidar/scene_conversion.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace xgc2_world_lidar;
void near(double a, double b) {
    if (!std::isfinite(a) || std::abs(a - b) > 1e-10)
        throw std::runtime_error("ring query mismatch");
}
int main() try {
    SceneObstacleDescription o;
    o.id = "ring";
    o.pose.position = {3, 4, 5};
    o.pose.orientation =
        Eigen::Quaterniond(Eigen::AngleAxisd(.7, Eigen::Vector3d(1, 2, 3).normalized()));
    ScenePartDescription p;
    p.geometry =
        xgc2_world_lidar::prepareGeometry(xgc2_math::geometry::encodeGeometry(xgc2_math::geometry::Geometry::ring({1, 1.2}, .05)));
    p.pose.position = {.1, .2, .3};
    p.pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitY()));
    o.parts.push_back(p);
    auto shapes = toObstacles({o});
    if (shapes.size() != 1 || shapes[0].radii != Eigen::Vector2d(1, 1.2) ||
        shapes[0].tubeRadius != .05)
        throw std::runtime_error("ring fields lost");
    SensorConfig c;
    WorldLidar lidar(c);
    lidar.setScene(shapes);
    const Eigen::Vector3d center = o.pose.position + o.pose.orientation * p.pose.position;
    const Eigen::Quaterniond q = o.pose.orientation * p.pose.orientation;
    near(lidar.castRay(center, q * Eigen::Vector3d::UnitX(), 10), .95);
    if (!std::isinf(lidar.castRay(center, q * Eigen::Vector3d::UnitZ(), 10)))
        throw std::runtime_error("hole filled");
    for (int field = 0; field < 3; ++field) {
        auto changed = shapes;
        if (field < 2)
            changed[0].radii[field] += .1;
        else
            changed[0].tubeRadius += .01;
        auto initial = std::make_shared<LidarScene>(shapes, .2, true);
        auto reuse = std::make_shared<LidarScene>(changed, .2, true, *initial);
        auto fresh = std::make_shared<LidarScene>(changed, .2, true);
        WorldLidar a(c), b(c);
        a.setScene(reuse);
        b.setScene(fresh);
        near(a.castRay(center, q * Eigen::Vector3d::UnitX(), 10),
             b.castRay(center, q * Eigen::Vector3d::UnitX(), 10));
        const auto x = a.globalMap(.2), y = b.globalMap(.2);
        if (x.size() != y.size())
            throw std::runtime_error("cache samples differ");
        for (size_t i = 0; i < x.size(); ++i)
            if (x[i] != y[i])
                throw std::runtime_error("cache parameter discarded");
    }
    std::cout
        << "PASS original world_lidar named mapping/composed poses/hole/query/all3 cache params\n";
    return 0;
} catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
}
