#include "xgc2_world_lidar/scene_conversion.h"
#include "xgc2_world_lidar/world_lidar.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace xgc2_world_lidar;
using xgc2_math::geometry::Geometry;
using xgc2_math::geometry::GeometryPayload;

using xgc2_math::geometry::encodeGeometry;

namespace {
void check(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}
ScenePartDescription part(const Geometry& geometry) {
    ScenePartDescription result;
    result.geometry = xgc2_world_lidar::prepareGeometry(encodeGeometry(geometry));
    return result;
}
std::vector<Obstacle> convert(const Geometry& geometry) {
    SceneObstacleDescription item;
    item.id = "input";
    item.parts.push_back(part(geometry));
    return toObstacles({item});
}
Geometry disconnectedMesh() {
    Geometry result;
    result.type = xgc2_math::geometry::GeometryType::Mesh;
    const std::array<std::array<std::uint32_t, 3>, 12> faces{{{0, 2, 1},
                                                              {0, 3, 2},
                                                              {4, 5, 6},
                                                              {4, 6, 7},
                                                              {0, 1, 5},
                                                              {0, 5, 4},
                                                              {1, 2, 6},
                                                              {1, 6, 5},
                                                              {2, 3, 7},
                                                              {2, 7, 6},
                                                              {3, 0, 4},
                                                              {3, 4, 7}}};
    for (double x : {-2., 2.}) {
        const auto start = static_cast<std::uint32_t>(result.vertices.size());
        for (const auto& v : std::vector<std::array<double, 3>>{{-.5, -.5, -.5},
                                                                {.5, -.5, -.5},
                                                                {.5, .5, -.5},
                                                                {-.5, .5, -.5},
                                                                {-.5, -.5, .5},
                                                                {.5, -.5, .5},
                                                                {.5, .5, .5},
                                                                {-.5, .5, .5}})
            result.vertices.push_back({x + v[0], v[1], v[2]});
        for (const auto& t : faces)
            result.triangles.push_back({start + t[0], start + t[1], start + t[2]});
    }
    return result;
}
} // namespace

int main() {
    try {
        SceneObstacleDescription item;
        item.id = "composed";
        item.pose.position = {2, 3, 4};
        item.pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(.6, Eigen::Vector3d::UnitY()));
        auto ring = part(Geometry::ring({3, 1}, .8));
        ring.pose.position = {.1, .2, .3};
        ring.pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitX()));
        item.parts.push_back(ring);
        const auto shapes = toObstacles({item});
        check(shapes.size() == 1 && shapes[0].type == Obstacle::kRing, "ring must remain analytic");
        check(shapes[0].radii == Eigen::Vector2d(3, 1) && shapes[0].tubeRadius == .8,
              "original ring parameters");
        const Eigen::Vector3d center =
            item.pose.position + item.pose.orientation * ring.pose.position;
        check((shapes[0].position - center).norm() < 1e-12, "parent/part translation");
        check(shapes[0].orientation.angularDistance(item.pose.orientation * ring.pose.orientation) <
                  1e-12,
              "parent/part rotation");
        WorldLidar sensor(SensorConfig{});
        sensor.setScene(shapes);
        check(std::isinf(
                  sensor.castRay(center, shapes[0].orientation * Eigen::Vector3d::UnitZ(), 10)),
              "ring hole remains free");
        sensor.setScene(convert(Geometry::ellipsoid({1, 2, 3})));
        check(std::abs(sensor.castRay({-4, 0, 0}, Eigen::Vector3d::UnitX(), 10) - 3) < 1e-12,
              "analytic ellipsoid ray");
        sensor.setScene(convert(Geometry::halfspaces({{1, 0, 0, -1},
                                                      {-1, 0, 0, -1},
                                                      {0, 1, 0, -1},
                                                      {0, -1, 0, -1},
                                                      {0, 0, 1, -1},
                                                      {0, 0, -1, -1}})));
        check(std::abs(sensor.castRay({-4, 0, 0}, Eigen::Vector3d::UnitX(), 10) - 3) < 1e-10,
              "finite H solid");
        sensor.setScene(convert(Geometry::convex({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}})));
        check(std::isfinite(sensor.castRay({-1, .1, .1}, Eigen::Vector3d::UnitX(), 10)),
              "runtime hull without template or triangle topology");
        sensor.setScene(convert(disconnectedMesh()));
        check(std::isinf(sensor.castRay({0, 0, -3}, Eigen::Vector3d::UnitZ(), 10)),
              "mesh gap must not become whole-body hull");
        check(std::abs(sensor.castRay({-2, 0, -3}, Eigen::Vector3d::UnitZ(), 10) - 2.5) < 1e-12,
              "mesh surface ray");
        const auto meshPayload = encodeGeometry(disconnectedMesh());
        const auto savedBytes = meshPayload.parameters;
        const auto sharedMesh = prepareGeometry(meshPayload);
        std::vector<SceneObstacleDescription> instances(100);
        for (std::size_t i = 0; i < instances.size(); ++i) {
            instances[i].id = "mesh-" + std::to_string(i);
            ScenePartDescription binding;
            binding.geometry = sharedMesh;
            instances[i].parts.push_back(binding);
            instances[i].pose.position.x() = static_cast<double>(i);
        }
        const auto initial = toObstacles(instances);
        for (auto& instance : instances)
            instance.pose.position.y() += 2;
        const auto moved = toObstacles(instances);
        for (std::size_t i = 0; i < instances.size(); ++i) {
            check(initial[i].meshData == sharedMesh->meshData &&
                      moved[i].meshData == sharedMesh->meshData,
                  "instances and state updates share one immutable prepared mesh");
            check(moved[i].position.y() == initial[i].position.y() + 2,
                  "pose updates still apply to shared geometry");
        }
        check(meshPayload.parameters == savedBytes, "source payload is not modified");
        item.parts[0].geometry.reset();
        bool rejected = false;
        try {
            toObstacles({item});
        } catch (const std::invalid_argument& error) {
            rejected = std::string(error.what()).find("composed") != std::string::npos;
        }
        check(rejected, "malformed payload is rejected with caller ID context");
        std::cout << "PASS: shared wire geometry, full poses, ring hole, ellipsoid, H, runtime \
hull, nonconvex mesh gap\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
