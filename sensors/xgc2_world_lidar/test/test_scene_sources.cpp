// Plain-assert tests of the two scene sources. Exit status 0 iff every check holds.
//
// The same toy scene is described once as a scene document (snapshot source,
// lightweight simulator) and once as the Gazebo obstacle truth that the
// xgc2_gazebo_scene plugin publishes for the materialized scene (gazebo
// source). Both must give the same solids, hence the same scans.

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include "xgc2_world_lidar/convex_body_conversion.h"
#include "xgc2_world_lidar/scene_conversion.h"
#include "xgc2_world_lidar/world_lidar.h"

using namespace xgc2_world_lidar;

namespace {

int g_failures = 0;
int g_checks = 0;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        ++g_checks;                                                                                \
        if (!(cond)) {                                                                             \
            ++g_failures;                                                                          \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                          \
        }                                                                                          \
    } while (0)

Pose pose(double x, double y, double z, double yaw = 0.0) {
    Pose p;
    p.position = Eigen::Vector3d(x, y, z);
    p.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    return p;
}

Pose compose(const Pose& a, const Pose& b) {
    Pose out;
    out.position = a.position + a.orientation * b.position;
    out.orientation = a.orientation * b.orientation;
    return out;
}

// Local-frame vertices of a triangular prism (convex part).
std::vector<Eigen::Vector3d> prism() {
    return {{-0.5, -0.4, 0.0},
            {0.5, -0.4, 0.0},
            {0.0, 0.5, 0.0},
            {-0.5, -0.4, 2.0},
            {0.5, -0.4, 2.0},
            {0.0, 0.5, 2.0}};
}

// Scene document: a box, a cylinder, a sphere, a convex prism and a two-post
// gate (one obstacle, two parts) with an open middle.
std::vector<SceneObstacleDescription> documentScene() {
    std::vector<SceneObstacleDescription> scene;
    auto part = [](const std::string& type, const Pose& p) {
        ScenePartDescription d;
        d.type = type;
        d.pose = p;
        return d;
    };
    {
        SceneObstacleDescription o{"box", pose(6.0, 0.0, 1.0, 0.3), {}};
        auto p = part("box", pose(0, 0, 0));
        p.size = Eigen::Vector3d(1.0, 2.0, 2.0);
        o.parts.push_back(p);
        scene.push_back(o);
    }
    {
        SceneObstacleDescription o{"pillar", pose(0.0, 5.0, 1.5), {}};
        auto p = part("cylinder", pose(0, 0, 0));
        p.radius = 0.4;
        p.height = 3.0;
        o.parts.push_back(p);
        scene.push_back(o);
    }
    {
        SceneObstacleDescription o{"ball", pose(-4.0, -3.0, 1.0), {}};
        auto p = part("sphere", pose(0, 0, 0));
        p.radius = 0.7;
        o.parts.push_back(p);
        scene.push_back(o);
    }
    {
        SceneObstacleDescription o{"rock", pose(-5.0, 2.0, 0.0, -0.7), {}};
        auto p = part("convex", pose(0, 0, 0));
        p.vertices = prism();
        o.parts.push_back(p);
        scene.push_back(o);
    }
    {
        SceneObstacleDescription o{"gate", pose(0.0, -6.0, 0.0, 0.2), {}};
        for (double y : {-1.0, 1.0}) {
            auto p = part("box", pose(0.0, y, 1.0));
            p.size = Eigen::Vector3d(0.3, 0.3, 2.0);
            o.parts.push_back(p);
        }
        scene.push_back(o);
    }
    return scene;
}

// What the Gazebo scene plugin publishes for the same materialized scene:
// one instance per part at world pose obstacle * part; cube scale = size,
// cylinder (r, r, full height), sphere (r, r, r), a collision mesh as a
// library template of support points with the collision scale.
void gazeboTruth(std::vector<GeometryTemplateDescription>* library,
                 std::vector<ConvexBodyDescription>* bodies) {
    const std::string mesh = "convex_mesh:file:///tmp/scene/rock.stl";
    GeometryTemplateDescription t;
    t.type = mesh;
    const Eigen::Vector3d mesh_scale(2.0, 2.0, 0.5);
    for (const auto& v : prism())
        t.support_points.push_back(v.cwiseQuotient(mesh_scale));
    library->push_back(t);
    for (const auto& o : documentScene()) {
        for (std::size_t i = 0; i < o.parts.size(); ++i) {
            const auto& part = o.parts[i];
            ConvexBodyDescription b;
            b.name = o.parts.size() == 1 ? o.id : o.id + "/" + std::to_string(i);
            b.pose = compose(o.pose, part.pose);
            if (part.type == "box") {
                b.geometry_type = "cube";
                b.scale = part.size;
            } else if (part.type == "cylinder") {
                b.geometry_type = "cylinder";
                b.scale = Eigen::Vector3d(part.radius, part.radius, part.height);
            } else if (part.type == "sphere") {
                b.geometry_type = "sphere";
                b.scale = Eigen::Vector3d::Constant(part.radius);
            } else {
                b.geometry_type = mesh;
                b.scale = mesh_scale;
            }
            bodies->push_back(b);
        }
    }
}

void testConversionRules() {
    std::printf("gazebo truth conversion: primitives, templates, refusals\n");
    std::vector<GeometryTemplateDescription> library;
    ConvexBodyDescription cube{"c", "cube", pose(1, 2, 3), Eigen::Vector3d(1, 2, 4)};
    const auto boxes = fromConvexBodies(library, {cube});
    CHECK(boxes.size() == 1 && boxes[0].type == Obstacle::kBox);
    CHECK((boxes[0].size - Eigen::Vector3d(1, 2, 4)).norm() < 1e-12);
    CHECK((boxes[0].position - Eigen::Vector3d(1, 2, 3)).norm() < 1e-12);

    ConvexBodyDescription cyl{"y", "cylinder", pose(0, 0, 0), Eigen::Vector3d(0.5, 0.5, 3.0)};
    const auto cyls = fromConvexBodies(library, {cyl});
    CHECK(cyls[0].type == Obstacle::kCylinder && cyls[0].radius == 0.5 && cyls[0].height == 3.0);

    ConvexBodyDescription ball{"s", "sphere", pose(0, 0, 0), Eigen::Vector3d::Constant(0.25)};
    CHECK(fromConvexBodies(library, {ball})[0].radius == 0.25);

    auto refused = [&](const std::vector<GeometryTemplateDescription>& lib,
                       const ConvexBodyDescription& b,
                       const std::string& needle) {
        try {
            fromConvexBodies(lib, {b});
        } catch (const std::invalid_argument& e) {
            return std::string(e.what()).find(needle) != std::string::npos;
        }
        return false;
    };
    ConvexBodyDescription unknown{
        "mystery", "convex_mesh:missing", pose(0, 0, 0), Eigen::Vector3d::Ones()};
    CHECK(refused(library, unknown, "mystery"));
    ConvexBodyDescription flat{"flat", "cube", pose(0, 0, 0), Eigen::Vector3d(1, 0, 1)};
    CHECK(refused(library, flat, "scale"));
    ConvexBodyDescription bad_q = cube;
    bad_q.pose.orientation = Eigen::Quaterniond(0, 0, 0, 0);
    CHECK(refused(library, bad_q, "orientation"));
    GeometryTemplateDescription degenerate{"convex_mesh:flat", {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}};
    ConvexBodyDescription flat_mesh{
        "sheet", "convex_mesh:flat", pose(0, 0, 0), Eigen::Vector3d::Ones()};
    CHECK(refused({degenerate}, flat_mesh, "sheet"));
}

void testSameScansFromBothSources() {
    std::printf("same toy scene, both sources: identical solids and scans in every mode\n");
    const auto from_document = toObstacles(documentScene());
    std::vector<GeometryTemplateDescription> library;
    std::vector<ConvexBodyDescription> bodies;
    gazeboTruth(&library, &bodies);
    const auto from_gazebo = fromConvexBodies(library, bodies);
    CHECK(from_document.size() == 6);
    CHECK(from_gazebo.size() == from_document.size());

    const std::vector<Pose> sensors = {
        pose(0, 0, 1.0), pose(1.5, -2.0, 1.2, 0.8), pose(-2.0, 1.0, 0.5, -2.0)};
    for (const auto mode :
         {SensorConfig::kRaycast, SensorConfig::kDepthFrustum, SensorConfig::kPenetrating}) {
        SensorConfig c;
        c.mode = mode;
        c.range = 15.0;
        c.h_res = 180;
        c.v_res = 16;
        c.v_fov_deg = mode == SensorConfig::kPenetrating ? 180.0 : 30.0;
        c.h_fov_deg = mode == SensorConfig::kDepthFrustum ? 90.0 : 360.0;
        c.surface_spacing = 0.2;
        WorldLidar document_lidar(c);
        WorldLidar gazebo_lidar(c);
        document_lidar.setScene(from_document);
        gazebo_lidar.setScene(from_gazebo);
        for (const auto& s : sensors) {
            const auto a = document_lidar.scan(s.position, s.orientation);
            const auto b = gazebo_lidar.scan(s.position, s.orientation);
            CHECK(!a.empty());
            CHECK(a.size() == b.size());
            double worst = 0.0;
            for (std::size_t i = 0; i < std::min(a.size(), b.size()); ++i)
                worst = std::max(worst, (a[i] - b[i]).norm());
            CHECK(worst < 1e-6);
            if (worst >= 1e-6)
                std::printf("    mode %d: worst point difference %.3g m\n", int(mode), worst);
        }
    }
    // The gate stays open: a beam through its middle passes.
    SensorConfig c;
    WorldLidar lidar(c);
    lidar.setScene(from_gazebo);
    const Eigen::Vector3d origin = pose(0.0, -6.0, 0.0, 0.2).position + Eigen::Vector3d(0, 0, 1.0);
    const Eigen::Vector3d through =
        Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d::UnitX();
    CHECK(std::isinf(lidar.castRay(origin - 3.0 * through, through, 6.0)));
}

} // namespace

int main() {
    testConversionRules();
    testSameScansFromBothSources();
    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
