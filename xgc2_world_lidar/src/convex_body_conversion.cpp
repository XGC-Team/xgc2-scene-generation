#include "xgc2_world_lidar/convex_body_conversion.h"

#include <cmath>
#include <map>
#include <stdexcept>

namespace xgc2_world_lidar {

namespace {

Eigen::Quaterniond normalized(const Eigen::Quaterniond& q) {
    const double n = q.norm();
    if (!std::isfinite(n) || n < 1e-9) throw std::invalid_argument("invalid orientation");
    return Eigen::Quaterniond(q.coeffs() / n);
}

bool positive(const Eigen::Vector3d& v) {
    return v.allFinite() && v.x() > 0.0 && v.y() > 0.0 && v.z() > 0.0;
}

}  // namespace

std::vector<Obstacle> fromConvexBodies(const std::vector<GeometryTemplateDescription>& library,
                                       const std::vector<ConvexBodyDescription>& bodies) {
    std::map<std::string, const GeometryTemplateDescription*> templates;
    for (const auto& t : library) templates[t.type] = &t;

    std::vector<Obstacle> out;
    out.reserve(bodies.size());
    for (const auto& body : bodies) {
        try {
            if (!body.pose.position.allFinite()) throw std::invalid_argument("invalid position");
            const Eigen::Quaterniond q = normalized(body.pose.orientation);
            const Eigen::Vector3d& p = body.pose.position;
            if (!positive(body.scale)) throw std::invalid_argument("scale must be positive and finite");
            Obstacle o;
            if (body.geometry_type == "cube") {
                o = Obstacle::box(p, body.scale, q);
            } else if (body.geometry_type == "sphere") {
                o = Obstacle::sphere(p, body.scale.x());
            } else if (body.geometry_type == "cylinder") {
                o = Obstacle::cylinder(p, body.scale.x(), body.scale.z(), q);
            } else {
                const auto found = templates.find(body.geometry_type);
                if (found == templates.end())
                    throw std::invalid_argument("no geometry template '" + body.geometry_type + "'");
                std::vector<Eigen::Vector3d> local;
                local.reserve(found->second->support_points.size());
                for (const auto& s : found->second->support_points) local.push_back(s.cwiseProduct(body.scale));
                o = Obstacle::convexHull(local, p, q);
            }
            sampleSurface(o, 1e9);  // validates geometry (one sample per face)
            out.push_back(std::move(o));
        } catch (const std::invalid_argument& e) {
            throw std::invalid_argument("convex body '" + body.name + "': " + e.what());
        }
    }
    return out;
}

}  // namespace xgc2_world_lidar
