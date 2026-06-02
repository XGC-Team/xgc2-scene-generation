#ifndef CONVEX_GEOMETRY_OCCUPIED_SETS_CONVEX_BODY_FACTORY_H
#define CONVEX_GEOMETRY_OCCUPIED_SETS_CONVEX_BODY_FACTORY_H

#include <string>
#include <vector>
#include <Eigen/Dense>
#include "convex_geometry/ConvexBodyInstance.h"
#include "convex_geometry/GeometryTemplate.h"
#include "convex_geometry/occupied_sets/box_set.h"
#include "convex_geometry/occupied_sets/capsule_set.h"
#include "convex_geometry/occupied_sets/convex_body.h"
#include "convex_geometry/occupied_sets/cylinder_set.h"
#include "convex_geometry/occupied_sets/ellipsoid_set.h"
#include "convex_geometry/occupied_sets/point_set.h"
#include "convex_geometry/occupied_sets/sphere_set.h"

namespace convex_geometry {

class ConvexBodyFactory {
public:
    static ConvexBody makeSphereBody(const Eigen::Vector3d& center,
                                     double radius,
                                     int id = -1,
                                     const std::string& name = "sphere_body") {
        ConvexBody body;
        body.id = id;
        body.name = name;
        body.geometry_type = "sphere";
        body.position = center;
        body.scale = Eigen::Vector3d(radius, radius, radius);
        body.shape = std::make_shared<SphereSet>(center, radius);
        return body;
    }

    static ConvexBody makeFromBodyInstance(const ConvexBodyInstance& instance,
                                           const GeometryTemplate& tmpl) {
        ConvexBody body;
        body.id = instance.id;
        body.name = instance.name;
        body.geometry_type = instance.geometry_type;
        body.position = Eigen::Vector3d(
            instance.pose.position.x,
            instance.pose.position.y,
            instance.pose.position.z);
        body.scale = Eigen::Vector3d(
            instance.scale.x,
            instance.scale.y,
            instance.scale.z);
        body.orientation = Eigen::Quaterniond(
            instance.pose.orientation.w,
            instance.pose.orientation.x,
            instance.pose.orientation.y,
            instance.pose.orientation.z);
        body.velocity = Eigen::Vector3d(
            instance.velocity.linear.x,
            instance.velocity.linear.y,
            instance.velocity.linear.z);

        if (instance.geometry_type == "sphere") {
            body.shape = std::make_shared<SphereSet>(body.position, body.scale.x());
            return body;
        }

        if (instance.geometry_type == "ellipsoid") {
            body.shape = std::make_shared<EllipsoidSet>(
                body.position, body.scale, body.orientation);
            return body;
        }

        if (instance.geometry_type == "cube") {
            body.shape = std::make_shared<BoxSet>(body.position, body.scale, body.orientation);
            return body;
        }

        if (instance.geometry_type == "cylinder") {
            body.shape = std::make_shared<CylinderSet>(
                body.position, body.scale.x(), body.scale.z(), body.orientation);
            return body;
        }

        if (instance.geometry_type == "capsule") {
            body.shape = std::make_shared<CapsuleSet>(
                body.position, body.scale.x(), body.scale.z(), body.orientation);
            return body;
        }

        if (isPolytopeType(instance.geometry_type)) {
            body.shape = std::make_shared<SupportPointSet>(
                body.position,
                transformTemplateSupportPoints(tmpl, body.scale, body.orientation));
            return body;
        }

        body.shape = std::make_shared<SupportPointSet>(
            body.position,
            transformTemplateSupportPoints(tmpl, body.scale, body.orientation));
        return body;
    }

private:
    static bool isPolytopeType(const std::string& geometry_type) {
        return geometry_type == "h_polytope" ||
               geometry_type == "v_polytope" ||
               geometry_type.find("h_polytope:") == 0 ||
               geometry_type.find("v_polytope:") == 0;
    }

    static std::vector<Eigen::Vector3d> transformTemplateSupportPoints(
        const GeometryTemplate& tmpl,
        const Eigen::Vector3d& scale,
        const Eigen::Quaterniond& orientation) {
        std::vector<Eigen::Vector3d> offsets;
        offsets.reserve(tmpl.support_points.size());
        const Eigen::Quaterniond unit_orientation = orientation.normalized();
        for (const auto& point : tmpl.support_points) {
            const Eigen::Vector3d scaled_point(
                point.x * scale.x(),
                point.y * scale.y(),
                point.z * scale.z());
            offsets.push_back(unit_orientation * scaled_point);
        }
        return offsets;
    }
};

}  // namespace convex_geometry

#endif  // CONVEX_GEOMETRY_OCCUPIED_SETS_CONVEX_BODY_FACTORY_H
