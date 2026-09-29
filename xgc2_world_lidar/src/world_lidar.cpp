#include "xgc2_world_lidar/world_lidar.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <stdexcept>
#include <string>

namespace xgc2_world_lidar {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kPi = 3.14159265358979323846;

bool finite(const Eigen::Vector3d& v) { return v.allFinite(); }

Eigen::Matrix3d rotationOf(const Eigen::Quaterniond& q) {
    const double n = q.norm();
    if (!std::isfinite(n) || n < 1e-9) throw std::invalid_argument("invalid orientation quaternion");
    return Eigen::Quaterniond(q.coeffs() / n).toRotationMatrix();
}

// Interval of the line o + t d (t real) inside the slab |x| <= h along one
// axis. Returns false when the line misses the slab.
bool slab(double o, double d, double h, double* t0, double* t1) {
    if (std::abs(d) < 1e-15) {
        if (std::abs(o) > h) return false;
        return true;
    }
    double a = (-h - o) / d;
    double b = (h - o) / d;
    if (a > b) std::swap(a, b);
    *t0 = std::max(*t0, a);
    *t1 = std::min(*t1, b);
    return *t0 <= *t1;
}

// Line o + t d against sphere |x - c| <= r, d unit.
bool sphereInterval(const Eigen::Vector3d& o, const Eigen::Vector3d& d, const Eigen::Vector3d& c,
                    double r, double* t0, double* t1) {
    const Eigen::Vector3d oc = o - c;
    const double b = oc.dot(d);
    const double disc = b * b - (oc.squaredNorm() - r * r);
    if (disc < 0.0) return false;
    const double s = std::sqrt(disc);
    *t0 = -b - s;
    *t1 = -b + s;
    return true;
}

// Line against the infinite z-cylinder x^2 + y^2 <= r^2 intersected with the
// slab |z| <= hh, d unit.
bool cylinderInterval(const Eigen::Vector3d& o, const Eigen::Vector3d& d, double r, double hh,
                      double* t0, double* t1) {
    double lo = -kInf, hi = kInf;
    const double a = d.x() * d.x() + d.y() * d.y();
    const double c = o.x() * o.x() + o.y() * o.y() - r * r;
    if (a < 1e-15) {
        if (c > 0.0) return false;
    } else {
        const double b = o.x() * d.x() + o.y() * d.y();
        const double disc = b * b - a * c;
        if (disc < 0.0) return false;
        const double s = std::sqrt(disc);
        lo = (-b - s) / a;
        hi = (-b + s) / a;
    }
    if (!slab(o.z(), d.z(), hh, &lo, &hi)) return false;
    *t0 = lo;
    *t1 = hi;
    return true;
}

// Fibonacci lattice on the unit sphere.
std::vector<Eigen::Vector3d> fibonacciSphere(int n) {
    std::vector<Eigen::Vector3d> out;
    out.reserve(n);
    const double golden = kPi * (3.0 - std::sqrt(5.0));
    for (int i = 0; i < n; ++i) {
        const double z = 1.0 - 2.0 * (i + 0.5) / n;
        const double rho = std::sqrt(std::max(0.0, 1.0 - z * z));
        const double phi = golden * i;
        out.emplace_back(rho * std::cos(phi), rho * std::sin(phi), z);
    }
    return out;
}

void ring(double radius, double z, double spacing, std::vector<Eigen::Vector3d>* out) {
    const int n = std::max(8, static_cast<int>(std::ceil(2.0 * kPi * radius / spacing)));
    for (int k = 0; k < n; ++k) {
        const double a = 2.0 * kPi * k / n;
        out->emplace_back(radius * std::cos(a), radius * std::sin(a), z);
    }
}

void disc(double radius, double z, double spacing, std::vector<Eigen::Vector3d>* out) {
    out->emplace_back(0.0, 0.0, z);
    const int rings = std::max(1, static_cast<int>(std::ceil(radius / spacing)));
    for (int k = 1; k <= rings; ++k) ring(radius * k / rings, z, spacing, out);
}

// Samples a planar convex polygon (ordered vertices) at `spacing`: interior
// cell centres plus the boundary.
void samplePolygon(const std::vector<Eigen::Vector3d>& poly, const Eigen::Vector3d& normal,
                   double spacing, std::vector<Eigen::Vector3d>* out) {
    const std::size_t m = poly.size();
    for (std::size_t i = 0; i < m; ++i) {
        const Eigen::Vector3d& a = poly[i];
        const Eigen::Vector3d& b = poly[(i + 1) % m];
        const int k = std::max(1, static_cast<int>(std::ceil((b - a).norm() / spacing)));
        for (int j = 0; j < k; ++j) out->push_back(a + (b - a) * (static_cast<double>(j) / k));
    }
    const Eigen::Vector3d u = (poly[1] - poly[0]).normalized();
    const Eigen::Vector3d w = normal.cross(u);
    std::vector<Eigen::Vector2d> p2(m);
    Eigen::Vector2d lo(kInf, kInf), hi(-kInf, -kInf);
    for (std::size_t i = 0; i < m; ++i) {
        const Eigen::Vector3d r = poly[i] - poly[0];
        p2[i] = Eigen::Vector2d(r.dot(u), r.dot(w));
        lo = lo.cwiseMin(p2[i]);
        hi = hi.cwiseMax(p2[i]);
    }
    double area = 0.0;
    for (std::size_t i = 0; i < m; ++i) {
        const Eigen::Vector2d& a = p2[i];
        const Eigen::Vector2d& b = p2[(i + 1) % m];
        area += a.x() * b.y() - a.y() * b.x();
    }
    const double sign = area >= 0.0 ? 1.0 : -1.0;
    const int nu = std::max(1, static_cast<int>(std::ceil((hi.x() - lo.x()) / spacing)));
    const int nw = std::max(1, static_cast<int>(std::ceil((hi.y() - lo.y()) / spacing)));
    const double du = (hi.x() - lo.x()) / nu;
    const double dw = (hi.y() - lo.y()) / nw;
    for (int i = 0; i < nu; ++i) {
        for (int j = 0; j < nw; ++j) {
            const Eigen::Vector2d q(lo.x() + (i + 0.5) * du, lo.y() + (j + 0.5) * dw);
            bool inside = true;
            for (std::size_t e = 0; e < m && inside; ++e) {
                const Eigen::Vector2d& a = p2[e];
                const Eigen::Vector2d& b = p2[(e + 1) % m];
                const double cross = (b.x() - a.x()) * (q.y() - a.y()) - (b.y() - a.y()) * (q.x() - a.x());
                inside = sign * cross >= -1e-12;
            }
            if (inside) out->push_back(poly[0] + q.x() * u + q.y() * w);
        }
    }
}

}  // namespace

// ---------------------------------------------------------------------------
namespace detail {

struct Aabb {
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(kInf);
    Eigen::Vector3d hi = Eigen::Vector3d::Constant(-kInf);
    void grow(const Eigen::Vector3d& p) {
        lo = lo.cwiseMin(p);
        hi = hi.cwiseMax(p);
    }
    void grow(const Aabb& b) {
        lo = lo.cwiseMin(b.lo);
        hi = hi.cwiseMax(b.hi);
    }
    // Entry distance of the ray (o, 1/d) within [0, tmax], or +inf.
    double enter(const Eigen::Vector3d& o, const Eigen::Vector3d& inv, double tmax) const {
        double t0 = 0.0, t1 = tmax;
        for (int i = 0; i < 3; ++i) {
            double a = (lo[i] - o[i]) * inv[i];
            double b = (hi[i] - o[i]) * inv[i];
            if (a > b) std::swap(a, b);
            t0 = std::max(t0, a);
            t1 = std::min(t1, b);
            if (t0 > t1) return kInf;
        }
        return t0;
    }
};

struct Plane {
    Eigen::Vector3d n;  // unit, outward
    double d;           // inside: n . x <= d
};

struct Shape {
    Obstacle::Type type = Obstacle::kBox;
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();  // local -> world
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d half = Eigen::Vector3d::Zero();
    double radius = 0.0;
    double half_height = 0.0;
    std::vector<Plane> planes;                        // local (box, convex)
    std::vector<std::vector<Eigen::Vector3d>> faces;  // local polygons (box, convex)
    Aabb bounds;                                      // world
    Eigen::Vector3d center = Eigen::Vector3d::Zero(); // world bounding sphere
    double bound_radius = 0.0;
    std::vector<Eigen::Vector3d> samples;             // world (kPenetrating only)

    Eigen::Vector3d toLocal(const Eigen::Vector3d& world) const {
        return rotation.transpose() * (world - position);
    }

    // Interval of the world line o + t d (d unit) inside the solid.
    bool intersect(const Eigen::Vector3d& o, const Eigen::Vector3d& d, double* t0, double* t1) const {
        const Eigen::Vector3d ol = toLocal(o);
        const Eigen::Vector3d dl = rotation.transpose() * d;
        switch (type) {
            case Obstacle::kSphere:
                return sphereInterval(ol, dl, Eigen::Vector3d::Zero(), radius, t0, t1);
            case Obstacle::kCylinder:
                return cylinderInterval(ol, dl, radius, half_height, t0, t1);
            case Obstacle::kCapsule: {
                // Convex union: the line meets it in one interval, the hull of
                // the pieces' intervals.
                double lo = kInf, hi = -kInf, a, b;
                if (cylinderInterval(ol, dl, radius, half_height, &a, &b)) lo = std::min(lo, a), hi = std::max(hi, b);
                if (sphereInterval(ol, dl, Eigen::Vector3d(0, 0, half_height), radius, &a, &b))
                    lo = std::min(lo, a), hi = std::max(hi, b);
                if (sphereInterval(ol, dl, Eigen::Vector3d(0, 0, -half_height), radius, &a, &b))
                    lo = std::min(lo, a), hi = std::max(hi, b);
                if (lo > hi) return false;
                *t0 = lo;
                *t1 = hi;
                return true;
            }
            case Obstacle::kBox: {
                double lo = -kInf, hi = kInf;
                for (int i = 0; i < 3; ++i)
                    if (!slab(ol[i], dl[i], half[i], &lo, &hi)) return false;
                *t0 = lo;
                *t1 = hi;
                return true;
            }
            case Obstacle::kConvex: {
                double lo = -kInf, hi = kInf;
                for (const Plane& p : planes) {
                    const double den = p.n.dot(dl);
                    const double num = p.d - p.n.dot(ol);
                    if (std::abs(den) < 1e-15) {
                        if (num < 0.0) return false;
                        continue;
                    }
                    const double t = num / den;
                    if (den < 0.0) lo = std::max(lo, t);
                    else hi = std::min(hi, t);
                    if (lo > hi) return false;
                }
                *t0 = lo;
                *t1 = hi;
                return true;
            }
        }
        return false;
    }

    double residual(const Eigen::Vector3d& world) const {
        const Eigen::Vector3d q = toLocal(world);
        switch (type) {
            case Obstacle::kSphere:
                return q.norm() - radius;
            case Obstacle::kCylinder:
                return std::max(std::hypot(q.x(), q.y()) - radius, std::abs(q.z()) - half_height);
            case Obstacle::kCapsule: {
                const double z = std::clamp(q.z(), -half_height, half_height);
                return (q - Eigen::Vector3d(0, 0, z)).norm() - radius;
            }
            case Obstacle::kBox:
                return (q.cwiseAbs() - half).maxCoeff();
            case Obstacle::kConvex: {
                double r = -kInf;
                for (const Plane& p : planes) r = std::max(r, p.n.dot(q) - p.d);
                return r;
            }
        }
        return kInf;
    }
};

namespace {

// Convex hull by supporting planes of vertex triples: O(n^4), meant for the
// scene library's hand-made polytopes (tens of vertices).
void buildHull(const std::vector<Eigen::Vector3d>& input, Shape* shape) {
    std::vector<Eigen::Vector3d> v;
    for (const auto& p : input) {
        if (!finite(p)) throw std::invalid_argument("non-finite convex vertex");
        bool dup = false;
        for (const auto& q : v) dup = dup || (p - q).norm() < 1e-9;
        if (!dup) v.push_back(p);
    }
    if (v.size() < 4) throw std::invalid_argument("convex obstacle needs >= 4 distinct vertices");
    Eigen::Vector3d lo = v[0], hi = v[0];
    for (const auto& p : v) lo = lo.cwiseMin(p), hi = hi.cwiseMax(p);
    const double extent = std::max((hi - lo).norm(), 1e-9);
    const double eps = 1e-7 * extent;
    const std::size_t n = v.size();
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            for (std::size_t k = j + 1; k < n; ++k) {
                Eigen::Vector3d nn = (v[j] - v[i]).cross(v[k] - v[i]);
                const double len = nn.norm();
                if (len < 1e-10 * extent * extent) continue;
                nn /= len;
                const double d = nn.dot(v[i]);
                double smax = -kInf, smin = kInf;
                for (const auto& p : v) {
                    const double s = nn.dot(p) - d;
                    smax = std::max(smax, s);
                    smin = std::min(smin, s);
                }
                Plane plane;
                if (smax <= eps) plane = {nn, d};
                else if (smin >= -eps) plane = {-nn, -d};
                else continue;
                bool dup = false;
                for (const Plane& q : shape->planes)
                    dup = dup || (q.n.dot(plane.n) > 1.0 - 1e-9 && std::abs(q.d - plane.d) < eps);
                if (!dup) shape->planes.push_back(plane);
            }
        }
    }
    if (shape->planes.size() < 4) throw std::invalid_argument("convex obstacle is flat");
    for (const Plane& pl : shape->planes) {
        std::vector<Eigen::Vector3d> face;
        Eigen::Vector3d c = Eigen::Vector3d::Zero();
        for (const auto& p : v)
            if (std::abs(pl.n.dot(p) - pl.d) <= eps) face.push_back(p), c += p;
        c /= static_cast<double>(face.size());
        const Eigen::Vector3d u = (face[0] - c).normalized();
        const Eigen::Vector3d w = pl.n.cross(u);
        std::sort(face.begin(), face.end(), [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
            return std::atan2((a - c).dot(w), (a - c).dot(u)) < std::atan2((b - c).dot(w), (b - c).dot(u));
        });
        shape->faces.push_back(std::move(face));
    }
}

void boxFaces(Shape* s) {
    const Eigen::Vector3d h = s->half;
    for (int axis = 0; axis < 3; ++axis) {
        for (int sgn = -1; sgn <= 1; sgn += 2) {
            const int a = (axis + 1) % 3, b = (axis + 2) % 3;
            std::vector<Eigen::Vector3d> f;
            const std::array<std::array<int, 2>, 4> corners{{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}};
            for (const auto& cs : corners) {
                Eigen::Vector3d p;
                p[axis] = sgn * h[axis];
                p[a] = cs[0] * h[a];
                p[b] = cs[1] * h[b];
                f.push_back(p);
            }
            s->faces.push_back(std::move(f));
            Plane pl;
            pl.n = Eigen::Vector3d::Zero();
            pl.n[axis] = sgn;
            pl.d = h[axis];
            s->planes.push_back(pl);
        }
    }
}

Shape compile(const Obstacle& o) {
    Shape s;
    s.type = o.type;
    if (!finite(o.position)) throw std::invalid_argument("non-finite obstacle position");
    s.rotation = rotationOf(o.orientation);
    s.position = o.position;
    auto positive = [](double x, const char* what) {
        if (!std::isfinite(x) || x <= 0.0) throw std::invalid_argument(std::string("invalid ") + what);
    };
    std::vector<Eigen::Vector3d> hull_points;  // local points whose hull bounds the solid
    switch (o.type) {
        case Obstacle::kBox: {
            positive(o.size.x(), "box size");
            positive(o.size.y(), "box size");
            positive(o.size.z(), "box size");
            s.half = 0.5 * o.size;
            boxFaces(&s);
            for (const auto& f : s.faces) hull_points.insert(hull_points.end(), f.begin(), f.end());
            break;
        }
        case Obstacle::kSphere:
            positive(o.radius, "sphere radius");
            s.radius = o.radius;
            break;
        case Obstacle::kCylinder:
            positive(o.radius, "cylinder radius");
            positive(o.height, "cylinder height");
            s.radius = o.radius;
            s.half_height = 0.5 * o.height;
            break;
        case Obstacle::kCapsule:
            positive(o.radius, "capsule radius");
            if (!std::isfinite(o.height) || o.height < 0.0) throw std::invalid_argument("invalid capsule height");
            s.radius = o.radius;
            s.half_height = 0.5 * o.height;
            break;
        case Obstacle::kConvex:
            buildHull(o.vertices, &s);
            for (const auto& f : s.faces) hull_points.insert(hull_points.end(), f.begin(), f.end());
            break;
        default:
            throw std::invalid_argument("unknown obstacle type");
    }
    if (!hull_points.empty()) {
        for (const auto& p : hull_points) s.bounds.grow(s.position + s.rotation * p);
    } else {
        Eigen::Vector3d ext;
        if (o.type == Obstacle::kSphere) {
            ext.setConstant(s.radius);
        } else {
            const Eigen::Vector3d axis = s.rotation.col(2);
            for (int i = 0; i < 3; ++i) {
                const double a = std::abs(axis[i]);
                ext[i] = s.half_height * a +
                         (o.type == Obstacle::kCapsule ? s.radius : s.radius * std::sqrt(std::max(0.0, 1.0 - a * a)));
            }
        }
        s.bounds.lo = s.position - ext;
        s.bounds.hi = s.position + ext;
    }
    s.center = 0.5 * (s.bounds.lo + s.bounds.hi);
    s.bound_radius = 0.5 * (s.bounds.hi - s.bounds.lo).norm();
    return s;
}

std::vector<Eigen::Vector3d> sampleShape(const Shape& s, double spacing) {
    if (!std::isfinite(spacing) || spacing <= 0.0) throw std::invalid_argument("invalid surface spacing");
    std::vector<Eigen::Vector3d> local;
    switch (s.type) {
        case Obstacle::kBox:
        case Obstacle::kConvex:
            for (std::size_t i = 0; i < s.faces.size(); ++i) samplePolygon(s.faces[i], s.planes[i].n, spacing, &local);
            break;
        case Obstacle::kSphere: {
            const int n = std::max(12, static_cast<int>(std::ceil(4.0 * kPi * s.radius * s.radius / (spacing * spacing))));
            for (const auto& p : fibonacciSphere(n)) local.push_back(s.radius * p);
            break;
        }
        case Obstacle::kCylinder:
        case Obstacle::kCapsule: {
            const double hh = s.half_height;
            const int rings = std::max(1, static_cast<int>(std::ceil(2.0 * hh / spacing)));
            if (hh > 0.0)
                for (int k = 0; k <= rings; ++k) ring(s.radius, -hh + 2.0 * hh * k / rings, spacing, &local);
            if (s.type == Obstacle::kCylinder) {
                disc(s.radius, hh, spacing, &local);
                disc(s.radius, -hh, spacing, &local);
            } else {
                const int n = std::max(12, static_cast<int>(std::ceil(4.0 * kPi * s.radius * s.radius / (spacing * spacing))));
                for (const auto& p : fibonacciSphere(n))
                    local.push_back(s.radius * p + Eigen::Vector3d(0, 0, p.z() >= 0.0 ? hh : -hh));
            }
            break;
        }
    }
    std::vector<Eigen::Vector3d> world;
    world.reserve(local.size());
    for (const auto& p : local) world.push_back(s.position + s.rotation * p);
    return world;
}

// Surface samples of every shape, without those strictly inside another
// shape (unless keep_buried): what remains lies on the boundary of the union
// of the solids.
std::vector<std::vector<Eigen::Vector3d>> sampleScene(const std::vector<Shape>& shapes, double spacing,
                                                      bool keep_buried) {
    std::vector<std::vector<Eigen::Vector3d>> out(shapes.size());
    for (std::size_t i = 0; i < shapes.size(); ++i) {
        if (keep_buried) {
            out[i] = sampleShape(shapes[i], spacing);
            continue;
        }
        std::vector<const Shape*> overlapping;
        for (std::size_t j = 0; j < shapes.size(); ++j) {
            if (j == i) continue;
            const Aabb& a = shapes[i].bounds;
            const Aabb& b = shapes[j].bounds;
            if ((a.lo.array() <= b.hi.array()).all() && (b.lo.array() <= a.hi.array()).all())
                overlapping.push_back(&shapes[j]);
        }
        for (const auto& p : sampleShape(shapes[i], spacing)) {
            bool buried = false;
            for (const Shape* other : overlapping) {
                if (other->residual(p) < -1e-9) {
                    buried = true;
                    break;
                }
            }
            if (!buried) out[i].push_back(p);
        }
    }
    return out;
}

}  // namespace

// Bounding-volume hierarchy over obstacle AABBs (median split on the longest
// centroid axis, <= 2 obstacles per leaf).
struct Bvh {
    struct Node {
        Aabb box;
        int left = -1, right = -1;
        int first = 0, count = 0;
    };
    std::vector<Node> nodes;
    std::vector<int> order;

    void build(const std::vector<Shape>& shapes) {
        nodes.clear();
        order.resize(shapes.size());
        for (std::size_t i = 0; i < shapes.size(); ++i) order[i] = static_cast<int>(i);
        if (!shapes.empty()) buildNode(shapes, 0, static_cast<int>(shapes.size()));
    }

    int buildNode(const std::vector<Shape>& shapes, int first, int count) {
        const int index = static_cast<int>(nodes.size());
        nodes.emplace_back();
        Aabb box, centers;
        for (int i = first; i < first + count; ++i) {
            box.grow(shapes[order[i]].bounds);
            centers.grow(shapes[order[i]].center);
        }
        nodes[index].box = box;
        if (count <= 2) {
            nodes[index].first = first;
            nodes[index].count = count;
            return index;
        }
        int axis = 0;
        (centers.hi - centers.lo).maxCoeff(&axis);
        const int mid = first + count / 2;
        std::nth_element(order.begin() + first, order.begin() + mid, order.begin() + first + count,
                         [&](int a, int b) { return shapes[a].center[axis] < shapes[b].center[axis]; });
        const int left = buildNode(shapes, first, mid - first);
        const int right = buildNode(shapes, mid, first + count - mid);
        nodes[index].left = left;
        nodes[index].right = right;
        return index;
    }

    // First surface distance in [0, tmax], or +inf.
    double closest(const std::vector<Shape>& shapes, const Eigen::Vector3d& o, const Eigen::Vector3d& d,
                   double tmax) const {
        if (nodes.empty()) return kInf;
        Eigen::Vector3d inv;
        for (int i = 0; i < 3; ++i) inv[i] = 1.0 / (std::abs(d[i]) < 1e-300 ? 1e-300 : d[i]);
        double best = kInf;
        int stack[128];
        int top = 0;
        if (nodes[0].box.enter(o, inv, tmax) < kInf) stack[top++] = 0;
        while (top > 0) {
            const Node& node = nodes[stack[--top]];
            if (node.box.enter(o, inv, std::min(best, tmax)) == kInf) continue;
            if (node.left < 0) {
                for (int i = node.first; i < node.first + node.count; ++i) {
                    double t0, t1;
                    if (!shapes[order[i]].intersect(o, d, &t0, &t1)) continue;
                    const double t = t0 >= 0.0 ? t0 : t1;
                    if (t >= 0.0 && t <= tmax && t < best) best = t;
                }
                continue;
            }
            const double tl = nodes[node.left].box.enter(o, inv, std::min(best, tmax));
            const double tr = nodes[node.right].box.enter(o, inv, std::min(best, tmax));
            // Push the far child first so the near one is visited first.
            if (tl <= tr) {
                if (tr < kInf) stack[top++] = node.right;
                if (tl < kInf) stack[top++] = node.left;
            } else {
                if (tl < kInf) stack[top++] = node.left;
                if (tr < kInf) stack[top++] = node.right;
            }
        }
        return best;
    }
};

}  // namespace detail

// ---------------------------------------------------------------------------
Obstacle Obstacle::box(const Eigen::Vector3d& center, const Eigen::Vector3d& size,
                       const Eigen::Quaterniond& orientation) {
    Obstacle o;
    o.type = kBox;
    o.position = center;
    o.orientation = orientation;
    o.size = size;
    return o;
}

Obstacle Obstacle::alignedBox(const Eigen::Vector3d& min_corner, const Eigen::Vector3d& max_corner) {
    return box(0.5 * (min_corner + max_corner), max_corner - min_corner);
}

Obstacle Obstacle::sphere(const Eigen::Vector3d& center, double radius) {
    Obstacle o;
    o.type = kSphere;
    o.position = center;
    o.radius = radius;
    return o;
}

Obstacle Obstacle::cylinder(const Eigen::Vector3d& center, double radius, double height,
                            const Eigen::Quaterniond& orientation) {
    Obstacle o;
    o.type = kCylinder;
    o.position = center;
    o.orientation = orientation;
    o.radius = radius;
    o.height = height;
    return o;
}

Obstacle Obstacle::capsule(const Eigen::Vector3d& center, double radius, double height,
                           const Eigen::Quaterniond& orientation) {
    Obstacle o = cylinder(center, radius, height, orientation);
    o.type = kCapsule;
    return o;
}

Obstacle Obstacle::convexHull(const std::vector<Eigen::Vector3d>& world_vertices) {
    return convexHull(world_vertices, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
}

Obstacle Obstacle::convexHull(const std::vector<Eigen::Vector3d>& local_vertices, const Eigen::Vector3d& position,
                              const Eigen::Quaterniond& orientation) {
    Obstacle o;
    o.type = kConvex;
    o.position = position;
    o.orientation = orientation;
    o.vertices = local_vertices;
    return o;
}

std::vector<Eigen::Vector3d> sampleSurface(const Obstacle& obstacle, double spacing) {
    return detail::sampleShape(detail::compile(obstacle), spacing);
}

// ---------------------------------------------------------------------------
WorldLidar::WorldLidar(SensorConfig config) : config_(config), bvh_(new detail::Bvh) {
    SensorConfig& c = config_;
    if (!std::isfinite(c.range) || c.range <= 0.0) throw std::invalid_argument("range must be > 0");
    if (!std::isfinite(c.min_range) || c.min_range < 0.0 || c.min_range >= c.range)
        throw std::invalid_argument("min_range must be in [0, range)");
    if (!(c.h_fov_deg > 0.0 && c.h_fov_deg <= 360.0)) throw std::invalid_argument("h_fov_deg must be in (0, 360]");
    if (!(c.v_fov_deg >= 0.0 && c.v_fov_deg <= 180.0)) throw std::invalid_argument("v_fov_deg must be in [0, 180]");
    if (!std::isfinite(c.noise_std) || c.noise_std < 0.0) throw std::invalid_argument("noise_std must be >= 0");
    if (c.mode == SensorConfig::kPenetrating && !(std::isfinite(c.surface_spacing) && c.surface_spacing > 0.0))
        throw std::invalid_argument("surface_spacing must be > 0");
    if (!(c.heading_cos_min >= -1.0 && c.heading_cos_min <= 1.0))
        throw std::invalid_argument("heading_cos_min must be in [-1, 1]");
    if (!(std::isfinite(c.vertical_slab_tan) && c.vertical_slab_tan >= 0.0))
        throw std::invalid_argument("vertical_slab_tan must be >= 0");

    if (c.mode == SensorConfig::kDepthFrustum) {
        if (c.width < 1 || c.height < 1) throw std::invalid_argument("width and height must be >= 1");
        if (c.fx == 0.0 || c.fy == 0.0) {
            if (!(c.h_fov_deg < 180.0)) throw std::invalid_argument("kDepthFrustum needs h_fov_deg < 180");
            const double f = 0.5 * c.width / std::tan(0.5 * c.h_fov_deg * kPi / 180.0);
            if (c.fx == 0.0) c.fx = f;
            if (c.fy == 0.0) c.fy = c.fx;
        }
        if (c.cx == 0.0) c.cx = 0.5 * c.width;
        if (c.cy == 0.0) c.cy = 0.5 * c.height;
        if (!(c.fx > 0.0 && c.fy > 0.0 && std::isfinite(c.cx) && std::isfinite(c.cy)))
            throw std::invalid_argument("invalid pinhole intrinsics");
        beams_.reserve(static_cast<std::size_t>(c.width) * c.height);
        for (int v = 0; v < c.height; ++v)
            for (int u = 0; u < c.width; ++u)
                beams_.push_back(
                    Eigen::Vector3d(1.0, -(u + 0.5 - c.cx) / c.fx, -(v + 0.5 - c.cy) / c.fy).normalized());
        return;
    }
    if (c.h_res < 1 || c.v_res < 1) throw std::invalid_argument("h_res and v_res must be >= 1");
    const double h = c.h_fov_deg * kPi / 180.0;
    const double v = c.v_fov_deg * kPi / 180.0;
    const bool full_circle = c.h_fov_deg >= 360.0;
    beams_.reserve(static_cast<std::size_t>(c.h_res) * c.v_res);
    for (int j = 0; j < c.v_res; ++j) {
        const double el = c.v_res == 1 ? 0.0 : -0.5 * v + v * j / (c.v_res - 1);
        for (int i = 0; i < c.h_res; ++i) {
            double az;
            if (full_circle) az = -kPi + 2.0 * kPi * i / c.h_res;
            else az = c.h_res == 1 ? 0.0 : -0.5 * h + h * i / (c.h_res - 1);
            beams_.emplace_back(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
        }
    }
}

WorldLidar::~WorldLidar() = default;

std::size_t WorldLidar::obstacleCount() const { return shapes_.size(); }

void WorldLidar::setScene(const std::vector<Obstacle>& obstacles) {
    std::vector<detail::Shape> shapes;
    shapes.reserve(obstacles.size());
    for (const auto& o : obstacles) shapes.push_back(detail::compile(o));
    if (config_.mode == SensorConfig::kPenetrating) {
        auto samples = detail::sampleScene(shapes, config_.surface_spacing, config_.penetrating_keep_buried);
        for (std::size_t i = 0; i < shapes.size(); ++i) shapes[i].samples = std::move(samples[i]);
    }
    auto bvh = std::make_unique<detail::Bvh>();
    bvh->build(shapes);
    shapes_ = std::move(shapes);
    bvh_ = std::move(bvh);
}

double WorldLidar::castRay(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, double max_range) const {
    const double n = direction.norm();
    if (!(n > 0.0) || !finite(origin)) return kInf;
    return bvh_->closest(shapes_, origin, direction / n, max_range);
}

double WorldLidar::surfaceResidual(const Eigen::Vector3d& point) const {
    double r = kInf;
    for (const auto& s : shapes_) r = std::min(r, s.residual(point));
    return r;
}

std::vector<Eigen::Vector3d> WorldLidar::globalMap(double spacing) const {
    std::vector<Eigen::Vector3d> out;
    if (config_.mode == SensorConfig::kPenetrating && spacing == config_.surface_spacing) {
        for (const auto& s : shapes_) out.insert(out.end(), s.samples.begin(), s.samples.end());
        return out;
    }
    for (const auto& pts : detail::sampleScene(shapes_, spacing, config_.penetrating_keep_buried))
        out.insert(out.end(), pts.begin(), pts.end());
    return out;
}

namespace {

std::mt19937_64 scanRng(unsigned seed, uint64_t index) {
    std::seed_seq seq{static_cast<uint32_t>(seed), static_cast<uint32_t>(index), static_cast<uint32_t>(index >> 32)};
    return std::mt19937_64(seq);
}

void checkBodies(const std::vector<VehicleBody>& others) {
    for (const auto& b : others)
        if (!finite(b.position) || !std::isfinite(b.radius) || b.radius <= 0.0)
            throw std::invalid_argument("invalid vehicle body");
}

}  // namespace

template <class Sink>
void WorldLidar::traceBeams(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation, uint64_t scan_index,
                            const std::vector<VehicleBody>& others, Sink&& sink) const {
    const bool noisy = config_.noise_std > 0.0;
    std::mt19937_64 rng = scanRng(config_.seed, scan_index);
    std::normal_distribution<double> noise(0.0, noisy ? config_.noise_std : 1.0);
    std::vector<const VehicleBody*> near;  // bodies that can be reached within range
    for (const auto& b : others)
        if ((b.position - position).norm() - b.radius <= config_.range) near.push_back(&b);
    Beam beam;
    beam.origin = position;
    for (const auto& local : beams_) {
        const Eigen::Vector3d d = rotation * local;
        double t = bvh_->closest(shapes_, position, d, config_.range);
        int vehicle = -1;
        for (const VehicleBody* b : near) {
            double t0, t1;
            if (!sphereInterval(position, d, b->position, b->radius, &t0, &t1)) continue;
            const double tb = t0 >= 0.0 ? t0 : t1;
            if (tb >= 0.0 && tb <= config_.range && tb < t) t = tb, vehicle = b->id;
        }
        beam.direction = d;
        beam.hit = t != kInf;
        beam.vehicle_id = vehicle;
        if (beam.hit && noisy) t += noise(rng);
        if (beam.hit && ((noisy && t <= 0.0) || t > config_.range || t < config_.min_range)) beam.hit = false;
        if (!beam.hit) beam.vehicle_id = -1;
        beam.range = beam.hit ? t : config_.range;
        sink(beam);
    }
}

bool WorldLidar::insideFov(const Eigen::Vector3d& v) const {
    constexpr double tol = 1e-9;
    if (config_.h_fov_deg < 360.0) {
        const double az = std::atan2(v.y(), v.x());
        if (std::abs(az) > 0.5 * config_.h_fov_deg * kPi / 180.0 + tol) return false;
    }
    if (config_.v_fov_deg < 180.0) {
        const double el = std::atan2(v.z(), std::hypot(v.x(), v.y()));
        if (std::abs(el) > 0.5 * config_.v_fov_deg * kPi / 180.0 + tol) return false;
    }
    return true;
}

// ZJU local_sensing CPU crop: heading test on the full body x axis and a
// z slab of half-height tan * sensing_horizon (not a per-point elevation).
bool WorldLidar::insideCrop(const Eigen::Vector3d& v, const Eigen::Matrix3d& rotation) const {
    if (!config_.penetrating_heading_crop) return true;
    if (v.normalized().dot(rotation.col(0)) < config_.heading_cos_min) return false;
    return std::abs(v.z()) <= config_.vertical_slab_tan * config_.range;
}

template <class Sink>
void WorldLidar::samplePenetrating(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation,
                                   uint64_t scan_index, const std::vector<VehicleBody>& others,
                                   Sink&& sink) const {
    const double r2 = config_.range * config_.range;
    const double rmin2 = config_.min_range * config_.min_range;
    const bool noisy = config_.noise_std > 0.0;
    std::mt19937_64 rng = scanRng(config_.seed, scan_index);
    std::normal_distribution<double> noise(0.0, noisy ? config_.noise_std : 1.0);
    const Eigen::Matrix3d to_sensor = rotation.transpose();
    auto consider = [&](const Eigen::Vector3d& p, int vehicle) {
        const Eigen::Vector3d v = p - position;
        const double d2 = v.squaredNorm();
        if (d2 > r2 || d2 == 0.0 || d2 < rmin2) return;
        if (!insideFov(to_sensor * v) || !insideCrop(v, rotation)) return;
        if (!noisy) {
            sink(p, vehicle);
            return;
        }
        const double dist = std::sqrt(d2);
        const double t = dist + noise(rng);
        if (t <= 0.0 || t > config_.range || t < config_.min_range) return;
        sink(Eigen::Vector3d(position + v * (t / dist)), vehicle);
    };
    for (const auto& s : shapes_) {
        if ((s.center - position).norm() - s.bound_radius > config_.range) continue;
        for (const auto& p : s.samples) consider(p, -1);
    }
    for (const auto& b : others) {
        if ((b.position - position).norm() - b.radius > config_.range) continue;
        const int n = std::max(12, static_cast<int>(std::ceil(4.0 * kPi * b.radius * b.radius /
                                                              (config_.surface_spacing * config_.surface_spacing))));
        for (const auto& u : fibonacciSphere(n)) consider(b.position + b.radius * u, b.id);
    }
}

std::vector<Eigen::Vector3d> WorldLidar::scan(const Eigen::Vector3d& position,
                                              const Eigen::Quaterniond& attitude) const {
    if (!finite(position)) throw std::invalid_argument("non-finite sensor position");
    const Eigen::Matrix3d rotation = rotationOf(attitude);
    const uint64_t index = scan_counter_.fetch_add(1, std::memory_order_relaxed);
    std::vector<Eigen::Vector3d> out;
    if (config_.mode == SensorConfig::kPenetrating) {
        samplePenetrating(position, rotation, index, {}, [&](const Eigen::Vector3d& p, int) { out.push_back(p); });
    } else {
        out.reserve(beams_.size() / 2);
        traceBeams(position, rotation, index, {}, [&](const Beam& b) {
            if (b.hit) out.push_back(b.origin + b.range * b.direction);
        });
    }
    return out;
}

std::vector<Beam> WorldLidar::scanWithBeams(const Eigen::Vector3d& position, const Eigen::Quaterniond& attitude,
                                            const std::vector<VehicleBody>& others) const {
    if (config_.mode == SensorConfig::kPenetrating)
        throw std::logic_error("scanWithBeams: kPenetrating has no beams");
    if (!finite(position)) throw std::invalid_argument("non-finite sensor position");
    checkBodies(others);
    const Eigen::Matrix3d rotation = rotationOf(attitude);
    const uint64_t index = scan_counter_.fetch_add(1, std::memory_order_relaxed);
    std::vector<Beam> out;
    out.reserve(beams_.size());
    traceBeams(position, rotation, index, others, [&](const Beam& b) { out.push_back(b); });
    return out;
}

std::vector<TaggedPoint> WorldLidar::scanTagged(const Eigen::Vector3d& position, const Eigen::Quaterniond& attitude,
                                                const std::vector<VehicleBody>& others) const {
    if (!finite(position)) throw std::invalid_argument("non-finite sensor position");
    checkBodies(others);
    const Eigen::Matrix3d rotation = rotationOf(attitude);
    const uint64_t index = scan_counter_.fetch_add(1, std::memory_order_relaxed);
    std::vector<TaggedPoint> out;
    if (config_.mode == SensorConfig::kPenetrating) {
        samplePenetrating(position, rotation, index, others,
                          [&](const Eigen::Vector3d& p, int id) { out.push_back({p, id}); });
    } else {
        traceBeams(position, rotation, index, others, [&](const Beam& b) {
            if (b.hit) out.push_back({b.origin + b.range * b.direction, b.vehicle_id});
        });
    }
    return out;
}

}  // namespace xgc2_world_lidar
