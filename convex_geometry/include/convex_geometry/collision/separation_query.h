#ifndef CONVEX_GEOMETRY_COLLISION_SEPARATION_QUERY_H
#define CONVEX_GEOMETRY_COLLISION_SEPARATION_QUERY_H

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <limits>

#include "convex_geometry/geometry/math_helpers.h"

#ifndef CONVEX_GEOMETRY_GJK_ALWAYS_TRUST_WARM_START
#define CONVEX_GEOMETRY_GJK_ALWAYS_TRUST_WARM_START 1
#endif

namespace convex_geometry {
namespace gjk {

struct SeparationQuadruple {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitX();
    double margin = std::numeric_limits<double>::infinity();
    Eigen::Vector3d point_a = Eigen::Vector3d::Zero();
    Eigen::Vector3d point_b = Eigen::Vector3d::Zero();
};

struct Result {
    enum class Status {
        kSuccess,       // Produced a usable separation result from the current query.
        kOverlap,       // Current sets fell below the minimum distance; returned fallback/current separator.
        kMaxIterations, // Iteration budget exhausted; returned the latest usable separator with warning.
        kInvalid        // Invalid inputs prevented the query from running.
    };

    Status status = Status::kInvalid;
    SeparationQuadruple separator;
    int iterations = 0;
    double distance_gjk_time_us = 0.0;
    double guide_correction_time_us = 0.0;
    bool guide_attempted = false;
    bool guide_success = false;
};

struct WarmStart {
    SeparationQuadruple separator;
    bool valid = false;  // True only after a previous query produced a usable separator.
};

namespace detail {

constexpr int kSimplexMaxVertices = 4;

struct Vertex {
    Eigen::Vector3d a = Eigen::Vector3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    Eigen::Vector3d w = Eigen::Vector3d::Zero();
    double weight = 0.0;
    int source = -1;
};

struct Simplex {
    std::array<Vertex, kSimplexMaxVertices> vertices{};
    int count = 0;
    Eigen::Vector3d closest = Eigen::Vector3d::Zero();
    Eigen::Vector3d point_a = Eigen::Vector3d::Zero();
    Eigen::Vector3d point_b = Eigen::Vector3d::Zero();

    void assignSingle(const Vertex& vertex) {
        vertices[0] = vertex;
        vertices[0].weight = 1.0;
        count = 1;
        updateClosest();
    }

    void append(const Vertex& vertex) {
        vertices[count++] = vertex;
    }

    void updateClosest() {
        point_a.setZero();
        point_b.setZero();
        closest.setZero();
        for (int i = 0; i < count; ++i) {
            point_a += vertices[i].weight * vertices[i].a;
            point_b += vertices[i].weight * vertices[i].b;
        }
        closest = point_b - point_a;
    }
};

inline void reduceToVertex(Simplex& simplex, int idx) {
    simplex.vertices[0] = simplex.vertices[idx];
    simplex.vertices[0].weight = 1.0;
    simplex.count = 1;
    simplex.updateClosest();
}

inline void reduceToSegment(Simplex& simplex, int idx0, int idx1, double w0, double w1) {
    simplex.vertices[0] = simplex.vertices[idx0];
    simplex.vertices[1] = simplex.vertices[idx1];
    simplex.vertices[0].weight = w0;
    simplex.vertices[1].weight = w1;
    simplex.count = 2;
    simplex.updateClosest();
}

inline void reduceToTriangle(Simplex& simplex,
                             int idx0, int idx1, int idx2,
                             double w0, double w1, double w2) {
    simplex.vertices[0] = simplex.vertices[idx0];
    simplex.vertices[1] = simplex.vertices[idx1];
    simplex.vertices[2] = simplex.vertices[idx2];
    simplex.vertices[0].weight = w0;
    simplex.vertices[1].weight = w1;
    simplex.vertices[2].weight = w2;
    simplex.count = 3;
    simplex.updateClosest();
}

inline void deduplicateVertices(Simplex& simplex) {
    if (simplex.count <= 1) {
        return;
    }

    for (int i = 0; i < simplex.count; ++i) {
        for (int j = i + 1; j < simplex.count;) {
            if ((simplex.vertices[i].w - simplex.vertices[j].w).squaredNorm() <= 1e-12 &&
                (simplex.vertices[i].a - simplex.vertices[j].a).squaredNorm() <= 1e-12 &&
                (simplex.vertices[i].b - simplex.vertices[j].b).squaredNorm() <= 1e-12) {
                simplex.vertices[i].weight += simplex.vertices[j].weight;
                for (int k = j; k + 1 < simplex.count; ++k) {
                    simplex.vertices[k] = simplex.vertices[k + 1];
                }
                --simplex.count;
                continue;
            }
            ++j;
        }
    }

    double weight_sum = 0.0;
    for (int i = 0; i < simplex.count; ++i) {
        weight_sum += simplex.vertices[i].weight;
    }
    if (weight_sum > 1e-12) {
        for (int i = 0; i < simplex.count; ++i) {
            simplex.vertices[i].weight /= weight_sum;
        }
    }
    simplex.updateClosest();
}

inline bool hasEquivalentVertex(const Simplex& simplex, const Vertex& candidate) {
    for (int i = 0; i < simplex.count; ++i) {
        if ((simplex.vertices[i].w - candidate.w).squaredNorm() <= 1e-12 &&
            (simplex.vertices[i].a - candidate.a).squaredNorm() <= 1e-12 &&
            (simplex.vertices[i].b - candidate.b).squaredNorm() <= 1e-12) {
            return true;
        }
    }
    return false;
}

inline void solveSegment(Simplex& simplex) {
    const Eigen::Vector3d& a = simplex.vertices[0].w;
    const Eigen::Vector3d& b = simplex.vertices[1].w;
    const Eigen::Vector3d ab = b - a;
    const double denom = ab.squaredNorm();
    if (denom <= 1e-12) {
        reduceToVertex(simplex, 0);
        return;
    }

    const double t = math_helpers::clamp(-a.dot(ab) / denom, 0.0, 1.0);
    if (t <= 0.0) {
        reduceToVertex(simplex, 0);
        return;
    }
    if (t >= 1.0) {
        reduceToVertex(simplex, 1);
        return;
    }

    reduceToSegment(simplex, 0, 1, 1.0 - t, t);
}

inline void solveTriangle(Simplex& simplex) {
    const Eigen::Vector3d& a = simplex.vertices[0].w;
    const Eigen::Vector3d& b = simplex.vertices[1].w;
    const Eigen::Vector3d& c = simplex.vertices[2].w;
    const Eigen::Vector3d ab = b - a;
    const Eigen::Vector3d ac = c - a;
    const Eigen::Vector3d ap = -a;

    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
        reduceToVertex(simplex, 0);
        return;
    }

    const Eigen::Vector3d bp = -b;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3) {
        reduceToVertex(simplex, 1);
        return;
    }

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = d1 / (d1 - d3);
        reduceToSegment(simplex, 0, 1, 1.0 - v, v);
        return;
    }

    const Eigen::Vector3d cp = -c;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6) {
        reduceToVertex(simplex, 2);
        return;
    }

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double w = d2 / (d2 - d6);
        reduceToSegment(simplex, 0, 2, 1.0 - w, w);
        return;
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        reduceToSegment(simplex, 1, 2, 1.0 - w, w);
        return;
    }

    const double denom = va + vb + vc;
    if (std::abs(denom) <= 1e-12) {
        reduceToVertex(simplex, 0);
        return;
    }

    const double v = vb / denom;
    const double w = vc / denom;
    const double u = 1.0 - v - w;
    reduceToTriangle(simplex, 0, 1, 2, u, v, w);
}

inline bool pointOutsideOfPlane(const Eigen::Vector3d& p,
                                const Eigen::Vector3d& a,
                                const Eigen::Vector3d& b,
                                const Eigen::Vector3d& c,
                                const Eigen::Vector3d& d) {
    const Eigen::Vector3d normal = (b - a).cross(c - a);
    const double sign_p = (p - a).dot(normal);
    const double sign_d = (d - a).dot(normal);
    if (std::abs(sign_d) <= 1e-12) {
        return false;
    }
    return sign_p * sign_d < 0.0;
}

inline bool solveTetrahedron(Simplex& simplex) {
    const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    const auto& a = simplex.vertices[0].w;
    const auto& b = simplex.vertices[1].w;
    const auto& c = simplex.vertices[2].w;
    const auto& d = simplex.vertices[3].w;

    struct FaceCandidate {
        bool valid = false;
        double dist_sq = std::numeric_limits<double>::infinity();
        int count = 0;
        std::array<int, 3> indices{0, 0, 0};
        std::array<double, 3> weights{0.0, 0.0, 0.0};
    };

    auto evaluate_face = [&](int idx0, int idx1, int idx2) -> FaceCandidate {
        FaceCandidate candidate;
        Simplex face_simplex;
        face_simplex.count = 3;
        face_simplex.vertices[0] = simplex.vertices[idx0];
        face_simplex.vertices[1] = simplex.vertices[idx1];
        face_simplex.vertices[2] = simplex.vertices[idx2];
        face_simplex.vertices[0].source = idx0;
        face_simplex.vertices[1].source = idx1;
        face_simplex.vertices[2].source = idx2;
        solveTriangle(face_simplex);
        candidate.valid = true;
        candidate.dist_sq = face_simplex.closest.squaredNorm();
        candidate.count = face_simplex.count;
        for (int i = 0; i < face_simplex.count; ++i) {
            candidate.indices[i] = face_simplex.vertices[i].source;
            candidate.weights[i] = face_simplex.vertices[i].weight;
        }
        return candidate;
    };

    std::array<FaceCandidate, 4> candidates{};
    int candidate_count = 0;

    if (pointOutsideOfPlane(origin, a, b, c, d)) {
        candidates[candidate_count++] = evaluate_face(0, 1, 2);
    }
    if (pointOutsideOfPlane(origin, a, c, d, b)) {
        candidates[candidate_count++] = evaluate_face(0, 2, 3);
    }
    if (pointOutsideOfPlane(origin, a, d, b, c)) {
        candidates[candidate_count++] = evaluate_face(0, 3, 1);
    }
    if (pointOutsideOfPlane(origin, b, d, c, a)) {
        candidates[candidate_count++] = evaluate_face(1, 3, 2);
    }

    if (candidate_count == 0) {
        return true;
    }

    int best_idx = 0;
    for (int i = 1; i < candidate_count; ++i) {
        if (candidates[i].dist_sq < candidates[best_idx].dist_sq) {
            best_idx = i;
        }
    }

    const FaceCandidate& best = candidates[best_idx];
    if (!best.valid || best.count <= 0) {
        return true;
    }

    if (best.count == 1) {
        reduceToVertex(simplex, best.indices[0]);
    } else if (best.count == 2) {
        reduceToSegment(simplex,
                        best.indices[0], best.indices[1],
                        best.weights[0], best.weights[1]);
    } else {
        reduceToTriangle(simplex,
                         best.indices[0], best.indices[1], best.indices[2],
                         best.weights[0], best.weights[1], best.weights[2]);
    }

    return false;
}

inline bool solveSimplex(Simplex& simplex) {
    switch (simplex.count) {
        case 1:
            simplex.vertices[0].weight = 1.0;
            simplex.updateClosest();
            deduplicateVertices(simplex);
            return false;
        case 2:
            solveSegment(simplex);
            deduplicateVertices(simplex);
            return false;
        case 3:
            solveTriangle(simplex);
            deduplicateVertices(simplex);
            return false;
        case 4:
            if (solveTetrahedron(simplex)) {
                return true;
            }
            deduplicateVertices(simplex);
            return false;
        default:
            return false;
    }
}

inline Eigen::Vector3d chooseInitialDirection(const Eigen::Vector3d& center_delta,
                                              const WarmStart* warm_start) {
    if (warm_start != nullptr &&
        warm_start->valid &&
        warm_start->separator.normal.squaredNorm() > 1e-12) {
        return warm_start->separator.normal;
    }
    if (center_delta.squaredNorm() > 1e-12) {
        return center_delta;
    }
    return Eigen::Vector3d::UnitX();
}

}  // namespace detail
}  // namespace gjk
}  // namespace convex_geometry

#endif  // CONVEX_GEOMETRY_COLLISION_SEPARATION_QUERY_H
