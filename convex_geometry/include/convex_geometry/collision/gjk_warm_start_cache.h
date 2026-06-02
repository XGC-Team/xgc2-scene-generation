#ifndef CONVEX_GEOMETRY_COLLISION_GJK_WARM_START_CACHE_H
#define CONVEX_GEOMETRY_COLLISION_GJK_WARM_START_CACHE_H

#include <algorithm>
#include <vector>
#include "convex_geometry/collision/separation_query.h"

namespace convex_geometry {

class GjkWarmStartCache {
public:
    struct CacheLayout {
        int static_obstacles = 0;
        int neighbors = 0;
    };

    void initialize(int horizon, int expected_static_obstacles, int num_neighbors) {
        cache_layout_.static_obstacles = std::max(0, expected_static_obstacles);
        cache_layout_.neighbors = std::max(0, num_neighbors);

        // Stage warm starts are indexed by the absolute MPC stage in [0, N].
        // The current intermediate constraint update only visits stages 1..N-1,
        // so slots 0 and N are reserved and currently unused here.
        // Terminal queries use the dedicated terminal_*_warm_starts_ caches below.
        stage_static_warm_starts_.resize((horizon + 1) * cache_layout_.static_obstacles);
        terminal_static_gamma_warm_starts_.resize(cache_layout_.static_obstacles);
        
        stage_neighbor_warm_starts_.resize((horizon + 1) * cache_layout_.neighbors);
        terminal_neighbor_gamma_warm_starts_.resize(cache_layout_.neighbors);
    }

    int expectedStaticObstacles() const {
        return cache_layout_.static_obstacles;
    }

    int expectedNeighbors() const {
        return cache_layout_.neighbors;
    }

    CacheLayout cacheLayout() const {
        return cache_layout_;
    }

    std::vector<gjk::WarmStart>& stageStaticWarmStarts() {
        return stage_static_warm_starts_;
    }

    std::vector<gjk::WarmStart>& terminalStaticGammaWarmStarts() {
        return terminal_static_gamma_warm_starts_;
    }

    std::vector<gjk::WarmStart>& stageNeighborWarmStarts() {
        return stage_neighbor_warm_starts_;
    }

    std::vector<gjk::WarmStart>& terminalNeighborGammaWarmStarts() {
        return terminal_neighbor_gamma_warm_starts_;
    }

private:
    CacheLayout cache_layout_;
    std::vector<gjk::WarmStart> stage_static_warm_starts_;
    std::vector<gjk::WarmStart> terminal_static_gamma_warm_starts_;
    std::vector<gjk::WarmStart> stage_neighbor_warm_starts_;
    std::vector<gjk::WarmStart> terminal_neighbor_gamma_warm_starts_;
};

}  // namespace convex_geometry

#endif  // CONVEX_GEOMETRY_COLLISION_GJK_WARM_START_CACHE_H
