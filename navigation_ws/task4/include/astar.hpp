#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "grid_map.hpp"

namespace nav {

enum class Heuristic { kZero, kManhattan, kEuclidean, kOctile };

struct AStarOptions {
    bool allow_diagonal = true;
    bool prevent_corner_cutting = true;
    Heuristic heuristic = Heuristic::kOctile;
    double heuristic_weight = 1.0;
    std::size_t snapshot_interval = 0;
    std::size_t max_snapshots = 4;
};

struct SearchStats {
    bool found = false;
    std::size_t expanded = 0;
    std::size_t generated = 0;
    std::size_t path_points = 0;
    double path_cost = 0.0;
};

struct SearchResult {
    std::vector<GridPoint> path;
    std::vector<std::uint8_t> state;
    std::vector<std::vector<std::uint8_t>> snapshots;
    std::vector<std::size_t> snapshot_expanded;
    SearchStats stats;
};

double heuristic_cost(Heuristic heuristic, GridPoint from, GridPoint to);

SearchResult astar(
    const GridMap& map, GridPoint start, GridPoint goal, const AStarOptions& options = {});

} // namespace nav
