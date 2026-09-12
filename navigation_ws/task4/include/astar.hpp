#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "grid_map.hpp"

namespace nav {

struct AStarOptions {
    bool allow_diagonal = true;
    bool prevent_corner_cutting = true;
};

struct SearchStats {
    bool found = false;
    std::size_t expanded = 0;
    std::size_t path_points = 0;
    double path_cost = 0.0;
};

struct SearchResult {
    std::vector<GridPoint> path;
    std::vector<std::uint8_t> state;
    SearchStats stats;
};

double heuristic_cost(GridPoint from, GridPoint to, bool allow_diagonal);

SearchResult astar(
    const GridMap& map, GridPoint start, GridPoint goal, const AStarOptions& options = {});

} // namespace nav
