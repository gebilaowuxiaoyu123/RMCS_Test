#include "astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace nav {

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kStraightCost = 1.0;
constexpr double kDiagonalCost = 1.4142135623730951;

struct QueueNode {
    double f = 0.0;
    double g = 0.0;
    int index = -1;

    bool operator>(const QueueNode& other) const {
        if (f != other.f)
            return f > other.f;
        return g < other.g;
    }
};

bool diagonal_allowed(
    const GridMap& map, GridPoint from, GridPoint to, bool prevent_corner_cutting) {
    if (!prevent_corner_cutting)
        return true;
    return map.is_free(GridPoint{to.x, from.y}) && map.is_free(GridPoint{from.x, to.y});
}

} // namespace

double heuristic_cost(Heuristic heuristic, GridPoint from, GridPoint to) {
    const double dx = std::abs(static_cast<double>(to.x - from.x));
    const double dy = std::abs(static_cast<double>(to.y - from.y));

    switch (heuristic) {
    case Heuristic::kZero: return 0.0;
    case Heuristic::kManhattan: return dx + dy;
    case Heuristic::kEuclidean: return std::sqrt(dx * dx + dy * dy);
    case Heuristic::kOctile:
        return kStraightCost * std::max(dx, dy) + (kDiagonalCost - kStraightCost) * std::min(dx, dy);
    }
    return 0.0;
}

SearchResult astar(const GridMap& map, GridPoint start, GridPoint goal, const AStarOptions& options) {
    if (!map.is_free(start) || !map.is_free(goal))
        throw std::invalid_argument("起点或终点落在障碍上");

    SearchResult result;
    result.state.assign(map.size(), 0);
    result.snapshots.reserve(options.max_snapshots);

    std::vector<double> g_score(map.size(), kInfinity);
    std::vector<int> parent(map.size(), -1);

    const int start_index = static_cast<int>(map.index(start));
    const int goal_index = static_cast<int>(map.index(goal));

    std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> open;

    g_score[static_cast<std::size_t>(start_index)] = 0.0;
    open.push(QueueNode{
        options.heuristic_weight * heuristic_cost(options.heuristic, start, goal), 0.0,
        start_index});
    result.state[static_cast<std::size_t>(start_index)] = 1;

    const int step[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    bool found = false;

    while (!open.empty()) {
        const QueueNode current = open.top();
        open.pop();

        const auto current_index = static_cast<std::size_t>(current.index);
        if (current.g > g_score[current_index])
            continue;

        result.state[current_index] = 2;
        ++result.stats.expanded;

        if (options.snapshot_interval != 0
            && result.stats.expanded % options.snapshot_interval == 0
            && result.snapshots.size() < options.max_snapshots) {
            result.snapshots.push_back(result.state);
            result.snapshot_expanded.push_back(result.stats.expanded);
        }

        if (current.index == goal_index) {
            found = true;
            break;
        }

        const GridPoint from{
            static_cast<int>(current_index % static_cast<std::size_t>(map.width())),
            static_cast<int>(current_index / static_cast<std::size_t>(map.width()))};

        const int neighbour_count = options.allow_diagonal ? 8 : 4;
        for (int i = 0; i < neighbour_count; ++i) {
            const GridPoint to{from.x + step[i][0], from.y + step[i][1]};
            if (!map.is_free(to))
                continue;

            const bool diagonal = step[i][0] != 0 && step[i][1] != 0;
            if (diagonal && !diagonal_allowed(map, from, to, options.prevent_corner_cutting))
                continue;

            const double tentative =
                g_score[current_index] + (diagonal ? kDiagonalCost : kStraightCost);
            const auto to_index = map.index(to);
            if (tentative >= g_score[to_index])
                continue;

            g_score[to_index] = tentative;
            parent[to_index] = current.index;
            result.state[to_index] = 1;
            ++result.stats.generated;
            open.push(QueueNode{
                tentative + options.heuristic_weight * heuristic_cost(options.heuristic, to, goal),
                tentative, static_cast<int>(to_index)});
        }
    }

    result.stats.found = found;

    if (found) {
        int index = goal_index;
        while (index != -1) {
            const GridPoint point{
                index % map.width(), index / map.width()};
            result.path.push_back(point);
            index = parent[static_cast<std::size_t>(index)];
        }
        std::reverse(result.path.begin(), result.path.end());
        result.stats.path_points = result.path.size();
        result.stats.path_cost = g_score[static_cast<std::size_t>(goal_index)];
    }

    return result;
}

} // namespace nav
