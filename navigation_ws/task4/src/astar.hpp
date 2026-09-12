#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <vector>

#include "map.hpp"

struct Result {
    bool found = false;
    int expanded = 0;
    double cost = 0.0;
    std::vector<Point> path;
    std::vector<unsigned char> visited;
};

inline Result find_path(const Map& map, Point start, Point goal, bool allowDiagonal) {
    const double INF = 1e18;
    const double ROOT2 = 1.4142135623730951;

    struct Node {
        double f = 0.0;
        double g = 0.0;
        int x = 0;
        int y = 0;

        bool operator>(const Node& other) const { return f > other.f; }
    };

    const auto heuristic = [goal, allowDiagonal, ROOT2](int x, int y) {
        const double distX = std::abs(x - goal.x);
        const double distY = std::abs(y - goal.y);
        if (!allowDiagonal)
            return distX + distY;
        return std::max(distX, distY) + (ROOT2 - 1.0) * std::min(distX, distY);
    };

    Result result;
    result.visited.assign(map.cells.size(), 0);

    std::vector<double> g(map.cells.size(), INF);
    std::vector<int> from(map.cells.size(), -1);
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

    const int startIndex = map.index(start.x, start.y);
    g[startIndex] = 0.0;
    open.push(Node{heuristic(start.x, start.y), 0.0, start.x, start.y});
    result.visited[startIndex] = 1;

    const int dirX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dirY[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!open.empty()) {
        const Node now = open.top();
        open.pop();

        const int nowIndex = map.index(now.x, now.y);
        if (now.g > g[nowIndex])
            continue;

        result.visited[nowIndex] = 2;
        ++result.expanded;

        if (now.x == goal.x && now.y == goal.y) {
            result.found = true;
            break;
        }

        const int count = allowDiagonal ? 8 : 4;
        for (int i = 0; i < count; ++i) {
            const int nextX = now.x + dirX[i];
            const int nextY = now.y + dirY[i];
            if (!map.free(nextX, nextY))
                continue;

            const bool diagonal = dirX[i] != 0 && dirY[i] != 0;
            if (diagonal && (!map.free(now.x + dirX[i], now.y) || !map.free(now.x, now.y + dirY[i])))
                continue;

            const double step = now.g + (diagonal ? ROOT2 : 1.0);
            const int nextIndex = map.index(nextX, nextY);
            if (step >= g[nextIndex])
                continue;

            g[nextIndex] = step;
            from[nextIndex] = nowIndex;
            result.visited[nextIndex] = 1;
            open.push(Node{step + heuristic(nextX, nextY), step, nextX, nextY});
        }
    }

    if (result.found) {
        const int goalIndex = map.index(goal.x, goal.y);
        result.cost = g[goalIndex];

        int index = goalIndex;
        while (index != -1) {
            result.path.push_back(Point{index % map.width, index / map.width});
            index = from[index];
        }
        std::reverse(result.path.begin(), result.path.end());
    }

    return result;
}
