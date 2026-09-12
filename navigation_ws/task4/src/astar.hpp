#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <vector>

#include "map.hpp"

inline constexpr double INF = 1e18;

struct Node {
    int x = 0;
    int y = 0;
    double g = 0.0;
    double h = 0.0;

    double f() const { return g + h; }

    bool operator>(const Node& other) const { return f() > other.f(); }
};

struct Result {
    bool found = false;
    int expanded = 0;
    double g = 0.0;
    std::vector<Point> path;
    std::vector<unsigned char> colored;
};

inline double calc_h(const Node& node, Point goal) {
    return std::abs(node.x - goal.x) + std::abs(node.y - goal.y);
}

inline Result search(const Map& map, Point start, Point goal) {
    Result result;
    result.colored.assign(map.cells.size(), 0);

    std::vector<double> gBest(map.cells.size(), INF);
    std::vector<int> parent(map.cells.size(), -1);

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;

    Node startNode{start.x, start.y};
    startNode.h = calc_h(startNode, goal);

    const int startIndex = map.index(start.x, start.y);
    gBest[startIndex] = 0.0;
    result.colored[startIndex] = 1;
    openList.push(startNode);

    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};

    while (!openList.empty()) {
        const Node now = openList.top();
        openList.pop();

        const int nowIndex = map.index(now.x, now.y);
        if (result.colored[nowIndex] == 2)
            continue;

        result.colored[nowIndex] = 2;
        ++result.expanded;

        if (now.x == goal.x && now.y == goal.y) {
            result.found = true;
            result.g = now.g;
            break;
        }

        for (int i = 0; i < 4; ++i) {
            const int nextX = now.x + dx[i];
            const int nextY = now.y + dy[i];
            if (!map.free(nextX, nextY))
                continue;

            const int nextIndex = map.index(nextX, nextY);
            if (result.colored[nextIndex] == 2)
                continue;

            Node child{nextX, nextY};
            child.g = now.g + 1.0;
            if (child.g >= gBest[nextIndex])
                continue;

            child.h = calc_h(child, goal);
            gBest[nextIndex] = child.g;
            parent[nextIndex] = nowIndex;
            result.colored[nextIndex] = 1;
            openList.push(child);
        }
    }

    if (result.found) {
        int index = map.index(goal.x, goal.y);
        while (index != -1) {
            result.path.push_back(Point{index % map.width, index / map.width});
            index = parent[index];
        }
        std::reverse(result.path.begin(), result.path.end());
    }

    return result;
}
