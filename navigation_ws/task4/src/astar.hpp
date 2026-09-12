#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "map.hpp"

// OpenList 里的一个待办点
struct Node {
    int x = 0;
    int y = 0;
    double g = 0.0;    // g(n)：从起点走到这里已经花了多少代价
    double h = 0.0;    // h(n)：估计从这里到终点还要花多少
};

// 一次寻路的结果
struct Result {
    bool found = false;                    // 找到没有
    int expanded = 0;                      // 展开了多少格（也就是 CloseList 的大小）
    double g = 0.0;                        // 终点的 g(n)，也就是整条路的代价
    std::vector<Point> path;               // 起点到终点的路
    std::vector<unsigned char> colored;    // 0 没碰过、1 在 OpenList 里、2 在 CloseList 里
};

// f(n) = g(n) + h(n)
double calc_f(Node node) {
    return node.g + node.h;
}

// h(n) 用曼哈顿距离：横着差多少格 + 竖着差多少格
double calc_h(Node node, Point goal) {
    int dx = std::abs(node.x - goal.x);
    int dy = std::abs(node.y - goal.y);
    return dx + dy;
}

Result search(const Map& map, Point start, Point goal) {
    const double INF = 1e18;    // 代表"还没到过"

    Result result;
    result.colored.assign(map.cells.size(), 0);

    // gBest[i] = 第 i 格目前找到的最好的 g，初始都是无穷大
    std::vector<double> gBest(map.cells.size(), INF);
    // parent[i] = 第 i 格是从哪一格走过来的
    std::vector<int> parent(map.cells.size(), -1);

    // OpenList 就用一个普通数组装着
    std::vector<Node> openList;

    Node startNode;
    startNode.x = start.x;
    startNode.y = start.y;
    startNode.g = 0.0;
    startNode.h = calc_h(startNode, goal);
    openList.push_back(startNode);

    int startIndex = map.index(start.x, start.y);
    gBest[startIndex] = 0.0;
    result.colored[startIndex] = 1;

    // 4 邻域：右、左、下、上
    int dx[4] = {1, -1, 0, 0};
    int dy[4] = {0, 0, 1, -1};

    while (openList.size() > 0) {
        // 从 OpenList 里找出 f 最小的那个
        int best = 0;
        for (int i = 1; i < (int)openList.size(); i++) {
            if (calc_f(openList[i]) < calc_f(openList[best]))
                best = i;
        }

        Node now = openList[best];
        openList.erase(openList.begin() + best);

        int nowIndex = map.index(now.x, now.y);
        result.colored[nowIndex] = 2;    // 放进 CloseList
        result.expanded = result.expanded + 1;

        // 拿出来的正好是终点，结束
        if (now.x == goal.x && now.y == goal.y) {
            result.found = true;
            result.g = now.g;
            break;
        }

        for (int i = 0; i < 4; i++) {
            int nextX = now.x + dx[i];
            int nextY = now.y + dy[i];

            if (!map.free(nextX, nextY))    // 出界或者撞墙
                continue;

            int nextIndex = map.index(nextX, nextY);
            if (result.colored[nextIndex] == 2)    // 已经在 CloseList 里了
                continue;

            // 4 邻域每走一步代价都是 1，所以 g(n) = g(parent) + 1
            double nextG = now.g + 1.0;
            if (nextG >= gBest[nextIndex])    // 不比已经找到的路更短，就不管
                continue;

            gBest[nextIndex] = nextG;
            parent[nextIndex] = nowIndex;

            if (result.colored[nextIndex] == 1) {
                // 这个点已经在 OpenList 里了，把它的 g 和 h 改掉
                for (int j = 0; j < (int)openList.size(); j++) {
                    if (openList[j].x == nextX && openList[j].y == nextY) {
                        openList[j].g = nextG;
                        openList[j].h = calc_h(openList[j], goal);
                        break;
                    }
                }
            } else {
                // 新点，加进 OpenList
                Node child;
                child.x = nextX;
                child.y = nextY;
                child.g = nextG;
                child.h = calc_h(child, goal);
                openList.push_back(child);
                result.colored[nextIndex] = 1;
            }
        }
    }

    // 找到了就顺着 parent 从终点往回想，最后倒过来
    if (result.found) {
        int index = map.index(goal.x, goal.y);
        while (index != -1) {
            Point p;
            p.x = index % map.width;
            p.y = index / map.width;
            result.path.push_back(p);
            index = parent[index];
        }
        std::reverse(result.path.begin(), result.path.end());
    }

    return result;
}
