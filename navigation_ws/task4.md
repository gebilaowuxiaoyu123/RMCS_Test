# 任务四 · A* 寻路

## 一、怎么跑

```bash
cd /workspaces/RMCS/navigation_ws/task4
cmake -S . -B build        # 配置，第一次（或改了 CMakeLists）才需要
cmake --build build -j8    # 编译
./build/task4_astar        # 运行
```

**不带任何参数**，地图写在代码里。跑完 `output/` 下会有 5 张图：

```
tiny_5x5.png        5×5 最小地图
regular_12x12.png   12×12 规则地图（三道错开的墙，绕 S 形走）
rooms.png           60×40 房间地图（墙 + 门洞）
random.png          60×40 随机障碍地图
neighbors.png       同一张图 4 邻域 vs 8 邻域对比
```

改地图就改 `src/main.cpp` 的 `build_maps()`：

```cpp
GridMap tiny(5, 5);                        // 建一张 5×5 的图
tiny.fill_rect(GridPoint{2, 1}, 1, 3);     // 从 (2,1) 开始画一道 1×3 的竖墙
maps.push_back({"tiny_5x5", std::move(tiny), {0, 0}, {4, 4}});
//               ↑名字                        ↑起点     ↑终点
```

代码分三层：`grid_map`（地图）管"哪格能走"，`astar`（算法）管"怎么找路"，`main` 管造地图和画图。三层互不知道对方内部实现。

## 二、代码

### include/astar.hpp

```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "grid_map.hpp"

namespace nav {

// 搜索参数，就两项
struct AStarOptions {
    bool allow_diagonal = true;          // 能不能斜着走（8 邻域）；false 就只能上下左右
    bool prevent_corner_cutting = true;  // 禁止从两个障碍的夹角里斜穿过去
};

// 统计量：衡量搜得快不快、路径短不短
struct SearchStats {
    bool found = false;           // 找到没有
    std::size_t expanded = 0;     // 展开了多少格（越小越快）
    std::size_t path_points = 0;  // 路径上有多少格
    double path_cost = 0.0;       // 路径总代价（越小越好）
};

// 一次搜索的产出
struct SearchResult {
    std::vector<GridPoint> path;      // 起点→终点的路径
    std::vector<std::uint8_t> state;  // 每格状态：0 没碰过 / 1 在待办清单里 / 2 已展开
    SearchStats stats;
};

// 算两个格子之间的启发值。按能不能斜走自动选公式
double heuristic_cost(GridPoint from, GridPoint to, bool allow_diagonal);

SearchResult astar(
    const GridMap& map, GridPoint start, GridPoint goal, const AStarOptions& options = {});

} // namespace nav
```

### src/astar.cpp

```cpp
#include "astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace nav {

namespace {

// 用无穷大表示"还没到过这一格"
constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kStraightCost = 1.0;                 // 直着走一步的代价
constexpr double kDiagonalCost = 1.4142135623730951;  // 斜着走一步的代价 = 根号2

// 优先队列里存的东西。f 决定谁先被展开
struct QueueNode {
    double f = 0.0;
    double g = 0.0;
    int index = -1;  // 格子在一维数组里的下标

    // 小顶堆的比较规则：f 小的排前面；
    // f 相同时 g 大的排前面（g 大说明离终点近，先展开它能更早撞到终点，能省不少无用展开）
    bool operator>(const QueueNode& other) const {
        if (f != other.f)
            return f > other.f;
        return g < other.g;
    }
};

// 斜走时检查会不会"从两个障碍的缝里穿过去"。
// 比如正上方和正左方都是墙还往左上斜穿，等于穿墙角，几何上不合法。
bool diagonal_allowed(
    const GridMap& map, GridPoint from, GridPoint to, bool prevent_corner_cutting) {
    if (!prevent_corner_cutting)
        return true;
    return map.is_free(GridPoint{to.x, from.y}) && map.is_free(GridPoint{from.x, to.y});
}

} // namespace

// 启发函数：估"从 from 到 to 还剩多远"。绝对不能高估，否则 A* 的最优性就没了
double heuristic_cost(GridPoint from, GridPoint to, bool allow_diagonal) {
    // 取绝对值，因为往哪个方向不重要，只关心差多远
    const double dx = std::abs(static_cast<double>(to.x - from.x));
    const double dy = std::abs(static_cast<double>(to.y - from.y));

    // 只能上下左右走：曼哈顿距离，在这个走法下刚好不高估
    if (!allow_diagonal)
        return dx + dy;

    // 能斜着走：先尽量斜走（一步根号2），剩下不够斜的再直走（一步 1）
    return kStraightCost * std::max(dx, dy) + (kDiagonalCost - kStraightCost) * std::min(dx, dy);
}

SearchResult astar(const GridMap& map, GridPoint start, GridPoint goal, const AStarOptions& options) {
    // 起终点本身就在墙里的话直接报错，免得后面算出莫名其妙的结果
    if (!map.is_free(start) || !map.is_free(goal))
        throw std::invalid_argument("起点或终点落在障碍上");

    SearchResult result;
    result.state.assign(map.size(), 0);  // 一开始所有格子都是"没碰过"

    // g_score[i] = 从起点到第 i 格目前找到的最小代价，初始都是无穷大
    std::vector<double> g_score(map.size(), kInfinity);
    // parent[i] = 第 i 格是从哪一格走过来的，最后靠它回推整条路径
    std::vector<int> parent(map.size(), -1);

    const int start_index = static_cast<int>(map.index(start));
    const int goal_index = static_cast<int>(map.index(goal));

    // 小顶堆当"待办清单"，f 最小的先出队
    std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> open;

    // 起点入队：g = 0，所以 f 就是它到终点的估计值
    g_score[static_cast<std::size_t>(start_index)] = 0.0;
    open.push(QueueNode{heuristic_cost(start, goal, options.allow_diagonal), 0.0, start_index});
    result.state[static_cast<std::size_t>(start_index)] = 1;  // 标记成"在待办清单里"

    // 8 个方向：前 4 个是上下左右，后 4 个是四个斜角
    const int step[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    bool found = false;

    while (!open.empty()) {
        const QueueNode current = open.top();
        open.pop();

        const auto current_index = static_cast<std::size_t>(current.index);

        // 惰性删除：同一格被改进多次时会在堆里躺好几份。
        // 这份的 g 比记录里的大就说明是过期数据，跳过。比从堆里精确删元素简单得多。
        if (current.g > g_score[current_index])
            continue;

        result.state[current_index] = 2;  // 标记成"已展开"
        ++result.stats.expanded;

        // 拿出来的正好是终点，收工
        if (current.index == goal_index) {
            found = true;
            break;
        }

        // 一维下标换回二维坐标
        const GridPoint from{
            static_cast<int>(current_index % static_cast<std::size_t>(map.width())),
            static_cast<int>(current_index / static_cast<std::size_t>(map.width()))};

        // 不允许斜走时只看前 4 个方向
        const int neighbour_count = options.allow_diagonal ? 8 : 4;
        for (int i = 0; i < neighbour_count; ++i) {
            const GridPoint to{from.x + step[i][0], from.y + step[i][1]};

            if (!map.is_free(to))  // 出界或撞墙
                continue;

            // 斜走还要额外检查能不能穿墙角
            const bool diagonal = step[i][0] != 0 && step[i][1] != 0;
            if (diagonal && !diagonal_allowed(map, from, to, options.prevent_corner_cutting))
                continue;

            // 从当前格走到这一格，一共要花多少
            const double tentative =
                g_score[current_index] + (diagonal ? kDiagonalCost : kStraightCost);

            const auto to_index = map.index(to);

            // 这条路不比已经知道的更好，就不用更新
            if (tentative >= g_score[to_index])
                continue;

            // 记下更好的走法
            g_score[to_index] = tentative;
            parent[to_index] = current.index;
            result.state[to_index] = 1;
            open.push(QueueNode{
                tentative + heuristic_cost(to, goal, options.allow_diagonal), tentative,
                static_cast<int>(to_index)});
        }
    }

    result.stats.found = found;

    // 找到就顺着 parent 从终点往回走，走完翻转过来就是"起点→终点"
    if (found) {
        int index = goal_index;
        while (index != -1) {
            result.path.push_back(GridPoint{index % map.width(), index / map.width()});
            index = parent[static_cast<std::size_t>(index)];
        }
        std::reverse(result.path.begin(), result.path.end());
        result.stats.path_points = result.path.size();
        result.stats.path_cost = g_score[static_cast<std::size_t>(goal_index)];
    }

    return result;
}

} // namespace nav
```

图上颜色对应：空地白、障碍深灰、已展开浅蓝、待办清单浅橙、最终路径红线、起点绿、终点蓝。格子够大时（≥12 像素）会画上浅灰网格线，方便直接数格子。

## 三、结果

四张地图全部找到路径：

| 地图 | 尺寸 | 展开格数 | 路径点数 | 路径代价 |
|---|---|---|---|---|
| tiny_5x5 | 5×5 | 12 | 8 | 7.4 |
| regular_12x12 | 12×12 | 88 | 41 | 41.7 |
| rooms | 60×40 | 559 | 60 | 71.8 |
| random | 60×40 | 609 | 69 | 77.9 |

**5×5 最小地图**：中间竖着一道 1×3 的墙，起点在左上角、终点在右下角，路径得绕过去。展开 12 格，路径 8 个点、代价 7.4（1 步斜走 1.41 + 6 步直走 6）。

![5x5 地图](task4/output/tiny_5x5.png)

**12×12 规则地图**：三道墙错开（右边留口、左边留口、右边留口），所以路径是个 S 形。展开 88 格。

![12x12 地图](task4/output/regular_12x12.png)

**60×40 房间地图**：墙把地图切成几个房间，得绕门洞走。浅蓝是搜过的格子，浅橙是待办清单贴着障碍的那层边。

![rooms 地图](task4/output/rooms.png)

## 四、顺带验证的一件事：4 邻域 vs 8 邻域

同一张 rooms 地图，只改"准不准斜走"一个开关：

![4邻域与8邻域](task4/output/neighbors.png)

| 配置 | 展开格数 | 路径点数 | 路径代价 |
|---|---|---|---|
| 4 邻域（只能上下左右） | 91 | 91 | 90.0 |
| 8 邻域（可斜走） | 559 | 60 | 71.8 |

- **4 邻域代价正好 90**：从 (2,2) 到 (57,37) 横竖各要走 55+35=90 步，一步代价 1，所以就是 90。8 邻域能斜走，代价降到 71.8，**少了 20%**，路径点数也从 91 降到 60（不再是一格一格的锯齿）。
- **展开格数反而是 8 邻域多**（559 vs 91），这不是"8 邻域更慢"：4 邻域的路径贴着地图上边和右边走，能搜的区域本来就窄；8 邻域是斜着穿过去，搜索面是宽的一条带，碰到的格子自然多。**展开数和路径代价要一起看，单看一个会得出错误结论。**

另外踩了两个坑：一是斜走会穿墙（加了 `diagonal_allowed` 检查），二是同一格子会在优先队列里躺好几份（用惰性删除跳过过期数据）。
