# 任务四 · A* 寻路

## 一、怎么跑起来（三条命令）

```bash
cd /workspaces/RMCS/navigation_ws/task4

cmake -S . -B build        # 配置，第一次（或改了 CMakeLists）才需要
cmake --build build -j8    # 编译
./build/task4_astar        # 运行
```

**运行不需要任何参数**，因为地图是在代码里造好的。跑完终端会打印每个地图的搜索统计，结果图全部落在 `output/` 目录：

```
output/rooms.png        ← 房间地图的最终路径
output/random.png       ← 随机障碍地图的最终路径
output/neighbors.png    ← 同一张图：4 邻域 vs 8 邻域对比
```

想改地图，就改 `src/main.cpp` 里的 `build_maps()` 函数：

```cpp
GridMap rooms(60, 40);                    // 建一张 60×40 的图
rooms.fill_border(1);                     // 四周加 1 格厚的边墙
rooms.fill_rect(GridPoint{1, 12}, 20, 1); // 从 (1,12) 开始画 20×1 的墙
maps.push_back(MapCase{"rooms", std::move(rooms), GridPoint{2, 2}, GridPoint{57, 37}});
//                      ↑名字                      ↑起点            ↑终点
```

## 二、目录结构

```
navigation_ws/
├── Hybrid_Astar_for_Navigation/   ← 参考模板（第三方克隆，未入库）
└── task4/
    ├── CMakeLists.txt
    ├── include/grid_map.hpp         地图的对外接口
    ├── src/grid_map.cpp             地图实现
    ├── include/astar.hpp            A* 的对外接口
    ├── src/astar.cpp                A* 实现
    ├── src/main.cpp                 造地图 + 跑搜索 + 画图
    └── output/                      结果图
```

分三层，各管各的，互相不知道对方的内部实现：

| 文件 | 只管什么 | 不管什么 |
|---|---|---|
| `grid_map` | 地图多大、某格能不能走、怎么造墙 | 怎么搜索、怎么画图 |
| `astar` | 怎么找路 | 地图长什么样、图怎么画 |
| `main` | 造地图、调算法、渲染 PNG | 搜索细节 |

这样换渲染方式不用动算法，换算法不用动地图。

## 三、A* 在干什么（大白话）

**地图**是一张格子图，每格要么空地要么障碍。

**核心公式**：

$$f(n) = g(n) + h(n)$$

- $g(n)$：从**起点**走到格子 $n$ 已经花掉的实际代价
- $h(n)$：从格子 $n$ 到**终点**大概还要多远（估计值，所以叫"启发函数"）
- $f(n)$：这一格"总的看起来要花多少"

**流程就三句话**：

1. 起点进"待办清单"；
2. 每次从清单里挑 **f 最小**的格子展开——看它的邻居，算出邻居的 $g/h/f$，放进清单；
3. 直到拿出来的正好是终点，顺着"我是从哪来的"回推，就得到路径。

**为什么能保证最短路**：A* 相当于"有方向感的 Dijkstra"。Dijkstra 只看 $g$（已经走了多远），所以像水波一样四面八方铺开；A* 多了 $h$，等于在说"我更想往终点那个方向铺"，于是铺得集中得多。只要 $h$ **不高估**真实剩余距离，找到的路径就一定是最优的。

**这就是为什么启发函数要按走法来选**：

- 只允许上下左右走（4 邻域）→ 用**曼哈顿距离** $|dx|+|dy|$
- 允许斜着走（8 邻域）→ 必须用 **Octile**：$\max(dx,dy) + (\sqrt2-1)\min(dx,dy)$

因为斜走一步的代价是 $\sqrt2 \approx 1.414$ 而不是 2。8 邻域下如果还用曼哈顿，等于把斜走当成 2 步算，**高估**了剩余距离，A* 就不再有最优保证了。我在代码里就是按 `allow_diagonal` 自动切换这两个公式的。

## 四、核心代码

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
project(task4_astar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)        # 默认 Release，搜索跑得快
endif()

find_package(OpenCV REQUIRED)            # 画图用

add_executable(task4_astar
    src/main.cpp
    src/grid_map.cpp
    src/astar.cpp
)

target_include_directories(task4_astar PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(task4_astar PRIVATE ${OpenCV_LIBS})
```

### include/astar.hpp

```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "grid_map.hpp"

namespace nav {

// 搜索的可调参数，就两项，够用了
struct AStarOptions {
    bool allow_diagonal = true;          // 允许斜着走（8 邻域）；false 就只能上下左右
    bool prevent_corner_cutting = true;  // 禁止从两个障碍的夹角里斜穿过去
};

// 搜索统计，用来衡量搜得快不快、路径短不短
struct SearchStats {
    bool found = false;           // 找到没有
    std::size_t expanded = 0;     // 展开了多少格（越小越快）
    std::size_t path_points = 0;  // 路径上有多少格
    double path_cost = 0.0;       // 路径总代价（越小越好）
};

// 一次搜索的产出
struct SearchResult {
    std::vector<GridPoint> path;        // 起点→终点的路径
    std::vector<std::uint8_t> state;    // 每格状态：0 没碰过 / 1 在待办清单里 / 2 已展开
    SearchStats stats;
};

// 算一对格子之间的启发值。按 allow_diagonal 自动选曼哈顿还是 Octile
double heuristic_cost(GridPoint from, GridPoint to, bool allow_diagonal);

// 主搜索。options 有默认值，所以只传地图和起终点也能用
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
constexpr double kStraightCost = 1.0;                 // 直走一步的代价
constexpr double kDiagonalCost = 1.4142135623730951;  // 斜走一步的代价 = 根号2

// 优先队列里存的东西。f 决定谁先被展开；g 用来在 f 相同时做取舍
struct QueueNode {
    double f = 0.0;
    double g = 0.0;
    int index = -1;  // 格子在一维数组里的下标

    // 小顶堆的比较规则：f 小的排前面；
    // f 相同时 g 大的排前面（g 大说明离终点近，先展开它能更早撞到终点，省不少无用展开）
    bool operator>(const QueueNode& other) const {
        if (f != other.f)
            return f > other.f;
        return g < other.g;
    }
};

// 斜走时检查会不会"从两个障碍之间的缝里穿过去"。
// 比如正上方和正左方都是墙，还往左上斜穿，就等于穿墙角，几何上不合法，机器人也过不去。
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

    // 能斜着走：先尽量斜走（一步根号2），剩下不够斜的再直走（一步1）
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

    // 8 个方向的偏移量：前 4 个是上下左右，后 4 个是四个斜角
    const int step[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    bool found = false;

    while (!open.empty()) {
        const QueueNode current = open.top();
        open.pop();

        const auto current_index = static_cast<std::size_t>(current.index);

        // 惰性删除：同一格被改进多次时会在堆里躺好几份。
        // 如果这份的 g 比记录里的大，说明是过期数据，跳过就行。
        // 比"从堆里精确删元素"简单得多，代价只是堆稍微大一点。
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

            // 如果从当前格走到这一格，一共要花多少
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

    // 找到就顺着 parent 从终点往回走，走完翻转过来就是"起点→终点"的顺序
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

`grid_map`（地图怎么造）和 `main.cpp`（结果怎么画）源码都在 `task4/` 里，逻辑很直白：`main.cpp` 就是把每格按状态刷成不同颜色——空地白、障碍深灰、已展开浅蓝、待办里浅橙、最终路径红线、起点绿、终点蓝。

## 五、测试结果

两张地图都找到了路径：

| 地图 | 尺寸 | 展开格数 | 路径点数 | 路径代价 |
|---|---|---|---|---|
| rooms（墙 + 门洞） | 60×40 | 559 | 60 | 71.8 |
| random（随机障碍 28%） | 60×40 | 609 | 69 | 77.9 |

### rooms 地图

![rooms 结果](task4/output/rooms.png)

浅蓝是搜索过的格子，浅橙是待办清单的边缘（边界贴着障碍的那层），红线就是最终路径。可以看到搜索区域明显贴着墙、并朝着终点方向收窄——这就是 $h$ 在起作用。

### random 地图

![random 结果](task4/output/random.png)

障碍散乱，路径绕得更多，展开格数也更多。

### 4 邻域 vs 8 邻域

同一张 rooms 地图，只改"准不准斜走"这一个开关：

![4邻域与8邻域对比](task4/output/neighbors.png)

| 配置 | 展开格数 | 路径点数 | 路径代价 |
|---|---|---|---|
| 4 邻域（只能上下左右） | 91 | 91 | 90.0 |
| 8 邻域（可斜走） | 559 | 60 | 71.8 |

**怎么读这组数**：

- 4 邻域只能横竖走，从 (2,2) 到 (57,37) 至少要 55+35=90 步，所以代价正好是 90；8 邻域能斜走，代价降到 71.8，少了 20%。**所以能斜走就一定要斜走**，路径短得多、也不会走出那种"楼梯状"的锯齿路径（路径点数从 91 降到 60 就是这个原因）。
- 展开格数反而是 8 邻域更多（559 vs 91）——这不是"8 邻域更慢"，而是两者的搜索形状不一样：4 邻域的路径贴着上边和右边缘走，能搜的区域本来就窄；8 邻域是斜着穿过去的，搜索面是宽的一条带，自然碰到的格子多。**展开数和路径代价要一起看，不能只看一个**。

## 六、踩到的坑

**坑一：斜走会穿墙。**
最开始的版本没做检查，路径会从两个斜对角障碍的缝里穿过去——几何上不合法，实车也走不了。加了 `diagonal_allowed()`（斜走时要求两个相邻方向也都是空地）就正常了。

**坑二：同一个格子在优先队列里会躺好几份。**
发现更好走法时会重新入队，但旧的那份还在堆里。不管它的话会重复展开、统计数字也虚高。做法是"惰性删除"：出队时比一下 $g$，比记录里大就说明是过期数据，直接跳过。

## 七、和参考模板的关系

参考的 `Hybrid_Astar_for_Navigation` 是 Hybrid A*（给车用的，带朝向、不能原地转向），本任务要求的是**栅格 A\***，所以算法是我自己写的，没有照搬它。借的是它的**工程拆分思路**：地图、算法、绘制分开，别把搜索逻辑和画图混在一起。

它用的是交互式可视化（鼠标点起终点、回车算路径），这在没有桌面的容器里跑不了，所以我换成了"地图自动生成 + 结果自动存成 PNG"。
