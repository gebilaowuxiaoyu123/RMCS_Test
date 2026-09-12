# 任务四 · A* 寻路

## 一、怎么跑

```bash
cd /workspaces/RMCS/navigation_ws/task4
cmake -S . -B build        # 配置，第一次（或改了 CMakeLists）才需要
cmake --build build -j8    # 编译
./build/task4_astar        # 运行
```

**不带参数**，地图我都写在代码里了。跑完 `output/` 下有 4 张图：

```
tiny_5x5.png        5×5 最小地图
regular_12x12.png   12×12 规则地图（三道错开的墙，绕 S 形走）
rooms.png           60×40 房间地图（墙 + 门洞）
random.png          60×40 随机障碍地图
```

代码就 3 个文件，都放在 `src/` 下，各管各的：

```
task4/
└── src/
    ├── map.hpp      地图：图多大、哪格能走、怎么造墙
    ├── astar.hpp    算法：怎么找路（接口和实现都在这一个文件里）
    └── main.cpp     主程序：造地图 + 调算法 + 画图
```

要改地图就改 `src/main.cpp` 的 `make_maps()`：

```cpp
Map tiny(5, 5);                    // 建一张 5×5 的图
tiny.fill_wall(2, 1, 1, 3);        // 从 (2,1) 开始画一道 1×3 的竖墙
maps.push_back(Case{"tiny_5x5", std::move(tiny), Point{0, 0}, Point{4, 4}});
//                 ↑名字                        ↑起点      ↑终点
```

**算法就是最经典的 A\***：4 邻域（只上下左右走，一步一格）、h(n) 用曼哈顿距离、每走一步代价 1。没做斜走那套，也就没有八方向距离和"穿墙角"要处理，代码短很多。

**图的样式是照那份资料里的格子图来的**：没搜过的空地浅灰、展开过的格子（CloseList）刷淡黄、还在待办清单里的（OpenList）刷稍深一点的黄、障碍近黑，最后那条路径**一格一格填成绿色**；每格都有一道浅灰边框，起点终点格子填白写个 `S` / `G`。**不画旗子和叉**，图上也不放标题和数字，看着干净；跑的数值只在终端里打印。

## 二、代码

名字我是照着那份 A\* 资料（h(n)、g(n)、f(n)、OpenList、CloseList、Parent 那套）起的，看代码的时候能直接对上：

| 资料里的写法 | 代码里的名字 |
|---|---|
| `Node(x, y, Parent, g, h)` | `Node`（x / y / g / h）+ `parent[]` |
| h(n)、CalcDeltaHValue | `calc_h()`、`node.h` |
| g(n) | `node.g`、`gBest[]`（每格已知最好的 g） |
| dg(n)、CalcDeltaGValue | 每步 `+ 1.0`（4 邻域一步一格，代价都一样） |
| f(n) = h(n) + g(n) | `Node::f()` |
| OpenList（优先队列） | `openList` |
| CloseList、Colored 数组 | `colored[]`（0 没碰过 / 1 在 OpenList / 2 在 CloseList） |
| Parent | `parent[]` |
| Hash 函数 `x * Size.Y + y` | `Map::index()` |
| `Search(start, end)` | `search()` |

### src/map.hpp

```cpp
#pragma once

#include <cstddef>
#include <random>
#include <vector>

// 一个格子坐标
struct Point {
    int x = 0;
    int y = 0;

    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
};

// 一张栅格地图。cells 里 0 是空地、1 是墙，按行存成一维数组
class Map {
public:
    Map(int w, int h)
        : width(w)
        , height(h)
        , cells(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0) {}

    // 坐标在不在图里
    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < width && y < height; }

    // 这一格能不能走（在界内 且 不是墙）
    bool free(int x, int y) const { return inside(x, y) && cells[index(x, y)] == 0; }

    // 二维坐标转成一维下标，就是资料里 2.1.6 说的那个 hash：y * width + x
    int index(int x, int y) const { return y * width + x; }

    void set_wall(int x, int y) {
        if (inside(x, y))
            cells[index(x, y)] = 1;
    }

    void clear_wall(int x, int y) {
        if (inside(x, y))
            cells[index(x, y)] = 0;
    }

    // 从 (x,y) 开始铺一块 w×h 的墙
    void fill_wall(int x, int y, int w, int h) {
        for (int j = y; j < y + h; ++j)
            for (int i = x; i < x + w; ++i)
                set_wall(i, j);
    }

    // 从 (x,y) 开始清出一块 w×h 的空地
    void clear_area(int x, int y, int w, int h) {
        for (int j = y; j < y + h; ++j)
            for (int i = x; i < x + w; ++i)
                clear_wall(i, j);
    }

    // 沿四周糊一圈墙
    void add_border() {
        fill_wall(0, 0, width, 1);
        fill_wall(0, height - 1, width, 1);
        fill_wall(0, 0, 1, height);
        fill_wall(width - 1, 0, 1, height);
    }

    // 按比例随机撒墙，seed 固定所以每次跑出来一样
    void random_walls(double ratio, unsigned seed) {
        std::mt19937 generator(seed);
        std::bernoulli_distribution distribution(ratio);
        for (auto& cell : cells)
            cell = distribution(generator) ? 1 : 0;
    }

    int width;
    int height;
    std::vector<unsigned char> cells;
};
```

### src/astar.hpp

```cpp
#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <vector>

#include "map.hpp"

inline constexpr double INF = 1e18;    // 代表"还没到过"

// 资料里的 Node 类：x, y, Parent, g, h。这里 x/y 是坐标，g 是 g(n)，h 是 h(n)
// f(n) = h(n) + g(n)，出待办清单时按 f 从小到大排
struct Node {
    int x = 0;
    int y = 0;
    double g = 0.0;
    double h = 0.0;

    double f() const { return g + h; }

    bool operator>(const Node& other) const { return f() > other.f(); }
};

// 一次寻路的结果
struct Result {
    bool found = false;                    // 找到没有
    int expanded = 0;                      // 展开了多少格，也就是 CloseList 的大小
    double g = 0.0;                        // 终点的 g(n)，也就是路径总代价
    std::vector<Point> path;               // 起点→终点的路径
    std::vector<unsigned char> colored;    // 资料里的 Colored 数组，1 = 在 OpenList，2 = 在 CloseList
};

// h(n)：曼哈顿距离，横着差多少格 + 竖着差多少格。
// 4 邻域下，真实最少要走的就是这么多，所以它永远不高估，A* 就保证能找到最短的
inline double calc_h(const Node& node, Point goal) {
    return std::abs(node.x - goal.x) + std::abs(node.y - goal.y);
}

inline Result search(const Map& map, Point start, Point goal) {
    Result result;
    // colored 数组：0 = 没碰过，1 = 在 OpenList 里，2 = 已出 OpenList 进 CloseList
    result.colored.assign(map.cells.size(), 0);

    // gBest[i] = 第 i 格目前找到的最好的 g(n)，用来判断新路要不要更新
    std::vector<double> gBest(map.cells.size(), INF);
    // parent[i] = 第 i 格的父节点（Parent），最后靠它回推整条路径
    std::vector<int> parent(map.cells.size(), -1);

    // OpenList：优先队列，f 最小的先出来
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;

    Node startNode{start.x, start.y};
    startNode.h = calc_h(startNode, goal);

    const int startIndex = map.index(start.x, start.y);
    gBest[startIndex] = 0.0;
    result.colored[startIndex] = 1;
    openList.push(startNode);

    // 4 邻域：上下左右
    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};

    while (!openList.empty()) {
        const Node now = openList.top();
        openList.pop();

        const int nowIndex = map.index(now.x, now.y);

        // 同一格可能被改进多次、在 OpenList 里躺了好几份。这份已经进过 CloseList，
        // 说明是过期数据，直接跳过。这就是资料里说的"每次从 OpenList 拿点时查一下 Colored"
        if (result.colored[nowIndex] == 2)
            continue;

        result.colored[nowIndex] = 2;   // 进 CloseList
        ++result.expanded;

        // 拿出来的正好是终点，收工
        if (now.x == goal.x && now.y == goal.y) {
            result.found = true;
            result.g = now.g;
            break;
        }

        for (int i = 0; i < 4; ++i) {
            const int nextX = now.x + dx[i];
            const int nextY = now.y + dy[i];
            if (!map.free(nextX, nextY))    // 出界或撞墙
                continue;

            const int nextIndex = map.index(nextX, nextY);
            if (result.colored[nextIndex] == 2)     // 已经在 CloseList 里了，不用再看
                continue;

            // g(n) = calc_g(n, parent) + g(parent)：4 邻域每步都是 1，所以就是 now.g + 1
            Node child{nextX, nextY};
            child.g = now.g + 1.0;
            if (child.g >= gBest[nextIndex])    // 不比已知的更好，就不用更新
                continue;

            child.h = calc_h(child, goal);
            gBest[nextIndex] = child.g;
            parent[nextIndex] = nowIndex;
            result.colored[nextIndex] = 1;
            openList.push(child);
        }
    }

    // 找到就顺着 parent 从终点往回走，走完翻转过来就是"起点→终点"
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
```

`src/main.cpp` 就干三件事：造地图、调 `search`、把结果画成 PNG。画图就是每格先用 `cv::rectangle` 填个底色（看 `colored`：2 淡黄、1 稍深的黄、墙近黑），再把路径上的格子整格填绿，然后补一遍网格线，最后起点终点格子填白、写上 `S` / `G`。数字在终端里看：

```bash
[tiny_5x5] 找到路径  expanded=22  points=9  g=8.0
[regular_12x12] 找到路径  expanded=89  points=45  g=44.0
[rooms] 找到路径  expanded=701  points=91  g=90.0
[random] 找到路径  expanded=740  points=93  g=92.0
```

## 三、结果

四张地图都跑通了：

| 地图 | 尺寸 | 展开格数 | 路径点数 | 总代价 g |
|---|---|---|---|---|
| tiny_5x5 | 5×5 | 22 | 9 | 8.0 |
| regular_12x12 | 12×12 | 89 | 45 | 44.0 |
| rooms | 60×40 | 701 | 91 | 90.0 |
| random | 60×40 | 740 | 93 | 92.0 |

**5×5 最小地图**：中间竖着一道 1×3 的墙，起点左上、终点右下。曼哈顿距离就是 8 格，路径正好 8 步走完，一格没多绕。

![5x5 地图](task4/output/tiny_5x5.png)

**12×12 规则地图**：三道墙错开（右边留口、左边留口、右边留口），只能这么拐着走。起点到终点直线只需要 22 步，被墙逼着绕成了 44 步——正好一倍，因为每次都要"钻门再折回来"。

![12x12 地图](task4/output/regular_12x12.png)

**60×40 房间地图**：墙把地图切成几间房，得绕门洞走。代价 90 正好等于曼哈顿距离 55+35=90，说明这张图虽然要绕门，但没多走一步冤枉路。淡黄是展开过的格子（CloseList），稍深的黄是还在待办清单里、贴着障碍的那层边（OpenList）。

![rooms 地图](task4/output/rooms.png)

**60×40 随机地图**：障碍是按 28% 概率随机撒的，代价 92，比曼哈顿的 90 多了 2 步，就是被挡了一下绕的。

![random 地图](task4/output/random.png)

## 四、按资料 2.1.6 的思路处理的两个地方

**第一，CloseList 用数组涂色，不在链表里查。** 资料里说地图大了以后"查这个点在不在 CloseList 里"很费时间，所以另开一个和地图一样大的数组：在 CloseList 里就是 2，在 OpenList 里就是 1，没碰过就是 0。查一次就是 `colored[map.index(x, y)]`，一步到位，不用遍历。

**第二，OpenList 里的"过期数据"要跳过。** 同一个格子可能会被不同的路走到好几次，每来一次都会往 OpenList 里塞一份，所以堆里会同时躺着同一个格子的好几份。出队的时候先看一眼 `colored`：如果这格已经进过 CloseList（等于 2），说明这份是旧的，直接 `continue`。这样不用去堆里精确删元素，实现简单很多，效果一样。

数据上也能对上：`expanded` 就是 CloseList 的大小（每格只算一次），比如 rooms 是 701 格；换成允许斜走的八邻域本来能少展开不少，但那就不是经典 A* 了，这里没做。
