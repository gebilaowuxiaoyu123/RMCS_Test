# 任务四 · A* 寻路

## 一、怎么跑

```bash
cd /workspaces/RMCS/navigation_ws/task4
cmake -S . -B build        # 配置，第一次（或改了 CMakeLists）才需要
cmake --build build -j8    # 编译
./build/task4_astar        # 运行
```

**不带参数**，地图我都写在代码里了。跑完 `output/` 下有 5 张图：

```
tiny_5x5.png        5×5 最小地图
regular_12x12.png   12×12 规则地图（三道错开的墙，绕 S 形走）
rooms.png           60×40 房间地图（墙 + 门洞）
random.png          60×40 随机障碍地图
neighbors.png       同一张图 4 邻域 vs 8 邻域对比
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

**图是黑白的**（灰度，没有彩色）：白=空地、黑=障碍、浅灰=搜过的格子、中灰=待办清单、深灰=最后那条路径；起点写个 `S`，终点写个 `G`。图上**不放任何标题和数字**，看着干净；跑的数值只在终端里打印。小地图（5×5、12×12）会画浅灰网格线，方便数格子；大地图不画。

## 二、代码

名字我是照着那份 A\* 资料（h(n)、g(n)、f(n)、OpenList、CloseList、Parent 那套）起的，看代码的时候能直接对上：

| 资料里的写法 | 代码里的名字 |
|---|---|
| `Node(x, y, Parent, g, h)` | `Node`（x / y / g / h）+ `parent[]` |
| h(n)、CalcDeltaHValue | `calc_h()`、`node.h` |
| dg(n)、CalcDeltaGValue | `calc_g()` |
| g(n) | `node.g`、`gBest[]`（每格已知最好的 g） |
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

const double INF = 1e18;                    // 代表"还没到过"
const double ROOT2 = 1.4142135623730951;    // 斜走一步的代价 = 根号2

// 资料里 Node 类：x, y, Parent, g, h。这里 x/y 是坐标，g 是 g(n)，h 是 h(n)
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
    int expanded = 0;                      // 展开（拓展）了多少格，也就是 CloseList 的大小
    double g = 0.0;                        // 终点的 g(n)，也就是路径总代价
    std::vector<Point> path;               // 起点→终点的路径
    std::vector<unsigned char> colored;    // 就是资料里的 Colored 数组，见下面 1 / 2 的含义
};

// dg(n)：从 parent 走到 node 多花多少代价。直走 1，斜走根号2
inline double calc_g(const Node& node, const Node& parent) {
    return (node.x != parent.x && node.y != parent.y) ? ROOT2 : 1.0;
}

// h(n)：估 node 到终点还剩多远。这个值不能高估，否则 A* 就不保证最优了
inline double calc_h(const Node& node, Point goal, bool allowDiagonal) {
    const double dx = std::abs(node.x - goal.x);
    const double dy = std::abs(node.y - goal.y);

    // 只能上下左右走：曼哈顿距离，在这个走法下刚好不高估
    if (!allowDiagonal)
        return dx + dy;

    // 能斜走：先尽量斜走（一步根号2），剩下不够斜的再直走（一步 1）
    return std::max(dx, dy) + (ROOT2 - 1.0) * std::min(dx, dy);
}

// allowDiagonal 为 true 就是 8 邻域（能斜走），false 就只能上下左右
inline Result search(const Map& map, Point start, Point goal, bool allowDiagonal) {
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
    startNode.h = calc_h(startNode, goal, allowDiagonal);

    const int startIndex = map.index(start.x, start.y);
    gBest[startIndex] = 0.0;
    result.colored[startIndex] = 1;
    openList.push(startNode);

    // 八个方向：前 4 个是上下左右，后 4 个是四个斜角
    const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

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

        // 不允许斜走时只看前 4 个方向
        const int count = allowDiagonal ? 8 : 4;
        for (int i = 0; i < count; ++i) {
            const int nextX = now.x + dx[i];
            const int nextY = now.y + dy[i];
            if (!map.free(nextX, nextY))    // 出界或撞墙
                continue;

            const int nextIndex = map.index(nextX, nextY);
            if (result.colored[nextIndex] == 2)     // 已经在 CloseList 里了，不用再看
                continue;

            // 斜走时还要看会不会从两个障碍的缝里穿过去（穿墙角不合法）
            const bool diagonal = dx[i] != 0 && dy[i] != 0;
            if (diagonal && (!map.free(now.x + dx[i], now.y) || !map.free(now.x, now.y + dy[i])))
                continue;

            // g(n) = calc_g(n, parent) + g(parent)：这里 g(parent) 就是 now.g
            Node child{nextX, nextY};
            child.g = now.g + calc_g(child, now);
            if (child.g >= gBest[nextIndex])    // 不比已知的更好，就不用更新
                continue;

            child.h = calc_h(child, goal, allowDiagonal);
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

`src/main.cpp` 就干三件事：造地图、调 `search`、把结果画成 PNG。画图就是每格用 `cv::rectangle` 填一种灰度（看 `colored`：2 画浅灰、1 画中灰），路径用 `cv::polylines` 连成折线，起点终点格子填白、写上 `S` / `G`，别的一律不画。数字改在终端里看：

```bash
[tiny_5x5] 找到路径  expanded=20  points=8  g=7.4
[regular_12x12] 找到路径  expanded=88  points=41  g=41.7
[rooms] 找到路径  expanded=476  points=60  g=71.8
[random] 找到路径  expanded=612  points=69  g=77.9
[rooms / 4-neighbor] 找到路径  expanded=701  points=91  g=90.0
[rooms / 8-neighbor] 找到路径  expanded=476  points=60  g=71.8
```

## 三、结果

四张地图我都跑了，全部找到路径：

| 地图 | 尺寸 | 展开格数 | 路径点数 | 总代价 g |
|---|---|---|---|---|
| tiny_5x5 | 5×5 | 20 | 8 | 7.4 |
| regular_12x12 | 12×12 | 88 | 41 | 41.7 |
| rooms | 60×40 | 476 | 60 | 71.8 |
| random | 60×40 | 612 | 69 | 77.9 |

**5×5 最小地图**：中间竖着一道 1×3 的墙，起点在左上角、终点在右下角，路径得绕过去。代价 7.4，就是 1 步斜走（1.41）加 6 步直走。

![5x5 地图](task4/output/tiny_5x5.png)

**12×12 规则地图**：三道墙错开（右边留口、左边留口、右边留口），所以路径是个 S 形。

![12x12 地图](task4/output/regular_12x12.png)

**60×40 房间地图**：墙把地图切成几个房间，得绕门洞走。浅灰是展开过的格子（CloseList），中灰是还在待办清单里、贴着障碍的那层边（OpenList）。

![rooms 地图](task4/output/rooms.png)

## 四、顺带验证的一件事

同一张 rooms 地图，我只改了"准不准斜走"一个开关：

![4邻域与8邻域](task4/output/neighbors.png)

| 配置 | 展开格数 | 路径点数 | 总代价 g |
|---|---|---|---|
| 4 邻域（只能上下左右） | 701 | 91 | 90.0 |
| 8 邻域（可斜走） | 476 | 60 | 71.8 |

这次两边一比，**8 邻域是又快又短**：展开的格子少了三成，代价从 90 降到 71.8，少了 20%。

4 邻域那个 90 也对得上——从 (2,2) 到 (57,37) 横竖各要走 55+35=90 步，一步代价 1，所以就是 90。8 邻域能斜走，路径点数从 91 掉到 60，也不再是一格一格的锯齿了。所以能斜走就让它斜走。

另外踩了两个坑：一是斜走会穿墙（就是代码里那段 `diagonal` 判断），二是同一个格子会在优先队列里躺好几份（用 `colored[nowIndex] == 2` 跳过过期数据）。顺便一提，`expanded` 以前会重复统计同一格，现在改成按 CloseList 去重，所以 rooms 从 527 变成了 476，路径和代价没变。
