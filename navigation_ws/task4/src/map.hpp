#pragma once

#include <cstdlib>
#include <vector>

// 一个格子的坐标
struct Point {
    int x = 0;
    int y = 0;
};

// 一张格子地图，cells 里 0 是空地、1 是墙
class Map {
public:
    Map(int w, int h) {
        width = w;
        height = h;
        cells = std::vector<unsigned char>(w * h, 0);
    }

    // 坐标在不在图里
    bool inside(int x, int y) const {
        if (x < 0 || y < 0 || x >= width || y >= height)
            return false;
        return true;
    }

    // 这一格能不能走（在界内 并且 不是墙）
    bool free(int x, int y) const {
        if (!inside(x, y))
            return false;
        if (cells[index(x, y)] == 1)
            return false;
        return true;
    }

    // 二维坐标转成一维下标
    int index(int x, int y) const {
        return y * width + x;
    }

    void set_wall(int x, int y) {
        if (inside(x, y))
            cells[index(x, y)] = 1;
    }

    void clear_wall(int x, int y) {
        if (inside(x, y))
            cells[index(x, y)] = 0;
    }

    // 从 (x,y) 开始铺一块 w 宽 h 高的墙
    void fill_wall(int x, int y, int w, int h) {
        for (int j = y; j < y + h; j++) {
            for (int i = x; i < x + w; i++) {
                set_wall(i, j);
            }
        }
    }

    // 从 (x,y) 开始清出一块 w 宽 h 高的空地
    void clear_area(int x, int y, int w, int h) {
        for (int j = y; j < y + h; j++) {
            for (int i = x; i < x + w; i++) {
                clear_wall(i, j);
            }
        }
    }

    // 四周糊一圈墙
    void add_border() {
        fill_wall(0, 0, width, 1);
        fill_wall(0, height - 1, width, 1);
        fill_wall(0, 0, 1, height);
        fill_wall(width - 1, 0, 1, height);
    }

    // 按比例随机撒墙，seed 固定所以每次跑出来一样
    void random_walls(double ratio, unsigned seed) {
        srand(seed);
        for (int i = 0; i < width * height; i++) {
            int r = rand() % 100;
            if (r < ratio * 100)
                cells[i] = 1;
            else
                cells[i] = 0;
        }
    }

    int width = 0;
    int height = 0;
    std::vector<unsigned char> cells;
};
