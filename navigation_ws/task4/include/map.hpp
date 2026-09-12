#pragma once

#include <cstddef>
#include <random>
#include <vector>

struct Point {
    int x = 0;
    int y = 0;

    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
};

class Map {
public:
    Map(int w, int h)
        : width(w)
        , height(h)
        , cells(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0) {}

    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < width && y < height; }

    bool free(int x, int y) const { return inside(x, y) && cells[index(x, y)] == 0; }

    int index(int x, int y) const { return y * width + x; }

    void set_wall(int x, int y) {
        if (inside(x, y))
            cells[index(x, y)] = 1;
    }

    void clear_wall(int x, int y) {
        if (inside(x, y))
            cells[index(x, y)] = 0;
    }

    void fill_wall(int x, int y, int w, int h) {
        for (int j = y; j < y + h; ++j)
            for (int i = x; i < x + w; ++i)
                set_wall(i, j);
    }

    void clear_area(int x, int y, int w, int h) {
        for (int j = y; j < y + h; ++j)
            for (int i = x; i < x + w; ++i)
                clear_wall(i, j);
    }

    void add_border() {
        fill_wall(0, 0, width, 1);
        fill_wall(0, height - 1, width, 1);
        fill_wall(0, 0, 1, height);
        fill_wall(width - 1, 0, 1, height);
    }

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
