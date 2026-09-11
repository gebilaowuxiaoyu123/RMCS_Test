#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace nav {

struct GridPoint {
    int x = 0;
    int y = 0;

    bool operator==(const GridPoint& other) const { return x == other.x && y == other.y; }
    bool operator!=(const GridPoint& other) const { return !(*this == other); }
};

class GridMap {
public:
    GridMap(int width, int height);

    static GridMap from_image(const std::string& path, int threshold = 128);

    int width() const { return width_; }
    int height() const { return height_; }
    std::size_t size() const { return cells_.size(); }

    std::size_t index(GridPoint p) const {
        return static_cast<std::size_t>(p.y) * static_cast<std::size_t>(width_)
             + static_cast<std::size_t>(p.x);
    }

    bool inside(GridPoint p) const {
        return p.x >= 0 && p.y >= 0 && p.x < width_ && p.y < height_;
    }
    bool is_free(GridPoint p) const { return inside(p) && cells_[index(p)] == 0; }
    bool is_occupied(GridPoint p) const { return !is_free(p); }

    void set_occupied(GridPoint p, bool occupied = true);
    void fill_rect(GridPoint top_left, int width, int height, bool occupied = true);
    void fill_border(int thickness);
    void fill_random(double obstacle_ratio, unsigned seed);
    void carve_maze(unsigned seed);

    std::vector<GridPoint> free_cells() const;

private:
    int width_;
    int height_;
    std::vector<std::uint8_t> cells_;
};

} // namespace nav
