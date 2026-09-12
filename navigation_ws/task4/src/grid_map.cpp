#include "grid_map.hpp"

#include <random>
#include <stdexcept>

namespace nav {

GridMap::GridMap(int width, int height)
    : width_(width)
    , height_(height)
    , cells_(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0) {
    if (width <= 0 || height <= 0)
        throw std::invalid_argument("地图尺寸必须为正");
}

void GridMap::set_occupied(GridPoint p, bool occupied) {
    if (!inside(p))
        return;
    cells_[index(p)] = occupied ? 1 : 0;
}

void GridMap::fill_rect(GridPoint top_left, int width, int height, bool occupied) {
    for (int y = top_left.y; y < top_left.y + height; ++y)
        for (int x = top_left.x; x < top_left.x + width; ++x)
            set_occupied(GridPoint{x, y}, occupied);
}

void GridMap::fill_border(int thickness) {
    fill_rect(GridPoint{0, 0}, width_, thickness);
    fill_rect(GridPoint{0, height_ - thickness}, width_, thickness);
    fill_rect(GridPoint{0, 0}, thickness, height_);
    fill_rect(GridPoint{width_ - thickness, 0}, thickness, height_);
}

void GridMap::fill_random(double obstacle_ratio, unsigned seed) {
    std::mt19937 generator(seed);
    std::bernoulli_distribution distribution(obstacle_ratio);
    for (auto& cell : cells_)
        cell = distribution(generator) ? 1 : 0;
}

} // namespace nav
