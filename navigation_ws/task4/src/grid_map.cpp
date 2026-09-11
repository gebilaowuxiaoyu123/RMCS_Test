#include "grid_map.hpp"

#include <algorithm>
#include <numeric>
#include <random>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace nav {

GridMap::GridMap(int width, int height)
    : width_(width)
    , height_(height)
    , cells_(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0) {
    if (width <= 0 || height <= 0)
        throw std::invalid_argument("地图尺寸必须为正");
}

GridMap GridMap::from_image(const std::string& path, int threshold) {
    const cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);
    if (image.empty())
        throw std::runtime_error("无法读取地图图片: " + path);

    cv::Mat binary;
    cv::threshold(image, binary, threshold, 255, cv::THRESH_BINARY);

    GridMap map(binary.cols, binary.rows);
    for (int y = 0; y < binary.rows; ++y) {
        for (int x = 0; x < binary.cols; ++x) {
            const auto value = binary.at<std::uint8_t>(y, x);
            map.cells_[map.index(GridPoint{x, y})] = value == 0 ? 1 : 0;
        }
    }
    return map;
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

void GridMap::carve_maze(unsigned seed) {
    std::fill(cells_.begin(), cells_.end(), static_cast<std::uint8_t>(1));

    std::mt19937 generator(seed);
    std::vector<GridPoint> stack;
    stack.push_back(GridPoint{1, 1});
    set_occupied(GridPoint{1, 1}, false);

    const int step[4][2] = {{2, 0}, {-2, 0}, {0, 2}, {0, -2}};

    while (!stack.empty()) {
        const GridPoint current = stack.back();

        int order[4] = {0, 1, 2, 3};
        std::shuffle(order, order + 4, generator);

        bool advanced = false;
        for (int i = 0; i < 4; ++i) {
            const int dx = step[order[i]][0];
            const int dy = step[order[i]][1];
            const GridPoint next{current.x + dx, current.y + dy};
            if (next.x <= 0 || next.y <= 0 || next.x >= width_ - 1 || next.y >= height_ - 1)
                continue;
            if (!is_occupied(next))
                continue;

            set_occupied(GridPoint{current.x + dx / 2, current.y + dy / 2}, false);
            set_occupied(next, false);
            stack.push_back(next);
            advanced = true;
            break;
        }

        if (!advanced)
            stack.pop_back();
    }
}

std::vector<GridPoint> GridMap::free_cells() const {
    std::vector<GridPoint> cells;
    for (int y = 0; y < height_; ++y)
        for (int x = 0; x < width_; ++x)
            if (is_free(GridPoint{x, y}))
                cells.push_back(GridPoint{x, y});
    return cells;
}

} // namespace nav
