#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "astar.hpp"
#include "grid_map.hpp"

namespace {

using nav::AStarOptions;
using nav::GridMap;
using nav::GridPoint;
using nav::SearchResult;

const cv::Scalar kFreeColor(255, 255, 255);
const cv::Scalar kOccupiedColor(70, 70, 70);
const cv::Scalar kClosedColor(235, 206, 135);
const cv::Scalar kOpenColor(170, 220, 255);
const cv::Scalar kPathColor(50, 40, 240);
const cv::Scalar kStartColor(60, 180, 75);
const cv::Scalar kGoalColor(230, 120, 60);

void put_label(cv::Mat& image, const std::string& text, int row) {
    const cv::Point origin{8, 22 + row * 20};
    cv::putText(
        image, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
    cv::putText(
        image, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2,
        cv::LINE_AA);
}

cv::Mat render(
    const GridMap& map, const SearchResult& result, GridPoint start, GridPoint goal,
    const std::vector<std::string>& labels) {

    const int cell = std::max(6, 480 / map.width());
    cv::Mat image(map.height() * cell, map.width() * cell, CV_8UC3, kFreeColor);

    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            const GridPoint point{x, y};
            cv::Scalar color = kFreeColor;
            const auto cell_state = result.state[map.index(point)];
            if (!map.is_free(point))
                color = kOccupiedColor;
            else if (cell_state == 2)
                color = kClosedColor;
            else if (cell_state == 1)
                color = kOpenColor;
            cv::rectangle(
                image, cv::Rect(x * cell, y * cell, cell, cell), color, cv::FILLED);
        }
    }

    if (!result.path.empty()) {
        std::vector<cv::Point> centers;
        centers.reserve(result.path.size());
        for (const auto& point : result.path)
            centers.emplace_back(point.x * cell + cell / 2, point.y * cell + cell / 2);
        cv::polylines(image, centers, false, kPathColor, 2, cv::LINE_AA);
    }

    const auto marker = [&](GridPoint point, const cv::Scalar& color) {
        cv::rectangle(image, cv::Rect(point.x * cell, point.y * cell, cell, cell), color, cv::FILLED);
    };
    marker(start, kStartColor);
    marker(goal, kGoalColor);

    for (std::size_t i = 0; i < labels.size(); ++i)
        put_label(image, labels[i], static_cast<int>(i));

    return image;
}

cv::Mat stack_row(const std::vector<cv::Mat>& panels, int padding = 12) {
    const int height = std::max_element(
                           panels.begin(), panels.end(),
                           [](const cv::Mat& a, const cv::Mat& b) { return a.rows < b.rows; })
                           ->rows;
    cv::Mat row;
    for (std::size_t i = 0; i < panels.size(); ++i) {
        cv::Mat panel = panels[i];
        if (panel.rows != height)
            cv::copyMakeBorder(
                panel, panel, 0, height - panel.rows, 0, 0, cv::BORDER_CONSTANT,
                cv::Scalar(235, 235, 235));
        if (i == 0) {
            row = panel.clone();
            continue;
        }
        cv::Mat padded;
        cv::copyMakeBorder(
            panel, padded, 0, 0, padding, 0, cv::BORDER_CONSTANT, cv::Scalar(235, 235, 235));
        cv::hconcat(row, padded, row);
    }
    return row;
}

struct MapCase {
    std::string name;
    GridMap map;
    GridPoint start;
    GridPoint goal;
};

std::vector<MapCase> build_maps() {
    std::vector<MapCase> maps;

    GridMap tiny(5, 5);
    tiny.fill_border(1);
    tiny.set_occupied(GridPoint{2, 2});
    maps.push_back(MapCase{"tiny_5x5", std::move(tiny), GridPoint{1, 1}, GridPoint{3, 3}});

    GridMap regular(12, 12);
    regular.fill_border(1);
    regular.fill_rect(GridPoint{1, 3}, 9, 1);
    regular.fill_rect(GridPoint{2, 6}, 9, 1);
    regular.fill_rect(GridPoint{1, 9}, 9, 1);
    maps.push_back(
        MapCase{"regular_12x12", std::move(regular), GridPoint{1, 1}, GridPoint{10, 10}});

    GridMap rooms(60, 40);
    rooms.fill_border(1);
    rooms.fill_rect(GridPoint{1, 12}, 20, 1);
    rooms.fill_rect(GridPoint{28, 12}, 20, 1);
    rooms.fill_rect(GridPoint{1, 26}, 15, 1);
    rooms.fill_rect(GridPoint{22, 26}, 25, 1);
    rooms.fill_rect(GridPoint{20, 1}, 1, 8);
    rooms.fill_rect(GridPoint{40, 18}, 1, 8);
    rooms.fill_rect(GridPoint{35, 27}, 1, 12);
    rooms.fill_rect(GridPoint{8, 20}, 10, 1);
    maps.push_back(MapCase{"rooms", std::move(rooms), GridPoint{2, 2}, GridPoint{57, 37}});

    GridMap random_map(60, 40);
    random_map.fill_random(0.28, 2026);
    random_map.fill_border(1);
    random_map.fill_rect(GridPoint{1, 1}, 4, 4, false);
    random_map.fill_rect(GridPoint{55, 35}, 4, 4, false);
    maps.push_back(MapCase{"random", std::move(random_map), GridPoint{2, 2}, GridPoint{57, 37}});

    return maps;
}

std::string stats_text(const SearchResult& result) {
    std::ostringstream text;
    text << "expanded=" << result.stats.expanded << "  points=" << result.stats.path_points
         << std::fixed << std::setprecision(1) << "  cost=" << result.stats.path_cost;
    return text.str();
}

void report(const std::string& name, const SearchResult& result) {
    std::cout << "[" << name << "] " << (result.stats.found ? "找到路径" : "未找到路径") << "  "
              << stats_text(result) << '\n';
}

} // namespace

int main() {
    const std::string output_dir = "output/";

    for (auto& item : build_maps()) {
        const AStarOptions options;
        const SearchResult result = nav::astar(item.map, item.start, item.goal, options);
        report(item.name, result);

        cv::Mat view = render(
            item.map, result, item.start, item.goal,
            {"A* 8-neighbor", stats_text(result)});
        cv::imwrite(output_dir + item.name + ".png", view);
    }

    const std::vector<MapCase> maps = build_maps();
    const MapCase& rooms = maps[2];

    std::vector<cv::Mat> panels;
    for (const bool allow_diagonal : {false, true}) {
        AStarOptions options;
        options.allow_diagonal = allow_diagonal;

        const SearchResult result = nav::astar(rooms.map, rooms.start, rooms.goal, options);
        const std::string name = allow_diagonal ? "rooms / 8-neighbor" : "rooms / 4-neighbor";
        report(name, result);

        panels.push_back(render(rooms.map, result, rooms.start, rooms.goal, {name, stats_text(result)}));
    }
    cv::imwrite(output_dir + "neighbors.png", stack_row(panels));

    std::cout << "结果图已写入 " << output_dir
              << "{tiny_5x5,regular_12x12,rooms,random,neighbors}.png\n";
    return 0;
}
