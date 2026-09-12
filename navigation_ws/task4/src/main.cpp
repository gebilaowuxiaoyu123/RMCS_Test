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

const int kCellPixels = 8;

void put_label(cv::Mat& image, const std::string& text, int row, double font_scale = 0.55) {
    const cv::Point origin{8, 22 + row * 20};
    cv::putText(
        image, text, origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 0, 0), 4,
        cv::LINE_AA);
    cv::putText(
        image, text, origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 255, 255), 2,
        cv::LINE_AA);
}

cv::Mat render(
    const GridMap& map, const SearchResult& result, GridPoint start, GridPoint goal,
    const std::vector<std::string>& labels) {

    cv::Mat image(
        map.height() * kCellPixels, map.width() * kCellPixels, CV_8UC3, kFreeColor);

    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            const GridPoint point{x, y};
            cv::Scalar color = kFreeColor;
            const auto cell = result.state[map.index(point)];
            if (!map.is_free(point))
                color = kOccupiedColor;
            else if (cell == 2)
                color = kClosedColor;
            else if (cell == 1)
                color = kOpenColor;
            cv::rectangle(
                image, cv::Rect(x * kCellPixels, y * kCellPixels, kCellPixels, kCellPixels), color,
                cv::FILLED);
        }
    }

    if (!result.path.empty()) {
        std::vector<cv::Point> centers;
        centers.reserve(result.path.size());
        for (const auto& point : result.path)
            centers.emplace_back(
                point.x * kCellPixels + kCellPixels / 2, point.y * kCellPixels + kCellPixels / 2);
        cv::polylines(image, centers, false, kPathColor, 2, cv::LINE_AA);
    }

    const auto marker = [&](GridPoint point, const cv::Scalar& color) {
        cv::rectangle(
            image, cv::Rect(point.x * kCellPixels, point.y * kCellPixels, kCellPixels, kCellPixels),
            color, cv::FILLED);
    };
    marker(start, kStartColor);
    marker(goal, kGoalColor);

    for (std::size_t i = 0; i < labels.size(); ++i)
        put_label(image, labels[i], static_cast<int>(i));

    return image;
}

cv::Mat stack_row(const std::vector<cv::Mat>& panels, int padding = 12) {
    cv::Mat row;
    for (std::size_t i = 0; i < panels.size(); ++i) {
        if (i == 0) {
            row = panels[i].clone();
            continue;
        }
        cv::Mat padded;
        cv::copyMakeBorder(
            panels[i], padded, 0, 0, padding, 0, cv::BORDER_CONSTANT, cv::Scalar(235, 235, 235));
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
            {"A* 8-neighbor (diagonal allowed)", stats_text(result)});
        cv::imwrite(output_dir + item.name + ".png", view);
    }

    const std::vector<MapCase> maps = build_maps();
    const MapCase& rooms = maps.front();

    std::vector<cv::Mat> panels;
    for (const bool allow_diagonal : {false, true}) {
        AStarOptions options;
        options.allow_diagonal = allow_diagonal;

        const SearchResult result = nav::astar(rooms.map, rooms.start, rooms.goal, options);
        const std::string name = allow_diagonal ? "rooms / 8-neighbor" : "rooms / 4-neighbor";
        report(name, result);

        panels.push_back(render(
            rooms.map, result, rooms.start, rooms.goal,
            {name, stats_text(result)}));
    }
    cv::imwrite(output_dir + "neighbors.png", stack_row(panels));

    std::cout << "结果图已写入 " << output_dir << "{rooms,random,neighbors}.png\n";
    return 0;
}
