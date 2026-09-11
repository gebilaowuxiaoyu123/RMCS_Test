#include <algorithm>
#include <cstdlib>
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
using nav::Heuristic;
using nav::SearchResult;

const cv::Scalar kFreeColor(255, 255, 255);
const cv::Scalar kOccupiedColor(70, 70, 70);
const cv::Scalar kClosedColor(235, 206, 135);
const cv::Scalar kOpenColor(170, 220, 255);
const cv::Scalar kPathColor(50, 40, 240);
const cv::Scalar kStartColor(60, 180, 75);
const cv::Scalar kGoalColor(230, 120, 60);

void put_label(cv::Mat& image, const std::string& text, double font_scale = 0.6) {
    const cv::Point origin{8, static_cast<int>(22 * font_scale / 0.6)};
    cv::putText(
        image, text, origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 0, 0), 4,
        cv::LINE_AA);
    cv::putText(
        image, text, origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 255, 255), 2,
        cv::LINE_AA);
}

cv::Mat render(
    const GridMap& map, const std::vector<std::uint8_t>* state,
    const std::vector<GridPoint>& path, GridPoint start, GridPoint goal, int cell_px) {

    cv::Mat image(map.height() * cell_px, map.width() * cell_px, CV_8UC3, kFreeColor);

    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            const GridPoint point{x, y};
            cv::Scalar color = kFreeColor;
            if (!map.is_free(point))
                color = kOccupiedColor;
            else if (state != nullptr && (*state)[map.index(point)] == 2)
                color = kClosedColor;
            else if (state != nullptr && (*state)[map.index(point)] == 1)
                color = kOpenColor;
            cv::rectangle(
                image, cv::Rect(x * cell_px, y * cell_px, cell_px, cell_px), color, cv::FILLED);
        }
    }

    if (!path.empty()) {
        std::vector<cv::Point> centers;
        centers.reserve(path.size());
        for (const auto& point : path)
            centers.emplace_back(
                point.x * cell_px + cell_px / 2, point.y * cell_px + cell_px / 2);
        cv::polylines(image, centers, false, kPathColor, 2, cv::LINE_AA);
    }

    const auto marker = [&](GridPoint point, const cv::Scalar& color) {
        cv::rectangle(
            image, cv::Rect(point.x * cell_px, point.y * cell_px, cell_px, cell_px), color,
            cv::FILLED);
    };
    marker(start, kStartColor);
    marker(goal, kGoalColor);

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
            panels[i], padded, 0, 0, padding, 0, cv::BORDER_CONSTANT, cv::Scalar(240, 240, 240));
        cv::hconcat(row, padded, row);
    }
    return row;
}

cv::Mat stack_grid(const std::vector<cv::Mat>& panels, int columns, int padding = 12) {
    std::vector<cv::Mat> rows;
    for (std::size_t i = 0; i < panels.size(); i += static_cast<std::size_t>(columns)) {
        std::vector<cv::Mat> row(panels.begin() + static_cast<long>(i),
                                 panels.begin() + static_cast<long>(std::min(panels.size(), i + static_cast<std::size_t>(columns))));
        rows.push_back(stack_row(row, padding));
    }
    cv::Mat grid;
    for (std::size_t i = 0; i < rows.size(); ++i) {
        if (i == 0) {
            grid = rows[i].clone();
            continue;
        }
        const int width = std::max(grid.cols, rows[i].cols);
        cv::Mat a, b;
        cv::copyMakeBorder(
            grid, a, 0, padding, 0, width - grid.cols, cv::BORDER_CONSTANT,
            cv::Scalar(240, 240, 240));
        cv::copyMakeBorder(
            rows[i], b, 0, 0, 0, width - rows[i].cols, cv::BORDER_CONSTANT,
            cv::Scalar(240, 240, 240));
        cv::vconcat(a, b, grid);
    }
    return grid;
}

struct MapCase {
    std::string name;
    GridMap map;
    GridPoint start;
    GridPoint goal;
};

std::vector<MapCase> build_maps() {
    std::vector<MapCase> cases;

    GridMap random_map(60, 40);
    random_map.fill_random(0.28, 2026);
    random_map.fill_border(1);
    random_map.fill_rect(GridPoint{1, 1}, 4, 4, false);
    random_map.fill_rect(GridPoint{55, 35}, 4, 4, false);
    cases.push_back(MapCase{"random", std::move(random_map), GridPoint{2, 2}, GridPoint{57, 37}});

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
    cases.push_back(MapCase{"rooms", std::move(rooms), GridPoint{2, 2}, GridPoint{57, 37}});

    GridMap maze(41, 31);
    maze.carve_maze(7);
    cases.push_back(MapCase{"maze", std::move(maze), GridPoint{1, 1}, GridPoint{39, 29}});

    return cases;
}

void report(const std::string& name, const SearchResult& result) {
    std::cout << "[" << name << "] 路径" << (result.stats.found ? "找到" : "未找到")
              << " 展开=" << result.stats.expanded << " 生成=" << result.stats.generated
              << " 路径点=" << result.stats.path_points << " 代价=" << result.stats.path_cost
              << '\n';
}

} // namespace

int main() {
    const int cell_px = 8;
    const std::string output_dir = "output/";

    for (auto& item : build_maps()) {
        AStarOptions options;
        options.allow_diagonal = true;
        options.heuristic = Heuristic::kOctile;

        const SearchResult dry_run = nav::astar(item.map, item.start, item.goal, options);
        report(item.name + " 预跑", dry_run);

        options.snapshot_interval = std::max<std::size_t>(1, dry_run.stats.expanded / 4);
        options.max_snapshots = 4;
        const SearchResult result = nav::astar(item.map, item.start, item.goal, options);
        report(item.name, result);

        cv::Mat final_view =
            render(item.map, &result.state, result.path, item.start, item.goal, cell_px);
        put_label(
            final_view,
            "A* 8-neighbor octile  expanded=" + std::to_string(result.stats.expanded));
        cv::imwrite(output_dir + item.name + "_result.png", final_view);

        std::vector<cv::Mat> frames;
        for (std::size_t i = 0; i < result.snapshots.size(); ++i) {
            cv::Mat frame =
                render(item.map, &result.snapshots[i], {}, item.start, item.goal, cell_px);
            put_label(frame, "expanded=" + std::to_string(result.snapshot_expanded[i]));
            frames.push_back(frame);
        }
        if (!frames.empty()) {
            cv::Mat last = render(item.map, &result.state, result.path, item.start, item.goal, cell_px);
            put_label(last, "done  expanded=" + std::to_string(result.stats.expanded));
            frames.push_back(last);
            cv::imwrite(output_dir + item.name + "_process.png", stack_row(frames));
        }
    }

    const auto cases = build_maps();
    const MapCase& rooms = cases[1];

    struct Variant {
        std::string label;
        AStarOptions options;
    };

    std::vector<Variant> variants;
    variants.push_back(Variant{"4-neighbor / manhattan", AStarOptions{false, true, Heuristic::kManhattan, 1.0, 0, 0}});
    variants.push_back(Variant{"8-neighbor / manhattan", AStarOptions{true, true, Heuristic::kManhattan, 1.0, 0, 0}});
    variants.push_back(Variant{"8-neighbor / octile", AStarOptions{true, true, Heuristic::kOctile, 1.0, 0, 0}});
    variants.push_back(Variant{"8-neighbor / dijkstra", AStarOptions{true, true, Heuristic::kZero, 1.0, 0, 0}});
    variants.push_back(Variant{"8-neighbor / octile w=3", AStarOptions{true, true, Heuristic::kOctile, 3.0, 0, 0}});
    variants.push_back(Variant{"8-neighbor / euclidean", AStarOptions{true, true, Heuristic::kEuclidean, 1.0, 0, 0}});

    std::vector<cv::Mat> panels;
    for (const auto& variant : variants) {
        const SearchResult result = nav::astar(rooms.map, rooms.start, rooms.goal, variant.options);
        report("rooms / " + variant.label, result);
        cv::Mat panel = render(rooms.map, &result.state, result.path, rooms.start, rooms.goal, cell_px);
        put_label(panel, variant.label, 0.5);
        std::ostringstream text;
        text << "expanded=" << result.stats.expanded << std::fixed << std::setprecision(1)
             << " cost=" << result.stats.path_cost;
        put_label(panel, text.str(), 0.5);
        panels.push_back(panel);
    }
    cv::imwrite(output_dir + "heuristics.png", stack_grid(panels, 3));

    return 0;
}
