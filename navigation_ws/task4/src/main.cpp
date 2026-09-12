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
#include "map.hpp"

namespace {

const cv::Scalar FREE_COLOR(255, 255, 255);
const cv::Scalar WALL_COLOR(70, 70, 70);
const cv::Scalar VISITED_COLOR(235, 206, 135);
const cv::Scalar OPEN_COLOR(170, 220, 255);
const cv::Scalar PATH_COLOR(50, 40, 240);
const cv::Scalar START_COLOR(60, 180, 75);
const cv::Scalar GOAL_COLOR(230, 120, 60);

void put_text(cv::Mat& image, const std::string& text, int row) {
    const cv::Point origin{8, 22 + row * 20};
    cv::putText(
        image, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
    cv::putText(
        image, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2,
        cv::LINE_AA);
}

std::string info_text(const Result& result) {
    std::ostringstream text;
    text << "expanded=" << result.expanded << "  points=" << result.path.size() << std::fixed
         << std::setprecision(1) << "  cost=" << result.cost;
    return text.str();
}

cv::Mat draw(
    const Map& map, const Result& result, Point start, Point goal, const std::string& title) {

    const int cell = std::max(6, 480 / map.width);
    cv::Mat image(map.height * cell, map.width * cell, CV_8UC3, FREE_COLOR);

    for (int y = 0; y < map.height; ++y) {
        for (int x = 0; x < map.width; ++x) {
            cv::Scalar color = FREE_COLOR;
            const auto state = result.visited[map.index(x, y)];
            if (!map.free(x, y))
                color = WALL_COLOR;
            else if (state == 2)
                color = VISITED_COLOR;
            else if (state == 1)
                color = OPEN_COLOR;
            cv::rectangle(image, cv::Rect(x * cell, y * cell, cell, cell), color, cv::FILLED);
        }
    }

    if (cell >= 12) {
        for (int x = 0; x <= map.width; ++x)
            cv::line(
                image, cv::Point{x * cell, 0}, cv::Point{x * cell, image.rows},
                cv::Scalar(205, 205, 205), 1);
        for (int y = 0; y <= map.height; ++y)
            cv::line(
                image, cv::Point{0, y * cell}, cv::Point{image.cols, y * cell},
                cv::Scalar(205, 205, 205), 1);
    }

    if (!result.path.empty()) {
        std::vector<cv::Point> centers;
        for (const auto& point : result.path)
            centers.emplace_back(point.x * cell + cell / 2, point.y * cell + cell / 2);
        cv::polylines(image, centers, false, PATH_COLOR, 2, cv::LINE_AA);
    }

    cv::rectangle(
        image, cv::Rect(start.x * cell, start.y * cell, cell, cell), START_COLOR, cv::FILLED);
    cv::rectangle(
        image, cv::Rect(goal.x * cell, goal.y * cell, cell, cell), GOAL_COLOR, cv::FILLED);

    put_text(image, title, 0);
    put_text(image, info_text(result), 1);
    return image;
}

cv::Mat side_by_side(const std::vector<cv::Mat>& images) {
    cv::Mat row;
    for (std::size_t i = 0; i < images.size(); ++i) {
        if (i == 0) {
            row = images[i].clone();
            continue;
        }
        cv::Mat padded;
        cv::copyMakeBorder(
            images[i], padded, 0, 0, 12, 0, cv::BORDER_CONSTANT, cv::Scalar(235, 235, 235));
        cv::hconcat(row, padded, row);
    }
    return row;
}

struct Case {
    std::string name;
    Map map;
    Point start;
    Point goal;
};

std::vector<Case> make_maps() {
    std::vector<Case> maps;

    Map tiny(5, 5);
    tiny.fill_wall(2, 1, 1, 3);
    maps.push_back(Case{"tiny_5x5", std::move(tiny), Point{0, 0}, Point{4, 4}});

    Map regular(12, 12);
    regular.fill_wall(0, 3, 11, 1);
    regular.fill_wall(1, 6, 11, 1);
    regular.fill_wall(0, 9, 11, 1);
    maps.push_back(Case{"regular_12x12", std::move(regular), Point{0, 0}, Point{11, 11}});

    Map rooms(60, 40);
    rooms.add_border();
    rooms.fill_wall(1, 12, 20, 1);
    rooms.fill_wall(28, 12, 20, 1);
    rooms.fill_wall(1, 26, 15, 1);
    rooms.fill_wall(22, 26, 25, 1);
    rooms.fill_wall(20, 1, 1, 8);
    rooms.fill_wall(40, 18, 1, 8);
    rooms.fill_wall(35, 27, 1, 12);
    rooms.fill_wall(8, 20, 10, 1);
    maps.push_back(Case{"rooms", std::move(rooms), Point{2, 2}, Point{57, 37}});

    Map randomMap(60, 40);
    randomMap.random_walls(0.28, 2026);
    randomMap.add_border();
    randomMap.clear_area(1, 1, 4, 4);
    randomMap.clear_area(55, 35, 4, 4);
    maps.push_back(Case{"random", std::move(randomMap), Point{2, 2}, Point{57, 37}});

    return maps;
}

void print_info(const std::string& name, const Result& result) {
    std::cout << "[" << name << "] " << (result.found ? "找到路径" : "未找到路径") << "  "
              << info_text(result) << '\n';
}

} // namespace

int main() {
    const std::string outDir = "output/";

    for (auto& item : make_maps()) {
        const Result result = find_path(item.map, item.start, item.goal, true);
        print_info(item.name, result);
        cv::imwrite(
            outDir + item.name + ".png",
            draw(item.map, result, item.start, item.goal, "A* 8-neighbor"));
    }

    const std::vector<Case> maps = make_maps();
    const Case& rooms = maps[2];

    std::vector<cv::Mat> images;
    for (const bool allowDiagonal : {false, true}) {
        const Result result = find_path(rooms.map, rooms.start, rooms.goal, allowDiagonal);
        const std::string title = allowDiagonal ? "rooms / 8-neighbor" : "rooms / 4-neighbor";
        print_info(title, result);
        images.push_back(draw(rooms.map, result, rooms.start, rooms.goal, title));
    }
    cv::imwrite(outDir + "neighbors.png", side_by_side(images));

    std::cout << "结果图已写入 " << outDir
              << "{tiny_5x5,regular_12x12,rooms,random,neighbors}.png\n";
    return 0;
}
