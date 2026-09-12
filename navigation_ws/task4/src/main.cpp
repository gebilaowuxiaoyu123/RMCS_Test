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

const int FREE_LEVEL = 255;
const int WALL_LEVEL = 0;
const int VISITED_LEVEL = 214;
const int OPEN_LEVEL = 168;
const int GRID_LEVEL = 238;
const int PATH_LEVEL = 70;

void put_mark(cv::Mat& image, Point point, int cell, const std::string& letter) {
    cv::rectangle(
        image, cv::Rect(point.x * cell, point.y * cell, cell, cell), cv::Scalar(FREE_LEVEL),
        cv::FILLED);

    const double scale = cell / 40.0;
    const int thickness = cell >= 30 ? 2 : 1;
    int baseline = 0;
    const cv::Size size = cv::getTextSize(letter, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline);
    const cv::Point center{point.x * cell + cell / 2, point.y * cell + cell / 2};
    cv::putText(
        image, letter, cv::Point{center.x - size.width / 2, center.y + size.height / 2},
        cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(WALL_LEVEL), thickness, cv::LINE_AA);
}

std::string info_text(const Result& result) {
    std::ostringstream text;
    text << "expanded=" << result.expanded << "  points=" << result.path.size() << std::fixed
         << std::setprecision(1) << "  cost=" << result.cost;
    return text.str();
}

cv::Mat draw(const Map& map, const Result& result, Point start, Point goal) {

    const int cell = std::max(14, 480 / map.width);
    cv::Mat image(map.height * cell, map.width * cell, CV_8UC1, cv::Scalar(FREE_LEVEL));

    for (int y = 0; y < map.height; ++y) {
        for (int x = 0; x < map.width; ++x) {
            int level = FREE_LEVEL;
            const auto state = result.visited[map.index(x, y)];
            if (!map.free(x, y))
                level = WALL_LEVEL;
            else if (state == 2)
                level = VISITED_LEVEL;
            else if (state == 1)
                level = OPEN_LEVEL;
            cv::rectangle(
                image, cv::Rect(x * cell, y * cell, cell, cell), cv::Scalar(level), cv::FILLED);
        }
    }

    if (cell >= 24) {
        for (int x = 0; x <= map.width; ++x)
            cv::line(
                image, cv::Point{x * cell, 0}, cv::Point{x * cell, image.rows},
                cv::Scalar(GRID_LEVEL), 1);
        for (int y = 0; y <= map.height; ++y)
            cv::line(
                image, cv::Point{0, y * cell}, cv::Point{image.cols, y * cell},
                cv::Scalar(GRID_LEVEL), 1);
    }
    if (!result.path.empty()) {
        std::vector<cv::Point> centers;
        for (const auto& point : result.path)
            centers.emplace_back(point.x * cell + cell / 2, point.y * cell + cell / 2);
        cv::polylines(image, centers, false, cv::Scalar(PATH_LEVEL), 2, cv::LINE_AA);
    }

    put_mark(image, start, cell, "S");
    put_mark(image, goal, cell, "G");

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
            images[i], padded, 0, 0, 12, 0, cv::BORDER_CONSTANT, cv::Scalar(GRID_LEVEL));
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
            outDir + item.name + ".png", draw(item.map, result, item.start, item.goal));
    }

    const std::vector<Case> maps = make_maps();
    const Case& rooms = maps[2];

    std::vector<cv::Mat> images;
    for (const bool allowDiagonal : {false, true}) {
        const Result result = find_path(rooms.map, rooms.start, rooms.goal, allowDiagonal);
        const std::string title = allowDiagonal ? "rooms / 8-neighbor" : "rooms / 4-neighbor";
        print_info(title, result);
        images.push_back(draw(rooms.map, result, rooms.start, rooms.goal));
    }
    cv::imwrite(outDir + "neighbors.png", side_by_side(images));

    std::cout << "结果图已写入 " << outDir
              << "{tiny_5x5,regular_12x12,rooms,random,neighbors}.png\n";
    return 0;
}
