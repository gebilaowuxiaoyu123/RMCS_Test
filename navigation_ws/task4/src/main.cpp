#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "astar.hpp"
#include "map.hpp"

// 图上每种东西的颜色
cv::Scalar FREE_COLOR(240, 240, 240);       // 没搜过的空地
cv::Scalar VISITED_COLOR(205, 246, 255);    // 搜过的格子
cv::Scalar OPEN_COLOR(152, 224, 255);       // 还在待办清单里的格子
cv::Scalar WALL_COLOR(55, 55, 55);          // 墙
cv::Scalar PATH_COLOR(90, 175, 75);         // 最后那条路
cv::Scalar GRID_COLOR(205, 205, 205);       // 网格线
cv::Scalar MARK_COLOR(255, 255, 255);       // 起点终点的格子
cv::Scalar TEXT_COLOR(0, 0, 0);             // S 和 G 两个字

// 在格子里写个字母（起点写 S，终点写 G）
void put_letter(cv::Mat& image, Point point, int cell, std::string letter) {
    cv::rectangle(
        image, cv::Rect(point.x * cell, point.y * cell, cell, cell), MARK_COLOR, cv::FILLED);

    double scale = cell / 40.0;
    int thickness = 1;
    if (cell >= 30)
        thickness = 2;

    int x = point.x * cell + cell / 4;
    int y = point.y * cell + cell * 4 / 5;
    cv::putText(
        image, letter, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, scale, TEXT_COLOR, thickness,
        cv::LINE_AA);
}

// 把地图和寻路结果画成一张图
cv::Mat draw(const Map& map, const Result& result, Point start, Point goal) {
    int cell = 480 / map.width;
    if (cell < 14)
        cell = 14;

    cv::Mat image(map.height * cell, map.width * cell, CV_8UC3, FREE_COLOR);

    // 一格一格填颜色
    for (int y = 0; y < map.height; y++) {
        for (int x = 0; x < map.width; x++) {
            cv::Scalar color = FREE_COLOR;
            if (map.free(x, y) == false)
                color = WALL_COLOR;
            else if (result.colored[map.index(x, y)] == 2)
                color = VISITED_COLOR;
            else if (result.colored[map.index(x, y)] == 1)
                color = OPEN_COLOR;
            cv::rectangle(image, cv::Rect(x * cell, y * cell, cell, cell), color, cv::FILLED);
        }
    }

    // 路径上的格子整格涂绿
    for (int i = 0; i < (int)result.path.size(); i++) {
        int x = result.path[i].x;
        int y = result.path[i].y;
        cv::rectangle(image, cv::Rect(x * cell, y * cell, cell, cell), PATH_COLOR, cv::FILLED);
    }

    // 补网格线
    for (int x = 0; x <= map.width; x++)
        cv::line(image, cv::Point(x * cell, 0), cv::Point(x * cell, image.rows), GRID_COLOR, 1);
    for (int y = 0; y <= map.height; y++)
        cv::line(image, cv::Point(0, y * cell), cv::Point(image.cols, y * cell), GRID_COLOR, 1);

    put_letter(image, start, cell, "S");
    put_letter(image, goal, cell, "G");

    return image;
}

// 跑一张地图：算一次、打印一行、存一张图
void run(Map map, Point start, Point goal, std::string name) {
    Result result = search(map, start, goal);

    std::cout << "[" << name << "]  ";
    if (result.found)
        std::cout << "找到路径";
    else
        std::cout << "没找到路径";
    std::cout << "  expanded=" << result.expanded << "  points=" << result.path.size()
              << "  g=" << result.g << std::endl;

    cv::imwrite("output/" + name + ".png", draw(map, result, start, goal));
}

int main() {
    // 5×5：中间一道竖墙
    Map tiny(5, 5);
    tiny.fill_wall(2, 1, 1, 3);
    run(tiny, Point{0, 0}, Point{4, 4}, "tiny_5x5");

    // 12×12：三道错开的墙
    Map regular(12, 12);
    regular.fill_wall(0, 3, 11, 1);
    regular.fill_wall(1, 6, 11, 1);
    regular.fill_wall(0, 9, 11, 1);
    run(regular, Point{0, 0}, Point{11, 11}, "regular_12x12");

    // 60×40：几间房间，靠门洞连通
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
    run(rooms, Point{2, 2}, Point{57, 37}, "rooms");

    // 60×40：随机撒墙，起点终点附近清空
    Map randomMap(60, 40);
    randomMap.random_walls(0.28, 2026);
    randomMap.add_border();
    randomMap.clear_area(1, 1, 4, 4);
    randomMap.clear_area(55, 35, 4, 4);
    run(randomMap, Point{2, 2}, Point{57, 37}, "random");

    return 0;
}
