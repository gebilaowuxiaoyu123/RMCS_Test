#pragma once

#include <vector>

#include "map.hpp"

struct Result {
    bool found = false;
    int expanded = 0;
    double cost = 0.0;
    std::vector<Point> path;
    std::vector<unsigned char> visited;
};

Result find_path(const Map& map, Point start, Point goal, bool allowDiagonal);
