/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file map.cpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-23
 */

#include "dvision/amcl/map.hpp"
#include "dconfig/dconstant.hpp"

#include <cmath>
#include <iostream>
#include <ros/ros.h>

using namespace dconstant;
using namespace std;

namespace dvision {

Map::Map()
{
}

void
Map::Init()
{
    // TODO(MWX): gaussian
    int gridSize = geometry::lineWidth;
    int h = geometry::fieldWidth + geometry::borderStripWidth * 2;
    int w = geometry::fieldLength + geometry::borderStripWidth * 2;
    width_ = w / gridSize + 1;
    height_ = h / gridSize + 1;

    // Calculate half from geometry constant
    int half_field_length = geometry::fieldLength / 2;
    int half_field_width = geometry::fieldWidth / 2;
    int half_goal_area_width = geometry::goalAreaWidth / 2;
    int half_goal_width = geometry::goalWidth / 2;

    std::cout << "Map w: " << width_ << " h: " << height_ << std::endl;
    grid_size_ = geometry::lineWidth;

    max_occ_dist_ = MAX_OCC_DIST / grid_size_;

    // Init occ dist
    cells_.resize(width_ * height_);
    for_each(cells_.begin(), cells_.end(), [this](Cell& c) { c.occ_dist = max_occ_dist_; });

    // TODO(MWX): use geometry constant
    auto upleft = fieldToMap(-half_field_length, half_field_width);
    auto lowright = fieldToMap(half_field_length, -half_field_width);
    fillRect(upleft, lowright, 0);

    auto goal1 = fieldToMap(-half_field_length, half_goal_area_width);
    auto goal2 = fieldToMap(-(half_field_length - geometry::goalAreaLength), -half_goal_area_width);
    fillRect(goal1, goal2, 0);

    goal1 = fieldToMap(half_field_length, -half_goal_area_width);
    goal2 = fieldToMap((half_field_length - geometry::goalAreaLength), half_goal_area_width);
    fillRect(goal2, goal1, 0);

    goal1 = fieldToMap(-(geometry::lineWidth * 2 + geometry::goalAreaWidth), half_goal_width);
    goal2 = fieldToMap(-half_field_length, -half_goal_width);
    fillRect(goal1, goal2, 0);

    goal1 = fieldToMap((geometry::lineWidth * 2 + geometry::goalAreaWidth), -half_goal_width);
    goal2 = fieldToMap(half_field_length, half_goal_width);
    fillRect(goal2, goal1, 0);

    auto c1 = fieldToMap(0, half_field_width);
    auto c2 = fieldToMap(0, -half_field_width);
    fillRect(c1, c2, 0);

    auto center = fieldToMap(0, 0);
    fillCircle(center.first, center.second, geometry::centerCircleDiameter / 2 / 5, 0);

    for (auto& occ : occupied_) {
        int maxOcc = max_occ_dist_;
        int curx = occ.first;
        int cury = occ.second;
        for (int dx = -maxOcc; dx <= maxOcc; ++dx) {
            for (int dy = -maxOcc; dy <= +maxOcc; ++dy) {
                int nx = curx + dx;
                int ny = cury + dy;
                if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_)
                    continue;
                double dist = sqrt(dx * dx + dy * dy);
                auto& cell = getCell(nx, ny);
                cell.occ_dist = min(cell.occ_dist, (int)dist);
            }
        }
    }
}

// Methods to init map

pair<int, int>
Map::fieldToMap(int x, int y)
{
    int rx = x + geometry::wholeWidth / 2;
    int ry = geometry::wholeHeight / 2 - y;
    int gridSize = geometry::lineWidth;

    // cout << "field to Map In: " << x << " " << y << " | " << rx << " " << ry;

    return make_pair(rx / gridSize, ry / gridSize);
}

std::pair<int, int>
Map::mapToField(int rx, int ry)
{
    int gridSize = geometry::lineWidth;
    int x = rx * gridSize - geometry::wholeWidth / 2;
    int y = geometry::wholeHeight / 2 - ry * gridSize;

    return make_pair(x, y);
}

void
Map::fillPoint(int x, int y, int dist)
{
    occupied_.insert(make_pair(x, y));
    getCell(x, y).occ_dist = dist;
}

void
Map::horizontalFill(int x, int y, int width, int dist)
{
    for (int i = 0; i < width; ++i) {
        fillPoint(x + i, y - 1, dist);
        fillPoint(x + i, y, dist);
        fillPoint(x + i, y + 1, dist);
    }
}

void
Map::verticalFill(int x, int y, int height, int dist)
{
    for (int i = 0; i < height; ++i) {
        fillPoint(x - 1, y + i, dist);
        fillPoint(x, y + i, dist);
        fillPoint(x + 1, y + i, dist);
    }
}

void
Map::fillRect(int x, int y, int width, int height, int dist)
{
    horizontalFill(x, y, width, dist);

    horizontalFill(x, y + height - 1, width, dist);

    verticalFill(x, y, height, dist);

    verticalFill(x + width - 1, y, height, dist);
}

void
Map::fillRect(std::pair<int, int> upperLeft, std::pair<int, int> lowerRight, int dist)
{
    int width = (lowerRight.first - upperLeft.first) + 1;
    int height = (lowerRight.second - upperLeft.second) + 1;
    fillRect(upperLeft.first, upperLeft.second, width, height, dist);
}

void
Map::fillCircle(int x, int y, int r, int dist)
{
    // FIXME(MWX): 4 she 5 ru
    for (int i = 0; i < 360; ++i) {
        int dx = r * sin(i * 180.0 / M_PI);
        int dy = r * cos(i * 180.0 / M_PI);

        fillPoint(x + dx, y + dy, dist);
    }
}
} // namespace dvision
