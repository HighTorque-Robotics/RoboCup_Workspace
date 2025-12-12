/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file map.hpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-23
 */

#pragma once
#include "dvision/parameters.hpp"

#include <set>
#include <utility>
#include <vector>

namespace dvision {
//! Maximum occupied distance (100cm)
static const int MAX_OCC_DIST = 25;

struct Cell
{
    int occ_dist;
    Cell()
      : occ_dist(200)
    {
    }
};

/**
 * @brief Rasterized map for AMCL.
 */
class Map
{
  public:
    //! Map constructor
    Map();
    //! Initialize Map
    void Init();

    /**
     * @brief Get cell
     *
     * @param x - position x of cell
     * @param y - position y of cell
     *
     * @return cell at position (x,y)
     */
    inline Cell& getCell(int x, int y)
    {
        if (x < 0 || x >= width_ || y < 0 || y >= height_) {
            ROS_ERROR("Map %d * %d get cell error: x: %d, y: %d", width_, height_, x, y);
            return cells_[0];
        }
        return cells_.at(y * width_ + x);
    }

    inline int getDist(float x, float y)
    {
        auto p = fieldToMap((int)x, (int)y);

        if (p.first < 0 || p.first >= width_ || p.second < 0 || p.second >= height_) {
            return max_occ_dist_;
        }

        return cells_.at(p.second * width_ + p.first).occ_dist;
    }

    /**
     * @brief Get width of map
     *
     * @return width of map
     */
    inline int width()
    {
        return width_;
    }

    /**
     * @brief Get height of map
     *
     * @return height of map
     */
    inline int height()
    {
        return height_;
    }

    /**
     * @brief Get maximum occupied distance
     *
     * @return
     */
    inline int maxOccDist()
    {
        return max_occ_dist_;
    }

    inline std::set<std::pair<int, int>> getOccpied()
    {
        return occupied_;
    };

    /**
     * @brief Get score at position (x,y)
     *
     * @param x - position x
     * @param y
     *
     * @return score at position (x,y)
     */

    float getScore(int x, int y);
    /**
     * @brief Get score at position (x,y)
     *
     * @param x - position x
     * @param y
     *
     * @return score at position (x,y)
     */
    float getScore(float x, float y);

    /**
     * @brief convert coordinate from field to map
     *
     * @param x - position x in field coordinate system
     * @param y - position y in field coordinate system
     *
     * @return postion in map coordinate system
     */
    std::pair<int, int> fieldToMap(int x, int y);
    /**
     * @brief convert coordinate from map to field
     *
     * @param x - position x in map coordinate system
     * @param y - position y in map coordinate system
     *
     * @return postion in field coordinate system
     */
    std::pair<int, int> mapToField(int x, int y);

  private:
    /**
     * @brief Fill point in map
     *
     * @param x - position x of point
     * @param y - position y of point
     * @param dist - occupied distance of point
     */
    void fillPoint(int x, int y, int dist);

    /**
     * @brief Fill a horizontal line
     *
     * @param x - position x of start point
     * @param y - position y of start point
     * @param width - length of line
     * @param dist - occupied distance of point
     */
    void horizontalFill(int x, int y, int width, int dist);

    /**
     * @brief Fill a vertical line
     *
     * @param x - position x of start point
     * @param y - position y of start point
     * @param height - length of line
     * @param dist - occupied distance of point
     */
    void verticalFill(int x, int y, int height, int dist);

    /**
     * @brief Fill a rectangle
     *
     * @param x - position x of start point
     * @param y - position y of start point
     * @param width - width of rectangle
     * @param height - height of rectangle
     * @param dist - occupied distance of point
     */
    void fillRect(int x, int y, int width, int height, int dist);

    /**
     * @brief Fill a rectangle with two vertex
     *
     * @param upperLeft - upper left point of rectangle
     * @param lowerRight - lower right point of rectangle
     * @param dist - occupied distance of point
     */
    void fillRect(std::pair<int, int> upperLeft, std::pair<int, int> lowerRight, int dist);

    /**
     * @brief Fill a circle
     *
     * @param x - position x of circle center
     * @param y - position y of circle center
     * @param r - radius of circle
     * @param dist - occupied distance of point
     */
    void fillCircle(int x, int y, int r, int dist);

  private:
    //! Map width
    int width_;
    //! Map height
    int height_;
    //! Maximum occupied distance of map
    int max_occ_dist_;
    //! Grid size of map
    int grid_size_;

    //! Cells of map
    std::vector<Cell> cells_;
    //! Occupancy of map???
    std::set<std::pair<int, int>> occupied_;
};

} // namespace dvision
