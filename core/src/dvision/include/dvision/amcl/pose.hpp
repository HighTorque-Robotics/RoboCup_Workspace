/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file pose.hpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-23
 */

#pragma once

namespace dvision {
class Pose
{
  public:
    /**
     * @brief Pose constructor (explicit)
     */
    Pose();
    /**
     * @brief Pose constructor with position and angle
     *
     * @param x - position x of pose
     * @param y - position y of pose
     * @param heading - heading angle of pose
     */
    Pose(double x, double y, double heading);
    /**
     * @brief Pose constructor (copy)
     *
     * @param other - given pose to copy
     */
    Pose(const Pose& other);

    bool operator==(const Pose& oth) const;
    bool operator<(const Pose& oth) const;
    double& operator[](int index);

    /**
     * @brief Get position x of pose
     *
     * @return position x
     */
    double x() const;
    /**
     * @brief Get position y of pose
     *
     * @return position y
     */
    double y() const;
    /**
     * @brief Get heading angle in degree
     *
     * @return heading angle in degree
     */
    double heading() const;
    /**
     * @brief Get heading angle in radian
     *
     * @return heading angle in radian
     */
    double headingR() const;
    /**
     * @brief Get distance from origin point
     *
     * @return distance from origin point
     */
    double length() const;

    /**
     * @brief set position x of pose
     *
     * @param x - given position x
     */
    void setX(double x);
    /**
     * @brief set position y of pose
     *
     * @param y - given position y
     */
    void setY(double y);
    /**
     * @brief set heading angle in degree 
     *
     * @param h - given heading angle in degree
     */
    void setHeading(double h);
    /**
     * @brief set heading angle in radian 
     *
     * @param h - given heading angle in radian
     */
    void setHeadingR(double h);
    /**
     * @brief rotate coordinate system  with given degree
     *
     * @param degree - given degree
     */
    void rotate(double degree);

    /**
     * @brief Get global position and angle of pose
     *
     * @return global pose
     */
    Pose getGlobal();

  private:
    //! Position x of pose
    double x_;
    //! Position y of pose
    double y_;
    //! Heading angle of pose
    double heading_;
};
}
