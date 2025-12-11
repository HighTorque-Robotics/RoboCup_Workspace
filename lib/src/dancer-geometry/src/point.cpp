#include "dancer_geometry/point.hpp"
#include <cmath>

#define EPSILON 0.000001

namespace dancer_geometry {

Point::Point() : x(0.0), y(0.0){};

Point::Point(const Point &other) : x(other.x), y(other.y){};

Point::Point(double x, double y) : x(x), y(y){};

dancer_geometry::Angle Point::theta() const {
  // TODO handle case with point (0,0)
  double theta = rad2deg(atan2(y, x));
  return dancer_geometry::Angle(theta);
}

double Point::dist(const Point &other) const {
  double dx = x - other.x;
  double dy = y - other.y;
  return sqrt(dx * dx + dy * dy);
}

double Point::length() const {
  static Point zero(0.0, 0.0);
  return dist(zero);
}

Point &Point::normalize(double newLength) {
  double len = length();
  if (newLength == 0 || len == 0) {
    x = 0;
    y = 0;
  } else {
    double denominator = len / newLength;
    x /= denominator;
    y /= denominator;
  }
  return *this;
}

bool Point::is_collinear(const Point &other) const {
  // TODO handle case with point (0,0)
  dancer_geometry::Angle deltaTheta = theta() - other.theta();
  double delta = fabs(deltaTheta.get_signed());
  return (delta < EPSILON || (180 - delta) < EPSILON);
}

Point Point::perpendicular() const { return Point(-y, x); }

Point Point::rotate(const dancer_geometry::Angle &a) const {
  return Point(x * cos(a) - y * sin(a), x * sin(a) + y * cos(a));
}

Point Point::rotate(const dancer_geometry::Angle &a,
                    const Point &center) const {
  Point diff = *this - center;
  return center + diff.rotate(a);
}

Point Point::operator-() const { return Point(-x, -y); }

double Point::dot(const Point &p1, const Point &p2) {
  return p1.x * p2.x + p1.y * p2.y;
}

double Point::perp_dot(const Point &p1, const Point &p2) {
  return p1.x * p2.y - p1.y * p2.x;
}

Point Point::operator+(const Point &other) const {
  return Point(x + other.x, y + other.y);
}

Point Point::operator-(const Point &other) const {
  return Point(x - other.x, y - other.y);
}

Point Point::operator*(double ratio) const {
  return Point(x * ratio, y * ratio);
}

Point Point::operator/(double ratio) const {
  return Point(x / ratio, y / ratio);
}

Point &Point::operator+=(const Point &other) {
  *this = *this + other;
  return *this;
}

Point &Point::operator-=(const Point &other) {
  *this = *this - other;
  return *this;
}

Point &Point::operator*=(double ratio) {
  *this = *this * ratio;
  return *this;
}
Point &Point::operator/=(double ratio) {
  *this = *this / ratio;
  return *this;
}

bool Point::operator==(const Point &other) const {
  bool sameX = x == other.x;
  bool sameY = y == other.y;
  return sameX && sameY;
}

}  // namespace dancer_geometry
