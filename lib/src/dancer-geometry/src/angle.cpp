#include "dancer_geometry/angle.hpp"
#include <cmath>
#include <iostream>

#define EPSILON 0.000001

namespace dancer_geometry {

Angle::Angle() : value(0.0){};

Angle::Angle(double degree) {
  value = fmod(degree + 180, 360);
  if (value < 0) {
    value += 360;
  }
  value -= 180;
}

Angle::Angle(const Angle &other) : value(other.value){};

Angle Angle::from_xy(double x, double y) {
  double radAngle = atan2(y, x);
  return Angle(radAngle * 180.0 / M_PI);
}
Angle Angle::mean(const std::vector<Angle> &angles, double *std) {
  double sum_cos(0), sum_sin(0);
  for (const auto &angle : angles) {
    sum_cos += cos(angle);
    sum_sin += sin(angle);
  }
  double mean_cos, mean_sin;
  mean_cos = sum_cos / angles.size();
  mean_sin = sum_sin / angles.size();
  Angle mean = from_xy(mean_cos, mean_sin);
  double norm = sqrt(mean_cos * mean_cos + mean_sin * mean_sin);
  if (norm >= 1) {  // Avoiding floating point exceptions
    *std = 0;
  } else {
    *std = sqrt(-2 * log(norm)) * 180 / M_PI;
  }
  return mean;
}

Angle Angle::mean(const std::vector<Angle> &angles) {
  double std;
  return mean(angles, &std);
}

double Angle::std(const std::vector<Angle> &angles) {
  double std;
  mean(angles, &std);
  return std;
}

bool Angle::is_equal(const Angle &other) const {
  double delta = fabs((*this - other).get_unsigned());
  return delta < EPSILON;
}

Angle Angle::asin(double x) { return Angle(rad2deg(std::asin(x))); }

Angle Angle::acos(double x) { return Angle(rad2deg(std::acos(x))); }

Angle Angle::weighted_average(const Angle &a1, double w1, const Angle &a2,
                              double w2) {
  double x = w1 * cos(a1) + w2 * cos(a2);
  double y = w1 * sin(a1) + w2 * sin(a2);
  return Angle(rad2deg(atan2(y, x)));
}

bool Angle::operator==(const Angle &a) const { return a.value == value; }

Angle Angle::operator-(const Angle &a) const { return Angle(value - a.value); }

Angle Angle::operator+(const Angle &a) const { return Angle(value + a.value); }

double cos(const Angle &a) { return std::cos(a.get_signed() * M_PI / 180.0); }

double sin(const Angle &a) { return std::sin(a.get_signed() * M_PI / 180.0); }

double tan(const Angle &a) { return std::tan(a.get_signed() * M_PI / 180.0); }

double deg2rad(double degree) { return M_PI * degree / 180.0; }

double rad2deg(double radian) { return radian * 180.0 / M_PI; }

Angle operator-(const Angle &a) { return Angle(-a.get_signed()); }

Angle operator*(double x, const Angle &a) { return Angle(x * a.get_signed()); }

Angle operator*(const Angle &a, double x) { return x * a; }

Angle operator/(const Angle &a, double x) { return Angle(a.get_signed() / x); }

std::ostream &operator<<(std::ostream &out, const Angle &a) {
  return out << a.get_signed();
}

double normalize_rad(double radian) {
  double value = fmod(radian, 2 * M_PI);  // Bound in [-2*pi, 2*pi]
  if (value < -M_PI) {
    value += 2 * M_PI;
  } else if (value > M_PI) {
    value -= 2 * M_PI;
  }
  return value;
}

}  // namespace dancer_geometry
